/*
 * dmxdev.c - DVB demultiplexer device
 *
 * Copyright (C) 2000 Ralph Metzler & Marcus Metzler
 *		      for convergence integrated media GmbH
 *
 * Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 */

#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/ioctl.h>
#include <linux/wait.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include "dmxdev.h"

static int debug;

module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Turn on/off debugging (default:off).");

#define DMX_DEFAULT_DECODER_BUFFER_SIZE (32768)

#define dprintk	if (debug) printk

static int dvb_dmxdev_buffer_write(struct dvb_ringbuffer *buf,
				   const u8 *src, size_t len)
{
	ssize_t free;

	if (!len)
		return 0;
	if (!buf->data)
		return 0;

	free = dvb_ringbuffer_free(buf);
	if (len > free) {
#ifdef CONFIG_DEBUG_PRINTK
		dprintk("dmxdev: buffer overflow\n");
#else
		d;
#endif
		return -EOVERFLOW;
	}

	return dvb_ringbuffer_write(buf, src, len);
}

static inline void dvb_dmxdev_notify_data_read(struct dmxdev_filter *filter,
					int bytes_read)
{
	if (!filter)
		return;

	if (filter->type == DMXDEV_TYPE_SEC) {
		if (filter->feed.sec.feed->notify_data_read)
			filter->feed.sec.feed->notify_data_read(
						filter->filter.sec,
						bytes_read);
	} else {
		struct dmxdev_feed *feed;

		/*
		 * All feeds of same demux-handle share the same output
		 * buffer, it is enough to notify on the buffer status
		 * on one of the feeds
		 */
		feed = list_first_entry(&filter->feed.ts,
					struct dmxdev_feed, next);

		if (feed->ts->notify_data_read)
			feed->ts->notify_data_read(
						feed->ts,
						bytes_read);
	}
}

static inline u32 dvb_dmxdev_advance_event_idx(u32 index)
{
	index++;
	if (index >= DMX_EVENT_QUEUE_SIZE)
		index = 0;

	return index;
}

static inline int dvb_dmxdev_events_is_full(struct dmxdev_events_queue *events)
{
	int new_write_index;

	new_write_index = dvb_dmxdev_advance_event_idx(events->write_index);
	if (new_write_index == events->read_index)
		return 1;

	return 0;

}

static inline void dvb_dmxdev_flush_events(struct dmxdev_events_queue *events)
{
	events->read_index = 0;
	events->write_index = 0;
	events->notified_index = 0;
	events->bytes_read_no_event = 0;
	events->current_event_data_size = 0;
}

static inline void dvb_dmxdev_flush_output(struct dvb_ringbuffer *buffer,
					struct dmxdev_events_queue *events)
{
	dvb_dmxdev_flush_events(events);
	dvb_ringbuffer_flush(buffer);
}

static int dvb_dmxdev_update_pes_event(struct dmx_filter_event *event,
					int bytes_read)
{
	int start_delta;

	if (event->params.pes.total_length <= bytes_read)
		return event->params.pes.total_length;

	/*
	 * only part of the data relevant to this event was read.
	 * Update the event's information to reflect the new state.
	 */
	event->params.pes.total_length -= bytes_read;

	start_delta = event->params.pes.start_offset -
		event->params.pes.base_offset;

	if (bytes_read <= start_delta) {
		event->params.pes.base_offset +=
			bytes_read;
	} else {
		start_delta =
			bytes_read - start_delta;

		event->params.pes.start_offset += start_delta;
		event->params.pes.actual_length -= start_delta;

		event->params.pes.base_offset =
			event->params.pes.start_offset;
	}

	return 0;
}

static int dvb_dmxdev_update_section_event(struct dmx_filter_event *event,
					int bytes_read)
{
	int start_delta;

	if (event->params.section.total_length <= bytes_read)
		return event->params.section.total_length;

	/*
	 * only part of the data relevant to this event was read.
	 * Update the event's information to reflect the new state.
	 */

	event->params.section.total_length -= bytes_read;

	start_delta = event->params.section.start_offset -
		event->params.section.base_offset;

	if (bytes_read <= start_delta) {
		event->params.section.base_offset +=
			bytes_read;
	} else {
		start_delta =
			bytes_read - start_delta;

		event->params.section.start_offset += start_delta;
		event->params.section.actual_length -= start_delta;

		event->params.section.base_offset =
			event->params.section.start_offset;
	}

	return 0;
}

static int dvb_dmxdev_update_rec_event(struct dmx_filter_event *event,
					int bytes_read)
{
	if (event->params.recording_chunk.size <= bytes_read)
		return event->params.recording_chunk.size;

	/*
	 * only part of the data relevant to this event was read.
	 * Update the event's information to reflect the new state.
	 */
	event->params.recording_chunk.size -= bytes_read;
	event->params.recording_chunk.offset += bytes_read;

	return 0;
}

static int dvb_dmxdev_add_event(struct dmxdev_events_queue *events,
					struct dmx_filter_event *event)
{
	int res;
	int new_write_index;
	int data_event;

	/* Check if we are adding an event that user already read its data */
	if (events->bytes_read_no_event) {
		data_event = 1;

		if (event->type == DMX_EVENT_NEW_PES)
			res = dvb_dmxdev_update_pes_event(event,
						events->bytes_read_no_event);
		else if (event->type == DMX_EVENT_NEW_SECTION)
			res = dvb_dmxdev_update_section_event(event,
						events->bytes_read_no_event);
		else if (event->type == DMX_EVENT_NEW_REC_CHUNK)
			res = dvb_dmxdev_update_rec_event(event,
						events->bytes_read_no_event);
		else
			data_event = 0;

		if (data_event) {
			if (res) {
				/*
				 * Data relevent to this event was fully
				 * consumed already, discard event.
				 */
				events->bytes_read_no_event -= res;
				return 0;
			}
			events->bytes_read_no_event = 0;
		} else {
			/*
			 * data was read beyond the non-data event,
			 * making it not relevant anymore
			 */
			return 0;
		}
	}

	new_write_index = dvb_dmxdev_advance_event_idx(events->write_index);
	if (new_write_index == events->read_index) {
		printk(KERN_ERR "dmxdev: events overflow\n");
		return -EOVERFLOW;
	}

	events->queue[events->write_index] = *event;
	events->write_index = new_write_index;

	return 0;
}

static int dvb_dmxdev_remove_event(struct dmxdev_events_queue *events,
					struct dmx_filter_event *event)
{
	if (events->notified_index == events->write_index)
		return -ENODATA;

	*event = events->queue[events->notified_index];

	events->notified_index =
		dvb_dmxdev_advance_event_idx(events->notified_index);

	return 0;
}

static int dvb_dmxdev_update_events(struct dmxdev_events_queue *events,
					int bytes_read)
{
	struct dmx_filter_event *event;
	int res;
	int data_event;

	/*
	 * Go through all events that were notified and
	 * remove them from the events queue if their respective
	 * data was read.
	 */
	while ((events->read_index != events->notified_index) &&
		   (bytes_read)) {
		event = events->queue + events->read_index;

		data_event = 1;

		if (event->type == DMX_EVENT_NEW_PES)
			res = dvb_dmxdev_update_pes_event(event, bytes_read);
		else if (event->type == DMX_EVENT_NEW_SECTION)
			res = dvb_dmxdev_update_section_event(event,
								bytes_read);
		else if (event->type == DMX_EVENT_NEW_REC_CHUNK)
			res = dvb_dmxdev_update_rec_event(event, bytes_read);
		else
			data_event = 0;

		if (data_event) {
			if (res) {
				/*
				 * Data relevant to this event was
				 * fully consumed, remove it from the queue.
				 */
				bytes_read -= res;
				events->read_index =
					dvb_dmxdev_advance_event_idx(
						events->read_index);
			} else {
				bytes_read = 0;
			}
		} else {
			/*
			 * non-data event was already notified,
			 * no need to keep it
			 */
			events->read_index = dvb_dmxdev_advance_event_idx(
						events->read_index);
		}
	}

	if (!bytes_read)
		return 0;

	/*
	 * If we reached here it means:
	 * bytes_read != 0
	 * events->read_index == events->notified_index
	 * Check if there are pending events in the queue
	 * which the user didn't read while their relevant data
	 * was read.
	 */
	while ((events->notified_index != events->write_index) &&
		   (bytes_read)) {
		event = events->queue + events->notified_index;

		data_event = 1;

		if (event->type == DMX_EVENT_NEW_PES)
			res = dvb_dmxdev_update_pes_event(event, bytes_read);
		else if (event->type == DMX_EVENT_NEW_SECTION)
			res = dvb_dmxdev_update_section_event(event,
								bytes_read);
		else if (event->type == DMX_EVENT_NEW_REC_CHUNK)
			res = dvb_dmxdev_update_rec_event(event, bytes_read);
		else
			data_event = 0;

		if (data_event) {
			if (res) {
				/*
				 * Data relevent to this event was
				 * fully consumed, remove it from the queue.
				 */
				bytes_read -= res;
				events->notified_index =
					dvb_dmxdev_advance_event_idx(
						events->notified_index);
			} else {
				bytes_read = 0;
			}
		} else {
			if (bytes_read)
				/*
				 * data was read beyond the non-data event,
				 * making it not relevant anymore
				 */
				events->notified_index =
					dvb_dmxdev_advance_event_idx(
						events->notified_index);
		}

		events->read_index = events->notified_index;
	}

	/*
	 * Check if data was read without having a respective
	 * event in the events-queue
	 */
	if (bytes_read)
		events->bytes_read_no_event += bytes_read;

	return 0;
}

static ssize_t dvb_dmxdev_buffer_read(struct dvb_ringbuffer *src,
					int non_blocking, char __user *buf,
					size_t count, loff_t *ppos)
{
	size_t todo;
	ssize_t avail;
	ssize_t ret = 0;

	if (!src->data)
		return 0;

	if (src->error) {
		ret = src->error;
		src->error = 0;
		return ret;
	}

	for (todo = count; todo > 0; todo -= ret) {
		if (non_blocking && dvb_ringbuffer_empty(src)) {
			ret = -EWOULDBLOCK;
			break;
		}

		ret = wait_event_interruptible(src->queue, (!src->data) ||
						!dvb_ringbuffer_empty(src) ||
						(src->error != 0));
		if (ret < 0)
			break;

		if (!src->data)
			return 0;

		if (src->error) {
			ret = src->error;
			src->error = 0;
			break;
		}

		avail = dvb_ringbuffer_avail(src);
		if (avail > todo)
			avail = todo;

		ret = dvb_ringbuffer_read_user(src, buf, avail);
		if (ret < 0)
			break;

		buf += ret;
	}

	if (count - todo) /* some data was read? */
		wake_up_all(&src->queue);

	return (count - todo) ? (count - todo) : ret;
}

static struct dmx_frontend *get_fe(struct dmx_demux *demux, int type)
{
	struct list_head *head, *pos;

	head = demux->get_frontends(demux);
	if (!head)
		return NULL;
	list_for_each(pos, head)
		if (DMX_FE_ENTRY(pos)->source == type)
			return DMX_FE_ENTRY(pos);

	return NULL;
}

static int dvr_input_thread_entry(void *arg)
{
	struct dmxdev *dmxdev = arg;
	struct dvb_ringbuffer *src = &dmxdev->dvr_input_buffer;
	int ret;
	size_t todo;
	int bytes_written;
	size_t split;

	while (1) {
		/* wait for input */
		ret = wait_event_interruptible(
			src->queue,
			(dvb_ringbuffer_avail(src) > 188) ||
			(src->error != 0) ||
			dmxdev->dvr_in_exit);

		if (ret < 0)
			break;

		spin_lock(&dmxdev->dvr_in_lock);

		if (dmxdev->exit || dmxdev->dvr_in_exit) {
			spin_unlock(&dmxdev->dvr_in_lock);
			break;
		}

		if (src->error) {
			spin_unlock(&dmxdev->dvr_in_lock);
			wake_up_all(&src->queue);
			break;
		}

		if (!src->data) {
			spin_unlock(&dmxdev->dvr_in_lock);
			continue;
		}

		dmxdev->dvr_processing_input = 1;

		ret = dvb_ringbuffer_avail(src);
		todo = ret;

		split = (src->pread + ret > src->size) ?
				src->size - src->pread :
				0;
		/*
		 * In DVR PULL mode, write might block.
		 * Lock on DVR buffer is released before calling to
		 * write, if DVR was released meanwhile, dvr_in_exit is
		 * prompted. Lock is acquired when updating the read pointer
		 * again to preserve read/write pointers consistency
		 */
		if (split > 0) {
			spin_unlock(&dmxdev->dvr_in_lock);
			bytes_written = dmxdev->demux->write(dmxdev->demux,
						src->data + src->pread,
						split);

			if (bytes_written < 0) {
				printk(KERN_ERR "dmxdev: dvr write error %d\n",
					bytes_written);
				continue;
			}

			if (dmxdev->dvr_in_exit)
				break;

			spin_lock(&dmxdev->dvr_in_lock);

			todo -= bytes_written;
			DVB_RINGBUFFER_SKIP(src, bytes_written);
			if (bytes_written < split) {
				dmxdev->dvr_processing_input = 0;
				spin_unlock(&dmxdev->dvr_in_lock);
				wake_up_all(&src->queue);
				continue;
			}

		}

		spin_unlock(&dmxdev->dvr_in_lock);
		bytes_written = dmxdev->demux->write(dmxdev->demux,
					src->data + src->pread, todo);

		if (bytes_written < 0) {
			printk(KERN_ERR "dmxdev: dvr write error %d\n",
				bytes_written);
			continue;
		}

		if (dmxdev->dvr_in_exit)
			break;

		spin_lock(&dmxdev->dvr_in_lock);

		DVB_RINGBUFFER_SKIP(src, bytes_written);
		dmxdev->dvr_processing_input = 0;
		spin_unlock(&dmxdev->dvr_in_lock);

		wake_up_all(&src->queue);
	}

	set_current_state(TASK_INTERRUPTIBLE);
	while (!kthread_should_stop()) {
		schedule();
		set_current_state(TASK_INTERRUPTIBLE);
	}
	set_current_state(TASK_RUNNING);

	return 0;
}


static int dvb_dvr_open(struct inode *inode, struct file *file)
{
	struct dvb_device *dvbdev = file->private_data;
	struct dmxdev *dmxdev = dvbdev->priv;
	struct dmx_frontend *front;
	void *mem;

	dprintk("function : %s(%X)\n",
			__func__,
			(file->f_flags & O_ACCMODE));

	if (mutex_lock_interruptible(&dmxdev->mutex))
		return -ERESTARTSYS;

	if (dmxdev->exit) {
		mutex_unlock(&dmxdev->mutex);
		return -ENODEV;
	}

	if ((file->f_flags & O_ACCMODE) == O_RDWR) {
		if (!(dmxdev->capabilities & DMXDEV_CAP_DUPLEX)) {
			mutex_unlock(&dmxdev->mutex);
			return -EOPNOTSUPP;
		}
	}

	if ((file->f_flags & O_ACCMODE) == O_RDONLY) {
		if (!dvbdev->readers) {
			mutex_unlock(&dmxdev->mutex);
			return -EBUSY;
		}
		mem = vmalloc_user(DVR_BUFFER_SIZE);
		if (!mem) {
			mutex_unlock(&dmxdev->mutex);
			return -ENOMEM;
		}
		dvb_ringbuffer_init(&dmxdev->dvr_buffer, mem, DVR_BUFFER_SIZE);
		dvb_dmxdev_flush_events(&dmxdev->dvr_output_events);
		dmxdev->dvr_feeds_count = 0;
		dmxdev->dvr_buffer_mode = DMX_BUFFER_MODE_INTERNAL;
		dmxdev->dvr_priv_buff_handle = NULL;

		dvbdev->readers--;
	} else if (!dvbdev->writers) {
		dmxdev->dvr_in_exit = 0;
		dmxdev->dvr_processing_input = 0;
		dmxdev->dvr_orig_fe = dmxdev->demux->frontend;

		if (!dmxdev->demux->write) {
			mutex_unlock(&dmxdev->mutex);
			return -EOPNOTSUPP;
		}

		front = get_fe(dmxdev->demux, DMX_MEMORY_FE);

		if (!front) {
			mutex_unlock(&dmxdev->mutex);
			return -EINVAL;
		}

		mem = vmalloc_user(DVR_BUFFER_SIZE);
		if (!mem) {
			mutex_unlock(&dmxdev->mutex);
			return -ENOMEM;
		}

		dmxdev->demux->disconnect_frontend(dmxdev->demux);
		dmxdev->demux->connect_frontend(dmxdev->demux, front);
		dmxdev->dvr_input_buffer_mode = DMX_BUFFER_MODE_INTERNAL;

		dvb_ringbuffer_init(&dmxdev->dvr_input_buffer,
							mem,
							DVR_BUFFER_SIZE);

		dmxdev->demux->dvr_input.priv_handle = NULL;
		dmxdev->demux->dvr_input.ringbuff = &dmxdev->dvr_input_buffer;
		dvbdev->writers--;

		dmxdev->dvr_input_thread =
			kthread_run(
				dvr_input_thread_entry,
				(void *)dmxdev,
				"dvr_input");

		if (IS_ERR(dmxdev->dvr_input_thread)) {
			mutex_unlock(&dmxdev->mutex);
			return -ENOMEM;
		}
	}

	dvbdev->users++;
	mutex_unlock(&dmxdev->mutex);
	return 0;
}

static int dvb_dvr_release(struct inode *inode, struct file *file)
{
	struct dvb_device *dvbdev = file->private_data;
	struct dmxdev *dmxdev = dvbdev->priv;

	mutex_lock(&dmxdev->mutex);

	if ((file->f_flags & O_ACCMODE) == O_RDONLY) {
		dvbdev->readers++;
		if (dmxdev->dvr_buffer.data) {
			void *mem = dmxdev->dvr_buffer.data;
			mb();
			spin_lock_irq(&dmxdev->lock);
			dmxdev->dvr_buffer.data = NULL;
			spin_unlock_irq(&dmxdev->lock);
			wake_up_all(&dmxdev->dvr_buffer.queue);

			if (dmxdev->dvr_buffer_mode == DMX_BUFFER_MODE_INTERNAL)
				vfree(mem);
		}

		if ((dmxdev->dvr_buffer_mode == DMX_BUFFER_MODE_EXTERNAL) &&
			dmxdev->dvr_priv_buff_handle) {
			dmxdev->demux->unmap_buffer(dmxdev->demux,
					dmxdev->dvr_priv_buff_handle);
			dmxdev->dvr_priv_buff_handle = NULL;
		}
	} else {
		int i;

		dmxdev->dvr_in_exit = 1;
		wake_up_all(&dmxdev->dvr_input_buffer.queue);

		/*
		 * There might be dmx filters reading now from DVR
		 * device, in PULL mode, they might be also stalled
		 * on output, signal to them that DVR is exiting.
		 */
		if (dmxdev->playback_mode == DMX_PB_MODE_PULL) {
			wake_up_all(&dmxdev->dvr_buffer.queue);

			for (i = 0; i < dmxdev->filternum; i++)
				if (dmxdev->filter[i].state == DMXDEV_STATE_GO)
					wake_up_all(
					&dmxdev->filter[i].buffer.queue);
		}

		/* notify kernel demux that we are canceling */
		if (dmxdev->demux->write_cancel)
			dmxdev->demux->write_cancel(dmxdev->demux);

		/*
		 * Now stop dvr-input thread so that no one
		 * would process data from dvr input buffer any more
		 * before it gets freed.
		 */
		kthread_stop(dmxdev->dvr_input_thread);

		dvbdev->writers++;
		dmxdev->demux->disconnect_frontend(dmxdev->demux);
		dmxdev->demux->connect_frontend(dmxdev->demux,
						dmxdev->dvr_orig_fe);

		if (dmxdev->dvr_input_buffer.data) {
			void *mem = dmxdev->dvr_input_buffer.data;
			mb();
			spin_lock_irq(&dmxdev->dvr_in_lock);
			dmxdev->dvr_input_buffer.data = NULL;
			spin_unlock_irq(&dmxdev->dvr_in_lock);

			if (dmxdev->dvr_input_buffer_mode ==
				DMX_BUFFER_MODE_INTERNAL)
				vfree(mem);
		}

		if ((dmxdev->dvr_input_buffer_mode ==
			DMX_BUFFER_MODE_EXTERNAL) &&
			(dmxdev->demux->dvr_input.priv_handle)) {
			dmxdev->demux->unmap_buffer(dmxdev->demux,
					dmxdev->demux->dvr_input.priv_handle);
			dmxdev->demux->dvr_input.priv_handle = NULL;
		}
	}
	/* TODO */
	dvbdev->users--;
	if (dvbdev->users == 1 && dmxdev->exit == 1) {
		fops_put(file->f_op);
		file->f_op = NULL;
		mutex_unlock(&dmxdev->mutex);
		wake_up(&dvbdev->wait_queue);
	} else
		mutex_unlock(&dmxdev->mutex);

	return 0;
}


static int dvb_dvr_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct dvb_device *dvbdev = filp->private_data;
	struct dmxdev *dmxdev = dvbdev->priv;
	struct dvb_ringbuffer *buffer;
	enum dmx_buffer_mode buffer_mode;
	int vma_size;
	int buffer_size;
	int ret;

	if (((filp->f_flags & O_ACCMODE) == O_RDONLY) &&
		(vma->vm_flags & VM_WRITE))
		return -EINVAL;

	if (mutex_lock_interruptible(&dmxdev->mutex))
		return -ERESTARTSYS;

	if (dmxdev->exit) {
		mutex_unlock(&dmxdev->mutex);
		return -ENODEV;
	}

	if ((filp->f_flags & O_ACCMODE) == O_RDONLY) {
		buffer = &dmxdev->dvr_buffer;
		buffer_mode = dmxdev->dvr_buffer_mode;
	} else {
		buffer = &dmxdev->dvr_input_buffer;
		buffer_mode = dmxdev->dvr_input_buffer_mode;
	}

	if (buffer_mode == DMX_BUFFER_MODE_EXTERNAL) {
		mutex_unlock(&dmxdev->mutex);
		return -EINVAL;
	}

	vma_size = vma->vm_end - vma->vm_start;

	/* Make sure requested mapping is not larger than buffer size */
	buffer_size = buffer->size + (PAGE_SIZE-1);
	buffer_size = buffer_size & ~(PAGE_SIZE-1);

	if (vma_size != buffer_size) {
		mutex_unlock(&dmxdev->mutex);
		return -EINVAL;
	}

	ret = remap_vmalloc_range(vma, buffer->data, 0);
	if (ret) {
		mutex_unlock(&dmxdev->mutex);
		return ret;
	}

	vma->vm_flags |= VM_RESERVED;
	vma->vm_flags |= VM_DONTEXPAND;

	mutex_unlock(&dmxdev->mutex);
	return ret;
}

static ssize_t dvb_dvr_write(struct file *file, const char __user *buf,
			     size_t count, loff_t *ppos)
{
	struct dvb_device *dvbdev = file->private_data;
	struct dmxdev *dmxdev = dvbdev->priv;
	struct dvb_ringbuffer *src = &dmxdev->dvr_input_buffer;
	int ret;
	size_t todo;
	ssize_t free_space;

	if (!dmxdev->demux->write)
		return -EOPNOTSUPP;

	if (((file->f_flags & O_ACCMODE) == O_RDONLY) ||
		(!src->data))
		return -EINVAL;

	if ((file->f_flags & O_NONBLOCK) &&
		(dvb_ringbuffer_free(src) == 0))
		return -EWOULDBLOCK;

	ret = 0;
	for (todo = count; todo > 0; todo -= ret) {
		ret = wait_event_interruptible(src->queue,
						   (!src->data) ||
					       (dvb_ringbuffer_free(src)) ||
					       (src->error != 0) ||
					       (dmxdev->dvr_in_exit));

		if (ret < 0)
			return ret;

		if (mutex_lock_interruptible(&dmxdev->mutex))
			return -ERESTARTSYS;

		if (!src->data) {
			mutex_unlock(&dmxdev->mutex);
			return 0;
		}

		if (dmxdev->exit || dmxdev->dvr_in_exit) {
			mutex_unlock(&dmxdev->mutex);
			return -ENODEV;
		}

		if (src->error) {
			ret = src->error;
			dvb_ringbuffer_flush(src);
			mutex_unlock(&dmxdev->mutex);
			wake_up_all(&src->queue);
			return ret;
		}

		free_space = dvb_ringbuffer_free(src);

		if (free_space > todo)
			free_space = todo;

		ret = dvb_ringbuffer_write_user(src, buf, free_space);

		if (ret < 0) {
			mutex_unlock(&dmxdev->mutex);
			return ret;
		}

		buf += ret;

		mutex_unlock(&dmxdev->mutex);
		wake_up_all(&src->queue);
	}

	return (count - todo) ? (count - todo) : ret;
}

static ssize_t dvb_dvr_read(struct file *file, char __user *buf, size_t count,
			    loff_t *ppos)
{
	ssize_t res;
	struct dvb_device *dvbdev = file->private_data;
	struct dmxdev *dmxdev = dvbdev->priv;
	ssize_t flush_len;

	if (dmxdev->exit)
		return -ENODEV;

	res = dvb_dmxdev_buffer_read(&dmxdev->dvr_buffer,
				      file->f_flags & O_NONBLOCK,
				      buf, count, ppos);

	if (res > 0) {
		dvb_dmxdev_notify_data_read(dmxdev->dvr_feed, res);
		spin_lock_irq(&dmxdev->lock);
		dvb_dmxdev_update_events(&dmxdev->dvr_output_events, res);
		spin_unlock_irq(&dmxdev->lock);

		/*
		 * in PULL mode, we might be stalling on
		 * event queue, so need to wake-up waiters
		 */
		if (dmxdev->playback_mode == DMX_PB_MODE_PULL)
			wake_up_all(&dmxdev->dvr_buffer.queue);
	} else if (res == -EOVERFLOW) {
		/*
		 * When buffer overflowed, demux-dev marked the buffer in
		 * error state.
		 * Data from underlying driver is discarded until
		 * user gets notified that buffer has overflowed.
		 * Now that the user is notified, notify underlying
		 * driver that data was flushed from output buffer.
		 */
		flush_len = dvb_ringbuffer_avail(&dmxdev->dvr_buffer);
		dvb_ringbuffer_flush(&dmxdev->dvr_buffer);
		dvb_dmxdev_notify_data_read(dmxdev->dvr_feed, flush_len);
	}

	return res;
}

static int dvb_dvr_set_buffer_size(struct dmxdev *dmxdev,
						unsigned int f_flags,
						unsigned long size)
{
	struct dvb_ringbuffer *buf;
	void *newmem;
	void *oldmem;
	spinlock_t *lock;
	enum dmx_buffer_mode buffer_mode;

#ifdef CONFIG_DEBUG_PRINTK
	dprintk("function : %s\n", __func__);
#else
	d;
#endif

	if ((f_flags & O_ACCMODE) == O_RDONLY) {
		buf = &dmxdev->dvr_buffer;
		lock = &dmxdev->lock;
		buffer_mode = dmxdev->dvr_buffer_mode;
	} else {
		buf = &dmxdev->dvr_input_buffer;
		lock = &dmxdev->dvr_in_lock;
		buffer_mode = dmxdev->dvr_input_buffer_mode;
	}

	if (buf->size == size)
		return 0;
	if ((!size) || (buffer_mode == DMX_BUFFER_MODE_EXTERNAL))
		return -EINVAL;

	newmem = vmalloc_user(size);
	if (!newmem)
		return -ENOMEM;

	oldmem = buf->data;

	spin_lock_irq(lock);

	if (((f_flags & O_ACCMODE) != O_RDONLY) &&
		(dmxdev->dvr_processing_input)) {
		spin_unlock_irq(lock);
		vfree(oldmem);
		return -EBUSY;
	}

	buf->data = newmem;
	buf->size = size;

	/* reset and not flush in case the buffer shrinks */
	dvb_ringbuffer_reset(buf);

	spin_unlock_irq(lock);

	vfree(oldmem);

	return 0;
}

static int dvb_dvr_set_buffer_mode(struct dmxdev *dmxdev,
			unsigned int f_flags, enum dmx_buffer_mode mode)
{
	struct dvb_ringbuffer *buf;
	spinlock_t *lock;
	enum dmx_buffer_mode *buffer_mode;
	void **buff_handle;
	void *oldmem;

	if ((mode != DMX_BUFFER_MODE_INTERNAL) &&
		(mode != DMX_BUFFER_MODE_EXTERNAL))
		return -EINVAL;

	if ((mode == DMX_BUFFER_MODE_INTERNAL) &&
		(dmxdev->capabilities & DMXDEV_CAP_EXTERNAL_BUFFS_ONLY))
		return -EINVAL;

	if ((mode == DMX_BUFFER_MODE_EXTERNAL) &&
		(!dmxdev->demux->map_buffer || !dmxdev->demux->unmap_buffer))
		return -EINVAL;

	if ((f_flags & O_ACCMODE) == O_RDONLY) {
		buf = &dmxdev->dvr_buffer;
		lock = &dmxdev->lock;
		buffer_mode = &dmxdev->dvr_buffer_mode;
		buff_handle = &dmxdev->dvr_priv_buff_handle;
	} else {
		buf = &dmxdev->dvr_input_buffer;
		lock = &dmxdev->dvr_in_lock;
		buffer_mode = &dmxdev->dvr_input_buffer_mode;
		buff_handle = &dmxdev->demux->dvr_input.priv_handle;
	}

	if (mode == *buffer_mode)
		return 0;

	oldmem = buf->data;
	spin_lock_irq(lock);
	buf->data = NULL;
	spin_unlock_irq(lock);

	*buffer_mode = mode;

	if (mode == DMX_BUFFER_MODE_INTERNAL) {
		/* switched from external to internal */
		if (*buff_handle) {
			dmxdev->demux->unmap_buffer(dmxdev->demux,
				*buff_handle);
			*buff_handle = NULL;
		}

		/* set default internal buffer */
		dvb_dvr_set_buffer_size(dmxdev, f_flags, DVR_BUFFER_SIZE);
	} else if (oldmem) {
		/* switched from internal to external */
		vfree(oldmem);
	}

	return 0;
}

static int dvb_dvr_set_buffer(struct dmxdev *dmxdev,
			unsigned int f_flags, struct dmx_buffer *dmx_buffer)
{
	struct dvb_ringbuffer *buf;
	spinlock_t *lock;
	enum dmx_buffer_mode buffer_mode;
	void **buff_handle;
	void *newmem;
	void *oldmem;

	if ((f_flags & O_ACCMODE) == O_RDONLY) {
		buf = &dmxdev->dvr_buffer;
		lock = &dmxdev->lock;
		buffer_mode = dmxdev->dvr_buffer_mode;
		buff_handle = &dmxdev->dvr_priv_buff_handle;
	} else {
		buf = &dmxdev->dvr_input_buffer;
		lock = &dmxdev->dvr_in_lock;
		buffer_mode = dmxdev->dvr_input_buffer_mode;
		buff_handle = &dmxdev->demux->dvr_input.priv_handle;
	}

	if ((!dmx_buffer->size) ||
		(buffer_mode == DMX_BUFFER_MODE_INTERNAL))
		return -EINVAL;

	oldmem = *buff_handle;
	if (dmxdev->demux->map_buffer(dmxdev->demux, dmx_buffer,
				buff_handle, &newmem))
		return -ENOMEM;

	spin_lock_irq(lock);
	buf->data = newmem;
	buf->size = dmx_buffer->size;
	dvb_ringbuffer_reset(buf);
	spin_unlock_irq(lock);

	if (oldmem)
		dmxdev->demux->unmap_buffer(dmxdev->demux, oldmem);

	return 0;
}

static int dvb_dvr_get_event(struct dmxdev *dmxdev,
				unsigned int f_flags,
				struct dmx_filter_event *event)
{
	int res;
	ssize_t flush_len;

	if (!((f_flags & O_ACCMODE) == O_RDONLY))
		return -EINVAL;

	spin_lock_irq(&dmxdev->lock);

	res = dvb_dmxdev_remove_event(&dmxdev->dvr_output_events, event);

	if (event->type == DMX_EVENT_BUFFER_OVERFLOW) {
		/*
		 * When buffer overflowed, demux-dev marked the buffer in
		 * error state.
		 * Data from underlying driver is discarded until
		 * user gets notified that buffer has overflowed.
		 * Now that the user is notified, notify underlying
		 * driver that data was flushed from output buffer.
		 */
		flush_len = dvb_ringbuffer_avail(&dmxdev->dvr_buffer);
		dvb_ringbuffer_flush(&dmxdev->dvr_buffer);
		dvb_dmxdev_notify_data_read(dmxdev->dvr_feed, flush_len);
		dmxdev->dvr_buffer.error = 0;
	}

	spin_unlock_irq(&dmxdev->lock);

	/*
	 * in PULL mode, we might be stalling on
	 * event queue, so need to wake-up waiters
	 */
	if (dmxdev->playback_mode == DMX_PB_MODE_PULL)
		wake_up_all(&dmxdev->dvr_buffer.queue);

	return res;
}

static int dvb_dvr_get_buffer_status(struct dmxdev *dmxdev,
				unsigned int f_flags,
				struct dmx_buffer_status *dmx_buffer_status)
{
	struct dvb_ringbuffer *buf;
	spinlock_t *lock;
	ssize_t flush_len;

	if ((f_flags & O_ACCMODE) == O_RDONLY) {
		buf = &dmxdev->dvr_buffer;
		lock = &dmxdev->lock;
	} else {
		buf = &dmxdev->dvr_input_buffer;
		lock = &dmxdev->dvr_in_lock;
	}

	spin_lock_irq(lock);

	dmx_buffer_status->error = buf->error;
	if (buf->error) {
		if (buf->error == -EOVERFLOW) {
			/*
			 * When buffer overflowed, demux-dev flushed the
			 * buffer and marked the buffer in error state.
			 * Data from underlying driver is discarded until
			 * user gets notified that buffer has overflowed.
			 * Now that the user is notified, notify underlying
			 * driver that data was flushed from output buffer.
			 */
			flush_len = dvb_ringbuffer_avail(buf);
			dvb_ringbuffer_flush(buf);
			dvb_dmxdev_notify_data_read(dmxdev->dvr_feed,
				flush_len);
		}

		buf->error = 0;
	}

	dmx_buffer_status->fullness = dvb_ringbuffer_avail(buf);
	dmx_buffer_status->free_bytes = dvb_ringbuffer_free(buf);
	dmx_buffer_status->read_offset = buf->pread;
	dmx_buffer_status->write_offset = buf->pwrite;
	dmx_buffer_status->size = buf->size;

	spin_unlock_irq(lock);

	return 0;
}

static int dvb_dvr_release_data(struct dmxdev *dmxdev,
					unsigned int f_flags,
					u32 bytes_count)
{
	ssize_t buff_fullness;

	if (!((f_flags & O_ACCMODE) == O_RDONLY))
		return -EINVAL;

	if (!bytes_count)
		return 0;

	buff_fullness = dvb_ringbuffer_avail(&dmxdev->dvr_buffer);

	if (bytes_count > buff_fullness)
		return -EINVAL;

	DVB_RINGBUFFER_SKIP(&dmxdev->dvr_buffer, bytes_count);

	dvb_dmxdev_notify_data_read(dmxdev->dvr_feed, bytes_count);
	spin_lock_irq(&dmxdev->lock);
	dvb_dmxdev_update_events(&dmxdev->dvr_output_events, bytes_count);
	spin_unlock_irq(&dmxdev->lock);

	wake_up_all(&dmxdev->dvr_buffer.queue);
	return 0;
}

static int dvb_dvr_feed_data(struct dmxdev *dmxdev,
					unsigned int f_flags,
					u32 bytes_count)
{
	ssize_t free_space;
	struct dvb_ringbuffer *buffer = &dmxdev->dvr_input_buffer;

	if ((f_flags & O_ACCMODE) == O_RDONLY)
		return -EINVAL;

	if (!bytes_count)
		return 0;

	free_space = dvb_ringbuffer_free(buffer);

	if (bytes_count > free_space)
		return -EINVAL;

	buffer->pwrite =
		(buffer->pwrite + bytes_count) % buffer->size;

	wake_up_all(&buffer->queue);

	return 0;
}

static inline void dvb_dmxdev_filter_state_set(struct dmxdev_filter
					       *dmxdevfilter, int state)
{
	spin_lock_irq(&dmxdevfilter->dev->lock);
	dmxdevfilter->state = state;
	spin_unlock_irq(&dmxdevfilter->dev->lock);
}

static int dvb_dmxdev_set_buffer_size(struct dmxdev_filter *dmxdevfilter,
				      unsigned long size)
{
	struct dvb_ringbuffer *buf = &dmxdevfilter->buffer;
	void *newmem;
	void *oldmem;

	if (buf->size == size)
		return 0;
	if ((!size) || (dmxdevfilter->buffer_mode == DMX_BUFFER_MODE_EXTERNAL))
		return -EINVAL;
	if (dmxdevfilter->state >= DMXDEV_STATE_GO)
		return -EBUSY;

	newmem = vmalloc_user(size);
	if (!newmem)
		return -ENOMEM;

	oldmem = buf->data;

	spin_lock_irq(&dmxdevfilter->dev->lock);
	buf->data = newmem;
	buf->size = size;

	/* reset and not flush in case the buffer shrinks */
	dvb_ringbuffer_reset(buf);
	spin_unlock_irq(&dmxdevfilter->dev->lock);

	vfree(oldmem);
	return 0;
}

static int dvb_dmxdev_set_buffer_mode(struct dmxdev_filter *dmxdevfilter,
					enum dmx_buffer_mode mode)
{
	struct dvb_ringbuffer *buf = &dmxdevfilter->buffer;
	struct dmxdev *dmxdev = dmxdevfilter->dev;
	void *oldmem;

	if (dmxdevfilter->state >= DMXDEV_STATE_GO)
		return -EBUSY;

	if ((mode != DMX_BUFFER_MODE_INTERNAL) &&
		(mode != DMX_BUFFER_MODE_EXTERNAL))
		return -EINVAL;

	if ((mode == DMX_BUFFER_MODE_INTERNAL) &&
		(dmxdev->capabilities & DMXDEV_CAP_EXTERNAL_BUFFS_ONLY))
		return -EINVAL;

	if ((mode == DMX_BUFFER_MODE_EXTERNAL) &&
		(!dmxdev->demux->map_buffer || !dmxdev->demux->unmap_buffer))
		return -EINVAL;

	if (mode == dmxdevfilter->buffer_mode)
		return 0;

	oldmem = buf->data;
	spin_lock_irq(&dmxdevfilter->dev->lock);
	buf->data = NULL;
	spin_unlock_irq(&dmxdevfilter->dev->lock);

	dmxdevfilter->buffer_mode = mode;

	if (mode == DMX_BUFFER_MODE_INTERNAL) {
		/* switched from external to internal */
		if (dmxdevfilter->priv_buff_handle) {
			dmxdev->demux->unmap_buffer(dmxdev->demux,
				dmxdevfilter->priv_buff_handle);
			dmxdevfilter->priv_buff_handle = NULL;
		}
	} else if (oldmem) {
		/* switched from internal to external */
		vfree(oldmem);
	}

	return 0;
}

static int dvb_dmxdev_set_buffer(struct dmxdev_filter *dmxdevfilter,
					struct dmx_buffer *buffer)
{
	struct dvb_ringbuffer *buf = &dmxdevfilter->buffer;
	struct dmxdev *dmxdev = dmxdevfilter->dev;
	void *newmem;
	void *oldmem;

	if (dmxdevfilter->state >= DMXDEV_STATE_GO)
		return -EBUSY;

	if ((!buffer->size) ||
		(dmxdevfilter->buffer_mode == DMX_BUFFER_MODE_INTERNAL))
		return -EINVAL;

	oldmem = dmxdevfilter->priv_buff_handle;
	if (dmxdev->demux->map_buffer(dmxdev->demux, buffer,
			&dmxdevfilter->priv_buff_handle, &newmem))
		return -ENOMEM;

	spin_lock_irq(&dmxdevfilter->dev->lock);
	buf->data = newmem;
	buf->size = buffer->size;
	dvb_ringbuffer_reset(buf);
	spin_unlock_irq(&dmxdevfilter->dev->lock);

	if (oldmem)
		dmxdev->demux->unmap_buffer(dmxdev->demux, oldmem);

	return 0;
}

static int dvb_dmxdev_set_tsp_out_format(struct dmxdev_filter *dmxdevfilter,
				enum dmx_tsp_format_t dmx_tsp_format)
{
	if (dmxdevfilter->state >= DMXDEV_STATE_GO)
		return -EBUSY;

	if ((dmx_tsp_format > DMX_TSP_FORMAT_192_HEAD) ||
		(dmx_tsp_format < DMX_TSP_FORMAT_188))
		return -EINVAL;

	dmxdevfilter->dmx_tsp_format = dmx_tsp_format;

	return 0;
}

static int dvb_dmxdev_set_decoder_buffer_size(
	struct dmxdev_filter *dmxdevfilter,
	unsigned long size)
{
	if (0 == size)
		return -EINVAL;

	if (dmxdevfilter->decoder_buffers.buffers_size == size)
		return 0;

	if (dmxdevfilter->state >= DMXDEV_STATE_GO)
		return -EBUSY;

	/*
	 * In case decoder buffers were already set before to some external
	 * buffers, setting the decoder buffer size alone implies transition
	 * to internal buffer mode.
	 */
	dmxdevfilter->decoder_buffers.buffers_size = size;
	dmxdevfilter->decoder_buffers.buffers_num = 0;
	dmxdevfilter->decoder_buffers.is_linear = 0;
	return 0;
}

static int dvb_dmxdev_set_source(struct dmxdev_filter *dmxdevfilter,
					dmx_source_t *source)
{
	struct dmxdev *dev;

	if (dmxdevfilter->state == DMXDEV_STATE_GO)
		return -EBUSY;

	dev = dmxdevfilter->dev;

	dev->source = *source;

	if (dev->demux->set_source)
		return dev->demux->set_source(dev->demux, source);

	return 0;
}

static int dvb_dmxdev_reuse_decoder_buf(struct dmxdev_filter *dmxdevfilter,
						int cookie)
{
	if ((dmxdevfilter->type == DMXDEV_TYPE_PES) &&
		(dmxdevfilter->params.pes.output == DMX_OUT_DECODER)) {
		struct dmxdev_feed *feed;
		int ret = -ENODEV;

		/* Only one feed should be in the list in case of decoder */
		feed = list_first_entry(&dmxdevfilter->feed.ts,
					struct dmxdev_feed, next);

		if (feed->ts->reuse_decoder_buffer)
			ret = feed->ts->reuse_decoder_buffer(
							feed->ts,
							cookie);

		return ret;
	}

	return -EPERM;
}

static int dvb_dmxdev_ts_fullness_callback(
				struct dmx_ts_feed *filter,
				int required_space)
{
	struct dmxdev_filter *dmxdevfilter = filter->priv;
	struct dvb_ringbuffer *src;
	struct dmxdev_events_queue *events;
	int ret;

	if (!dmxdevfilter) {
		pr_err("%s: NULL demux filter object!\n", __func__);
		return -ENODEV;
	}

	if (dmxdevfilter->params.pes.output != DMX_OUT_TS_TAP) {
		src = &dmxdevfilter->buffer;
		events = &dmxdevfilter->events;
	} else {
		src = &dmxdevfilter->dev->dvr_buffer;
		events = &dmxdevfilter->dev->dvr_output_events;
	}

	do {
		ret = 0;

		if (dmxdevfilter->dev->dvr_in_exit)
			return -ENODEV;

		spin_lock(&dmxdevfilter->dev->lock);

		if ((!src->data) ||
			(dmxdevfilter->state != DMXDEV_STATE_GO))
			ret = -EINVAL;
		else if (src->error)
			ret = src->error;

		if (ret) {
			spin_unlock(&dmxdevfilter->dev->lock);
			return ret;
		}

		if ((required_space <= dvb_ringbuffer_free(src)) &&
			(!dvb_dmxdev_events_is_full(events))) {
			spin_unlock(&dmxdevfilter->dev->lock);
			return 0;
		}

		spin_unlock(&dmxdevfilter->dev->lock);

		ret = wait_event_interruptible(src->queue,
				(!src->data) ||
				((dvb_ringbuffer_free(src) >= required_space) &&
				 (!dvb_dmxdev_events_is_full(events))) ||
				(src->error != 0) ||
				(dmxdevfilter->state != DMXDEV_STATE_GO) ||
				dmxdevfilter->dev->dvr_in_exit);

		if (ret < 0)
			return ret;
	} while (1);
}

static int dvb_dmxdev_sec_fullness_callback(
				struct dmx_section_filter *filter,
				int required_space)
{
	struct dmxdev_filter *dmxdevfilter = filter->priv;
	struct dvb_ringbuffer *src;
	struct dmxdev_events_queue *events;
	int ret;

	if (!dmxdevfilter) {
		pr_err("%s: NULL demux filter object!\n", __func__);
		return -ENODEV;
	}

	src = &dmxdevfilter->buffer;
	events = &dmxdevfilter->events;

	do {
		ret = 0;

		if (dmxdevfilter->dev->dvr_in_exit)
			return -ENODEV;

		spin_lock(&dmxdevfilter->dev->lock);

		if ((!src->data) ||
			(dmxdevfilter->state != DMXDEV_STATE_GO))
			ret = -EINVAL;
		else if (src->error)
			ret = src->error;

		if (ret) {
			spin_unlock(&dmxdevfilter->dev->lock);
			return ret;
		}

		if ((required_space <= dvb_ringbuffer_free(src)) &&
			(!dvb_dmxdev_events_is_full(events))) {
			spin_unlock(&dmxdevfilter->dev->lock);
			return 0;
		}

		spin_unlock(&dmxdevfilter->dev->lock);

		ret = wait_event_interruptible(src->queue,
				(!src->data) ||
				((dvb_ringbuffer_free(src) >= required_space) &&
				 (!dvb_dmxdev_events_is_full(events))) ||
				(src->error != 0) ||
				(dmxdevfilter->state != DMXDEV_STATE_GO) ||
				dmxdevfilter->dev->dvr_in_exit);

		if (ret < 0)
			return ret;
	} while (1);
}

static int dvb_dmxdev_set_playback_mode(struct dmxdev_filter *dmxdevfilter,
					enum dmx_playback_mode_t playback_mode)
{
	struct dmxdev *dmxdev = dmxdevfilter->dev;

	if ((playback_mode != DMX_PB_MODE_PUSH) &&
		(playback_mode != DMX_PB_MODE_PULL))
		return -EINVAL;

	if (((dmxdev->source < DMX_SOURCE_DVR0) ||
		!dmxdev->demux->set_playback_mode ||
		!(dmxdev->capabilities & DMXDEV_CAP_PULL_MODE)) &&
		(playback_mode == DMX_PB_MODE_PULL))
		return -EPERM;

	if (dmxdevfilter->state == DMXDEV_STATE_GO)
		return -EBUSY;

	dmxdev->playback_mode = playback_mode;

	return dmxdev->demux->set_playback_mode(
				dmxdev->demux,
				dmxdev->playback_mode,
				dvb_dmxdev_ts_fullness_callback,
				dvb_dmxdev_sec_fullness_callback);
}

static int dvb_dmxdev_get_buffer_status(
		struct dmxdev_filter *dmxdevfilter,
		struct dmx_buffer_status *dmx_buffer_status)
{
	struct dvb_ringbuffer *buf = &dmxdevfilter->buffer;
	ssize_t flush_len;

	/*
	 * Note: Taking the dmxdevfilter->dev->lock spinlock is required only
	 * when getting the status of the Demux-userspace data ringbuffer .
	 * In case we are getting the status of a decoder buffer, taking this
	 * spinlock is not required and in fact might lead to a deadlock.
	 */
	if ((dmxdevfilter->type == DMXDEV_TYPE_PES) &&
		(dmxdevfilter->params.pes.output == DMX_OUT_DECODER)) {
		struct dmxdev_feed *feed;
		int ret;

		/* Only one feed should be in the list in case of decoder */
		feed = list_first_entry(&dmxdevfilter->feed.ts,
					struct dmxdev_feed, next);

		/* Ask for status of decoder's buffer from underlying HW */
		if (feed->ts->get_decoder_buff_status)
			ret = feed->ts->get_decoder_buff_status(
					feed->ts,
					dmx_buffer_status);
		else
			ret = -ENODEV;

		return ret;
	}

	spin_lock_irq(&dmxdevfilter->dev->lock);

	if (!buf->data) {
		spin_unlock_irq(&dmxdevfilter->dev->lock);
		return -EINVAL;
	}

	dmx_buffer_status->error = buf->error;
	if (buf->error) {
		if (buf->error == -EOVERFLOW) {
			/*
			 * When buffer overflowed, demux-dev marked the buffer
			 * in error state.
			 * Data from underlying driver is discarded until
			 * user gets notified that buffer has overflowed.
			 * Now that the user is notified, notify underlying
			 * driver that data was flushed from output buffer.
			 */
			flush_len = dvb_ringbuffer_avail(buf);
			dvb_ringbuffer_flush(buf);
			dvb_dmxdev_notify_data_read(dmxdevfilter, flush_len);
		}
		buf->error = 0;
	}

	dmx_buffer_status->fullness = dvb_ringbuffer_avail(buf);
	dmx_buffer_status->free_bytes = dvb_ringbuffer_free(buf);
	dmx_buffer_status->read_offset = buf->pread;
	dmx_buffer_status->write_offset = buf->pwrite;
	dmx_buffer_status->size = buf->size;

	spin_unlock_irq(&dmxdevfilter->dev->lock);

	return 0;
}

static int dvb_dmxdev_release_data(struct dmxdev_filter *dmxdevfilter,
					u32 bytes_count)
{
	ssize_t buff_fullness;

	if (!dmxdevfilter->buffer.data)
		return -EINVAL;

	if (!bytes_count)
		return 0;

	buff_fullness = dvb_ringbuffer_avail(&dmxdevfilter->buffer);

	if (bytes_count > buff_fullness)
		return -EINVAL;

	DVB_RINGBUFFER_SKIP(&dmxdevfilter->buffer, bytes_count);

	dvb_dmxdev_notify_data_read(dmxdevfilter, bytes_count);
	spin_lock_irq(&dmxdevfilter->dev->lock);
	dvb_dmxdev_update_events(&dmxdevfilter->events, bytes_count);
	spin_unlock_irq(&dmxdevfilter->dev->lock);

	wake_up_all(&dmxdevfilter->buffer.queue);

	return 0;
}

static int dvb_dmxdev_get_event(struct dmxdev_filter *dmxdevfilter,
					struct dmx_filter_event *event)
{
	int res;
	ssize_t flush_len;
	spin_lock_irq(&dmxdevfilter->dev->lock);

	res = dvb_dmxdev_remove_event(&dmxdevfilter->events, event);
	if (res) {
		spin_unlock_irq(&dmxdevfilter->dev->lock);
		return res;
	}

	if (event->type == DMX_EVENT_BUFFER_OVERFLOW) {
		/*
		 * When buffer overflowed, demux-dev marked the buffer in
		 * error state.
		 * Data from underlying driver is discarded until
		 * user gets notified that buffer has overflowed.
		 * Now that the user is notified, notify underlying
		 * driver that data was flushed from output buffer.
		 */
		flush_len = dvb_ringbuffer_avail(&dmxdevfilter->buffer);
		dvb_ringbuffer_flush(&dmxdevfilter->buffer);
		dvb_dmxdev_notify_data_read(dmxdevfilter, flush_len);
		dmxdevfilter->buffer.error = 0;
	}

	/*
	 * Decoder filters have no data in the data buffer and their
	 * events can be removed now from the queue.
	 */
	if ((dmxdevfilter->type == DMXDEV_TYPE_PES) &&
		(dmxdevfilter->params.pes.output == DMX_OUT_DECODER))
		dmxdevfilter->events.read_index =
			dvb_dmxdev_advance_event_idx(
				dmxdevfilter->events.read_index);

	spin_unlock_irq(&dmxdevfilter->dev->lock);

	/*
	 * in PULL mode, we might be stalling on
	 * event queue, so need to wake-up waiters
	 */
	if (dmxdevfilter->dev->playback_mode == DMX_PB_MODE_PULL)
		wake_up_all(&dmxdevfilter->buffer.queue);

	return res;

}

static void dvb_dmxdev_filter_timeout(unsigned long data)
{
	struct dmxdev_filter *dmxdevfilter = (struct dmxdev_filter *)data;

	dmxdevfilter->buffer.error = -ETIMEDOUT;
	spin_lock_irq(&dmxdevfilter->dev->lock);
	dmxdevfilter->state = DMXDEV_STATE_TIMEDOUT;
	spin_unlock_irq(&dmxdevfilter->dev->lock);
	wake_up_all(&dmxdevfilter->buffer.queue);
}

static void dvb_dmxdev_filter_timer(struct dmxdev_filter *dmxdevfilter)
{
	struct dmx_sct_filter_params *para = &dmxdevfilter->params.sec;

	del_timer(&dmxdevfilter->timer);
	if (para->timeout) {
		dmxdevfilter->timer.function = dvb_dmxdev_filter_timeout;
		dmxdevfilter->timer.data = (unsigned long)dmxdevfilter;
		dmxdevfilter->timer.expires =
		    jiffies + 1 + (HZ / 2 + HZ * para->timeout) / 1000;
		add_timer(&dmxdevfilter->timer);
	}
}

static int dvb_dmxdev_section_callback(const u8 *buffer1, size_t buffer1_len,
				       const u8 *buffer2, size_t buffer2_len,
				       struct dmx_section_filter *filter,
				       enum dmx_success success)
{
	struct dmxdev_filter *dmxdevfilter = filter->priv;
	struct dmx_filter_event event;
	int ret;

	if (dmxdevfilter->buffer.error) {
		wake_up_all(&dmxdevfilter->buffer.queue);
		return 0;
	}
	spin_lock(&dmxdevfilter->dev->lock);
	if (dmxdevfilter->state != DMXDEV_STATE_GO) {
		spin_unlock(&dmxdevfilter->dev->lock);
		return 0;
	}

	if ((buffer1_len + buffer2_len) == 0) {
		if (DMX_CRC_ERROR == success) {
			/* Section was dropped due to CRC error */
			event.type = DMX_EVENT_SECTION_CRC_ERROR;
			dvb_dmxdev_add_event(&dmxdevfilter->events, &event);

			spin_unlock(&dmxdevfilter->dev->lock);
			wake_up_all(&dmxdevfilter->buffer.queue);
		} else {
			spin_unlock(&dmxdevfilter->dev->lock);
		}

		return 0;
	}

	event.params.section.base_offset = dmxdevfilter->buffer.pwrite;
	event.params.section.start_offset = dmxdevfilter->buffer.pwrite;

	del_timer(&dmxdevfilter->timer);
	ret = dvb_dmxdev_buffer_write(&dmxdevfilter->buffer, buffer1,
				      buffer1_len);
	if (ret == buffer1_len)
		ret = dvb_dmxdev_buffer_write(&dmxdevfilter->buffer, buffer2,
					      buffer2_len);

	if (ret < 0) {
		dvb_dmxdev_flush_events(&dmxdevfilter->events);
		dmxdevfilter->buffer.error = ret;

		event.type = DMX_EVENT_BUFFER_OVERFLOW;
	} else {
		event.type = DMX_EVENT_NEW_SECTION;
		event.params.section.total_length =
			buffer1_len + buffer2_len;
		event.params.section.actual_length =
			event.params.section.total_length;

		if (success == DMX_MISSED_ERROR)
			event.params.section.flags =
					DMX_FILTER_CC_ERROR;
		else
			event.params.section.flags = 0;
	}

	dvb_dmxdev_add_event(&dmxdevfilter->events, &event);

	if (dmxdevfilter->params.sec.flags & DMX_ONESHOT)
		dmxdevfilter->state = DMXDEV_STATE_DONE;
	spin_unlock(&dmxdevfilter->dev->lock);
	wake_up_all(&dmxdevfilter->buffer.queue);
	return 0;
}

static int dvb_dmxdev_ts_callback(const u8 *buffer1, size_t buffer1_len,
				  const u8 *buffer2, size_t buffer2_len,
				  struct dmx_ts_feed *feed,
				  enum dmx_success success)
{
	struct dmxdev_filter *dmxdevfilter = feed->priv;
	struct dvb_ringbuffer *buffer;
	struct dmxdev_events_queue *events;
	struct dmx_filter_event event;
	int ret;

	spin_lock(&dmxdevfilter->dev->lock);
	if (dmxdevfilter->params.pes.output == DMX_OUT_DECODER) {
		spin_unlock(&dmxdevfilter->dev->lock);
		return 0;
	}

	if (dmxdevfilter->params.pes.output == DMX_OUT_TAP
	    || dmxdevfilter->params.pes.output == DMX_OUT_TSDEMUX_TAP) {
		buffer = &dmxdevfilter->buffer;
		events = &dmxdevfilter->events;
	} else {
		buffer = &dmxdevfilter->dev->dvr_buffer;
		events = &dmxdevfilter->dev->dvr_output_events;
	}

	if (buffer->error) {
		spin_unlock(&dmxdevfilter->dev->lock);
		wake_up_all(&buffer->queue);
		return 0;
	}

	if (dmxdevfilter->state != DMXDEV_STATE_GO) {
		spin_unlock(&dmxdevfilter->dev->lock);
		return 0;
	}

	if (dmxdevfilter->params.pes.output == DMX_OUT_TAP) {
		if ((success == DMX_OK) &&
			(!events->current_event_data_size)) {
			events->current_event_start_offset = buffer->pwrite;
		} else if (success == DMX_OK_PES_END) {
			event.type = DMX_EVENT_NEW_PES;

			event.params.pes.actual_length =
				events->current_event_data_size;
			event.params.pes.total_length =
				events->current_event_data_size;

			event.params.pes.base_offset =
				events->current_event_start_offset;
			event.params.pes.start_offset =
				events->current_event_start_offset;

			event.params.pes.flags = 0;
			event.params.pes.stc = 0;
			event.params.pes.transport_error_indicator_counter = 0;
			event.params.pes.continuity_error_counter = 0;
			event.params.pes.ts_packets_num = 0;

			dvb_dmxdev_add_event(events, &event);
			events->current_event_data_size = 0;
		}
	} else {
		if (!events->current_event_data_size) {
			events->current_event_start_offset =
				buffer->pwrite;
		}
	}

	if (buffer1_len + buffer2_len) {
		ret = dvb_dmxdev_buffer_write(buffer, buffer1, buffer1_len);
		if (ret == buffer1_len)
			ret = dvb_dmxdev_buffer_write(buffer, buffer2,
								buffer2_len);
		if (ret < 0) {
			/* Enter buffer overflow state */
			dprintk("dmxdev: buffer overflow\n");
			buffer->error = ret;
			dvb_dmxdev_flush_events(events);
			event.type = DMX_EVENT_BUFFER_OVERFLOW;
			dvb_dmxdev_add_event(events, &event);
		} else {
			events->current_event_data_size +=
				(buffer1_len + buffer2_len);

			if (((dmxdevfilter->params.pes.output ==
				DMX_OUT_TS_TAP) ||
				(dmxdevfilter->params.pes.output ==
				DMX_OUT_TSDEMUX_TAP)) &&
				(events->current_event_data_size >=
				dmxdevfilter->params.pes.rec_chunk_size)) {

				event.type = DMX_EVENT_NEW_REC_CHUNK;
				event.params.recording_chunk.offset =
					events->current_event_start_offset;

				event.params.recording_chunk.size =
					events->current_event_data_size;

				dvb_dmxdev_add_event(events, &event);
				events->current_event_data_size = 0;
			}
		}
	}

	spin_unlock(&dmxdevfilter->dev->lock);
	wake_up_all(&buffer->queue);
	return 0;
}

static int dvb_dmxdev_section_event_cb(struct dmx_section_filter *filter,
			struct dmx_data_ready *dmx_data_ready)
{
	int res;
	struct dmxdev_filter *dmxdevfilter = filter->priv;
	struct dmx_filter_event event;
	int free;

	if (dmxdevfilter->buffer.error) {
		wake_up_all(&dmxdevfilter->buffer.queue);
		return 0;
	}

	spin_lock(&dmxdevfilter->dev->lock);

	if (dmxdevfilter->state != DMXDEV_STATE_GO) {
		spin_unlock(&dmxdevfilter->dev->lock);
		return 0;
	}

	if (dmx_data_ready->data_length == 0) {
		if (DMX_CRC_ERROR == dmx_data_ready->status) {
			/* Section was dropped due to CRC error */
			event.type = DMX_EVENT_SECTION_CRC_ERROR;
			dvb_dmxdev_add_event(&dmxdevfilter->events, &event);

			spin_unlock(&dmxdevfilter->dev->lock);
			wake_up_all(&dmxdevfilter->buffer.queue);
		} else {
			spin_unlock(&dmxdevfilter->dev->lock);
		}
		return 0;
	}

	free = dvb_ringbuffer_free(&dmxdevfilter->buffer);

	if ((DMX_OVERRUN_ERROR == dmx_data_ready->status) ||
		(dmx_data_ready->data_length > free)) {

		dprintk("dmxdev: buffer overflow\n");

		dmxdevfilter->buffer.error = -EOVERFLOW;
		dvb_dmxdev_flush_events(&dmxdevfilter->events);
		event.type = DMX_EVENT_BUFFER_OVERFLOW;
		dvb_dmxdev_add_event(&dmxdevfilter->events, &event);
		spin_unlock(&dmxdevfilter->dev->lock);
		wake_up_all(&dmxdevfilter->buffer.queue);
		return 0;
	}

	event.type = DMX_EVENT_NEW_SECTION;
	event.params.section.base_offset = dmxdevfilter->buffer.pwrite;
	event.params.section.start_offset = dmxdevfilter->buffer.pwrite;
	event.params.section.total_length = dmx_data_ready->data_length;
	event.params.section.actual_length = dmx_data_ready->data_length;

	if (dmx_data_ready->status == DMX_MISSED_ERROR)
		event.params.section.flags = DMX_FILTER_CC_ERROR;
	else
		event.params.section.flags = 0;

	res = dvb_dmxdev_add_event(&dmxdevfilter->events, &event);
	DVB_RINGBUFFER_PUSH(&dmxdevfilter->buffer, dmx_data_ready->data_length);

	spin_unlock(&dmxdevfilter->dev->lock);
	wake_up_all(&dmxdevfilter->buffer.queue);

	return res;
}

static int dvb_dmxdev_ts_event_cb(struct dmx_ts_feed *feed,
			struct dmx_data_ready *dmx_data_ready)
{
	struct dmxdev_filter *dmxdevfilter = feed->priv;
	struct dvb_ringbuffer *buffer;
	struct dmxdev_events_queue *events;
	struct dmx_filter_event event;
	int free;

	spin_lock(&dmxdevfilter->dev->lock);

	if (dmxdevfilter->state != DMXDEV_STATE_GO) {
		spin_unlock(&dmxdevfilter->dev->lock);
		return 0;
	}

	if (dmxdevfilter->params.pes.output != DMX_OUT_TS_TAP) {
		buffer = &dmxdevfilter->buffer;
		events = &dmxdevfilter->events;
	} else {
		buffer = &dmxdevfilter->dev->dvr_buffer;
		events = &dmxdevfilter->dev->dvr_output_events;
	}

	if (dmx_data_ready->status == DMX_OK_PCR) {
		dprintk("dmxdev: event callback DMX_OK_PCR\n");
		event.type = DMX_EVENT_NEW_PCR;
		event.params.pcr.pcr = dmx_data_ready->pcr.pcr;
		event.params.pcr.stc = dmx_data_ready->pcr.stc;
		if (dmx_data_ready->pcr.disc_indicator_set)
			event.params.pcr.flags =
				DMX_FILTER_DISCONTINUITY_INDICATOR;
		else
			event.params.pcr.flags = 0;

		dvb_dmxdev_add_event(events, &event);
		spin_unlock(&dmxdevfilter->dev->lock);
		wake_up_all(&buffer->queue);
		return 0;
	}

	if (dmx_data_ready->status == DMX_OK_DECODER_BUF) {
		event.type = DMX_EVENT_NEW_ES_DATA;
		event.params.es_data.buf_handle = dmx_data_ready->buf.handle;
		event.params.es_data.cookie = dmx_data_ready->buf.cookie;
		event.params.es_data.offset = dmx_data_ready->buf.offset;
		event.params.es_data.data_len = dmx_data_ready->buf.len;
		event.params.es_data.pts_valid = dmx_data_ready->buf.pts_exists;
		event.params.es_data.pts = dmx_data_ready->buf.pts;
		event.params.es_data.dts_valid = dmx_data_ready->buf.dts_exists;
		event.params.es_data.dts = dmx_data_ready->buf.dts;
		event.params.es_data.transport_error_indicator_counter =
				dmx_data_ready->buf.tei_counter;
		event.params.es_data.continuity_error_counter =
				dmx_data_ready->buf.cont_err_counter;
		event.params.es_data.ts_packets_num =
				dmx_data_ready->buf.ts_packets_num;
		event.params.es_data.ts_dropped_bytes =
				dmx_data_ready->buf.ts_dropped_bytes;
		dvb_dmxdev_add_event(events, &event);
		spin_unlock(&dmxdevfilter->dev->lock);
		wake_up_all(&buffer->queue);
		return 0;
	}

	if (dmxdevfilter->params.pes.output == DMX_OUT_DECODER) {
		if (DMX_OVERRUN_ERROR == dmx_data_ready->status) {
			dprintk("dmxdev: buffer overflow\n");
			event.type = DMX_EVENT_BUFFER_OVERFLOW;
			dvb_dmxdev_add_event(&dmxdevfilter->events, &event);
		}
		spin_unlock(&dmxdevfilter->dev->lock);
		wake_up_all(&buffer->queue);
		return 0;
	}

	if (buffer->error) {
		spin_unlock(&dmxdevfilter->dev->lock);
		wake_up_all(&buffer->queue);
		return 0;
	}

	free = dvb_ringbuffer_free(&dmxdevfilter->buffer);

	if ((DMX_OVERRUN_ERROR == dmx_data_ready->status) ||
		(dmx_data_ready->data_length > free)) {

		/*
		 * Enter buffer overflow state:
		 * Set buffer overflow error state, flush all pending demux
		 * device events to ensure user can receive the overflow event
		 * and report the event to user
		 */
		dprintk("dmxdev: buffer overflow\n");

		buffer->error = -EOVERFLOW;
		dvb_dmxdev_flush_events(events);
		event.type = DMX_EVENT_BUFFER_OVERFLOW;
		dvb_dmxdev_add_event(&dmxdevfilter->events, &event);

		spin_unlock(&dmxdevfilter->dev->lock);
		return 0;
	}

	if (dmxdevfilter->params.pes.output == DMX_OUT_TAP) {
		if ((dmx_data_ready->status == DMX_OK) &&
			(!events->current_event_data_size)) {
			events->current_event_start_offset =
				dmxdevfilter->buffer.pwrite;
		} else if (dmx_data_ready->status == DMX_OK_PES_END) {
			event.type = DMX_EVENT_NEW_PES;

			event.params.pes.base_offset =
				events->current_event_start_offset;
			event.params.pes.start_offset =
				events->current_event_start_offset +
				dmx_data_ready->pes_end.start_gap;

			event.params.pes.actual_length =
				dmx_data_ready->pes_end.actual_length;
			event.params.pes.total_length =
				events->current_event_data_size;

			event.params.pes.flags = 0;
			if (dmx_data_ready->pes_end.disc_indicator_set)
				event.params.pes.flags |=
					DMX_FILTER_DISCONTINUITY_INDICATOR;
			if (dmx_data_ready->pes_end.pes_length_mismatch)
				event.params.pes.flags |=
					DMX_FILTER_PES_LENGTH_ERROR;

			event.params.pes.stc = dmx_data_ready->pes_end.stc;
			event.params.pes.transport_error_indicator_counter =
				dmx_data_ready->pes_end.tei_counter;
			event.params.pes.continuity_error_counter =
				dmx_data_ready->pes_end.cont_err_counter;
			event.params.pes.ts_packets_num =
				dmx_data_ready->pes_end.ts_packets_num;

			dvb_dmxdev_add_event(events, &event);

			events->current_event_data_size = 0;
		}
	} else {
		if (!events->current_event_data_size)
			events->current_event_start_offset =
					dmxdevfilter->buffer.pwrite;
	}

	events->current_event_data_size += dmx_data_ready->data_length;
	DVB_RINGBUFFER_PUSH(&dmxdevfilter->buffer, dmx_data_ready->data_length);

	if ((dmxdevfilter->params.pes.output == DMX_OUT_TS_TAP) ||
		(dmxdevfilter->params.pes.output == DMX_OUT_TSDEMUX_TAP)) {
		if (events->current_event_data_size >=
			dmxdevfilter->params.pes.rec_chunk_size) {
			event.type = DMX_EVENT_NEW_REC_CHUNK;
			event.params.recording_chunk.offset =
				events->current_event_start_offset;

			event.params.recording_chunk.size =
				events->current_event_data_size;

			dvb_dmxdev_add_event(events, &event);

			events->current_event_data_size = 0;
		 }
	}
	spin_unlock(&dmxdevfilter->dev->lock);
	wake_up_all(&buffer->queue);
	return 0;
}

/* stop feed but only mark the specified filter as stopped (state set) */
static int dvb_dmxdev_feed_stop(struct dmxdev_filter *dmxdevfilter)
{
	struct dmxdev_feed *feed;

	dvb_dmxdev_filter_state_set(dmxdevfilter, DMXDEV_STATE_SET);

	switch (dmxdevfilter->type) {
	case DMXDEV_TYPE_SEC:
		del_timer(&dmxdevfilter->timer);
		dmxdevfilter->feed.sec.feed->stop_filtering(
			dmxdevfilter->feed.sec.feed);
		break;
	case DMXDEV_TYPE_PES:
		list_for_each_entry(feed, &dmxdevfilter->feed.ts, next) {
			if (dmxdevfilter->params.pes.output == DMX_OUT_TS_TAP) {
				dmxdevfilter->dev->dvr_feeds_count--;
				if (!dmxdevfilter->dev->dvr_feeds_count)
					dmxdevfilter->dev->dvr_feed = NULL;
			}
			feed->ts->stop_filtering(feed->ts);
		}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/* start feed associated with the specified filter */
static int dvb_dmxdev_feed_start(struct dmxdev_filter *filter)
{
	struct dmxdev_feed *feed;
	int ret;

	dvb_dmxdev_filter_state_set(filter, DMXDEV_STATE_GO);

	switch (filter->type) {
	case DMXDEV_TYPE_SEC:
		return filter->feed.sec.feed->start_filtering(
			filter->feed.sec.feed);
	case DMXDEV_TYPE_PES:
		list_for_each_entry(feed, &filter->feed.ts, next) {
			ret = feed->ts->start_filtering(feed->ts);
			if (ret < 0) {
				dvb_dmxdev_feed_stop(filter);
				return ret;
			}
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* restart section feed if it has filters left associated with it,
   otherwise release the feed */
static int dvb_dmxdev_feed_restart(struct dmxdev_filter *filter)
{
	int i;
	struct dmxdev *dmxdev = filter->dev;
	u16 pid = filter->params.sec.pid;

	for (i = 0; i < dmxdev->filternum; i++)
		if (dmxdev->filter[i].state >= DMXDEV_STATE_GO &&
		    dmxdev->filter[i].type == DMXDEV_TYPE_SEC &&
		    dmxdev->filter[i].params.sec.pid == pid) {
			dvb_dmxdev_feed_start(&dmxdev->filter[i]);
			return 0;
		}

	filter->dev->demux->release_section_feed(dmxdev->demux,
						 filter->feed.sec.feed);

	return 0;
}

static int dvb_dmxdev_filter_stop(struct dmxdev_filter *dmxdevfilter)
{
	struct dmxdev_feed *feed;
	struct dmx_demux *demux;

	if (dmxdevfilter->state < DMXDEV_STATE_GO)
		return 0;

	switch (dmxdevfilter->type) {
	case DMXDEV_TYPE_SEC:
		if (!dmxdevfilter->feed.sec.feed)
			break;
		dvb_dmxdev_feed_stop(dmxdevfilter);
		if (dmxdevfilter->filter.sec)
			dmxdevfilter->feed.sec.feed->
			    release_filter(dmxdevfilter->feed.sec.feed,
					   dmxdevfilter->filter.sec);
		dvb_dmxdev_feed_restart(dmxdevfilter);
		dmxdevfilter->feed.sec.feed = NULL;
		break;
	case DMXDEV_TYPE_PES:
		dvb_dmxdev_feed_stop(dmxdevfilter);
		demux = dmxdevfilter->dev->demux;
		list_for_each_entry(feed, &dmxdevfilter->feed.ts, next) {
			demux->release_ts_feed(demux, feed->ts);
			feed->ts = NULL;
		}
		break;
	default:
		if (dmxdevfilter->state == DMXDEV_STATE_ALLOCATED)
			return 0;
		return -EINVAL;
	}

	spin_lock_irq(&dmxdevfilter->dev->lock);
	dvb_dmxdev_flush_output(&dmxdevfilter->buffer, &dmxdevfilter->events);
	dvb_ringbuffer_reset(&dmxdevfilter->buffer);
	spin_unlock_irq(&dmxdevfilter->dev->lock);

	wake_up_all(&dmxdevfilter->buffer.queue);

	return 0;
}

static void dvb_dmxdev_delete_pids(struct dmxdev_filter *dmxdevfilter)
{
	struct dmxdev_feed *feed, *tmp;

	/* delete all PIDs */
	list_for_each_entry_safe(feed, tmp, &dmxdevfilter->feed.ts, next) {
		list_del(&feed->next);
		kfree(feed);
	}

	BUG_ON(!list_empty(&dmxdevfilter->feed.ts));
}

static inline int dvb_dmxdev_filter_reset(struct dmxdev_filter *dmxdevfilter)
{
	if (dmxdevfilter->state < DMXDEV_STATE_SET)
		return 0;

	if (dmxdevfilter->type == DMXDEV_TYPE_PES)
		dvb_dmxdev_delete_pids(dmxdevfilter);

	dmxdevfilter->type = DMXDEV_TYPE_NONE;
	dvb_dmxdev_filter_state_set(dmxdevfilter, DMXDEV_STATE_ALLOCATED);
	return 0;
}

static int dvb_dmxdev_start_feed(struct dmxdev *dmxdev,
				 struct dmxdev_filter *filter,
				 struct dmxdev_feed *feed)
{
	struct timespec timeout = { 0 };
	struct dmx_pes_filter_params *para = &filter->params.pes;
	dmx_output_t otype;
	int ret;
	int ts_type;
	dmx_pes_type_t ts_pes;
	struct dmx_ts_feed *tsfeed;

	feed->ts = NULL;
	otype = para->output;

	ts_pes = para->pes_type;

	if (ts_pes < DMX_PES_OTHER)
		ts_type = TS_DECODER;
	else
		ts_type = 0;

	if (otype == DMX_OUT_TS_TAP)
		ts_type |= TS_PACKET;
	else if (otype == DMX_OUT_TSDEMUX_TAP)
		ts_type |= TS_PACKET | TS_DEMUX;
	else if (otype == DMX_OUT_TAP)
		ts_type |= TS_PACKET | TS_DEMUX | TS_PAYLOAD_ONLY;

	ret = dmxdev->demux->allocate_ts_feed(dmxdev->demux, &feed->ts,
					      dvb_dmxdev_ts_callback);
	if (ret < 0)
		return ret;

	tsfeed = feed->ts;
	tsfeed->priv = filter;

	if (filter->params.pes.output == DMX_OUT_TS_TAP) {
		tsfeed->buffer.ringbuff = &dmxdev->dvr_buffer;
		tsfeed->buffer.priv_handle = dmxdev->dvr_priv_buff_handle;
		if (!dmxdev->dvr_feeds_count)
			dmxdev->dvr_feed = filter;
		dmxdev->dvr_feeds_count++;
	} else if (filter->params.pes.output == DMX_OUT_DECODER) {
		tsfeed->buffer.ringbuff = &filter->buffer;
		tsfeed->decoder_buffers = &filter->decoder_buffers;
		tsfeed->buffer.priv_handle = filter->priv_buff_handle;
	} else {
		tsfeed->buffer.ringbuff = &filter->buffer;
		tsfeed->buffer.priv_handle = filter->priv_buff_handle;
	}

	if (tsfeed->data_ready_cb) {
		ret = tsfeed->data_ready_cb(tsfeed, dvb_dmxdev_ts_event_cb);

		if (ret < 0) {
			dmxdev->demux->release_ts_feed(dmxdev->demux, tsfeed);
			return ret;
		}
	}

	ret = tsfeed->set(tsfeed, feed->pid,
					ts_type, ts_pes,
					filter->decoder_buffers.buffers_size,
					timeout);
	if (ret < 0) {
		dmxdev->demux->release_ts_feed(dmxdev->demux, tsfeed);
		return ret;
	}

	if (tsfeed->set_tsp_out_format)
		tsfeed->set_tsp_out_format(tsfeed, filter->dmx_tsp_format);

	if (tsfeed->set_secure_mode)
		tsfeed->set_secure_mode(tsfeed, &feed->sec_mode);

	/* Support indexing for video PES */
	if ((para->pes_type == DMX_PES_VIDEO0) ||
	    (para->pes_type == DMX_PES_VIDEO1) ||
	    (para->pes_type == DMX_PES_VIDEO2) ||
	    (para->pes_type == DMX_PES_VIDEO3)) {

		if (tsfeed->set_indexing_params) {
			ret = tsfeed->set_indexing_params(tsfeed,
							&para->video_params);

			if (ret < 0) {
				dmxdev->demux->release_ts_feed(dmxdev->demux,
								tsfeed);
				return ret;
			}
		}
	}

	ret = tsfeed->start_filtering(tsfeed);
	if (ret < 0) {
		dmxdev->demux->release_ts_feed(dmxdev->demux, tsfeed);
		return ret;
	}

	return 0;
}

static int dvb_dmxdev_filter_start(struct dmxdev_filter *filter)
{
	struct dmxdev *dmxdev = filter->dev;
	struct dmxdev_feed *feed;
	void *mem;
	int ret, i;

	if (filter->state < DMXDEV_STATE_SET)
		return -EINVAL;

	if (filter->state >= DMXDEV_STATE_GO)
		dvb_dmxdev_filter_stop(filter);

	if (!filter->buffer.data) {
		if ((filter->buffer_mode == DMX_BUFFER_MODE_EXTERNAL) ||
			(dmxdev->capabilities & DMXDEV_CAP_EXTERNAL_BUFFS_ONLY))
			return -ENOMEM;
		mem = vmalloc_user(filter->buffer.size);
		if (!mem)
			return -ENOMEM;
		spin_lock_irq(&filter->dev->lock);
		filter->buffer.data = mem;
		spin_unlock_irq(&filter->dev->lock);
	}

	spin_lock_irq(&filter->dev->lock);
	dvb_dmxdev_flush_output(&filter->buffer, &filter->events);
	spin_unlock_irq(&filter->dev->lock);

	switch (filter->type) {
	case DMXDEV_TYPE_SEC:
	{
		struct dmx_sct_filter_params *para = &filter->params.sec;
		struct dmx_section_filter **secfilter = &filter->filter.sec;
		struct dmx_section_feed **secfeed = &filter->feed.sec.feed;

		*secfilter = NULL;
		*secfeed = NULL;

		/* find active filter/feed with same PID */
		for (i = 0; i < dmxdev->filternum; i++) {
			if (dmxdev->filter[i].state >= DMXDEV_STATE_GO &&
			    dmxdev->filter[i].type == DMXDEV_TYPE_SEC &&
			    dmxdev->filter[i].params.sec.pid == para->pid) {
				*secfeed = dmxdev->filter[i].feed.sec.feed;
				break;
			}
		}

		/* if no feed found, try to allocate new one */
		if (!*secfeed) {
			ret = dmxdev->demux->allocate_section_feed(dmxdev->demux,
						secfeed,
						dvb_dmxdev_section_callback);
			if (ret < 0) {
				printk("DVB (%s): could not alloc feed\n",
				       __func__);
				return ret;
			}

			if ((*secfeed)->data_ready_cb) {
				ret = (*secfeed)->data_ready_cb(
						*secfeed,
						dvb_dmxdev_section_event_cb);

				if (ret < 0) {
					printk(KERN_ERR "DVB (%s): could not set event cb\n",
				       __func__);
					dvb_dmxdev_feed_restart(filter);
					return ret;
				}
			}

			ret = (*secfeed)->set(*secfeed, para->pid, 32768,
					      (para->flags & DMX_CHECK_CRC) ? 1 : 0);
			if (ret < 0) {
				printk("DVB (%s): could not set feed\n",
				       __func__);
				dvb_dmxdev_feed_restart(filter);
				return ret;
			}

			if ((*secfeed)->set_secure_mode)
				(*secfeed)->set_secure_mode(*secfeed,
					&filter->feed.sec.sec_mode);
		} else {
			dvb_dmxdev_feed_stop(filter);
		}

		ret = (*secfeed)->allocate_filter(*secfeed, secfilter);
		if (ret < 0) {
			dvb_dmxdev_feed_restart(filter);
			filter->feed.sec.feed->start_filtering(*secfeed);
			dprintk("could not get filter\n");
			return ret;
		}

		(*secfilter)->priv = filter;
		(*secfilter)->buffer.ringbuff = &filter->buffer;
		(*secfilter)->buffer.priv_handle = filter->priv_buff_handle;

		memcpy(&((*secfilter)->filter_value[3]),
		       &(para->filter.filter[1]), DMX_FILTER_SIZE - 1);
		memcpy(&(*secfilter)->filter_mask[3],
		       &para->filter.mask[1], DMX_FILTER_SIZE - 1);
		memcpy(&(*secfilter)->filter_mode[3],
		       &para->filter.mode[1], DMX_FILTER_SIZE - 1);

		(*secfilter)->filter_value[0] = para->filter.filter[0];
		(*secfilter)->filter_mask[0] = para->filter.mask[0];
		(*secfilter)->filter_mode[0] = para->filter.mode[0];
		(*secfilter)->filter_mask[1] = 0;
		(*secfilter)->filter_mask[2] = 0;

		filter->todo = 0;

		ret = filter->feed.sec.feed->start_filtering(
				filter->feed.sec.feed);
		if (ret < 0)
			return ret;

		dvb_dmxdev_filter_timer(filter);
		break;
	}
	case DMXDEV_TYPE_PES:
		if (filter->params.pes.rec_chunk_size <
			DMX_REC_BUFF_CHUNK_MIN_SIZE)
			filter->params.pes.rec_chunk_size =
				DMX_REC_BUFF_CHUNK_MIN_SIZE;

		if (filter->params.pes.rec_chunk_size >=
			filter->buffer.size)
			filter->params.pes.rec_chunk_size =
				filter->buffer.size >> 2;

		ret = 0;
		list_for_each_entry(feed, &filter->feed.ts, next) {
			ret = dvb_dmxdev_start_feed(dmxdev, filter, feed);
			if (ret)
				break;
		}

		if (!ret)
			break;

		/* cleanup feeds that were started before the failure */
		list_for_each_entry(feed, &filter->feed.ts, next) {
			if (!feed->ts)
				continue;
			feed->ts->stop_filtering(feed->ts);
			dmxdev->demux->release_ts_feed(dmxdev->demux, feed->ts);
			feed->ts = NULL;

			if (filter->params.pes.output == DMX_OUT_TS_TAP) {
				filter->dev->dvr_feeds_count--;
				if (!filter->dev->dvr_feeds_count)
					filter->dev->dvr_feed = NULL;
			}
		}
		return ret;

	default:
		return -EINVAL;
	}

	dvb_dmxdev_filter_state_set(filter, DMXDEV_STATE_GO);
	return 0;
}

static int dvb_demux_open(struct inode *inode, struct file *file)
{
	struct dvb_device *dvbdev = file->private_data;
	struct dmxdev *dmxdev = dvbdev->priv;
	int i;
	struct dmxdev_filter *dmxdevfilter;

	if (!dmxdev->filter)
		return -EINVAL;

	if (mutex_lock_interruptible(&dmxdev->mutex))
		return -ERESTARTSYS;

	for (i = 0; i < dmxdev->filternum; i++)
		if (dmxdev->filter[i].state == DMXDEV_STATE_FREE)
			break;

	if (i == dmxdev->filternum) {
		mutex_unlock(&dmxdev->mutex);
		return -EMFILE;
	}

	dmxdevfilter = &dmxdev->filter[i];
	mutex_init(&dmxdevfilter->mutex);
	file->private_data = dmxdevfilter;

	memset(&dmxdevfilter->decoder_buffers,
			0,
			sizeof(dmxdevfilter->decoder_buffers));
	dmxdevfilter->decoder_buffers.buffers_size =
		DMX_DEFAULT_DECODER_BUFFER_SIZE;
	dmxdevfilter->buffer_mode = DMX_BUFFER_MODE_INTERNAL;
	dmxdevfilter->priv_buff_handle = NULL;
	dvb_ringbuffer_init(&dmxdevfilter->buffer, NULL, 8192);
	dvb_dmxdev_flush_events(&dmxdevfilter->events);

	dmxdevfilter->type = DMXDEV_TYPE_NONE;
	dvb_dmxdev_filter_state_set(dmxdevfilter, DMXDEV_STATE_ALLOCATED);
	init_timer(&dmxdevfilter->timer);

	dmxdevfilter->dmx_tsp_format = DMX_TSP_FORMAT_188;
	dvbdev->users++;

	mutex_unlock(&dmxdev->mutex);
	return 0;
}

static int dvb_dmxdev_filter_free(struct dmxdev *dmxdev,
				  struct dmxdev_filter *dmxdevfilter)
{
	mutex_lock(&dmxdev->mutex);
	mutex_lock(&dmxdevfilter->mutex);

	dvb_dmxdev_filter_stop(dmxdevfilter);

	dvb_dmxdev_filter_reset(dmxdevfilter);

	if (dmxdevfilter->buffer.data) {
		void *mem = dmxdevfilter->buffer.data;

		spin_lock_irq(&dmxdev->lock);
		dmxdevfilter->buffer.data = NULL;
		spin_unlock_irq(&dmxdev->lock);
		if (dmxdevfilter->buffer_mode == DMX_BUFFER_MODE_INTERNAL)
			vfree(mem);
	}

	if ((dmxdevfilter->buffer_mode == DMX_BUFFER_MODE_EXTERNAL) &&
		dmxdevfilter->priv_buff_handle) {
		dmxdev->demux->unmap_buffer(dmxdev->demux,
			dmxdevfilter->priv_buff_handle);
		dmxdevfilter->priv_buff_handle = NULL;
	}

	dvb_dmxdev_filter_state_set(dmxdevfilter, DMXDEV_STATE_FREE);
	wake_up_all(&dmxdevfilter->buffer.queue);
	mutex_unlock(&dmxdevfilter->mutex);
	mutex_unlock(&dmxdev->mutex);
	return 0;
}

static inline void invert_mode(dmx_filter_t *filter)
{
	int i;

	for (i = 0; i < DMX_FILTER_SIZE; i++)
		filter->mode[i] ^= 0xff;
}

static int dvb_dmxdev_add_pid(struct dmxdev *dmxdev,
			      struct dmxdev_filter *filter, u16 pid)
{
	struct dmxdev_feed *feed;

	if ((filter->type != DMXDEV_TYPE_PES) ||
	    (filter->state < DMXDEV_STATE_SET))
		return -EINVAL;

	/* only TS packet filters may have multiple PIDs */
	if ((filter->params.pes.output != DMX_OUT_TSDEMUX_TAP) &&
	    (!list_empty(&filter->feed.ts)))
		return -EINVAL;

	feed = kzalloc(sizeof(struct dmxdev_feed), GFP_KERNEL);
	if (feed == NULL)
		return -ENOMEM;

	feed->pid = pid;
	feed->sec_mode.is_secured = 0;
	list_add(&feed->next, &filter->feed.ts);

	if (filter->state >= DMXDEV_STATE_GO)
		return dvb_dmxdev_start_feed(dmxdev, filter, feed);

	return 0;
}

static int dvb_dmxdev_remove_pid(struct dmxdev *dmxdev,
				  struct dmxdev_filter *filter, u16 pid)
{
	int feed_count;
	struct dmxdev_feed *feed, *tmp;

	if ((filter->type != DMXDEV_TYPE_PES) ||
	    (filter->state < DMXDEV_STATE_SET))
		return -EINVAL;

	feed_count = 0;
	list_for_each_entry(tmp, &filter->feed.ts, next)
		feed_count++;

	if (feed_count <= 1)
		return -EINVAL;

	list_for_each_entry_safe(feed, tmp, &filter->feed.ts, next) {
		if (feed->pid == pid) {
			if (feed->ts != NULL) {
				feed->ts->stop_filtering(feed->ts);
				filter->dev->demux->release_ts_feed(
							filter->dev->demux,
							feed->ts);
			}
			list_del(&feed->next);
			kfree(feed);
		}
	}

	return 0;
}

static int dvb_dmxdev_filter_set(struct dmxdev *dmxdev,
				 struct dmxdev_filter *dmxdevfilter,
				 struct dmx_sct_filter_params *params)
{
	dprintk("function : %s\n", __func__);

	dvb_dmxdev_filter_stop(dmxdevfilter);

	dmxdevfilter->type = DMXDEV_TYPE_SEC;
	memcpy(&dmxdevfilter->params.sec,
	       params, sizeof(struct dmx_sct_filter_params));
	invert_mode(&dmxdevfilter->params.sec.filter);
	dmxdevfilter->feed.sec.sec_mode.is_secured = 0;
	dvb_dmxdev_filter_state_set(dmxdevfilter, DMXDEV_STATE_SET);

	if (params->flags & DMX_IMMEDIATE_START)
		return dvb_dmxdev_filter_start(dmxdevfilter);

	return 0;
}

static int dvb_dmxdev_set_secure_mode(
	struct dmxdev *dmxdev,
	struct dmxdev_filter *filter,
	struct dmx_secure_mode *sec_mode)
{
	struct dmxdev_feed *feed;
	struct dmxdev_feed *ts_feed = NULL;
	struct dmxdev_sec_feed *sec_feed = NULL;

	if (NULL == dmxdev || NULL == filter || NULL == sec_mode)
		return -EINVAL;

	if (filter->state < DMXDEV_STATE_SET ||
		filter->state > DMXDEV_STATE_GO) {
		printk(KERN_ERR "%s: invalid filter state\n", __func__);
		return -EPERM;
	}
	dprintk(KERN_DEBUG "%s: key_id=%d, secure=%d, looking for pid=%d\n",
		__func__, sec_mode->key_ladder_id, sec_mode->is_secured,
		sec_mode->pid);
	switch (filter->type) {
	case DMXDEV_TYPE_PES:
		list_for_each_entry(feed, &filter->feed.ts, next) {
			if (feed->pid == sec_mode->pid) {
				ts_feed = feed;
				ts_feed->sec_mode = *sec_mode;
				if (filter->state == DMXDEV_STATE_GO &&
					ts_feed->ts->set_secure_mode)
					ts_feed->ts->set_secure_mode(
						ts_feed->ts, sec_mode);
				break;
			}
		}
		break;
	case DMXDEV_TYPE_SEC:
		if (filter->params.sec.pid == sec_mode->pid) {
			sec_feed = &filter->feed.sec;
			sec_feed->sec_mode = *sec_mode;
			if (filter->state == DMXDEV_STATE_GO &&
				sec_feed->feed->set_secure_mode)
				sec_feed->feed->set_secure_mode(sec_feed->feed,
						sec_mode);
		}
		break;

	default:
		return -EINVAL;
	}

	if (!ts_feed && !sec_feed) {
		printk(KERN_ERR "%s: pid %d is undefined for this filter\n",
			__func__, sec_mode->pid);
		return -EINVAL;
	}

	return 0;
}

static int dvb_dmxdev_pes_filter_set(struct dmxdev *dmxdev,
				     struct dmxdev_filter *dmxdevfilter,
				     struct dmx_pes_filter_params *params)
{
	int ret;

	dvb_dmxdev_filter_stop(dmxdevfilter);
	dvb_dmxdev_filter_reset(dmxdevfilter);

	if (params->pes_type > DMX_PES_OTHER || params->pes_type < 0)
		return -EINVAL;

	if (params->flags & DMX_ENABLE_INDEXING) {
		if (!(dmxdev->capabilities & DMXDEV_CAP_INDEXING))
			return -EINVAL;

		/* can do indexing only on video PES */
		if ((params->pes_type != DMX_PES_VIDEO0) &&
		    (params->pes_type != DMX_PES_VIDEO1) &&
		    (params->pes_type != DMX_PES_VIDEO2) &&
		    (params->pes_type != DMX_PES_VIDEO3))
			return -EINVAL;

		/* can do indexing only when recording */
		if ((params->output != DMX_OUT_TS_TAP) &&
		    (params->output != DMX_OUT_TSDEMUX_TAP))
			return -EINVAL;
	}

	dmxdevfilter->type = DMXDEV_TYPE_PES;
	memcpy(&dmxdevfilter->params, params,
	       sizeof(struct dmx_pes_filter_params));
	INIT_LIST_HEAD(&dmxdevfilter->feed.ts);

	dvb_dmxdev_filter_state_set(dmxdevfilter, DMXDEV_STATE_SET);

	ret = dvb_dmxdev_add_pid(dmxdev, dmxdevfilter,
				 dmxdevfilter->params.pes.pid);
	if (ret < 0)
		return ret;

	if (params->flags & DMX_IMMEDIATE_START)
		return dvb_dmxdev_filter_start(dmxdevfilter);

	return 0;
}

static int dvb_dmxdev_set_decoder_buffer(struct dmxdev *dmxdev,
		struct dmxdev_filter *filter,
		struct dmx_decoder_buffers *buffs)
{
	int i;
	struct dmx_decoder_buffers *dec_buffs;
	struct dmx_caps caps;

	if (NULL == dmxdev || NULL == filter || NULL == buffs)
		return -EINVAL;

	dec_buffs = &filter->decoder_buffers;
	dmxdev->demux->get_caps(dmxdev->demux, &caps);

	if ((buffs->buffers_size == 0) ||
		(buffs->is_linear &&
		 ((buffs->buffers_num <= 1) ||
		  (buffs->buffers_num > DMX_MAX_DECODER_BUFFER_NUM))))
		return -EINVAL;

	if (0 == buffs->buffers_num) {
		/* Internal mode - linear buffers not supported in this mode */
		if (!(caps.decoder.flags & DMX_BUFFER_INTERNAL_SUPPORT) ||
			buffs->is_linear)
			return -EINVAL;
	} else {
		/* External buffer(s) mode */
		if ((!(caps.decoder.flags & DMX_BUFFER_LINEAR_GROUP_SUPPORT) &&
			buffs->buffers_num > 1) ||
			!(caps.decoder.flags & DMX_BUFFER_EXTERNAL_SUPPORT) ||
			buffs->buffers_num > caps.decoder.max_buffer_num)
			return -EINVAL;

		dec_buffs->is_linear = buffs->is_linear;
		dec_buffs->buffers_num = buffs->buffers_num;
		dec_buffs->buffers_size = buffs->buffers_size;
		for (i = 0; i < dec_buffs->buffers_num; i++)
			dec_buffs->handles[i] = buffs->handles[i];
	}

	return 0;
}

static ssize_t dvb_dmxdev_read_sec(struct dmxdev_filter *dfil,
				   struct file *file, char __user *buf,
				   size_t count, loff_t *ppos)
{
	int result, hcount;
	int done = 0;

	if (dfil->todo <= 0) {
		hcount = 3 + dfil->todo;
		if (hcount > count)
			hcount = count;
		result = dvb_dmxdev_buffer_read(&dfil->buffer,
						file->f_flags & O_NONBLOCK,
						buf, hcount, ppos);
		if (result < 0) {
			dfil->todo = 0;
			return result;
		}
		if (copy_from_user(dfil->secheader - dfil->todo, buf, result))
			return -EFAULT;
		buf += result;
		done = result;
		count -= result;
		dfil->todo -= result;
		if (dfil->todo > -3)
			return done;
		dfil->todo = ((dfil->secheader[1] << 8) | dfil->secheader[2]) & 0xfff;
		if (!count)
			return done;
	}
	if (count > dfil->todo)
		count = dfil->todo;
	result = dvb_dmxdev_buffer_read(&dfil->buffer,
					file->f_flags & O_NONBLOCK,
					buf, count, ppos);
	if (result < 0)
		return result;
	dfil->todo -= result;
	return (result + done);
}

static ssize_t
dvb_demux_read(struct file *file, char __user *buf, size_t count,
	       loff_t *ppos)
{
	struct dmxdev_filter *dmxdevfilter = file->private_data;
	int ret;
	ssize_t flush_len;

	if (mutex_lock_interruptible(&dmxdevfilter->mutex))
		return -ERESTARTSYS;

	if (dmxdevfilter->type == DMXDEV_TYPE_SEC)
		ret = dvb_dmxdev_read_sec(dmxdevfilter, file, buf, count, ppos);
	else
		ret = dvb_dmxdev_buffer_read(&dmxdevfilter->buffer,
					    file->f_flags & O_NONBLOCK,
					    buf, count, ppos);

	if (ret > 0) {
		dvb_dmxdev_notify_data_read(dmxdevfilter, ret);
		spin_lock_irq(&dmxdevfilter->dev->lock);
		dvb_dmxdev_update_events(&dmxdevfilter->events, ret);
		spin_unlock_irq(&dmxdevfilter->dev->lock);

		/*
		 * in PULL mode, we might be stalling on
		 * event queue, so need to wake-up waiters
		 */
		if (dmxdevfilter->dev->playback_mode == DMX_PB_MODE_PULL)
			wake_up_all(&dmxdevfilter->buffer.queue);
	} else if (ret == -EOVERFLOW) {
		/*
		 * When buffer overflowed, demux-dev marked the buffer in
		 * error state.
		 * Data from underlying driver is discarded until
		 * user gets notified that buffer has overflowed.
		 * Now that the user is notified, notify underlying
		 * driver that data was flushed from output buffer.
		 */
		flush_len = dvb_ringbuffer_avail(&dmxdevfilter->buffer);
		dvb_ringbuffer_flush(&dmxdevfilter->buffer);
		dvb_dmxdev_notify_data_read(dmxdevfilter->dev->dvr_feed,
			flush_len);
	}

	mutex_unlock(&dmxdevfilter->mutex);
	return ret;
}

static int dvb_demux_do_ioctl(struct file *file,
			      unsigned int cmd, void *parg)
{
	struct dmxdev_filter *dmxdevfilter = file->private_data;
	struct dmxdev *dmxdev = dmxdevfilter->dev;
	unsigned long arg = (unsigned long)parg;
	int ret = 0;

	if (mutex_lock_interruptible(&dmxdev->mutex))
		return -ERESTARTSYS;

	switch (cmd) {
	case DMX_START:
		if (mutex_lock_interruptible(&dmxdevfilter->mutex)) {
			mutex_unlock(&dmxdev->mutex);
			return -ERESTARTSYS;
		}
		if (dmxdevfilter->state < DMXDEV_STATE_SET)
			ret = -EINVAL;
		else
			ret = dvb_dmxdev_filter_start(dmxdevfilter);
		mutex_unlock(&dmxdevfilter->mutex);
		break;

	case DMX_STOP:
		if (mutex_lock_interruptible(&dmxdevfilter->mutex)) {
			mutex_unlock(&dmxdev->mutex);
			return -ERESTARTSYS;
		}
		ret = dvb_dmxdev_filter_stop(dmxdevfilter);
		mutex_unlock(&dmxdevfilter->mutex);
		break;

	case DMX_SET_FILTER:
		if (mutex_lock_interruptible(&dmxdevfilter->mutex)) {
			mutex_unlock(&dmxdev->mutex);
			return -ERESTARTSYS;
		}
		ret = dvb_dmxdev_filter_set(dmxdev, dmxdevfilter, parg);
		mutex_unlock(&dmxdevfilter->mutex);
		break;

	case DMX_SET_PES_FILTER:
		if (mutex_lock_interruptible(&dmxdevfilter->mutex)) {
			mutex_unlock(&dmxdev->mutex);
			return -ERESTARTSYS;
		}
		ret = dvb_dmxdev_pes_filter_set(dmxdev, dmxdevfilter, parg);
		mutex_unlock(&dmxdevfilter->mutex);
		break;

	case DMX_SET_BUFFER_SIZE:
		if (mutex_lock_interruptible(&dmxdevfilter->mutex)) {
			mutex_unlock(&dmxdev->mutex);
			return -ERESTARTSYS;
		}
		ret = dvb_dmxdev_set_buffer_size(dmxdevfilter, arg);
		mutex_unlock(&dmxdevfilter->mutex);
		break;

	case DMX_SET_BUFFER_MODE:
		if (mutex_lock_interruptible(&dmxdevfilter->mutex)) {
			mutex_unlock(&dmxdev->mutex);
			return -ERESTARTSYS;
		}
		ret = dvb_dmxdev_set_buffer_mode(dmxdevfilter,
				*(enum dmx_buffer_mode *)parg);
		mutex_unlock(&dmxdevfilter->mutex);
		break;

	case DMX_SET_BUFFER:
		if (mutex_lock_interruptible(&dmxdevfilter->mutex)) {
			mutex_unlock(&dmxdev->mutex);
			return -ERESTARTSYS;
		}
		ret = dvb_dmxdev_set_buffer(dmxdevfilter, parg);
		mutex_unlock(&dmxdevfilter->mutex);
		break;

	case DMX_GET_BUFFER_STATUS:
		if (mutex_lock_interruptible(&dmxdevfilter->mutex)) {
			mutex_unlock(&dmxdev->mutex);
			return -ERESTARTSYS;
		}
		ret = dvb_dmxdev_get_buffer_status(dmxdevfilter, parg);
		mutex_unlock(&dmxdevfilter->mutex);
		break;

	case DMX_RELEASE_DATA:
		if (mutex_lock_interruptible(&dmxdevfilter->mutex)) {
			mutex_unlock(&dmxdev->mutex);
			return -ERESTARTSYS;
		}
		ret = dvb_dmxdev_release_data(dmxdevfilter, arg);
		mutex_unlock(&dmxdevfilter->mutex);
		break;

	case DMX_GET_PES_PIDS:
		if (!dmxdev->demux->get_pes_pids) {
			ret = -EINVAL;
			break;
		}
		dmxdev->demux->get_pes_pids(dmxdev->demux, parg);
		break;

	case DMX_GET_CAPS:
		if (!dmxdev->demux->get_caps) {
			ret = -EINVAL;
			break;
		}
		ret = dmxdev->demux->get_caps(dmxdev->demux, parg);
		break;

	case DMX_SET_SOURCE:
		if (mutex_lock_interruptible(&dmxdevfilter->mutex)) {
			mutex_unlock(&dmxdev->mutex);
			return -ERESTARTSYS;
		}
		ret = dvb_dmxdev_set_source(dmxdevfilter, parg);
		mutex_unlock(&dmxdevfilter->mutex);
		break;

	case DMX_SET_TS_PACKET_FORMAT:
		if (!dmxdev->demux->set_tsp_format) {
			ret = -EINVAL;
			break;
		}

		if (dmxdevfilter->state >= DMXDEV_STATE_GO) {
			ret = -EBUSY;
			break;
		}
		ret = dmxdev->demux->set_tsp_format(
				dmxdev->demux,
				*(enum dmx_tsp_format_t *)parg);
		break;

	case DMX_SET_TS_OUT_FORMAT:
		if (mutex_lock_interruptible(&dmxdevfilter->mutex)) {
			mutex_unlock(&dmxdev->mutex);
			return -ERESTARTSYS;
		}

		ret = dvb_dmxdev_set_tsp_out_format(dmxdevfilter,
				*(enum dmx_tsp_format_t *)parg);

		mutex_unlock(&dmxdevfilter->mutex);
		break;

	case DMX_SET_DECODER_BUFFER_SIZE:
		if (mutex_lock_interruptible(&dmxdevfilter->mutex)) {
			mutex_unlock(&dmxdev->mutex);
			return -ERESTARTSYS;
		}

		ret = dvb_dmxdev_set_decoder_buffer_size(dmxdevfilter, arg);
		mutex_unlock(&dmxdevfilter->mutex);
		break;

	case DMX_SET_PLAYBACK_MODE:
		ret = dvb_dmxdev_set_playback_mode(
				dmxdevfilter,
				*(enum dmx_playback_mode_t *)parg);
		break;

	case DMX_GET_EVENT:
		if (mutex_lock_interruptible(&dmxdevfilter->mutex)) {
			mutex_unlock(&dmxdev->mutex);
			return -ERESTARTSYS;
		}
		ret = dvb_dmxdev_get_event(dmxdevfilter, parg);
		mutex_unlock(&dmxdevfilter->mutex);
		break;

	case DMX_GET_STC:
		if (!dmxdev->demux->get_stc) {
			ret = -EINVAL;
			break;
		}
		ret = dmxdev->demux->get_stc(dmxdev->demux,
					     ((struct dmx_stc *)parg)->num,
					     &((struct dmx_stc *)parg)->stc,
					     &((struct dmx_stc *)parg)->base);
		break;

	case DMX_ADD_PID:
		if (mutex_lock_interruptible(&dmxdevfilter->mutex)) {
			ret = -ERESTARTSYS;
			break;
		}
		ret = dvb_dmxdev_add_pid(dmxdev, dmxdevfilter, *(u16 *)parg);
		mutex_unlock(&dmxdevfilter->mutex);
		break;

	case DMX_REMOVE_PID:
		if (mutex_lock_interruptible(&dmxdevfilter->mutex)) {
			ret = -ERESTARTSYS;
			break;
		}
		ret = dvb_dmxdev_remove_pid(dmxdev, dmxdevfilter, *(u16 *)parg);
		mutex_unlock(&dmxdevfilter->mutex);
		break;

	case DMX_SET_DECODER_BUFFER:
		if (mutex_lock_interruptible(&dmxdevfilter->mutex)) {
			ret = -ERESTARTSYS;
			break;
		}
		ret = dvb_dmxdev_set_decoder_buffer(dmxdev, dmxdevfilter, parg);
		mutex_unlock(&dmxdevfilter->mutex);
		break;

	case DMX_SET_SECURE_MODE:
		if (mutex_lock_interruptible(&dmxdevfilter->mutex)) {
			ret = -ERESTARTSYS;
			break;
		}
		ret = dvb_dmxdev_set_secure_mode(dmxdev, dmxdevfilter, parg);
		mutex_unlock(&dmxdevfilter->mutex);
		break;

	case DMX_REUSE_DECODER_BUFFER:
		if (mutex_lock_interruptible(&dmxdevfilter->mutex)) {
			mutex_unlock(&dmxdev->mutex);
			return -ERESTARTSYS;
		}
		ret = dvb_dmxdev_reuse_decoder_buf(dmxdevfilter, arg);
		mutex_unlock(&dmxdevfilter->mutex);
		break;

	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&dmxdev->mutex);
	return ret;
}

static long dvb_demux_ioctl(struct file *file, unsigned int cmd,
			    unsigned long arg)
{
	return dvb_usercopy(file, cmd, arg, dvb_demux_do_ioctl);
}

static unsigned int dvb_demux_poll(struct file *file, poll_table *wait)
{
	struct dmxdev_filter *dmxdevfilter = file->private_data;
	unsigned int mask = 0;

	if (!dmxdevfilter)
		return -EINVAL;

	poll_wait(file, &dmxdevfilter->buffer.queue, wait);

	if (dmxdevfilter->state != DMXDEV_STATE_GO &&
	    dmxdevfilter->state != DMXDEV_STATE_DONE &&
	    dmxdevfilter->state != DMXDEV_STATE_TIMEDOUT)
		return 0;

	if (dmxdevfilter->buffer.error)
		mask |= (POLLIN | POLLRDNORM | POLLERR);

	if (!dvb_ringbuffer_empty(&dmxdevfilter->buffer))
		mask |= (POLLIN | POLLRDNORM);

	if (dmxdevfilter->events.notified_index !=
		dmxdevfilter->events.write_index) {
		mask |= POLLPRI;
	}

	return mask;
}

static int dvb_demux_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct dmxdev_filter *dmxdevfilter = filp->private_data;
	struct dmxdev *dmxdev = dmxdevfilter->dev;
	int ret;
	int vma_size;
	int buffer_size;

	vma_size = vma->vm_end - vma->vm_start;

	if (vma->vm_flags & VM_WRITE)
		return -EINVAL;

	if (mutex_lock_interruptible(&dmxdev->mutex))
		return -ERESTARTSYS;

	if (mutex_lock_interruptible(&dmxdevfilter->mutex)) {
		mutex_unlock(&dmxdev->mutex);
		return -ERESTARTSYS;
	}

	if ((!dmxdevfilter->buffer.data) ||
		(dmxdevfilter->buffer_mode == DMX_BUFFER_MODE_EXTERNAL)) {
		mutex_unlock(&dmxdevfilter->mutex);
		mutex_unlock(&dmxdev->mutex);
		return -EINVAL;
	}

	/* Make sure requested mapping is not larger than buffer size */
	buffer_size = dmxdevfilter->buffer.size + (PAGE_SIZE-1);
	buffer_size = buffer_size & ~(PAGE_SIZE-1);

	if (vma_size != buffer_size) {
		mutex_unlock(&dmxdevfilter->mutex);
		mutex_unlock(&dmxdev->mutex);
		return -EINVAL;
	}

	ret = remap_vmalloc_range(vma, dmxdevfilter->buffer.data, 0);
	if (ret) {
		mutex_unlock(&dmxdevfilter->mutex);
		mutex_unlock(&dmxdev->mutex);
		return ret;
	}

	vma->vm_flags |= VM_RESERVED;
	vma->vm_flags |= VM_DONTEXPAND;

	mutex_unlock(&dmxdevfilter->mutex);
	mutex_unlock(&dmxdev->mutex);

	return 0;
}

static int dvb_demux_release(struct inode *inode, struct file *file)
{
	struct dmxdev_filter *dmxdevfilter = file->private_data;
	struct dmxdev *dmxdev = dmxdevfilter->dev;
	int ret;
	ret = dvb_dmxdev_filter_free(dmxdev, dmxdevfilter);

	mutex_lock(&dmxdev->mutex);
	dmxdev->dvbdev->users--;
	if(dmxdev->dvbdev->users==1 && dmxdev->exit==1) {
		fops_put(file->f_op);
		file->f_op = NULL;
		mutex_unlock(&dmxdev->mutex);
		wake_up(&dmxdev->dvbdev->wait_queue);
	} else
		mutex_unlock(&dmxdev->mutex);

	return ret;
}

static const struct file_operations dvb_demux_fops = {
	.owner = THIS_MODULE,
	.read = dvb_demux_read,
	.unlocked_ioctl = dvb_demux_ioctl,
	.open = dvb_demux_open,
	.release = dvb_demux_release,
	.poll = dvb_demux_poll,
	.llseek = default_llseek,
	.mmap = dvb_demux_mmap,
};

static struct dvb_device dvbdev_demux = {
	.priv = NULL,
	.users = 1,
	.writers = 1,
	.fops = &dvb_demux_fops
};

static int dvb_dvr_do_ioctl(struct file *file,
			    unsigned int cmd, void *parg)
{
	struct dvb_device *dvbdev = file->private_data;
	struct dmxdev *dmxdev = dvbdev->priv;
	unsigned long arg = (unsigned long)parg;
	int ret;

	if (mutex_lock_interruptible(&dmxdev->mutex))
		return -ERESTARTSYS;

	switch (cmd) {
	case DMX_SET_BUFFER_SIZE:
		ret = dvb_dvr_set_buffer_size(dmxdev, file->f_flags, arg);
		break;

	case DMX_SET_BUFFER_MODE:
		ret = dvb_dvr_set_buffer_mode(dmxdev, file->f_flags,
			*(enum dmx_buffer_mode *)parg);
		break;

	case DMX_SET_BUFFER:
		ret = dvb_dvr_set_buffer(dmxdev, file->f_flags, parg);
		break;

	case DMX_GET_BUFFER_STATUS:
		ret = dvb_dvr_get_buffer_status(dmxdev, file->f_flags, parg);
		break;

	case DMX_RELEASE_DATA:
		ret = dvb_dvr_release_data(dmxdev, file->f_flags, arg);
		break;

	case DMX_FEED_DATA:
		ret = dvb_dvr_feed_data(dmxdev, file->f_flags, arg);
		break;

	case DMX_GET_EVENT:
		ret = dvb_dvr_get_event(dmxdev, file->f_flags, parg);
		break;

	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&dmxdev->mutex);
	return ret;
}

static long dvb_dvr_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	return dvb_usercopy(file, cmd, arg, dvb_dvr_do_ioctl);
}

static unsigned int dvb_dvr_poll(struct file *file, poll_table *wait)
{
	struct dvb_device *dvbdev = file->private_data;
	struct dmxdev *dmxdev = dvbdev->priv;
	unsigned int mask = 0;

#ifdef CONFIG_DEBUG_PRINTK
	dprintk("function : %s\n", __func__);
#else
	d;
#endif

	if ((file->f_flags & O_ACCMODE) == O_RDONLY) {
		poll_wait(file, &dmxdev->dvr_buffer.queue, wait);

		if (dmxdev->dvr_buffer.error)
			mask |= (POLLIN | POLLRDNORM | POLLERR);

		if (!dvb_ringbuffer_empty(&dmxdev->dvr_buffer))
			mask |= (POLLIN | POLLRDNORM);

		if (dmxdev->dvr_output_events.notified_index !=
			dmxdev->dvr_output_events.write_index)
			mask |= POLLPRI;
	} else {
		poll_wait(file, &dmxdev->dvr_input_buffer.queue, wait);
		if (dmxdev->dvr_input_buffer.error)
			mask |= (POLLOUT | POLLRDNORM | POLLPRI | POLLERR);

		if (dvb_ringbuffer_free(&dmxdev->dvr_input_buffer))
			mask |= (POLLOUT | POLLRDNORM | POLLPRI);
	}

	return mask;
}

static const struct file_operations dvb_dvr_fops = {
	.owner = THIS_MODULE,
	.read = dvb_dvr_read,
	.write = dvb_dvr_write,
	.mmap = dvb_dvr_mmap,
	.unlocked_ioctl = dvb_dvr_ioctl,
	.open = dvb_dvr_open,
	.release = dvb_dvr_release,
	.poll = dvb_dvr_poll,
	.llseek = default_llseek,
};

static struct dvb_device dvbdev_dvr = {
	.priv = NULL,
	.readers = 1,
	.users = 1,
	.fops = &dvb_dvr_fops
};


/**
 * debugfs service to print active filters information.
 */
static int dvb_dmxdev_dbgfs_print(struct seq_file *s, void *p)
{
	int i;
	struct dmxdev *dmxdev = s->private;
	struct dmxdev_filter *filter;
	int active_count = 0;
	struct dmx_buffer_status buffer_status;
	const char *pes_feeds[] = {"DEC", "PES", "DVR", "REC"};

	if (!dmxdev)
		return 0;

	for (i = 0; i < dmxdev->filternum; i++) {
		filter = &dmxdev->filter[i];
		if (filter->state >= DMXDEV_STATE_GO) {
			active_count++;

			seq_printf(s, "filter_%02d - ", i);

			if (filter->type == DMXDEV_TYPE_SEC) {
				seq_printf(s, "type: SEC, ");
				seq_printf(s, "PID %04d ",
						filter->params.sec.pid);
			} else {
				seq_printf(s, "type: %s, ",
					pes_feeds[filter->params.pes.output]);
				seq_printf(s, "PID: %04d ",
						filter->params.pes.pid);
			}

			if (0 == dvb_dmxdev_get_buffer_status(
						filter, &buffer_status)) {
				seq_printf(s, "size: %08d, ",
					buffer_status.size);
				seq_printf(s, "fullness: %08d, ",
					buffer_status.fullness);
				seq_printf(s, "error: %d\n",
					buffer_status.error);
			} else {
				seq_printf(s, "\n");
			}
		}
	}

	if (!active_count)
		seq_printf(s, "No active filters\n");

	return 0;
}

static int dvb_dmxdev_dbgfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, dvb_dmxdev_dbgfs_print, inode->i_private);
}

static const struct file_operations dbgfs_filters_fops = {
	.open = dvb_dmxdev_dbgfs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

int dvb_dmxdev_init(struct dmxdev *dmxdev, struct dvb_adapter *dvb_adapter)
{
	int i;

	if (dmxdev->demux->open(dmxdev->demux) < 0)
		return -EUSERS;

	dmxdev->filter = vmalloc(dmxdev->filternum * sizeof(struct dmxdev_filter));
	if (!dmxdev->filter)
		return -ENOMEM;

	dmxdev->playback_mode = DMX_PB_MODE_PUSH;

	mutex_init(&dmxdev->mutex);
	spin_lock_init(&dmxdev->lock);
	spin_lock_init(&dmxdev->dvr_in_lock);
	for (i = 0; i < dmxdev->filternum; i++) {
		dmxdev->filter[i].dev = dmxdev;
		dmxdev->filter[i].buffer.data = NULL;
		dvb_dmxdev_filter_state_set(&dmxdev->filter[i],
					    DMXDEV_STATE_FREE);
	}

	dvb_register_device(dvb_adapter, &dmxdev->dvbdev, &dvbdev_demux, dmxdev,
			    DVB_DEVICE_DEMUX);
	dvb_register_device(dvb_adapter, &dmxdev->dvr_dvbdev, &dvbdev_dvr,
			    dmxdev, DVB_DEVICE_DVR);

	dvb_ringbuffer_init(&dmxdev->dvr_buffer, NULL, 8192);
	dvb_ringbuffer_init(&dmxdev->dvr_input_buffer, NULL, 8192);

	if (dmxdev->demux->debugfs_demux_dir)
		debugfs_create_file("filters", S_IRUGO,
			dmxdev->demux->debugfs_demux_dir, dmxdev,
			&dbgfs_filters_fops);

	return 0;
}

EXPORT_SYMBOL(dvb_dmxdev_init);

void dvb_dmxdev_release(struct dmxdev *dmxdev)
{
	dmxdev->exit=1;
	if (dmxdev->dvbdev->users > 1) {
		wait_event(dmxdev->dvbdev->wait_queue,
				dmxdev->dvbdev->users==1);
	}
	if (dmxdev->dvr_dvbdev->users > 1) {
		wait_event(dmxdev->dvr_dvbdev->wait_queue,
				dmxdev->dvr_dvbdev->users==1);
	}

	dvb_unregister_device(dmxdev->dvbdev);
	dvb_unregister_device(dmxdev->dvr_dvbdev);

	vfree(dmxdev->filter);
	dmxdev->filter = NULL;
	dmxdev->demux->close(dmxdev->demux);
}

EXPORT_SYMBOL(dvb_dmxdev_release);
