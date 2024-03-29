/* Driver for Realtek PCI-Express card reader
 *
 * Copyright(c) 2009 Realtek Semiconductor Corp. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 * Author:
 *   wwang (wei_wang@realsil.com.cn)
 *   No. 450, Shenhu Road, Suzhou Industry Park, Suzhou, China
 */

#include <linux/blkdev.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/workqueue.h>

#include "rtsx.h"
#include "rtsx_chip.h"
#include "rtsx_transport.h"
#include "rtsx_scsi.h"
#include "rtsx_card.h"
#include "general.h"

#include "ms.h"
#include "sd.h"
#include "xd.h"

#define DRIVER_VERSION "v1.10"

MODULE_DESCRIPTION("Realtek PCI-Express card reader driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

static unsigned int delay_use = 1;
module_param(delay_use, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(delay_use, "seconds to delay before using a new device");

static int ss_en;
module_param(ss_en, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(ss_en, "enable selective suspend");

static int ss_interval = 50;
module_param(ss_interval, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(ss_interval, "Interval to enter ss state in seconds");

static int auto_delink_en;
module_param(auto_delink_en, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(auto_delink_en, "enable auto delink");

static unsigned char aspm_l0s_l1_en;
module_param(aspm_l0s_l1_en, byte, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(aspm_l0s_l1_en, "enable device aspm");

static int msi_en;
module_param(msi_en, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(msi_en, "enable msi");

static irqreturn_t rtsx_interrupt(int irq, void *dev_id);

/***********************************************************************
 * Host functions
 ***********************************************************************/

static const char *host_info(struct Scsi_Host *host)
{
	return "SCSI emulation for PCI-Express Mass Storage devices";
}

static int slave_alloc(struct scsi_device *sdev)
{
	/*
	 * Set the INQUIRY transfer length to 36.  We don't use any of
	 * the extra data and many devices choke if asked for more or
	 * less than 36 bytes.
	 */
	sdev->inquiry_len = 36;
	return 0;
}

static int slave_configure(struct scsi_device *sdev)
{
	/* Scatter-gather buffers (all but the last) must have a length
	 * divisible by the bulk maxpacket size.  Otherwise a data packet
	 * would end up being short, causing a premature end to the data
	 * transfer.  Since high-speed bulk pipes have a maxpacket size
	 * of 512, we'll use that as the scsi device queue's DMA alignment
	 * mask.  Guaranteeing proper alignment of the first buffer will
	 * have the desired effect because, except at the beginning and
	 * the end, scatter-gather buffers follow page boundaries. */
	blk_queue_dma_alignment(sdev->request_queue, (512 - 1));

	/* Set the SCSI level to at least 2.  We'll leave it at 3 if that's
	 * what is originally reported.  We need this to avoid confusing
	 * the SCSI layer with devices that report 0 or 1, but need 10-byte
	 * commands (ala ATAPI devices behind certain bridges, or devices
	 * which simply have broken INQUIRY data).
	 *
	 * NOTE: This means /dev/sg programs (ala cdrecord) will get the
	 * actual information.  This seems to be the preference for
	 * programs like that.
	 *
	 * NOTE: This also means that /proc/scsi/scsi and sysfs may report
	 * the actual value or the modified one, depending on where the
	 * data comes from.
	 */
	if (sdev->scsi_level < SCSI_2)
		sdev->scsi_level = sdev->sdev_target->scsi_level = SCSI_2;

	return 0;
}


/***********************************************************************
 * /proc/scsi/ functions
 ***********************************************************************/

/* we use this macro to help us write into the buffer */
#undef SPRINTF
#define SPRINTF(args...) \
	do { if (pos < buffer+length) pos += sprintf(pos, ## args); } while (0)

static int proc_info(struct Scsi_Host *host, char *buffer,
		char **start, off_t offset, int length, int inout)
{
	char *pos = buffer;

	/* if someone is sending us data, just throw it away */
	if (inout)
		return length;

	/* print the controller name */
	SPRINTF("   Host scsi%d: %s\n", host->host_no, CR_DRIVER_NAME);

	/* print product, vendor, and driver version strings */
	SPRINTF("       Vendor: Realtek Corp.\n");
	SPRINTF("      Product: PCIE Card Reader\n");
	SPRINTF("      Version: %s\n", DRIVER_VERSION);

	/*
	 * Calculate start of next buffer, and return value.
	 */
	*start = buffer + offset;

	if ((pos - buffer) < offset)
		return 0;
	else if ((pos - buffer - offset) < length)
		return pos - buffer - offset;
	else
		return length;
}

/* queue a command */
/* This is always called with scsi_lock(host) held */
static int queuecommand_lck(struct scsi_cmnd *srb,
			void (*done)(struct scsi_cmnd *))
{
	struct rtsx_dev *dev = host_to_rtsx(srb->device->host);
	struct rtsx_chip *chip = dev->chip;

	/* check for state-transition errors */
	if (chip->srb != NULL) {
		printk(KERN_ERR "Error in %s: chip->srb = %p\n",
			__func__, chip->srb);
		return SCSI_MLQUEUE_HOST_BUSY;
	}

	/* fail the command if we are disconnecting */
	if (rtsx_chk_stat(chip, RTSX_STAT_DISCONNECT)) {
#ifdef CONFIG_DEBUG_PRINTK
		printk(KERN_INFO "Fail command during disconnect\n");
#else
		;
#endif
		srb->result = DID_NO_CONNECT << 16;
		done(srb);
		return 0;
	}

	/* enqueue the command and wake up the control thread */
	srb->scsi_done = done;
	chip->srb = srb;
	complete(&dev->cmnd_ready);

	return 0;
}

static DEF_SCSI_QCMD(queuecommand)

/***********************************************************************
 * Error handling functions
 ***********************************************************************/

/* Command timeout and abort */
static int command_abort(struct scsi_cmnd *srb)
{
	struct Scsi_Host *host = srb->device->host;
	struct rtsx_dev *dev = host_to_rtsx(host);
	struct rtsx_chip *chip = dev->chip;

#ifdef CONFIG_DEBUG_PRINTK
	printk(KERN_INFO "%s called\n", __func__);
#else
	;
#endif

	scsi_lock(host);

	/* Is this command still active? */
	if (chip->srb != srb) {
		scsi_unlock(host);
#ifdef CONFIG_DEBUG_PRINTK
		printk(KERN_INFO "-- nothing to abort\n");
#else
		;
#endif
		return FAILED;
	}

	rtsx_set_stat(chip, RTSX_STAT_ABORT);

	scsi_unlock(host);

	/* Wait for the aborted command to finish */
	wait_for_completion(&dev->notify);

	return SUCCESS;
}

/* This invokes the transport reset mechanism to reset the state of the
 * device */
static int device_reset(struct scsi_cmnd *srb)
{
	int result = 0;

#ifdef CONFIG_DEBUG_PRINTK
	printk(KERN_INFO "%s called\n", __func__);
#else
	;
#endif

	return result < 0 ? FAILED : SUCCESS;
}

/* Simulate a SCSI bus reset by resetting the device's USB port. */
static int bus_reset(struct scsi_cmnd *srb)
{
	int result = 0;

#ifdef CONFIG_DEBUG_PRINTK
	printk(KERN_INFO "%s called\n", __func__);
#else
	;
#endif

	return result < 0 ? FAILED : SUCCESS;
}


/*
 * this defines our host template, with which we'll allocate hosts
 */

static struct scsi_host_template rtsx_host_template = {
	/* basic userland interface stuff */
	.name =				CR_DRIVER_NAME,
	.proc_name =			CR_DRIVER_NAME,
	.proc_info =			proc_info,
	.info =				host_info,

	/* command interface -- queued only */
	.queuecommand =			queuecommand,

	/* error and abort handlers */
	.eh_abort_handler =		command_abort,
	.eh_device_reset_handler =	device_reset,
	.eh_bus_reset_handler =		bus_reset,

	/* queue commands only, only one command per LUN */
	.can_queue =			1,
	.cmd_per_lun =			1,

	/* unknown initiator id */
	.this_id =			-1,

	.slave_alloc =			slave_alloc,
	.slave_configure =		slave_configure,

	/* lots of sg segments can be handled */
	.sg_tablesize =			SG_ALL,

	/* limit the total size of a transfer to 120 KB */
	.max_sectors =                  240,

	/* merge commands... this seems to help performance, but
	 * periodically someone should test to see which setting is more
	 * optimal.
	 */
	.use_clustering =		1,

	/* emulated HBA */
	.emulated =			1,

	/* we do our own delay after a device or bus reset */
	.skip_settle_delay =		1,

	/* module management */
	.module =			THIS_MODULE
};


static int rtsx_acquire_irq(struct rtsx_dev *dev)
{
	struct rtsx_chip *chip = dev->chip;

#ifdef CONFIG_DEBUG_PRINTK
	printk(KERN_INFO "%s: chip->msi_en = %d, pci->irq = %d\n",
			__func__, chip->msi_en, dev->pci->irq);
#else
	;
#endif

	if (request_irq(dev->pci->irq, rtsx_interrupt,
			chip->msi_en ? 0 : IRQF_SHARED,
			CR_DRIVER_NAME, dev)) {
		printk(KERN_ERR "rtsx: unable to grab IRQ %d, "
		       "disabling device\n", dev->pci->irq);
		return -1;
	}

	dev->irq = dev->pci->irq;
	pci_intx(dev->pci, !chip->msi_en);

	return 0;
}


int rtsx_read_pci_cfg_byte(u8 bus, u8 dev, u8 func, u8 offset, u8 *val)
{
	struct pci_dev *pdev;
	u8 data;
	u8 devfn = (dev << 3) | func;

	pdev = pci_get_bus_and_slot(bus, devfn);
	if (!pdev)
		return -1;

	pci_read_config_byte(pdev, offset, &data);
	if (val)
		*val = data;

	return 0;
}

#ifdef CONFIG_PM
/*
 * power management
 */
static int rtsx_suspend(struct pci_dev *pci, pm_message_t state)
{
	struct rtsx_dev *dev = (struct rtsx_dev *)pci_get_drvdata(pci);
	struct rtsx_chip *chip;

#ifdef CONFIG_DEBUG_PRINTK
	printk(KERN_INFO "Ready to suspend\n");
#else
	;
#endif

	if (!dev) {
		printk(KERN_ERR "Invalid memory\n");
		return 0;
	}

	/* lock the device pointers */
	mutex_lock(&(dev->dev_mutex));

	chip = dev->chip;

	rtsx_do_before_power_down(chip, PM_S3);

	if (dev->irq >= 0) {
		synchronize_irq(dev->irq);
		free_irq(dev->irq, (void *)dev);
		dev->irq = -1;
	}

	if (chip->msi_en)
		pci_disable_msi(pci);

	pci_save_state(pci);
	pci_enable_wake(pci, pci_choose_state(pci, state), 1);
	pci_disable_device(pci);
	pci_set_power_state(pci, pci_choose_state(pci, state));

	/* unlock the device pointers */
	mutex_unlock(&dev->dev_mutex);

	return 0;
}

static int rtsx_resume(struct pci_dev *pci)
{
	struct rtsx_dev *dev = (struct rtsx_dev *)pci_get_drvdata(pci);
	struct rtsx_chip *chip;

#ifdef CONFIG_DEBUG_PRINTK
	printk(KERN_INFO "Ready to resume\n");
#else
	;
#endif

	if (!dev) {
		printk(KERN_ERR "Invalid memory\n");
		return 0;
	}

	chip = dev->chip;

	/* lock the device pointers */
	mutex_lock(&(dev->dev_mutex));

	pci_set_power_state(pci, PCI_D0);
	pci_restore_state(pci);
	if (pci_enable_device(pci) < 0) {
		printk(KERN_ERR "%s: pci_enable_device failed, "
		       "disabling device\n", CR_DRIVER_NAME);
		/* unlock the device pointers */
		mutex_unlock(&dev->dev_mutex);
		return -EIO;
	}
	pci_set_master(pci);

	if (chip->msi_en) {
		if (pci_enable_msi(pci) < 0)
			chip->msi_en = 0;
	}

	if (rtsx_acquire_irq(dev) < 0) {
		/* unlock the device pointers */
		mutex_unlock(&dev->dev_mutex);
		return -EIO;
	}

	rtsx_write_register(chip, HOST_SLEEP_STATE, 0x03, 0x00);
	rtsx_init_chip(chip);

	/* unlock the device pointers */
	mutex_unlock(&dev->dev_mutex);

	return 0;
}
#endif /* CONFIG_PM */

static void rtsx_shutdown(struct pci_dev *pci)
{
	struct rtsx_dev *dev = (struct rtsx_dev *)pci_get_drvdata(pci);
	struct rtsx_chip *chip;

#ifdef CONFIG_DEBUG_PRINTK
	printk(KERN_INFO "Ready to shutdown\n");
#else
	;
#endif

	if (!dev) {
		printk(KERN_ERR "Invalid memory\n");
		return;
	}

	chip = dev->chip;

	rtsx_do_before_power_down(chip, PM_S1);

	if (dev->irq >= 0) {
		synchronize_irq(dev->irq);
		free_irq(dev->irq, (void *)dev);
		dev->irq = -1;
	}

	if (chip->msi_en)
		pci_disable_msi(pci);

	pci_disable_device(pci);

	return;
}

static int rtsx_control_thread(void *__dev)
{
	struct rtsx_dev *dev = (struct rtsx_dev *)__dev;
	struct rtsx_chip *chip = dev->chip;
	struct Scsi_Host *host = rtsx_to_host(dev);

	for (;;) {
		if (wait_for_completion_interruptible(&dev->cmnd_ready))
			break;

		/* lock the device pointers */
		mutex_lock(&(dev->dev_mutex));

		/* if the device has disconnected, we are free to exit */
		if (rtsx_chk_stat(chip, RTSX_STAT_DISCONNECT)) {
#ifdef CONFIG_DEBUG_PRINTK
			printk(KERN_INFO "-- rtsx-control exiting\n");
#else
			;
#endif
			mutex_unlock(&dev->dev_mutex);
			break;
		}

		/* lock access to the state */
		scsi_lock(host);

		/* has the command aborted ? */
		if (rtsx_chk_stat(chip, RTSX_STAT_ABORT)) {
			chip->srb->result = DID_ABORT << 16;
			goto SkipForAbort;
		}

		scsi_unlock(host);

		/* reject the command if the direction indicator
		 * is UNKNOWN
		 */
		if (chip->srb->sc_data_direction == DMA_BIDIRECTIONAL) {
			printk(KERN_ERR "UNKNOWN data direction\n");
			chip->srb->result = DID_ERROR << 16;
		}

		/* reject if target != 0 or if LUN is higher than
		 * the maximum known LUN
		 */
		else if (chip->srb->device->id) {
			printk(KERN_ERR "Bad target number (%d:%d)\n",
				chip->srb->device->id,
				chip->srb->device->lun);
			chip->srb->result = DID_BAD_TARGET << 16;
		}

		else if (chip->srb->device->lun > chip->max_lun) {
			printk(KERN_ERR "Bad LUN (%d:%d)\n",
				chip->srb->device->id,
				chip->srb->device->lun);
			chip->srb->result = DID_BAD_TARGET << 16;
		}

		/* we've got a command, let's do it! */
		else {
			RTSX_DEBUG(scsi_show_command(chip->srb));
			rtsx_invoke_transport(chip->srb, chip);
		}

		/* lock access to the state */
		scsi_lock(host);

		/* did the command already complete because of a disconnect? */
		if (!chip->srb)
			;		/* nothing to do */

		/* indicate that the command is done */
		else if (chip->srb->result != DID_ABORT << 16) {
			chip->srb->scsi_done(chip->srb);
		} else {
SkipForAbort:
			printk(KERN_ERR "scsi command aborted\n");
		}

		if (rtsx_chk_stat(chip, RTSX_STAT_ABORT)) {
			complete(&(dev->notify));

			rtsx_set_stat(chip, RTSX_STAT_IDLE);
		}

		/* finished working on this command */
		chip->srb = NULL;
		scsi_unlock(host);

		/* unlock the device pointers */
		mutex_unlock(&dev->dev_mutex);
	} /* for (;;) */

	/* notify the exit routine that we're actually exiting now
	 *
	 * complete()/wait_for_completion() is similar to up()/down(),
	 * except that complete() is safe in the case where the structure
	 * is getting deleted in a parallel mode of execution (i.e. just
	 * after the down() -- that's necessary for the thread-shutdown
	 * case.
	 *
	 * complete_and_exit() goes even further than this -- it is safe in
	 * the case that the thread of the caller is going away (not just
	 * the structure) -- this is necessary for the module-remove case.
	 * This is important in preemption kernels, which transfer the flow
	 * of execution immediately upon a complete().
	 */
	complete_and_exit(&dev->control_exit, 0);
}


static int rtsx_polling_thread(void *__dev)
{
	struct rtsx_dev *dev = (struct rtsx_dev *)__dev;
	struct rtsx_chip *chip = dev->chip;
	struct sd_info *sd_card = &(chip->sd_card);
	struct xd_info *xd_card = &(chip->xd_card);
	struct ms_info *ms_card = &(chip->ms_card);

	sd_card->cleanup_counter = 0;
	xd_card->cleanup_counter = 0;
	ms_card->cleanup_counter = 0;

	/* Wait until SCSI scan finished */
	wait_timeout((delay_use + 5) * 1000);

	for (;;) {

		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(POLLING_INTERVAL);

		/* lock the device pointers */
		mutex_lock(&(dev->dev_mutex));

		/* if the device has disconnected, we are free to exit */
		if (rtsx_chk_stat(chip, RTSX_STAT_DISCONNECT)) {
#ifdef CONFIG_DEBUG_PRINTK
			printk(KERN_INFO "-- rtsx-polling exiting\n");
#else
			;
#endif
			mutex_unlock(&dev->dev_mutex);
			break;
		}

		mutex_unlock(&dev->dev_mutex);

		mspro_polling_format_status(chip);

		/* lock the device pointers */
		mutex_lock(&(dev->dev_mutex));

		rtsx_polling_func(chip);

		/* unlock the device pointers */
		mutex_unlock(&dev->dev_mutex);
	}

	complete_and_exit(&dev->polling_exit, 0);
}

/*
 * interrupt handler
 */
static irqreturn_t rtsx_interrupt(int irq, void *dev_id)
{
	struct rtsx_dev *dev = dev_id;
	struct rtsx_chip *chip;
	int retval;
	u32 status;

	if (dev)
		chip = dev->chip;
	else
		return IRQ_NONE;

	if (!chip)
		return IRQ_NONE;

	spin_lock(&dev->reg_lock);

	retval = rtsx_pre_handle_interrupt(chip);
	if (retval == STATUS_FAIL) {
		spin_unlock(&dev->reg_lock);
		if (chip->int_reg == 0xFFFFFFFF)
			return IRQ_HANDLED;
		else
			return IRQ_NONE;
	}

	status = chip->int_reg;

	if (dev->check_card_cd) {
		if (!(dev->check_card_cd & status)) {
			/* card not exist, return TRANS_RESULT_FAIL */
			dev->trans_result = TRANS_RESULT_FAIL;
			if (dev->done)
				complete(dev->done);
			goto Exit;
		}
	}

	if (status & (NEED_COMPLETE_INT | DELINK_INT)) {
		if (status & (TRANS_FAIL_INT | DELINK_INT)) {
			if (status & DELINK_INT)
				RTSX_SET_DELINK(chip);
			dev->trans_result = TRANS_RESULT_FAIL;
			if (dev->done)
				complete(dev->done);
		} else if (status & TRANS_OK_INT) {
			dev->trans_result = TRANS_RESULT_OK;
			if (dev->done)
				complete(dev->done);
		} else if (status & DATA_DONE_INT) {
			dev->trans_result = TRANS_NOT_READY;
			if (dev->done && (dev->trans_state == STATE_TRANS_SG))
				complete(dev->done);
		}
	}

Exit:
	spin_unlock(&dev->reg_lock);
	return IRQ_HANDLED;
}


/* Release all our dynamic resources */
static void rtsx_release_resources(struct rtsx_dev *dev)
{
#ifdef CONFIG_DEBUG_PRINTK
	printk(KERN_INFO "-- %s\n", __func__);
#else
	;
#endif

	/* Tell the control thread to exit.  The SCSI host must
	 * already have been removed so it won't try to queue
	 * any more commands.
	 */
	printk(KERN_INFO "-- sending exit command to thread\n");
	complete(&dev->cmnd_ready);
	if (dev->ctl_thread)
		wait_for_completion(&dev->control_exit);
	if (dev->polling_thread)
		wait_for_completion(&dev->polling_exit);

	wait_timeout(200);

	if (dev->rtsx_resv_buf) {
		dma_free_coherent(&(dev->pci->dev), RTSX_RESV_BUF_LEN,
				dev->rtsx_resv_buf, dev->rtsx_resv_buf_addr);
		dev->chip->host_cmds_ptr = NULL;
		dev->chip->host_sg_tbl_ptr = NULL;
	}

	if (dev->irq > 0)
		free_irq(dev->irq, (void *)dev);
	if (dev->chip->msi_en)
		pci_disable_msi(dev->pci);
	if (dev->remap_addr)
		iounmap(dev->remap_addr);

	pci_disable_device(dev->pci);
	pci_release_regions(dev->pci);

	rtsx_release_chip(dev->chip);
	kfree(dev->chip);
}

/* First stage of disconnect processing: stop all commands and remove
 * the host */
static void quiesce_and_remove_host(struct rtsx_dev *dev)
{
	struct Scsi_Host *host = rtsx_to_host(dev);
	struct rtsx_chip *chip = dev->chip;

	/* Prevent new transfers, stop the current command, and
	 * interrupt a SCSI-scan or device-reset delay */
	mutex_lock(&dev->dev_mutex);
	scsi_lock(host);
	rtsx_set_stat(chip, RTSX_STAT_DISCONNECT);
	scsi_unlock(host);
	mutex_unlock(&dev->dev_mutex);
	wake_up(&dev->delay_wait);
	wait_for_completion(&dev->scanning_done);

	/* Wait some time to let other threads exist */
	wait_timeout(100);

	/* queuecommand won't accept any new commands and the control
	 * thread won't execute a previously-queued command.  If there
	 * is such a command pending, complete it with an error. */
	mutex_lock(&dev->dev_mutex);
	if (chip->srb) {
		chip->srb->result = DID_NO_CONNECT << 16;
		scsi_lock(host);
		chip->srb->scsi_done(dev->chip->srb);
		chip->srb = NULL;
		scsi_unlock(host);
	}
	mutex_unlock(&dev->dev_mutex);

	/* Now we own no commands so it's safe to remove the SCSI host */
	scsi_remove_host(host);
}

/* Second stage of disconnect processing: deallocate all resources */
static void release_everything(struct rtsx_dev *dev)
{
	rtsx_release_resources(dev);

	/* Drop our reference to the host; the SCSI core will free it
	 * when the refcount becomes 0. */
	scsi_host_put(rtsx_to_host(dev));
}

/* Thread to carry out delayed SCSI-device scanning */
static int rtsx_scan_thread(void *__dev)
{
	struct rtsx_dev *dev = (struct rtsx_dev *)__dev;
	struct rtsx_chip *chip = dev->chip;

	/* Wait for the timeout to expire or for a disconnect */
	if (delay_use > 0) {
#ifdef CONFIG_DEBUG_PRINTK
		printk(KERN_INFO "%s: waiting for device "
				"to settle before scanning\n", CR_DRIVER_NAME);
#else
		;
#endif
		wait_event_interruptible_timeout(dev->delay_wait,
				rtsx_chk_stat(chip, RTSX_STAT_DISCONNECT),
				delay_use * HZ);
	}

	/* If the device is still connected, perform the scanning */
	if (!rtsx_chk_stat(chip, RTSX_STAT_DISCONNECT)) {
		scsi_scan_host(rtsx_to_host(dev));
#ifdef CONFIG_DEBUG_PRINTK
		printk(KERN_INFO "%s: device scan complete\n", CR_DRIVER_NAME);
#else
		;
#endif

		/* Should we unbind if no devices were detected? */
	}

	complete_and_exit(&dev->scanning_done, 0);
}

static void rtsx_init_options(struct rtsx_chip *chip)
{
	chip->vendor_id = chip->rtsx->pci->vendor;
	chip->product_id = chip->rtsx->pci->device;
	chip->adma_mode = 1;
	chip->lun_mc = 0;
	chip->driver_first_load = 1;
#ifdef HW_AUTO_SWITCH_SD_BUS
	chip->sdio_in_charge = 0;
#endif

	chip->mspro_formatter_enable = 1;
	chip->ignore_sd = 0;
	chip->use_hw_setting = 0;
	chip->lun_mode = DEFAULT_SINGLE;
	chip->auto_delink_en = auto_delink_en;
	chip->ss_en = ss_en;
	chip->ss_idle_period = ss_interval * 1000;
	chip->remote_wakeup_en = 0;
	chip->aspm_l0s_l1_en = aspm_l0s_l1_en;
	chip->dynamic_aspm = 1;
	chip->fpga_sd_sdr104_clk = CLK_200;
	chip->fpga_sd_ddr50_clk = CLK_100;
	chip->fpga_sd_sdr50_clk = CLK_100;
	chip->fpga_sd_hs_clk = CLK_100;
	chip->fpga_mmc_52m_clk = CLK_80;
	chip->fpga_ms_hg_clk = CLK_80;
	chip->fpga_ms_4bit_clk = CLK_80;
	chip->fpga_ms_1bit_clk = CLK_40;
	chip->asic_sd_sdr104_clk = 203;
	chip->asic_sd_sdr50_clk = 98;
	chip->asic_sd_ddr50_clk = 98;
	chip->asic_sd_hs_clk = 98;
	chip->asic_mmc_52m_clk = 98;
	chip->asic_ms_hg_clk = 117;
	chip->asic_ms_4bit_clk = 78;
	chip->asic_ms_1bit_clk = 39;
	chip->ssc_depth_sd_sdr104 = SSC_DEPTH_2M;
	chip->ssc_depth_sd_sdr50 = SSC_DEPTH_2M;
	chip->ssc_depth_sd_ddr50 = SSC_DEPTH_1M;
	chip->ssc_depth_sd_hs = SSC_DEPTH_1M;
	chip->ssc_depth_mmc_52m = SSC_DEPTH_1M;
	chip->ssc_depth_ms_hg = SSC_DEPTH_1M;
	chip->ssc_depth_ms_4bit = SSC_DEPTH_512K;
	chip->ssc_depth_low_speed = SSC_DEPTH_512K;
	chip->ssc_en = 1;
	chip->sd_speed_prior = 0x01040203;
	chip->sd_current_prior = 0x00010203;
	chip->sd_ctl = SD_PUSH_POINT_AUTO |
		       SD_SAMPLE_POINT_AUTO |
		       SUPPORT_MMC_DDR_MODE;
	chip->sd_ddr_tx_phase = 0;
	chip->mmc_ddr_tx_phase = 1;
	chip->sd_default_tx_phase = 15;
	chip->sd_default_rx_phase = 15;
	chip->pmos_pwr_on_interval = 200;
	chip->sd_voltage_switch_delay = 1000;
	chip->ms_power_class_en = 3;

	chip->sd_400mA_ocp_thd = 1;
	chip->sd_800mA_ocp_thd = 5;
	chip->ms_ocp_thd = 2;

	chip->card_drive_sel = 0x55;
	chip->sd30_drive_sel_1v8 = 0x03;
	chip->sd30_drive_sel_3v3 = 0x01;

	chip->do_delink_before_power_down = 1;
	chip->auto_power_down = 1;
	chip->polling_config = 0;

	chip->force_clkreq_0 = 1;
	chip->ft2_fast_mode = 0;

	chip->sdio_retry_cnt = 1;

	chip->xd_timeout = 2000;
	chip->sd_timeout = 10000;
	chip->ms_timeout = 2000;
	chip->mspro_timeout = 15000;

	chip->power_down_in_ss = 1;

	chip->sdr104_en = 1;
	chip->sdr50_en = 1;
	chip->ddr50_en = 1;

	chip->delink_stage1_step = 100;
	chip->delink_stage2_step = 40;
	chip->delink_stage3_step = 20;

	chip->auto_delink_in_L1 = 1;
	chip->blink_led = 1;
	chip->msi_en = msi_en;
	chip->hp_watch_bios_hotplug = 0;
	chip->max_payload = 0;
	chip->phy_voltage = 0;

	chip->support_ms_8bit = 1;
	chip->s3_pwr_off_delay = 1000;
}

static int __devinit rtsx_probe(struct pci_dev *pci,
				const struct pci_device_id *pci_id)
{
	struct Scsi_Host *host;
	struct rtsx_dev *dev;
	int err = 0;
	struct task_struct *th;

	RTSX_DEBUGP("Realtek PCI-E card reader detected\n");

	err = pci_enable_device(pci);
	if (err < 0) {
		printk(KERN_ERR "PCI enable device failed!\n");
		return err;
	}

	err = pci_request_regions(pci, CR_DRIVER_NAME);
	if (err < 0) {
		printk(KERN_ERR "PCI request regions for %s failed!\n",
		       CR_DRIVER_NAME);
		pci_disable_device(pci);
		return err;
	}

	/*
	 * Ask the SCSI layer to allocate a host structure, with extra
	 * space at the end for our private rtsx_dev structure.
	 */
	host = scsi_host_alloc(&rtsx_host_template, sizeof(*dev));
	if (!host) {
		printk(KERN_ERR "Unable to allocate the scsi host\n");
		pci_release_regions(pci);
		pci_disable_device(pci);
		return -ENOMEM;
	}

	dev = host_to_rtsx(host);
	memset(dev, 0, sizeof(struct rtsx_dev));

	dev->chip = kzalloc(sizeof(struct rtsx_chip), GFP_KERNEL);
	if (dev->chip == NULL)
		goto errout;

	spin_lock_init(&dev->reg_lock);
	mutex_init(&(dev->dev_mutex));
	init_completion(&dev->cmnd_ready);
	init_completion(&dev->control_exit);
	init_completion(&dev->polling_exit);
	init_completion(&(dev->notify));
	init_completion(&dev->scanning_done);
	init_waitqueue_head(&dev->delay_wait);

	dev->pci = pci;
	dev->irq = -1;

	printk(KERN_INFO "Resource length: 0x%x\n",
	       (unsigned int)pci_resource_len(pci, 0));
	dev->addr = pci_resource_start(pci, 0);
	dev->remap_addr = ioremap_nocache(dev->addr, pci_resource_len(pci, 0));
	if (dev->remap_addr == NULL) {
		printk(KERN_ERR "ioremap error\n");
		err = -ENXIO;
		goto errout;
	}

	/*
	 * Using "unsigned long" cast here to eliminate gcc warning in
	 * 64-bit system
	 */
	printk(KERN_INFO "Original address: 0x%lx, remapped address: 0x%lx\n",
	       (unsigned long)(dev->addr), (unsigned long)(dev->remap_addr));

	dev->rtsx_resv_buf = dma_alloc_coherent(&(pci->dev), RTSX_RESV_BUF_LEN,
			&(dev->rtsx_resv_buf_addr), GFP_KERNEL);
	if (dev->rtsx_resv_buf == NULL) {
		printk(KERN_ERR "alloc dma buffer fail\n");
		err = -ENXIO;
		goto errout;
	}
	dev->chip->host_cmds_ptr = dev->rtsx_resv_buf;
	dev->chip->host_cmds_addr = dev->rtsx_resv_buf_addr;
	dev->chip->host_sg_tbl_ptr = dev->rtsx_resv_buf + HOST_CMDS_BUF_LEN;
	dev->chip->host_sg_tbl_addr = dev->rtsx_resv_buf_addr +
				      HOST_CMDS_BUF_LEN;

	dev->chip->rtsx = dev;

	rtsx_init_options(dev->chip);

#ifdef CONFIG_DEBUG_PRINTK
	printk(KERN_INFO "pci->irq = %d\n", pci->irq);
#else
	;
#endif

	if (dev->chip->msi_en) {
		if (pci_enable_msi(pci) < 0)
			dev->chip->msi_en = 0;
	}

	if (rtsx_acquire_irq(dev) < 0) {
		err = -EBUSY;
		goto errout;
	}

	pci_set_master(pci);
	synchronize_irq(dev->irq);

	rtsx_init_chip(dev->chip);

	/* set the supported max_lun and max_id for the scsi host
	 * NOTE: the minimal value of max_id is 1 */
	host->max_id = 1;
	host->max_lun = dev->chip->max_lun;

	/* Start up our control thread */
	th = kthread_run(rtsx_control_thread, dev, CR_DRIVER_NAME);
	if (IS_ERR(th)) {
		printk(KERN_ERR "Unable to start control thread\n");
		err = PTR_ERR(th);
		goto errout;
	}
	dev->ctl_thread = th;

	err = scsi_add_host(host, &pci->dev);
	if (err) {
		printk(KERN_ERR "Unable to add the scsi host\n");
		goto errout;
	}

	/* Start up the thread for delayed SCSI-device scanning */
	th = kthread_create(rtsx_scan_thread, dev, "rtsx-scan");
	if (IS_ERR(th)) {
		printk(KERN_ERR "Unable to start the device-scanning thread\n");
		complete(&dev->scanning_done);
		quiesce_and_remove_host(dev);
		err = PTR_ERR(th);
		goto errout;
	}

	wake_up_process(th);

	/* Start up the thread for polling thread */
	th = kthread_run(rtsx_polling_thread, dev, "rtsx-polling");
	if (IS_ERR(th)) {
		printk(KERN_ERR "Unable to start the device-polling thread\n");
		quiesce_and_remove_host(dev);
		err = PTR_ERR(th);
		goto errout;
	}
	dev->polling_thread = th;

	pci_set_drvdata(pci, dev);

	return 0;

	/* We come here if there are any problems */
errout:
	printk(KERN_ERR "rtsx_probe() failed\n");
	release_everything(dev);

	return err;
}


static void __devexit rtsx_remove(struct pci_dev *pci)
{
	struct rtsx_dev *dev = (struct rtsx_dev *)pci_get_drvdata(pci);

#ifdef CONFIG_DEBUG_PRINTK
	printk(KERN_INFO "rtsx_remove() called\n");
#else
	;
#endif

	quiesce_and_remove_host(dev);
	release_everything(dev);

	pci_set_drvdata(pci, NULL);
}

/* PCI IDs */
static DEFINE_PCI_DEVICE_TABLE(rtsx_ids) = {
	{ PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0x5208), PCI_CLASS_OTHERS << 16, 0xFF0000 },
	{ PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0x5209), PCI_CLASS_OTHERS << 16, 0xFF0000 },
	{ PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0x5288), PCI_CLASS_OTHERS << 16, 0xFF0000 },
	{ 0, },
};

MODULE_DEVICE_TABLE(pci, rtsx_ids);

/* pci_driver definition */
static struct pci_driver driver = {
	.name = CR_DRIVER_NAME,
	.id_table = rtsx_ids,
	.probe = rtsx_probe,
	.remove = __devexit_p(rtsx_remove),
#ifdef CONFIG_PM
	.suspend = rtsx_suspend,
	.resume = rtsx_resume,
#endif
	.shutdown = rtsx_shutdown,
};

static int __init rtsx_init(void)
{
#ifdef CONFIG_DEBUG_PRINTK
	printk(KERN_INFO "Initializing Realtek PCIE storage driver...\n");
#else
	;
#endif

	return pci_register_driver(&driver);
}

static void __exit rtsx_exit(void)
{
#ifdef CONFIG_DEBUG_PRINTK
	printk(KERN_INFO "rtsx_exit() called\n");
#else
	;
#endif

	pci_unregister_driver(&driver);

#ifdef CONFIG_DEBUG_PRINTK
	printk(KERN_INFO "%s module exit\n", CR_DRIVER_NAME);
#else
	;
#endif
}

module_init(rtsx_init)
module_exit(rtsx_exit)

