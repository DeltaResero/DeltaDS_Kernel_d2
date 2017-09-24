/* tiered_gpu_governor.c
 * Dummy parameter wrapper for tiered GPU governor.
 */

#include <linux/kernel.h>
#include <linux/module.h>

extern int tiered_gpu_active;
extern void tiered_gpu_activate(void);

static int tiered_set_activate(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_int(val, kp);
	tiered_gpu_active = !!tiered_gpu_active;
	tiered_gpu_activate();
	return ret;
}

static const struct kernel_param_ops activate_ops = {
	.set = tiered_set_activate,
	.get = param_get_int,
};

module_param_cb(tiered_gpu_activate, &activate_ops, &tiered_gpu_active, 0644);
MODULE_PARM_DESC(tiered_gpu_activate, "Enable tiered GPU governor");
