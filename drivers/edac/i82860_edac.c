/*
 * Intel 82860 Memory Controller kernel module
 * (C) 2005 Red Hat (http://www.redhat.com)
 * This file may be distributed under the terms of the
 * GNU General Public License.
 *
 * Written by Ben Woodard <woodard@redhat.com>
 * shamelessly copied from and based upon the edac_i82875 driver
 * by Thayne Harbaugh of Linux Networx. (http://lnxi.com)
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/edac.h>
#include "edac_core.h"

#define  I82860_REVISION " Ver: 2.0.2"
#define EDAC_MOD_STR	"i82860_edac"

#ifdef CONFIG_DEBUG_PRINTK
#define i82860_printk(level, fmt, arg...) \
	edac_printk(level, "i82860", fmt, ##arg)

#define i82860_mc_printk(mci, level, fmt, arg...) \
	edac_mc_chipset_printk(mci, level, "i82860", fmt, ##arg)

#ifndef PCI_DEVICE_ID_INTEL_82860_0
#define PCI_DEVICE_ID_INTEL_82860_0	0x2531
#endif				/* PCI_DEVICE_ID_INTEL_82860_0 */

#define I82860_MCHCFG 0x50
#define I82860_GBA 0x60
#define I82860_GBA_MASK 0x7FF
#define I82860_GBA_SHIFT 24
#define I82860_ERRSTS 0xC8
#define I82860_EAP 0xE4
#define I82860_DERRCTL_STS 0xE2

enum i82860_chips {
	I82860 = 0,
};
#else
#define i82860_;
#endif

struct i82860_dev_info {
	const char *ctl_name;
};

struct i82860_error_info {
	u16 errsts;
	u32 eap;
	u16 derrsyn;
	u16 errsts2;
};

static const struct i82860_dev_info i82860_devs[] = {
	[I82860] = {
		.ctl_name = "i82860"},
};

static struct pci_dev *mci_pdev;	/* init dev: in case that AGP code
					 * has already registered driver
					 */
static struct edac_pci_ctl_info *i82860_pci;

static void i82860_get_error_info(struct mem_ctl_info *mci,
				struct i82860_error_info *info)
{
	struct pci_dev *pdev;

	pdev = to_pci_dev(mci->dev);

	/*
	 * This is a mess because there is no atomic way to read all the
	 * registers at once and the registers can transition from CE being
	 * overwritten by UE.
	 */
	pci_read_config_word(pdev, I82860_ERRSTS, &info->errsts);
	pci_read_config_dword(pdev, I82860_EAP, &info->eap);
	pci_read_config_word(pdev, I82860_DERRCTL_STS, &info->derrsyn);
	pci_read_config_word(pdev, I82860_ERRSTS, &info->errsts2);

	pci_write_bits16(pdev, I82860_ERRSTS, 0x0003, 0x0003);

	/*
	 * If the error is the same for both reads then the first set of reads
	 * is valid.  If there is a change then there is a CE no info and the
	 * second set of reads is valid and should be UE info.
	 */
	if (!(info->errsts2 & 0x0003))
		return;

	if ((info->errsts ^ info->errsts2) & 0x0003) {
		pci_read_config_dword(pdev, I82860_EAP, &info->eap);
		pci_read_config_word(pdev, I82860_DERRCTL_STS, &info->derrsyn);
	}
}

static int i82860_process_error_info(struct mem_ctl_info *mci,
				struct i82860_error_info *info,
				int handle_errors)
{
	int row;

	if (!(info->errsts2 & 0x0003))
		return 0;

	if (!handle_errors)
		return 1;

	if ((info->errsts ^ info->errsts2) & 0x0003) {
		edac_mc_handle_ce_no_info(mci, "UE overwrote CE");
		info->errsts = info->errsts2;
	}

	info->eap >>= PAGE_SHIFT;
	row = edac_mc_find_csrow_by_page(mci, info->eap);

	if (info->errsts & 0x0002)
		edac_mc_handle_ue(mci, info->eap, 0, row, "i82860 UE");
	else
		edac_mc_handle_ce(mci, info->eap, 0, info->derrsyn, row, 0,
				"i82860 UE");

	return 1;
}

static void i82860_check(struct mem_ctl_info *mci)
{
	struct i82860_error_info info;

	debugf1("MC%d: %s()\n", mci->mc_idx, __func__);
	i82860_get_error_info(mci, &info);
	i82860_process_error_info(mci, &info, 1);
}

static void i82860_init_csrows(struct mem_ctl_info *mci, struct pci_dev *pdev)
{
	unsigned long last_cumul_size;
	u16 mchcfg_ddim;	/* DRAM Data Integrity Mode 0=none, 2=edac */
	u16 value;
	u32 cumul_size;
	struct csrow_info *csrow;
	int index;

	pci_read_config_word(pdev, I82860_MCHCFG, &mchcfg_ddim);
	mchcfg_ddim = mchcfg_ddim & 0x180;
	last_cumul_size = 0;

	/* The group row boundary (GRA) reg values are boundary address
	 * for each DRAM row with a granularity of 16MB.  GRA regs are
	 * cumulative; therefore GRA15 will contain the total memory contained
	 * in all eight rows.
	 */
	for (index = 0; index < mci->nr_csrows; index++) {
		csrow = &mci->csrows[index];
		pci_read_config_word(pdev, I82860_GBA + index * 2, &value);
		cumul_size = (value & I82860_GBA_MASK) <<
			(I82860_GBA_SHIFT - PAGE_SHIFT);
		debugf3("%s(): (%d) cumul_size 0x%x\n", __func__, index,
			cumul_size);

		if (cumul_size == last_cumul_size)
			continue;	/* not populated */

		csrow->first_page = last_cumul_size;
		csrow->last_page = cumul_size - 1;
		csrow->nr_pages = cumul_size - last_cumul_size;
		last_cumul_size = cumul_size;
		csrow->grain = 1 << 12;	/* I82860_EAP has 4KiB reolution */
		csrow->mtype = MEM_RMBS;
		csrow->dtype = DEV_UNKNOWN;
		csrow->edac_mode = mchcfg_ddim ? EDAC_SECDED : EDAC_NONE;
	}
}

static int i82860_probe1(struct pci_dev *pdev, int dev_idx)
{
	struct mem_ctl_info *mci;
	struct i82860_error_info discard;

	/* RDRAM has channels but these don't map onto the abstractions that
	   edac uses.
	   The device groups from the GRA registers seem to map reasonably
	   well onto the notion of a chip select row.
	   There are 16 GRA registers and since the name is associated with
	   the channel and the GRA registers map to physical devices so we are
	   going to make 1 channel for group.
	 */
	mci = edac_mc_alloc(0, 16, 1, 0);

	if (!mci)
		return -ENOMEM;

	debugf3("%s(): init mci\n", __func__);
	mci->dev = &pdev->dev;
	mci->mtype_cap = MEM_FLAG_DDR;
	mci->edac_ctl_cap = EDAC_FLAG_NONE | EDAC_FLAG_SECDED;
	/* I"m not sure about this but I think that all RDRAM is SECDED */
	mci->edac_cap = EDAC_FLAG_SECDED;
	mci->mod_name = EDAC_MOD_STR;
	mci->mod_ver = I82860_REVISION;
	mci->ctl_name = i82860_devs[dev_idx].ctl_name;
	mci->dev_name = pci_name(pdev);
	mci->edac_check = i82860_check;
	mci->ctl_page_to_phys = NULL;
	i82860_init_csrows(mci, pdev);
	i82860_get_error_info(mci, &discard);	/* clear counters */

	/* Here we assume that we will never see multiple instances of this
	 * type of memory controller.  The ID is therefore hardcoded to 0.
	 */
	if (edac_mc_add_mc(mci)) {
		debugf3("%s(): failed edac_mc_add_mc()\n", __func__);
		goto fail;
	}

	/* allocating generic PCI control info */
	i82860_pci = edac_pci_create_generic_ctl(&pdev->dev, EDAC_MOD_STR);
	if (!i82860_pci) {
#ifdef CONFIG_DEBUG_PRINTK
		printk(KERN_WARNING
			"%s(): Unable to create PCI control\n",
			__func__);
#else
		;
#endif
#ifdef CONFIG_DEBUG_PRINTK
		printk(KERN_WARNING
			"%s(): PCI error report via EDAC not setup\n",
			__func__);
#else
		;
#endif
	}

	/* get this far and it's successful */
	debugf3("%s(): success\n", __func__);

	return 0;

fail:
	edac_mc_free(mci);
	return -ENODEV;
}

/* returns count (>= 0), or negative on error */
static int __devinit i82860_init_one(struct pci_dev *pdev,
				const struct pci_device_id *ent)
{
	int rc;

	debugf0("%s()\n", __func__);
#ifdef CONFIG_DEBUG_PRINTK
	i82860_printk(KERN_INFO, "i82860 init one\n");
#else
	i82860_;
#endif

	if (pci_enable_device(pdev) < 0)
		return -EIO;

	rc = i82860_probe1(pdev, ent->driver_data);

	if (rc == 0)
		mci_pdev = pci_dev_get(pdev);

	return rc;
}

static void __devexit i82860_remove_one(struct pci_dev *pdev)
{
	struct mem_ctl_info *mci;

	debugf0("%s()\n", __func__);

	if (i82860_pci)
		edac_pci_release_generic_ctl(i82860_pci);

	if ((mci = edac_mc_del_mc(&pdev->dev)) == NULL)
		return;

	edac_mc_free(mci);
}

static DEFINE_PCI_DEVICE_TABLE(i82860_pci_tbl) = {
	{
	 PCI_VEND_DEV(INTEL, 82860_0), PCI_ANY_ID, PCI_ANY_ID, 0, 0,
	 I82860},
	{
	 0,
	 }			/* 0 terminated list. */
};

MODULE_DEVICE_TABLE(pci, i82860_pci_tbl);

static struct pci_driver i82860_driver = {
	.name = EDAC_MOD_STR,
	.probe = i82860_init_one,
	.remove = __devexit_p(i82860_remove_one),
	.id_table = i82860_pci_tbl,
};

static int __init i82860_init(void)
{
	int pci_rc;

	debugf3("%s()\n", __func__);

       /* Ensure that the OPSTATE is set correctly for POLL or NMI */
       opstate_init();

	if ((pci_rc = pci_register_driver(&i82860_driver)) < 0)
		goto fail0;

	if (!mci_pdev) {
		mci_pdev = pci_get_device(PCI_VENDOR_ID_INTEL,
					PCI_DEVICE_ID_INTEL_82860_0, NULL);

		if (mci_pdev == NULL) {
			debugf0("860 pci_get_device fail\n");
			pci_rc = -ENODEV;
			goto fail1;
		}

		pci_rc = i82860_init_one(mci_pdev, i82860_pci_tbl);

		if (pci_rc < 0) {
			debugf0("860 init fail\n");
			pci_rc = -ENODEV;
			goto fail1;
		}
	}

	return 0;

fail1:
	pci_unregister_driver(&i82860_driver);

fail0:
	if (mci_pdev != NULL)
		pci_dev_put(mci_pdev);

	return pci_rc;
}

static void __exit i82860_exit(void)
{
	debugf3("%s()\n", __func__);

	pci_unregister_driver(&i82860_driver);

	if (mci_pdev != NULL)
		pci_dev_put(mci_pdev);
}

module_init(i82860_init);
module_exit(i82860_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Red Hat Inc. (http://www.redhat.com) "
		"Ben Woodard <woodard@redhat.com>");
MODULE_DESCRIPTION("ECC support for Intel 82860 memory hub controllers");

module_param(edac_op_state, int, 0444);
MODULE_PARM_DESC(edac_op_state, "EDAC Error Reporting state: 0=Poll,1=NMI");
