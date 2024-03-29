/*
 *   Fujitu mb86a20s ISDB-T/ISDB-Tsb Module driver
 *
 *   Copyright (C) 2010 Mauro Carvalho Chehab <mchehab@redhat.com>
 *   Copyright (C) 2009-2010 Douglas Landgraf <dougsland@redhat.com>
 *
 *   FIXME: Need to port to DVB v5.2 API
 *
 *   This program is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public License as
 *   published by the Free Software Foundation version 2.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   General Public License for more details.
 */

#include <linux/kernel.h>
#include <asm/div64.h>

#include "dvb_frontend.h"
#include "mb86a20s.h"

static int debug = 1;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Activates frontend debugging (default:0)");

#define rc(args...)  do {						\
	printk(KERN_ERR  "mb86a20s: " args);				\
} while (0)

#ifdef CONFIG_DEBUG_PRINTK
#define dprintk(args...)						\
	do {								\
		if (debug) {						\
			printk(KERN_DEBUG "mb86a20s: %s: ", __func__);	\
#else
#define d;
#endif
#ifdef CONFIG_DEBUG_PRINTK
			printk(args);					\
#else
			;
#endif
		}							\
	} while (0)

struct mb86a20s_state {
	struct i2c_adapter *i2c;
	const struct mb86a20s_config *config;

	struct dvb_frontend frontend;

	bool need_init;
};

struct regdata {
	u8 reg;
	u8 data;
};

/*
 * Initialization sequence: Use whatevere default values that PV SBTVD
 * does on its initialisation, obtained via USB snoop
 */
static struct regdata mb86a20s_init[] = {
	{ 0x70, 0x0f },
	{ 0x70, 0xff },
	{ 0x08, 0x01 },
	{ 0x09, 0x3e },
	{ 0x50, 0xd1 }, { 0x51, 0x22 },
	{ 0x39, 0x01 },
	{ 0x71, 0x00 },
	{ 0x28, 0x2a }, { 0x29, 0x00 }, { 0x2a, 0xff }, { 0x2b, 0x80 },
	{ 0x28, 0x20 }, { 0x29, 0x33 }, { 0x2a, 0xdf }, { 0x2b, 0xa9 },
	{ 0x28, 0x22 }, { 0x29, 0x00 }, { 0x2a, 0x1f }, { 0x2b, 0xf0 },
	{ 0x3b, 0x21 },
	{ 0x3c, 0x3a },
	{ 0x01, 0x0d },
	{ 0x04, 0x08 }, { 0x05, 0x05 },
	{ 0x04, 0x0e }, { 0x05, 0x00 },
	{ 0x04, 0x0f }, { 0x05, 0x14 },
	{ 0x04, 0x0b }, { 0x05, 0x8c },
	{ 0x04, 0x00 }, { 0x05, 0x00 },
	{ 0x04, 0x01 }, { 0x05, 0x07 },
	{ 0x04, 0x02 }, { 0x05, 0x0f },
	{ 0x04, 0x03 }, { 0x05, 0xa0 },
	{ 0x04, 0x09 }, { 0x05, 0x00 },
	{ 0x04, 0x0a }, { 0x05, 0xff },
	{ 0x04, 0x27 }, { 0x05, 0x64 },
	{ 0x04, 0x28 }, { 0x05, 0x00 },
	{ 0x04, 0x1e }, { 0x05, 0xff },
	{ 0x04, 0x29 }, { 0x05, 0x0a },
	{ 0x04, 0x32 }, { 0x05, 0x0a },
	{ 0x04, 0x14 }, { 0x05, 0x02 },
	{ 0x04, 0x04 }, { 0x05, 0x00 },
	{ 0x04, 0x05 }, { 0x05, 0x22 },
	{ 0x04, 0x06 }, { 0x05, 0x0e },
	{ 0x04, 0x07 }, { 0x05, 0xd8 },
	{ 0x04, 0x12 }, { 0x05, 0x00 },
	{ 0x04, 0x13 }, { 0x05, 0xff },
	{ 0x04, 0x15 }, { 0x05, 0x4e },
	{ 0x04, 0x16 }, { 0x05, 0x20 },
	{ 0x52, 0x01 },
	{ 0x50, 0xa7 }, { 0x51, 0xff },
	{ 0x50, 0xa8 }, { 0x51, 0xff },
	{ 0x50, 0xa9 }, { 0x51, 0xff },
	{ 0x50, 0xaa }, { 0x51, 0xff },
	{ 0x50, 0xab }, { 0x51, 0xff },
	{ 0x50, 0xac }, { 0x51, 0xff },
	{ 0x50, 0xad }, { 0x51, 0xff },
	{ 0x50, 0xae }, { 0x51, 0xff },
	{ 0x50, 0xaf }, { 0x51, 0xff },
	{ 0x5e, 0x07 },
	{ 0x50, 0xdc }, { 0x51, 0x01 },
	{ 0x50, 0xdd }, { 0x51, 0xf4 },
	{ 0x50, 0xde }, { 0x51, 0x01 },
	{ 0x50, 0xdf }, { 0x51, 0xf4 },
	{ 0x50, 0xe0 }, { 0x51, 0x01 },
	{ 0x50, 0xe1 }, { 0x51, 0xf4 },
	{ 0x50, 0xb0 }, { 0x51, 0x07 },
	{ 0x50, 0xb2 }, { 0x51, 0xff },
	{ 0x50, 0xb3 }, { 0x51, 0xff },
	{ 0x50, 0xb4 }, { 0x51, 0xff },
	{ 0x50, 0xb5 }, { 0x51, 0xff },
	{ 0x50, 0xb6 }, { 0x51, 0xff },
	{ 0x50, 0xb7 }, { 0x51, 0xff },
	{ 0x50, 0x50 }, { 0x51, 0x02 },
	{ 0x50, 0x51 }, { 0x51, 0x04 },
	{ 0x45, 0x04 },
	{ 0x48, 0x04 },
	{ 0x50, 0xd5 }, { 0x51, 0x01 },		/* Serial */
	{ 0x50, 0xd6 }, { 0x51, 0x1f },
	{ 0x50, 0xd2 }, { 0x51, 0x03 },
	{ 0x50, 0xd7 }, { 0x51, 0x3f },
	{ 0x28, 0x74 }, { 0x29, 0x00 }, { 0x28, 0x74 }, { 0x29, 0x40 },
	{ 0x28, 0x46 }, { 0x29, 0x2c }, { 0x28, 0x46 }, { 0x29, 0x0c },
	{ 0x04, 0x40 }, { 0x05, 0x01 },
	{ 0x28, 0x00 }, { 0x29, 0x10 },
	{ 0x28, 0x05 }, { 0x29, 0x02 },
	{ 0x1c, 0x01 },
	{ 0x28, 0x06 }, { 0x29, 0x00 }, { 0x2a, 0x00 }, { 0x2b, 0x03 },
	{ 0x28, 0x07 }, { 0x29, 0x00 }, { 0x2a, 0x00 }, { 0x2b, 0x0d },
	{ 0x28, 0x08 }, { 0x29, 0x00 }, { 0x2a, 0x00 }, { 0x2b, 0x02 },
	{ 0x28, 0x09 }, { 0x29, 0x00 }, { 0x2a, 0x00 }, { 0x2b, 0x01 },
	{ 0x28, 0x0a }, { 0x29, 0x00 }, { 0x2a, 0x00 }, { 0x2b, 0x21 },
	{ 0x28, 0x0b }, { 0x29, 0x00 }, { 0x2a, 0x00 }, { 0x2b, 0x29 },
	{ 0x28, 0x0c }, { 0x29, 0x00 }, { 0x2a, 0x00 }, { 0x2b, 0x16 },
	{ 0x28, 0x0d }, { 0x29, 0x00 }, { 0x2a, 0x00 }, { 0x2b, 0x31 },
	{ 0x28, 0x0e }, { 0x29, 0x00 }, { 0x2a, 0x00 }, { 0x2b, 0x0e },
	{ 0x28, 0x0f }, { 0x29, 0x00 }, { 0x2a, 0x00 }, { 0x2b, 0x4e },
	{ 0x28, 0x10 }, { 0x29, 0x00 }, { 0x2a, 0x00 }, { 0x2b, 0x46 },
	{ 0x28, 0x11 }, { 0x29, 0x00 }, { 0x2a, 0x00 }, { 0x2b, 0x0f },
	{ 0x28, 0x12 }, { 0x29, 0x00 }, { 0x2a, 0x00 }, { 0x2b, 0x56 },
	{ 0x28, 0x13 }, { 0x29, 0x00 }, { 0x2a, 0x00 }, { 0x2b, 0x35 },
	{ 0x28, 0x14 }, { 0x29, 0x00 }, { 0x2a, 0x01 }, { 0x2b, 0xbe },
	{ 0x28, 0x15 }, { 0x29, 0x00 }, { 0x2a, 0x01 }, { 0x2b, 0x84 },
	{ 0x28, 0x16 }, { 0x29, 0x00 }, { 0x2a, 0x03 }, { 0x2b, 0xee },
	{ 0x28, 0x17 }, { 0x29, 0x00 }, { 0x2a, 0x00 }, { 0x2b, 0x98 },
	{ 0x28, 0x18 }, { 0x29, 0x00 }, { 0x2a, 0x00 }, { 0x2b, 0x9f },
	{ 0x28, 0x19 }, { 0x29, 0x00 }, { 0x2a, 0x07 }, { 0x2b, 0xb2 },
	{ 0x28, 0x1a }, { 0x29, 0x00 }, { 0x2a, 0x06 }, { 0x2b, 0xc2 },
	{ 0x28, 0x1b }, { 0x29, 0x00 }, { 0x2a, 0x07 }, { 0x2b, 0x4a },
	{ 0x28, 0x1c }, { 0x29, 0x00 }, { 0x2a, 0x01 }, { 0x2b, 0xbc },
	{ 0x28, 0x1d }, { 0x29, 0x00 }, { 0x2a, 0x04 }, { 0x2b, 0xba },
	{ 0x28, 0x1e }, { 0x29, 0x00 }, { 0x2a, 0x06 }, { 0x2b, 0x14 },
	{ 0x50, 0x1e }, { 0x51, 0x5d },
	{ 0x50, 0x22 }, { 0x51, 0x00 },
	{ 0x50, 0x23 }, { 0x51, 0xc8 },
	{ 0x50, 0x24 }, { 0x51, 0x00 },
	{ 0x50, 0x25 }, { 0x51, 0xf0 },
	{ 0x50, 0x26 }, { 0x51, 0x00 },
	{ 0x50, 0x27 }, { 0x51, 0xc3 },
	{ 0x50, 0x39 }, { 0x51, 0x02 },
	{ 0x28, 0x6a }, { 0x29, 0x00 }, { 0x2a, 0x00 }, { 0x2b, 0x00 },
	{ 0xd0, 0x00 },
};

static struct regdata mb86a20s_reset_reception[] = {
	{ 0x70, 0xf0 },
	{ 0x70, 0xff },
	{ 0x08, 0x01 },
	{ 0x08, 0x00 },
};

static int mb86a20s_i2c_writereg(struct mb86a20s_state *state,
			     u8 i2c_addr, int reg, int data)
{
	u8 buf[] = { reg, data };
	struct i2c_msg msg = {
		.addr = i2c_addr, .flags = 0, .buf = buf, .len = 2
	};
	int rc;

	rc = i2c_transfer(state->i2c, &msg, 1);
	if (rc != 1) {
#ifdef CONFIG_DEBUG_PRINTK
		printk("%s: writereg error (rc == %i, reg == 0x%02x,"
			 " data == 0x%02x)\n", __func__, rc, reg, data);
#else
		;
#endif
		return rc;
	}

	return 0;
}

static int mb86a20s_i2c_writeregdata(struct mb86a20s_state *state,
				     u8 i2c_addr, struct regdata *rd, int size)
{
	int i, rc;

	for (i = 0; i < size; i++) {
		rc = mb86a20s_i2c_writereg(state, i2c_addr, rd[i].reg,
					   rd[i].data);
		if (rc < 0)
			return rc;
	}
	return 0;
}

static int mb86a20s_i2c_readreg(struct mb86a20s_state *state,
				u8 i2c_addr, u8 reg)
{
	u8 val;
	int rc;
	struct i2c_msg msg[] = {
		{ .addr = i2c_addr, .flags = 0, .buf = &reg, .len = 1 },
		{ .addr = i2c_addr, .flags = I2C_M_RD, .buf = &val, .len = 1 }
	};

	rc = i2c_transfer(state->i2c, msg, 2);

	if (rc != 2) {
		rc("%s: reg=0x%x (error=%d)\n", __func__, reg, rc);
		return rc;
	}

	return val;
}

#define mb86a20s_readreg(state, reg) \
	mb86a20s_i2c_readreg(state, state->config->demod_address, reg)
#define mb86a20s_writereg(state, reg, val) \
	mb86a20s_i2c_writereg(state, state->config->demod_address, reg, val)
#define mb86a20s_writeregdata(state, regdata) \
	mb86a20s_i2c_writeregdata(state, state->config->demod_address, \
	regdata, ARRAY_SIZE(regdata))

static int mb86a20s_initfe(struct dvb_frontend *fe)
{
	struct mb86a20s_state *state = fe->demodulator_priv;
	int rc;
	u8  regD5 = 1;

#ifdef CONFIG_DEBUG_PRINTK
	dprintk("\n");
#else
	d;
#endif

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);

	/* Initialize the frontend */
	rc = mb86a20s_writeregdata(state, mb86a20s_init);
	if (rc < 0)
		goto err;

	if (!state->config->is_serial) {
		regD5 &= ~1;

		rc = mb86a20s_writereg(state, 0x50, 0xd5);
		if (rc < 0)
			goto err;
		rc = mb86a20s_writereg(state, 0x51, regD5);
		if (rc < 0)
			goto err;
	}

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);

err:
	if (rc < 0) {
		state->need_init = true;
#ifdef CONFIG_DEBUG_PRINTK
		printk(KERN_INFO "mb86a20s: Init failed. Will try again later\n");
#else
		;
#endif
	} else {
		state->need_init = false;
#ifdef CONFIG_DEBUG_PRINTK
		dprintk("Initialization succeeded.\n");
#else
		d;
#endif
	}
	return rc;
}

static int mb86a20s_read_signal_strength(struct dvb_frontend *fe, u16 *strength)
{
	struct mb86a20s_state *state = fe->demodulator_priv;
	unsigned rf_max, rf_min, rf;
	u8	 val;

#ifdef CONFIG_DEBUG_PRINTK
	dprintk("\n");
#else
	d;
#endif

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);

	/* Does a binary search to get RF strength */
	rf_max = 0xfff;
	rf_min = 0;
	do {
		rf = (rf_max + rf_min) / 2;
		mb86a20s_writereg(state, 0x04, 0x1f);
		mb86a20s_writereg(state, 0x05, rf >> 8);
		mb86a20s_writereg(state, 0x04, 0x20);
		mb86a20s_writereg(state, 0x04, rf);

		val = mb86a20s_readreg(state, 0x02);
		if (val & 0x08)
			rf_min = (rf_max + rf_min) / 2;
		else
			rf_max = (rf_max + rf_min) / 2;
		if (rf_max - rf_min < 4) {
			*strength = (((rf_max + rf_min) / 2) * 65535) / 4095;
			break;
		}
	} while (1);

#ifdef CONFIG_DEBUG_PRINTK
	dprintk("signal strength = %d\n", *strength);
#else
	d;
#endif

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);

	return 0;
}

static int mb86a20s_read_status(struct dvb_frontend *fe, fe_status_t *status)
{
	struct mb86a20s_state *state = fe->demodulator_priv;
	u8 val;

#ifdef CONFIG_DEBUG_PRINTK
	dprintk("\n");
#else
	d;
#endif
	*status = 0;

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);
	val = mb86a20s_readreg(state, 0x0a) & 0xf;
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);

	if (val >= 2)
		*status |= FE_HAS_SIGNAL;

	if (val >= 4)
		*status |= FE_HAS_CARRIER;

	if (val >= 5)
		*status |= FE_HAS_VITERBI;

	if (val >= 7)
		*status |= FE_HAS_SYNC;

	if (val >= 8)				/* Maybe 9? */
		*status |= FE_HAS_LOCK;

#ifdef CONFIG_DEBUG_PRINTK
	dprintk("val = %d, status = 0x%02x\n", val, *status);
#else
	d;
#endif

	return 0;
}

static int mb86a20s_set_frontend(struct dvb_frontend *fe)
{
	struct mb86a20s_state *state = fe->demodulator_priv;
	int rc;
#if 0
	/*
	 * FIXME: Properly implement the set frontend properties
	 */
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
#endif

#ifdef CONFIG_DEBUG_PRINTK
	dprintk("\n");
#else
	d;
#endif

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);
	dprintk("Calling tuner set parameters\n");
	fe->ops.tuner_ops.set_params(fe);

	/*
	 * Make it more reliable: if, for some reason, the initial
	 * device initialization doesn't happen, initialize it when
	 * a SBTVD parameters are adjusted.
	 *
	 * Unfortunately, due to a hard to track bug at tda829x/tda18271,
	 * the agc callback logic is not called during DVB attach time,
	 * causing mb86a20s to not be initialized with Kworld SBTVD.
	 * So, this hack is needed, in order to make Kworld SBTVD to work.
	 */
	if (state->need_init)
		mb86a20s_initfe(fe);

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);
	rc = mb86a20s_writeregdata(state, mb86a20s_reset_reception);
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);

	return rc;
}

static int mb86a20s_get_modulation(struct mb86a20s_state *state,
				   unsigned layer)
{
	int rc;
	static unsigned char reg[] = {
		[0] = 0x86,	/* Layer A */
		[1] = 0x8a,	/* Layer B */
		[2] = 0x8e,	/* Layer C */
	};

	if (layer >= ARRAY_SIZE(reg))
		return -EINVAL;
	rc = mb86a20s_writereg(state, 0x6d, reg[layer]);
	if (rc < 0)
		return rc;
	rc = mb86a20s_readreg(state, 0x6e);
	if (rc < 0)
		return rc;
	switch ((rc & 0x70) >> 4) {
	case 0:
		return DQPSK;
	case 1:
		return QPSK;
	case 2:
		return QAM_16;
	case 3:
		return QAM_64;
	default:
		return QAM_AUTO;
	}
}

static int mb86a20s_get_fec(struct mb86a20s_state *state,
			    unsigned layer)
{
	int rc;

	static unsigned char reg[] = {
		[0] = 0x87,	/* Layer A */
		[1] = 0x8b,	/* Layer B */
		[2] = 0x8f,	/* Layer C */
	};

	if (layer >= ARRAY_SIZE(reg))
		return -EINVAL;
	rc = mb86a20s_writereg(state, 0x6d, reg[layer]);
	if (rc < 0)
		return rc;
	rc = mb86a20s_readreg(state, 0x6e);
	if (rc < 0)
		return rc;
	switch (rc) {
	case 0:
		return FEC_1_2;
	case 1:
		return FEC_2_3;
	case 2:
		return FEC_3_4;
	case 3:
		return FEC_5_6;
	case 4:
		return FEC_7_8;
	default:
		return FEC_AUTO;
	}
}

static int mb86a20s_get_interleaving(struct mb86a20s_state *state,
				     unsigned layer)
{
	int rc;

	static unsigned char reg[] = {
		[0] = 0x88,	/* Layer A */
		[1] = 0x8c,	/* Layer B */
		[2] = 0x90,	/* Layer C */
	};

	if (layer >= ARRAY_SIZE(reg))
		return -EINVAL;
	rc = mb86a20s_writereg(state, 0x6d, reg[layer]);
	if (rc < 0)
		return rc;
	rc = mb86a20s_readreg(state, 0x6e);
	if (rc < 0)
		return rc;
	if (rc > 3)
		return -EINVAL;	/* Not used */
	return rc;
}

static int mb86a20s_get_segment_count(struct mb86a20s_state *state,
				      unsigned layer)
{
	int rc, count;

	static unsigned char reg[] = {
		[0] = 0x89,	/* Layer A */
		[1] = 0x8d,	/* Layer B */
		[2] = 0x91,	/* Layer C */
	};

	if (layer >= ARRAY_SIZE(reg))
		return -EINVAL;
	rc = mb86a20s_writereg(state, 0x6d, reg[layer]);
	if (rc < 0)
		return rc;
	rc = mb86a20s_readreg(state, 0x6e);
	if (rc < 0)
		return rc;
	count = (rc >> 4) & 0x0f;

	return count;
}

static int mb86a20s_get_frontend(struct dvb_frontend *fe)
{
	struct mb86a20s_state *state = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	int i, rc;

	/* Fixed parameters */
	p->delivery_system = SYS_ISDBT;
	p->bandwidth_hz = 6000000;

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);

	/* Check for partial reception */
	rc = mb86a20s_writereg(state, 0x6d, 0x85);
	if (rc >= 0)
		rc = mb86a20s_readreg(state, 0x6e);
	if (rc >= 0)
		p->isdbt_partial_reception = (rc & 0x10) ? 1 : 0;

	/* Get per-layer data */
	p->isdbt_layer_enabled = 0;
	for (i = 0; i < 3; i++) {
		rc = mb86a20s_get_segment_count(state, i);
			if (rc >= 0 && rc < 14)
				p->layer[i].segment_count = rc;
		if (rc == 0x0f)
			continue;
		p->isdbt_layer_enabled |= 1 << i;
		rc = mb86a20s_get_modulation(state, i);
			if (rc >= 0)
				p->layer[i].modulation = rc;
		rc = mb86a20s_get_fec(state, i);
			if (rc >= 0)
				p->layer[i].fec = rc;
		rc = mb86a20s_get_interleaving(state, i);
			if (rc >= 0)
				p->layer[i].interleaving = rc;
	}

	p->isdbt_sb_mode = 0;
	rc = mb86a20s_writereg(state, 0x6d, 0x84);
	if ((rc >= 0) && ((rc & 0x60) == 0x20)) {
		p->isdbt_sb_mode = 1;
		/* At least, one segment should exist */
		if (!p->isdbt_sb_segment_count)
			p->isdbt_sb_segment_count = 1;
	} else
		p->isdbt_sb_segment_count = 0;

	/* Get transmission mode and guard interval */
	p->transmission_mode = TRANSMISSION_MODE_AUTO;
	p->guard_interval = GUARD_INTERVAL_AUTO;
	rc = mb86a20s_readreg(state, 0x07);
	if (rc >= 0) {
		if ((rc & 0x60) == 0x20) {
			switch (rc & 0x0c >> 2) {
			case 0:
				p->transmission_mode = TRANSMISSION_MODE_2K;
				break;
			case 1:
				p->transmission_mode = TRANSMISSION_MODE_4K;
				break;
			case 2:
				p->transmission_mode = TRANSMISSION_MODE_8K;
				break;
			}
		}
		if (!(rc & 0x10)) {
			switch (rc & 0x3) {
			case 0:
				p->guard_interval = GUARD_INTERVAL_1_4;
				break;
			case 1:
				p->guard_interval = GUARD_INTERVAL_1_8;
				break;
			case 2:
				p->guard_interval = GUARD_INTERVAL_1_16;
				break;
			}
		}
	}

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);

	return 0;
}

static int mb86a20s_tune(struct dvb_frontend *fe,
			bool re_tune,
			unsigned int mode_flags,
			unsigned int *delay,
			fe_status_t *status)
{
	int rc = 0;

	dprintk("\n");

	if (re_tune)
		rc = mb86a20s_set_frontend(fe);

	if (!(mode_flags & FE_TUNE_MODE_ONESHOT))
		mb86a20s_read_status(fe, status);

	return rc;
}

static void mb86a20s_release(struct dvb_frontend *fe)
{
	struct mb86a20s_state *state = fe->demodulator_priv;

#ifdef CONFIG_DEBUG_PRINTK
	dprintk("\n");
#else
	d;
#endif

	kfree(state);
}

static struct dvb_frontend_ops mb86a20s_ops;

struct dvb_frontend *mb86a20s_attach(const struct mb86a20s_config *config,
				    struct i2c_adapter *i2c)
{
	u8	rev;

	/* allocate memory for the internal state */
	struct mb86a20s_state *state =
		kzalloc(sizeof(struct mb86a20s_state), GFP_KERNEL);

#ifdef CONFIG_DEBUG_PRINTK
	dprintk("\n");
#else
	d;
#endif
	if (state == NULL) {
		rc("Unable to kzalloc\n");
		goto error;
	}

	/* setup the state */
	state->config = config;
	state->i2c = i2c;

	/* create dvb_frontend */
	memcpy(&state->frontend.ops, &mb86a20s_ops,
		sizeof(struct dvb_frontend_ops));
	state->frontend.demodulator_priv = state;

	/* Check if it is a mb86a20s frontend */
	rev = mb86a20s_readreg(state, 0);

	if (rev == 0x13) {
#ifdef CONFIG_DEBUG_PRINTK
		printk(KERN_INFO "Detected a Fujitsu mb86a20s frontend\n");
#else
		;
#endif
	} else {
		printk(KERN_ERR "Frontend revision %d is unknown - aborting.\n",
		       rev);
		goto error;
	}

	return &state->frontend;

error:
	kfree(state);
	return NULL;
}
EXPORT_SYMBOL(mb86a20s_attach);

static struct dvb_frontend_ops mb86a20s_ops = {
	.delsys = { SYS_ISDBT },
	/* Use dib8000 values per default */
	.info = {
		.name = "Fujitsu mb86A20s",
		.caps = FE_CAN_INVERSION_AUTO | FE_CAN_RECOVER |
			FE_CAN_FEC_1_2  | FE_CAN_FEC_2_3 | FE_CAN_FEC_3_4 |
			FE_CAN_FEC_5_6  | FE_CAN_FEC_7_8 | FE_CAN_FEC_AUTO |
			FE_CAN_QPSK     | FE_CAN_QAM_16  | FE_CAN_QAM_64 |
			FE_CAN_TRANSMISSION_MODE_AUTO | FE_CAN_QAM_AUTO |
			FE_CAN_GUARD_INTERVAL_AUTO    | FE_CAN_HIERARCHY_AUTO,
		/* Actually, those values depend on the used tuner */
		.frequency_min = 45000000,
		.frequency_max = 864000000,
		.frequency_stepsize = 62500,
	},

	.release = mb86a20s_release,

	.init = mb86a20s_initfe,
	.set_frontend = mb86a20s_set_frontend,
	.get_frontend = mb86a20s_get_frontend,
	.read_status = mb86a20s_read_status,
	.read_signal_strength = mb86a20s_read_signal_strength,
	.tune = mb86a20s_tune,
};

MODULE_DESCRIPTION("DVB Frontend module for Fujitsu mb86A20s hardware");
MODULE_AUTHOR("Mauro Carvalho Chehab <mchehab@redhat.com>");
MODULE_LICENSE("GPL");
