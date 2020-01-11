/*
 * EBI2 access for LCD controller on MDM 9x35
 *
 * Copyright (c) 2013, 2014 Netgear
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

/*
 * MDM9x35 supports 9-bit bus to LCD controller when using DMA
 * This code here manipulates data in a way that LCD controller
 * when connected over 8-bit bus will understand them.
 *
 * works for RGB565
 *
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/of_address.h>
#include <video/ntgr-lcd_interface.h>
#include <linux/err.h>

#include <asm/mach-types.h>
#include <linux/msm-sps.h>
#include <linux/dma-mapping.h>
#include <linux/msm-bus.h>


/* register offsets */
#define QPIC_LCDC_CTRL			0x22000
#define QPIC_LCDC_STTS			0x22014
#define QPIC_LCDC_CMD_DATA_CYCLE_CNT	0x22018
#define QPIC_LCDC_CMD_0	 		0x23000
#define QPIC_LCDC_CFG0			0x22020
#define QPIC_LCDC_CFG1			0x22024
#define QPIC_LCDC_CFG2			0x22028
#define QPIC_LCDC_RESET			0x2202C
#define QPIC_LCDC_DEVICE_RESET		0x22030

#define QPIC_LCDC_FIFO_SOF		0x22100
#define QPIC_LCDC_FIFO_DATA		0x22140
#define QPIC_LCDC_FIFO_EOF		0x22180


#define DMA_ADDR_INVALID	(~(dma_addr_t)0)

struct qpic_sps_endpt {
	 struct sps_pipe *handle;
	 struct sps_connect config;
	 struct sps_register_event bam_event;
	 struct completion completion;
};

struct ebi2_lcd {
	void * lcd_ifb;
	struct platform_device * lcd;
	struct bus_access acc;

	u32 sps_init;
	struct qpic_sps_endpt sps;

	void * virt_addr;
	u32    maxlen;
	struct clk *qpic_clk;
	u32 bus_handle;
	int bus_allocated;
};

static int ebi2_init_bam(struct ebi2_lcd * thiz);
static void ebi2_write_cmd_generic_pio( struct ebi2_lcd * thiz, 
			u8 cmd, void *params, u32 paramlen );

static void ebi2_clk_vote( struct ebi2_lcd * thiz, int enable)
{
	int ret = 0;

	if (enable)
	{
		/* Vote to enable the clock, */
		clk_prepare_enable(thiz->qpic_clk);
		if( thiz->bus_handle )
			ret = msm_bus_scale_client_update_request( thiz->bus_handle, 1 );
	}
	else
	{
		/* Release clock vote. */
		clk_disable_unprepare(thiz->qpic_clk);
		if( thiz->bus_handle )
			ret = msm_bus_scale_client_update_request( thiz->bus_handle, 0 );
	}
	if( ret )
		dev_err( &thiz->lcd->dev, "bus scale request failed" );
}


static inline u32 qpic_readl( struct ebi2_lcd * thiz, u32 off )
{
	u8 * base = (u8 *)thiz->lcd_ifb;
	return __raw_readl( base + off );
}
static inline void qpic_writel( struct ebi2_lcd * thiz, u32 off, u32 v )
{
	u8 * base = (u8 *)thiz->lcd_ifb;
	__raw_writel( v, base + off );
}

/*
 * This function takes 16 bit data and converts them into a u32 value
 * that can be fed to the silicon driving EBI2 communication with
 * LCD controller.
 *
 * This is necessary because we have 8 bit bus connected, however the
 * controller silicon thingy works with 9 bit bus only.
 *
 * Also the thingy takes apart the data as RGB565 and sends them as 2 write
 * transactions over 9 bit bus - converting data to RGB666
 *
 * We will prepare RGB666 data in a way that it will create RGB565 on 8 bit bus!
 */
static u32 convert_data( u16 v )
{
	u32 result;
	u32 r, g, b;
	u32 gh, gl;

	r = (v >> 11) & 0x1F; /* 5 bits */
	g = (v >> 5)  & 0x3F; /* 6 bits */
	b = (v >> 0 ) & 0x1F; /* 5 bits */

	/*
	 * split g to 3 MSB bits and 3 LSB bits
	 * 3 MSB bits are in first 9bit write,
	 */
	gh = g & 0x38;
	gl = g & 0x07;

	/* First write transaction value
	 * Most significant bit is not connected!
	 */
	result	= (r << 18) | (gh << 10) ;

	/* Most significant bit is not connected */
	result |= (gl << 9) | (b << 2 );
	/* put least significant g bit to b top bit */
	result |= (gl & 1) ? 0x80 : 0x00;

	return result;
}

static void ebi2_write_cmd_generic_bam( struct ebi2_lcd * thiz, u8 cmd, void *params, u32 paramlen )
{
	/* paramlen is in bytes, we convert 16 bits -> 32 bits */
	u32 dmalen = paramlen * sizeof(u32) / sizeof(u16);
	u32 * dp = thiz->virt_addr;
	u16 * wp = params;
	u16 * ep = wp + (paramlen / sizeof(*wp));
	u32 cfg2;
	int	 err;
	int	ret;
	dma_addr_t phys;

	if( dmalen > thiz->maxlen )
	{
		dev_err( &thiz->lcd->dev,
			"%s - Too long request. Max: %d", 
			__FUNCTION__, thiz->maxlen );
		return;
	}

	/* Late Init here */
	if( !thiz->sps_init ) {
		if( ebi2_init_bam( thiz ) ) {
			/* NAND not initilized yet */
			ebi2_write_cmd_generic_pio( thiz, cmd, 
						    params, paramlen );
			return;
		}
		thiz->sps_init = 1;
	}
	
	/*
	* This is for RGB 32 bits (2 << 8) in CFG2, see enable_lcdc
	*/
	while( wp < ep )
	{
		*dp = convert_data( *wp );
		dp++;
		wp++;
	}

	cfg2 = qpic_readl( thiz, QPIC_LCDC_CFG2 );
	cfg2 &= ~(0xFF); /* remove old command */
	cfg2 |= cmd;
	qpic_writel( thiz, QPIC_LCDC_CFG2, cfg2 );

	phys = dma_map_single( &thiz->lcd->dev, thiz->virt_addr, dmalen, DMA_TO_DEVICE );

	if( phys == DMA_ADDR_INVALID ) {
		dev_err( &thiz->lcd->dev, "%s - Failed to map buffer for DMA", __FUNCTION__ );
		return;
	}

	err = sps_transfer_one( thiz->sps.handle,
				phys,
				dmalen,
				NULL,
				SPS_IOVEC_FLAG_EOT );
	if( !err ) {
		ret = wait_for_completion_interruptible_timeout( 
				&thiz->sps.completion, 
				msecs_to_jiffies(400) );

		if( ret <= 0 ) {
			dev_err( &thiz->lcd->dev, 
				"%s - Failed completion wait %d", 
				__FUNCTION__, ret );
		}
	}

	dma_unmap_single( &thiz->lcd->dev, phys, dmalen, DMA_TO_DEVICE );
}

/* Wait for FIFO to be under a certain level of fullness */
static inline void ebi2_fifo_wait( struct ebi2_lcd * thiz )
{
	u32 v = qpic_readl( thiz, QPIC_LCDC_STTS );

	while( (v & 0x3F) > 8 ) {
		usleep_range(5,20);
		v = qpic_readl( thiz, QPIC_LCDC_STTS );
	}
}

static void ebi2_write_cmd_generic_pio( struct ebi2_lcd * thiz, u8 cmd, void *params, u32 paramlen )
{
	u8	* bp;
	u16 * wp = params;
	u32 cfg2;
	int i = 0;

	cfg2 = qpic_readl( thiz, QPIC_LCDC_CFG2 );
	cfg2 &= ~(0xFF); /* remove command */

	cfg2 |= cmd;
	cfg2 |= (1 << 24); /* transparent mode - undocumented */
	qpic_writel( thiz, QPIC_LCDC_CFG2, cfg2 );

	/* Start Of Frame */
	qpic_writel( thiz, QPIC_LCDC_FIFO_SOF, 0 );

	while( paramlen >= sizeof(*wp) ) {
		const u16 word = *wp;

		ebi2_fifo_wait( thiz );

		qpic_writel( thiz, QPIC_LCDC_FIFO_DATA, word >> 8 );
		qpic_writel( thiz, QPIC_LCDC_FIFO_DATA, word );

		paramlen -= sizeof(word);
		wp++;
	}

	bp = (u8 *)wp;
	while( paramlen > 0 )
	{
		const u8 val = *bp;

		ebi2_fifo_wait( thiz );
		qpic_writel( thiz, QPIC_LCDC_FIFO_DATA, val );

		paramlen -= sizeof(*bp);
		bp++;
		i++;
	}

	/* End Of Frame */
	qpic_writel( thiz, QPIC_LCDC_FIFO_EOF, 0 );

	cfg2 = qpic_readl( thiz, QPIC_LCDC_CFG2 );
	cfg2 &= ~(1 << 24);	/* transparent mode off */
	qpic_writel( thiz, QPIC_LCDC_CFG2, cfg2 );
}
/* flash function */
static void ebi2_flush( void * aobj )
{
	struct ebi2_lcd * thiz = aobj;

	/*
	 * Release bus allocation.
	 */
	if( thiz->bus_allocated ) {
		thiz->bus_allocated = 0;
		ebi2_clk_vote( thiz, 0 );
	}
}

void ebi2_read( void * aobj, u8 cmd, void * dataout, size_t datalen )
{
	memset( dataout, 0, datalen );
}

/* send command to LCDC with specified data */
static void ebi2_write_cmd( void * aobj, u8 cmd, void *params, u32 paramlen )
{
	u32	paramval;
	struct ebi2_lcd * thiz = aobj;

	if( !thiz->bus_allocated ) {
		thiz->bus_allocated = 1;
		ebi2_clk_vote(thiz, 1);
	}

	switch( paramlen )
	{
	case 0: /* parameterless command */
		paramval = 0;
		break;
	case sizeof(u8): /* one byte parameter */
		paramval = *((u8 *)params);
		break;
	case sizeof(u16): /* word parameter */
		paramval = *((u16 *)params);
		break;
	default:
		if( (paramlen % 2) != 0 )
			ebi2_write_cmd_generic_pio( thiz, cmd, params, 
							paramlen );
		else
			ebi2_write_cmd_generic_bam( thiz, cmd, params,
							paramlen );
		return;
	}
	/*
	 * Order on bus:
	 * CMD0-7 PAR0-7 PAR8-15 PAR16-23 PAR24-31
	 */
	qpic_writel( thiz, QPIC_LCDC_CMD_DATA_CYCLE_CNT, paramlen );
	qpic_writel( thiz, QPIC_LCDC_CMD_0 + cmd * 4,	 paramval );
}

/* set min write cycle length */
static void ebi2_set_speed( void * aobj, long min_wr_cycle_ns )
{
	struct ebi2_lcd * thiz = aobj;
	int ticks;
	int wr_hold;
	int wr_active;
	const int base_per	= 10; /* in ns - based on 100Mhz clock */

	ebi2_clk_vote(thiz, 1);

	/* calc number of tick rounded up */
	ticks = (min_wr_cycle_ns + base_per - 1) / base_per;

	/* one clock is added by default from CS_WR_RD_SETUP(0) */
	ticks--;

	wr_hold = ticks / 2;
	wr_active = ticks - wr_hold;

	qpic_writel( thiz, QPIC_LCDC_CFG0, 
			((1)	 << 25) /* ADDR_CS_SETUP */
			| ((wr_active-1) << 20)
			| ((wr_hold-1)	 << 15)
			| ((0)		 << 10) /* CS_WR_RD_SETUP */
			| ((3)		 <<  5) /* RD_ACTIVE */
			| ((5)		 <<  0) /* RD_CS_HOLD */
		);

	dev_dbg( NULL, "%s - set h:%d, a:%d for write cycle: %ld",
		 __FUNCTION__, wr_hold, wr_active, min_wr_cycle_ns );

	qpic_writel( thiz, QPIC_LCDC_CFG1, 0 );
	ebi2_clk_vote(thiz, 0);
}

static void inline ebi2_enable_lcdc( struct ebi2_lcd * thiz )
{
	u32 v;

	/* LCDC_CTRL */
	v = qpic_readl( thiz, QPIC_LCDC_CTRL );
	v &= 0x00ffffff;
	v |= (1<<8);	/* LCD Enable */
	v |= (1<<1); /* BAM_MODE */
	qpic_writel( thiz, QPIC_LCDC_CTRL, v );

	dev_dbg( NULL, "%s: [%x] = %x\n", __FUNCTION__,
			QPIC_LCDC_CTRL, v );


	/* LCDC_CFG2 */
	dev_dbg( NULL, "%s: Before [%x] = %x\n", __FUNCTION__,
		QPIC_LCDC_CFG2, qpic_readl( thiz, QPIC_LCDC_CFG2 ) );

	v = qpic_readl( thiz, QPIC_LCDC_CFG2 );
	v &= 0xFFFC0000;

	v |= 0x2C; /* default command, we will change it when needed */
	/*
	 * Set to RGB32 - it will take 32 bits of RGB per pixel and
	 * send out 2 writes - each 9 bits.
	 *
	 * R7-R0 is << 16
	 * G7-G0 is <<  8
	 * B7-B0 is <<  0
	 */
	v |= (2 << 8); /* RGB */

	/* This doesn't seem to make any difference! */
	v |= (0 << 16);

	/*
	 * 9 bit interface - when set to 0 it is supposed to select 
	 *                   8 bit interface
	 * Note: Once 0 there is never any data on bus - command stays 
	 * for whole transaction cycle!
	 */
	v |= (1 << 12);
	v |= (1 << 17); /* rdx polarity */
	v |= (1 << 18); /* wrx polarity */

	qpic_writel( thiz, QPIC_LCDC_CFG2, v );

	dev_dbg( NULL, "%s: After [%x] = %x\n", __FUNCTION__,
			QPIC_LCDC_CFG2, v );
}

int ebi2_remove( struct bus_access * acc )
{
	struct ebi2_lcd * lcd = container_of( acc, struct ebi2_lcd, acc);

	dev_dbg( &lcd->lcd->dev, "%s", __FUNCTION__ );

	sps_disconnect( lcd->sps.handle );
	sps_free_endpoint( lcd->sps.handle );

	dmam_free_coherent( &lcd->lcd->dev,
			lcd->sps.config.desc.size,
			lcd->sps.config.desc.base,
			lcd->sps.config.desc.phys_base
			);
	iounmap( lcd->lcd_ifb );
	kfree( lcd->virt_addr );
	kfree( lcd );

	return 0;
}
EXPORT_SYMBOL_GPL(ebi2_remove);

static int ebi2_init_bam(struct ebi2_lcd * thiz)
{
	int rc = 0;
	struct sps_pipe *pipe_handle;
	struct sps_connect *sps_config = &thiz->sps.config;
	struct sps_register_event *sps_event = &thiz->sps.bam_event;
	struct sps_bam_props bam = {0};
	unsigned long bam_handle = 0;

	dma_set_coherent_mask( &thiz->lcd->dev, DMA_BIT_MASK(32) );
	dma_set_mask( &thiz->lcd->dev, DMA_BIT_MASK(32) );
	
	/* 
	* TODO - There is a problem here.....
	* 
	* The same SPS/BAM is used for NAND and LCD. However NAND
	* doesn't work when this code initializes/registers the 
	* thing. 
	* 
	* !!!!!!!!!!! The device will not boot!!!!!!!!!!!!!!!!
	*/
	bam.phys_addr = 0x7980000 + 0x4000;
	bam.virt_addr = (void *)bam.phys_addr/*((u32)thiz->lcd_ifb + 0x4000) */;
	bam.irq = 247;
	bam.manage = SPS_BAM_MGR_DEVICE_REMOTE | SPS_BAM_MGR_MULTI_EE;

	rc = sps_phy2h(bam.phys_addr, &bam_handle);
	if (rc) {
		 dev_err( &thiz->lcd->dev, "%s bam_handle is NULL",
		                           __FUNCTION__);
		 rc = -ENOMEM;
		 goto out;
	}

	pipe_handle = sps_alloc_endpoint();
	if (!pipe_handle) {
		dev_err( &thiz->lcd->dev, "sps_alloc_endpoint failed");
		rc = -ENOMEM;
		goto out;
	}

	rc = sps_get_config(pipe_handle, sps_config);
	if (rc) {
		 dev_err( &thiz->lcd->dev, "sps_get_config failed %d", rc);
		 goto free_endpoint;
	}

	/* WRITE CASE: source - system memory; destination - BAM */
	sps_config->source = SPS_DEV_HANDLE_MEM;
	sps_config->destination = bam_handle;
	sps_config->mode = SPS_MODE_DEST;
	sps_config->dest_pipe_index = 6;

	sps_config->options = SPS_O_AUTO_ENABLE | SPS_O_EOT;
	sps_config->lock_group = 0;
	/*
	* Descriptor FIFO is a cyclic FIFO. If 64 descriptors
	* are allowed to be submitted before we get any ack for any of them,
	* the descriptor FIFO size should be: (SPS_MAX_DESC_NUM + 1) *
	* sizeof(struct sps_iovec).
	*/
	sps_config->desc.size = (64) * sizeof(struct sps_iovec);
	sps_config->desc.base = dmam_alloc_coherent(&thiz->lcd->dev,
					sps_config->desc.size,
					&sps_config->desc.phys_base,
					GFP_KERNEL);
	if (!sps_config->desc.base) {
		dev_err( &thiz->lcd->dev,
			"dmam_alloc_coherent() failed for size %x",
			sps_config->desc.size);
		rc = -ENOMEM;
		goto free_endpoint;
	}
	memset(sps_config->desc.base, 0x00, sps_config->desc.size);

	rc = sps_connect(pipe_handle, sps_config);
	if (rc) {
		dev_err( &thiz->lcd->dev, "sps_connect() failed %d", rc);
		goto free_endpoint;
	}

	init_completion(&thiz->sps.completion);
	sps_event->mode = SPS_TRIGGER_WAIT;
	sps_event->options = SPS_O_EOT;
	sps_event->xfer_done = &thiz->sps.completion;
	sps_event->callback = NULL;
	sps_event->user = NULL;

	rc = sps_register_event(pipe_handle, sps_event);
	if (rc) {
		dev_err( &thiz->lcd->dev, 
			"sps_register_event() failed %d", rc);
		goto sps_disconnect;
	}

	thiz->sps.handle = pipe_handle;
	goto out;
sps_disconnect:
	sps_disconnect(pipe_handle);
free_endpoint:
	sps_free_endpoint(pipe_handle);
out:
	return rc;
}

static int ebi2_bus_register( struct platform_device *pdev )
{
	struct msm_bus_scale_pdata *use_cases;

	use_cases = msm_bus_cl_get_pdata(pdev);
	if( !use_cases ) {
		dev_err( &pdev->dev, "Failed to register bus client" );
		return 0;
	}
	return msm_bus_scale_register_client(use_cases);
}

struct bus_access * ebi2_probe( struct platform_device *dev )
{
	struct ebi2_lcd * lcd;
	void __iomem * io_base;
	
	dev_dbg( &dev->dev, "%s", __FUNCTION__ );

	io_base = of_iomap( dev->dev.of_node, 0 );
	if( !io_base ) {
		dev_err( &dev->dev, "%s: Missing EBI2 base address",	__FUNCTION__);
		return NULL;
	}

	lcd = kzalloc( sizeof(*lcd), GFP_KERNEL );
	if( !lcd ) {
		dev_err( &dev->dev, "%s: Alloc error", __FUNCTION__);
		return NULL;
	}

	/* SPS/BAM/DMA limited to 32K */
	lcd->maxlen = 32768;
	lcd->virt_addr = kmalloc( lcd->maxlen, GFP_KERNEL | GFP_DMA );
	if( !lcd->virt_addr ) {
		dev_err( &dev->dev, "%s: DMA Alloc error", __FUNCTION__);
		kfree( lcd );
		return NULL;
	}

	/* Config GPIOs for EBI2 access */
	{
		struct pinctrl * pinctrl = NULL;
		struct pinctrl_state * pinstate = NULL;
		int err = 0XAAAA;

		pinctrl = devm_pinctrl_get( &dev->dev );
		if( pinctrl )
			pinstate = pinctrl_lookup_state( pinctrl, "ebi2_default" );

		if( pinstate )
			err = pinctrl_select_state( pinctrl, pinstate );

		dev_dbg( &dev->dev, "%s: PIN state changed %d",	__FUNCTION__, err );
	}

	/* Need to initialize SPS/BAM later because of a conflict with 
	 * NAND!
	 */
	lcd->sps_init = 0;

	lcd->lcd_ifb = io_base;

	lcd->lcd		= dev;
	lcd->acc.aobj		= lcd;
	lcd->acc.write_cmd	= ebi2_write_cmd;
	lcd->acc.read_data	= ebi2_read;
	lcd->acc.flush		= ebi2_flush;
	lcd->acc.set_speed	= ebi2_set_speed;
	lcd->bus_allocated  = 0;


	lcd->qpic_clk = of_clk_get_by_name( dev->dev.of_node, "core_a_clk" );
	if (IS_ERR_OR_NULL(lcd->qpic_clk)) {
		dev_err( &dev->dev, "%s: devm_clk_get error", __FUNCTION__);
		kfree( lcd );
		return NULL;
	}

	platform_set_drvdata( dev, lcd );
	
	lcd->bus_handle = ebi2_bus_register( dev );

	ebi2_clk_vote(lcd, 1);
	/* Enable LCD controller */
	ebi2_enable_lcdc( lcd );
	ebi2_set_speed( lcd, 180 );
	ebi2_clk_vote(lcd, 0);

	return &lcd->acc;
}
EXPORT_SYMBOL_GPL( ebi2_probe );

static int __init ebi2_init(void)
{
	return 0;
}

static void __exit ebi2_exit(void)
{
	return;
}
module_init( ebi2_init );
module_exit( ebi2_exit );

MODULE_AUTHOR("msafar");
MODULE_DESCRIPTION("EBI2 for LCD");
MODULE_VERSION("3.0");

MODULE_LICENSE("GPL");
