/*
 *  ST7789S driver
 *
 *  Copyright (c) 2013, 2014 Netgear
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */
//#define DEBUG
//#define VERBOSE_DEBUG


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/bug.h>
#include <linux/suspend.h>


#include <linux/fb.h>
#include <linux/init.h>
#include <video/ntgr-lcd_interface.h>

/* Minimum write cycle length in ns */
#define LCD_MIN_WRITE_CYCLE_NS	66
/* worked with 48 */

/* The only depth we support */
#define BITS_PER_PIXEL	16
#define BYTES_PER_PIXEL (BITS_PER_PIXEL / 8)

/* The only resolutions we support */
#define X_RESOLUTION	320
#define Y_RESOLUTION	240

struct i9341 {
	struct backlight_device * bldev;
	struct bus_access	* acc;
	struct mutex		bus_lock;
	struct fb_deferred_io 	dio;
	u32			pallete[16];
	int			brightness;
	struct fb_info		* fbinfo;
	struct delayed_work	sleep_in;
};
static bool noupdate = false;
module_param( noupdate, bool, S_IRUGO | S_IWUSR );
MODULE_PARM_DESC( noupdate, "Disable LCD data updates" );

/* ST7789S commands */
#define ST7789S_RESET		0x01

#define ST7789S_READ_ID4	0xD3
#define ST7789S_ID4_ID		0x009341

#define ST7789S_SLEEPOUT	0x11
#define ST7789S_SLEEPIN		0x10

#define ST7789S_DISP_ON		0x29

#define ST7789S_RASET		0x2B
#define ST7789S_CASET		0x2A
#define ST7789S_DATA		0x2C
#define ST7789S_DATA_CONTINUE    0x3C
#define ST7789S_MADCTL		0x36
#define ST7789S_PIXFMT		0x3A
#define ST7789S_TEAREFFCT	0x35

#define ST7789S_BLCTRL		0x53
#define ST7789S_BLBRIGHTNESS	0x51
#define ST7789S_BLTRANSCTRL	0xbc

static void for_each_page( void * b, unsigned long size, void (*act)(struct page*) )
{
	u8	* adr = b;
	while (size > 0) {
		act(vmalloc_to_page(adr));
		adr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}
}
static void *rvmalloc(unsigned long size)
{
	void *mem;

	size = PAGE_ALIGN(size);
	mem = vmalloc_32(size);
	if (mem) {
		memset(mem, 0, size);
		for_each_page(mem, size, SetPageReserved);
	}
	return mem;
}

static void rvfree(void *mem, unsigned long size)
{
	if (!mem)
		return;

	size = PAGE_ALIGN(size);
	for_each_page(mem, size, ClearPageReserved);
	vfree(mem);
}


static inline struct bus_access * toacc( struct fb_info * info )
{
	struct i9341 * par = info->par;
	return par->acc;
}

static void st7789_cmd_data( struct fb_info * info, u8 cmd, void * dp, u32 dlen )
{
	struct bus_access * acc = toacc( info );
	struct i9341 * par = info->par;

	mutex_lock( &par->bus_lock );

	acc->write_cmd( acc->aobj, cmd, dp, dlen );
	acc->flush(acc->aobj);

	mutex_unlock( &par->bus_lock );
}

static void st7789_cmd( struct fb_info * info, u8 cmd )
{
	st7789_cmd_data( info, cmd, NULL, 0 );
}

static void st7789_single_par_cmd( struct fb_info * info, u8 cmd, u8 cmdpar )
{
	st7789_cmd_data( info, cmd, &cmdpar, sizeof(cmdpar) );
}

/* Setup rectangle for data the will follow
 *
 * par->bus_lock must be held!
 */
static void st7789_setup_rect( struct fb_info * info, int xs, int xe, int ys, int ye )
{
	struct bus_access * acc = toacc( info );
	
	u16 row[] = { ys, ye };
	u16 col[] = { xs, xe };

	dev_dbg( info->device, "%s: %d - %d, %d - %d", __FUNCTION__,
				                      xs, xe, ys, ye );

	acc->write_cmd( acc->aobj, ST7789S_RASET, &row, sizeof(row) );
	
	acc->write_cmd( acc->aobj, ST7789S_CASET, &col, sizeof(col) );

}
/* Sync rectangle of data stored in our memory with framebuffer in controller! */
static void st7789_update_rect( struct fb_info * info, int x, int y, int dx, int dy )
{
	struct bus_access * acc = toacc( info );
	struct i9341 * par = info->par;
	u8 cmd = ST7789S_DATA;

	u16 * pix;

	dev_dbg( info->device, "%s", __FUNCTION__ );
	
	mutex_lock( &par->bus_lock );

	st7789_setup_rect( info, x, x+dx-1, y, y+dy-1 );

	pix = (u16 *)info->fix.smem_start + (y * info->var.xres);

	if( x == 0 && dx == X_RESOLUTION ) {
		/* optimization for whole screen updates - transfer
		 * up to 25 lines in one request
		 */
		while( dy > 0 ) {
			int lines = 25;
			if( lines > dy )
				lines = dy;

			acc->write_cmd( acc->aobj, cmd,
					&pix[x],
					X_RESOLUTION * sizeof(*pix) * lines );
			pix += X_RESOLUTION * lines;

			/* next command is going to be continue */
			cmd = ST7789S_DATA_CONTINUE;

			dy -= lines;
		}
	}
	else {
		while( dy > 0 )
		{
			acc->write_cmd( acc->aobj, cmd, &pix[x], dx * sizeof(*pix) );
			pix += info->var.xres;

			/* next command is going to be continue */
			cmd = ST7789S_DATA_CONTINUE;

			dy --;
		}
	}
	acc->flush(acc->aobj);

	mutex_unlock( &par->bus_lock );
}

static void st7789_update(struct fb_info *info, struct list_head *pagelist)
{
	struct i9341 * par = info->par;

	if( !par->brightness || noupdate )
	{
		dev_dbg( info->device, "%s - bailing out", __FUNCTION__ );
		return;
	}
	dev_dbg( info->device, "%s", __FUNCTION__ );

	st7789_update_rect( info, 0, 0, info->var.xres, info->var.yres );

	dev_dbg( info->device, "%s - done", __FUNCTION__ );
}

static void st7789_schedule_update(struct fb_info *info, long delay)
{
	schedule_delayed_work(&info->deferred_work, msecs_to_jiffies(delay) );
}

static void st7789_fillrect(struct fb_info *info, const struct fb_fillrect *rect)
{
	dev_dbg( info->device, "%s", __FUNCTION__ );
	sys_fillrect( info, rect );
	st7789_update_rect( info, rect->dx, rect->dy, rect->width, rect->height );
}
static void st7789_copyarea(struct fb_info *info, const struct fb_copyarea *area)
{
	dev_dbg( info->device, "%s", __FUNCTION__ );
	sys_copyarea( info, area );
	st7789_update_rect( info, area->dx, area->dy, area->width, area->height );
}
static void st7789_imageblit(struct fb_info *info, const struct fb_image *image)
{
	dev_dbg( info->device, "%s", __FUNCTION__ );
	sys_imageblit( info, image );
	st7789_update_rect( info, image->dx, image->dy, image->width, image->height );
}

static ssize_t st7789_sys_write(struct fb_info *info, const char __user *buf,
				size_t count, loff_t *ppos)
{
	ssize_t rv;
	
	dev_dbg( info->device, "%s", __FUNCTION__ );

	rv = fb_sys_write( info, buf, count, ppos );
	st7789_schedule_update( info, 100 );
	/* alternatively we could calc the rectangle that changed and update it */
	return rv;
}

static int st7789_check_var(struct fb_var_screeninfo *var,
			 struct fb_info *info)
{
	dev_dbg( info->device, "%s", __FUNCTION__ );

	if(var->bits_per_pixel == BITS_PER_PIXEL )
		return 0;
	
	return -EINVAL;
}
/* needed for FB operation */
static int st7789_set_par(struct fb_info *info)
{
	dev_dbg( info->device, "%s", __FUNCTION__ );
	info->fix.line_length = X_RESOLUTION * BYTES_PER_PIXEL;
	dev_dbg( info->device, "%s - var: %d", __FUNCTION__, info->var.red.offset );
	if( info->var.red.offset != 0 )
		return EINVAL;
	return 0;
}
/* needed for FB operation */
static int st7789_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			 u_int transp, struct fb_info *info)
{
	if (regno >= info->cmap.len)	/* no. of hw registers */
		return 1;
		
#define CNVT_TOHW(val,width) ((((val)<<(width))+0x7FFF-(val))>>16)
	red = CNVT_TOHW(red, info->var.red.length);
	green = CNVT_TOHW(green, info->var.green.length);
	blue = CNVT_TOHW(blue, info->var.blue.length);
	transp = CNVT_TOHW(transp, info->var.transp.length);
#undef CNVT_TOHW
	dev_dbg( info->device, "%s, [%d]%d, %d, %d : %04x", __FUNCTION__, regno, red, green, blue,
			(red << info->var.red.offset) |
			(green << info->var.green.offset) |
			(blue << info->var.blue.offset) |
			(transp << info->var.transp.offset)
			);
	((u32 *) (info->pseudo_palette))[regno] = 
			(red << info->var.red.offset) |
			(green << info->var.green.offset) |
			(blue << info->var.blue.offset) |
			(transp << info->var.transp.offset);
	return 0;
}
/* Backlight operation functions */
static void st7789_sleep( struct work_struct * work )
{
	struct i9341 * par = container_of(work, struct i9341, sleep_in.work);
	struct fb_info * info = par->fbinfo;

	dev_dbg( info->device, "%s", __FUNCTION__ );
	st7789_cmd( info, ST7789S_SLEEPIN );
}

static int st7789_check_fb(struct backlight_device *bdev, struct fb_info * info)
{
	struct i9341 * par = info->par;
	dev_dbg( &bdev->dev, "%s", __FUNCTION__ );
	return ( bdev == par->bldev );
}
static int st7789_update_brightness(struct backlight_device *bdev)
{
	struct fb_info * info = bl_get_data(bdev);
	struct i9341   * par  = info->par;
	int              brightness = bdev->props.brightness;
	bool             was_on;

	was_on = (par->brightness != 0);

	dev_dbg( &bdev->dev, "%s (%d)", __FUNCTION__, brightness );

	if( bdev->props.power != FB_BLANK_UNBLANK ||
		bdev->props.state & (BL_CORE_SUSPENDED | BL_CORE_FBBLANK) )
	{
		brightness = 0;
	}

	/* set brightness */
	par->brightness = brightness;

	if( brightness != 0 && !was_on )
	{
		/* turning ON */
		
		/* Unit cannot suspend - we need to serve UI */
		pm_stay_awake( &bdev->dev );

		/* get LCD out of sleep state and update screen data */
		cancel_delayed_work_sync( &par->sleep_in );

		/* Enable backlight control  - No Dimming */
		st7789_single_par_cmd( info, ST7789S_BLCTRL, 0x24 );
		st7789_cmd( info, ST7789S_SLEEPOUT );

		mdelay( 150 );

		st7789_schedule_update( info, 0 );
		flush_delayed_work( &info->deferred_work );
	}
	else if( brightness == 0 && was_on )
	{
		st7789_single_par_cmd( info, ST7789S_BLCTRL, 0x2C );
		/* turning OFF - wait for dimming effect to finish and go to sleep state */
		schedule_delayed_work( &par->sleep_in, msecs_to_jiffies(600) );
		
		/* remove the suspend block */
		pm_relax( &bdev->dev );
	}
	st7789_single_par_cmd( info, ST7789S_BLBRIGHTNESS, brightness );

	return 0;
}

static int st7789_get_brightness(struct backlight_device *bdev)
{
	struct fb_info * info = bl_get_data(bdev);
	struct i9341   * par  = info->par;

	dev_dbg( &bdev->dev, "%s", __FUNCTION__ );

	return par->brightness;
}


static struct fb_var_screeninfo st7789_default = {
	.xres =		X_RESOLUTION,
	.yres =		Y_RESOLUTION,
	.xres_virtual =	X_RESOLUTION,
	.yres_virtual =	Y_RESOLUTION,
	.bits_per_pixel = BITS_PER_PIXEL,

	.red =		{ 11, 5, 0 },
	.green =	{  5, 6, 0 },
	.blue =		{  0, 5, 0 },

	.activate =	FB_ACTIVATE_TEST,
	.height =	-1,
	.width =	-1,
	.pixclock =	20000,
	.hsync_len =	64,
	.vsync_len =	2,
	.vmode =	FB_VMODE_NONINTERLACED,
};

static struct fb_fix_screeninfo st7789_fix = {
	.id =		"ST7789S",
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	FB_VISUAL_TRUECOLOR,
	.xpanstep =	0,
	.ypanstep =	0,
	.ywrapstep =0,
	.accel =	FB_ACCEL_NONE,
	.line_length = X_RESOLUTION * BYTES_PER_PIXEL,
};

static struct fb_ops st7789_ops = {
	.fb_read        = fb_sys_read,
	.fb_write       = st7789_sys_write,
	.fb_check_var	= st7789_check_var,
	.fb_set_par	= st7789_set_par,
	.fb_setcolreg	= st7789_setcolreg,
	.fb_fillrect	= st7789_fillrect,
	.fb_copyarea	= st7789_copyarea,
	.fb_imageblit	= st7789_imageblit,
};

static const struct backlight_ops st7789_backlight_ops = {
		.update_status = st7789_update_brightness,
		.get_brightness = st7789_get_brightness,
		.check_fb = st7789_check_fb,
};

static int st7789_probe(struct platform_device *dev)
{
	struct backlight_device * bl;
	unsigned long videomemorysize = X_RESOLUTION * Y_RESOLUTION * BYTES_PER_PIXEL;
	unsigned long videomemory;
	struct fb_info *info;
	int retval = -ENOMEM;
	struct i9341 * par;
	struct bus_access * acc;

	dev_dbg( &dev->dev, "%s", __FUNCTION__ );

	acc = ebi2_probe( dev );
	if( !acc ) {
		dev_dbg( &dev->dev, "%s EBI2 init failed", __FUNCTION__ );
		return -ENXIO;
	}

	if (!(videomemory = (unsigned long)rvmalloc(videomemorysize))) {
		ebi2_remove( acc );
		return retval;
	}
	info = framebuffer_alloc(sizeof(struct i9341), &dev->dev);
	if (!info)
		goto err;

	par = info->par;
	par->fbinfo = info;

	par->acc = acc;
	par->dio.deferred_io = st7789_update;
	par->dio.delay = msecs_to_jiffies( 20 );
	mutex_init( &par->bus_lock );
	INIT_DELAYED_WORK( &par->sleep_in, st7789_sleep );

	info->screen_base = (char __iomem *)videomemory;
	info->fbops = &st7789_ops;
	info->var = st7789_default;

	st7789_fix.smem_start = videomemory;
	st7789_fix.smem_len = videomemorysize;
	info->fix = st7789_fix;
	info->pseudo_palette = par->pallete;
	info->flags = FBINFO_FLAG_DEFAULT;

	retval = fb_alloc_cmap(&info->cmap, 16, 0);
	if (retval < 0)
		goto err1;

	info->fbdefio = &par->dio;
	fb_deferred_io_init( info );

	/* H/W access */
	acc->set_speed( acc->aobj, LCD_MIN_WRITE_CYCLE_NS ); /* spec says > 66ns */

	/*
	 * Take device from sleep, and turn the display On - if it wasn't done in
	 * bootloader
	 */
	st7789_cmd( info, ST7789S_SLEEPOUT );
	msleep( 120 );
	st7789_cmd( info, ST7789S_DISP_ON );


	st7789_single_par_cmd( info, ST7789S_PIXFMT, 0x55 ); /* 16 bits/pix */
	st7789_single_par_cmd( info, ST7789S_MADCTL, 0x60 ); /* Landscape */

//	st7789_single_par_cmd( info, ST7789S_TEAREFFCT, 0x0 ); /* tearing effect line on */

	/* We leave the frame buffer inside LCD untouched - the bootloader could
	 * have put there a splash screen.
	 */
	retval = register_framebuffer(info);
	if (retval < 0)
		goto err2;
	platform_set_drvdata(dev, info);

	dev_info( &dev->dev, "%s - initialized.", __FUNCTION__ );

	/*
	 * For general use we should make it dependent on some platform data (?)
	 */
	bl = backlight_device_register( "st7789bl", &dev->dev, info, &st7789_backlight_ops, NULL );
	if( !IS_ERR(bl) )
	{
		device_init_wakeup( &bl->dev, true );
		pm_stay_awake( &bl->dev );
		
		bl->props.max_brightness = 0xFF;
		bl->props.brightness = 0xFF;
		/*
		 * We need to set this to !=0, otherwise we trigger the "get out of sleep" sequence
		 * that includes fb update and that blanks the screen.
		 */
		par->brightness = 0xFF;

		/* Enable backlight control */
		st7789_single_par_cmd( info, ST7789S_BLCTRL, 0x2C );

		par->bldev = bl;

		backlight_update_status(bl);
	}

	/* Disable VT switching */
	pm_set_vt_switch( 0 );

	return 0;
err2:
	fb_dealloc_cmap(&info->cmap);
err1:
	framebuffer_release(info);
err:
	rvfree((void *)videomemory, videomemorysize);
	ebi2_remove( acc );
	return retval;
}

static int st7789_remove(struct platform_device *dev)
{
	struct fb_info * info = platform_get_drvdata(dev);
	struct i9341   * par = NULL;

	dev_dbg( info->device, "%s", __FUNCTION__ );
	
	if( info )
		par = info->par;

	if( par ) {
		cancel_delayed_work_sync( &par->sleep_in );
		st7789_sleep( &par->sleep_in.work );
    backlight_device_unregister( par->bldev );
	}

	if (info) {

		unregister_framebuffer(info);
		fb_dealloc_cmap(&info->cmap);
    fb_deferred_io_cleanup(info);
		framebuffer_release(info);
		rvfree((void *)info->fix.smem_start, info->fix.smem_len);
	}
	return 0;
}

static struct of_device_id st7789_dt_info[] = {
	{
		.compatible = "Sitronix,ST7789S",
	},
	{}
};

static struct platform_driver st7789_driver = {
	.probe	= st7789_probe,
	.remove = st7789_remove,
	.driver = {
		.owner =	THIS_MODULE,
		.name = "st7789sebi2",
		.of_match_table = st7789_dt_info,
	},
};

static int __init st7789_init(void)
{
	dev_dbg( NULL, "%s", __FUNCTION__ );
	return platform_driver_register(&st7789_driver);
}


static void __exit st7789_exit(void)
{
	dev_dbg( NULL, "%s", __FUNCTION__ );
	platform_driver_unregister(&st7789_driver);
}

module_exit(st7789_exit);
module_init(st7789_init);

MODULE_AUTHOR("msafar");
MODULE_DESCRIPTION("ST7789S");
MODULE_VERSION("1.02");

MODULE_LICENSE("GPL");
