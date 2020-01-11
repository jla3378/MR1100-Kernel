/*
 *	Periodic waker
 *
 *	Copyright (c) 2014-2015 Netgear Inc.
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License version 2 as
 *	published by the Free Software Foundation.
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/rtc.h>
#include <linux/suspend.h>
#include <soc/qcom/event_timer.h>
#include <linux/time.h>

static struct notifier_block snb;
static struct wakeup_source * ws;
static struct rtc_device * rtc = NULL;


static int sleep_dur = 5;
module_param( sleep_dur, int, S_IRUGO | S_IWUSR );
MODULE_PARM_DESC( sleep_dur, "Max. Sleep duration (seconds)" );

static int wake_dur = 0;
module_param( wake_dur, int, S_IRUGO | S_IWUSR );
MODULE_PARM_DESC( wake_dur, "Min. Wake duration (ms)" );


static unsigned long now( void )
{
	struct rtc_time rtcnow;
	long _now;

	rtc_read_time( rtc, &rtcnow );
	rtc_tm_to_time( &rtcnow, &_now );

	return _now;
}

static void ntgr_suspend( void )
{
	unsigned long susp_time;
	struct rtc_wkalrm alarm;

	if( !sleep_dur )
		return;

	susp_time = now();

	rtc_time_to_tm( susp_time + sleep_dur, &alarm.time );
	alarm.enabled = true;

	rtc_set_alarm( rtc, &alarm );

	printk( KERN_INFO "%s wakeup scheduled %ld\n", __FUNCTION__, susp_time+sleep_dur );
}


static void ntgr_resume( void )
{
	printk( KERN_INFO "%s\n", __FUNCTION__ );

	if( wake_dur )
		__pm_wakeup_event( ws, wake_dur );
}


static int waker_ntf( struct notifier_block *ntf, unsigned long msg, void * unused )
{
	switch(msg)
	{
	case PM_SUSPEND_PREPARE:
		ntgr_suspend();
		break;

	case PM_POST_SUSPEND:
		ntgr_resume();
		break;
	}

	return 0;
}

static int __init has_wakealarm(struct device *dev, const void *data)
{
	struct rtc_device *candidate = to_rtc_device(dev);

	if (!candidate->ops->set_alarm)
		return 0;
	if (!device_may_wakeup(candidate->dev.parent))
		return 0;

	return 1;
}

static int __init waker_init(void)
{
	struct device * devrtc;

	devrtc = class_find_device( rtc_class, NULL, NULL, has_wakealarm );
	if( devrtc )
	{

		rtc = rtc_class_open( (char *)dev_name(devrtc) );
		if( !rtc )
		{
			printk( KERN_ERR "Failed to open RTC %s\n", dev_name(devrtc) );
			return -EIO;
		}
	}
	else
	{
		printk( KERN_INFO "Not RTC with wakeup capabilities found\n" );
		return -ENOENT;
	}

	ws = wakeup_source_register( "waker" );
	if( !ws )
	{
		printk( KERN_ERR "Failed to create wakeup source\n" );
		return -ENOMEM;
	}
	snb.notifier_call = waker_ntf;
	snb.priority = 0;
	register_pm_notifier( &snb );

	return 0;
}

static void __exit waker_exit(void)
{
	unregister_pm_notifier( &snb );

	rtc_class_close( rtc );

	wakeup_source_unregister( ws );

	return;
}

late_initcall( waker_init );
module_exit( waker_exit );

MODULE_AUTHOR("msafar");
MODULE_DESCRIPTION("Periodic waker");
MODULE_VERSION("0.1");

MODULE_LICENSE("GPL");
