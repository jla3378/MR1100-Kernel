/*
 * Copyright (C) 2008-2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 *
 *
 * NTGRSTART
 *  Copyright (C) 2014 Netgear, Inc.
 *
 *   - device driver for Fairchild FAN 5646 blinker
 *   - device tree compatible
 */

/*
 * Fairchild FAN5646 blinking LED Driver for Android
 *
 * Alina Yakovleva qvdh43@motorola.com
 */

//#define DEBUG

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>

#include <linux/ctype.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/of_gpio.h>


#define FAN5646_SLEW1_REG   0
#define FAN5646_PULSE1_REG  0x1
#define FAN5646_SLEW2_REG   0x2
#define FAN5646_PULSE2_REG  0x3
#define FAN5646_CONTROL_REG 0x4

#define FAN5646_ISET_5mA  0
#define FAN5646_ISET_10mA 0x40
#define FAN5646_ISET_15mA 0x80
#define FAN5646_ISET_20mA 0xC0

#define FAN5646_FOLLOW 0x1
#define FAN5646_PLAY   0x2
#define FAN5646_SLOW   0x4

#define FAN5646_MAX_TON        1600
#define FAN5646_MAX_TOFF       4800
#define FAN5646_MAX_TRISE      1550
#define FAN5646_MAX_TFALL      1550
#define FAN5646_MAX_ON         4700 // tRise + tFall + tOn
#define FAN5646_MAX_ON_SLOW    6300 // tRise + tFall + 2 * tOn
#define FAN5646_MAX_OFF        FAN5646_MAX_TOFF
#define FAN5646_MAX_OFF_SLOW   9600

#define NUM_LEDS 1  /* code is not ready for != 1 */
#define TRESET 110 // Sleep time in us

#define DEFAULT_UP   45
#define DEFAULT_DOWN 45

#define MODULE_NAME "leds_fan5646"
enum {
    TRACE_BRIGHTNESS = 0x1,
    TRACE_TINYWIRE = 0x2,
};

struct fan5646 {
    struct led_classdev leds[NUM_LEDS];
    unsigned is_reg[NUM_LEDS];
    struct device *dev;
    __u8 default_current;
    unsigned tsleep;
    spinlock_t lock;
    struct vreg *vr;
    int gpionum;
};

static void fan5646_brightness_set (struct led_classdev *led_cdev, 
    enum led_brightness bvalue);
static void fan5646_set_pulse (unsigned msOn, unsigned msOff,
    unsigned ramp_up, unsigned ramp_down, __u8 *slew, __u8 *pulse);
static int fan5646_blink_set (struct led_classdev *led_cdev,
    unsigned long *delay_on, unsigned long *delay_off);
static void tinywire_write_reg (int gpio, __u8 reg, __u8 value, unsigned tsleep);
static void tinywire_send_bit (int gpio, __u8 bit, unsigned tsleep);

static struct fan5646 fan5646_data = {
    .leds = {
        {
            .name            = "messaging",
            .brightness_set  = fan5646_brightness_set,
            .blink_set       = fan5646_blink_set,
        },
    },
    .default_current = FAN5646_ISET_5mA,
    .tsleep = 7,
//    .lock = SPIN_LOCK_UNLOCKED,
    .vr = NULL,

};

void inline tinywire_send_bit (int gpio, __u8 bit, unsigned tsleep)
{
    //printk (KERN_ERR "%s: gpio=%d, bit=%d, tsleep=%d\n",
    //    __FUNCTION__, gpio, bit, tsleep);
    if (bit == 0) {
        gpio_set_value (gpio, 1);
        udelay (tsleep);
        gpio_set_value (gpio, 0);
        udelay (tsleep * 4);
    } else {
        gpio_set_value (gpio, 1);
        udelay (tsleep * 4);
        gpio_set_value (gpio, 0);
        udelay (tsleep);
    }
}

void inline tinywire_send_reset (int gpio)
{
    gpio_set_value (gpio, 0);
    udelay (TRESET);
}

void inline tinywire_send_exec (int gpio)
{
    gpio_set_value (gpio, 1);
    udelay (TRESET);
}

void tinywire_write_reg (int gpio, __u8 reg, __u8 value, unsigned tsleep)
{
    int i;
    __u8 mask = 0x1;

    dev_dbg( fan5646_data.dev, "%s: reg=0x%x, value=0x%0x, tsleep=%dus",
        __FUNCTION__, reg, value, tsleep);
    /* Register address is 3 bits.  Send it LSB first */
    for (i = 0; i < 3; i++) {
        tinywire_send_bit (gpio, reg & (mask << i), tsleep);
    }
    /* Now send data LSB first */
    for (i = 0; i < 8; i++) {
        tinywire_send_bit (gpio, value & (mask << i), tsleep);
    }
    /* Send STOP bit */
    tinywire_send_bit (gpio, 0, tsleep);
    /* Wait for TRESET so that it stays IDLE */
    udelay (TRESET);
}

static void fan5646_brightness_set (struct led_classdev *led_cdev, 
    enum led_brightness value)
{
    __u8 ctrl_value;
    unsigned long flags;

    dev_dbg( led_cdev->dev, "%s: %d\n", __FUNCTION__, value );

    spin_lock_irqsave (&fan5646_data.lock, flags);
    tinywire_send_reset (fan5646_data.gpionum);
    if (value) {
        /* Set default current and follow bit and raise control */
        ctrl_value = fan5646_data.default_current | FAN5646_FOLLOW;
        tinywire_write_reg (fan5646_data.gpionum, FAN5646_CONTROL_REG, ctrl_value,
            fan5646_data.tsleep);
        /* Clear second pulse or it will keep blinking */
        tinywire_write_reg (fan5646_data.gpionum, FAN5646_PULSE1_REG, 0,
            fan5646_data.tsleep);
        tinywire_send_exec (fan5646_data.gpionum);
    }
    spin_unlock_irqrestore (&fan5646_data.lock, flags);
}

static int fan5646_blink_set (struct led_classdev *led_cdev,
    unsigned long *delay_on, unsigned long *delay_off)
{
    __u8 ctrl_value = fan5646_data.default_current | FAN5646_PLAY;
    __u8 slew, pulse;
    unsigned long flags;

    dev_dbg( led_cdev->dev, "%s: delay_on = %lu, delay_off = %lu\n",
        __FUNCTION__, *delay_on, *delay_off);
    if (*delay_on == 0 && *delay_off == 0) {
        *delay_on = 500;
        *delay_off = 500;
    }

    spin_lock_irqsave (&fan5646_data.lock, flags);
    tinywire_send_reset (fan5646_data.gpionum);
    tinywire_write_reg (fan5646_data.gpionum, FAN5646_CONTROL_REG, ctrl_value,
        fan5646_data.tsleep);
    fan5646_set_pulse (*delay_on, *delay_off, DEFAULT_UP, DEFAULT_DOWN,
        &slew, &pulse);
    tinywire_write_reg (fan5646_data.gpionum, FAN5646_SLEW1_REG, slew,
        fan5646_data.tsleep);
    tinywire_write_reg (fan5646_data.gpionum, FAN5646_PULSE1_REG, pulse,
        fan5646_data.tsleep);
    tinywire_write_reg (fan5646_data.gpionum, FAN5646_SLEW2_REG, 0,
        fan5646_data.tsleep);
    tinywire_write_reg (fan5646_data.gpionum, FAN5646_PULSE2_REG, 0,
        fan5646_data.tsleep);
    tinywire_send_exec (fan5646_data.gpionum);
    spin_unlock_irqrestore (&fan5646_data.lock, flags);
    return 0;
}

static void fan5646_set_pulse (unsigned msOn, unsigned msOff,
    unsigned ramp_up, unsigned ramp_down, __u8 *slew, __u8 *pulse)
{
    __u8 nRise, nFall, nOn, nOff;
    unsigned tRise, tFall, tOn, tOff;
    unsigned slow = 0;

    dev_dbg( fan5646_data.dev, "%s: msOn = %d, msOff = %d, ramp up = %d%%, down = %d%%",
        __FUNCTION__, msOn, msOff, ramp_up, ramp_down);
    *slew = 0;
    *pulse = 0;

    if (msOn == 0 && msOff == 0)
        return;
    /* We won't do slow for now */
    if (msOn > FAN5646_MAX_ON) {
        msOn = FAN5646_MAX_ON;
    }
    if (msOff > FAN5646_MAX_OFF) {
        msOff = FAN5646_MAX_OFF;
    }
    tOff = msOff;
    /* Now the blinking part 
     * msOn consists of 3 parts: tRise, tFall, and tOn.
     */
    if (ramp_up + ramp_down > 100) {
      dev_err( fan5646_data.dev, "%s: incorrect percent up %d%%, percent down %d%%; resetting",
            __FUNCTION__, ramp_up, ramp_down);
        ramp_up = DEFAULT_UP;
        ramp_down = DEFAULT_DOWN;
    }
    tOn = (100 - ramp_up - ramp_down) * msOn / 100; 
    tRise = ramp_up * msOn / 100;
    tFall = ramp_down * msOn / 100;
    //tRise = tFall = (msOn - tOn) / 2;
    if (tRise > FAN5646_MAX_TRISE) {
        tOn += tRise - FAN5646_MAX_TRISE;
        tRise = FAN5646_MAX_TRISE;
    }
    if (tFall > FAN5646_MAX_TRISE) {
        tOn += tFall - FAN5646_MAX_TRISE;
        tFall = FAN5646_MAX_TRISE;
    }
    // Now we need to calculate nRise, nFall, nOn and nOff
    // tRise = 31 * nRise * 3.33 ms, same for tFall
    // nRise = tRise / 103.23
    nRise = tRise * 100 / 10323;
    if (nRise > 0xF)
        nRise = 0xF;
    if (nRise == 0 && ramp_up != 0)
        nRise = 1;
    nFall = tFall * 100 / 10323;
    if (nFall > 0xF)
        nFall = 0xF;
    if (nFall == 0 && ramp_down != 0)
        nFall = 1;

    *slew = nRise << 4 | nFall;

    /* Now tOn and tOff
     * tOn = (SLOW + 1) * nOn * 106.6
     * tOff = (SLOW + 1) * nOff * 320
     * nOn = tOn / ((SLOW + 1) * 106.6)
     * nOff = tOff / ((SLOW + 1) * 320)
     */
    nOn = tOn * 10 / ((slow + 1) * 1066);
    nOff = tOff / ((slow + 1) * 320);
    if (nOn > 0xF)
        nOn = 0xF;
    if (nOff > 0xF)
        nOff = 0xF;
    if (nOn == 0 && (ramp_up + ramp_down < 100))
        nOn = 1;
    if (nOff == 0 && msOff != 0)
        nOff = 1;
    *pulse = nOn << 4 | nOff;

    dev_dbg( fan5646_data.dev, "%s: tRise = %d, tFall = %d, tOn = %d, tOff = %d, slow = %d",
        __FUNCTION__, tRise, tFall, tOn, tOff, slow);
    dev_dbg( fan5646_data.dev, "%s: nRise = 0x%x, nFall = 0x%x, nOn = 0x%x, nOff = 0x%x",
        __FUNCTION__, nRise, nFall, nOn, nOff);
}

static ssize_t fan5646_blink_show (struct device *dev,
                      struct device_attribute *attr, char *buf)
{
    return 0;
}

static ssize_t fan5646_blink_store (struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t size)
{
    unsigned msOn = 0, msOff = 0, bvalue = 0;
    unsigned msOn1 = 0, msOff1 = 0;
    int n;
    struct led_classdev *led_cdev;
    __u8 ctrl_value, slew, pulse, slew1 = 0, pulse1 = 0;
    int ramp_up = DEFAULT_UP;
    int ramp_down = DEFAULT_DOWN;
    int ramp_up1 = DEFAULT_UP;
    int ramp_down1 = DEFAULT_DOWN;
    char *ptr;
    unsigned long flags;

    if (!buf || size == 0) {
        dev_err(dev, "%s: invalid command", __FUNCTION__);
        return -EINVAL;
    }
    led_cdev = dev_get_drvdata (dev);

    /* The format is: "brightness msOn msOff" */
    n = sscanf (buf, "%d %d %d %d %d", &bvalue, &msOn, &msOff, &msOn1, &msOff1);

    if (n != 3 && n != 5) {
        dev_err( dev, "%s: invalid command: %s", __FUNCTION__, buf);
        return -EINVAL;
    }
    dev_dbg( dev, "%s: %s, b = %d, msOn = %d, msOff = %d, msOn1 = %d, msOff1 = %d",
        __FUNCTION__, led_cdev->name, bvalue, msOn, msOff, msOn1, msOff1);

    if (bvalue == 0 || (bvalue != 0 && msOn == 0 && msOff == 0)) {
        fan5646_brightness_set (led_cdev, bvalue);
        return size;
    }

    /* Now see if ramp values are there */
    ptr = strstr (buf, "ramp");
    if (ptr) {
        ptr = strpbrk (ptr, "0123456789");
        if (!ptr) {
            dev_err( dev, "%s: invalid command (ramp): %s",
                __FUNCTION__, buf);
            return -EINVAL;
        }
        n = sscanf (ptr, "%d %d %d %d", 
            &ramp_up, &ramp_down, &ramp_up1, &ramp_down1);
        if (n < 2) {
            dev_err( dev, "%s: invalid command (ramp): %s",
                __FUNCTION__, buf);
            return -EINVAL;
        }
        if (ramp_up < 0)
            ramp_up = DEFAULT_UP;
        if (ramp_down < 0)
            ramp_down = DEFAULT_DOWN;
        if (ramp_up1 < 0)
            ramp_up1 = DEFAULT_UP;
        if (ramp_down1 < 0)
            ramp_down1 = DEFAULT_DOWN;
        if (ramp_up + ramp_down > 100 || ramp_up1 + ramp_down1 > 100) {
            dev_err( dev, "%s: inavlid ramp times: %d%% up, %d%% down, %d%% up1, %d%% down1",
                __FUNCTION__, ramp_up, ramp_down, ramp_up1, ramp_down1);
            return -EINVAL;
        }
    }
    dev_dbg( dev, "%s: %s, ramp up = %d%%, ramp down = %d%%, ramp up1 = %d%%, ramp down1 = %d%%",
        __FUNCTION__, led_cdev->name, ramp_up, ramp_down, ramp_up1, ramp_down1);

    fan5646_set_pulse (msOn, msOff, ramp_up, ramp_down, &slew, &pulse);
    fan5646_set_pulse (msOn1, msOff1, ramp_up1, ramp_down1, &slew1, &pulse1);
    ctrl_value = fan5646_data.default_current | FAN5646_PLAY;

    spin_lock_irqsave (&fan5646_data.lock, flags);
    tinywire_send_reset (fan5646_data.gpionum);
    tinywire_write_reg (fan5646_data.gpionum, FAN5646_SLEW1_REG, slew,
        fan5646_data.tsleep);
    tinywire_write_reg (fan5646_data.gpionum, FAN5646_PULSE1_REG, pulse,
        fan5646_data.tsleep);
    tinywire_write_reg (fan5646_data.gpionum, FAN5646_SLEW2_REG, slew1,
        fan5646_data.tsleep);
    tinywire_write_reg (fan5646_data.gpionum, FAN5646_PULSE2_REG, pulse1,
        fan5646_data.tsleep);
    tinywire_write_reg (fan5646_data.gpionum, FAN5646_CONTROL_REG, ctrl_value,
        fan5646_data.tsleep);
    tinywire_send_exec (fan5646_data.gpionum);
    spin_unlock_irqrestore (&fan5646_data.lock, flags);
    return size;
}

static DEVICE_ATTR(blink, 0777, fan5646_blink_show, fan5646_blink_store);

static ssize_t fan5646_settings_show (struct device *dev,
                      struct device_attribute *attr, char *buf)
{
    sprintf (buf, "current = %dmA, timing = %d\n", 
        (fan5646_data.default_current + 1) * 5, fan5646_data.tsleep);
    return strlen(buf) + 1;
}

static ssize_t fan5646_settings_store (struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t size)
{
    unsigned value = 0;
    unsigned type = 0;
    char *ptr;

    if (!buf || size == 0) {
        dev_err( dev, "%s: invalid command", __FUNCTION__);
        return -EINVAL;
    }

    if (strstr (buf, "current")) {
        type = 1;
    } else if (strstr (buf, "timing")) {
        type = 2;
    } else {
        dev_err( dev, "%s: invalid command: %s", __FUNCTION__, buf);
        return -EINVAL;
    }
    ptr = strpbrk (buf, "0123456789");
    if (ptr)
        value = simple_strtoul (ptr, NULL, 10);
    else {
        dev_err( dev, "%s: invalid command: %s", __FUNCTION__, buf);
        return -EINVAL;
    }

    switch (type) {
        case 1:
            switch (value) {
                case 5: fan5646_data.default_current = FAN5646_ISET_5mA;
                    break;
                case 10: fan5646_data.default_current = FAN5646_ISET_10mA;
                    break;
                case 15: fan5646_data.default_current = FAN5646_ISET_15mA;
                    break;
                case 20: fan5646_data.default_current = FAN5646_ISET_20mA;
                    break;
                default:
                    dev_err( dev, "%s: inavlid current value: %d",
                        __FUNCTION__, value);
                    return -EINVAL;
            }
            dev_err( dev, "%s: changing current to %dmA",
                __FUNCTION__, value);
            break;
        case 2:
            dev_err( dev, "%s: changing timing to %dus",
                __FUNCTION__, value);
            fan5646_data.tsleep = value;
            break;
        default:
            dev_err( dev, "%s: inavlid command: %s", __FUNCTION__, buf);
            return -EINVAL;
    }

    return size;
}

static DEVICE_ATTR(settings, 0777, fan5646_settings_show, fan5646_settings_store);

static ssize_t fan5646_prot_show (struct device *dev,
                      struct device_attribute *attr, char *buf)
{
    return 0;
}

static ssize_t fan5646_prot_store (struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t size)
{
    unsigned reg, value = 0;
    __u8 _reg, _value = 0;
    int n;
    unsigned long flags;

    if (!buf || size == 0) {
        dev_err( dev, "%s: invalid command", __FUNCTION__);
        return -EINVAL;
    }

    n = sscanf (buf, "%d %d", &reg, &value);
    _reg = (__u8)reg;
    _value = (__u8)value;

    if (n == 0) {
        dev_err( dev, "%s: invalid command: %s\n", __FUNCTION__, buf);
        return -EINVAL;
    }
    if (n == 1) {
        dev_info( dev, "%s: %s CTRL", __FUNCTION__,
            reg ? "raising" : "lowering");
        spin_lock_irqsave (&fan5646_data.lock, flags);
        if (reg)
            tinywire_send_exec (fan5646_data.gpionum);
        else
            tinywire_send_reset (fan5646_data.gpionum);
        spin_unlock_irqrestore (&fan5646_data.lock, flags);
    } else if (n == 2) {
        dev_info( dev, "%s: reg=%d, value=%d",
            __FUNCTION__, _reg, _value);
        spin_lock_irqsave (&fan5646_data.lock, flags);
        tinywire_send_reset (fan5646_data.gpionum);
        tinywire_write_reg (fan5646_data.gpionum, _reg, _value, fan5646_data.tsleep);
        spin_unlock_irqrestore (&fan5646_data.lock, flags);
    }
    return size;
}

static DEVICE_ATTR(prot, 0777, fan5646_prot_show, fan5646_prot_store);

static void fan5646_remove_device_files (void)
{
    int i;

    for (i = 0; i < NUM_LEDS; i++) {
        device_remove_file (fan5646_data.leds[i].dev, &dev_attr_blink);
    }
    device_remove_file (fan5646_data.leds[0].dev, &dev_attr_prot);
    device_remove_file (fan5646_data.leds[0].dev, &dev_attr_settings);
}

static int fan5646_create_device_files (void)
{
    int i, ret;

    for (i = 0; i < NUM_LEDS; i++) {
        ret = device_create_file (fan5646_data.leds[i].dev, &dev_attr_blink);
        if (ret) {
            dev_err( fan5646_data.leds[i].dev, "%s: unable to create device file for %s: %d",
                __FUNCTION__, fan5646_data.leds[i].name, ret);
            fan5646_remove_device_files ();
            return ret;
        }
        dev_set_drvdata (fan5646_data.leds[i].dev, &fan5646_data.leds[i]);
    }
    ret = device_create_file (fan5646_data.leds[0].dev, &dev_attr_prot);
    if (ret) {
        dev_err( fan5646_data.leds[0].dev,"%s: unable to create \"prot\" device file for %s: %d",
                __FUNCTION__, fan5646_data.leds[0].name, ret);
        fan5646_remove_device_files ();
        return ret;
    }
    ret = device_create_file (fan5646_data.leds[0].dev, &dev_attr_settings);
    if (ret) {
        dev_err( fan5646_data.leds[0].dev, "%s: unable to create \"settings\" device file for %s: %d",
            __FUNCTION__, fan5646_data.leds[0].name, ret);
        fan5646_remove_device_files ();
        return ret;
    }
    return 0;
}

static void fan5646_unregister_leds (void)
{
    int i;

    for (i = 0; i < NUM_LEDS; i++) {
        if (fan5646_data.is_reg[i]) {
            led_classdev_unregister (&fan5646_data.leds[i]);
            fan5646_data.is_reg[i] = 0;
        }
    }
}

static int fan5646_register_leds (struct device *dev)
{
    int i;
    int ret;

    for (i = 0; i < NUM_LEDS; i++) {
        ret = led_classdev_register (dev, &fan5646_data.leds[i]);
        if (ret) {
            dev_err( dev, "%s: unable to register led %s: error %d",
                __FUNCTION__, fan5646_data.leds[i].name, ret);
            fan5646_unregister_leds ();
            return ret;
        } else {
            fan5646_data.is_reg[i] = 1;
        }
    }
    return 0;
}

static int fan5646_probe (struct platform_device *pdev)
{
    int ret;
    enum of_gpio_flags flgs;

    dev_dbg( &pdev->dev, "%s", __FUNCTION__ );

    fan5646_data.gpionum = of_get_named_gpio_flags( pdev->dev.of_node, "gpioctrl", 0, &flgs );
    if( !gpio_is_valid( fan5646_data.gpionum ) )
    {
      dev_err( &pdev->dev, "%s: no gpioctrl property", __FUNCTION__ );
      return -EINVAL;
    }
    ret = gpio_request( fan5646_data.gpionum, "fan5646_ctrl" );
    if( ret )
    {
      dev_err( &pdev->dev, "%s: gpio %d error %d", __FUNCTION__,
               fan5646_data.gpionum, ret  );
      return -EBUSY;
    }

    spin_lock_init(&fan5646_data.lock);
    gpio_direction_output( fan5646_data.gpionum, 0 );

    fan5646_data.dev = &pdev->dev;
    ret = fan5646_register_leds (&pdev->dev);
    if (ret) {
        gpio_free ( fan5646_data.gpionum );
        return ret;
    }

    ret = fan5646_create_device_files ();
    if (ret) {
        gpio_free ( fan5646_data.gpionum );
        fan5646_unregister_leds ();
        return ret;
    }
    //hrtimer_init (&fan5646_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    //fan5646_timer.function = fan5646_timer_func;

    return ret;
}

static int __exit fan5646_remove(struct platform_device *pdev)
{
    dev_dbg( &pdev->dev, "%s", __FUNCTION__ );
    gpio_set_value( fan5646_data.gpionum, 0 );
    gpio_free( fan5646_data.gpionum );
    fan5646_remove_device_files ();
    fan5646_unregister_leds ();

    return 0;
}
static struct of_device_id fan5646_dt_info[] = {
  {
    .compatible = "fairchild,fan5646",
  },
  {}
};
static struct platform_driver fan5646_driver = {
    .probe        = fan5646_probe,
    .remove       = fan5646_remove,
    .driver        = {
        .name        = "fan5646",
        .owner = THIS_MODULE,
        .of_match_table = fan5646_dt_info,
    },
};


static int __init fan5646_init(void)
{
    int ret;
    ret = platform_driver_register(&fan5646_driver);
    return ret;
}

static void __exit fan5646_exit(void)
{
    platform_driver_unregister( &fan5646_driver );
}

module_init(fan5646_init);
module_exit(fan5646_exit);

MODULE_AUTHOR("Alina Yakovleva qvdh43@motorola.com");
MODULE_DESCRIPTION("Fairchild FAN5646 LED Driver");
MODULE_LICENSE("GPL");
/* NTGRSTOP */
