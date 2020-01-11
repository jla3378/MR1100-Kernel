/*
 *
 * Copyright (c) 2013, 2014 Netgear Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  NTGRSTART
 */

/* Turn on debug messages */
#define DEBUG
//#define VERBOSE_DEBUG

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/pm.h>

#include <linux/gpio.h>
#include <mach/irqs.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#include <linux/device.h>
#include <linux/regulator/consumer.h>

#define ADC_MIN_X 50
#define ADC_MAX_X 2000
#define ADC_MIN_Y 150
#define ADC_MAX_Y 2000

struct msg2133 {
  struct input_dev  *input;
  char      phys[32];
  struct delayed_work work;

  struct i2c_client *client;

  int     irq;
  atomic_t    suspend_cnt;
  int     gpion;
  int     gpiorst;
};

enum power_save_state
{
  POWER_SAVE_OFF,
  POWER_SAVE_ON
};


#define KEY_N1  KEY_F2
#define KEY_N2  KEY_F1
#define KEY_N3  KEY_F3


/* Control power save */
static unsigned int power_save = POWER_SAVE_OFF;

/* Variables used to hold values for minimum and maximum x and y values */
typedef int min_x_t;
typedef int max_x_t;
typedef int min_y_t;
typedef int max_y_t;

static min_x_t min_x = ADC_MIN_X;
static max_x_t max_x = ADC_MAX_X;
static min_y_t min_y = ADC_MIN_Y;
static max_y_t max_y = ADC_MAX_Y;
static struct input_dev *xy_input_dev = NULL;

#define param_check_min_x_t(name, p) __param_check(name, p, min_x_t);
#define param_check_max_x_t(name, p) __param_check(name, p, max_x_t);
#define param_check_min_y_t(name, p) __param_check(name, p, min_y_t);
#define param_check_max_y_t(name, p) __param_check(name, p, max_y_t);

static int param_set_min_x(const char *val, const struct kernel_param *kp)
{
  int rv = param_set_int( val, kp );
  if((!rv)&&(xy_input_dev != NULL))
  {
    printk( KERN_INFO "Set min_x to %d\n", min_x);
    input_set_abs_params(xy_input_dev, ABS_X, min_x, max_x, 0, 0);
    input_set_abs_params(xy_input_dev, ABS_MT_POSITION_X, min_x, max_x, 0, 0);
  }
  return 0;
}
static const struct kernel_param_ops param_ops_min_x_t = {
  .set = param_set_min_x,
  .get = param_get_int,
  .free = NULL,
};
module_param(min_x, min_x_t, 0644);
MODULE_PARM_DESC(min_x, "Left LCD screen margin");

static int param_set_max_x(const char *val, const struct kernel_param *kp)
{
  int rv = param_set_int( val, kp );
  if((!rv)&&(xy_input_dev != NULL))
  {
    printk( KERN_INFO "Set max_x to %d\n", max_x);
    input_set_abs_params(xy_input_dev, ABS_X, min_x, max_x, 0, 0);
    input_set_abs_params(xy_input_dev, ABS_MT_POSITION_X, min_x, max_x, 0, 0);
  }
  return rv;
}
static const struct kernel_param_ops param_ops_max_x_t = {
  .set = param_set_max_x,
  .get = param_get_int,
  .free = NULL,
};
module_param(max_x, max_x_t, 0644);
MODULE_PARM_DESC(max_x, "Right LCD screen margin");

static int param_set_min_y(const char *val, const struct kernel_param *kp)
{
  int rv = param_set_int( val, kp );
  if((!rv)&&(xy_input_dev != NULL))
  {
    printk( KERN_INFO "Set min_y to %d\n", min_y);
    input_set_abs_params(xy_input_dev, ABS_Y, min_y, max_y, 0, 0);
    input_set_abs_params(xy_input_dev, ABS_MT_POSITION_Y, min_y, max_y, 0, 0);
  }
  return rv;
}
static const struct kernel_param_ops param_ops_min_y_t = {
  .set = param_set_min_y,
  .get = param_get_int,
  .free = NULL,
};
module_param(min_y, min_y_t, 0644);
MODULE_PARM_DESC(min_y, "Top LCD screen margin");

static int param_set_max_y(const char *val, const struct kernel_param *kp)
{
  int rv = param_set_int( val, kp );
  if((!rv)&&(xy_input_dev != NULL))
  {
    printk( KERN_INFO "Set max_y to %d\n", max_y);
    input_set_abs_params(xy_input_dev, ABS_Y, min_y, max_y, 0, 0);
    input_set_abs_params(xy_input_dev, ABS_MT_POSITION_Y, min_y, max_y, 0, 0);
  }
  return rv;
}
static const struct kernel_param_ops param_ops_max_y_t = {
  .set = param_set_max_y,
  .get = param_get_int,
  .free = NULL,
};
module_param(max_y, max_y_t, 0644);
MODULE_PARM_DESC(max_y, "Bottom LCD screen margin");


/************
 *
 * Name:     msg2133_free_gpio
 *
 * Purpose: To release interrupt gpio.
 *
 * Parms:   none
 *
 * Return:   none
 *
 * Abort:   none
 *
 ************/
static void msg2133_free_gpio(struct msg2133 * ts)
{
  dev_dbg(&ts->client->dev, "%s\n", __FUNCTION__ );

  if( gpio_is_valid( ts->gpion) )
    gpio_free(ts->gpion);
  if( gpio_is_valid( ts->gpiorst) )
    gpio_free(ts->gpiorst);
}

/************
 *
 * Name:  msg2133_init_gpio
 *
 * Purpose: Initialize correct gpio as input so that it can serve as irq line
 *    when PINTDAV pin on the i2c controller is pulled low.
 *
 * Parms: None
 *
 * Return:  0 - gpio initialized
 *    non zero - error
 *
 * Abort: none
 *
 ************/
static int msg2133_init_gpio(struct msg2133 * ts)
{
  int status;

  dev_dbg(&ts->client->dev, "%s", __FUNCTION__);

  if( gpio_is_valid(ts->gpiorst) )
  {
    status = gpio_request( ts->gpiorst, "touch_reset" );
    if(status != 0)
    {
      dev_err(&ts->client->dev, "%s: Couldn't reserve rst %d, status=%d\n",
            __FUNCTION__, ts->gpiorst, status);
      goto release_gpio;
    }
    status = gpio_direction_output( ts->gpiorst, 1 );
    if(status != 0)
    {
      dev_err(&ts->client->dev,"%s: gpio_direction_output() failed, status=%d",__FUNCTION__, status);
      goto release_gpio;
    }
    msleep( 50 );
  }

  if( gpio_is_valid(ts->gpion) )
  {
    status = gpio_request(ts->gpion, "touchscr_irq");
    if(status != 0)
    {
      dev_err(&ts->client->dev, "%s: Couldn't reserve gpio %d, status=%d\n",
            __FUNCTION__, ts->gpion, status);
      goto release_gpio;
    }

    /* Set the direction of the pin reading the PINTDAV signal (irq) to input */
    status = gpio_direction_input(ts->gpion);
    if(status != 0)
    {
      dev_err(NULL,"%s: gpio_direction_input() failed, status=%d",__FUNCTION__, status);
      goto release_gpio;
    }
  }

  return (status);

release_gpio:

  msg2133_free_gpio( ts );
  return status;
}

/* Read touch position data from controller */
static int msg2133_read_values(struct msg2133 *tsc, u8 * dp, u32 len )
{
  int i;
  int sum = 0;
  int rcvd;

  rcvd = i2c_master_recv( tsc->client, dp, len );

  if( rcvd != len )
    return -1;

  for( i = 0; i < rcvd-1; i++ )
  {
    sum += dp[i];
  }

  if( ((u8)(-sum)) != dp[rcvd-1] )
    return -2;

  return 0;
}

static void msg2133_work(struct work_struct *work)
{
  struct msg2133 *ts =
    container_of(to_delayed_work(work), struct msg2133, work);
  u8 regs[8];
  int err;

  err = msg2133_read_values(ts, regs, sizeof(regs) );

#ifdef VERBOSE_DEBUG
  dev_dbg( &ts->client->dev, "%d: %02X %02X %02X %02X %02X %02X %02X %02X", err,
                      regs[0],
                      regs[1],
                      regs[2],
                      regs[3],
                      regs[4],
                      regs[5],
                      regs[6],
                      regs[7]
                      );
#endif
  if( !err && regs[0] == 0x52 )
  {
    int x, y, dx, dy;

    x =  ((regs[1] & 0xF0) << 4) | regs[2];
    y =  ((regs[1] & 0x0F) << 8) | regs[3];
    dx = ((regs[4] & 0xF0) << 4) | regs[5];
    dy = ((regs[4] & 0x0F) << 8) | regs[6];


    if( x == 0xFFF && y == 0xFFF && regs[5] != 0xFF )
    {
      dev_dbg( &ts->client->dev, "Buttons: %x", regs[5] );

      /* buttons */
      /* Bottom button */
      if( regs[5] & 0x01 )
        input_report_key(ts->input, KEY_N1, 1);
      
      /* Middle - unused */
      if( regs[5] & 0x02 )
        input_report_key(ts->input, KEY_N3, 1);

      /* Top most - (Home) button */
      if( regs[5] & 0x04 )
        input_report_key(ts->input, KEY_N2, 1);
        
      if( regs[5] == 0 ) {
        input_report_key(ts->input, KEY_N1, 0);
        input_report_key(ts->input, KEY_N2, 0);
        input_report_key(ts->input, KEY_N3, 0);
      }

    }
    if( dx == 0 && dy == 0 ) {

      input_report_abs(ts->input, ABS_Y, y);
      input_report_abs(ts->input, ABS_X, x);
      input_report_key(ts->input, BTN_TOUCH, 1);

      dev_dbg( &ts->client->dev, "Point: %d, %d", x, y );

    } else if( dx != 4095 && dy != 4095 ) {

      input_report_abs(ts->input, ABS_MT_POSITION_Y, y);
      input_report_abs(ts->input, ABS_MT_POSITION_X, x);
      input_mt_sync(ts->input);
      input_report_abs(ts->input, ABS_MT_POSITION_Y, dy );
      input_report_abs(ts->input, ABS_MT_POSITION_X, dx );
      input_mt_sync(ts->input);

      input_report_key(ts->input, BTN_TOUCH, 1);

      dev_dbg( &ts->client->dev, "2 Points: %d, %d - %d, %d", x, y, dx, dy );

    } else {

      input_report_key(ts->input, BTN_TOUCH, 0);
      dev_dbg( &ts->client->dev, "Release" );
    }

    input_sync(ts->input);
  }

  enable_irq(ts->irq);
}

static irqreturn_t msg2133_irq(int irq, void *handle)
{
  struct msg2133 *ts = handle;

  disable_irq_nosync(ts->irq);
  schedule_work( &ts->work.work );

  return IRQ_HANDLED;
}

static void msg2133_free_irq(struct msg2133 *ts)
{
  free_irq(ts->irq, ts);
  if (cancel_delayed_work_sync(&ts->work)) {
    /*
     * Work was pending, therefore we need to enable
     * IRQ here to balance the disable_irq() done in the
     * interrupt handler.
     */
    enable_irq(ts->irq);
  }
}

#ifdef CONFIG_PM
static int msg2133_suspend(struct device *dev)
{
  int cnt;
  struct msg2133  *ts = dev_get_drvdata(dev);

  dev_dbg( dev, "%s", __FUNCTION__ );

  /* track re-entries */
  cnt = atomic_inc_return( &ts->suspend_cnt );
  if (cnt == 1) {
    dev_dbg( dev, "%s - Disabling", __FUNCTION__ );
    disable_irq(ts->irq);

    input_report_key(ts->input, BTN_TOUCH, 0);
    input_report_key(ts->input, KEY_N1, 0);
    input_report_key(ts->input, KEY_N2, 0);
    input_report_key(ts->input, KEY_N3, 0);

    if( gpio_is_valid(ts->gpiorst) )
      gpio_direction_output( ts->gpiorst, 0 );

  }

  return 0;
}

static int msg2133_resume(struct device *dev)
{
  int cnt;
  struct msg2133  *ts = dev_get_drvdata(dev);

  dev_dbg( dev, "%s", __FUNCTION__ );

  /* track re-entries */
  cnt = atomic_dec_return( &ts->suspend_cnt );
  if ( cnt == 0 ) {

    if( gpio_is_valid(ts->gpiorst) )
      gpio_direction_output( ts->gpiorst, 1 );

    dev_dbg( dev, "%s - Enabling", __FUNCTION__ );
    enable_irq(ts->irq);
  }

  return 0;
}

static const struct dev_pm_ops msg2133_pm_ops = {
  .suspend  = msg2133_suspend,
  .resume   = msg2133_resume,
};
#endif

static ssize_t show_pwr_save(struct device *dev, struct device_attribute *attr,
         char *buf)
{
  int n = 0;

  if (attr == NULL || buf == NULL) {
    dev_err(dev, "EINVAL" );
    return 0;
  }

  n = scnprintf(buf, PAGE_SIZE, "%d", power_save);

  return n;
}
static ssize_t store_pwr_save(struct device *dev,
             struct device_attribute *attr,
             const char *buf, size_t count)
{
  unsigned int state;

  if (attr == NULL || buf == NULL) {
    dev_err( dev, "EINVAL" );
    goto done;
  }

  if ((sscanf(buf, "%u", &state) != 1) ||
      ((state != POWER_SAVE_OFF) && (state != POWER_SAVE_ON))) {
    dev_err(dev, "input data invalid" );
    goto done;
  }

  if (state != power_save) {
    power_save = state;
    if (power_save == POWER_SAVE_ON) {
      msg2133_suspend(dev);
    }
    else if (power_save == POWER_SAVE_OFF) {
      msg2133_resume(dev);
    }
  }

done:
  return count;
}
static DEVICE_ATTR(pwr_save, S_IRUSR | S_IWUSR, show_pwr_save, store_pwr_save);

static ssize_t msg2133_version_show( struct device *dev, struct device_attribute *attr, char *buf )
{
  struct i2c_client * client = to_i2c_client( dev );
  u8 vercmd[] = { 0x53, 0x00, 0x2A };
  u8 rsp[4] = { 0 }; /* init to 0 */
  int err;

  /* Make sure the device is active */
  msg2133_resume(dev);

  /* Let it stabilize - in case we are turing it on */
  msleep( 100 );

  err = i2c_master_send( client, vercmd, sizeof(vercmd) );
  err = i2c_master_recv( client, rsp, sizeof(rsp) );

  msg2133_suspend(dev);

  return sprintf( buf, "%02x%02x.%02x%02x\n", rsp[1], rsp[0], rsp[3], rsp[2] );
}

static DEVICE_ATTR( version, S_IRUSR, msg2133_version_show, NULL );
/**
 * msg2133_sysfs_create_files: initializes the attribute interface
 * @dev: device
 *
 * This function returns an error code
 */
__maybe_unused static int msg2133_sysfs_create_files(struct device *dev)
{
  int retval = 0;

  if (dev == NULL)
    return -EINVAL;
  /* Create sysfs to /sys/bus/i2c/devices/X-00XX/ */
  retval = device_create_file(dev, &dev_attr_pwr_save);
  if( !retval )
    retval = device_create_file(dev, &dev_attr_version);

  return retval;
}

/**
 * msg2133_sysfs_remove_files: destroys the attribute interface
 * @dev: device
 *
 * This function returns an error code
 */
__maybe_unused static int msg2133_sysfs_remove_files(struct device *dev)
{
  if (dev == NULL)
    return -EINVAL;
  device_remove_file(dev, &dev_attr_pwr_save);
  device_remove_file(dev, &dev_attr_version);
  return 0;
}

static int  msg2133_probe(struct i2c_client *client,
           const struct i2c_device_id *id)
{
  struct msg2133 *ts;
  struct input_dev *input_dev;
  int err;
  u8 rx_buffer[8];
  enum of_gpio_flags flags;
  struct regulator * reg = NULL;
  const char * regName = NULL;
  u32 volt;

  dev_dbg(&client->dev,"%s", __FUNCTION__);

  /* Compatibility check */
  if (!i2c_check_functionality(client->adapter,
             I2C_FUNC_SMBUS_READ_WORD_DATA)) {

    dev_err(&client->dev, "%s exit with -EIO",__FUNCTION__);
    return -EIO;
  }

  of_property_read_string( client->dev.of_node, "vdd-vreg", &regName );
  reg = regulator_get( NULL, regName );
  if( !IS_ERR(reg) ) {

    of_property_read_u32( client->dev.of_node, "vdd-voltage", &volt );
    regulator_set_voltage( reg, volt, volt);
    dev_dbg(&client->dev, "Setting up regulator %s voltage %duV.", regName, volt );
    if( regulator_enable( reg ) )
      dev_err( &client->dev, "Failed to enable %s.", regName );
    msleep( 50 );

  }
  else
  {
    dev_dbg(&client->dev, "Didn't get regulator." );
  }

  ts = kzalloc(sizeof(struct msg2133), GFP_KERNEL);
  input_dev = input_allocate_device();
  if (!ts || !input_dev) {
    err = -ENOMEM;
    goto err_free_mem;
  }
  input_dev->id.vendor = 0x1b20; /* Mstar Semiconductor - USB vendor ID*/
  input_dev->id.product = 0x2133;

  ts->client = client;
  /* get GPIO for pen down */
  ts->gpion = of_get_named_gpio_flags( client->dev.of_node,
          "pen-down", 0, &flags);
  if( !gpio_is_valid(ts->gpion))
  {
    dev_err( &client->dev,"gpio 'pen-down' not specified in dts node" );
  }
  ts->gpiorst =of_get_named_gpio_flags( client->dev.of_node, "rst-gpio", 0, &flags);
  if( !gpio_is_valid(ts->gpiorst) )
  {
    dev_err( &client->dev,"'rst-gpio' not specified in dts node" );
  }


  ts->irq = irq_of_parse_and_map( client->dev.of_node, 0 );
  if( !ts->irq ) {
    dev_err(&client->dev,"Couldn't get irq" );
    err = -ENOMEM;
    goto err_free_mem;
  }

  ts->input = input_dev;
  INIT_DELAYED_WORK(&ts->work, msg2133_work);

  atomic_set( &ts->suspend_cnt, 0 );

  /* Store pointer to input device for later use when x and y limits are modified */
  xy_input_dev = input_dev;

  snprintf(ts->phys, sizeof(ts->phys),
     "%s/input0", dev_name(&client->dev));

  input_dev->name = "MSG2133 Touchscreen";
  input_dev->phys = ts->phys;
  input_dev->id.bustype = BUS_I2C;

  input_dev->evbit[BIT_WORD(EV_KEY)] |= BIT_MASK(EV_KEY);
  input_dev->evbit[BIT_WORD(EV_ABS)] |= BIT_MASK(EV_ABS);

  input_dev->keybit[BIT_WORD(BTN_TOUCH)] |= BIT_MASK(BTN_TOUCH);
  input_dev->keybit[BIT_WORD(KEY_N1)] |= BIT_MASK(KEY_N1);
  input_dev->keybit[BIT_WORD(KEY_N2)] |= BIT_MASK(KEY_N2);
  input_dev->keybit[BIT_WORD(KEY_N3)] |= BIT_MASK(KEY_N3);

  input_set_abs_params(input_dev, ABS_X, min_x, max_x, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_POSITION_X, min_x, max_x, 0, 0);
  input_set_abs_params(input_dev, ABS_Y, min_y, max_y, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_POSITION_Y, min_y, max_y, 0, 0);

  i2c_set_clientdata(client, ts);

  if (msg2133_sysfs_create_files(&client->dev)) {
    dev_err(&client->dev, "Create sysfs failed");
  }
  /* Initialize gpio to serve as interrupt input only after confirming
   * the device is there
   */
  if(0 != msg2133_init_gpio(ts)){
    err = -ENXIO;
    goto err_free_gpio;
  }
  /*
   * Assumption here: if gpio is active low, the trigger is on falling edge
   */
  err = request_irq(ts->irq, msg2133_irq, IRQF_TRIGGER_RISING,
        client->dev.driver->name, ts);
  if (err < 0) {
    dev_err(&client->dev, "irq %d busy?", ts->irq);
    goto err_free_gpio;
  }

  err = msg2133_read_values( ts, rx_buffer, sizeof(rx_buffer) );
  if (err < 0) {
    dev_err(&client->dev, "Send Setup error");
    goto err_free_gpio;
  }

  err = input_register_device(input_dev);
  if (err)
    goto err_free_irq;

  return 0;

err_free_irq:
  msg2133_free_irq(ts);

err_free_gpio:
  msg2133_free_gpio( ts );

  msg2133_sysfs_remove_files( &client->dev );

err_free_mem:

  input_free_device(input_dev);
  kfree(ts);

  dev_err(&client->dev, "%s exit with err %d",__FUNCTION__, err);

  return err;
}

static int msg2133_remove(struct i2c_client *client)
{
  struct msg2133  *ts = i2c_get_clientdata(client);
  dev_dbg(&client->dev, "%s", __FUNCTION__);

  msg2133_sysfs_remove_files(&client->dev);
  msg2133_free_irq(ts);
  msg2133_free_gpio(ts);

  input_unregister_device(ts->input);
  kfree(ts);

  return 0;
}
static struct of_device_id msm_spi_dt_match[] = {
  {
    .compatible = "msg2133",
  },
  {}
};

static const struct i2c_device_id msg2133_idtable[] = {
  { "msg2133", 0x4c },
  { }
};

MODULE_DEVICE_TABLE(i2c, msg2133_idtable);

static struct i2c_driver msg2133_driver = {
  .driver = {
    .owner  = THIS_MODULE,
    .name = "msg2133",
    .of_match_table = msm_spi_dt_match,

#ifdef CONFIG_PM
    .pm = &msg2133_pm_ops,
#endif
  },
  .id_table = msg2133_idtable,
  .probe    = msg2133_probe,
  .remove   = msg2133_remove,
};

static int __init msg2133_init(void)
{
  dev_dbg( NULL, "%s", __FUNCTION__);
  return i2c_add_driver(&msg2133_driver);
}

static void __exit msg2133_exit(void)
{
  dev_dbg( NULL, "%s", __FUNCTION__);
  i2c_del_driver(&msg2133_driver);
}

module_init(msg2133_init);
module_exit(msg2133_exit);

MODULE_DESCRIPTION("MSG2133 TouchScreen Driver");
MODULE_LICENSE("GPL");
