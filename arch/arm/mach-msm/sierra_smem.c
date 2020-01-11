/* arch/arm/mach-msm/sierra_smem.c
 *
 * Copyright (C) 2013 Netgear, Inc
 * Copyright (C) 2012 Sierra Wireless, Inc
 * Author: Brad Du <bdu@sierrawireless.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>

#include <sierra_smem.h>

void __iomem *smem_virtual_addr = 0;

void __iomem *sierra_smem_vitual_addr_get(void)
{
    if(!smem_virtual_addr)
    {
        smem_virtual_addr = ioremap_nocache(SIERRA_SMEM_BASE_PHYS, SIERRA_SMEM_SIZE);
        if(!smem_virtual_addr)
        {
            pr_err("sierra_smem_vitual_addr_init error");
        }
    }
    return smem_virtual_addr;
}

static ssize_t sierra_smem_read(struct file *fp, char __user *buf,
            size_t count, loff_t *posp)
{
    unsigned char *memp;
    int ret;
    loff_t pos = *posp;
    ssize_t len;
    uint64_t fusedata;
    
    memp = smem_virtual_addr;

    if(!memp)
    {
      return -EFAULT;
    }
    
    if(pos < SIERRA_SMEM_SIZE)
    {
        /* normal SMEM read */
        len = min(count, (size_t)(SIERRA_SMEM_SIZE - pos));
    
        memp += pos;
        ret = copy_to_user(buf, memp, len);
    
        if(ret)
        {
            pr_err("%s: copy to user failed\n", __func__);
            return -EFAULT;
        }
        else
        {
            *posp += len;
        }
    }
    else if(pos == TZFUSE_OEM_PID_ROW_CMD)
    {
        /* QFUSE Product ID read command */
        len = sierra_smem_qfuse_read_pid(TZFUSE_OEM_PID_ADDRESS, &fusedata);

        if(len > 0)
        {
            ret = copy_to_user(buf, (void *)&fusedata, sizeof(fusedata));

            if(ret)
            {
                pr_err("%s: copy to user failed\n", __func__);
                return -EFAULT;
            }
        }
    }
    else if(pos == TZFUSE_SPARE18_PID_ROW_CMD)
    {
        /* QFUSE Product ID read command */
        len = sierra_smem_qfuse_read_pid(TZFUSE_SPARE_REGION18_ADDRESS, &fusedata);

        if(len > 0)
        {
            ret = copy_to_user(buf, (void *)&fusedata, sizeof(fusedata));

            if(ret)
            {
                pr_err("%s: copy to user failed\n", __func__);
                return -EFAULT;
            }
        }
    }
    else if(pos == TZFUSE_SPARE19_PID_ROW_CMD)
    {
        /* QFUSE Product ID read command */
        len = sierra_smem_qfuse_read_pid(TZFUSE_SPARE_REGION19_ADDRESS, &fusedata);

        if(len > 0)
        {
            ret = copy_to_user(buf, (void *)&fusedata, sizeof(fusedata));

            if(ret)
            {
                pr_err("%s: copy to user failed\n", __func__);
                return -EFAULT;
            }
        }
    }
    else
    {
        /* other offset not supported */
        len = -EFAULT;
    }

    return len;
}

/* the write function will be used mainly to write errdump related fields
 * offset field will indicate which field to update
 */
static ssize_t sierra_smem_write(struct file *fp, const char __user *buf,
             size_t count, loff_t *posp)
{
    int r;
    unsigned char cmddata[ERROR_STRING_LEN]  __attribute__((aligned(4)));
    loff_t pos = *posp;
    struct stackframe *frp;
    uint32_t time_stamp;
    uint64_t fusedata;
 
    if (count > sizeof(cmddata))
    {
        count = sizeof(cmddata);
    }

    r = copy_from_user(cmddata, buf, count);
    if (r) 
    {
        printk(KERN_ERR "sierra_smem_write - copy_from_user failed %d\n", r);
        return -EFAULT;
    }

    if(pos == ERDUMP_SAVE_CMD_START)
    {
        sierra_smem_errdump_save_start();        
        /* timestamp in cmddata */
        memcpy((void *)&time_stamp, cmddata, sizeof(time_stamp));
        sierra_smem_errdump_save_timestamp(time_stamp);        
    }
    else if(pos == ERDUMP_SAVE_CMD_ERRSTR)
    {
        sierra_smem_errdump_save_errstr(cmddata);        
    }
    else if(pos == ERDUMP_SAVE_CMD_FMTSTR)
    {
        sierra_smem_errdump_save_fmtstr(cmddata);        
    }
    else if(pos == ERDUMP_SAVE_CMD_FRAME)
    {
        /* cast to frame pointer, can case since cmddata is 4 byte aligned */
        frp = (struct stackframe *)&cmddata;
        sierra_smem_errdump_save_frame(0, frp);        
    }
    else if(pos == TZFUSE_OEM_PID_ROW_CMD)
    {
        /* QFUSE Product ID write command */
        memcpy((void *)&fusedata, cmddata, sizeof(fusedata));
        count = sierra_smem_qfuse_write_pid(TZFUSE_OEM_PID_ADDRESS, fusedata);
    }
    else if(pos == TZFUSE_SPARE18_PID_ROW_CMD)
    {
        /* QFUSE Product ID write command */
        memcpy((void *)&fusedata, cmddata, sizeof(fusedata));
        count = sierra_smem_qfuse_write_pid(TZFUSE_SPARE_REGION18_ADDRESS, fusedata);
    }
    else if(pos == TZFUSE_SPARE19_PID_ROW_CMD)
    {
        /* QFUSE Product ID write command */
        memcpy((void *)&fusedata, cmddata, sizeof(fusedata));
        count = sierra_smem_qfuse_write_pid(TZFUSE_SPARE_REGION19_ADDRESS, fusedata);
    }
    else if(pos == TZFUSE_WRITE_DISABLE_CMD)
    {
        /* QFUSE write disable command */
        count = sierra_smem_qfuse_write_disable();
    }
    else if(pos == ERDUMP_CLR_ALL_CMD)
    {
        /* same with gcdump clear command */
        /* normal SMEM write */  
        if (!sierra_smem_errdump_clear_all())
        {
          count = -EFAULT;
        }
    }
    else
    {
        /* SMEM write is not implemented yet */
        count = -EFAULT;
    }

    return count;
}
    
static int sierra_smem_open(struct inode *inode, struct file *file)
{
    if(sierra_smem_vitual_addr_get() == 0)
    {
      return -EFAULT;
    }
    else
    {
      return 0;
    }
}

static int sierra_smem_release(struct inode *inode, struct file *file)
{
    return 0;
}

static struct file_operations sierra_smem_fops = {
    .owner = THIS_MODULE,
    .read = sierra_smem_read,
    .write = sierra_smem_write,
    .llseek = default_llseek,
    .open = sierra_smem_open,
    .release = sierra_smem_release,
};

static struct miscdevice sierra_smem_misc = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "sierra_smem",
    .fops = &sierra_smem_fops,
};

static int __init sierra_smem_init(void)
{
    (void)sierra_smem_vitual_addr_get();
    return misc_register(&sierra_smem_misc);
}

static void __exit sierra_smem_exit(void)
{
    misc_deregister(&sierra_smem_misc);
}

module_init(sierra_smem_init);
module_exit(sierra_smem_exit);

MODULE_AUTHOR("Brad Du <bdu@sierrawireless.com>");
MODULE_DESCRIPTION("Sierra SMEM driver");
MODULE_LICENSE("GPL v2");
