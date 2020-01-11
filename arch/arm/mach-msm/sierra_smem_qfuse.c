/* arch/arm/mach-msm/sierra_smem_qfuse.c
 *
 * OEM QFPROM read/write functions
 *
 * Copyright (C) 2013 Netgear, Inc
 * Author: brad.du@netgear.com
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

#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <soc/qcom/scm.h>
#include <asm/cacheflush.h>

#include <sierra_smem.h>

#define SCM_QFPROM_WRITE_ROW_CMD	0x3
#define SCM_QFPROM_READ_ROW_CMD	    0x5
#define QFPROM_NO_ERR               0

uint32_t api_results_g;
uint64_t rowdata_g;

ssize_t sierra_smem_qfuse_read_pid(uint32_t addr, uint64_t *fuse_pidp)
{
    int ret;

    struct {
        uint32_t   row_address;
        int32_t    addr_type;
        uint32_t  *row_data;
        uint32_t  *qfprom_api_status;
    } fuse_read_cmd_buf;

    fuse_read_cmd_buf.row_address = addr;
    fuse_read_cmd_buf.addr_type = 0;   /* raw address */
    fuse_read_cmd_buf.row_data = (void *)virt_to_phys(&rowdata_g);
    fuse_read_cmd_buf.qfprom_api_status = (void *)virt_to_phys(&api_results_g);

    ret = scm_call(SCM_SVC_FUSE, SCM_QFPROM_READ_ROW_CMD,
                    &fuse_read_cmd_buf, sizeof(fuse_read_cmd_buf), NULL, 0);

    /* Cache maintenance on the command and response buffers is taken care of 
     * by scm_call; however, callers are responsible for any other cached 
     * buffers passed over to the secure world
     * Invalidate cache since TZ touched this address range
     */
    dmac_inv_range((void *)&api_results_g, (void *)&api_results_g + sizeof(api_results_g));
    dmac_inv_range((void *)&rowdata_g, (void *)&rowdata_g + sizeof(rowdata_g));

    if (ret || api_results_g != QFPROM_NO_ERR)
    {
        pr_err("sierra_smem: read addr 0x%08X failed, ret %d, apiresult: %d\n", addr, ret, api_results_g);
        return -EFAULT;
    }
    else
    {
        pr_info("sierra_smem: read addr 0x%08X command OK %d, data %016llX\n", addr, api_results_g, rowdata_g);
        *fuse_pidp = rowdata_g;
        return sizeof(*fuse_pidp);       
    }
}
EXPORT_SYMBOL(sierra_smem_qfuse_read_pid);

static ssize_t sierra_smem_qfuse_write_row(uint32_t row_address, uint64_t rowdata)
{
    int ret;

    struct {
        uint32_t   raw_row_address;
        uint32_t  *row_data;
        uint32_t   bus_clk_khz;
        uint32_t  *qfprom_api_status;
    } fuse_write_cmd_buf;

    fuse_write_cmd_buf.raw_row_address = row_address;
    fuse_write_cmd_buf.row_data = (void *)virt_to_phys(&rowdata_g);
    fuse_write_cmd_buf.bus_clk_khz = 0;
    fuse_write_cmd_buf.qfprom_api_status = (void *)virt_to_phys(&api_results_g);
 
    rowdata_g = rowdata;
    /* Invalidate cache so that TZ will get the real data */
    dmac_inv_range((void *)&rowdata_g, (void *)&rowdata_g + sizeof(rowdata_g));
    
    ret = scm_call(SCM_SVC_FUSE, SCM_QFPROM_WRITE_ROW_CMD,
                    &fuse_write_cmd_buf, sizeof(fuse_write_cmd_buf), NULL, 0);

    /* Invalidate cache since TZ touched this address range */
    dmac_inv_range((void *)&api_results_g, (void *)&api_results_g + sizeof(api_results_g));

    if (ret || api_results_g != QFPROM_NO_ERR)
    {
        pr_err("sierra_smem: write row command failed ret %d, apiresult %d, address %08X\n", ret, api_results_g, row_address);
        return -EFAULT;
    }
    else
    {
       pr_info("sierra_smem: write row command OK %d, address %08X, data %016llX\n", api_results_g, row_address, rowdata_g);
       return sizeof(rowdata);
    }
}

ssize_t sierra_smem_qfuse_write_pid(uint32_t addr, uint64_t fuse_pid)
{
  return sierra_smem_qfuse_write_row(addr, fuse_pid);
}
EXPORT_SYMBOL(sierra_smem_qfuse_write_pid);

ssize_t sierra_smem_qfuse_write_disable(void)
{
  uint64_t fusedata;

  fusedata = 0xc004000000000;   /* set OEM config and spare region 18/19 write disable bit */
  return sierra_smem_qfuse_write_row(TZFUSE_OEM_CONFIG_ADDRESS, fusedata);
}
EXPORT_SYMBOL(sierra_smem_qfuse_write_disable);
