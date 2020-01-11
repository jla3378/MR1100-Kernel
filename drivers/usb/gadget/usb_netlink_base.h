/************
 *
 * $Id$
 *
 * Filename:  usb_netlink_base.h - user definitions netlink driver
 *
 * Copyright: (c) 2013 Netgear, Inc.
 *            All rights reserved
 *
 ************/

#ifndef USB_NETLINK_BASE_H
#define USB_NETLINK_BASE_H


#include <linux/types.h>
#include <net/sock.h>
#include <net/netlink.h>


struct usb_cdrom_verfile_ioctl_type
{
  char   version[32];
  unsigned short enable;
  unsigned short cdpc;
  char   cd_version[4];
  char   description[64];
  unsigned short os1_sipc;
  unsigned short os1_siver;
  char   os1_sipc_desc[32];
  char   os1_proj[32];
  char   os1_ui_ver[4];
  unsigned short os2_sipc;
  unsigned short os2_siver;
  unsigned short os3_sipc;
  unsigned short os3_siver;
  unsigned short os4_sipc;
  unsigned short os4_siver;
};
struct usb_sku_info_ioctl_type
{
  char   sku[10];
};


void sendnlmsg(char *message, int msg_len, unsigned int usr_pid);

void getcdromverinfo(struct usb_cdrom_verfile_ioctl_type *info);
void getskuinfo(struct usb_sku_info_ioctl_type *info);

#endif /* USB_NETLINK_BASE_H */
