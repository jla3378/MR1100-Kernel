/************
 *
 * $Id$
 *
 * Filename:  netgear_ududefs - user definitions USB gadget driver
 *
 * Copyright: (c) 2013 Netgear Inc.
 *            All rights reserved
 *
 ************/
#ifndef NETGEAR_UDUDEFS_H
#define NETGEAR_UDUDEFS_H

/* Vendor specific setup request (bRequest) */
#define UD_SWI_SETUP_REQ_SET_DEVICE_POWER_STATE   0x00
#define UD_SWI_SETUP_REQ_SET_MODE_NON_MUX         0x01
#define UD_SWI_SETUP_REQ_SET_MODE_MUX             0x02
#define UD_SWI_SETUP_REQ_GET_MODE_MUX             0x03
#define UD_SWI_SETUP_REQ_GET_NDIS_SUPPORT         0x04
#define UD_SWI_SETUP_REQ_GET_NDIS_PREF            0x05
#define UD_SWI_SETUP_REQ_GET_ATTRIBUTE            0x06
#define UD_SWI_SETUP_REQ_SET_MODE_NMEA            0x07
#define UD_SWI_SETUP_REQ_GET_MODE_NMEA            0x08
#define UD_SWI_SETUP_REQ_SET_HOST_POWER_STATE     0x09
#define UD_SWI_SETUP_REQ_GET_DEV_SWOC_INFO        0x0A
#define UD_SWI_SETUP_REQ_SET_DEV_SWOC_MODE        0x0B
#define UD_SWI_SETUP_REQ_GET_CONFIG_ITEM          0x0C
#define UD_SWI_SETUP_REQ_SET_CONFIG_ITEM          0x0D
#define UD_SWI_SETUP_REQ_SET_DEVICE_RESET         0x0E
#define UD_SWI_SETUP_REQ_GET_OS_FEATURE_REQUEST   0x20
#define UD_SWI_SETUP_REQ_NULL                     0xFF


/*Data types for TRU-Install*/
typedef enum{
  SWITCH_TO_ECM,
  SWITCH_TO_CDROM,
  SWITCH_TO_HID,
}ud_usb_dev_switch;

typedef struct {
  unsigned char MsgType;
} ud_msg_notify;

#endif	/* NETGEAR_UDUDEFS_H */
