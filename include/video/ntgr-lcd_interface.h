/*
 *  EBI2 access for LCD controller on MDM 9x25
 *
 *  Copyright (c) 2013 Netgear
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#ifndef LCD_INTERFACE_H_
#define LCD_INTERFACE_H_

struct bus_access {
  void *aobj;
  void (*write_cmd)( void * aobj, u8 cmd, void * data, size_t datalen );
  void (*read_data)( void * aobj, u8 cmd, void * dataout, size_t datalen );
  void (*flush)( void * aobj );

  void (*set_speed)( void * aobj, long min_wr_cycle_ns );
};

struct bus_access * ebi2_probe( struct platform_device *dev );
int ebi2_remove( struct bus_access * acc );

#endif /* LCD_INTERFACE_H_ */
