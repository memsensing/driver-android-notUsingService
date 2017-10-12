/*
*
* (C) Copyright 2008
* MediaTek <www.mediatek.com>
*
* Sensors IO command file for MT6516
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/
#ifndef SENSORS_IO_H
#define SENSORS_IO_H

#include <linux/ioctl.h>

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif



typedef struct {
	int x;
	int y;
	int z;
} SENSOR_DATA;

typedef struct {
unsigned short x;	/**< X axis */
unsigned short y;	/**< Y axis */
unsigned short z;	/**< Z axis */
} GSENSOR_VECTOR3D;


#define GSENSOR								0x85
#define GSENSOR_IOCTL_INIT					_IO(GSENSOR,  0x01)
#define GSENSOR_IOCTL_READ_CHIPINFO			_IOR(GSENSOR, 0x02, int)
#define GSENSOR_IOCTL_READ_SENSORDATA		_IOR(GSENSOR, 0x03, int)
#define GSENSOR_IOCTL_READ_GAIN				_IOR(GSENSOR, 0x05, GSENSOR_VECTOR3D)
#define GSENSOR_IOCTL_READ_RAW_DATA			_IOR(GSENSOR, 0x06, int)
#define GSENSOR_IOCTL_SET_CALI				_IOW(GSENSOR, 0x06, SENSOR_DATA)
#define GSENSOR_IOCTL_GET_CALI				_IOW(GSENSOR, 0x07, SENSOR_DATA)
#define GSENSOR_IOCTL_CLR_CALI				_IO(GSENSOR, 0x08)
#define GSENSOR_IOCTL_GET_DELAY				_IOR(GSENSOR, 0x10, int)
#define GSENSOR_IOCTL_GET_STATUS			_IOR(GSENSOR, 0x11, int)
#define GSENSOR_IOCTL_GET_DATA				_IOR(GSENSOR, 0x12, int[3])
#define GSENSOR_IOCTL_SET_DATA				_IOW(GSENSOR, 0x13, int[3])
#define GSENSOR_IOCTL_GET_TEMP				_IOR(GSENSOR, 0x14, int)
#define GSENSOR_IOCTL_GET_DANT				_IOR(GSENSOR, 0x15, int[4])
#define GSENSOR_IOCTL_READ_REG				_IOR(GSENSOR, 0x19, int)

#define GSENSOR_IOCTL_GET_LAYOUT			_IOR(GSENSOR, 0x21, int)
#define GSENSOR_IOCTL_WRITE_REG				_IOW(GSENSOR, 0x1A, int)


#ifdef CONFIG_COMPAT
#define COMPAT_GSENSOR_IOCTL_INIT			_IO(GSENSOR,  0x01)
#define COMPAT_GSENSOR_IOCTL_READ_CHIPINFO		_IOR(GSENSOR, 0x02, compat_int_t)
#define COMPAT_GSENSOR_IOCTL_READ_GAIN			_IOR(GSENSOR, 0x05, GSENSOR_VECTOR3D)
#define COMPAT_GSENSOR_IOCTL_READ_RAW_DATA		_IOR(GSENSOR, 0x06, compat_int_t)
#define COMPAT_GSENSOR_IOCTL_SET_CALI			_IOW(GSENSOR, 0x06, SENSOR_DATA)
#define COMPAT_GSENSOR_IOCTL_GET_CALI			_IOW(GSENSOR, 0x07, SENSOR_DATA)
#define COMPAT_GSENSOR_IOCTL_CLR_CALI			_IO(GSENSOR, 0x08)
#define COMPAT_GSENSOR_IOCTL_READ_SENSORDATA		_IOR(GSENSOR, 0x03, compat_int_t)

#define COMPAT_GSENSOR_IOCTL_GET_DELAY			_IOR(GSENSOR, 0x10, compat_int_t)
#define COMPAT_GSENSOR_IOCTL_GET_DANT			_IOR(GSENSOR, 0x15, compat_int_t[4])
#define COMPAT_GSENSOR_IOCTL_READ_REG			_IOR(GSENSOR, 0x19, compat_int_t)
#define COMPAT_GSENSOR_IOCTL_GET_LAYOUT			_IOR(GSENSOR, 0x21, compat_int_t)
#define COMPAT_GSENSOR_IOCTL_WRITE_REG			_IOW(GSENSOR, 0x1A, compat_int_t)

#define COMPAT_GSENSOR_IOCTL_GET_STATUS			_IOR(GSENSOR, 0x11, int)
#define COMPAT_GSENSOR_IOCTL_GET_DATA			_IOR(GSENSOR, 0x12, int[3])
#define COMPAT_GSENSOR_IOCTL_SET_DATA			_IOW(GSENSOR, 0x13, int[3])
#define COMPAT_GSENSOR_IOCTL_GET_TEMP			_IOR(GSENSOR, 0x14, int)
#endif

#endif
