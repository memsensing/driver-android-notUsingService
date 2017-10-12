/*
 * Copyright (C) 2017 MEMSENSING, Inc.
 *
 * Initial Code:
 *	Shawn Shao
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

/*
 * Definitions for MSA300 accelorometer sensor chip.
 */

#ifndef __MSA300_H__
#define __MSA300_H__

#define DRI_VER                			"1.0"

#define MSA300_DRV_NAME 				"msa300"
#define MSA300_I2C_ADDR		    		0x26		

#define MSA300_AXIS_X          			0
#define MSA300_AXIS_Y         			1
#define MSA300_AXIS_Z          			2
#define MSA300_AXES_NUM        			3
#define	MSA300_DATA_LEN					6
#define MSA300_BUFSIZE					256


/* MSA300 register address/value */
#define MSA300_REG_ID					0x01
#define MSA300_REG_X_LSB				0x02
#define MSA300_REG_X_MSB				0x03
#define MSA300_REG_Y_LSB				0x04
#define MSA300_REG_Y_MSB				0x05
#define MSA300_REG_Z_LSB				0x06
#define MSA300_REG_Z_MSB				0x07
#define MSA300_REG_RESOLUTION_FS		0X0F
#define MSA300_REG_ODR       			0x10
#define MSA300_REG_POWERMODE_BW     	0x11
#define MSA300_REG_ODR       			0x10
#define MSA300_ID						0x13

/*MSA300 para setting*/
#define MSA300_NORMAL_BW_500HZ			0x1E	/* power on normal mode */
#define MSA300_LOWPOWER_BW_500HZ		0x5E	/* power on low power mode*/
#define MSA300_SUSPEND					0x80	/* power down */

#define MSA300_ODR_250HZ				0x08	

#define MSA300_RANGE_2G_12BIT			0x04

/*MSA300 para for calibration*/

#define MSA300_AIX_HISTORY_SIZE			20
#define MSA300_STICK_THRESHOLD			2000

#define MSA300_STICK_YES				0
#define MSA300_STICK_NO					-1

#define MSA300_CALI_FINISH_YES			0
#define MSA300_CALI_FINISH_NO			-1

#define MSA300_GRAVITY_2G				1024
#define MSA300_GRAVITY_VAL				MSA300_GRAVITY_2G

#define MSA300_STABLE_THRESHOLD_XY_2G	400
#define MSA300_STABLE_THRESHOLD_Z_2G   	800
#define MSA300_STABLE_DIFF_2G			125
#define MSA300_STABLE_THRESHOLD_XY  	MSA300_STABLE_THRESHOLD_XY_2G
#define MSA300_STABLE_THRESHOLD_Z   	MSA300_STABLE_THRESHOLD_Z_2G
#define MSA300_STABLE_DIFF				MSA300_STABLE_DIFF_2G
#define MSA300_STABLE_SAMPLE_CNT_SUM	10	
#define	MSA300_STABLE_CONFIRM_YES		0
#define MSA300_STABLE_CONFIRM_NO		-1

/*MSA300 ERR code*/
#define MSA300_SUCCESS					0
#define MSA300_ERR_I2C					-11
#define MSA300_ERR_CHIPID				-12
#define MSA300_ERR_READDATA				-13

#endif /* !__MSA300_H__ */


