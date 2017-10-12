/******************** (C) COPYRIGHT 2017 MEMSENSING ****************************
 *
 * File Name          : msa300.c
 * Description        : MSA300 accelerometer sensor API
 *
 *******************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License  as  published 
 * by the Free Software Foundation.
 *
 * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
 * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
 * AS A RESULT, MEMSENSING SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH MEMSENSING PARTS.
 *

 ******************************************************************************/


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <asm/uaccess.h>


#include "sensors_io.h"
#include "msa300.h"


/* Log define  --------------------------------------------------------------------*/

#define GSE_DEBUG_ON          		1
#define GSE_DEBUG_FUNC_ON     		0

#define GSE_INFO(fmt, arg...)      	pr_warn("<<-GSENSOR INFO->> "fmt"\n", ##arg)
#define GSE_ERR(fmt, arg...)        pr_err("<<-GSENSOR ERROR->> "fmt"\n", ##arg)
#define GSE_DEBUG(fmt, arg...)		do {\
										if (GSE_DEBUG_ON)\
											pr_warn("<<-GSENSOR DEBUG->> [%d]"fmt"\n", __LINE__, ##arg);\
									   } while (0)
#define GSE_DEBUG_FUNC()			do {\
										if (GSE_DEBUG_FUNC_ON)\
											pr_debug("<<-GSENSOR FUNC->> Func:%s@Line:%d\n", __func__, __LINE__);\
									   } while (0)	
/*---------------------------------------------------------------------------------*/


static bool sensor_power = false;
/*										
struct msa300_i2c_data {
		 struct i2c_client *client;
		 struct acc_hw *hw;
		 struct hwmsen_convert	 cvt;
		 struct data_resolution *reso;
		 atomic_t				 trace;
		 atomic_t				 suspend;
		 atomic_t				 filter;
		 s16					 cali_sw[MSA300_AXES_NUM+1];

		 s8 					 offset[MSA300_AXES_NUM+1];
		 s16					 data[MSA300_AXES_NUM+1];
		 
#if defined(CONFIG_HAS_EARLYSUSPEND)
		 struct early_suspend	 early_drv;
#endif
};	
*/
/*---------------------------------------------------------------------------------*/

static struct class *class;

//字符驱动程序
static int major; //主设备号
static struct i2c_client *msa300_client;

static struct i2c_board_info __initdata msa300_i2c_info = {
	I2C_BOARD_INFO(MSA300_DRV_NAME, MSA300_I2C_ADDR) //BMA_I2C_ADDRESS(BMA2X2, PIN_HIGH))
};
	
/*---------------------------------------------------------------------------------*/

static int msa300_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{	
	u8 beg = addr;	
	int ret;

	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,	.flags = 0,
			.len = 1,	.buf = &beg
		},
		{
			.addr = client->addr,	.flags = I2C_M_RD,
			.len = len,	.buf = data,
		}
	};

	if (!client){
		GSE_DEBUG("msa300 client is null! ");
		return -EINVAL;
	}

	ret = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	
	if (ret != 2) {
		ret = -EIO;
		GSE_ERR("msa300 i2c_transfer error: (%d %p %d) %d\n", addr, data, len, ret);
	} else {
		ret = MSA300_SUCCESS;
	}
	return ret;
}

	
/*---------------------------------------------------------------------------------*/

static s16 aixsHistort[MSA300_AIX_HISTORY_SIZE];
static s16 aixsHistoryIndex = 0;
static s8  zStick = MSA300_STICK_NO;
static s16 offsetz = 0;
static s8 isCalibrated = MSA300_CALI_FINISH_NO;
static u8 stable_sample_cnt = 0;
static s16 last_x = 0, last_y = 0, last_z = 0;
static s16 avg_x = 0, avg_y = 0, avg_z = 0;

static void msa300_addAixsHistory(short z){
    aixsHistort[aixsHistoryIndex++] = z;  
	if(aixsHistoryIndex >= MSA300_AIX_HISTORY_SIZE)
		aixsHistoryIndex = 0;
}

static s8 msa300_isZStick(void)
{
	int i;
	
	for (i = 0; i < MSA300_AIX_HISTORY_SIZE; i++)
	{
	    if (abs(aixsHistort[i]) < MSA300_STICK_THRESHOLD)
	        return MSA300_STICK_NO;
	}
	
	return MSA300_STICK_YES; 
}

static int msa300_squareRoot(int val){
    int r = 0;
    int shift;
	
    val = val < 0 ? -val : val;
    
    for(shift=0;shift<32;shift+=2)
    { 
        int x=0x40000000l >> shift;
        if(x + r <= val)
        { 
            val -= x + r;
            r = (r >> 1) | x;
        } else{ 
            r = r >> 1;
        }
    }
    
    return r;
}


static int msa300_stable_condition_confirm(s16 x, s16 y, s16 z)
{
	int stable_x = 0, stable_y = 0, stable_z = 0;
	
	stable_x = ( abs(x) < MSA300_STABLE_THRESHOLD_XY  && abs(x - last_x) < MSA300_STABLE_DIFF ) ? 1 : 0 ;
	stable_y = ( abs(y) < MSA300_STABLE_THRESHOLD_XY  && abs(y - last_y) < MSA300_STABLE_DIFF ) ? 1 : 0 ;
	stable_z = ( abs(z - last_z) < MSA300_STABLE_DIFF ) ? 1 : 0 ;

	last_x = x;
	last_y = y;
	last_z = z;

	avg_x += x;
	avg_y += y;
	avg_z += z;

	stable_sample_cnt++;

	GSE_DEBUG("msa300 stable confirm sample cnt = %d ", stable_sample_cnt);
	GSE_DEBUG("msa300 stable confirm avg = %d  %d %d", x,y,z);

	if(stable_x + stable_y + stable_z != 3)
	{
		stable_sample_cnt = 0;
		avg_x = 0;
		avg_y = 0;
		avg_z = 0;
		return MSA300_STABLE_CONFIRM_NO; 
	}

	if(stable_sample_cnt < MSA300_STABLE_SAMPLE_CNT_SUM)
		return MSA300_STABLE_CONFIRM_NO;

	avg_x /= MSA300_STABLE_SAMPLE_CNT_SUM;
	avg_y /= MSA300_STABLE_SAMPLE_CNT_SUM;
	avg_z /= MSA300_STABLE_SAMPLE_CNT_SUM;

	stable_sample_cnt = 0;

	return MSA300_STABLE_CONFIRM_YES;	
}

static void msa300_calibration(s16 x, s16 y, s16 z)
{
	s16 target = 0;
	int ret = 0;
	
	GSE_DEBUG("msa300 enter calibration ");
	
	ret = msa300_stable_condition_confirm(x, y, z);
	if(ret != MSA300_STABLE_CONFIRM_YES)
		return;

	target = msa300_squareRoot(MSA300_GRAVITY_VAL * MSA300_GRAVITY_VAL - avg_x * avg_x - avg_y * avg_y);
	target = avg_z > 0 ? target : -target;

	offsetz = target - avg_z;		

	isCalibrated = MSA300_CALI_FINISH_YES;
}

static int msa300_read_data(struct i2c_client *client, s16 *x, s16 *y, s16 *z)
{
    int    ret = 0;
	u8 buf[MSA300_DATA_LEN] = {0};

	client = msa300_client;

	ret = msa300_i2c_read_block(client, MSA300_REG_X_LSB, buf, MSA300_DATA_LEN);

	if(ret != MSA300_SUCCESS)
	{
		GSE_DEBUG("msa300 read data fail ! %d ",ret);
	}
	
	else
	{
		*x = (s16)(buf[1] << 8 | buf[0]) >> 4;
		*y = (s16)(buf[3] << 8 | buf[2]) >> 4;
		*z = (s16)(buf[5] << 8 | buf[4]) >> 4;
		GSE_DEBUG("msa300 read data  x = %d  y = %d  z = %d \n", *x, *y , *z);
    }


	*z += 500;

	msa300_addAixsHistory(*z);

	GSE_DEBUG_FUNC();

	//判断是否卡轴,以及卡轴后是否恢复
   	zStick = msa300_isZStick();
	
	if(zStick == MSA300_STICK_YES)
		*z = msa300_squareRoot(MSA300_GRAVITY_VAL * MSA300_GRAVITY_VAL - (*x)*(*x) - (*y)*(*y));

	//如果未校准,未卡轴. 进行校准
	if(isCalibrated == MSA300_CALI_FINISH_NO && zStick == MSA300_STICK_NO)
		msa300_calibration(*x, *y, *z);

	*z += offsetz;

	GSE_DEBUG("msa300 calibration done ? = %d , offsetz = %d", isCalibrated, offsetz);

    return MSA300_SUCCESS;
}


/*---------------------------------------------------------------------------------*/
static int msa300_checkDeviceID(struct i2c_client *client)
{
	u8 chipId = 0;
	int ret = 0;

	chipId = MSA300_REG_ID;

	GSE_DEBUG("msa300_enter check ID function ");

	ret = msa300_i2c_read_block(client,MSA300_REG_ID,&chipId,0x01);

	GSE_DEBUG("msa300_read_chipID =  %d ",chipId);
	
	if (ret != MSA300_SUCCESS)
	{
		GSE_ERR("MSA300 Device ID read faild\n");
		return MSA300_ERR_I2C;
	}

	if(chipId != MSA300_ID)
		return MSA300_ERR_CHIPID;

	return MSA300_SUCCESS;
}

static int msa300_setReg(struct i2c_client *client, int reg, int val)
{
	u8 databuf[2] = {0};
	int ret = 0, i=0;

	databuf[1] = val;
	databuf[0] = reg;
	
	while (i++ < 3)
	{
		ret = i2c_master_send(client, databuf, sizeof(databuf));
		msleep(5);
		if(ret > 0)
			break;
	}

	if(ret <= 0)
	{
		GSE_ERR("msa300 setReg fail!  reg = %d , val = %d ",reg,val);
		return MSA300_ERR_I2C;
	}
	
	return MSA300_SUCCESS;
}

static int msa300_init(struct i2c_client *client)
{
	//struct msa300_i2c_data *obj = i2c_get_clientdata(client);
	int ret = MSA300_SUCCESS;

	GSE_DEBUG("msa300_init");

	if(!client)
		GSE_DEBUG("msa300_init_client is null!");

	ret = msa300_checkDeviceID(client);

	GSE_DEBUG("msa300_checkID ret = %d",ret);
	
	if (ret != MSA300_SUCCESS)
		return ret;

	//set powermode & bw
	ret = msa300_setReg(client, MSA300_REG_POWERMODE_BW , MSA300_LOWPOWER_BW_500HZ);
	if (ret != MSA300_SUCCESS)
		return ret;

	//set power status = on
	sensor_power = true;
	
	//set odr
	ret = msa300_setReg(client, MSA300_REG_ODR , MSA300_ODR_250HZ);
	if (ret != MSA300_SUCCESS)
		return ret;

	//set dataformat
	ret = msa300_setReg(client, MSA300_REG_RESOLUTION_FS, MSA300_RANGE_2G_12BIT);
	if (ret != MSA300_SUCCESS)
		return ret;

	//gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;

	GSE_INFO("msa300 init OK !");

	msleep(20);

	return MSA300_SUCCESS;

}


static int msa300_readSensorData(struct i2c_client *client, char *buf)
{    
	s16 x = 0, y = 0 , z = 0;
  //  struct msa_i2c_data *obj = (struct msa_i2c_data*)i2c_get_clientdata(client);
    int ret = 0;
 //   memset(databuf, 0, sizeof(unsigned char)*10);
/*
    if(!buf)
        return MSA300_ERR_READDATA;

    if(!client)
        return MSA300_ERR_I2C;
*/
    ret = msa300_read_data(msa300_client, &x, &y , &z);
    if(ret != MSA300_SUCCESS)
    {        
        return MSA300_ERR_READDATA;
    }

	//GSE_DEBUG("msa300 read data 2  x = %d  y = %d  z = %d \n", x, y , z);
	//ret = sprintf(buf, "%04x %04x %04x", x , y , z);
	//GSE_DEBUG("msa300 read date 2 %d %d %d %d %d %d %d", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5] , ret);
     
    return MSA300_SUCCESS;
}

/*---------------------------------------------------------------------------------*/

static long msa300_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{

	int err = 0;
	void __user *data;
	char buf[MSA300_BUFSIZE];

	memset(buf, 0, sizeof(buf));

	/*
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	if(!client)
		return MSA300_ERR_I2C;
	*/
	
	if(_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if(err)
	{
		GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

/*
	if(!client)
		GSE_DEBUG("msa300 unlocked ioctl client is null");
*/
	switch(cmd) {
		case GSENSOR_IOCTL_INIT: 
			msa300_init(msa300_client);
			break;

		case GSENSOR_IOCTL_READ_CHIPINFO: 
			break;

		case GSENSOR_IOCTL_READ_SENSORDATA: 
		/*	data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;
			}
		*/	
			if(!sensor_power)
				msa300_init(msa300_client);		
  
       		 msa300_readSensorData(msa300_client, buf);
		/*
	        if(copy_to_user(data, buf, sizeof(buf) + 1))
	        {
	            err = -EFAULT;
				GSE_DEBUG("msa300 read date fail ");
	            break;      
	        }  

			GSE_DEBUG("msa300 read date %d %d %d %d %d %d ", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
		*/	
	        break;

		case GSENSOR_IOCTL_READ_GAIN: 
			
			break;

		case GSENSOR_IOCTL_READ_RAW_DATA: 

			break;

		case GSENSOR_IOCTL_SET_CALI: 

			break;

		case GSENSOR_IOCTL_GET_CALI: 

			break;

		case GSENSOR_IOCTL_CLR_CALI: 

			break;
		
		default:
			return -EINVAL;
	}

	return err;
}

/*---------------------------------------------------------------------------------*/

static int msa300_open(struct inode * inode, struct file * file)
{
	return 0;	
}

/*---------------------------------------------------------------------------------*/

static struct file_operations msa300_fops = {
	.owner = THIS_MODULE,
	.open = msa300_open,
	//.release = msa300_release,
	.unlocked_ioctl = msa300_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = msa300_compat_ioctl,
#endif
};


/* I2C client driver---------------------------------------------------------------*/

static int msa300_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	GSE_DEBUG("msa i2c client addr %d ",client->addr);
	GSE_DEBUG("msa300 probe enter ");
	msa300_client = client;
	if(!client)
		GSE_DEBUG("msa300 probe clent is null ");

	GSE_DEBUG("msa i2c client addr %d ",client->addr);

	major = register_chrdev(0, MSA300_DRV_NAME, &msa300_fops);
	class = class_create(THIS_MODULE, MSA300_DRV_NAME);
	device_create(class, NULL, MKDEV(major, 0), NULL, MSA300_DRV_NAME); 

	return 0;
}


static int msa300_remove(void)
{
	device_destroy(class, MKDEV(major, 0));
	class_destroy(class);
	unregister_chrdev(major, MSA300_DRV_NAME);
	return 0;
}


static int msa300_suspend(struct i2c_client *client, pm_message_t state)
{
	return 0;
}

static int msa300_resume(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id msa300_id_table[] = {
	{ MSA300_DRV_NAME, 0 },
	{ },
};


static struct i2c_driver msa300_i2c_driver = {
	.driver		= {
		.name	= MSA300_DRV_NAME,
	},

	.probe		= msa300_probe,
	.remove		= msa300_remove,
	//.suspend	= msa300_suspend,
	//.resume		= msa300_resume,
	.id_table	= msa300_id_table,
};


/*---------------------------------------------------------------------------------*/
static int __init msa300_driver_init(void)
{
	int ret = 0;

	//ret = i2c_register_board_info(0, &msa300_i2c_info,1);
	//GSE_DEBUG("msa300_i2c_register_result %d ",ret);
	ret = i2c_add_driver(&msa300_i2c_driver);
	GSE_DEBUG("msa300_i2c_add_driver_result %d ",ret);
	return ret;
}

void __exit msa300_driver_exit(void)
{
	i2c_del_driver(&msa300_i2c_driver);
}

module_init(msa300_driver_init);
module_exit(msa300_driver_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Shawn Shao@memsensing.com");


