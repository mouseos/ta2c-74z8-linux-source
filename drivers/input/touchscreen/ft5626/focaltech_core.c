/*
 *
 * FocalTech ftxxxx TouchScreen driver.
 * 
 * Copyright (c) 2010-2015, Focaltech Ltd. All rights reserved.
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

 /*******************************************************************************
*
* File Name: Ftxxxx_ts.c
*
*    Author: Tsai HsiangYu
*
*   Created: 2015-01-29
*
*  Abstract:
*
* Reference:
*
*******************************************************************************/

/*******************************************************************************
* 1.Included header files
*******************************************************************************/
#include "focaltech_core.h"
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>/*ori #include <mach/irqs.h>*/
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/input/mt.h>
#include <linux/switch.h>
#include <linux/gpio.h>

#include <linux/acpi.h>
#include <linux/acpi_gpio.h>

/*ori #include <mach/gpio.h>*/
/*
#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>
#include <linux/gpio.h>
*/
//#include "linux/input/proximity_class.h"
//#include <linux/ft3x17.h>
#if TPD_PROXIMITY
	#include <linux/hwmsensor.h>
	#include <linux/hwmsen_dev.h>
	#include <linux/sensors_io.h>
#endif

/* Add early suspend and late resume for Goodix GT9xx touchscreen by xmwuwh@20160122 */
#ifdef CONFIG_PM_SLEEP
#include <linux/power_hal_sysfs.h>
int gtp_is_suspend = 0;
static void fts_ts_hal_suspend(struct device *dev);
static void fts_ts_hal_resume(struct device *dev);
#endif

#define GTP_RST_PORT    366// 128 for baytrail
#define GTP_INT_PORT    360// 133 for baytrail

#define GTP_INT_IRQ     gpio_to_irq(GTP_INT_PORT)

//#if FTS_CTL_IIC_EN
//	#include "focaltech_ctl.h"
//#endif
//#if SYSFS_DEBUG_EN
//	#include "focaltech_ex_fun.h"
//#endif


/*******************************************************************************
* 2.Private constant and macro definitions using #define
*******************************************************************************/
#define FTS_NAME							"ft5626"
#if GTP_ESD_PROTECT
	#define TPD_ESD_CHECK_CIRCLE        			200
#endif
#if TPD_PROXIMITY
	#define APS_ERR(fmt,arg...)                 pr_info("<<proximity>> "fmt"\n",##arg)
	#define TPD_PROXIMITY_DEBUG(fmt,arg...) pr_info("<<proximity>> "fmt"\n",##arg)
	#define TPD_PROXIMITY_DMESG(fmt,arg...) pr_info("<<proximity>> "fmt"\n",##arg)
#endif


/*******************************************************************************
* 3.Private enumerations, structures and unions using typedef
*******************************************************************************/
#ifdef CONFIG_HAS_EARLYSUSPEND
	static struct early_suspend focal_early_suspend = 
	{
		.level = EARLY_SUSPEND_LEVEL_DISABLE_FB,
		.suspend = fts_ts_suspend,
		.resume = fts_ts_resume,
	};
#endif
/*******************************************************************************
* 4.Static variables
*******************************************************************************/
//static bool touch_down_up_status;
static u8 buf_addr[2] = { 0 };
static u8 buf_value[2] = { 0 };
#if GTP_ESD_PROTECT
	static int count_irq = 0;
	static unsigned long esd_check_circle = TPD_ESD_CHECK_CIRCLE;
	static u8 run_check_91_register = 0;
#endif
#if TPD_PROXIMITY
	static u8 tpd_proximity_flag                       = 0;
	//add for tpd_proximity by wangdongfang
	static u8 tpd_proximity_flag_one               = 0; 
	//0-->close ; 1--> far away
	static u8 tpd_proximity_detect                   = 1;
#endif
#if GTP_ESD_PROTECT
	static struct delayed_work gtp_esd_check_work;
	static struct workqueue_struct *gtp_esd_check_workqueue = NULL;
#endif

/*******************************************************************************
* 5.Global variable or extern global variabls/functions
*******************************************************************************/
static void fts_ts_suspend(struct early_suspend *handler);
static void fts_ts_resume(struct early_suspend *handler);
static DEFINE_MUTEX(i2c_rw_access);

#if GTP_ESD_PROTECT
	static void gtp_esd_check_func(struct work_struct *);
#endif
int fts_init_success = 0;
unsigned char IC_FW;
struct fts_ts_data *fts_wq_data;
struct i2c_client *fts_i2c_client;
struct input_dev *fts_input_dev;
/*******************************************************************************
* 6.Static function prototypes
*******************************************************************************/

/************************************************************************
* Name: fts_i2c_read
* Brief: i2c read
* Input: i2c info, write buf, write len, read buf, read len
* Output: get data in the 3rd buf
* Return: fail <0
***********************************************************************/
int fts_i2c_read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen)
{
	int ret;
	mutex_lock(&i2c_rw_access);
	
	if (writelen > 0) 
	{
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = 0,
				.len = writelen,
				.buf = writebuf,
			},
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "f%s: i2c read error. error code = %d \n", __func__, ret);
	} 
	else 
	{
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.  error code = %d \n", __func__, ret);
	}
	mutex_unlock(&i2c_rw_access);
	return ret;
}
/************************************************************************
* Name: fts_i2c_write
* Brief: i2c write
* Input: i2c info, write buf, write len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_i2c_write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;
	mutex_lock(&i2c_rw_access);
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = writelen,
			.buf = writebuf,
		},
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s i2c write error.\n", __func__);
	mutex_unlock(&i2c_rw_access);
	return ret;
}


/************************************************************************
* Name: fts_write_reg
* Brief:  write register
* Input: i2c info, address, value
* Output: no
* Return: success >0
***********************************************************************/
int fts_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};
	buf[0] = regaddr;
	buf[1] = regvalue;

	return fts_i2c_write(client, buf, sizeof(buf));
}
/************************************************************************
* Name: fts_read_reg
* Brief:  read register
* Input: i2c info, address, value
* Output: register value
* Return: success >0
***********************************************************************/
int fts_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{
	return fts_i2c_read(client, &regaddr, 1, regvalue, 1);
}

#if TPD_PROXIMITY
	/************************************************************************
	* Name: tpd_read_ps
	* Brief: read value
	* Input: no
	* Output: no
	* Return: 0
	***********************************************************************/
	int tpd_read_ps(void)
	{
	      tpd_proximity_detect;
	      return 0;    
	}
	/************************************************************************
	* Name: tpd_get_ps_value
	* Brief: get ps value
	* Input: no
	* Output: no
	* Return: ps value
	***********************************************************************/
	static int tpd_get_ps_value(void)
	{
	      return tpd_proximity_detect;
	}
	/************************************************************************
	* Name: tpd_enable_ps
	* Brief: enable value
	* Input: enable or not
	* Output: no
	* Return: success =0
	***********************************************************************/
	static int tpd_enable_ps(int enable)
	{
	       u8 state,state2;
	      	int ret = -1;

	      	//i2c_smbus_read_i2c_block_data(i2c_client, 0xB0, 1, &state);
		  ret=fts_read_reg(fts_i2c_client, 0xB0,&state);
 			if (ret<0) 
 			{
 				pr_info("[Focal][Touch] read value fail");
 				//return ret;
 			}
	      	pr_info("[proxi_5206]read: 999 0xb0's value is 0x%02X\n", state);
	      	if (enable)
		{
	                state |= 0x01;
	                tpd_proximity_flag = 1;
	                TPD_PROXIMITY_DEBUG("[proxi_5206]ps function is on\n");      
	      	}
		else
		{
	                state &= 0x00;      
	                tpd_proximity_flag = 0;
	                TPD_PROXIMITY_DEBUG("[proxi_5206]ps function is off\n");
	      	}

	      	//ret = i2c_smbus_write_i2c_block_data(i2c_client, 0xB0, 1, &state);
		ret=fts_write_reg(fts_i2c_client, 0xB0, state);
		if (ret<0) 
		{
			pr_info("[Focal][Touch] write value fail");
			//return ret;
		}
	      	TPD_PROXIMITY_DEBUG("[proxi_5206]write: 0xB0's value is 0x%02X\n", state);
	      
	      	//i2c_smbus_read_i2c_block_data(i2c_client, 0xB0, 1, &state2);
		  ret=fts_read_reg(fts_i2c_client, 0xB0,&state2);
 			if (ret<0) 
 			{
 				pr_info("[Focal][Touch] read value fail");
 				//return ret;
 			}
	      	if(state!=state2)
	      	{
	      		tpd_proximity_flag=0;
	      		pr_info("[proxi_5206]ps fail!!! state = 0x%x,  state2 =  0x%X\n", state,state2);
	      	}
	      
	      	return 0;
	}
	/************************************************************************
	* Name: tpd_ps_operate
	* Brief: oprate for ps
	* Input: command type, buffIn/buffOut data and len 
	* Output: no
	* Return: fail <0
	***********************************************************************/
	int tpd_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
	                void* buff_out, int size_out, int* actualout)
	{
	      int err = 0;
	      int value;
	      
	      hwm_sensor_data *sensor_data;
	      TPD_DEBUG("[proxi_5206]command = 0x%02X\n", command);                
	      switch (command)
	      {
	                case SENSOR_DELAY:
	                      if((buff_in == NULL) || (size_in < sizeof(int)))
	                      {
	                                APS_ERR("Set delay parameter error!\n");
	                                err = -EINVAL;
	                      }
	                      // Do nothing
	                      break;

	                case SENSOR_ENABLE:
	                      if((buff_in == NULL) || (size_in < sizeof(int)))
	                      {
	                                APS_ERR("Enable sensor parameter error!\n");
	                                err = -EINVAL;
	                      }
	                      else
	                      {                                
	                                value = *(int *)buff_in;
	                                if(value)
	                                {                
	                                      if((tpd_enable_ps(1) != 0))
	                                      {
	                                                APS_ERR("enable ps fail: %d\n", err); 
	                                                return -1;
	                                      }
	                                }
	                                else
	                                {
	                                      if((tpd_enable_ps(0) != 0))
	                                      {
	                                                APS_ERR("disable ps fail: %d\n", err); 
	                                                return -1;
	                                      }
	                                }
	                      }
	                      break;

	                case SENSOR_GET_DATA:
	                      if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
	                      {
	                                APS_ERR("get sensor data parameter error!\n");
	                                err = -EINVAL;
	                      }
	                      else
	                      {
	                                
	                                sensor_data = (hwm_sensor_data *)buff_out;                                
	                                
	                                if((err = tpd_read_ps()))
	                                {
	                                      err = -1;;
	                                }
	                                else
	                                {
	                                      sensor_data->values[0] = tpd_get_ps_value();
	                                      TPD_PROXIMITY_DEBUG("huang sensor_data->values[0] 1082 = %d\n", sensor_data->values[0]);
	                                      sensor_data->value_divide = 1;
	                                      sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
	                                }      
	                                
	                      }
	                      break;
	                default:
	                      APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
	                      err = -1;
	                      break;
	      }
	      
	      return err;      
	}
#endif


 /************************************************************************
* Name: fts_read_Touchdata
* Brief: report the point information
* Input: event info
* Output: get touch data in pinfo
* Return: success is zero
***********************************************************************/
static unsigned int buf_count_add=0;
static unsigned int buf_count_neg=0;
//unsigned int buf_count_add1;
//unsigned int buf_count_neg1;
u8 buf_touch_data[30*POINT_READ_BUF] = { 0 };//0xFF
static int fts_read_Touchdata(struct fts_ts_data *data)
{
	struct fts_event *event = &data->event;
	u8 buf[POINT_READ_BUF] = { 0 };//0xFF
	int ret = -1;
	int i = 0;
	u8 pointid = FTS_MAX_ID;
	//u8 pt00f=0;
	ret = fts_i2c_read(data->client, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0) 
	{
		dev_err(&data->client->dev, "%s read touchdata failed.\n", __func__);
		return ret;
	}
	buf_count_add++;
	//buf_count_add1=buf_count_add;
	memcpy( buf_touch_data+(((buf_count_add-1)%30)*POINT_READ_BUF), buf, sizeof(u8)*POINT_READ_BUF );
	
	return 0;
}

/************************************************************************
* Name: fts_report_value
* Brief: report the point information
* Input: event info
* Output: no
* Return: success is zero
***********************************************************************/
static int fts_report_value(struct fts_ts_data *data)
{	
	struct fts_event *event = &data->event;
	int i;
	int uppoint = 0;
	int touchs = 0;
	u8 pointid = FTS_MAX_ID;
	u8 buf[POINT_READ_BUF] = { 0 };//0xFF
	//atic u8 last_touchpoint=0; //release all touches in final
	buf_count_neg++;
	//buf_count_neg1=buf_count_neg;
	memcpy( buf,buf_touch_data+(((buf_count_neg-1)%30)*POINT_READ_BUF), sizeof(u8)*POINT_READ_BUF );

	
	/*Ft_Printf_Touchdata(data,buf);*/	/*打印报点调试信息*/

	memset(event, 0, sizeof(struct fts_event));
	event->touch_point_num=buf[FT_TOUCH_POINT_NUM] & 0x0F;
	event->touch_point = 0;
	for (i = 0; i < CFG_MAX_TOUCH_POINTS/*fts_updateinfo_curr.TPD_MAX_POINTS*/; i++) 
	{
		pointid = (buf[FTS_TOUCH_ID_POS + FTS_TOUCH_STEP * i]) >> 4;
		if (pointid >= FTS_MAX_ID)
			break;
		else
			event->touch_point++;
		
		event->au16_x[i] =
			(((s16) buf[FTS_TOUCH_X_H_POS + FTS_TOUCH_STEP * i]) & 0x0F) <<
			8 | (((s16) buf[FTS_TOUCH_X_L_POS + FTS_TOUCH_STEP * i])& 0xFF);
		event->au16_y[i] =
			(((s16) buf[FTS_TOUCH_Y_H_POS + FTS_TOUCH_STEP * i]) & 0x0F) <<
			8 | (((s16) buf[FTS_TOUCH_Y_L_POS + FTS_TOUCH_STEP * i]) & 0xFF);
		event->au8_touch_event[i] =
			buf[FTS_TOUCH_EVENT_POS + FTS_TOUCH_STEP * i] >> 6;
		event->au8_finger_id[i] =
			(buf[FTS_TOUCH_ID_POS + FTS_TOUCH_STEP * i]) >> 4;
		event->pressure[i] =
			(buf[FTS_TOUCH_XY_POS + FTS_TOUCH_STEP * i]);//cannot constant value
		event->area[i] =
			(buf[FTS_TOUCH_MISC + FTS_TOUCH_STEP * i]) >> 4;
		if((event->au8_touch_event[i]==0 || event->au8_touch_event[i]==2)&&((event->touch_point_num==0)||(event->pressure[i]==0 && event->area[i]==0  )))
			return 1;
		//if ( event->au16_x[i]==0 && event->au16_y[i] ==0)
		//	pt00f++; 
		#if 0
			dev_err(&fts_i2c_client->dev,"===1=========, id=%d event=%d x=%d y=%d pressure=%d area=%d\n", event->au8_finger_id[i],
				event->au8_touch_event[i], event->au16_x[i], event->au16_y[i], event->pressure[i], event->area[i]);
		#endif

		event->au16_x[i] = (event->au16_x[i]*1366/1280);
		event->au16_y[i] = (event->au16_y[i]*768/800);
		#if 0
			dev_err(&fts_i2c_client->dev,"===2=========, id=%d event=%d x=%d y=%d pressure=%d area=%d\n", event->au8_finger_id[i],
				event->au8_touch_event[i], event->au16_x[i], event->au16_y[i], event->pressure[i], event->area[i]);
		#endif
		/*
		if(0==event->pressure[i])
		{
			event->pressure[i]=FTS_PRESS;
		}
		if(0==event->area[i])
		{
			event->area[i]=FTS_PRESS;
		}
		*/
	}

	/*event->pressure = FTS_PRESS;*/
	/*event->pressure = 200;*/
	/*
	if (pt00f>0)
	{    
		for(i=0;i<POINT_READ_BUF;i++)
		{
        		pr_info(KERN_WARNING "The xy00 is %x \n",buf[i]);
		}
		pr_info(KERN_WARNING "\n");		
	}
	*/
	/*protocol B*/
	for (i = 0; i < event->touch_point; i++) 
	{
		input_mt_slot(data->input_dev, event->au8_finger_id[i]);

		if (event->au8_touch_event[i]== 0 || event->au8_touch_event[i] == 2) 
		{
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);
			input_report_abs(data->input_dev, ABS_MT_PRESSURE, event->pressure[i]);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->area[i]);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->au16_x[i]);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->au16_y[i]);
			touchs |= BIT(event->au8_finger_id[i]);
			data->touchs |= BIT(event->au8_finger_id[i]);
			#if 0
			dev_err(&fts_i2c_client->dev,"[Focal][Touch] report_abs_X = %d, report_abs_Y = %d  !\n", event->au16_x[i], event->au16_y[i]);
			#endif
		} 
		else 
		{
			uppoint++;
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
			data->touchs &= ~BIT(event->au8_finger_id[i]);
		}
	}
	if(unlikely(data->touchs ^ touchs)){
		for(i = 0; i < CFG_MAX_TOUCH_POINTS; i++)
		{
			if(BIT(i) & (data->touchs ^ touchs))
			{
				uppoint++;
				input_mt_slot(data->input_dev, i);
				input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
			}
		}
	}
	data->touchs = touchs;
	/*
	 if((last_touchpoint>0)&&(event->touch_point_num==0))    //release all touches in final
	{	
		for (j = 0; j < CFG_MAX_TOUCH_POINTS; j++) 
		{
			input_mt_slot(data->input_dev, j);
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
		}
			last_touchpoint=0;
    	} 
	*/
	if(event->touch_point == uppoint) 
	{
		input_report_key(data->input_dev, BTN_TOUCH, 0);
		//touch_down_up_status = 0;
		//pr_info("[Focal][Touch] touch up !\n");
	} 
	else 
	{
		input_report_key(data->input_dev, BTN_TOUCH, event->touch_point > 0);
		//if (touch_down_up_status == 0) 
		//{
		//	touch_down_up_status = 1;
		//	pr_info("[Focal][Touch] touch down !\n");
		//}
			
	}
	input_sync(data->input_dev);
	//last_touchpoint=event->touch_point_num ;//release all touches in final
	return 0;
}


/************************************************************************
* Name: fts_ts_interrupt
* Brief: the focaltech device will signal the host about TRIGGER_FALLING, and processed when the interrupt is asserted.
* Input: irq, device id
* Output: no
* Return: irq handle
***********************************************************************/
static irqreturn_t fts_ts_interrupt(int irq, void *dev_id)
{
	/*struct fts_ts_data *fts_wq_data = dev_id;  use globle fts_wq_data data*/
	int ret = 0;
	u8 state;
	#if TPD_PROXIMITY
      		int err;
      		hwm_sensor_data sensor_data;
      		u8 proximity_status;      
	#endif

	disable_irq_nosync(fts_wq_data->client->irq);
	#if TPD_PROXIMITY
                if (tpd_proximity_flag == 1)
                {
                      //i2c_smbus_read_i2c_block_data(i2c_client, 0xB0, 1, &state);
			  ret=fts_read_reg(fts_i2c_client, 0xB0,&state);
 			if (ret<0) 
 			{
 				pr_info("[Focal][Touch] read value fail");
 				//return ret;
 			}
                      TPD_PROXIMITY_DEBUG("proxi_5206 0xB0 state value is 1131 0x%02X\n", state);

                      if(!(state&0x01))
                      {
                                tpd_enable_ps(1);
                      }

                     // i2c_smbus_read_i2c_block_data(i2c_client, 0x01, 1, &proximity_status);
			 ret=fts_read_reg(fts_i2c_client, 0x01,&proximity_status);
 			if (ret<0) 
 			{
 				pr_info("[Focal][Touch] read value fail");
 				//return ret;
 			}
                      TPD_PROXIMITY_DEBUG("proxi_5206 0x01 value is 1139 0x%02X\n", proximity_status);
                      
                      if (proximity_status == 0xC0)
                      {
                                tpd_proximity_detect = 0;      
                      }
                      else if(proximity_status == 0xE0)
                      {
                                tpd_proximity_detect = 1;
                      }

                      TPD_PROXIMITY_DEBUG("tpd_proximity_detect 1149 = %d\n", tpd_proximity_detect);

                      if ((err = tpd_read_ps()))
                      {
                                TPD_PROXIMITY_DMESG("proxi_5206 read ps data 1156: %d\n", err);      
                      }
                      sensor_data.values[0] = tpd_get_ps_value();
                      sensor_data.value_divide = 1;
                      sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
                      if ((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
                      {
                                TPD_PROXIMITY_DMESG(" proxi_5206 call hwmsen_get_interrupt_data failed= %d\n", err);      
                      }
                }  
	#endif

	#if FTS_GESTRUE_EN
		//i2c_smbus_read_i2c_block_data(fts_wq_data->client, 0xd0, 1, &state);
		ret=fts_read_reg(fts_i2c_client, 0xd0,&state);
		if (ret<0) 
		{
			pr_info("[Focal][Touch] read value fail");
			//return ret;
		}
		/*pr_info("tpd fts_read_Gestruedata state=%d\n", state);*/
		if (state == 1) 
		{
			fts_read_Gestruedata(fts_wq_data);
			enable_irq(fts_wq_data->client->irq);
			/*continue;*/
		}
		else
		{
	#endif
			//disable_irq_nosync(fts_wq_data->irq);
                        #if GTP_ESD_PROTECT
				count_irq ++;				
			#endif
			ret = fts_read_Touchdata(fts_wq_data);
			enable_irq(fts_wq_data->client->irq);
			//if (ret == 0)
			fts_report_value(fts_wq_data);
	#if FTS_GESTRUE_EN
		}
	#endif
	//enable_irq(fts_wq_data->irq);
	return IRQ_HANDLED;
}
  /************************************************************************
* Name: fts_reset_tp
* Brief: reset TP
* Input: pull low or high
* Output: no
* Return: 0
***********************************************************************/
void fts_reset_tp(int HighOrLow)
{
	pr_info("[Focal] %s : set tp reset pin to %d\n", __func__, HighOrLow);
	gpio_set_value(fts_wq_data->pdata->rst_gpio, HighOrLow);
}
/************************************************************************
* Name: fts_init_gpio_hw
* Brief: initial gpio
* Input: data point
* Output: no
* Return: success =0
***********************************************************************/
static int fts_init_gpio_hw(struct fts_ts_data *fts_wq_data)
{
	int ret = 0;

	ret = gpio_request(fts_wq_data->pdata->rst_gpio, FTS_RESET_PIN_NAME);
	if (ret) {
		pr_info("[Focal][Touch] %s: request GPIO %s for reset failed, ret = %d\n",
			__func__, FTS_RESET_PIN_NAME, ret);
		return ret;
	}

	//rst_gpio status is reversed. tangjun 2016.03.21
	ret = gpio_direction_output(fts_wq_data->pdata->rst_gpio, 1);	/* change reset set high*/
	msleep(20);
	ret = gpio_direction_output(fts_wq_data->pdata->rst_gpio, 0);	/* change reset set high*/
	msleep(20);

	if (ret) 
	{
		pr_info("[Focal][Touch] %s: set %s gpio to out put high failed, ret = %d\n",
			__func__, FTS_RESET_PIN_NAME, ret);
		return ret;
	}

	return ret;
}
/************************************************************************
* Name: fts_un_init_gpio_hw
* Brief: uninitial gpio
* Input: data point
* Output: no
* Return: no
***********************************************************************/
static void fts_un_init_gpio_hw(struct fts_ts_data *fts_wq_data)
{
	gpio_free(fts_wq_data->pdata->rst_gpio);
}


#ifdef CONFIG_PM_SLEEP
static ssize_t ft5626_power_hal_suspend_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n",gtp_is_suspend ? "suspend" : "resume");
}

static ssize_t ft5626_power_hal_suspend_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{


	if (!strncmp(buf, POWER_HAL_SUSPEND_ON, POWER_HAL_SUSPEND_STATUS_LEN))
		fts_ts_hal_suspend(dev);
	else
		fts_ts_hal_resume(dev);

	return count;
}

static struct device_attribute dev_attr_power_HAL_suspend =
	__ATTR(power_HAL_suspend, S_IRUGO | S_IWUSR, ft5626_power_hal_suspend_show, ft5626_power_hal_suspend_store);
#endif



/************************************************************************
* Name: fts_ts_probe
* Brief: driver entrance function for initial/power on/create channel 
* Input: i2c info, device id
* Output: no
* Return: 0
***********************************************************************/
static int fts_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct focal_i2c_platform_data *pdata = (struct focal_i2c_platform_data *)client->dev.platform_data;
	struct input_dev *input_dev;
	int err = 0;
	unsigned char uc_reg_value;
	unsigned char uc_reg_addr;
	
#ifdef CONFIG_PM_SLEEP
    struct device *dev = &client->dev;
#endif

	pr_info("[Focal][Touch] fts_ts_probe !\n");

	fts_i2c_client = client;
	client->addr = 0x38;

	pdata = kzalloc(sizeof(struct focal_i2c_platform_data), GFP_KERNEL);
		
	pdata->intr_gpio = GTP_INT_PORT;
	pdata->rst_gpio = GTP_RST_PORT;
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	fts_wq_data = kzalloc(sizeof(struct fts_ts_data), GFP_KERNEL);
	if (!fts_wq_data) 
	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	i2c_set_clientdata(client, fts_wq_data);

	fts_wq_data->client = client;
	fts_wq_data->init_success = 0;
	fts_wq_data->pdata = pdata;
	fts_wq_data->x_max = TOUCH_MAX_X;
	fts_wq_data->y_max = TOUCH_MAX_Y;
	#if 0
	if (0 >= fts_wq_data->x_max)
		fts_wq_data->x_max = TOUCH_MAX_X;
	if (0 >= fts_wq_data->y_max)
		fts_wq_data->y_max = TOUCH_MAX_Y;
	#endif
	
	fts_wq_data->irq = GTP_INT_PORT;//fts_wq_data->pdata->intr_gpio;

	if (fts_init_gpio_hw (fts_wq_data) < 0)
		goto exit_init_gpio;

	if((fts_read_reg(client, 0x00, &uc_reg_value))< 0)
	{
		dev_err(&client->dev,"[FTS]fts_ts_probe I2C transfer error, line: %d\n", __LINE__);
		return -1; 
	}

	if (gpio_request(fts_wq_data->irq, FTS_INT_PIN_NAME)) 
	{
		pr_info("[Focal][Touch] %s: gpio %d request for interrupt fail.\n", __func__, fts_wq_data->irq);
		goto exit_irq_request_failed;
	}
	gpio_direction_input(fts_wq_data->irq);

	fts_wq_data->client->irq = gpio_to_irq(fts_wq_data->irq);
	err = request_threaded_irq(fts_wq_data->client->irq, NULL, fts_ts_interrupt,
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT, client->dev.driver->name,
		fts_wq_data);

	if (fts_wq_data->client->irq < 0) 
	{
		dev_err(&client->dev, "[Focal][Touch] %s: request irq fail. \n", __func__);
		goto exit_irq_request_failed;
	}

	disable_irq(fts_wq_data->client->irq);	/*need mutex protect, should add latter*/

	input_dev = input_allocate_device();
	if (!input_dev) 
	{
		err = -ENOMEM;
		dev_err(&client->dev, "[Focal][Touch] %s: failed to allocate input device\n", __func__);
		goto exit_input_dev_alloc_failed;
	}

	fts_wq_data->input_dev = input_dev;
	fts_input_dev = input_dev;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	//#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0))
		input_mt_init_slots(input_dev, CFG_MAX_TOUCH_POINTS, 0);
	//#endif
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 31, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, TOUCH_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, TOUCH_MAX_Y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, FTS_PRESS_MAX, 0, 0);
	#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 8, 0))
		//for linux 3.8
		input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, CFG_MAX_TOUCH_POINTS, 0, 0);
	#endif
	input_dev->name = FTS_INPUT_DEV_NAME;
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev, "[Focal][Touch] %s: failed to register input device: %s\n", __func__, dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}
	/*make sure CTP already finish startup process */
	msleep(200);
	
#if SYSFS_DEBUG_EN
	pr_info("[Focal][Touch] fts_create_sysfs Start !\n");
	fts_create_sysfs(client);
	
	pr_info("[Focal][Touch] fts_create_sysfs End !\n");

	mutex_init(&fts_wq_data->g_device_mutex);
	
#endif

	HidI2c_To_StdI2c(client);
	fts_get_upgrade_array();
#if FTS_CTL_IIC_EN
	if (fts_rw_iic_drv_init(client) < 0)
		dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n", __func__);
#endif
//fts_i2c_client = client;
//fts_input_dev=fts_wq_data->input_dev;
#if FTS_APK_DEBUG_EN
	fts_create_apk_debug_channel(client);
#endif
#ifdef FTS_AUTO_UPGRADEG
	fts_ctpm_auto_upgrade(client);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&focal_early_suspend);
#endif
	/*get some register information */

	uc_reg_addr = FTS_REG_FW_VER;
	err = fts_i2c_read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	if (err < 0)
		fts_wq_data->init_success = 0;
	else 
	{
		fts_wq_data->init_success = 1;
		pr_info("[FTS] Firmware version = 0x%x\n", uc_reg_value);
		IC_FW = uc_reg_value;
	}

	uc_reg_addr = FTS_REG_POINT_RATE;
	err = fts_i2c_read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	if (err < 0)
		fts_wq_data->init_success = 0;
	else 
	{
		fts_wq_data->init_success = 1;
		pr_info("[FTS] report rate is %dHz.\n", uc_reg_value * 10);
	}

	uc_reg_addr = FTS_REG_THGROUP;
	err = fts_i2c_read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	if (err < 0)
		fts_wq_data->init_success = 0;
	else 
	{
		fts_wq_data->init_success = 1;
		pr_info("[FTS] touch threshold is %d.\n", uc_reg_value * 4);
	}
		
	uc_reg_addr = FTS_REG_VENDOR_ID;
	err = fts_i2c_read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	if (err < 0)
		fts_wq_data->init_success = 0;
	else 
	{
		fts_wq_data->init_success = 1;
		pr_info("[FTS] VENDOR ID = 0x%x\n", uc_reg_value);
	}

	#if GTP_ESD_PROTECT
   		INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);
    		gtp_esd_check_workqueue = create_workqueue("gtp_esd_check");
    		queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, TPD_ESD_CHECK_CIRCLE);
	#endif

	#if FTS_GESTRUE_EN	
		fts_Gesture_init(input_dev);
	#endif
	#if TPD_PROXIMITY
	      struct hwmsen_object obj_ps;
	      //interrupt mode
	      obj_ps.polling = 0;
	      obj_ps.sensor_operate = tpd_ps_operate;
	      if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
	      {
	                APS_ERR("proxi_fts attach fail = %d\n", err);
	      }
	      else
	      {
	                APS_ERR("proxi_fts attach ok = %d\n", err);
	      }                
	#endif

	enable_irq(fts_wq_data->client->irq);
	dev_err(&client->dev,"[Focal][Touch][INFO] client name = %s irq = %d\n", client->name, client->irq);
	dev_err(&client->dev,"[Focal][Touch] X-RES = %d, Y-RES = %d, RST gpio = %d, gpio irq = %d, client irq = %d\n",
	fts_wq_data->pdata->abs_x_max, fts_wq_data->pdata->abs_y_max, fts_wq_data->pdata->rst_gpio, fts_wq_data->irq, fts_wq_data->client->irq);

	if (fts_wq_data->init_success == 1)
		fts_init_success = 1;	

#ifdef CONFIG_PM_SLEEP

       err = device_create_file(dev, &dev_attr_power_HAL_suspend);
       if (err < 0) {
           dev_err(&client->dev,"unable to create suspend entry \n");
           return err;
       }
       err = register_power_hal_suspend_device(dev);
       if (err < 0)
       {
           dev_err(&client->dev," unable to register for power hal \n");
            return err;
       }
#endif




	return 0;
	exit_input_register_device_failed:
		input_free_device(input_dev);

	exit_input_dev_alloc_failed:
		free_irq(client->irq, fts_wq_data);
	#if CONFIG_PM_EN
		#if 0
			exit_request_reset:
			gpio_free(fts_wq_data->pdata->reset);
		#endif
	#endif

	exit_init_gpio:
		fts_un_init_gpio_hw(fts_wq_data);

	exit_irq_request_failed:
		i2c_set_clientdata(client, NULL);
		kfree(fts_wq_data);

	exit_alloc_data_failed:
		pr_info("[%s] alloc fts_ts_data failed !! \n", __func__);
	exit_check_functionality_failed:
	return err;
}
#if GTP_ESD_PROTECT
/************************************************************************
* Name: force_reset_guitar
* Brief: reset
* Input: no
* Output: no
* Return: 0
***********************************************************************/
static void force_reset_guitar(void)
{
    	/*
    	s32 i;
    	s32 ret;

    	TPD_DMESG("force_reset_guitar\n");
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
	msleep(10);

	hwPowerDown(MT6323_POWER_LDO_VGP1,  "TP");
	msleep(200);
	hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_2800, "TP");
	msleep(5);

  
	msleep(10);
	TPD_DMESG(" fts ic reset\n");
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);

	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);

	msleep(300);
	
	#if TPD_PROXIMITY
		if (FT_PROXIMITY_ENABLE == tpd_proximity_flag) 
		{
			tpd_enable_ps(FT_PROXIMITY_ENABLE);
		}
	#endif
	*/
}
//0 for no apk upgrade, 1 for apk upgrade
extern int apk_debug_flag; 
#define A3_REG_VALUE								0x54
#define RESET_91_REGVALUE_SAMECOUNT 				5
static u8 g_old_91_Reg_Value = 0x00;
static u8 g_first_read_91 = 0x01;
static u8 g_91value_same_count = 0;
/************************************************************************
* Name: gtp_esd_check_func
* Brief: esd check function
* Input: struct work_struct
* Output: no
* Return: 0
***********************************************************************/
static void gtp_esd_check_func(struct work_struct *work)
{
	int i;
	int ret = -1;
	u8 data, data_old;
	u8 flag_error = 0;
	int reset_flag = 0;
	u8 check_91_reg_flag = 0;

	//if (tpd_halt ) {
	//	return;
	//}
	if(apk_debug_flag) 
	{
		queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, esd_check_circle);
		return;
	}

	run_check_91_register = 0;
	for (i = 0; i < 3; i++) 
	{
		//ret = fts_i2c_smbus_read_i2c_block_data(ftxxxx_ts->client, 0xA3, 1, &data);
		ret=fts_read_reg(fts_i2c_client, 0xA3,&data);
		if (ret<0) 
		{
			pr_info("[Focal][Touch] read value fail");
			//return ret;
		}
		if (ret==1 && A3_REG_VALUE==data) 
		{
		    break;
		}
	}

	if (i >= 3) 
	{
		force_reset_guitar();
		pr_info("focal--tpd reset. i >= 3  ret = %d	A3_Reg_Value = 0x%02x\n ", ret, data);
		reset_flag = 1;
		goto FOCAL_RESET_A3_REGISTER;
	}

	//esd check for count
  	//ret = fts_i2c_smbus_read_i2c_block_data(ftxxxx_ts->client, 0x8F, 1, &data);
	ret=fts_read_reg(fts_i2c_client, 0x8F,&data);
	if (ret<0) 
	{
		pr_info("[Focal][Touch] read value fail");
		//return ret;
	}
	pr_info("0x8F:%d, count_irq is %d\n", data, count_irq);
			
	flag_error = 0;
	if((count_irq - data) > 10) 
	{
		if((data+200) > (count_irq+10) )
		{
			flag_error = 1;
		}
	}
	
	if((data - count_irq ) > 10) 
	{
		flag_error = 1;		
	}
		
	if(1 == flag_error) 
	{	
		pr_info("focal--tpd reset.1 == flag_error...data=%d	count_irq\n ", data, count_irq);
	    	force_reset_guitar();
		reset_flag = 1;
		goto FOCAL_RESET_INT;
	}

	run_check_91_register = 1;
	//ret = fts_i2c_smbus_read_i2c_block_data(ftxxxx_ts->client, 0x91, 1, &data);
	ret=fts_read_reg(fts_i2c_client, 0x91,&data);
	if (ret<0) 
	{
		pr_info("[Focal][Touch] read value fail");
		//return ret;
	}
	pr_info("focal---------91 register value = 0x%02x	old value = 0x%02x\n",	data, g_old_91_Reg_Value);
	if(0x01 == g_first_read_91) 
	{
		g_old_91_Reg_Value = data;
		g_first_read_91 = 0x00;
	} 
	else 
	{
		if(g_old_91_Reg_Value == data)
		{
			g_91value_same_count++;
			pr_info("focal 91 value ==============, g_91value_same_count=%d\n", g_91value_same_count);
			if(RESET_91_REGVALUE_SAMECOUNT == g_91value_same_count) 
			{
				force_reset_guitar();
				pr_info("focal--tpd reset. g_91value_same_count = 5\n");
				g_91value_same_count = 0;
				reset_flag = 1;
			}
			
			//run_check_91_register = 1;
			esd_check_circle = TPD_ESD_CHECK_CIRCLE / 2;
			g_old_91_Reg_Value = data;
		} 
		else 
		{
			g_old_91_Reg_Value = data;
			g_91value_same_count = 0;
			//run_check_91_register = 0;
			esd_check_circle = TPD_ESD_CHECK_CIRCLE;
		}
	}
FOCAL_RESET_INT:
FOCAL_RESET_A3_REGISTER:
	count_irq=0;
	data=0;
	//fts_i2c_smbus_write_i2c_block_data(ftxxxx_ts->client, 0x8F, 1, &data);

	ret=fts_write_reg(fts_i2c_client, 0x8F, data);
	if (ret<0) 
	{
		pr_info("[Focal][Touch] write value fail");
		//return ret;
	}
	if(0 == run_check_91_register)
	{
		g_91value_same_count = 0;
	}
	#ifdef TPD_PROXIMITY
	if( (1 == reset_flag) && ( FT_PROXIMITY_ENABLE == tpd_proximity_flag) )
	{
		if((tpd_enable_ps(FT_PROXIMITY_ENABLE) != 0))
		{
			APS_ERR("enable ps fail\n"); 
			return -1;
		}
	}
	#endif
	//end esd check for count// add by zhaofei - 2014-02-18-17-04

    	//if (!tpd_halt)
    	//{
        	//queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, TPD_ESD_CHECK_CIRCLE);
        	queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, esd_check_circle);
    	//}

    	return;
}
#endif

#ifdef CONFIG_PM
	 /************************************************************************
	* Name: fts_ts_suspend
	* Brief: system sleep
	* Input: no use
	* Output: no
	* Return: no
	***********************************************************************/
	static void fts_ts_suspend(struct early_suspend *handler)
	{
		struct fts_ts_data *ts = fts_wq_data;
		
		int i=0,ret = 0;
		buf_addr[0]=0xC0;
		buf_addr[1]=0x8B;
				
		for(i=0;i<2;i++)
		{
			ret = fts_read_reg(ts->client, buf_addr[i], (buf_value+i));
			if (ret<0) 
			{
				pr_info("[Focal][Touch] read value fail");
				//return ret;
			}
		}
		#if TPD_PROXIMITY
      			if (tpd_proximity_flag == 1)
      			{
                		tpd_proximity_flag_one = 1;      
                		return;
      			}
		#endif

		#if FTS_GESTRUE
			pr_info("[focal] open gestrue mode");
			fts_write_reg(ts->client, 0xd0, 0x01);
			if (fts_updateinfo_curr.CHIP_ID==0x54 || fts_updateinfo_curr.CHIP_ID==0x58)
			{
				fts_write_reg(ts->client, 0xd1, 0xff);
				fts_write_reg(ts->client, 0xd2, 0xff);
				fts_write_reg(ts->client, 0xd5, 0xff);
				fts_write_reg(ts->client, 0xd6, 0xff);
				fts_write_reg(ts->client, 0xd7, 0xff);
				fts_write_reg(ts->client, 0xd8, 0xff);
			}
			return;
		#else
			dev_err(&ts->client->dev, "[focal] Touch suspend");			
			disable_irq(ts->client->irq);
			#if GTP_ESD_PROTECT
    				cancel_delayed_work_sync(&gtp_esd_check_work);
			#endif

			disable_irq_nosync(ts->pdata->intr_gpio);
			
			if ((fts_updateinfo_curr.CHIP_ID==0x59))
				fts_write_reg(ts->client,0xa5,0x02);
			else
				fts_write_reg(ts->client,0xa5,0x03);
			msleep(10);
			/*release add touches*/
			
			for (i = 0; i <CFG_MAX_TOUCH_POINTS; i++) 
			{
				input_mt_slot(ts->input_dev, i);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
			}
			input_mt_report_pointer_emulation(ts->input_dev, false);
			input_sync(ts->input_dev);
		#endif
	}
	 /************************************************************************
	* Name: fts_ts_resume
	* Brief: system wake up 
	* Input: no use
	* Output: no
	* Return: no
	***********************************************************************/
	static void fts_ts_resume(struct early_suspend *handler)
	{
		struct fts_ts_data *ts = fts_wq_data;
		
		int i=0,ret = 0;		
		dev_dbg(&ts->client->dev, "[FTS]focaltech resume.\n");
		buf_addr[0]=0xC0;
		buf_addr[1]=0x8B;
				
		for(i=0;i<2;i++)
		{
			ret = fts_write_reg(ts->client, buf_addr[i], buf_value[i]);
			if (ret<0) 
			{
				pr_info("[Focal][Touch] write value fail");
				//return ret;
			}
		}
		#if TPD_PROXIMITY 
      			if (tpd_proximity_flag == 1)
      			{
                		if(tpd_proximity_flag_one == 1)
                		{
                      		tpd_proximity_flag_one = 0;      
                      		//TPD_DMESG(TPD_DEVICE " tpd_proximity_flag_one \n"); 
                      		return;
                		}
      			}
		#endif      

		#if FTS_GESTRUE	
			fts_write_reg(ts->client, 0xD0, 0x00);
		#endif
		#if GTP_ESD_PROTECT
                        count_irq = 0;
    			queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, TPD_ESD_CHECK_CIRCLE);
		#endif
		//gpio_set_value(ts->pdata->rst_gpio, 1);//rst_gpio status is reversed. tangjun 2016.03.21
		//msleep(5);
		gpio_set_value(ts->pdata->rst_gpio, 0);
		enable_irq(ts->client->irq);	

		msleep(30);

	}
#else
	#define fts_ts_suspend		NULL
	#define fts_ts_resume		NULL
#endif



#ifdef CONFIG_PM_SLEEP
	 /************************************************************************
	* Name: fts_ts_suspend
	* Brief: system sleep
	* Input: no use
	* Output: no
	* Return: no
	***********************************************************************/
	static void fts_ts_hal_suspend(struct device *dev)
	{
		struct fts_ts_data *ts = fts_wq_data;
		
		int i=0,ret = 0;
		buf_addr[0]=0xC0;
		buf_addr[1]=0x8B;
		dev_err(&ts->client->dev, "[FTS]focaltech fts_ts_suspend. gtp_is_suspend = %d \n", gtp_is_suspend);

		gtp_is_suspend = 1;
				
		for(i=0;i<2;i++)
		{
			ret = fts_read_reg(ts->client, buf_addr[i], (buf_value+i));
			if (ret<0) 
			{
				dev_err(&ts->client->dev, "[Focal][Touch] read value fail");
				//return ret;
			}
		}
		#if TPD_PROXIMITY
      			if (tpd_proximity_flag == 1)
      			{
                		tpd_proximity_flag_one = 1;      
                		return;
      			}
		#endif

		#if FTS_GESTRUE
			dev_err(&ts->client->dev, "[focal] open gestrue mode");
			fts_write_reg(ts->client, 0xd0, 0x01);
			if (fts_updateinfo_curr.CHIP_ID==0x54 || fts_updateinfo_curr.CHIP_ID==0x58)
			{
				fts_write_reg(ts->client, 0xd1, 0xff);
				fts_write_reg(ts->client, 0xd2, 0xff);
				fts_write_reg(ts->client, 0xd5, 0xff);
				fts_write_reg(ts->client, 0xd6, 0xff);
				fts_write_reg(ts->client, 0xd7, 0xff);
				fts_write_reg(ts->client, 0xd8, 0xff);
			}
			return;
		#else
			dev_err(&ts->client->dev, "[focal] Touch suspend");			
			disable_irq(fts_wq_data->client->irq);
			#if GTP_ESD_PROTECT
    				cancel_delayed_work_sync(&gtp_esd_check_work);
			#endif

			//disable_irq_nosync(fts_wq_data->client->irq);
			
			if ((fts_updateinfo_curr.CHIP_ID==0x59))
				fts_write_reg(ts->client,0xa5,0x02);
			else
				fts_write_reg(ts->client,0xa5,0x03);
			msleep(10);
			/*release add touches*/
			
			for (i = 0; i <CFG_MAX_TOUCH_POINTS; i++) 
			{
				input_mt_slot(ts->input_dev, i);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
			}
			input_mt_report_pointer_emulation(ts->input_dev, false);
			input_sync(ts->input_dev);
			
		#endif
	}
	 /************************************************************************
	* Name: fts_ts_resume
	* Brief: system wake up 
	* Input: no use
	* Output: no
	* Return: no
	***********************************************************************/
	static void fts_ts_hal_resume(struct device *dev)//(struct early_suspend *handler)
	{
		struct fts_ts_data *ts = fts_wq_data;
		
		int i=0,ret = 0;		
		dev_err(&ts->client->dev, "[FTS]focaltech resume. gtp_is_suspend = %d \n", gtp_is_suspend);

		if (gtp_is_suspend)
		{
			buf_addr[0]=0xC0;
			buf_addr[1]=0x8B;
					
			for(i=0;i<2;i++)
			{
				ret = fts_write_reg(ts->client, buf_addr[i], buf_value[i]);
				if (ret<0) 
				{
					pr_info("[Focal][Touch] write value fail");
					//return ret;
				}
			}
			#if TPD_PROXIMITY 
	      			if (tpd_proximity_flag == 1)
	      			{
	                		if(tpd_proximity_flag_one == 1)
	                		{
	                      		tpd_proximity_flag_one = 0;      
	                      		//TPD_DMESG(TPD_DEVICE " tpd_proximity_flag_one \n"); 
	                      		return;
	                		}
	      			}
			#endif      

			#if FTS_GESTRUE	
				fts_write_reg(ts->client, 0xD0, 0x00);
			#endif
			#if GTP_ESD_PROTECT
	                        count_irq = 0;
	    			queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, TPD_ESD_CHECK_CIRCLE);
			#endif
			gpio_set_value(ts->pdata->rst_gpio, 1);//rst_gpio status is reversed. tangjun 2016.03.21
			pr_err("xmyxh,hal set rst_gpio=1");
			msleep(5);
			gpio_set_value(ts->pdata->rst_gpio, 0);
			pr_err("xmyxh,hal set rst_gpio=0");
			enable_irq(fts_wq_data->client->irq);
			pr_err("xmyxh,enable the irq");

			msleep(30);
            pr_err("fininshed resume!!");

			HidI2c_To_StdI2c(ts->client);                     //切换到I2C 模式
			pr_err("xmyxh,change to i2c mode");
			gtp_is_suspend = 0;
		}
	}

#endif
/************************************************************************
* Name: fts_ts_remove
* Brief: remove driver/channel
* Input: i2c info
* Output: no
* Return: 0
***********************************************************************/
static int fts_ts_remove(struct i2c_client *client)
{
	struct fts_ts_data *fts_wq_data;
	fts_wq_data = i2c_get_clientdata(client);
	input_unregister_device(fts_wq_data->input_dev);

#ifdef CONFIG_PM_SLEEP
    device_remove_file(&client->dev, &dev_attr_power_HAL_suspend);
    unregister_power_hal_suspend_device(&client->dev);
#endif

	#ifdef CONFIG_PM
		gpio_free(fts_wq_data->pdata->rst_gpio);
	#endif
	#if FTS_CTL_IIC_EN
		fts_rw_iic_drv_exit();
	#endif
	#if SYSFS_DEBUG_EN
		fts_remove_sysfs(client);
	#endif
	#if FTS_APK_DEBUG_EN
		fts_release_apk_debug_channel();
	#endif
	#if GTP_ESD_PROTECT
    		destroy_workqueue(gtp_esd_check_workqueue);
	#endif
	fts_un_init_gpio_hw(fts_wq_data);

	free_irq(client->irq, fts_wq_data);

	kfree(fts_wq_data);
	i2c_set_clientdata(client, NULL);

	return 0;
}

#define FT_NAME           	"FTSC1000"

static const struct i2c_device_id fts_ts_id[] = {
    { "FTSC1000:00", 0 },
    { }
};

static struct acpi_device_id ft_acpi_match[] = {
	{FT_NAME, 0},
	{ },
};

/*MODULE_DEVICE_TABLE(i2c, fts_ts_id);*/

static struct i2c_driver fts_ts_driver = {
	.probe = fts_ts_probe,
	.remove = fts_ts_remove,
	.id_table = fts_ts_id,
	.driver = {
		.name = FTS_NAME,
		.owner = THIS_MODULE,
        	.acpi_match_table = ACPI_PTR(ft_acpi_match),
	},
};
 /************************************************************************
* Name: fts_ts_init
* Brief: add driver info
* Input: no
* Output: no
* Return: fail <0
***********************************************************************/
static int __init fts_ts_init(void)
{
	int ret;

	pr_info("[Focal][Touch] fts_ts_init !\n");
	ret = i2c_add_driver(&fts_ts_driver);
	if (ret) 
	{
		pr_info( "Adding focaltech driver failed "
			"(errno = %d)\n", ret);
	} 
	else 
	{
		pr_info("Successfully added driver %s\n",fts_ts_driver.driver.name);
	}
	return ret;
}
/************************************************************************
* Name: fts_ts_exit
* Brief:  should never be called
* Input: no
* Output: no
* Return: 0
***********************************************************************/
static void __exit fts_ts_exit(void)
{
	i2c_del_driver(&fts_ts_driver);
}

//module_init(fts_ts_init);
late_initcall(fts_ts_init);
module_exit(fts_ts_exit);

MODULE_AUTHOR("<OSTeam>");
MODULE_DESCRIPTION("FocalTech TouchScreen driver");
MODULE_LICENSE("GPL");

