#include "tofsense.h"
#include "i2cconfig.h"

uint8_t iic_read_buff[256];//读取缓存数组
uint8_t iic_write_buff[256];//写入缓存数组
uint16_t iic_test_count=0;
uint16_t iic_test_i=0;
tofsense_f_output_parameter tofsense_f_output;//解码后存放TOF输出数据的结构体


tofsense_f_output_parameter tof_f2;//存放ID为0的TOFSense-F输出数据的结构体
tofsense_f_output_parameter tof_f1;//存放ID为1的TOFSense-F输出数据的结构体

/************************************************
函数名称 ： IIC_Unpack_Data
函数功能 ： 通过IIC读取所有寄存器信息并进行解码，将解码后变量存入结构体成员变量中
参    数 ： *pdata缓存数组的指针  slave_addr从机地址  *pdata1存放变量的结构体指针
返 回 值 ： mark 是否读取和解码成功标志位，0错误，1正确
调用方法：例如：IIC_Unpack_Data(iic_read_buff,ADDR_SLAVE,&tofsense_f_output);
*************************************************/
uint8_t IIC_Unpack_Data(uint8_t *pdata,uint8_t slave_addr,tofsense_f_output_parameter *pdata1)
{
	uint8_t mark=0;//是否读取和解码成功标志位

	mark=IIC_Get_All_Register_Data(pdata,slave_addr);
	if(mark == 0)
	{
		pdata1->interface_mode=pdata[TOF_ADDR_MODE]&0x07;
		pdata1->id=pdata[TOF_ADDR_ID];
		pdata1->uart_baudrate=(uint32_t)(((uint32_t)pdata[TOF_ADDR_UART_BAUDRATE])|((uint32_t)pdata[TOF_ADDR_UART_BAUDRATE+1]<<8)|
				                                       ((uint32_t)pdata[TOF_ADDR_UART_BAUDRATE+2]<<16)|((uint32_t)pdata[TOF_ADDR_UART_BAUDRATE+3]<<24));
		pdata1->system_time=(uint32_t)(((uint32_t)pdata[TOF_ADDR_SYSTEM_TIME])|((uint32_t)pdata[TOF_ADDR_SYSTEM_TIME+1]<<8)|
				                                     ((uint32_t)pdata[TOF_ADDR_SYSTEM_TIME+2]<<16)|((uint32_t)pdata[TOF_ADDR_SYSTEM_TIME+3]<<24));
		pdata1->dis=(float)(((uint32_t)pdata[TOF_ADDR_DIS])|((uint32_t)pdata[TOF_ADDR_DIS+1]<<8)|
				                          ((uint32_t)pdata[TOF_ADDR_DIS+2]<<16)|((uint32_t)pdata[TOF_ADDR_DIS+3]<<24))/1000;
		pdata1->dis_status=(uint16_t)(((uint16_t)pdata[TOF_ADDR_DIS_STATUS])|((uint16_t)pdata[TOF_ADDR_DIS_STATUS+1]<<8));
		pdata1->signal_strength=(uint16_t)(((uint16_t)pdata[TOF_ADDR_SIGNAL_STRENGTH])|((uint16_t)pdata[TOF_ADDR_SIGNAL_STRENGTH+1]<<8));
		pdata1->range_precision=pdata[TOF_ADDR_RANGE_PRECISION];
	}

	return mark;//返回标志位
}



/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
static esp_err_t tof_register_read(uint8_t reg_addr, uint8_t *data, size_t len, uint8_t slave_addr)
{
    return i2c_master_read_from_device(I2C_MASTER_NUM, slave_addr, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

/**
 * @brief Write a byte to a MPU9250 sensor register
 */
static esp_err_t tof_register_write_byte(uint8_t reg_addr, uint8_t data, uint8_t slave_addr)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, slave_addr, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

/************************************************
函数名称 ： IIC_Get_All_Register_Data
函数功能 ： 通过IIC按顺序读取所有寄存器的数据并存入指定数组
参    数 ： *pdata缓存数组的指针  slave_addr从机地址
返 回 值 ： mark 操作是否正确标志位，0错误，1正确
*************************************************/
uint8_t IIC_Get_All_Register_Data(uint8_t *pdata, uint8_t slave_addr)
{
	uint8_t mark=0;//操作是否正确标志位

//	mark=TOF_IIC_Read_N_Byte(0x00,TOF_REGISTER_TOTAL_SIZE,pdata,slave_addr);
  mark = tof_register_read(0x00, pdata, TOF_REGISTER_TOTAL_SIZE, slave_addr);
	return mark;//返回标志位
}


/************************************************
函数名称 ： IIC_Change_Mode_To_UART
函数功能 ： 通过IIC将通信模式改为UART模式
参    数 ： slave_addr从机地址
返 回 值 ： mark 操作是否正确标志位，0错误，1正确
*************************************************/
uint8_t IIC_Change_Mode_To_UART(uint8_t slave_addr)
{
	uint8_t mark=0;//操作是否正确标志位
  mark = tof_register_write_byte(TOF_ADDR_MODE, IIC_CHANGE_TO_UART_DATA, ADDR_SLAVE);
	return mark;//返回标志位
}

/************************************************
函数名称 ： TOF_IIC_Read_N_Byte
函数功能 ： 通过IIC读取指定地址的N个字节数据，存入指定的数组中
参    数 ： Addr 需要读取的寄存器起始地址    num读取的字节数    *pdata读取缓存数组的指针  slave_addr从机地址
返 回 值 ： mark 操作是否正确标志位，0错误，1正确
*************************************************/
uint8_t TOF_IIC_Read_N_Byte(uint8_t Addr,uint8_t num,uint8_t *pdata,uint8_t slave_addr)
{
	uint8_t iic_ack=1;//从机响应状态变量，0表示应答
	uint8_t mark=0;//操作是否正确标志位
	uint16_t i=0;//循环计数变量
  uint8_t buffer[2048] = { 0 };

  i2c_cmd_handle_t handle = i2c_cmd_link_create_static(buffer, sizeof(buffer));
	i2c_master_start(handle);
  
//	Analog_IIC_Send_Byte((uint8_t)((slave_addr<<1)|0x00));//发送从机写地址
  iic_ack = i2c_master_write_byte(handle,(uint8_t)((slave_addr<<1)|I2C_MASTER_WRITE), true);
	//iic_ack=Analog_IIC_Wait_Ack();//等待从机响应
	if(iic_ack == ESP_OK )//如果从机响应
	{
		iic_ack=1;
		// Analog_IIC_Send_Byte(Addr);//发送需要读取的寄存器地址
		// iic_ack=Analog_IIC_Wait_Ack();//等待从机响应
    iic_ack = i2c_master_write_byte(handle, Addr, true);
		if(iic_ack == ESP_OK)//如果从机响应
		{
			iic_ack=1;
			//Analog_IIC_Stop();
      i2c_master_stop(handle);
			//Analog_IIC_Start();
      i2c_master_start(handle);
			// Analog_IIC_Send_Byte((uint8_t)((slave_addr<<1)|0x01));//发送从机读地址
			// iic_ack=Analog_IIC_Wait_Ack();//等待从机响应

      iic_ack = i2c_master_write_byte(handle,(uint8_t)((slave_addr<<1)|I2C_MASTER_READ), true);
			if(iic_ack == ESP_OK)//如果从机响应
			{
				iic_ack=1;
				for(i=0;i<num;i++)//读取num个字节并存入数组
				{
					if(i<num-1)
					{
            i2c_master_read_byte(handle,&pdata[i],true);
						//pdata[i]=Analog_IIC_Read_Byte(1);//读取一个字节并发送ACK;
					}
					else if(i == num-1)
					{
            i2c_master_read_byte(handle,&pdata[i],false);
						//pdata[i]=Analog_IIC_Read_Byte(0);//读取一个字节并发送NACK
						mark=1;//正确读取
						//Analog_IIC_Stop();
            i2c_master_stop(handle);
					}

				}

			}
		}
		else
		{
			//Analog_IIC_Stop();
      i2c_master_stop(handle);
				
		}
	}
	else
	{
		//Analog_IIC_Stop();
    i2c_master_stop(handle);
	}

	return mark;//返回标志位
}
