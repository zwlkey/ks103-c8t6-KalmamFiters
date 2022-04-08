#ifndef __VL53LXX_I2C_H
#define __VL53LXX_I2C_H

#include "sys.h"
#include "stdbool.h"

#define SCL_H         GPIO_I2C->BSRR = I2C_Pin_SCL
#define SCL_L         GPIO_I2C->BRR  = I2C_Pin_SCL
#define SDA_H         GPIO_I2C->BSRR = I2C_Pin_SDA
#define SDA_L         GPIO_I2C->BRR  = I2C_Pin_SDA
#define READ_SDA      GPIO_I2C->IDR  & I2C_Pin_SDA


#define RCC_I2C		RCC_APB2Periph_GPIOB
#define GPIO_I2C	GPIOB
#define I2C_Pin_SCL		GPIO_Pin_10
#define I2C_Pin_SDA		GPIO_Pin_11

//VL53���в�������
void vl53IICInit(void);			/*��ʼ��VL53��IO��*/				 
u8 vl53IICReadByte(u8 devaddr,u8 addr, u8* data);		/*��һ�ֽ�*/
void vl53IICWriteByte(u8 devaddr,u8 addr,u8 data);		/*дһ�ֽ�*/
void vl53IICRead(u8 devaddr,u8 addr,u8 len,u8 *rbuf);	/*������ȡ����ֽ�*/
void vl53IICWrite(u8 devaddr,u8 addr,u8 len,u8 *wbuf);	/*����д�����ֽ�*/
bool vl53IICWriteBit(u8 devaddr,u8 addr, u8 bitNum, u8 data);	/*iic д��ĳ��λ*/

void vl53l1Read(u8 devaddr,u16 addr,u8 len,u8 *rbuf);	/*������ȡ����ֽ�*/
void vl53l1Write(u8 devaddr,u16 addr,u8 len,u8 *wbuf);	/*����д�����ֽ�*/
	
#endif 


