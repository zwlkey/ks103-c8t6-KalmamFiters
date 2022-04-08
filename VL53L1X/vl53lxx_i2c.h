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

//VL53所有操作函数
void vl53IICInit(void);			/*初始化VL53的IO口*/				 
u8 vl53IICReadByte(u8 devaddr,u8 addr, u8* data);		/*读一字节*/
void vl53IICWriteByte(u8 devaddr,u8 addr,u8 data);		/*写一字节*/
void vl53IICRead(u8 devaddr,u8 addr,u8 len,u8 *rbuf);	/*连续读取多个字节*/
void vl53IICWrite(u8 devaddr,u8 addr,u8 len,u8 *wbuf);	/*连续写入多个字节*/
bool vl53IICWriteBit(u8 devaddr,u8 addr, u8 bitNum, u8 data);	/*iic 写入某个位*/

void vl53l1Read(u8 devaddr,u16 addr,u8 len,u8 *rbuf);	/*连续读取多个字节*/
void vl53l1Write(u8 devaddr,u16 addr,u8 len,u8 *wbuf);	/*连续写入多个字节*/
	
#endif 


