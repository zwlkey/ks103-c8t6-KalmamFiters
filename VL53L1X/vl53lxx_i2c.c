#include "vl53lxx_i2c.h"
#include "stdbool.h"	

static void vl53IICStart(void);			//发送iic开始信号
static void vl53IICStop(void);	  		//发送iic停止信号
static void vl53IICAck(void);			//iic发送ACK信号
static void vl53IICNAck(void);			//iic不发送ACK信号 
static u8 vl53IICWaitAck(void);			//iic等待ACK信号
static void vl53IICSendByte(u8 txd);	//iic发送一个字节
static u8 vl53IICReceiveByte(u8 ack);	//iic读取一个字节

//初始化iic
void vl53IICInit(void)
{	
  GPIO_InitTypeDef  GPIO_InitStructure; 
	
  RCC_APB2PeriphClockCmd(RCC_I2C , ENABLE );
	
  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SCL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  
  GPIO_Init(GPIO_I2C, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SDA;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(GPIO_I2C, &GPIO_InitStructure);		

	GPIO_SetBits(GPIO_I2C,I2C_Pin_SCL);//SCL输出高
	GPIO_SetBits(GPIO_I2C,I2C_Pin_SDA);//SDA输出高
}

void I2c_Soft_delay(void)
{ 
	__NOP();
	__NOP();
}

//产生VL53起始信号
static void vl53IICStart(void)
{
	SDA_H;
	SCL_H;
	I2c_Soft_delay();
	SDA_L;
	I2c_Soft_delay();
	SDA_L;
	I2c_Soft_delay();
}	  
//产生VL53停止信号
static void vl53IICStop(void)
{
	SCL_L;
	I2c_Soft_delay();
	SDA_L;
	I2c_Soft_delay();
	SCL_H;
	I2c_Soft_delay();
	SDA_H;
	I2c_Soft_delay();					   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
static u8 vl53IICWaitAck(void)
{
  u8 ErrTime = 0;
	SCL_L;
	I2c_Soft_delay();
	SDA_H;			
	I2c_Soft_delay();
	SCL_H;
	I2c_Soft_delay();
	if(READ_SDA)
	{
		ErrTime++;
		if(ErrTime>200)
		{
			vl53IICStop();
			return 1;
		}
	}
	SCL_L;
	I2c_Soft_delay();
	return 0;
} 
//产生ACK应答
static void vl53IICAck(void)
{
	SCL_L;
	I2c_Soft_delay();
	SDA_L;
	I2c_Soft_delay();
	SCL_H;
	I2c_Soft_delay();
	SCL_L;
	I2c_Soft_delay();
}
//不产生ACK应答		    
static void vl53IICNAck(void)
{
	SCL_L;
	I2c_Soft_delay();
	SDA_H;
	I2c_Soft_delay();
	SCL_H;
	I2c_Soft_delay();
	SCL_L;
	I2c_Soft_delay();
}					 				     
//VL53发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
static void vl53IICSendByte(u8 SendByte)
{                        
	u8 i=8;
	while(i--)
	{
			SCL_L;
			I2c_Soft_delay();
		if(SendByte&0x80)
			SDA_H;  
		else 
			SDA_L;   
			SendByte<<=1;
			I2c_Soft_delay();
			SCL_H;
			I2c_Soft_delay();
	}
	SCL_L;
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
static u8 vl53IICReceiveByte(u8 ack)
{
    u8 i=8;
    u8 ReceiveByte=0;

    SDA_H;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L;
      I2c_Soft_delay();
			SCL_H;
      I2c_Soft_delay();	
      if(READ_SDA)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L;

	if(ack)
		vl53IICAck();
	else
		vl53IICNAck();  
    return ReceiveByte;
}

//从指定地址读出一个数据
u8 vl53IICReadByte(u8 devaddr,u8 addr, u8* data)
{				  
	u8 temp=0;		  	    																 
	vl53IICStart();  
	vl53IICSendByte(devaddr);//发送器件写命令 	   
	vl53IICWaitAck(); 
	vl53IICSendByte(addr);   //发送低地址
	vl53IICWaitAck();	

	vl53IICStart();  	 	   
	vl53IICSendByte(devaddr|1);//发送器件读命令			   
	vl53IICWaitAck();	 
	temp=vl53IICReceiveByte(0);			   
	vl53IICStop();//产生一个停止条件	 
	*data = temp;
	return temp;
}
//连续读多个字节
void vl53IICRead(u8 devaddr,u8 addr,u8 len,u8 *rbuf)
{
	int i=0;
	vl53IICStart();  
	vl53IICSendByte(devaddr);  
	vl53IICWaitAck();	
	vl53IICSendByte(addr);//地址自增  
	vl53IICWaitAck();	

	vl53IICStart();  	
	vl53IICSendByte(devaddr|1);  	
	vl53IICWaitAck();	
	for(i=0; i<len; i++)
	{
		if(i==len-1)
		{
			rbuf[i] = vl53IICReceiveByte(0);//最后一个字节不应答
		}
		else
			rbuf[i] = vl53IICReceiveByte(1);
	}
	vl53IICStop( );	
}
//连续读多个字节
void vl53l1Read(u8 devaddr,u16 addr,u8 len,u8 *rbuf)
{
	int i=0;
	vl53IICStart();  
	vl53IICSendByte(devaddr);  
	vl53IICWaitAck();	
	vl53IICSendByte(addr>>8);  //地址高位
	vl53IICWaitAck();
	vl53IICSendByte(addr&0x00FF);  //地址低位
	vl53IICWaitAck();	

	vl53IICStart();  	
	vl53IICSendByte(devaddr|1);  	
	vl53IICWaitAck();	
	for(i=0; i<len; i++)
	{
		if(i==len-1)
		{
			rbuf[i] = vl53IICReceiveByte(0);//最后一个字节不应答
		}
		else
			rbuf[i] = vl53IICReceiveByte(1);
	}
	vl53IICStop( );	
}
//从指定地址写入一个数据
void vl53IICWriteByte(u8 devaddr,u8 addr,u8 data)
{				   	  	    																 
	vl53IICStart();  
	vl53IICSendByte(devaddr); //发送器件写命令 	 
	vl53IICWaitAck();	   
	vl53IICSendByte(addr);   //发送低地址
	vl53IICWaitAck(); 	 										  		   
	vl53IICSendByte(data); //发送字节							   
	vl53IICWaitAck();  		    	   
	vl53IICStop();		//产生一个停止条件 	 
}

//连续写多个字节
void vl53IICWrite(u8 devaddr,u8 addr,u8 len,u8 *wbuf)
{
	int i=0;
	vl53IICStart();  
	vl53IICSendByte(devaddr);  	
	vl53IICWaitAck();	
	vl53IICSendByte(addr);  //地址自增
	vl53IICWaitAck();	
	for(i=0; i<len; i++)
	{
		vl53IICSendByte(wbuf[i]);  
		vl53IICWaitAck();		
	}
	vl53IICStop( );	
}
//连续写多个字节
void vl53l1Write(u8 devaddr,u16 addr,u8 len,u8 *wbuf)
{
	int i=0;
	vl53IICStart();  
	vl53IICSendByte(devaddr);  	
	vl53IICWaitAck();	
	vl53IICSendByte(addr>>8);  //地址高位
	vl53IICWaitAck();
	vl53IICSendByte(addr&0x00FF);  //地址低位
	vl53IICWaitAck();	
	for(i=0; i<len; i++)
	{
		vl53IICSendByte(wbuf[i]);  
		vl53IICWaitAck();		
	}
	vl53IICStop( );	
}
//iic 写入某个位
bool vl53IICWriteBit(u8 devaddr,u8 addr, u8 bitNum, u8 data)
{
	u8 byte;
	vl53IICReadByte(devaddr, addr, &byte);
	byte = (data != 0) ? (byte | (1 << bitNum)) : (byte & ~(1 << bitNum));
	vl53IICWriteByte(devaddr, addr, byte);
	return true;
}







