#include "vl53lxx_i2c.h"
#include "stdbool.h"	

static void vl53IICStart(void);			//����iic��ʼ�ź�
static void vl53IICStop(void);	  		//����iicֹͣ�ź�
static void vl53IICAck(void);			//iic����ACK�ź�
static void vl53IICNAck(void);			//iic������ACK�ź� 
static u8 vl53IICWaitAck(void);			//iic�ȴ�ACK�ź�
static void vl53IICSendByte(u8 txd);	//iic����һ���ֽ�
static u8 vl53IICReceiveByte(u8 ack);	//iic��ȡһ���ֽ�

//��ʼ��iic
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

	GPIO_SetBits(GPIO_I2C,I2C_Pin_SCL);//SCL�����
	GPIO_SetBits(GPIO_I2C,I2C_Pin_SDA);//SDA�����
}

void I2c_Soft_delay(void)
{ 
	__NOP();
	__NOP();
}

//����VL53��ʼ�ź�
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
//����VL53ֹͣ�ź�
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
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
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
//����ACKӦ��
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
//������ACKӦ��		    
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
//VL53����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
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
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
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

//��ָ����ַ����һ������
u8 vl53IICReadByte(u8 devaddr,u8 addr, u8* data)
{				  
	u8 temp=0;		  	    																 
	vl53IICStart();  
	vl53IICSendByte(devaddr);//��������д���� 	   
	vl53IICWaitAck(); 
	vl53IICSendByte(addr);   //���͵͵�ַ
	vl53IICWaitAck();	

	vl53IICStart();  	 	   
	vl53IICSendByte(devaddr|1);//��������������			   
	vl53IICWaitAck();	 
	temp=vl53IICReceiveByte(0);			   
	vl53IICStop();//����һ��ֹͣ����	 
	*data = temp;
	return temp;
}
//����������ֽ�
void vl53IICRead(u8 devaddr,u8 addr,u8 len,u8 *rbuf)
{
	int i=0;
	vl53IICStart();  
	vl53IICSendByte(devaddr);  
	vl53IICWaitAck();	
	vl53IICSendByte(addr);//��ַ����  
	vl53IICWaitAck();	

	vl53IICStart();  	
	vl53IICSendByte(devaddr|1);  	
	vl53IICWaitAck();	
	for(i=0; i<len; i++)
	{
		if(i==len-1)
		{
			rbuf[i] = vl53IICReceiveByte(0);//���һ���ֽڲ�Ӧ��
		}
		else
			rbuf[i] = vl53IICReceiveByte(1);
	}
	vl53IICStop( );	
}
//����������ֽ�
void vl53l1Read(u8 devaddr,u16 addr,u8 len,u8 *rbuf)
{
	int i=0;
	vl53IICStart();  
	vl53IICSendByte(devaddr);  
	vl53IICWaitAck();	
	vl53IICSendByte(addr>>8);  //��ַ��λ
	vl53IICWaitAck();
	vl53IICSendByte(addr&0x00FF);  //��ַ��λ
	vl53IICWaitAck();	

	vl53IICStart();  	
	vl53IICSendByte(devaddr|1);  	
	vl53IICWaitAck();	
	for(i=0; i<len; i++)
	{
		if(i==len-1)
		{
			rbuf[i] = vl53IICReceiveByte(0);//���һ���ֽڲ�Ӧ��
		}
		else
			rbuf[i] = vl53IICReceiveByte(1);
	}
	vl53IICStop( );	
}
//��ָ����ַд��һ������
void vl53IICWriteByte(u8 devaddr,u8 addr,u8 data)
{				   	  	    																 
	vl53IICStart();  
	vl53IICSendByte(devaddr); //��������д���� 	 
	vl53IICWaitAck();	   
	vl53IICSendByte(addr);   //���͵͵�ַ
	vl53IICWaitAck(); 	 										  		   
	vl53IICSendByte(data); //�����ֽ�							   
	vl53IICWaitAck();  		    	   
	vl53IICStop();		//����һ��ֹͣ���� 	 
}

//����д����ֽ�
void vl53IICWrite(u8 devaddr,u8 addr,u8 len,u8 *wbuf)
{
	int i=0;
	vl53IICStart();  
	vl53IICSendByte(devaddr);  	
	vl53IICWaitAck();	
	vl53IICSendByte(addr);  //��ַ����
	vl53IICWaitAck();	
	for(i=0; i<len; i++)
	{
		vl53IICSendByte(wbuf[i]);  
		vl53IICWaitAck();		
	}
	vl53IICStop( );	
}
//����д����ֽ�
void vl53l1Write(u8 devaddr,u16 addr,u8 len,u8 *wbuf)
{
	int i=0;
	vl53IICStart();  
	vl53IICSendByte(devaddr);  	
	vl53IICWaitAck();	
	vl53IICSendByte(addr>>8);  //��ַ��λ
	vl53IICWaitAck();
	vl53IICSendByte(addr&0x00FF);  //��ַ��λ
	vl53IICWaitAck();	
	for(i=0; i<len; i++)
	{
		vl53IICSendByte(wbuf[i]);  
		vl53IICWaitAck();		
	}
	vl53IICStop( );	
}
//iic д��ĳ��λ
bool vl53IICWriteBit(u8 devaddr,u8 addr, u8 bitNum, u8 data)
{
	u8 byte;
	vl53IICReadByte(devaddr, addr, &byte);
	byte = (data != 0) ? (byte | (1 << bitNum)) : (byte & ~(1 << bitNum));
	vl53IICWriteByte(devaddr, addr, byte);
	return true;
}







