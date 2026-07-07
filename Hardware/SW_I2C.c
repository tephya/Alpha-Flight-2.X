#include "SW_I2C.h"
#include "Delay.h"

/**
  * @brief  初始化I2C1引脚，并拉高时钟线和数据线
  */
void IIC_Init(){
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;      // open-drain
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;  // external pull-up on PCB
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	Set_SCL;
	Set_SDA;
}

/**
  * @brief  写寄存器
  * @param  Reg  寄存器地址
  * @param  Data 待写入数据
  * @retval 0:成功 1/2/3:各阶段NACK错误
  */
uint8_t IIC_WriteReg(uint8_t addr, uint8_t Reg, uint8_t Data){
    uint8_t ret = 1;
    IIC_Start();
    if(IIC_SendSlaveAddr(addr, Write) == NAck){ goto stop; }  // slave addr + W
    if(IIC_SendRegAddr(Reg) == NAck){ goto stop; }  // register addr
    IIC_SendData(Data);
    if(IIC_WaitAck() == NAck){ goto stop; }         // data
	ret = 0;
stop:
    IIC_Stop();
    return ret;
}

/**
  * @brief  读寄存器
  * @param  Reg   寄存器地址
  * @param  Data  读取结果输出指针
  * @retval 0:成功 1/2/3:各阶段NACK错误
  */
uint8_t IIC_ReadReg(uint8_t addr, uint8_t Reg, uint8_t *Data){
    uint8_t ret = 1;
    IIC_Start();
    if(IIC_SendSlaveAddr(addr, Write) == NAck){ goto stop; }  // slave addr + W
    if(IIC_SendRegAddr(Reg) == NAck){ goto stop; }  // register addr
    IIC_Start();                                              // repeated START
    if(IIC_SendSlaveAddr(addr, Read) == NAck){ goto stop; }  // slave addr + R
    IIC_ReadData(Data);
    IIC_SendNAck();                                           // master NACK: stop reading
	ret = 0;
stop:
    IIC_Stop();
    return ret;
}

/**
  * @brief  发送从机地址
  * @param  rw  0:写 1:读
  * @retval Ack / NAck
  */
uint8_t IIC_SendSlaveAddr(uint8_t addr, uint8_t rw){
    IIC_SendData((addr << 1) | rw);
    return (IIC_WaitAck() == Ack) ? Ack : NAck;
}

/**
  * @brief  发送寄存器地址
  * @param  reg 寄存器地址
  * @retval Ack / NAck
  */
uint8_t IIC_SendRegAddr(uint8_t reg){
    IIC_SendData(reg);
    return (IIC_WaitAck() == Ack) ? Ack : NAck;
}

/**
  * @brief  产生I2C START条件
  *         SCL高电平期间SDA下降沿
  */
void IIC_Start(){
    Set_SCL;
	
    Set_SDA;
    Delay_us(10);
    Reset_SDA;   // SDA falling down while SCL high -> START
	
    Delay_us(10);
    Reset_SCL;
}

/**
  * @brief  产生I2C STOP条件
  *         SCL高电平期间SDA上升沿
  */
void IIC_Stop(){
    Reset_SDA;
    Set_SCL;
    Delay_us(10);
    Set_SDA;     // SDA raising while SCL high -> STOP
}

/**
  * @brief  等待从机ACK（第9个时钟脉冲）
  * @retval Ack(SDA=0) / NAck(SDA=1)
  */
uint8_t IIC_WaitAck(){
    Reset_SCL;
    Set_SDA;        // release SDA, slave drives it
    Set_SCL;		// MCU(Master) Sampling SDA
    Delay_us(5);
    uint8_t ack = (SDA_VAL == 0) ? Ack : NAck;	// Read SDA
    Reset_SCL;
    return ack;
}

/**
  * @brief  主机发送NACK（通知从机停止发送）
  */
void IIC_SendNAck(){
    Reset_SCL;
    Set_SDA;        // NACK: SDA high
    Set_SCL;		// Slave Sampling SDA
    Delay_us(5);
    Reset_SCL;
}

void IIC_SendAck(){
    Reset_SCL;
    Reset_SDA;      // ACK: SDA low
    Set_SCL;		// Slave Sampling SDA
    Delay_us(5);
    Reset_SCL;
    Set_SDA;        // 释放SDA
}

/**
  * @brief  发送1字节数据，MSB first
  * @param  Data 待发送数据
  */
void IIC_SendData(uint8_t Data){
    for(int i = 0; i < 8; i++){
        Reset_SCL;					/* 拉低SCL，SCL mos导通，此时SCL被Master选中，Master可以修改SDA电平 */
        if((Data & (0x80 >> i)) != 0)
            Set_SDA;				/* 为什么不是 Reset SDA，见头文件关于 Set 的说明 */
        else
            Reset_SDA;
        Set_SCL;					/* 拉高SCL，让SDA稳定，Slave读SDA */
        Delay_us(5);
    }
    Reset_SCL;
}

/**
  * @brief  接收1字节数据，MSB first
  * @param  Data 接收结果输出指针
  */
void IIC_ReadData(uint8_t *Data){
	*Data = 0;
    Set_SDA;        // release SDA, slave drives it
    for(int i = 0; i < 8; i++){
        Set_SCL;			// SCL高：主机等数据稳定； 从机等
        Delay_us(5);
        *Data <<= 1;
        if(SDA_VAL == 1)	// 主机采样
            *Data |= 0x01;
        Reset_SCL;			// SCL低：从机准备下一bit； 主机等
        Delay_us(5);
    }
}