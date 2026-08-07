/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */
#define IIC_TIMEOUT_MS  100   // IIC通信超时时间

/* Bus recovery不要求400kHz速度，1ms低速脉冲更便于从机识别，
 * 且恢复只在通信已经失败时执行，不影响正常FlightCtrl周期。 */
static void IIC_BusRecoveryDelay(void)
{
  HAL_Delay(1U);
}
/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = SCL_Pin|SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(SCL_GPIO_Port, SCL_Pin);

    HAL_GPIO_DeInit(SDA_GPIO_Port, SDA_Pin);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**
 * @brief   写指定寄存器
 * @param   dev_addr  7位从机地址
 * @param   reg_addr  寄存器地址
 * @param   data      待写入数据
 * @retval  HAL_OK=成功，其余为HAL错误码
 */
HAL_StatusTypeDef IIC_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
  return HAL_I2C_Mem_Write(&hi2c1,
                          (uint16_t)(dev_addr << 1),      // 7位地址左移1位拼接8位地址
                          reg_addr,
                          I2C_MEMADD_SIZE_8BIT,
                          &data,
                          1,
                          IIC_TIMEOUT_MS);
}


/**
 * @brief   读指定寄存器
 * @param   dev_addr  7位从机地址
 * @param   reg_addr  寄存器地址
 * @param   data      读寄存器结果输出指针
 * @retval  HAL_OK=成功，其余为HAL错误码
 */
HAL_StatusTypeDef IIC_ReadReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data)
{
  return HAL_I2C_Mem_Read(&hi2c1,
                          (uint16_t)(dev_addr << 1),
                          reg_addr,
                          I2C_MEMADD_SIZE_8BIT,
                          data,
                          1,
                          IIC_TIMEOUT_MS);
}


/**
 * @brief   从指定寄存器开始连续读多个字节
 * @param   dev_addr  7位从机地址
 * @param   reg_addr  起始寄存器地址
 * @param   buf       接收缓冲区
 * @param   len       读收字节数
 * @retval  HAL_OK=写成功，其余为HAL错误码
 */
HAL_StatusTypeDef IIC_ReadBurst(uint8_t dev_addr, uint8_t reg_addr, uint8_t *buf, uint16_t len)
{
  return HAL_I2C_Mem_Read(&hi2c1,
                          (uint16_t)(dev_addr << 1),
                          reg_addr,
                          I2C_MEMADD_SIZE_8BIT,
                          buf,
                          len,
                          IIC_TIMEOUT_MS);
}

/**
 * @brief   恢复SDA被从机持续拉低的I2C1 Bus
 * @note    当前工程I2C1使用PB6=SCL、PB7=SDA，且Bus上只有QMC。
 *          恢复完成后重新初始化I2C1，但不负责重新配置QMC寄存器。
 * @retval  HAL_OK : SCL/SDA均已释放，I2C1不再处于BUSY
 *          HAL_BUSY : 恢复后线路或外设仍处于BUSY
 */
HAL_StatusTypeDef IIC_RecoverBus(void)
{
  GPIO_InitTypeDef gpio = {0};

  /* 关闭I2C外设并释放其对PB6/PB7复用功能的控制。
   * HAL_I2C_DeInit同时会调用HAL_I2C_MspDeInit。 */
  (void)HAL_I2C_DeInit(&hi2c1);

  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* 先将ODR置高，再切换为开漏输出，避免切换瞬间主动拉低线路。
   * 开漏输出“高”实际表现释放线路，由外部上拉产生高电平。 */
  HAL_GPIO_WritePin(GPIOB, SCL_Pin | SDA_Pin, GPIO_PIN_SET);

  gpio.Pin = SCL_Pin | SDA_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);

  IIC_BusRecoveryDelay();

  for (uint8_t pulse = 0U; 
      pulse < 9U && 
      HAL_GPIO_ReadPin(SDA_GPIO_Port, SDA_Pin) == GPIO_PIN_RESET; 
      pulse++)
  {
    HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, GPIO_PIN_RESET);
    IIC_BusRecoveryDelay();

    HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, GPIO_PIN_SET);
    IIC_BusRecoveryDelay();
  }

  /* 手工产生STOP：SCL为高期间，SDA从低跳变为高 */
  HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, GPIO_PIN_RESET);
  IIC_BusRecoveryDelay();

  HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, GPIO_PIN_SET);
  IIC_BusRecoveryDelay();

  HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, GPIO_PIN_SET);
  IIC_BusRecoveryDelay();

  const GPIO_PinState scl_state = HAL_GPIO_ReadPin(SCL_GPIO_Port, SCL_Pin);
  const GPIO_PinState sda_state = HAL_GPIO_ReadPin(SDA_GPIO_Port, SDA_Pin);

  /* 恢复I2C1复用功能和HAL状态。
   * MX_I2C1_Init内部会重新配置PB6/PB7为AF_OD。 */
  MX_I2C1_Init();

  if(scl_state != GPIO_PIN_SET || sda_state != GPIO_PIN_SET)
    return HAL_BUSY;

  if(__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) != RESET)
    return HAL_BUSY;

  return HAL_OK;
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
