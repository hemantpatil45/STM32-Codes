#ifndef __HMC5883L_H
#define __HMC5883L_H

#include "stm32f1xx_hal.h"   // change if using different MCU

#define HMC5883L_ADDR (0x1E << 1)  // 7-bit address shifted for HAL

void HMC5883L_Init(I2C_HandleTypeDef *hi2c);
void HMC5883L_Read(I2C_HandleTypeDef *hi2c, int16_t *mx, int16_t *my, int16_t *mz);

#endif
