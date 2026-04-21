#include "hmc5883l.h"

void HMC5883L_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t configA[2] = {0x00, 0x70};
    uint8_t configB[2] = {0x01, 0xA0};
    uint8_t mode[2]    = {0x02, 0x00};

    HAL_I2C_Master_Transmit(hi2c, HMC5883L_ADDR, configA, 2, HAL_MAX_DELAY);
    HAL_I2C_Master_Transmit(hi2c, HMC5883L_ADDR, configB, 2, HAL_MAX_DELAY);
    HAL_I2C_Master_Transmit(hi2c, HMC5883L_ADDR, mode, 2, HAL_MAX_DELAY);
}

void HMC5883L_Read(I2C_HandleTypeDef *hi2c, int16_t *mx, int16_t *my, int16_t *mz)
{
    uint8_t data[6];
    uint8_t reg = 0x03;

    HAL_I2C_Master_Transmit(hi2c, HMC5883L_ADDR, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(hi2c, HMC5883L_ADDR, data, 6, HAL_MAX_DELAY);

    *mx = (int16_t)(data[0] << 8 | data[1]);
    *mz = (int16_t)(data[2] << 8 | data[3]);
    *my = (int16_t)(data[4] << 8 | data[5]);
}
