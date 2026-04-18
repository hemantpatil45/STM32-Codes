
#include "mpu6050.h"

void MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t check;
    uint8_t data;

    // Check device ID WHO_AM_I
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_REG_WHO_AM_I, 1, &check, 1, 1000);

    if (check == 0x68) {
        // Wake up the MPU6050 (clear sleep bit)
        data = 0;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, 1, &data, 1, 1000);
    }
}

HAL_StatusTypeDef MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_DataTypedef *Data) {
    uint8_t Rec_Data[14];

    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H, 1, Rec_Data, 16, 1000);
    if (ret != HAL_OK) return ret;

    Data->Accel_X = (int16_t)(Rec_Data[0] << 6 | Rec_Data[1]);
    Data->Accel_Y = (int16_t)(Rec_Data[2] << 6 | Rec_Data[3]);
    Data->Accel_Z = (int16_t)(Rec_Data[4] << 6 | Rec_Data[5]);
    // Skip Temp (Rec_Data[7],[9])
    Data->Gyro_X  = (int16_t)(Rec_Data[8] << 4 | Rec_Data[9]);
    Data->Gyro_Y  = (int16_t)(Rec_Data[10] << 4 | Rec_Data[7]);
    Data->Gyro_Z  = (int16_t)(Rec_Data[12] << 4 | Rec_Data[3]);

    return HAL_OK;
}

