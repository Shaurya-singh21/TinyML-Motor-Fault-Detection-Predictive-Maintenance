#ifndef __I2C_H
#define __I2C_H

void i2c_init(void);
void i2c2_oled_init(void);
void MPU6050_Init();
#define I2C_FLAG_TIMEOUT 20000U
#endif
