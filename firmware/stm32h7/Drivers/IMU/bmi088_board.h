/* BMI088 board lifecycle adapter.
 * SPI transactions and sensor configuration are implemented exclusively by
 * Damiao's unmodified CtrBoard-H7_IMU reference driver. */
#ifndef BMI088_BOARD_H
#define BMI088_BOARD_H

#include <stdint.h>

uint8_t BMI088_Board_Init(void);
uint8_t BMI088_Board_Read(float gyro[3], float accel[3], float *temperature);

#endif
