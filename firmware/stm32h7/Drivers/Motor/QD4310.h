/* Device driver: QD4310 interface. */
#ifndef QD4310_H
#define QD4310_H

#include <stdbool.h>
#include <stdint.h>
#include "fdcan.h"

typedef enum {
    QD4310_CMD_NOP = 0x00,
    QD4310_CMD_ENABLE = 0x01,
    QD4310_CMD_DISABLE = 0x02,
    QD4310_CMD_CURRENT = 0x03,
    QD4310_CMD_SPEED = 0x04,
    QD4310_CMD_ANGLE = 0x05,
    QD4310_CMD_LOW_SPEED = 0x06,
    QD4310_CMD_STEP_ANGLE = 0x07,
    QD4310_CMD_CLEAR_ERROR = 0xFB,
    QD4310_CMD_SET_ZERO_POS = 0xFE,
    QD4310_CMD_REBOOT = 0xFF,
} QD4310_Command_t;

typedef struct {
    volatile bool enabled;
    uint8_t id;
    volatile float speed;
    volatile float angle;
    volatile float current;
    FDCAN_HandleTypeDef *hcan;
} QD4310_t;

extern QD4310_t Motor_0;
extern QD4310_t Motor_1;
extern volatile uint32_t qd4310_tx_error_count;
extern volatile uint32_t qd4310_feedback_count;
extern volatile uint32_t qd4310_feedback_count_motor0;
extern volatile uint32_t qd4310_feedback_count_motor1;
extern volatile uint8_t qd4310_active_can;
extern volatile uint32_t qd4310_feedback_tick_motor0;
extern volatile uint32_t qd4310_feedback_tick_motor1;

void QD4310_SendCommand(QD4310_t *motor, QD4310_Command_t cmd, int16_t value);
/* Requests one feedback frame without changing the active control target. */
void QD4310_RequestFeedback(QD4310_t *motor);
void QD4310_Update(QD4310_t *motor, const uint8_t feedback[8]);
void QD4310_Enable(QD4310_t *motor);
void QD4310_Disable(QD4310_t *motor);
void QD4310_ClearError(QD4310_t *motor);
void QD4310_Reboot(QD4310_t *motor);
void QD4310_SetZeroPos(QD4310_t *motor);
void QD4310_SetAngle(QD4310_t *motor, float angle);
void QD4310_SetStepAngle(QD4310_t *motor, float step_angle);
void QD4310_SetSpeed(QD4310_t *motor, float speed);
void QD4310_SetLowSpeed(QD4310_t *motor, float speed);
void QD4310_SetCurrent(QD4310_t *motor, float current);

#endif
