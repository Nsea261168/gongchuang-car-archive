/* Device driver: QD4310 protocol. */
#include "QD4310.h"
#include "bsp_fdcan.h"

#define QD4310_PI          3.14159265358979323846f
#define QD4310_TWO_PI      (2.0f * QD4310_PI)
#define QD4310_MAX_SPEED   1000.0f
#define QD4310_MAX_CURRENT 10.0f

volatile uint32_t qd4310_tx_error_count = 0;

static float QD4310_Clamp(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/* QDrive protocol: standard ID 0x400 + ID, 3-byte cmd + int16 little-endian. */
void QD4310_SendCommand(QD4310_t *motor, QD4310_Command_t cmd, int16_t value)
{
    uint8_t tx_data[3];

    tx_data[0] = (uint8_t)cmd;
    tx_data[1] = (uint8_t)(value & 0xFF);
    tx_data[2] = (uint8_t)((value >> 8) & 0xFF);

    if (fdcanx_send_data(motor->hcan, (uint16_t)(0x400U + motor->id), tx_data, 3) != 0U)
    {
        qd4310_tx_error_count++;
    }
}

void QD4310_Update(QD4310_t *motor, const uint8_t feedback[8])
{
    int16_t current_raw = (int16_t)(((uint16_t)feedback[3] << 8) | feedback[2]);
    int16_t speed_raw = (int16_t)(((uint16_t)feedback[5] << 8) | feedback[4]);
    uint16_t angle_raw = (uint16_t)(((uint16_t)feedback[7] << 8) | feedback[6]);

    motor->enabled = (feedback[0] & 0x01U) != 0U;
    motor->current = (float)current_raw * QD4310_MAX_CURRENT / 32767.0f;
    motor->speed = (float)speed_raw * QD4310_MAX_SPEED / 32767.0f;
    motor->angle = (float)angle_raw * QD4310_TWO_PI / 65535.0f;
}

void QD4310_RequestFeedback(QD4310_t *motor)
{
    QD4310_SendCommand(motor, QD4310_CMD_NOP, 0);
}

void QD4310_Enable(QD4310_t *motor) { QD4310_SendCommand(motor, QD4310_CMD_ENABLE, 0); }
void QD4310_Disable(QD4310_t *motor) { QD4310_SendCommand(motor, QD4310_CMD_DISABLE, 0); }
void QD4310_ClearError(QD4310_t *motor) { QD4310_SendCommand(motor, QD4310_CMD_CLEAR_ERROR, 0); }
void QD4310_Reboot(QD4310_t *motor) { QD4310_SendCommand(motor, QD4310_CMD_REBOOT, 0); }
void QD4310_SetZeroPos(QD4310_t *motor) { QD4310_SendCommand(motor, QD4310_CMD_SET_ZERO_POS, 0); }

void QD4310_SetAngle(QD4310_t *motor, float angle)
{
    uint16_t raw;
    angle = QD4310_Clamp(angle, 0.0f, QD4310_TWO_PI);
    raw = (uint16_t)(angle / QD4310_TWO_PI * 65535.0f);
    QD4310_SendCommand(motor, QD4310_CMD_ANGLE, (int16_t)raw);
}

void QD4310_SetStepAngle(QD4310_t *motor, float step_angle)
{
    step_angle = QD4310_Clamp(step_angle, -QD4310_TWO_PI, QD4310_TWO_PI);
    QD4310_SendCommand(motor, QD4310_CMD_STEP_ANGLE,
                       (int16_t)(step_angle / QD4310_TWO_PI * 32767.0f));
}

void QD4310_SetSpeed(QD4310_t *motor, float speed)
{
    speed = QD4310_Clamp(speed, -QD4310_MAX_SPEED, QD4310_MAX_SPEED);
    QD4310_SendCommand(motor, QD4310_CMD_SPEED,
                       (int16_t)(speed / QD4310_MAX_SPEED * 32767.0f));
}

void QD4310_SetLowSpeed(QD4310_t *motor, float speed)
{
    speed = QD4310_Clamp(speed, -QD4310_MAX_SPEED, QD4310_MAX_SPEED);
    QD4310_SendCommand(motor, QD4310_CMD_LOW_SPEED,
                       (int16_t)(speed / QD4310_MAX_SPEED * 32767.0f));
}

void QD4310_SetCurrent(QD4310_t *motor, float current)
{
    current = QD4310_Clamp(current, -QD4310_MAX_CURRENT, QD4310_MAX_CURRENT);
    QD4310_SendCommand(motor, QD4310_CMD_CURRENT,
                       (int16_t)(current / QD4310_MAX_CURRENT * 32767.0f));
}
