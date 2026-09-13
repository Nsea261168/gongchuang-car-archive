#include "songjia_motor.h"

#include <stdio.h>
#include <string.h>

#include "bsp_uart7.h"

#define SONGJIA_FRAME_MAX_LENGTH  80U
#define SONGJIA_PARSE_BUDGET      64U
#define SONGJIA_SPEED_LIMIT_CENTI 127
#define SONGJIA_PWM_LIMIT_PERMILLE 1000
#define SONGJIA_POLARITY_RETRY_MS 100U

static const char sEncoderPrefix[] = "$MOTOR_4CH_Encoder_20ms:";
static char sRxFrame[SONGJIA_FRAME_MAX_LENGTH];
static uint8_t sRxLength;
static uint8_t sCollecting;
static uint8_t sInitialized;
static SongjiaMotorFeedback_t sFeedback;
static SongjiaEncoderPolarityState_t sPolarity;
static uint8_t sPolarityTarget;
static uint8_t sPolarityPending;

static int16_t ClampI16(int16_t value, int16_t low, int16_t high)
{
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

static uint8_t SendText(const char *text)
{
  return BspUart7_Write((const uint8_t *)text, (uint16_t)strlen(text));
}

static uint8_t ParseSigned16(const char **cursor, int16_t *value)
{
  int32_t result = 0;
  int32_t sign = 1;
  uint8_t digit_count = 0U;

  if (**cursor == '-')
  {
    sign = -1;
    (*cursor)++;
  }
  else if (**cursor == '+')
  {
    (*cursor)++;
  }

  while ((**cursor >= '0') && (**cursor <= '9'))
  {
    result = result * 10 + (int32_t)(**cursor - '0');
    if (result > 32768)
      return 0U;
    (*cursor)++;
    digit_count++;
  }
  result *= sign;
  if ((digit_count == 0U) || (result < -32768) || (result > 32767))
    return 0U;
  *value = (int16_t)result;
  return 1U;
}

static void ParseFrame(uint32_t now_ms)
{
  const char *cursor;
  int16_t encoder[4];
  uint8_t index;

  sRxFrame[sRxLength] = '\0';
  {
    static const char prefix[] = "$MOTOR_4CH_SET_ENCPDER_POLARITY_OK:";
    size_t prefix_len = sizeof(prefix) - 1U;
    if (strncmp(sRxFrame, prefix, prefix_len) == 0)
    {
      if (sRxFrame[prefix_len + 1U] == '\0' &&
          (sRxFrame[prefix_len] == '0' || sRxFrame[prefix_len] == '1'))
      {
        sPolarity.value = (uint8_t)(sRxFrame[prefix_len] - '0');
        if (sPolarity.value == sPolarityTarget)
        {
          sPolarity.confirmed = 1U;
          sPolarity.ack_tick_ms = now_ms;
          sPolarityPending = 0U;
        }
        return;
      }
      sPolarity.invalid_ack_count++;
      return;
    }
  }
  if (strncmp(sRxFrame, sEncoderPrefix, sizeof(sEncoderPrefix) - 1U) != 0)
    return;

  cursor = &sRxFrame[sizeof(sEncoderPrefix) - 1U];
  for (index = 0U; index < 4U; index++)
  {
    if (ParseSigned16(&cursor, &encoder[index]) == 0U)
      break;
    if (index < 3U)
    {
      if (*cursor != ',')
        break;
      cursor++;
    }
  }

  if ((index != 4U) || (*cursor != '\0'))
  {
    sFeedback.invalid_frame_count++;
    return;
  }

  for (index = 0U; index < 4U; index++)
    sFeedback.encoder_20ms[index] = encoder[index];
  sFeedback.received_tick_ms = now_ms;
  sFeedback.sequence++;
}

static void ConsumeByte(uint8_t byte, uint32_t now_ms)
{
  if (byte == '$')
  {
    sCollecting = 1U;
    sRxLength = 0U;
    sRxFrame[sRxLength++] = '$';
    return;
  }
  if (sCollecting == 0U)
    return;
  if (byte == '!')
  {
    ParseFrame(now_ms);
    sCollecting = 0U;
    sRxLength = 0U;
    return;
  }
  if (sRxLength >= (SONGJIA_FRAME_MAX_LENGTH - 1U))
  {
    sCollecting = 0U;
    sRxLength = 0U;
    sFeedback.invalid_frame_count++;
    return;
  }
  sRxFrame[sRxLength++] = (char)byte;
}

static void FormatCentiMps(char *output, uint32_t output_size, int16_t value)
{
  int32_t magnitude = value;
  int32_t fraction;
  if (magnitude < 0)
    magnitude = -magnitude;
  fraction = (magnitude % 100) * 10000L;
  (void)snprintf(output, output_size, "%s%ld.%06ld",
                 (value < 0) ? "-" : "", (long)(magnitude / 100),
                 (long)fraction);
}

void SongjiaMotor_Init(void)
{
  BspUart7_Init();
  sRxLength = 0U;
  sCollecting = 0U;
  memset(&sFeedback, 0, sizeof(sFeedback));
  memset(&sPolarity, 0, sizeof(sPolarity));
  sPolarityPending = 0U;
  sInitialized = 1U;
}

void SongjiaMotor_Process(uint32_t now_ms)
{
  uint8_t byte;
  uint8_t budget = SONGJIA_PARSE_BUDGET;
  while ((budget-- != 0U) && (BspUart7_ReadByte(&byte) != 0U))
    ConsumeByte(byte, now_ms);
  if (sPolarityPending != 0U && sPolarity.confirmed == 0U &&
      (uint32_t)(now_ms - sPolarity.last_attempt_ms) >= SONGJIA_POLARITY_RETRY_MS)
  {
    if (SongjiaMotor_SetEncoderPolarity(sPolarityTarget) != 0U)
    {
      sPolarity.last_attempt_ms = now_ms;
      sPolarity.attempt_count++;
    }
  }
}

void SongjiaMotor_StartEncoderPolarityConfig(uint8_t polarity, uint32_t now_ms)
{
  sPolarityTarget = (polarity > 1U) ? 1U : polarity;
  sPolarity.confirmed = 0U;
  sPolarity.value = 0U;
  sPolarity.last_attempt_ms = now_ms - SONGJIA_POLARITY_RETRY_MS;
  sPolarity.ack_tick_ms = 0U;
  sPolarity.attempt_count = 0U;
  sPolarity.invalid_ack_count = 0U;
  sPolarityPending = 1U;
  SongjiaMotor_Process(now_ms);
}

void SongjiaMotor_CancelEncoderPolarityConfig(void)
{
  sPolarityPending = 0U;
}

uint8_t SongjiaMotor_IsEncoderPolarityReady(void)
{ return (sPolarity.confirmed != 0U) && (sPolarity.value == sPolarityTarget); }

uint8_t SongjiaMotor_IsEncoderPolarityFailed(void)
{ return (sPolarityPending != 0U && sPolarity.confirmed == 0U) ? 1U : 0U; }

const SongjiaEncoderPolarityState_t *SongjiaMotor_GetEncoderPolarityState(void)
{ return &sPolarity; }

uint8_t SongjiaMotor_SetMotorType(uint8_t type)
{
  char command[24];
  if ((sInitialized == 0U) || (type > 2U)) return 0U;
  (void)snprintf(command, sizeof(command), "$MOTOR_4CH_SET:%u!", type);
  return SendText(command);
}

uint8_t SongjiaMotor_SetEncoderPolarity(uint8_t polarity)
{
  char command[48];
  if ((sInitialized == 0U) || (polarity > 1U)) return 0U;
  (void)snprintf(command, sizeof(command),
                 "$MOTOR_4CH_SET_ENCPDER_POLARITY:%u!", polarity);
  return SendText(command);
}

uint8_t SongjiaMotor_SetParameters(uint8_t wheel_diameter_cm,
                                   uint8_t encoder_ring_lines,
                                   uint8_t motor_gear_ratio)
{
  char command[40];
  if ((sInitialized == 0U) || (wheel_diameter_cm == 0U) ||
      (encoder_ring_lines == 0U) || (motor_gear_ratio == 0U))
    return 0U;
  (void)snprintf(command, sizeof(command), "$MOTOR_PARAM:%u,%u,%u!",
                 wheel_diameter_cm, encoder_ring_lines, motor_gear_ratio);
  return SendText(command);
}

uint8_t SongjiaMotor_RequestEncoder20ms(void)
{
  if (sInitialized == 0U) return 0U;
  return SendText("$MOTOR_4CH_READ:encoder_20ms!");
}

uint8_t SongjiaMotor_SetSpeedCentiMps(int16_t motor_a, int16_t motor_b,
                                      int16_t motor_c, int16_t motor_d)
{
  char a[10], b[10], c[10], d[10];
  char command[56];
  if (sInitialized == 0U) return 0U;
  if (!SongjiaMotor_IsEncoderPolarityReady() &&
      (motor_a != 0 || motor_b != 0 || motor_c != 0 || motor_d != 0)) return 0U;

  FormatCentiMps(a, sizeof(a), ClampI16(motor_a, -SONGJIA_SPEED_LIMIT_CENTI,
                                        SONGJIA_SPEED_LIMIT_CENTI));
  FormatCentiMps(b, sizeof(b), ClampI16(motor_b, -SONGJIA_SPEED_LIMIT_CENTI,
                                        SONGJIA_SPEED_LIMIT_CENTI));
  FormatCentiMps(c, sizeof(c), ClampI16(motor_c, -SONGJIA_SPEED_LIMIT_CENTI,
                                        SONGJIA_SPEED_LIMIT_CENTI));
  FormatCentiMps(d, sizeof(d), ClampI16(motor_d, -SONGJIA_SPEED_LIMIT_CENTI,
                                        SONGJIA_SPEED_LIMIT_CENTI));
  (void)snprintf(command, sizeof(command), "$Car:%s,%s,%s,%s!", a, b, c, d);
  return SendText(command);
}

uint8_t SongjiaMotor_SetPwmPermille(int16_t motor_a, int16_t motor_b,
                                    int16_t motor_c, int16_t motor_d)
{
  char command[48];
  int16_t percent_a;
  int16_t percent_b;
  int16_t percent_c;
  int16_t percent_d;
  if (sInitialized == 0U) return 0U;
  if (!SongjiaMotor_IsEncoderPolarityReady() &&
      (motor_a != 0 || motor_b != 0 || motor_c != 0 || motor_d != 0)) return 0U;

  motor_a = ClampI16(motor_a, -SONGJIA_PWM_LIMIT_PERMILLE,
                      SONGJIA_PWM_LIMIT_PERMILLE);
  motor_b = ClampI16(motor_b, -SONGJIA_PWM_LIMIT_PERMILLE,
                      SONGJIA_PWM_LIMIT_PERMILLE);
  motor_c = ClampI16(motor_c, -SONGJIA_PWM_LIMIT_PERMILLE,
                      SONGJIA_PWM_LIMIT_PERMILLE);
  motor_d = ClampI16(motor_d, -SONGJIA_PWM_LIMIT_PERMILLE,
                      SONGJIA_PWM_LIMIT_PERMILLE);
  percent_a = (int16_t)(motor_a / 10);
  percent_b = (int16_t)(motor_b / 10);
  percent_c = (int16_t)(motor_c / 10);
  percent_d = (int16_t)(motor_d / 10);
  (void)snprintf(command, sizeof(command), "$Car_Pwm:%d,%d,%d,%d!",
                 percent_a, percent_b, percent_c, percent_d);
  return SendText(command);
}

uint8_t SongjiaMotor_Stop(void)
{
  uint8_t pwm_ok;
  uint8_t speed_ok;
  if (sInitialized == 0U) return 1U;
  pwm_ok = SongjiaMotor_SetPwmPermille(0, 0, 0, 0);
  speed_ok = SongjiaMotor_SetSpeedCentiMps(0, 0, 0, 0);
  return ((pwm_ok != 0U) && (speed_ok != 0U)) ? 1U : 0U;
}

const SongjiaMotorFeedback_t *SongjiaMotor_GetFeedback(void)
{
  return &sFeedback;
}

uint8_t SongjiaMotor_IsFeedbackFresh(uint32_t now_ms, uint32_t timeout_ms)
{
  if ((sFeedback.sequence == 0U) ||
      ((uint32_t)(now_ms - sFeedback.received_tick_ms) > timeout_ms))
    return 0U;
  return 1U;
}
