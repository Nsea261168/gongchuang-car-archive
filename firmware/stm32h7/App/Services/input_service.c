#include "input_service.h"

#include "adc_modlue.h"
#include "key_driver.h"

#define INPUT_ADC_CENTER_MAX  250U
#define INPUT_ADC_DOWN_MIN    500U
#define INPUT_ADC_DOWN_MAX   1000U
#define INPUT_ADC_UP_MIN     1200U
#define INPUT_ADC_UP_MAX     1750U
#define INPUT_ADC_LEFT_MIN   1900U
#define INPUT_ADC_LEFT_MAX   2450U
#define INPUT_ADC_RIGHT_MIN  2600U
#define INPUT_ADC_RIGHT_MAX  3150U

static volatile InputEvent_t sEvent;

static void Input_Up(void)     { sEvent = INPUT_EVENT_UP; }
static void Input_Down(void)   { sEvent = INPUT_EVENT_DOWN; }
static void Input_Left(void)   { sEvent = INPUT_EVENT_LEFT; }
static void Input_Right(void)  { sEvent = INPUT_EVENT_RIGHT; }
static void Input_Center(void) { sEvent = INPUT_EVENT_CENTER; }

static uint8_t InputService_InRange(uint16_t value, uint16_t minimum,
                                    uint16_t maximum)
{
  return ((value >= minimum) && (value <= maximum)) ? 1U : 0U;
}

static void InputService_ApplyAdcCalibration(uint16_t value)
{
  /* Active-low flags consumed by the unmodified QDcan key driver.
     Measured on DMstm32: idle=3588, down=736, up=1470,
     left=2188, right=2880, center=1. */
  key_mid = (value <= INPUT_ADC_CENTER_MAX) ? 0U : 1U;
  key_right = InputService_InRange(value, INPUT_ADC_DOWN_MIN,
                                   INPUT_ADC_DOWN_MAX) ? 0U : 1U;
  key_left = InputService_InRange(value, INPUT_ADC_UP_MIN,
                                  INPUT_ADC_UP_MAX) ? 0U : 1U;
  key_up = InputService_InRange(value, INPUT_ADC_LEFT_MIN,
                                INPUT_ADC_LEFT_MAX) ? 0U : 1U;
  key_down = InputService_InRange(value, INPUT_ADC_RIGHT_MIN,
                                  INPUT_ADC_RIGHT_MAX) ? 0U : 1U;
}

void InputService_Init(void)
{
  sEvent = INPUT_EVENT_NONE;
  key_driver_init();
  /* Verified QDcan board mapping. Do not infer directions from the
     key_up/down/left/right variable names inside the ADC driver. */
  key_attach(key_sw6, key_click_one, Input_Up);
  key_attach(key_sw5, key_click_one, Input_Down);
  key_attach(key_sw3, key_click_one, Input_Left);
  key_attach(key_sw4, key_click_one, Input_Right);
  key_attach(key_sw2, key_click_one, Input_Center);
}

void InputService_Poll(void)
{
  get_key_adc();
  InputService_ApplyAdcCalibration(adc_val[1]);
  key_process();
}

InputEvent_t InputService_TakeEvent(void)
{
  InputEvent_t event = sEvent;
  sEvent = INPUT_EVENT_NONE;
  return event;
}
