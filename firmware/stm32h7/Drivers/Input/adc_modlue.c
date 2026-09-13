#include "adc_modlue.h"
#include "main.h"

uint16_t key_value;
uint8_t key_up;
uint8_t key_down;
uint8_t key_mid;
uint8_t key_left;
uint8_t key_right;

/**
***********************************************************************
* @brief:      get_key_adc
* @param:			 void
* @retval:     void
* @details:    获取按键adc建值并转化为 0 1 信号
***********************************************************************
**/
void get_key_adc(void)
{
	key_value = (float)adc_val[1];
	
	if (key_value>=0 && key_value<200)
		key_mid = 0;
	else
		key_mid = 1;
	if (key_value>2200 && key_value<2500)
		key_up = 0;
	else
		key_up = 1;
	if (key_value>2800 && key_value<3500)
		key_down = 0;
	else
		key_down = 1;
	if (key_value>1500 && key_value<1800)
		key_left = 0;
	else
		key_left = 1;
	if (key_value>700 && key_value<1000)
		key_right = 0;
	else
		key_right = 1;
}







