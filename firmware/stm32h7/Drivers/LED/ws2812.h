#ifndef __WS2812_H__
#define __WS2812_H__
 
 
#include "main.h" 


#define WS2812_SPI_UNIT     hspi6
extern SPI_HandleTypeDef WS2812_SPI_UNIT;

/* 全局亮度上限：原 48，已按用户要求调整为约 60%（48*0.6 ≈ 29）。 */
#define WS2812_BRIGHTNESS_MAX  29U
 
void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b);
#endif
