#ifndef LD06_H
#define LD06_H

#include <stdint.h>

#define LD06_ANGLE_BINS 360U
#define LD06_RAW_SAMPLE_SIZE 24U
#define LD06_PROCESS_BYTE_BUDGET 256U

typedef struct
{
  uint16_t distance_mm[LD06_ANGLE_BINS];
  uint8_t confidence[LD06_ANGLE_BINS];
  uint16_t speed_dps;
  uint32_t raw_byte_count;
  uint32_t sync54_count;
  uint32_t header_count;
  uint32_t packet_count;
  uint32_t crc_error_count;
  uint32_t uart_restart_count;
  uint32_t uart_error_count;
  uint32_t last_uart_error;
  uint32_t last_packet_ms;
  uint32_t scan_sequence;
  uint32_t scan_complete_ms;
  uint16_t valid_point_count;
  uint8_t raw_sample[LD06_RAW_SAMPLE_SIZE];
  uint8_t raw_sample_count;
} LD06_Data_t;

uint8_t LD06_Init(void);
void LD06_Process(void);
void LD06_Stop(void);
const LD06_Data_t *LD06_GetData(void);
const LD06_Data_t *LD06_GetSnapshot(void);
uint8_t LD06_IsFresh(uint32_t now_ms, uint32_t timeout_ms);

#endif
