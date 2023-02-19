#pragma once

#include <stdint.h>

#define DI_NUM_MAX 16
#define DIO_VALUE_MASK 3
#define DIO_SPI_MSG_LEN 16
#define DIO_SPI_TIMEOUT 500
#define DO_1_TO_4_INDEX 11
#define DO_5_TO_6_INDEX 10

#define GET_ONE_BIT_VALUE(FIELD, BIT_NUM) ((FIELD >> BIT_NUM) & 1)
#define FLIP_BIT(FIELD, BIT_NUM) ((FIELD ^= 1 << BIT_NUM))
#define SET_DO_1_TO_4_ON(DO_NUM) ((3 << ((DO_NUM * 2))))
#define SET_DO_5_TO_6_ON(DO_NUM) ((3 << (((DO_NUM - 4) * 2))))
#define DO_1_TO_4_TX_BUFF_IND 11
#define DO_5_TO_6_TX_BUFF_IND 10

extern uint8_t DO_field;
extern uint16_t DI_field;

typedef enum {
  DO1_NUM,
  DO2_NUM,
  DO3_NUM,
  DO4_NUM,
  DO5_NUM,
  DO6_NUM,
  DO_NUM_MAX,
} DO_num;

void setDO(uint8_t* transmit_buffer);
void getDI(const uint8_t* receive_buffer);