#include "DIO.h"
#include <stdint.h>

uint8_t DO_field = 0;
uint16_t DI_field = 0;

static void set_DO_value_tx_buff(DO_num do_num, uint8_t* spi_tx_buff)
{
    if (do_num >= 0 && do_num < 4) {
        spi_tx_buff[DO_1_TO_4_TX_BUFF_IND] += GET_ONE_BIT_VALUE(DO_field, do_num) ? SET_DO_1_TO_4_ON(do_num) : 0;
    } else if (do_num > 3 && do_num < 6) {
        spi_tx_buff[DO_5_TO_6_TX_BUFF_IND] += GET_ONE_BIT_VALUE(DO_field, do_num) ? SET_DO_5_TO_6_ON(do_num) : 0;
    } else {
        ;
    }
}

void setDO(uint8_t* spi_tx_buff)
{
    for (DO_num do_num = 0; do_num < DO_NUM_MAX; do_num++) {
        set_DO_value_tx_buff(do_num, spi_tx_buff);
    }
}

void getDI(const uint8_t* spi_rx_buffer)
{
    uint32_t di_word = 0;
    uint16_t DI_result = 0;
    di_word += spi_rx_buffer[3];
    di_word += spi_rx_buffer[2] << 8;
    di_word += spi_rx_buffer[1] << 16;
    di_word += spi_rx_buffer[0] << 24;
    di_word = ~di_word;

    for (int i = 0; i < DI_NUM_MAX; i++) {
        DI_result += ((di_word >> (i * 2 + 1)) & DIO_VALUE_MASK) ? (1 << i) : 0;
    }

    DI_field = DI_result;
}