#ifndef _USART_DMA__H
#define _USART_DMA__H

#include "board.h"
#define CONTROL_RX_BUF_NUM 10u      //副控发送字节数的2倍，无符号整型存储
#define CONTROL_FRAME_LENGTH 5u      //副控发送字节数    
#define CONTROL_TX_BUF_NUM 9u      //发送给副控的数据字节数


static void Rx_data_processing(uint8_t *comm_buf);


#endif