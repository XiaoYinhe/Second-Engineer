#ifndef _USART_DMA__H
#define _USART_DMA__H

#include "board.h"
#define CONTROL_RX_BUF_NUM 10u      //���ط����ֽ�����2�����޷������ʹ洢
#define CONTROL_FRAME_LENGTH 5u      //���ط����ֽ���    
#define CONTROL_TX_BUF_NUM 9u      //���͸����ص������ֽ���


static void Rx_data_processing(uint8_t *comm_buf);


#endif