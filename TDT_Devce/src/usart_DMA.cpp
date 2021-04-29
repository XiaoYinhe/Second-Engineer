/***************
@brief 主副控通信，串口3

**************/
#include "ore_task.h"
#include "usart_DMA.h"
#include "crc.h"
uint8_t CONTROL_rx_buf[2][CONTROL_RX_BUF_NUM];    //主控板发送的数据，二维数组
static uint8_t CONTROL_tx_buf[CONTROL_TX_BUF_NUM];     //向外设传送数据的地址
static uint8_t CONTROL_tx_tran_buf[CONTROL_TX_BUF_NUM];   //向外设传输数据的中转站   CONTROL_tx_buf到CONTROL_tx_tran_buf到USART1->DR
 
 void usart_DMA_init()
{
	//USART3-TX  PB10 
	//USART3-RX  PB11
	/* -------------- Enable Module Clock Source ----------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3, DISABLE);
    
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3); 
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
	
	/* -------------- Configure GPIO ---------------------------------------*/
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 |GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	USART_DeInit(USART3);										
	
	USART_InitStructure.USART_BaudRate = 1000000;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART3, &USART_InitStructure);
	
	USART_DMACmd(USART3, USART_DMAReq_Rx|USART_DMAReq_Tx, ENABLE);
	
	USART_ClearFlag(USART3, USART_FLAG_IDLE);
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
	
	USART_Cmd(USART3, ENABLE);
	
	/* -------------- Configure NVIC ---------------------------------------*/
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}	
	//DMA1 stream1 ch4 RX  or (DMA1 stream3 ch4 TX)    !!!!!!! P205 of the datasheet
	/* -------------- Configure RX DMA -----------------------------------------*/
{
	DMA_InitTypeDef DMA_InitStructure;
	
	DMA_DeInit(DMA1_Stream1);
	
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART3->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)CONTROL_rx_buf[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = CONTROL_RX_BUF_NUM;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream1, &DMA_InitStructure);
	
	DMA_DoubleBufferModeConfig(DMA1_Stream1, (uint32_t)CONTROL_rx_buf[1], DMA_Memory_0);
	DMA_DoubleBufferModeCmd(DMA1_Stream1, ENABLE);
	DMA_Cmd(DMA1_Stream1, DISABLE); //Add a disable
	DMA_Cmd(DMA1_Stream1, ENABLE);
}
	/* -------------- Configure TX DMA -----------------------------------------*/
{	
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	    
	DMA_DeInit(DMA1_Stream3);
	
	DMA_DeInit(DMA1_Stream3);
	DMA_InitStructure.DMA_Channel= DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)CONTROL_tx_buf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = CONTROL_TX_BUF_NUM;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream3,&DMA_InitStructure);
	
	DMA_ITConfig(DMA1_Stream3,DMA_IT_TC,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	DMA_Cmd(DMA1_Stream3,DISABLE);
}	
	
}

  int Flag_Tx_Gsm_Busy;

void USART3_IRQHandler(void)
{
	if (USART_GetITStatus(USART3,  USART_IT_TC) != RESET)
    {
        /* 关闭发送完成中断  */ 
        USART_ITConfig(USART3,USART_IT_TC,DISABLE);  
        Flag_Tx_Gsm_Busy = 0; 
        USART_ClearITPendingBit(USART3, USART_IT_TC);           

    }
	else if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        USART_ReceiveData(USART3);
    }   
    
    else if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
    {
        static uint16_t this_time_rx_len = 0;
        USART_ReceiveData(USART3);

        if(DMA_GetCurrentMemoryTarget(DMA1_Stream1) == 0)
        {
            //重新设置DMA
            DMA_Cmd(DMA1_Stream1, DISABLE);
            this_time_rx_len = CONTROL_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA1_Stream1);
            DMA_SetCurrDataCounter(DMA1_Stream1, CONTROL_RX_BUF_NUM);
            DMA1_Stream1->CR |= DMA_SxCR_CT;
            //清DMA中断标志
            DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);
            DMA_Cmd(DMA1_Stream1, ENABLE);
            if(this_time_rx_len == CONTROL_FRAME_LENGTH)
            {
                //处理遥控器数据
                Rx_data_processing(CONTROL_rx_buf[0]);
            }
        }
        else
        {
            //重新设置DMA
            DMA_Cmd(DMA1_Stream1, DISABLE);
            this_time_rx_len = CONTROL_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA1_Stream1);
            DMA_SetCurrDataCounter(DMA1_Stream1, CONTROL_RX_BUF_NUM);
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            //清DMA中断标志
            DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);
            DMA_Cmd(DMA1_Stream1, ENABLE);
            if(this_time_rx_len == CONTROL_FRAME_LENGTH)
            {
                //处理遥控器数据
                Rx_data_processing(CONTROL_rx_buf[1]);
            }

        }
    }


}


static void Rx_data_processing(uint8_t *comm_buf)
{
	if(comm_buf[0]==0xFF &&  (Verify_CRC16_Check_Sum(comm_buf,CONTROL_FRAME_LENGTH)))
    {    
		//commLostCnt=200;//串口丢失标志位
		          
			flagCmd.climb	= (u8)(comm_buf[1]>>4);
			flagCmd.pumpTurn = (u8)(comm_buf[2]);
			flagCmd.clip = (u8)(comm_buf[1]>>2);
			flagCmd.turnTable = ((u8)(comm_buf[1]))&0x3;
			flagCmd.pump = (u8)(comm_buf[3]>>4);
//			Tran_Singal=(u8)(comm_buf[2]>>7);
//			upTranResetState=(u8)(comm_buf[2]&0x7f);
		
    }

}




void Communicate_SendChar(void)  
{
//	if(ABS(Claw[0].Feedback.totalAngle)<800)//爪子是否收回标志位判断
//		clawFlag=0;//爪子已收回
//	else if(ABS(Claw[0].Feedback.totalAngle)>3000)
//		clawFlag=1;//爪子已伸出
//	else
//		clawFlag=2;//爪子正在伸出
//	
//	if(ABS(Tran[0].Feedback.totalAngle-(tran_position[Tran_Pos-1]+ tranFeed + handleTranFeed))<300 && tranResetFlag==0 && upTranResetState==0){//平移架位置是否有到标志位
//		Tran_Singal=1;
//	}
//	else
//		Tran_Singal=0;
//	
	
	
	CONTROL_tx_tran_buf[0]=0xff;
    CONTROL_tx_tran_buf[1]=(u8)STA.climb_STA<<4|STA.pumpTurn_STA;
    CONTROL_tx_tran_buf[2]=(u8)STA.pump_STA<<7;
	
	Append_CRC16_Check_Sum(CONTROL_tx_tran_buf,CONTROL_TX_BUF_NUM);


    //等待空闲  
    while (Flag_Tx_Gsm_Busy);  
    Flag_Tx_Gsm_Busy = 1;

    //复制数据
    memcpy(CONTROL_tx_buf,CONTROL_tx_tran_buf,CONTROL_TX_BUF_NUM);  
    //设置传输数据长度
    DMA_SetCurrDataCounter(DMA1_Stream3,CONTROL_TX_BUF_NUM);  
    //打开DMA,开始发送
    DMA_Cmd(DMA1_Stream3,ENABLE); 
    delayUs(100);
}





void DMA1_Stream3_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream3,DMA_IT_TCIF3) != RESET)   
    {  
        /* 清除标志位 */
        DMA_ClearFlag(DMA1_Stream3,DMA_IT_TCIF3);  
        /* 关闭DMA */
        DMA_Cmd(DMA1_Stream3,DISABLE);
        /* 打开发送完成中断,确保最后一个字节发送成功 */
        USART_ITConfig(USART3,USART_IT_TC,ENABLE);  
    }  
}