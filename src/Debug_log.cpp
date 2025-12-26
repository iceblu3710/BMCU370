#include "Debug_log.h"
#include <inttypes.h>

#ifdef Debug_log_on
uint32_t stack[1000];
// mbed::Timer USB_debug_timer;
DMA_InitTypeDef Debug_log_DMA_InitStructure;

/**
 * @brief Initialize the Debug Serial Port (USART3).
 * 
 * Configures USART3 on PB10 (TX) and PB11 (RX) at 921600 baud.
 * Sets up DMA1 Channel 2 for non-blocking TX.
 * Enable RX interrupt for basic echoing.
 */
void Debug_log_init()
{

    GPIO_InitTypeDef GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // USART3 TX-->B.10  RX-->B.11
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = Debug_log_baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_Even;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART3, &USART_InitStructure);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Configure DMA1 channel 2 for USART3 TX
    Debug_log_DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DATAR;
    Debug_log_DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)0;
    Debug_log_DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    Debug_log_DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    Debug_log_DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    Debug_log_DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    Debug_log_DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    Debug_log_DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    Debug_log_DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    Debug_log_DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    Debug_log_DMA_InitStructure.DMA_BufferSize = 0;

    USART_Cmd(USART3, ENABLE);
}

uint64_t Debug_log_count64()
{
    return 0;
}

void Debug_log_time()
{
}

/**
 * @brief Write a string to the Debug Port.
 * 
 * @param data Null-terminated string.
 */
void Debug_log_write(const void *data)
{

    int i = strlen((const char *)data);
    Debug_log_write_num((const char *)data, i);
}

/**
 * @brief Write a buffer to the Debug Port using DMA.
 * 
 * Configures and enables DMA transfer for high-speed logging.
 * 
 * @param data Pointer to data buffer.
 * @param num Length of data.
 */
void Debug_log_write_num(const void *data, int num)
{
    DMA_DeInit(DMA1_Channel2);
    // Configure DMA1 channel 2 for USART3 TX
    Debug_log_DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)data;
    Debug_log_DMA_InitStructure.DMA_BufferSize = num;
    DMA_Init(DMA1_Channel2, &Debug_log_DMA_InitStructure);
    DMA_Cmd(DMA1_Channel2, ENABLE);
    // 使能USART3 DMA发送
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
}

void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        // uint8_t x =
        USART_ReceiveData(USART3);
        // USART_SendData(USART3, x);
    }
}

#endif