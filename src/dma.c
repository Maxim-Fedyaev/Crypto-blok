#include "mik32_hal_dma.h"
#include "mik32_hal_usart.h"

extern uint8_t array0[16];
extern uint8_t array1[16];
extern uint8_t array2[16];
extern uint8_t array3[16];
extern uint8_t array0_full;
extern uint8_t array1_full;
extern uint8_t array2_empty;
extern uint8_t array3_empty;

DMA_InitTypeDef hdma = {0};
DMA_ChannelHandleTypeDef hdma_ch0 = {0};
DMA_ChannelHandleTypeDef hdma_ch1 = {0};
DMA_ChannelHandleTypeDef hdma_ch2 = {0};
DMA_ChannelHandleTypeDef hdma_ch3 = {0};
extern USART_HandleTypeDef husart0;
extern USART_HandleTypeDef husart1;


static void DMA_CH0_Init(DMA_InitTypeDef *hdma)
{
    hdma_ch0.dma = hdma;

    /* Настройки канала */
    hdma_ch0.ChannelInit.Channel = DMA_CHANNEL_0;
    hdma_ch0.ChannelInit.Priority = DMA_CHANNEL_PRIORITY_VERY_HIGH;

    hdma_ch0.ChannelInit.ReadMode = DMA_CHANNEL_MODE_PERIPHERY;
    hdma_ch0.ChannelInit.ReadInc = DMA_CHANNEL_INC_DISABLE;
    hdma_ch0.ChannelInit.ReadSize = DMA_CHANNEL_SIZE_BYTE; /* data_len должно быть кратно read_size */
    hdma_ch0.ChannelInit.ReadBurstSize = 0;                /* read_burst_size должно быть кратно read_size */
    hdma_ch0.ChannelInit.ReadRequest = DMA_CHANNEL_USART_1_REQUEST;
    hdma_ch0.ChannelInit.ReadAck = DMA_CHANNEL_ACK_DISABLE;

    hdma_ch0.ChannelInit.WriteMode = DMA_CHANNEL_MODE_MEMORY;
    hdma_ch0.ChannelInit.WriteInc = DMA_CHANNEL_INC_ENABLE;
    hdma_ch0.ChannelInit.WriteSize = DMA_CHANNEL_SIZE_BYTE; /* data_len должно быть кратно write_size */
    hdma_ch0.ChannelInit.WriteBurstSize = 0;                /* write_burst_size должно быть кратно read_size */
    hdma_ch0.ChannelInit.WriteRequest = DMA_CHANNEL_USART_1_REQUEST;
    hdma_ch0.ChannelInit.WriteAck = DMA_CHANNEL_ACK_ENABLE;

    //HAL_DMA_LocalIRQEnable(&hdma_ch0, DMA_IRQ_ENABLE);
}

static void DMA_CH1_Init(DMA_InitTypeDef *hdma)
{
    hdma_ch0.dma = hdma;

    /* Настройки канала */
    hdma_ch1.ChannelInit.Channel = DMA_CHANNEL_1;
    hdma_ch1.ChannelInit.Priority = DMA_CHANNEL_PRIORITY_VERY_HIGH;

    hdma_ch1.ChannelInit.ReadMode = DMA_CHANNEL_MODE_PERIPHERY;
    hdma_ch1.ChannelInit.ReadInc = DMA_CHANNEL_INC_DISABLE;
    hdma_ch1.ChannelInit.ReadSize = DMA_CHANNEL_SIZE_BYTE; /* data_len должно быть кратно read_size */
    hdma_ch1.ChannelInit.ReadBurstSize = 0;                /* read_burst_size должно быть кратно read_size */
    hdma_ch1.ChannelInit.ReadRequest = DMA_CHANNEL_USART_1_REQUEST;
    hdma_ch1.ChannelInit.ReadAck = DMA_CHANNEL_ACK_DISABLE;

    hdma_ch1.ChannelInit.WriteMode = DMA_CHANNEL_MODE_MEMORY;
    hdma_ch1.ChannelInit.WriteInc = DMA_CHANNEL_INC_ENABLE;
    hdma_ch1.ChannelInit.WriteSize = DMA_CHANNEL_SIZE_BYTE; /* data_len должно быть кратно write_size */
    hdma_ch1.ChannelInit.WriteBurstSize = 0;                /* write_burst_size должно быть кратно read_size */
    hdma_ch1.ChannelInit.WriteRequest = DMA_CHANNEL_USART_1_REQUEST;
    hdma_ch1.ChannelInit.WriteAck = DMA_CHANNEL_ACK_ENABLE;

    //HAL_DMA_LocalIRQEnable(&hdma_ch1, DMA_IRQ_ENABLE);
}

static void DMA_CH2_Init(DMA_InitTypeDef *hdma)
{
    hdma_ch0.dma = hdma;

    /* Настройки канала */
    hdma_ch2.ChannelInit.Channel = DMA_CHANNEL_2;
    hdma_ch2.ChannelInit.Priority = DMA_CHANNEL_PRIORITY_VERY_HIGH;

    hdma_ch2.ChannelInit.ReadMode = DMA_CHANNEL_MODE_MEMORY;
    hdma_ch2.ChannelInit.ReadInc = DMA_CHANNEL_INC_ENABLE;
    hdma_ch2.ChannelInit.ReadSize = DMA_CHANNEL_SIZE_BYTE; /* data_len должно быть кратно read_size */
    hdma_ch2.ChannelInit.ReadBurstSize = 0;                /* read_burst_size должно быть кратно read_size */
    hdma_ch2.ChannelInit.ReadRequest = DMA_CHANNEL_USART_0_REQUEST;
    hdma_ch2.ChannelInit.ReadAck = DMA_CHANNEL_ACK_ENABLE;

    hdma_ch2.ChannelInit.WriteMode = DMA_CHANNEL_MODE_PERIPHERY;
    hdma_ch2.ChannelInit.WriteInc = DMA_CHANNEL_INC_DISABLE;
    hdma_ch2.ChannelInit.WriteSize = DMA_CHANNEL_SIZE_BYTE; /* data_len должно быть кратно write_size */
    hdma_ch2.ChannelInit.WriteBurstSize = 0;                /* write_burst_size должно быть кратно read_size */
    hdma_ch2.ChannelInit.WriteRequest = DMA_CHANNEL_USART_0_REQUEST;
    hdma_ch2.ChannelInit.WriteAck = DMA_CHANNEL_ACK_DISABLE;

    //HAL_DMA_LocalIRQEnable(&hdma_ch2, DMA_IRQ_ENABLE);
}

static void DMA_CH3_Init(DMA_InitTypeDef *hdma)
{
    hdma_ch0.dma = hdma;

    /* Настройки канала */
    hdma_ch3.ChannelInit.Channel = DMA_CHANNEL_3;
    hdma_ch3.ChannelInit.Priority = DMA_CHANNEL_PRIORITY_VERY_HIGH;

    hdma_ch3.ChannelInit.ReadMode = DMA_CHANNEL_MODE_MEMORY;
    hdma_ch3.ChannelInit.ReadInc = DMA_CHANNEL_INC_ENABLE;
    hdma_ch3.ChannelInit.ReadSize = DMA_CHANNEL_SIZE_BYTE; /* data_len должно быть кратно read_size */
    hdma_ch3.ChannelInit.ReadBurstSize = 0;                /* read_burst_size должно быть кратно read_size */
    hdma_ch3.ChannelInit.ReadRequest = DMA_CHANNEL_USART_0_REQUEST;
    hdma_ch3.ChannelInit.ReadAck = DMA_CHANNEL_ACK_ENABLE;

    hdma_ch3.ChannelInit.WriteMode = DMA_CHANNEL_MODE_PERIPHERY;
    hdma_ch3.ChannelInit.WriteInc = DMA_CHANNEL_INC_DISABLE;
    hdma_ch3.ChannelInit.WriteSize = DMA_CHANNEL_SIZE_BYTE; /* data_len должно быть кратно write_size */
    hdma_ch3.ChannelInit.WriteBurstSize = 0;                /* write_burst_size должно быть кратно read_size */
    hdma_ch3.ChannelInit.WriteRequest = DMA_CHANNEL_USART_0_REQUEST;
    hdma_ch3.ChannelInit.WriteAck = DMA_CHANNEL_ACK_DISABLE;

    //HAL_DMA_LocalIRQEnable(&hdma_ch3, DMA_IRQ_ENABLE);
}

void DMA_Init(void)
{
    /* Настройки DMA */
    hdma.Instance = DMA_CONFIG;
    hdma.CurrentValue = DMA_CURRENT_VALUE_ENABLE;

    HAL_DMA_Init(&hdma);

    // /* Настройка глобального прерывания DMA */
    // HAL_DMA_GlobalIRQEnable(&hdma, DMA_IRQ_ENABLE);
    // /* Настройка прерывания DMA при возникновении ошибки */
    // HAL_DMA_ErrorIRQEnable(&hdma, DMA_IRQ_ENABLE);

    /* Инициализация канала */
    DMA_CH0_Init(&hdma);
    DMA_CH1_Init(&hdma);
    DMA_CH2_Init(&hdma);
    DMA_CH3_Init(&hdma);
}

void DMA_IRQHandler(void)
{
    if (HAL_DMA_GetChannelIrq(&hdma_ch0))
    {
        HAL_DMA_ChannelDisable(&hdma_ch0);
        HAL_DMA_Start(&hdma_ch1, (void *)&husart1.Instance->RXDATA, (void *)&array1, 15);
        array1_full = 1;
    }
    if (HAL_DMA_GetChannelIrq(&hdma_ch1))
    {
        HAL_DMA_ChannelDisable(&hdma_ch1);
        HAL_DMA_Start(&hdma_ch0, (void *)&husart1.Instance->RXDATA, (void *)&array0, 15);
        array0_full = 1;
    }
    if (HAL_DMA_GetChannelIrq(&hdma_ch2))
    {
        HAL_DMA_ChannelDisable(&hdma_ch2);
        array3_empty = 1;
    }
    if (HAL_DMA_GetChannelIrq(&hdma_ch3))
    {
        HAL_DMA_ChannelDisable(&hdma_ch3);
        array2_empty = 1;
    }
    HAL_DMA_ClearLocalIrq(&hdma);
}




