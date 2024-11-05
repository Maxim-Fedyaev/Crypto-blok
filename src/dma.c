#include "mik32_hal_dma.h"

DMA_InitTypeDef hdma;
DMA_ChannelHandleTypeDef hdma_ch0;
DMA_ChannelHandleTypeDef hdma_ch1;
DMA_ChannelHandleTypeDef hdma_ch2;
DMA_ChannelHandleTypeDef hdma_ch3;

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
}

static void DMA_CH1_Init(DMA_InitTypeDef *hdma)
{
    hdma_ch1.dma = hdma;
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
}

static void DMA_CH2_Init(DMA_InitTypeDef *hdma)
{
    hdma_ch2.dma = hdma;
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
}

static void DMA_CH3_Init(DMA_InitTypeDef *hdma)
{
    hdma_ch3.dma = hdma;
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
}

void DMA_Init(void)
{

    /* Настройки DMA */
    hdma.Instance = DMA_CONFIG;
    hdma.CurrentValue = DMA_CURRENT_VALUE_ENABLE;
    if (HAL_DMA_Init(&hdma) != HAL_OK)
    {
        xprintf("DMA_Init Error\n");
    }

    /* Инициализация канала */
    DMA_CH0_Init(&hdma);
    DMA_CH1_Init(&hdma);
    DMA_CH2_Init(&hdma);
    DMA_CH3_Init(&hdma);
}