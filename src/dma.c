#include "mik32_hal_dma.h"
#include "mik32_hal_usart.h"

static uint32_t CFGWriteBuffer[4] = {0}; 
static uint32_t ConfigStatusWriteBuffer = 0;

extern uint8_t array0[16];
extern uint8_t array1[16];
extern uint8_t array2[16];
extern uint8_t array3[16];
extern uint8_t array0_full;
extern uint8_t array1_full;
extern uint8_t array2_empty;
extern uint8_t array3_empty;

static void DMA_CH0_Init(void)
{
    DMA_CONFIG->CHANNELS[0].SRC = (uint32_t)&(UART_1->RXDATA);
    DMA_CONFIG->CHANNELS[0].DST = (uint32_t)&array0;
    DMA_CONFIG->CHANNELS[0].LEN = 15;

    CFGWriteBuffer[0] |= 0 // DMA_CH_CFG_ENABLE_M 
        | (DMA_CHANNEL_PRIORITY_VERY_HIGH << DMA_CH_CFG_PRIOR_S) 
        | (DMA_CHANNEL_MODE_PERIPHERY << DMA_CH_CFG_READ_MODE_S) 
        | (DMA_CHANNEL_INC_DISABLE << DMA_CH_CFG_READ_INCREMENT_S) 
        | (DMA_CHANNEL_SIZE_BYTE << DMA_CH_CFG_READ_SIZE_S) 
        | (4 << DMA_CH_CFG_READ_BURST_SIZE_S) 
        | (DMA_CHANNEL_USART_1_REQUEST << DMA_CH_CFG_READ_REQUEST_S) 
        | (DMA_CHANNEL_ACK_DISABLE << DMA_CH_CFG_READ_ACK_EN_S) 
        | (DMA_CHANNEL_MODE_MEMORY << DMA_CH_CFG_WRITE_MODE_S) 
        | (DMA_CHANNEL_INC_ENABLE << DMA_CH_CFG_WRITE_INCREMENT_S) 
        | (DMA_CHANNEL_SIZE_BYTE << DMA_CH_CFG_WRITE_SIZE_S) 
        | (4 << DMA_CH_CFG_WRITE_BURST_SIZE_S) 
        | (DMA_CHANNEL_USART_1_REQUEST << DMA_CH_CFG_WRITE_REQUEST_S) 
        | (DMA_CHANNEL_ACK_ENABLE << DMA_CH_CFG_WRITE_ACK_EN_S)
        | (DMA_IRQ_ENABLE << DMA_CH_CFG_IRQ_EN_S);

    DMA_CONFIG->CHANNELS[0].CFG = CFGWriteBuffer[0];
}

static void DMA_CH1_Init(void)
{
    DMA_CONFIG->CHANNELS[1].SRC = (uint32_t)&(UART_1->RXDATA);
    DMA_CONFIG->CHANNELS[1].DST = (uint32_t)&array1;
    DMA_CONFIG->CHANNELS[1].LEN = 15;

    CFGWriteBuffer[1] |= 0 // DMA_CH_CFG_ENABLE_M 
        | (DMA_CHANNEL_PRIORITY_VERY_HIGH << DMA_CH_CFG_PRIOR_S) 
        | (DMA_CHANNEL_MODE_PERIPHERY << DMA_CH_CFG_READ_MODE_S) 
        | (DMA_CHANNEL_INC_DISABLE << DMA_CH_CFG_READ_INCREMENT_S) 
        | (DMA_CHANNEL_SIZE_BYTE << DMA_CH_CFG_READ_SIZE_S) 
        | (4 << DMA_CH_CFG_READ_BURST_SIZE_S) 
        | (DMA_CHANNEL_USART_1_REQUEST << DMA_CH_CFG_READ_REQUEST_S) 
        | (DMA_CHANNEL_ACK_DISABLE << DMA_CH_CFG_READ_ACK_EN_S) 
        | (DMA_CHANNEL_MODE_MEMORY << DMA_CH_CFG_WRITE_MODE_S) 
        | (DMA_CHANNEL_INC_ENABLE << DMA_CH_CFG_WRITE_INCREMENT_S) 
        | (DMA_CHANNEL_SIZE_BYTE << DMA_CH_CFG_WRITE_SIZE_S) 
        | (4 << DMA_CH_CFG_WRITE_BURST_SIZE_S) 
        | (DMA_CHANNEL_USART_1_REQUEST << DMA_CH_CFG_WRITE_REQUEST_S) 
        | (DMA_CHANNEL_ACK_ENABLE << DMA_CH_CFG_WRITE_ACK_EN_S)
        | (DMA_IRQ_ENABLE << DMA_CH_CFG_IRQ_EN_S);

    DMA_CONFIG->CHANNELS[1].CFG = CFGWriteBuffer[1];
}

static void DMA_CH2_Init(void)
{
    DMA_CONFIG->CHANNELS[2].SRC = (uint32_t)&array2;
    DMA_CONFIG->CHANNELS[2].DST = (uint32_t)&(UART_0->TXDATA);
    DMA_CONFIG->CHANNELS[2].LEN = 15;

    CFGWriteBuffer[2] |= 0 // DMA_CH_CFG_ENABLE_M 
        | (DMA_CHANNEL_PRIORITY_VERY_HIGH << DMA_CH_CFG_PRIOR_S) 
        | (DMA_CHANNEL_MODE_MEMORY << DMA_CH_CFG_READ_MODE_S) 
        | (DMA_CHANNEL_INC_ENABLE << DMA_CH_CFG_READ_INCREMENT_S) 
        | (DMA_CHANNEL_SIZE_BYTE << DMA_CH_CFG_READ_SIZE_S) 
        | (4 << DMA_CH_CFG_READ_BURST_SIZE_S) 
        | (DMA_CHANNEL_USART_0_REQUEST << DMA_CH_CFG_READ_REQUEST_S) 
        | (DMA_CHANNEL_ACK_ENABLE << DMA_CH_CFG_READ_ACK_EN_S) 
        | (DMA_CHANNEL_MODE_PERIPHERY << DMA_CH_CFG_WRITE_MODE_S) 
        | (DMA_CHANNEL_INC_DISABLE << DMA_CH_CFG_WRITE_INCREMENT_S) 
        | (DMA_CHANNEL_SIZE_BYTE << DMA_CH_CFG_WRITE_SIZE_S) 
        | (4 << DMA_CH_CFG_WRITE_BURST_SIZE_S) 
        | (DMA_CHANNEL_USART_0_REQUEST << DMA_CH_CFG_WRITE_REQUEST_S) 
        | (DMA_CHANNEL_ACK_DISABLE << DMA_CH_CFG_WRITE_ACK_EN_S)
        | (DMA_IRQ_ENABLE << DMA_CH_CFG_IRQ_EN_S);

    DMA_CONFIG->CHANNELS[2].CFG = CFGWriteBuffer[2];
}

static void DMA_CH3_Init(void)
{
    DMA_CONFIG->CHANNELS[3].SRC = (uint32_t)&array3;
    DMA_CONFIG->CHANNELS[3].DST = (uint32_t)&(UART_0->TXDATA);
    DMA_CONFIG->CHANNELS[3].LEN = 15;

    CFGWriteBuffer[3] |= 0 // DMA_CH_CFG_ENABLE_M 
        | (DMA_CHANNEL_PRIORITY_VERY_HIGH << DMA_CH_CFG_PRIOR_S) 
        | (DMA_CHANNEL_MODE_MEMORY << DMA_CH_CFG_READ_MODE_S) 
        | (DMA_CHANNEL_INC_ENABLE << DMA_CH_CFG_READ_INCREMENT_S) 
        | (DMA_CHANNEL_SIZE_BYTE << DMA_CH_CFG_READ_SIZE_S) 
        | (4 << DMA_CH_CFG_READ_BURST_SIZE_S) 
        | (DMA_CHANNEL_USART_0_REQUEST << DMA_CH_CFG_READ_REQUEST_S) 
        | (DMA_CHANNEL_ACK_ENABLE << DMA_CH_CFG_READ_ACK_EN_S) 
        | (DMA_CHANNEL_MODE_PERIPHERY << DMA_CH_CFG_WRITE_MODE_S) 
        | (DMA_CHANNEL_INC_DISABLE << DMA_CH_CFG_WRITE_INCREMENT_S) 
        | (DMA_CHANNEL_SIZE_BYTE << DMA_CH_CFG_WRITE_SIZE_S) 
        | (4 << DMA_CH_CFG_WRITE_BURST_SIZE_S) 
        | (DMA_CHANNEL_USART_0_REQUEST << DMA_CH_CFG_WRITE_REQUEST_S) 
        | (DMA_CHANNEL_ACK_DISABLE << DMA_CH_CFG_WRITE_ACK_EN_S)
        | (DMA_IRQ_ENABLE << DMA_CH_CFG_IRQ_EN_S);

    DMA_CONFIG->CHANNELS[3].CFG = CFGWriteBuffer[3];
}

void DMA_ClearLocalIrq(void)
{
    ConfigStatusWriteBuffer &= ~(DMA_CONFIG_CLEAR_LOCAL_IRQ_M | DMA_CONFIG_CLEAR_GLOBAL_IRQ_M | DMA_CONFIG_CLEAR_ERROR_IRQ_M);
    ConfigStatusWriteBuffer |= DMA_CONFIG_CLEAR_LOCAL_IRQ_M;
    DMA_CONFIG->CONFIG_STATUS = ConfigStatusWriteBuffer;
    ConfigStatusWriteBuffer &= ~(DMA_CONFIG_CLEAR_LOCAL_IRQ_M | DMA_CONFIG_CLEAR_GLOBAL_IRQ_M | DMA_CONFIG_CLEAR_ERROR_IRQ_M);
}

void DMA_Init(void)
{
    __HAL_PCC_DMA_CLK_ENABLE();

    ConfigStatusWriteBuffer = 0;

    DMA_ClearLocalIrq();

    ConfigStatusWriteBuffer &= ~DMA_CONFIG_CURRENT_VALUE_M;
    ConfigStatusWriteBuffer |= DMA_CURRENT_VALUE_ENABLE <<  DMA_CONFIG_CURRENT_VALUE_S;
    DMA_CONFIG->CONFIG_STATUS = ConfigStatusWriteBuffer;

    /* Инициализация канала */
    // DMA_CH0_Init();
    // DMA_CH1_Init();
    // DMA_CH2_Init();
    DMA_CH3_Init();
}

void DMA_Channel_Start(uint8_t ChannelIndex)
{
    CFGWriteBuffer[ChannelIndex] |= DMA_CH_CFG_ENABLE_M;
    DMA_CONFIG->CHANNELS[ChannelIndex].CFG = CFGWriteBuffer[ChannelIndex];
}

void DMA_Channel_Stop(uint8_t ChannelIndex)
{
    CFGWriteBuffer[ChannelIndex] &= ~DMA_CH_CFG_ENABLE_M;
    DMA_CONFIG->CHANNELS[ChannelIndex].CFG = CFGWriteBuffer[ChannelIndex];
}

int DMA_GetChannelIrq(uint8_t ChannelIndex)
{
    uint32_t ChannelIrq = (DMA_CONFIG->CONFIG_STATUS) & ((1 << ChannelIndex) << DMA_STATUS_CHANNEL_IRQ_S);
    ChannelIrq = ( ChannelIrq >> DMA_STATUS_CHANNEL_IRQ_S ) >> ChannelIndex;
    return ChannelIrq;
}

void DMA_IRQHandler(void)
{
    if (DMA_GetChannelIrq(0))
    {
        DMA_Channel_Stop(0);
        DMA_Channel_Start(1);
        array1_full = 1;
    }
    if (DMA_GetChannelIrq(1))
    {
        DMA_Channel_Stop(1);
        DMA_Channel_Start(0);
        array0_full = 1;
    }
    if (DMA_GetChannelIrq(2))
    {
        DMA_Channel_Stop(2);
        array3_empty = 1;
    }
    if (DMA_GetChannelIrq(3))
    {
        DMA_Channel_Stop(3);
        array2_empty = 1;
    }
    DMA_ClearLocalIrq();
}




