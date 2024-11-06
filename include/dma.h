
void DMA_Init(void);
void DMA_IRQHandler(void);
void DMA_Channel_Start(uint8_t ChannelIndex);
void DMA_Channel_Stop(uint8_t ChannelIndex);
int DMA_GetChannelIrq(uint8_t ChannelIndex);
void DMA_ClearLocalIrq(void);