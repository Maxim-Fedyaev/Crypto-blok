#include "mik32_hal_pcc.h"
#include "mik32_hal_gpio.h"
#include "csr.h"
#include "mik32_hal_crypto.h"
#include "mik32_hal_usart.h"
#include "mik32_hal_irq.h"
#include "mik32_hal_timer32.h"
#include "dma.h"
#include "main.h"
#include "mik32_hal_dma.h"

#define BLOCK_SIZE (CRYPTO_BLOCK_KUZNECHIK*4)

#define PAD_MODE_1 0x01
#define PAD_MODE_2 0x02
#define PAD_MODE_3 0x03

#define CODER   0
#define DECODER 1

Crypto_HandleTypeDef hcrypto;
USART_HandleTypeDef husart0 = {0};
USART_HandleTypeDef husart1 = {0};
TIMER32_HandleTypeDef htimer32_0;
// extern DMA_ChannelHandleTypeDef hdma_ch0;
// extern DMA_ChannelHandleTypeDef hdma_ch1;
// extern DMA_ChannelHandleTypeDef hdma_ch2;
// extern DMA_ChannelHandleTypeDef hdma_ch3;


static  eMBSndState eSndState = STATE_TX_IDLE;
static  eMBRcvState eRcvState = STATE_RX_IDLE;
static  eMBEventType xNeedPoll = EV_READY;

uint8_t  ucRTUBuf[256];

static  uint8_t *pucSndBufferCur;
static  uint16_t usSndBufferCount = 0;
static  uint16_t usRcvBufferPos = 0;

//uint32_t init_vector[IV_LENGTH_KUZNECHIK_CTR] = {0x12345678, 0x90ABCEF0};
uint32_t init_vector[IV_LENGTH_KUZNECHIK_CBC] = {0x12341234, 0x11114444, 0xABCDABCD, 0xAAAABBBB};

//uint32_t init_vector[IV_LENGTH_MAGMA_CTR] = {0x12345678};
//uint32_t init_vector[IV_LENGTH_MAGMA_CBC] = {0x12341234, 0x11114444};

uint32_t crypto_key[CRYPTO_KEY_KUZNECHIK] = {0x8899aabb, 0xccddeeff, 0x00112233, 0x44556677, 0xfedcba98, 0x76543210, 0x01234567, 0x89abcdef};                                                              

uint32_t key_length = sizeof(crypto_key)/sizeof(*crypto_key);

void SystemClock_Config(void);
void GPIO_Init(void);
void Crypto_Init(void);
void USART_Init();
void UART1_IRQHandler(void);
void UART0_IRQHandler(void);
void Decoder (void);
void Coder (void);
void UART_IRQHandler(void);
void vMBPortSerialEnable(uint8_t xRxEnable, uint8_t xTxEnable);
void Timer32_0_IRQHandler(void);
void TimersEnable(void);
void TimersDisable(void);
void TimersInit(uint16_t usTim1Timerout50us);
void GPIOControlRS485Set ( RS485 State );
void GPIOControlRS485Init ( void );

extern unsigned long __TEXT_START__; // это "метка" для обработчика прерываний
volatile void trap_handler(void)     // сам обработчик всех прерываний
{
    if(HAL_EPIC_GetStatus() & HAL_EPIC_TIMER32_0_MASK)
    {
        Timer32_0_IRQHandler();
        HAL_EPIC_Clear(HAL_EPIC_TIMER32_1_MASK);
    }
    if(HAL_EPIC_GetStatus() & HAL_EPIC_UART_0_MASK)
    {
        UART0_IRQHandler();
        HAL_EPIC_Clear(HAL_EPIC_UART_0_MASK);
    }
    if(HAL_EPIC_GetStatus() & HAL_EPIC_UART_1_MASK)
    {
        UART1_IRQHandler();
        HAL_EPIC_Clear(HAL_EPIC_UART_1_MASK);
    }
    /* Сброс прерываний */
    HAL_EPIC_Clear(0xFFFFFFFF);
}

uint8_t get_size_pad(uint64_t size, uint8_t pad_mode)
{
    if (pad_mode == PAD_MODE_1)
        // Если дополнение для процедуры 1 не нужно, возвращаем 0
        if ((BLOCK_SIZE - (size % BLOCK_SIZE)) == BLOCK_SIZE)
            return 0;

    if (pad_mode == PAD_MODE_3)
        // Если дополнение для процедуры 3 не нужно, возвращаем 0
        if ((BLOCK_SIZE - (size % BLOCK_SIZE)) == BLOCK_SIZE)
            return 0;
    // Возвращаем длину дополнения
    return BLOCK_SIZE - (size % BLOCK_SIZE);
}

void set_padding(uint8_t *out_buf, uint8_t *in_buf, uint8_t pad_size, uint64_t size, uint8_t pad_mode)
{
    for (uint64_t i = 0; i < size; i++)
    {
        out_buf[i] = in_buf[i];
    }
    
    if (pad_size > 0)
    {
        if (pad_mode == PAD_MODE_1) // Для процедуры 1
        {
            uint64_t i;
            for (i = size; i < size + pad_size; i++)
                // Пишем все нули
                out_buf[i] = 0x00;
        }
        if (pad_mode == PAD_MODE_2) // Для процедуры 2
        {
            // Пишем единичку в первый бит
            out_buf[size] = 0x80;
            uint64_t i;
            for (i = size + 1; i < size + pad_size; i++)
                // Далее заполняем все остальное нулями
                out_buf[i] = 0x00;
        }
        if (pad_mode == PAD_MODE_3) // Для процедуры 3
        {
            // Пишем единичку в первый бит
            out_buf[size] = 0x80;
            uint64_t i;
            for (i = size + 1; i < size + pad_size; i++)
                // Далее заполняем все остальное нулями
                out_buf[i] = 0x00;
        }
    }
}

int main()
{
    write_csr(mtvec, &__TEXT_START__); // операция, настраивающая вектор прерываний
    
    __HAL_PCC_EPIC_CLK_ENABLE();
    HAL_EPIC_MaskEdgeSet(HAL_EPIC_UART_0_MASK | HAL_EPIC_TIMER32_0_MASK | HAL_EPIC_UART_1_MASK); 
    HAL_IRQ_EnableInterrupts();

    GPIOControlRS485Init();
    #if DECODER
        GPIOControlRS485Set ( RS485_RX ); 
    #endif
    #if CODER
        GPIOControlRS485Set ( RS485_TX ); 
    #endif
    SystemClock_Config();
    Crypto_Init();
    USART_Init();
    TimersInit(35);

    vMBPortSerialEnable(1, 0);

    /* Задать режим шифрования */
    HAL_Crypto_SetCipherMode(&hcrypto, CRYPTO_CIPHER_MODE_ECB);
    /* Установка вектора инициализации */  
    //HAL_Crypto_SetIV(&hcrypto, init_vector, sizeof(init_vector)/sizeof(*init_vector)); 
    /* Установка ключа */
    HAL_Crypto_SetKey(&hcrypto, crypto_key);



    //HAL_DMA_Start(&hdma_ch0, (void*)&UART_1->RXDATA, (void*)array1, 15);
    //HAL_DMA_Start(&hdma_ch1, (void*)&UART_1->RXDATA, (void*)array2, 15);
    //HAL_DMA_Start(&hdma_ch0, (void*)array1, (void*)&UART_0->RXDATA, 15);
    //HAL_DMA_Start(&hdma_ch1, (void*)array2, (void*)&UART_0->RXDATA, 15);

    while (1)
    {

        #if CODER
            Coder();
        #endif
        #if DECODER
            Decoder();
        #endif

    }
}

void Decoder (void)
{
    uint8_t cipher_text_dec_length;
    uint32_t cipher_text_dec[64];
    uint8_t expect_cipher_text_length; 
    uint32_t expect_cipher_text[64];

    switch ( xNeedPoll )
    {
    case EV_READY:

        break;
    case EV_FRAME_RECEIVED:
        cipher_text_dec_length = usRcvBufferPos/4;
        expect_cipher_text_length = cipher_text_dec_length;
        for(int i = 0; i < cipher_text_dec_length; i++)
            cipher_text_dec[i] = (ucRTUBuf[4*i] << 24) + (ucRTUBuf[4*i+1] << 16) + (ucRTUBuf[4*i+2] << 8) + ucRTUBuf[4*i+3];
        HAL_Crypto_Decode(&hcrypto, cipher_text_dec, expect_cipher_text, expect_cipher_text_length);
        if(expect_cipher_text[0] == 0)  
        {
            HAL_USART_Print(UART_1, "Ошибка передачи", 50);
            return;
        }

        pucSndBufferCur = ( uint8_t * ) ucRTUBuf;
        usSndBufferCount = 0;
        for (uint32_t i=0; i < expect_cipher_text_length; i++)
        {
            ucRTUBuf[4*i] = (uint8_t)(expect_cipher_text[i] >> 24);
            ucRTUBuf[4*i + 1] = (uint8_t)(expect_cipher_text[i] >> 16);
            ucRTUBuf[4*i + 2] = (uint8_t)(expect_cipher_text[i] >> 8);
            ucRTUBuf[4*i + 3] = (uint8_t)(expect_cipher_text[i]);
            usSndBufferCount += 4;
        }
        while ((ucRTUBuf[usSndBufferCount - 1] == 0x80 && ucRTUBuf[usSndBufferCount] == 0) 
            || (ucRTUBuf[usSndBufferCount - 1] == 0 && ucRTUBuf[usSndBufferCount] != 0)
            || (ucRTUBuf[usSndBufferCount - 1] == 0 && ucRTUBuf[usSndBufferCount] == 0)
            || (ucRTUBuf[usSndBufferCount - 1] == 0x80 && ucRTUBuf[usSndBufferCount] != 0))
            { usSndBufferCount--; } // Ищем конец массива
        xNeedPoll = EV_FRAME_SENT;  
        eSndState = STATE_TX_XMIT;
        vMBPortSerialEnable( 0, 1 );

        break;

    case EV_FRAME_SENT:
        break;
    }
}

void Coder (void)
{
    uint8_t GetSize;
    uint8_t uint8plain_text_length;
    uint8_t uint8plain_text[256];
    uint8_t plain_text_length;
    uint32_t plain_text[64];
    uint32_t cipher_text[64];

    switch ( xNeedPoll )
    {
    case EV_READY:

        break;
    case EV_FRAME_RECEIVED:

        GetSize = get_size_pad(usRcvBufferPos, PAD_MODE_3);
        uint8plain_text_length = usRcvBufferPos + GetSize;
        set_padding(uint8plain_text, ucRTUBuf, GetSize, usRcvBufferPos, PAD_MODE_3);
        plain_text_length = uint8plain_text_length/4;

        for (uint32_t i=0; i < uint8plain_text_length/4; i++)
            plain_text[i] = (uint8plain_text[4*i] << 24) + (uint8plain_text[4*i+1] << 16) + (uint8plain_text[4*i+2] << 8) + uint8plain_text[4*i+3];              
        HAL_Crypto_Encode(&hcrypto, plain_text, cipher_text, plain_text_length);

        pucSndBufferCur = ( uint8_t * ) ucRTUBuf;
        usSndBufferCount = 0;
        for (uint32_t i=0; i < plain_text_length; i++)
        {
            ucRTUBuf[4*i] = (uint8_t)(cipher_text[i] >> 24);
            ucRTUBuf[4*i + 1] = (uint8_t)(cipher_text[i] >> 16);
            ucRTUBuf[4*i + 2] = (uint8_t)(cipher_text[i] >> 8);
            ucRTUBuf[4*i + 3] = (uint8_t)(cipher_text[i]);
            usSndBufferCount += 4;
        }
        xNeedPoll = EV_FRAME_SENT;  
        eSndState = STATE_TX_XMIT;
        GPIOControlRS485Set ( RS485_TX ); 
        vMBPortSerialEnable( 0, 1 );

        break;

    case EV_FRAME_SENT:                    
        break;
    }
}

void SystemClock_Config(void)
{
    PCC_InitTypeDef PCC_OscInit = {0};

    PCC_OscInit.OscillatorEnable = PCC_OSCILLATORTYPE_ALL;
    PCC_OscInit.FreqMon.OscillatorSystem = PCC_OSCILLATORTYPE_OSC32M;
    PCC_OscInit.FreqMon.ForceOscSys = PCC_FORCE_OSC_SYS_UNFIXED;
    PCC_OscInit.FreqMon.Force32KClk = PCC_FREQ_MONITOR_SOURCE_OSC32K;
    PCC_OscInit.AHBDivider = 0;
    PCC_OscInit.APBMDivider = 0;
    PCC_OscInit.APBPDivider = 0;
    PCC_OscInit.HSI32MCalibrationValue = 128;
    PCC_OscInit.LSI32KCalibrationValue = 128;
    PCC_OscInit.RTCClockSelection = PCC_RTC_CLOCK_SOURCE_AUTO;
    PCC_OscInit.RTCClockCPUSelection = PCC_CPU_RTC_CLOCK_SOURCE_OSC32K;
    HAL_PCC_Config(&PCC_OscInit);
}

void Crypto_Init(void)
{
    hcrypto.Instance = CRYPTO;

    hcrypto.Algorithm = CRYPTO_ALG_KUZNECHIK;
    hcrypto.CipherMode = CRYPTO_CIPHER_MODE_ECB;
    hcrypto.SwapMode = CRYPTO_SWAP_MODE_NONE; 
    hcrypto.OrderMode = CRYPTO_ORDER_MODE_MSW;

    HAL_Crypto_Init(&hcrypto);
}

void USART_Init(void)
{
    husart1.Instance = UART_1;
    #if DECODER
    husart1.transmitting = Enable;
    husart1.receiving = Disable;
    #endif
    #if CODER
    husart1.transmitting = Disable;
    husart1.receiving = Enable;
    husart1.Interrupt.rxneie = Enable;
    #endif
    husart1.frame = Frame_8bit;
    husart1.baudrate = 9600;
    HAL_USART_Init(&husart1);

    husart0.Instance = UART_0;
    #if CODER
    husart0.transmitting = Enable;
    husart0.receiving = Disable;
    #endif
    #if DECODER
    husart0.transmitting = Disable;
    husart0.receiving = Enable;
    husart0.Interrupt.rxneie = Enable;
    #endif
    husart0.frame = Frame_8bit;
    husart0.baudrate = 9600;
    HAL_USART_Init(&husart0);
}

void UART1_IRQHandler(void) //подпрограмма обработки прерываний от UART
{
    #if DECODER
    /* UART in mode Transmitter ------------------------------------------------*/
    if(((husart1.Instance->FLAGS & UART_FLAGS_TXE_M) ? 1 : 0) && ((husart1.Instance->CONTROL1 & (1<<7)) != 0))
    {
        switch ( eSndState )
        {
        case STATE_TX_IDLE:
            vMBPortSerialEnable( 1, 0 );
            break;

        case STATE_TX_XMIT:
            if( usSndBufferCount != 0 )
            {
                husart1.Instance->TXDATA = (int32_t)*pucSndBufferCur;
                pucSndBufferCur++;  /* next byte in sendbuffer. */
                usSndBufferCount--;
            }
            else
            {
                xNeedPoll = EV_READY;
                eSndState = STATE_TX_IDLE;
                vMBPortSerialEnable( 1, 0 );
                for (size_t i = 0; i < 1000; i++){};
                GPIOControlRS485Set ( RS485_RX ); 
            }
            break;
        }
    }
    #endif    
    #if CODER
    /* UART in mode Receiver ---------------------------------------------------*/
    if(((husart1.Instance->FLAGS & UART_FLAGS_RXNE_M) ? 1 : 0) && ((husart1.Instance->CONTROL1 & (1<<5)) != 0))
    { 
        uint8_t           ucByte;

        HAL_TIMER32_VALUE_CLEAR(&htimer32_0);
        ucByte = husart1.Instance->RXDATA;

        switch ( eRcvState )
        {
        case STATE_RX_ERROR:
            TimersEnable();
            break;

        case STATE_RX_IDLE:
            usRcvBufferPos = 0;
            ucRTUBuf[usRcvBufferPos++] = ucByte;
            eRcvState = STATE_RX_RCV;

            TimersEnable();
            break;

        case STATE_RX_RCV:
            ucRTUBuf[usRcvBufferPos++] = ucByte;
            TimersEnable();
            break;
        }
    }
    #endif    
}

void UART0_IRQHandler(void) //подпрограмма обработки прерываний от UART
{
    #if CODER
    /* UART in mode Transmitter ------------------------------------------------*/
    if(((husart0.Instance->FLAGS & UART_FLAGS_TXE_M) ? 1 : 0) && ((husart0.Instance->CONTROL1 & (1<<7)) != 0))
    {
        switch ( eSndState )
        {
        case STATE_TX_IDLE:
            vMBPortSerialEnable( 1, 0 );
            break;

        case STATE_TX_XMIT:
            if( usSndBufferCount != 0 )
            {
                husart0.Instance->TXDATA = (int32_t)*pucSndBufferCur;
                pucSndBufferCur++;  /* next byte in sendbuffer. */
                usSndBufferCount--;
            }
            else
            {
                xNeedPoll = EV_READY;
                eSndState = STATE_TX_IDLE;
                vMBPortSerialEnable( 1, 0 );
                for (size_t i = 0; i < 1000; i++){};
            }
            break;
        }
    }
    #endif
    #if DECODER
    /* UART in mode Receiver ---------------------------------------------------*/
    if(((husart0.Instance->FLAGS & UART_FLAGS_RXNE_M) ? 1 : 0) && ((husart0.Instance->CONTROL1 & (1<<5)) != 0))
    { 
        uint8_t           ucByte;

        HAL_TIMER32_VALUE_CLEAR(&htimer32_0);
        ucByte = husart0.Instance->RXDATA;

        switch ( eRcvState )
        {
        case STATE_RX_ERROR:
            TimersEnable();
            break;

        case STATE_RX_IDLE:
            usRcvBufferPos = 0;
            ucRTUBuf[usRcvBufferPos++] = ucByte;
            eRcvState = STATE_RX_RCV;

            TimersEnable();
            break;

        case STATE_RX_RCV:
            ucRTUBuf[usRcvBufferPos++] = ucByte;
            TimersEnable();
            break;
        }
    }  
    #endif
}

void GPIOControlRS485Init ( void )
{ 
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_PCC_GPIO_0_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = HAL_GPIO_MODE_GPIO_OUTPUT;
    GPIO_InitStruct.Pull = HAL_GPIO_PULL_NONE;
    HAL_GPIO_Init(GPIO_0, &GPIO_InitStruct);
        
    __HAL_PCC_GPIO_1_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = HAL_GPIO_MODE_GPIO_OUTPUT;
    GPIO_InitStruct.Pull = HAL_GPIO_PULL_NONE;
    HAL_GPIO_Init(GPIO_1, &GPIO_InitStruct);   
}

void GPIOControlRS485Set ( RS485 State )
{ 
    switch (State)
    {
    case RS485_TX:
        HAL_GPIO_WritePin(GPIO_0, GPIO_PIN_0, GPIO_PIN_HIGH);
        HAL_GPIO_WritePin(GPIO_1, GPIO_PIN_14, GPIO_PIN_HIGH); 
        break;
    case RS485_RX:
        HAL_GPIO_WritePin(GPIO_0, GPIO_PIN_0, GPIO_PIN_LOW);
        HAL_GPIO_WritePin(GPIO_1, GPIO_PIN_14, GPIO_PIN_LOW); 
        break;
    default:
        break;
    }
}

void Timer32_0_IRQHandler(void)
{
    switch ( eRcvState )
    {
    case STATE_RX_RCV:
        xNeedPoll = EV_FRAME_RECEIVED;
        break;

    default:
    }
    eRcvState = STATE_RX_IDLE;
    TimersDisable();
    HAL_TIMER32_INTERRUPTFLAGS_CLEAR(&htimer32_0);
}

void TimersInit(uint16_t usTim1Timerout50us)
{
    htimer32_0.Instance = TIMER32_0;
    htimer32_0.Top = 25*usTim1Timerout50us;
    htimer32_0.State = TIMER32_STATE_DISABLE;
    htimer32_0.Clock.Source = TIMER32_SOURCE_PRESCALER;
    htimer32_0.Clock.Prescaler = 63;
    htimer32_0.InterruptMask = 0;
    htimer32_0.CountMode = TIMER32_COUNTMODE_FORWARD;
    HAL_Timer32_Init(&htimer32_0);
}

void TimersEnable(void)
{
    HAL_TIMER32_VALUE_CLEAR(&htimer32_0);
    HAL_Timer32_InterruptMask_Set(&htimer32_0, TIMER32_INT_OVERFLOW_M);
    HAL_Timer32_Start(&htimer32_0);
}

void TimersDisable(void)
{
    HAL_Timer32_InterruptMask_Clear(&htimer32_0, TIMER32_INT_OVERFLOW_M);
    HAL_Timer32_Stop(&htimer32_0);
}

void vMBPortSerialEnable(uint8_t xRxEnable, uint8_t xTxEnable)
{
    #if DECODER
    if (xRxEnable)
        HAL_USART_RXNE_EnableInterrupt(&husart0);
    else
        HAL_USART_RXNE_DisableInterrupt(&husart0);
    if (xTxEnable)
        HAL_USART_TXE_EnableInterrupt(&husart1);
    else
        HAL_USART_TXE_DisableInterrupt(&husart1);
    #endif
    #if CODER
    if (xRxEnable)
        HAL_USART_RXNE_EnableInterrupt(&husart1);
    else
        HAL_USART_RXNE_DisableInterrupt(&husart1);
    if (xTxEnable)
        HAL_USART_TXE_EnableInterrupt(&husart0);
    else
        HAL_USART_TXE_DisableInterrupt(&husart0);
    #endif
}