#include "mik32_hal_pcc.h"
#include "mik32_hal_gpio.h"
#include "csr.h"
#include "mik32_hal_crypto.h"
#include "mik32_hal_usart.h"
#include "mik32_hal_irq.h"
#include "mik32_hal_timer32.h"

/*
 * Данный пример демонстрирует работу с GPIO и PAD_CONFIG.
 * В примере настраивается вывод, который подключенный к светодиоду, в режим GPIO.
 *
 * Плата выбирается ниже в #define
 */
#define BLOCK_SIZE CRYPTO_BLOCK_KUZNECHIK*4

#define PAD_MODE_1 0x01
#define PAD_MODE_2 0x02
#define PAD_MODE_3 0x03

Crypto_HandleTypeDef hcrypto;
USART_HandleTypeDef husart0;
TIMER32_HandleTypeDef htimer32;

void SystemClock_Config();
void GPIO_Init();
static void Crypto_Init(void);
void USART_Init();
void Decoder (void);
void Coder (void);
void TIMER32_Init(void);
void UART_IRQHandler(void);

uint32_t flag = 0;
uint32_t length_input_text = 0;

//uint32_t init_vector[IV_LENGTH_KUZNECHIK_CTR] = {0x12345678, 0x90ABCEF0};
uint32_t init_vector[IV_LENGTH_KUZNECHIK_CBC] = {0x12341234, 0x11114444, 0xABCDABCD, 0xAAAABBBB};

//uint32_t init_vector[IV_LENGTH_MAGMA_CTR] = {0x12345678};
//uint32_t init_vector[IV_LENGTH_MAGMA_CBC] = {0x12341234, 0x11114444};

uint32_t crypto_key[CRYPTO_KEY_KUZNECHIK] = {0x8899aabb, 0xccddeeff, 0x00112233, 0x44556677, 0xfedcba98, 0x76543210, 0x01234567, 0x89abcdef};                     

uint8_t output_text[] = { 
                            0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 
                            0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
                            0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
                            0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
                            0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
                            0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
                            0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
                            0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
                            0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
                            0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
                            0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
                            0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
                            0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35
                        };  
uint8_t input_text[256] = { 
                       };                                                

uint32_t key_length = sizeof(crypto_key)/sizeof(*crypto_key);

extern unsigned long __TEXT_START__; // это "метка" для обработчика прерываний
volatile void trap_handler(void)     // сам обработчик всех прерываний
{

    if (EPIC_CHECK_GPIO_IRQ())
    {
        if (HAL_GPIO_LineInterruptState(GPIO_LINE_7))
        {
            HAL_DelayMs(50);
            HAL_GPIO_TogglePin(GPIO_0, GPIO_PIN_9);
            flag = !flag;
        }
        HAL_GPIO_ClearInterrupts();
    }
        if(HAL_EPIC_GetStatus() & HAL_EPIC_UART_1_MASK)
    {
        UART_IRQHandler();
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

void kuznechik_ECB_code(uint32_t *plain, uint32_t *cipher, uint32_t plain_length)
{
    /* Задать режим шифрования */
    HAL_Crypto_SetCipherMode(&hcrypto, CRYPTO_CIPHER_MODE_ECB);
    /* Установка ключа */
    HAL_Crypto_SetKey(&hcrypto, crypto_key);
    /* Зашифровать данные */
    HAL_Crypto_Encode(&hcrypto, plain, cipher, plain_length); 
}

void kuznechik_ECB_decode(uint32_t *cipher, uint32_t *expect_cipher, uint32_t plain_length)
{
    /* Задать режим шифрования */
    HAL_Crypto_SetCipherMode(&hcrypto, CRYPTO_CIPHER_MODE_ECB);
    /* Установка ключа */
    HAL_Crypto_SetKey(&hcrypto, crypto_key);
    /* Расшифровать данные */
    HAL_Crypto_Decode(&hcrypto, cipher, expect_cipher, plain_length); 
}

void kuznechik_CBC_code(uint32_t *plain, uint32_t *cipher, uint32_t plain_length)
{
    /* Задать режим шифрования */
    HAL_Crypto_SetCipherMode(&hcrypto, CRYPTO_CIPHER_MODE_CBC);
    /* Установка вектора инициализации */  
    HAL_Crypto_SetIV(&hcrypto, init_vector, sizeof(init_vector)/sizeof(*init_vector)); 
    /* Установка ключа */
    HAL_Crypto_SetKey(&hcrypto, crypto_key);
    /* Зашифровать данные */
    HAL_Crypto_Encode(&hcrypto, plain, cipher, plain_length);     
}

void kuznechik_CBC_decode(uint32_t *cipher, uint32_t *expect_cipher, uint32_t plain_length)
{
    /* Задать режим шифрования */
    HAL_Crypto_SetCipherMode(&hcrypto, CRYPTO_CIPHER_MODE_CBC);
    /* Установка вектора инициализации */  
    HAL_Crypto_SetIV(&hcrypto, init_vector, sizeof(init_vector)/sizeof(*init_vector)); 
    /* Установка ключа */
    HAL_Crypto_SetKey(&hcrypto, crypto_key);
    /* Расшифровать данные */
    HAL_Crypto_Decode(&hcrypto, cipher, expect_cipher, plain_length); 
}

void kuznechik_CTR_code(uint32_t *plain, uint32_t *cipher, uint32_t plain_length)
{
    /* Задать режим шифрования */
    HAL_Crypto_SetCipherMode(&hcrypto, CRYPTO_CIPHER_MODE_CTR);
    /* Установка вектора инициализации */  
    HAL_Crypto_SetIV(&hcrypto, init_vector, sizeof(init_vector)/sizeof(*init_vector)); 
    /* Установка ключа */
    HAL_Crypto_SetKey(&hcrypto, crypto_key);
    /* Зашифровать данные */
    HAL_Crypto_Encode(&hcrypto, plain, cipher, plain_length); 
}

void kuznechik_CTR_decode(uint32_t *cipher, uint32_t *expect_cipher, uint32_t plain_length)
{
    /* Задать режим шифрования */
    HAL_Crypto_SetCipherMode(&hcrypto, CRYPTO_CIPHER_MODE_CTR);
    /* Установка вектора инициализации */  
    HAL_Crypto_SetIV(&hcrypto, init_vector, sizeof(init_vector)/sizeof(*init_vector)); 
    /* Установка ключа */
    HAL_Crypto_SetKey(&hcrypto, crypto_key);
    /* Расшифровать данные */
    HAL_Crypto_Decode(&hcrypto, cipher, expect_cipher, plain_length); 
}

int main()
{
    write_csr(mtvec, &__TEXT_START__); // операция, настраивающая вектор прерываний
    
    __HAL_PCC_EPIC_CLK_ENABLE();
    HAL_EPIC_MaskEdgeSet(HAL_EPIC_UART_1_MASK); 
    HAL_EPIC_MaskLevelSet(HAL_EPIC_GPIO_IRQ_MASK);

    HAL_IRQ_EnableInterrupts();

    SystemClock_Config();

    Crypto_Init();

    USART_Init();

    GPIO_Init();

    TIMER32_Init();

    // kuznechik_ECB_code();
    // kuznechik_ECB_decode();

    // kuznechik_CBC_code();
    // kuznechik_CBC_decode();

    // kuznechik_CTR_code(); 
    // kuznechik_CTR_decode();
      
    while (1)
    {
        HAL_GPIO_TogglePin(GPIO_0, GPIO_PIN_10);
        HAL_DelayMs(10000);
        switch (flag)
        {
        case 0:
            Coder();
            break;
        default:
            Decoder();
            break;
        }
    }
}

void Decoder (void)
{
    for (int i = 0; i < 256; i++) {input_text[i] = 0;}
    length_input_text = 0;    
    while (!HAL_USART_IDLE_ReadFlag(&husart0)) {};
    HAL_USART_IDLE_ClearFlag(&husart0);
    uint32_t cipher_text_dec[length_input_text/4];
    for(int i = 0; i < sizeof(cipher_text_dec)/sizeof(*cipher_text_dec); i++)
    {
        cipher_text_dec[i] = (input_text[4*i] << 24) + (input_text[4*i+1] << 16) + (input_text[4*i+2] << 8) + input_text[4*i+3];
    }

    uint32_t expect_cipher_text[sizeof(cipher_text_dec)/sizeof(*cipher_text_dec)];

    kuznechik_ECB_decode(cipher_text_dec, expect_cipher_text, sizeof(cipher_text_dec)/sizeof(*cipher_text_dec));

     for (uint32_t i=0; i < sizeof(expect_cipher_text)/sizeof(*expect_cipher_text); i++)
     {
         HAL_USART_Transmit(&husart0, expect_cipher_text[i]>>24, 100);
         HAL_USART_Transmit(&husart0, expect_cipher_text[i]>>16, 100);
         HAL_USART_Transmit(&husart0, expect_cipher_text[i]>>8, 100);
         HAL_USART_Transmit(&husart0, expect_cipher_text[i], 100);                         
     }
}

void Coder (void)
{
    while (!HAL_USART_IDLE_ReadFlag(&husart0)) {};
    HAL_USART_IDLE_ClearFlag(&husart0);

    uint8_t uint8plain_text[sizeof(output_text)/sizeof(*output_text) + get_size_pad(sizeof(output_text)/sizeof(*output_text), PAD_MODE_3)];

    set_padding(uint8plain_text, output_text, get_size_pad(sizeof(output_text)/sizeof(*output_text), PAD_MODE_3), sizeof(output_text)/sizeof(*output_text), PAD_MODE_3);

    //for (uint32_t i=0; i < sizeof(uint8plain_text)/sizeof(*uint8plain_text); i++)
    //{
    //    HAL_USART_Transmit(&husart0, uint8plain_text[i], 100);                         
    //}  

    uint32_t plain_text[sizeof(uint8plain_text)/sizeof(*uint8plain_text)/4];

    for (uint32_t i=0; i < sizeof(uint8plain_text)/sizeof(*uint8plain_text)/4; i++)
     {
         plain_text[i] = (uint8plain_text[4*i] << 24) + (uint8plain_text[4*i+1] << 16) + (uint8plain_text[4*i+2] << 8) + uint8plain_text[4*i+3];              
     }       

    uint32_t cipher_text[sizeof(plain_text)/sizeof(*plain_text)];

    //HAL_TIMER32_VALUE_CLEAR(&htimer32);
    //HAL_Timer32_Base_Start_IT(&htimer32);

    kuznechik_ECB_code(plain_text, cipher_text, sizeof(plain_text)/sizeof(*plain_text));

    //HAL_Timer32_Base_Stop_IT(&htimer32);
    //HAL_USART_Transmit(&husart0, TIMER32_0->VALUE>>24, 100);
    //HAL_USART_Transmit(&husart0, TIMER32_0->VALUE>>16, 100);
    //HAL_USART_Transmit(&husart0, TIMER32_0->VALUE>>8, 100);
    //HAL_USART_Transmit(&husart0, TIMER32_0->VALUE, 100);
    //HAL_USART_Transmit(&husart0, 0xFF, 100);

     for (uint32_t i=0; i < sizeof(plain_text)/sizeof(*plain_text); i++)
     {
         HAL_USART_Transmit(&husart0, cipher_text[i]>>24, 100);
         HAL_USART_Transmit(&husart0, cipher_text[i]>>16, 100);
         HAL_USART_Transmit(&husart0, cipher_text[i]>>8, 100);
         HAL_USART_Transmit(&husart0, cipher_text[i], 100);                         
     }

     uint32_t expect_cipher_text[sizeof(cipher_text)/sizeof(*cipher_text)];

    //HAL_TIMER32_VALUE_CLEAR(&htimer32);
    //HAL_Timer32_Base_Start_IT(&htimer32);

    kuznechik_ECB_decode(cipher_text, expect_cipher_text, sizeof(cipher_text)/sizeof(*cipher_text));
    
    //HAL_Timer32_Base_Stop_IT(&htimer32);
    //HAL_USART_Transmit(&husart0, TIMER32_0->VALUE>>24, 100);
    //HAL_USART_Transmit(&husart0, TIMER32_0->VALUE>>16, 100);
    //HAL_USART_Transmit(&husart0, TIMER32_0->VALUE>>8, 100);
    //HAL_USART_Transmit(&husart0, TIMER32_0->VALUE, 100);

    // for (uint32_t i=0; i < sizeof(expect_cipher_text)/sizeof(*expect_cipher_text); i++)
    // {
    //     HAL_USART_Transmit(&husart0, expect_cipher_text[i]>>24, 100);
    //     HAL_USART_Transmit(&husart0, expect_cipher_text[i]>>16, 100);
    //     HAL_USART_Transmit(&husart0, expect_cipher_text[i]>>8, 100);
    //     HAL_USART_Transmit(&husart0, expect_cipher_text[i], 100);                         
    // }

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

void GPIO_Init()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_PCC_GPIO_0_CLK_ENABLE();
    __HAL_PCC_GPIO_1_CLK_ENABLE();
    __HAL_PCC_GPIO_2_CLK_ENABLE();
    __HAL_PCC_GPIO_IRQ_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = HAL_GPIO_MODE_GPIO_OUTPUT;
    GPIO_InitStruct.Pull = HAL_GPIO_PULL_NONE;
    HAL_GPIO_Init(GPIO_0, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIO_0, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = HAL_GPIO_MODE_GPIO_INPUT;
    HAL_GPIO_Init(GPIO_1, &GPIO_InitStruct); 

    HAL_GPIO_InitInterruptLine(GPIO_MUX_LINE_7_PORT1_15, GPIO_INT_MODE_RISING);  
}

static void Crypto_Init(void)
{
    hcrypto.Instance = CRYPTO;

    hcrypto.Algorithm = CRYPTO_ALG_KUZNECHIK;
    hcrypto.CipherMode = CRYPTO_CIPHER_MODE_ECB;
    hcrypto.SwapMode = CRYPTO_SWAP_MODE_NONE; 
    hcrypto.OrderMode = CRYPTO_ORDER_MODE_MSW;

    HAL_Crypto_Init(&hcrypto);
}

void USART_Init()
{
    husart0.Instance = UART_1;
    husart0.transmitting = Enable;
    husart0.receiving = Enable;
    husart0.frame = Frame_8bit;
    husart0.parity_bit = Disable;
    husart0.parity_bit_inversion = Disable;
    husart0.bit_direction = LSB_First;
    husart0.data_inversion = Disable;
    husart0.tx_inversion = Disable;
    husart0.rx_inversion = Disable;
    husart0.swap = Disable;
    husart0.lbm = Disable;
    husart0.stop_bit = StopBit_1;
    husart0.mode = Asynchronous_Mode;
    husart0.xck_mode = XCK_Mode3;
    husart0.last_byte_clock = Disable;
    husart0.overwrite = Disable;
    husart0.rts_mode = AlwaysEnable_mode;
    husart0.dma_tx_request = Disable;
    husart0.dma_rx_request = Disable;
    husart0.channel_mode = Duplex_Mode;
    husart0.tx_break_mode = Disable;
    husart0.Interrupt.ctsie = Disable;
    husart0.Interrupt.eie = Disable;
    husart0.Interrupt.idleie = Disable;
    husart0.Interrupt.lbdie = Disable;
    husart0.Interrupt.peie = Disable;
    husart0.Interrupt.rxneie = Enable;
    husart0.Interrupt.tcie = Disable;
    husart0.Interrupt.txeie = Disable;
    husart0.Modem.rts = Disable; //out
    husart0.Modem.cts = Disable; //in
    husart0.Modem.dtr = Disable; //out
    husart0.Modem.dcd = Disable; //in
    husart0.Modem.dsr = Disable; //in
    husart0.Modem.ri = Disable;  //in
    husart0.Modem.ddis = Disable;//out
    husart0.baudrate = 9600;
    HAL_USART_Init(&husart0);
}

void TIMER32_Init(void)
{
    htimer32.Instance = TIMER32_0;
    htimer32.Top = 0xFFFF;
    htimer32.State = TIMER32_STATE_DISABLE;
    htimer32.Clock.Source = TIMER32_SOURCE_PRESCALER;
    htimer32.Clock.Prescaler = 0;
    htimer32.InterruptMask = 0;
    htimer32.CountMode = TIMER32_COUNTMODE_FORWARD;
    HAL_Timer32_Init(&htimer32);
}

void UART_IRQHandler() //подпрограмма обработки прерываний от UART
{
    /* UART in mode Receiver ---------------------------------------------------*/
    if((HAL_USART_RXNE_ReadFlag(&husart0) != 0) && ((husart0.Instance->CONTROL1 & (1<<5)) != 0))
    { 
        input_text[length_input_text++] = HAL_USART_ReadByte(&husart0);
    }    
}