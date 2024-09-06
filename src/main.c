#include "mik32_hal_pcc.h"
#include "mik32_hal_gpio.h"
#include "csr.h"
#include "mik32_hal_crypto.h"
#include "mik32_hal_usart.h"

/*
 * Данный пример демонстрирует работу с GPIO и PAD_CONFIG.
 * В примере настраивается вывод, который подключенный к светодиоду, в режим GPIO.
 *
 * Плата выбирается ниже в #define
 */
#define BLOCK_SIZE 16

#define PAD_MODE_1 0x01
#define PAD_MODE_2 0x02
#define PAD_MODE_3 0x03

Crypto_HandleTypeDef hcrypto;
USART_HandleTypeDef husart0;

void SystemClock_Config();
void GPIO_Init();
static void Crypto_Init(void);
void USART_Init();

uint32_t init_vector[IV_LENGTH_KUZNECHIK_CTR] = {0x12345678, 0x90ABCEF0};
//uint32_t init_vector[IV_LENGTH_KUZNECHIK_CBC] = {0x12341234, 0x11114444, 0xABCDABCD, 0xAAAABBBB};

uint32_t crypto_key[CRYPTO_KEY_KUZNECHIK] = {0x8899aabb, 0xccddeeff, 0x00112233, 0x44556677, 0xfedcba98, 0x76543210, 0x01234567, 0x89abcdef};                     

uint8_t input_text[] = { 
                            "Maksim, u tebja pochti poluchilos" 
                       };                         

uint32_t key_length = sizeof(crypto_key)/sizeof(*crypto_key);

extern unsigned long __TEXT_START__; // это "метка" для обработчика прерываний
volatile void trap_handler(void)     // сам обработчик всех прерываний
{
    HAL_GPIO_TogglePin(GPIO_0, GPIO_PIN_9);
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

    SystemClock_Config();

    Crypto_Init();

    USART_Init();

    GPIO_Init();

    uint8_t uint8plain_text[sizeof(input_text)/sizeof(*input_text) + get_size_pad(sizeof(input_text)/sizeof(*input_text), PAD_MODE_3)];

    set_padding(uint8plain_text, input_text, get_size_pad(sizeof(input_text)/sizeof(*input_text), PAD_MODE_3), sizeof(input_text)/sizeof(*input_text), PAD_MODE_3);

     for (uint32_t i=0; i < sizeof(uint8plain_text)/sizeof(*uint8plain_text); i++)
     {
         HAL_USART_Transmit(&husart0, uint8plain_text[i], 100);                         
     }  

    uint32_t plain_text[sizeof(uint8plain_text)/sizeof(*uint8plain_text)/4];

    for (uint32_t i=0; i < sizeof(uint8plain_text)/sizeof(*uint8plain_text)/4; i++)
     {
         plain_text[i] = (uint8plain_text[4*i] << 24) + (uint8plain_text[4*i+1] << 16) + (uint8plain_text[4*i+2] << 8) + uint8plain_text[4*i+3];              
     }       

    uint32_t cipher_text[sizeof(plain_text)/sizeof(*plain_text)];

    kuznechik_ECB_code(plain_text, cipher_text, sizeof(plain_text)/sizeof(*plain_text));

     for (uint32_t i=0; i < sizeof(plain_text)/sizeof(*plain_text); i++)
     {
         HAL_USART_Transmit(&husart0, cipher_text[i]>>24, 100);
         HAL_USART_Transmit(&husart0, cipher_text[i]>>16, 100);
         HAL_USART_Transmit(&husart0, cipher_text[i]>>8, 100);
         HAL_USART_Transmit(&husart0, cipher_text[i], 100);                         
     }

     uint32_t expect_cipher_text[sizeof(cipher_text)/sizeof(*cipher_text)];

    kuznechik_ECB_decode(cipher_text, expect_cipher_text, sizeof(cipher_text)/sizeof(*cipher_text));

     for (uint32_t i=0; i < sizeof(expect_cipher_text)/sizeof(*expect_cipher_text); i++)
     {
         HAL_USART_Transmit(&husart0, expect_cipher_text[i]>>24, 100);
         HAL_USART_Transmit(&husart0, expect_cipher_text[i]>>16, 100);
         HAL_USART_Transmit(&husart0, expect_cipher_text[i]>>8, 100);
         HAL_USART_Transmit(&husart0, expect_cipher_text[i], 100);                         
     }

    /*
    uint32_t charcipher_text[];
    HAL_USART_Read(&husart0, charcipher_text, 16, 100000);
    uint32_t cipher_text[sizeof(charcipher_text)/sizeof(*charcipher_text)];
    for(int i = 0; i < sizeof(cipher_text)/sizeof(*cipher_text); i++)
    {
        cipher_text[i] = (charcipher_text[4*i] << 24) + (charcipher_text[4*i+1] << 16) + (charcipher_text[4*i+2] << 8) + charcipher_text[4*i+3];
    }

    uint32_t expect_cipher_text[sizeof(cipher_text)/sizeof(*cipher_text)];

    kuznechik_ECB_decode(cipher_text, expect_cipher_text, sizeof(cipher_text)/sizeof(*cipher_text));

     for (uint32_t i=0; i < sizeof(expect_cipher_text)/sizeof(*expect_cipher_text); i++)
     {
         HAL_USART_Transmit(&husart0, expect_cipher_text[i]>>24, 100);
         HAL_USART_Transmit(&husart0, expect_cipher_text[i]>>16, 100);
         HAL_USART_Transmit(&husart0, expect_cipher_text[i]>>8, 100);
         HAL_USART_Transmit(&husart0, expect_cipher_text[i], 100);                         
     }
*/
    // kuznechik_CBC_code();
    // kuznechik_CBC_decode();


    // kuznechik_CTR_code(); 
    // kuznechik_CTR_decode();
      
    while (1)
    {
        HAL_GPIO_TogglePin(GPIO_0, GPIO_PIN_10);
        HAL_DelayMs(500);
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
    husart0.Interrupt.rxneie = Disable;
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