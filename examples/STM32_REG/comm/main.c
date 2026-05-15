/*
 * BC7215 Example Application :
 *  Infrared Data Communication
 *
 * Target platform: STM32F103C8
 *
 * Description:
 *   This application demostrates how to us BC7215 and its library in
 *   the IR wireless communication. The system has 2 buttons and 3 LEDs,
 *   one button for sending data with CRC checksum, and the other send
 *   the same data without CRC.
 *   3 LEDs show the status: working mode, CRC check error, and CRC 
 *   check passed.
 *
 * Author: Bitcode
 * Date: 2024-03
 *
 * Version: 1.0
 *
 * Notes:
 *   The bc7215_lib_config.h file is changed according to the requirments
 *   of this application.
 *
 */

#include "bc7215_lib.h"
#include "stm32f10x.h"

const bc7215FormatPkt_t NECFormat = { { { 0x34 } },
    { 0x14, 0x5D, 0x0D, 0x5D, 0x14, 0x3D, 0x3D, 0x1D, 0x1C, 0x9C, 0x62, 0xA0, 0x29, 0xB2, 0x99, 0x44, 0x00, 0x00, 0xC2,
        0x36, 0x9F, 0xF7, 0xFA, 0xB8, 0xE2, 0x9A, 0xA3, 0x26, 0xEA, 0x90, 0x87, 0x30 } };

const uint8_t BinData[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20 };
const char    TxtData[] = { "Hello World." };

uint8_t BufData[32];
uint8_t crc;

uint8_t  i, Len;
uint8_t  WorkingMode;
uint8_t  KeyValue;
uint8_t  Counter;
uint32_t DownCounter;

/* USER CODE END PV */

#define MOD_PIN_PORT  GPIOB
#define MOD_PIN_NUM   15
#define BUSY_PIN_PORT GPIOB
#define BUSY_PIN_NUM  14
#define LED1_PIN_PORT GPIOC
#define LED1_PIN_NUM  13
#define LED3_PIN_PORT GPIOA
#define LED3_PIN_NUM  2
#define LED2_PIN_PORT GPIOA
#define LED2_PIN_NUM  3

/* Private function prototypes -----------------------------------------------*/
void Clock_Config(void);
void GPIO_Init(void);
void UART_Init(void);
void NVIC_Config(void);        // Config NVIC
void delay_ms(uint16_t Ms);

// functions needed by BC7215 library
void    set_MOD_high(void);                    // function to set MOD pin high
void    set_MOD_low(void);                     // function to set MOD pin low
uint8_t read_MOD(void);                        // function to read MOD pin
uint8_t read_BUSY(void);                       // function to read BUSY pin
void    UART_INT_en(void);                     // function to enable UART RX interrupt
void    UART_INT_dis(void);                    // function to disable UART RX interrupt
void    UART_send_1_byte(uint8_t data);        // function to send 1 byte

uint8_t Read_Keypad(void);
void    Led1_On(void);
void    Led1_Off(void);
void    Led2_On(void);
void    Led2_Off(void);
void    Led3_On(void);
void    Led3_Off(void);

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    // System Init.
    Clock_Config();
    GPIO_Init();
    UART_Init();
    NVIC_Config();        // enable interrupt

    // bc7215 related init
    bc7215_config_uart_int_en_funct(UART_INT_en);
    bc7215_config_uart_int_dis_funct(UART_INT_dis);
    bc7215_config_uart_send_byte(UART_send_1_byte);
    bc7215_config_mod_set_high_funct(set_MOD_high);
    bc7215_config_mod_set_low_funct(set_MOD_low);
    bc7215_config_read_mod_funct(read_MOD);
    bc7215_config_read_busy_funct(read_BUSY);

    Led1_Off();
    Led2_Off();
    Led3_Off();

    bc7215_set_rx();        // bc7215 set to receiving mode
    delay_ms(20);

    while (1)
    {
        KeyValue = Read_Keypad();
        switch (KeyValue)
        {
        case 0x01:        // button 1 pressed, send data with CRC
            Led1_On();
            bc7215_set_tx();        // switch BC7215 to TX mode
            delay_ms(2);

            // load format in case the internal format information was over written
            // by received IR signal. If we are staying in TX mode, then reload the
            // format information is not necessary.
            bc7215_load_format(&NECFormat);

            // sending binary data
            // User must make sure sizeof(BinData) is not larger than sizeof(BufData)-1
            for (i = 0; i < sizeof(BinData); i++)
            {
                BufData[i] = BinData[i];        // user should make sure BufData is at least 1 byte larger than BinData
            }
            crc = bc7215_crc8(BinData, sizeof(BinData));
            BufData[sizeof(BinData)] = crc;        // add CRC at end
            bc7215_send_raw(BufData, sizeof(BinData) + 1);
            while (!bc7215_cmd_completed())
                ;

            delay_ms(100);        // should wait at least 40ms before another transmitting

            // sending text data, no need to load format information again
            // User must make sure sizeof(TxtData) is not larger than sizeof(BufData)-1
            for (i = 0; i < sizeof(TxtData) - 1; i++)
            {
                BufData[i] = TxtData[i];        // user should make sure BufData is at least of same size as TxtData
            }
            crc = bc7215_crc8(
                (uint8_t*)TxtData, sizeof(TxtData) - 1);        // only calculate the text part, not including the '0' at the end.
            BufData[sizeof(TxtData) - 1] = crc;
            bc7215_send_raw(BufData, sizeof(TxtData));
            while (!bc7215_cmd_completed())
                ;

            Led1_Off();
            bc7215_set_rx();        // turn back to receiving mode, waiting for incoming data
            delay_ms(20);
            break;
        case 0x02:        // button 2 pressed, send data without CRC
            Led1_On();
            bc7215_set_tx();        // switch BC7215 to TX mode
            delay_ms(2);

            // load format in case the internal format information was over written
            // by received IR signal. If we are staying in TX mode, then reload the
            // format information is not necessary.
            bc7215_load_format(&NECFormat);

            // sending binary data
            bc7215_send_raw(BinData, sizeof(BinData));
            while (!bc7215_cmd_completed())
                ;

            delay_ms(100);        // should wait at least 40ms before another transmitting

            // sending text data, no need to load format information again
            bc7215_send_raw(TxtData, sizeof(TxtData)-1);
            while (!bc7215_cmd_completed())
                ;

            Led1_Off();
            bc7215_set_rx();        // turn back to receiving mode, waiting for incoming data
            delay_ms(20);
            break;
        default:
            if ((Counter & 0x0f) == 0)        // every 1600ms
            {
                Led1_On();
            }
            else
            {
                Led1_Off();
            }
            break;
        }
        if (bc7215_data_ready())
        {
            Len = (bc7215_get_len() + 7) / 8;        // get data size in byte
            bc7215_get_raw(BufData, Len);            // get all received data

            // *** data received, process the data any way you want here ***

            if (bc7215_crc8(BufData, Len) == 0)        // check if CRC is correct
            {
                Led2_On();
                delay_ms(200);
                Led2_Off();
            }
            else        // CRC check failed, flash LED twice
            {
                Led3_On();
                delay_ms(200);
                Led3_Off();
            }
        }
        Counter++;
        delay_ms(100);
    }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void Clock_Config(void)
{
    SystemCoreClockUpdate();
    RCC->APB2ENR
        |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;
    SysTick_Config(SystemCoreClock / 1000);        // set SysTick period to 1ms
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
void UART_Init(void)
{
    USART1->BRR = SystemCoreClock / 19200;        // set baud rate to 19200
    USART1->CR2 = USART_CR2_STOP_1;               // set to 2 stop bits
    USART1->CR3 = 0;
    USART1->CR1 = USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_TE;
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
void GPIO_Init(void)
{
    /* CRL/CRH mostly used values:
     4 -- float input	(default)
     3 -- push-pull 50MHz
     2 -- push-pull 2MHz
     1 -- push-pull 10MHz
     B -- alternate push-pull	50MHz
     A -- alternate push-pull	2MHz
     9 -- alternate push-pull	10MHz
     8 -- input with pull-up/pull-down
     0 -- analog input
    */

    GPIOA->CRL = 0x44442288;        // PA0-PA1 set as input with pullup(key pad input), PA2-PA3 as output
    GPIOA->CRH = 0x444448B4;        // PA10(USART1 RX) set as input, A9(UART1 TX) set as output
    GPIOA->ODR = 0x0000060F;        // RX(PA10) and PA0-PA1 set pull-up, PA2-PA3 set 1

    GPIOB->CRH = 0x24444444;        // PB15 set as output(MOD), PB14 set as input (BUSY)

    GPIOC->CRH = 0x44244444;        // PC13 set as output (LED)
}

void set_MOD_high(void)        // function to set MOD pin high
{
    MOD_PIN_PORT->BSRR = 1UL << MOD_PIN_NUM;
}

void set_MOD_low(void)        // function to set MOD pin low
{
    MOD_PIN_PORT->BRR = 1UL << MOD_PIN_NUM;
}

uint8_t read_MOD(void)        // function to read MOD pin
{
    if (MOD_PIN_PORT->IDR & (1UL << MOD_PIN_NUM))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

uint8_t read_BUSY(void)        // function to read BUSY pin
{
    if (BUSY_PIN_PORT->IDR & (1UL << BUSY_PIN_NUM))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void UART_INT_en(void)        // function to enable UART RX interrupt
{
    NVIC_EnableIRQ(USART1_IRQn);
}

void UART_INT_dis(void)        // function to disable UART RX interrupt
{
    NVIC_DisableIRQ(USART1_IRQn);
}

void UART_send_1_byte(uint8_t data)        // function to send 1 byte
{
    (void)USART1->SR;        // read SR then write to DR clears TC bit
    USART1->DR = data;
    while (!(USART1->SR & USART_SR_TC))
        ;        // wait for sending to complete
}

void NVIC_Config(void)        // Config NVIC
{
    NVIC_EnableIRQ(USART1_IRQn);        // Enable interrupt
}

void USART1_IRQHandler(void)
{
    if (USART1->SR & USART_SR_RXNE)        // if the	interrupt source is RXNE
    {
        bc7215_process_uart_data(USART1->DR & 0XFF);
    }
}

void SysTick_Handler(void)
{
    if (DownCounter != 0)
    {
        --DownCounter;
    }
}

void delay_ms(uint16_t Ms)
{
    DownCounter = Ms + 1;
    while (DownCounter != 0)
        ;
}

uint8_t Read_Keypad(void)
{
    uint8_t ButtonMap;
    if ((GPIOA->IDR & 0x03) == 0x03)        // if no button is pressed
    {
        return 0;
    }
    else
    {
        ButtonMap = (~GPIOA->IDR) & 0x03;
        while ((GPIOA->IDR & 0x0003) != 0x0003)
            ;        // wait for button to be released
        return ButtonMap;
    }
}

void Led1_On(void)
{
    LED1_PIN_PORT->BRR = 1UL << LED1_PIN_NUM;        // SET LED PIN LOW (ON)
}

void Led1_Off(void)
{
    LED1_PIN_PORT->BSRR = 1UL << LED1_PIN_NUM;        // SET LED PIN HIGH (OFF)
}

void Led2_On(void)
{
    LED2_PIN_PORT->BRR = 1UL << LED2_PIN_NUM;        // SET LED PIN LOW (ON)
}

void Led2_Off(void)
{
    LED2_PIN_PORT->BSRR = 1UL << LED2_PIN_NUM;        // SET LED PIN HIGH (OFF)
}

void Led3_On(void)
{
    LED3_PIN_PORT->BRR = 1UL << LED3_PIN_NUM;        // SET LED PIN LOW (ON)
}

void Led3_Off(void)
{
    LED3_PIN_PORT->BSRR = 1UL << LED3_PIN_NUM;        // SET LED PIN HIGH (OFF)
}
