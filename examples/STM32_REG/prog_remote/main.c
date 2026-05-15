/*
 * BC7215 Example Application :
 *  Programmable 4-key remote controller
 *
 * Target platform: STM32F103C8
 *
 * Description:
 *   This application demostrates how to us BC7215 and its library to 
 *   implement 4-key IR remote controller which can independently 'learn'
 *   4 different codes.
 *
 * Author: Bitcode
 * Date: 2024-03
 *
 * Version: 1.0
 *
 * Notes:
 *
 */

#include "bc7215_lib.h"
#include "stm32f10x.h"

/* USER CODE BEGIN PV */
bc7215DataMaxPkt_t IRData[4];        // For simplicity, we use 4 max sized array to save data
                                     // instead of variable length
bc7215FormatPkt_t IRFormat[4];

uint8_t  RxByte;
uint8_t  WorkingMode;
uint8_t  KeyValue;
uint8_t  Counter;
uint32_t DownCounter;

/* USER CODE END PV */

#define MOD_PIN_PORT  GPIOB
#define MOD_PIN_NUM   15
#define BUSY_PIN_PORT GPIOB
#define BUSY_PIN_NUM  14
#define LED_PIN_PORT  GPIOC
#define LED_PIN_NUM   13

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
void    Led_On(void);
void    Led_Off(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* Configure the system clock */
    Clock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    GPIO_Init();
    UART_Init();

    // bc7215 related init
    bc7215_config_uart_int_en_funct(UART_INT_en);
    bc7215_config_uart_int_dis_funct(UART_INT_dis);
    bc7215_config_uart_send_byte(UART_send_1_byte);
    bc7215_config_mod_set_high_funct(set_MOD_high);
    bc7215_config_mod_set_low_funct(set_MOD_low);
    bc7215_config_read_mod_funct(read_MOD);
    bc7215_config_read_busy_funct(read_BUSY);

    NVIC_Config();

    bc7215_set_tx();        // bc7215 set to transmitting mode
    delay_ms(2);

    Led_Off();

    while (1)
    {
        KeyValue = Read_Keypad();
        switch (WorkingMode)
        {
        case 0:        // WorkingMode 0 is IR transmitting mode
            switch (KeyValue)
            {
            case 0x01:        // button 1 pressed
                Led_On();
                bc7215_load_format(&IRFormat[0]);
                bc7215_IR_tx((bc7215DataVarPkt_t*)&IRData[0]);
                Led_Off();
                break;
            case 0x02:        // button 2 pressed
                Led_On();
                bc7215_load_format(&IRFormat[1]);
                bc7215_IR_tx((bc7215DataVarPkt_t*)&IRData[1]);
                Led_Off();
                break;
            case 0x04:        // button 3 pressed
                Led_On();
                bc7215_load_format(&IRFormat[2]);
                bc7215_IR_tx((bc7215DataVarPkt_t*)&IRData[2]);
                Led_Off();
                break;
            case 0x08:        // button 4 pressed
                Led_On();
                bc7215_load_format(&IRFormat[3]);
                bc7215_IR_tx((bc7215DataVarPkt_t*)&IRData[3]);
                Led_Off();
                break;
            case 0x03:                  // button 1 & button 2 pressed
                bc7215_set_rx();        // set BC7215 TO receiving mode
                delay_ms(20);
                bc7215_set_rx_mode(1);        // set BC7215 to complex mode
                WorkingMode = 5;
                break;
            default:
                if ((Counter & 0x0f) == 0)        // every 1600ms
                {
                    Led_On();
                }
                else
                {
                    Led_Off();
                }
                break;
            }
            break;
        case 1:        // WorkingMode 1 is BUTTON1 learning mode
            if (bc7215_format_ready())
            {
                bc7215_get_format(&IRFormat[0]);

                // if data packet is available and size is smaller than the max size
                if (bc7215_data_ready() && (bc7215_get_len() <= sizeof(IRData[0]) * 8))
                {
                    bc7215_get_data((bc7215DataVarPkt_t*)&IRData[0]);
                    Led_Off();
                    bc7215_set_tx();        // switch BC7215 back to transmitting mode
                    delay_ms(2);
                    WorkingMode = 0;        // program change back to normal mode
                }
            }
            break;
        case 2:        // WorkingMode 2 is BUTTON2 learning mode
            if (bc7215_format_ready())
            {
                bc7215_get_format(&IRFormat[1]);

                // if data packet is available and size is smaller than the max size
                if (bc7215_data_ready() && (bc7215_get_len() <= sizeof(IRData[1]) * 8))
                {
                    bc7215_get_data((bc7215DataVarPkt_t*)&IRData[1]);
                    Led_Off();
                    bc7215_set_tx();        // switch BC7215 back to transmitting mode
                    delay_ms(2);
                    WorkingMode = 0;        // program change back to normal mode
                }
            }
            break;
        case 3:        // WorkingMode 3 is BUTTON3 learning mode
            if (bc7215_format_ready())
            {
                bc7215_get_format(&IRFormat[2]);

                // if data packet is available and size is smaller than the max size
                if (bc7215_data_ready() && (bc7215_get_len() <= sizeof(IRData[2]) * 8))
                {
                    bc7215_get_data((bc7215DataVarPkt_t*)&IRData[2]);
                    Led_Off();
                    bc7215_set_tx();        // switch BC7215 back to transmitting mode
                    delay_ms(2);
                    WorkingMode = 0;        // program change back to normal mode
                }
            }
            break;
        case 4:        // WorkingMode 4 is BUTTON4 learning mode
            if (bc7215_format_ready())
            {
                bc7215_get_format(&IRFormat[3]);

                // if data packet is available and size is smaller than the max size
                if (bc7215_data_ready() && (bc7215_get_len() <= sizeof(IRData[3]) * 8))
                {
                    bc7215_get_data((bc7215DataVarPkt_t*)&IRData[3]);
                    Led_Off();
                    bc7215_set_tx();        // switch BC7215 back to transmitting mode
                    delay_ms(2);
                    WorkingMode = 0;        // program change back to normal mode
                }
            }
            break;
        case 5:        //  WorkingMode 5 is entering learning mode
            switch (KeyValue)
            {
            case 0x01:        // button 1 pressed
                Led_On();
                WorkingMode = 1;
                break;
            case 0x02:        // button 2 pressed
                Led_On();
                WorkingMode = 2;
                break;
            case 0x04:        // button 3 pressed
                Led_On();
                WorkingMode = 3;
                break;
            case 0x08:        // button 4 pressed
                Led_On();
                WorkingMode = 4;
                break;
            default:
                if (Counter & 0x02)        // every 200ms
                {
                    Led_On();
                }
                else
                {
                    Led_Off();
                }
                break;
            }
        default:
            break;
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

    GPIOA->CRL = 0x44448888;        // PA0-PA3 set as input with pullup(key pad input)
    GPIOA->CRH = 0x444448A4;        // PA10(USART1 RX) set as input, A9(UART1 TX) set as output
    GPIOA->ODR = 0x0000040F;        // RX(PA10) and PA0-PA3 set pull-up

    GPIOB->CRH = 0x24444444;        // PB15 set as output(MOD), PB14 set as input (BUSY)

    GPIOC->CRH = 0x44244444;        // PC13 set as output (LED)
}

/* USER CODE BEGIN 4 */
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

uint8_t Read_Keypad(void)
{
    uint8_t ButtonMap;
    ButtonMap = GPIOA->IDR & 0x0F;
    if (ButtonMap == 0x0F)        // if no button is pressed
    {
        return 0;
    }
    else
    {
        while ((GPIOA->IDR & 0x000F) != 0x000F)        // if buttons are not released
        {
            if ((GPIOA->IDR & 0x000F) == 0x000C)
            {
                ButtonMap = 0x0C;
            }
        }
        return ButtonMap ^ 0x0f;
    }
}

void Led_On(void)
{
    LED_PIN_PORT->BRR = 1UL << LED_PIN_NUM;        // SET LED PIN LOW (ON)
}

void Led_Off(void)
{
    LED_PIN_PORT->BSRR = 1UL << LED_PIN_NUM;        // SET LED PIN HIGH (OFF)
}

void delay_ms(uint16_t Ms)
{
    DownCounter = Ms + 1;
    while (DownCounter != 0)
        ;
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
