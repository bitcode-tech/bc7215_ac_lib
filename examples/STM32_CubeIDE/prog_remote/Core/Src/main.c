/* USER CODE BEGIN Header */
/**
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bc7215_lib.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_PIN_PORT GPIOC
#define LED_PIN_NUM  GPIO_PIN_13
#define S1_PIN_PORT   GPIOA
#define S1_PIN_NUM    GPIO_PIN_0
#define S2_PIN_PORT   GPIOA
#define S2_PIN_NUM    GPIO_PIN_1
#define S3_PIN_PORT   GPIOA
#define S3_PIN_NUM    GPIO_PIN_2
#define S4_PIN_PORT   GPIOA
#define S4_PIN_NUM    GPIO_PIN_3
#define MOD_PIN_PORT  GPIOB
#define MOD_PIN_NUM   GPIO_PIN_15
#define BUSY_PIN_PORT GPIOB
#define BUSY_PIN_NUM  GPIO_PIN_14
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
bc7215DataMaxPkt_t IRData[4];        // For simplicity, we use 4 max sized array to save data
                                     // instead of variable length
bc7215FormatPkt_t IRFormat[4];
uint8_t           RxByte;
uint8_t           WorkingMode;
uint8_t           KeyValue;
uint8_t           Counter;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void        SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void    set_MOD_high(void);                    // function to set MOD pin high
void    set_MOD_low(void);                     // function to set MOD pin low
uint8_t read_MOD(void);                        // function to read MOD pin
uint8_t read_BUSY(void);                       // function to read BUSY pin
void    UART_INT_en(void);                     // function to enable UART RX interrupt
void    UART_INT_dis(void);                    // function to disable UART RX interrupt
void    UART_send_1_byte(uint8_t data);        // function to send 1 byte

void NVIC_Config(void);        // Config NVIC

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
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    /* USER CODE BEGIN 2 */
    bc7215_config_uart_int_en_funct(UART_INT_en);
    bc7215_config_uart_int_dis_funct(UART_INT_dis);
    bc7215_config_uart_send_byte(UART_send_1_byte);
    bc7215_config_mod_set_high_funct(set_MOD_high);
    bc7215_config_mod_set_low_funct(set_MOD_low);
    bc7215_config_read_mod_funct(read_MOD);
    bc7215_config_read_busy_funct(read_BUSY);

    NVIC_Config();
    HAL_UART_Receive_IT(&huart1, &RxByte, 1);
    bc7215_set_tx();
    HAL_Delay(2);
    Led_Off();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
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
                HAL_Delay(20);
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
                    HAL_Delay(2);
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
                    HAL_Delay(2);
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
                    HAL_Delay(2);
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
                    HAL_Delay(2);
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
        HAL_Delay(100);
    }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 19200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_2;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

    /*Configure GPIO pin : PC13 */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : PB14 */
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : PB15 */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void set_MOD_high(void)        // function to set MOD pin high
{
    HAL_GPIO_WritePin(MOD_PIN_PORT, MOD_PIN_NUM, GPIO_PIN_SET);        // MOD pin is connected to PA12
}

void set_MOD_low(void)        // function to set MOD pin low
{
    HAL_GPIO_WritePin(MOD_PIN_PORT, MOD_PIN_NUM, GPIO_PIN_RESET);        // MOD pin is connected to PA12
}

uint8_t read_MOD(void)        // function to read MOD pin
{
    return HAL_GPIO_ReadPin(MOD_PIN_PORT, MOD_PIN_NUM);        // return status of PA12
}

uint8_t read_BUSY(void)        // function to read BUSY pin
{
    return HAL_GPIO_ReadPin(BUSY_PIN_PORT, BUSY_PIN_NUM);        // MOD pin is connected to PA11
}

void UART_INT_en(void)        // function to enable UART RX interrupt
{
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void UART_INT_dis(void)        // function to disable UART RX interrupt
{
    HAL_NVIC_DisableIRQ(USART1_IRQn);
}

void UART_send_1_byte(uint8_t data)        // function to send 1 byte
{
    HAL_UART_Transmit(&huart1, &data, 1, HAL_MAX_DELAY);
}

void NVIC_Config(void)        // Config NVIC
{
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 1);        // Set priority
    HAL_NVIC_EnableIRQ(USART1_IRQn);                // Enable interrupt
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
    HAL_GPIO_WritePin(LED_PIN_PORT, LED_PIN_NUM, GPIO_PIN_RESET);        // LED on
}

void Led_Off(void)
{
    HAL_GPIO_WritePin(LED_PIN_PORT, LED_PIN_NUM, GPIO_PIN_SET);        // LED off
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)        // RXNE interrupt callback
{
    if (huart->Instance == USART1)
    {
        bc7215_process_uart_data(RxByte);
        HAL_UART_Receive_IT(huart, &RxByte, 1);
    }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
