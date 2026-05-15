/* USER CODE BEGIN Header */
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
#define LED1_PIN_PORT GPIOC
#define LED1_PIN_NUM  GPIO_PIN_13
#define LED2_PIN_PORT GPIOA
#define LED2_PIN_NUM  GPIO_PIN_3
#define LED3_PIN_PORT GPIOA
#define LED3_PIN_NUM  GPIO_PIN_2
#define S1_PIN_PORT   GPIOA
#define S1_PIN_NUM    GPIO_PIN_0
#define S2_PIN_PORT   GPIOA
#define S2_PIN_NUM    GPIO_PIN_1
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
const bc7215FormatPkt_t NECFormat = { { { 0x34 } },
    { 0x14, 0x5D, 0x0D, 0x5D, 0x14, 0x3D, 0x3D, 0x1D, 0x1C, 0x9C, 0x62, 0xA0, 0x29, 0xB2, 0x99, 0x44, 0x00, 0x00, 0xC2,
        0x36, 0x9F, 0xF7, 0xFA, 0xB8, 0xE2, 0x9A, 0xA3, 0x26, 0xEA, 0x90, 0x87, 0x30 } };

const uint8_t BinData[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20 };
const char    TxtData[] = { "Hello World." };

uint8_t BufData[32];
uint8_t crc;

uint8_t  i, Len;
uint8_t  RxByte;
uint8_t  WorkingMode;
uint8_t  KeyValue;
uint8_t  Counter;
uint32_t DownCounter;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void        SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
// functions needed by BC7215 library
void    set_MOD_high(void);                    // function to set MOD pin high
void    set_MOD_low(void);                     // function to set MOD pin low
uint8_t read_MOD(void);                        // function to read MOD pin
uint8_t read_BUSY(void);                       // function to read BUSY pin
void    UART_INT_en(void);                     // function to enable UART RX interrupt
void    UART_INT_dis(void);                    // function to disable UART RX interrupt
void    UART_send_1_byte(uint8_t data);        // function to send 1 byte

// functions to operate I/O
void    NVIC_Config(void);        // Config NVIC
uint8_t Read_Keypad(void);
void    Led1_On(void);
void    Led1_Off(void);
void    Led2_On(void);
void    Led2_Off(void);
void    Led3_On(void);
void    Led3_Off(void);
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

    NVIC_Config();
    HAL_UART_Receive_IT(&huart1, &RxByte, 1);
    bc7215_set_rx();        // bc7215 set to receiving mode
    HAL_Delay(20);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        KeyValue = Read_Keypad();
        switch (KeyValue)
        {
        case 0x01:        // button 1 pressed, send data with CRC
            Led1_On();
            bc7215_set_tx();        // switch BC7215 to TX mode
            HAL_Delay(2);

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

            HAL_Delay(100);        // should wait at least 40ms before another transmitting

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
            HAL_Delay(20);
            break;
        case 0x02:        // button 2 pressed, send data without CRC
            Led1_On();
            bc7215_set_tx();        // switch BC7215 to TX mode
            HAL_Delay(2);

            // load format in case the internal format information was over written
            // by received IR signal. If we are staying in TX mode, then reload the
            // format information is not necessary.
            bc7215_load_format(&NECFormat);

            // sending binary data
            bc7215_send_raw(BinData, sizeof(BinData));
            while (!bc7215_cmd_completed())
                ;

            HAL_Delay(100);        // should wait at least 40ms before another transmitting

            // sending text data, no need to load format information again
            bc7215_send_raw(TxtData, sizeof(TxtData) - 1);
            while (!bc7215_cmd_completed())
                ;

            Led1_Off();
            bc7215_set_rx();        // turn back to receiving mode, waiting for incoming data
            HAL_Delay(20);
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
                HAL_Delay(200);
                Led2_Off();
            }
            else        // CRC check failed, flash LED twice
            {
                Led3_On();
                HAL_Delay(200);
                Led3_Off();
            }
        }
        Counter++;
        HAL_Delay(100);
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
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
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

    /*Configure GPIO pin : PC13 */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : PA0 PA1 */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : PA2 PA3 */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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
    HAL_GPIO_WritePin(MOD_PIN_PORT, MOD_PIN_NUM, GPIO_PIN_SET);
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
    ButtonMap = GPIOA->IDR & 0x03;
    if (ButtonMap == 0x03)        // if no button is pressed
    {
        return 0;
    }
    else
    {
        while ((GPIOA->IDR & 0x0003) != 0x0003)        // if buttons are not released
        {
            if ((GPIOA->IDR & 0x0003) == 0x0000)
            {
                ButtonMap = 0x00;
            }
        }
        return ButtonMap ^ 0x03;
    }
}

void Led1_On(void)
{
    HAL_GPIO_WritePin(LED1_PIN_PORT, LED1_PIN_NUM, GPIO_PIN_RESET);        // LED on
}

void Led1_Off(void)
{
    HAL_GPIO_WritePin(LED1_PIN_PORT, LED1_PIN_NUM, GPIO_PIN_SET);        // LED off
}

void Led2_On(void)
{
    HAL_GPIO_WritePin(LED2_PIN_PORT, LED2_PIN_NUM, GPIO_PIN_RESET);        // LED on
}

void Led2_Off(void)
{
    HAL_GPIO_WritePin(LED2_PIN_PORT, LED2_PIN_NUM, GPIO_PIN_SET);        // LED off
}

void Led3_On(void)
{
    HAL_GPIO_WritePin(LED3_PIN_PORT, LED3_PIN_NUM, GPIO_PIN_RESET);        // LED on
}

void Led3_Off(void)
{
    HAL_GPIO_WritePin(LED3_PIN_PORT, LED3_PIN_NUM, GPIO_PIN_SET);        // LED off
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
    while (1) { }
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
