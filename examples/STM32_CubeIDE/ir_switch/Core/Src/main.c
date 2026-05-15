/* USER CODE BEGIN Header */
/**
 * BC7215 Example Application :
 *  IR remote control power switch
 *
 * Target platform: STM32F103C8
 *
 * Description:
 *   This application demostrates how to us BC7215 and its library to 
 *   implement an IR remote controlled switch that can 'pair' with any
 *   remote controller. The system has 2 buttons and 1 LED, and 2 relays
 *   driven by the micro.
 *   User and pair the system with any key (usually a spare key) on any
 *   remote controller and then use that key to controll the switch.
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
#define LED_PIN_PORT GPIOC
#define LED_PIN_NUM  GPIO_PIN_13
#define S1_PIN_PORT  GPIOA
#define S1_PIN_NUM   GPIO_PIN_0
#define S2_PIN_PORT  GPIOA
#define S2_PIN_NUM   GPIO_PIN_1
#define SW1_PIN_PORT GPIOA
#define SW1_PIN_NUM  GPIO_PIN_3
#define SW2_PIN_PORT GPIOA
#define SW2_PIN_NUM  GPIO_PIN_2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t             IRRam[50];        // this is memory pool for IR data packets
bc7215DataVarPkt_t* IRData[2];        // 2 pointers for saved IR data for each controlled switch
bc7215DataVarPkt_t* RcvdData;         // This is the received IR data

uint8_t  Sig;
uint8_t  RxByte;
uint8_t  WorkingMode;
uint8_t  KeyValue;
uint8_t  Counter;
uint32_t DownCounter;
uint16_t Len;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void        SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
// functions needed by BC7215 library
// In this application, BC7215 is hardwared to receiving mode,
// so only these 3 functions are needed.
uint8_t read_MOD(void);            // function to read MOD pin
void    UART_INT_en(void);         // function to enable UART RX interrupt
void    UART_INT_dis(void);        // function to disable UART RX interrupt

void    NVIC_Config(void);        // Config NVIC
uint8_t Read_Keypad(void);
void    Led_On(void);
void    Led_Off(void);
void    Toggle_Sw1(void);
void    Toggle_Sw2(void);
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
    bc7215_config_read_mod_funct(read_MOD);
    Led_Off();

    NVIC_Config();
    HAL_UART_Receive_IT(&huart1, &RxByte, 1);

    IRData[0] = (bc7215DataVarPkt_t*)&IRRam[0];        // store IRData[0] at the beginning of IRRam;
    IRData[1] = IRData[0] + 1;                         // set IRData[1] to the address follow IRData[0]
    RcvdData = IRData[1] + 1;

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        KeyValue = Read_Keypad();        // read if there is key press
        switch (WorkingMode)
        {
        case 0:                               // WorkingMode 0 is normal mode
            if ((Counter & 0x1f) == 0)        // flash LED every 3200ms to show it's in working mode
            {
                Led_On();
            }
            else
            {
                Led_Off();
            }

            switch (KeyValue)
            {
            case 0x01:                  // button 1 pressed
                Led_On();               // turn on LED to indicate it's in setting mode
                WorkingMode = 1;        // Mode1 is to set Sw1
                break;
            case 0x02:                  // button 2 pressed
                Led_On();               // turn on LED to indicate it's in setting mode
                WorkingMode = 2;        // Mode2 is to set Sw2
                break;
            default:
                break;
            }

            if (bc7215_data_ready())        // if remote data is received
            {
                Len = bc7215_dpkt_size();
                if (bc7215_cal_size(IRData[0]) + bc7215_cal_size(IRData[1]) + Len
                    < sizeof(IRRam))        // if data will not overflow the IR RAM
                {
                    Sig = bc7215_get_data(RcvdData);        // read data and clear status
                    // some PPM remotes such as RC5 and RC6 have a toggle bit in data, which will toggle each time
                    // the button is pressed. To make comparasion easier, we mask these bits if the signal is PPM
                    if ((Sig & 0x30)
                        != 0x30)        // if TP1:TP0 != 11 (PPM signal), mask possible toggle bits in RC5 and RC6
                    {
                        RcvdData->data[0] |= 0x28;        // mask bit5(RC5) and bit3(RC6)
                    }
                    if ((RcvdData->bitLen == IRData[0]->bitLen) && (bc7215_compare_dpkt(Sig, RcvdData, IRData[0]) == 1))
                    // if received IR is same as 1st stored data packet
                    {
                        Toggle_Sw1();

                        // some remote such as RC5 repeat sending while the button is pressed,
                        // we only leave when there is no IR data received in 0.2s
                        // to prevent the switch being toggled by repeating signals.
                        do
                        {
                            bc7215_clr_data();        // clear any received data packet
                            HAL_Delay(200);
                        } while (bc7215_data_ready());
                    }
                    else if ((RcvdData->bitLen == IRData[1]->bitLen)
                        && (bc7215_compare_dpkt(Sig, RcvdData, IRData[1]) == 1))
                    // if received IR is same as 2nd stored data packet
                    {
                        Toggle_Sw2();

                        // some remote such as RC5 repeat sending while the button is pressed,
                        // we only leave when there is no IR data received in 0.2s
                        // to prevent the switch being toggled by repeating signals.
                        do
                        {
                            bc7215_clr_data();        // clear any received data packet
                            HAL_Delay(200);
                        } while (bc7215_data_ready());
                    }
                }
                else
                {
                    bc7215_clr_data();
                }
            }

            break;
        case 1:        // WorkingMode 1 is Sw1 setting mode
            if (bc7215_data_ready())
            {
                Len = bc7215_dpkt_size();
                if (Len + bc7215_cal_size(IRData[1]) < sizeof(IRRam))
                {
                    bc7215_copy_dpkt((bc7215DataVarPkt_t*)&IRRam[Len],
                        IRData[1]);        // move IRData[1] to get space for IRData[0]
                    IRData[1] = (bc7215DataVarPkt_t*)&IRRam[Len];
                    Sig = bc7215_get_data(IRData[0]);
                    // save data in IRData[0], in real product the data should be saved in EEPROM

                    // some PPM remotes such as RC5 and RC6 have a toggle bit in data, which will toggle each time
                    // the button is pressed. To make comparasion easier, we mask these bits if the signal is PPM
                    if ((Sig & 0x30)
                        != 0x30)        // if TP1:TP0 != 11 (PPM signal), mask possible toggle bits in RC5 and RC6
                    {
                        IRData[0]->data[0] |= 0x28;        // mask bit5(RC5) and bit3(RC6)
                    }

                    Led_Off();

                    // some remote such as RC5 repeat sending while the button is pressed,
                    // we only leave when there is no IR data received in 0.2s
                    // to prevent the switch being toggled by repeating signals.
                    do
                    {
                        bc7215_clr_data();        // clear data
                        HAL_Delay(200);
                    } while (bc7215_data_ready());        // if there is no new data in 200ms
                    RcvdData = (bc7215DataVarPkt_t*)((uint8_t*)IRData[1] + bc7215_cal_size(IRData[1]));
                    WorkingMode = 0;        // switch back to normal mode
                }
                else
                {
                    bc7215_clr_data();        // if data will overflow, discard received data
                }
            }
            break;
        case 2:        // WorkingMode 2 is Sw2 setting mode
            if (bc7215_data_ready())
            {
                Len = bc7215_dpkt_size();
                if (bc7215_cal_size(IRData[0]) + Len < sizeof(IRRam))        // if data will not overflow the IR RAM
                {
                    Sig = bc7215_get_data(IRData[1]);
                    // save data in IRData[1], in real product the data should be saved in EEPROM

                    // some PPM remotes such as RC5 and RC6 have a toggle bit in data, which will toggle each time
                    // the button is pressed. To make comparasion easier, we mask these bits if the signal is PPM
                    if ((Sig & 0x30)
                        != 0x30)        // if TP1:TP0 != 11 (PPM signal), mask possible toggle bits in RC5 and RC6
                    {
                        IRData[1]->data[0] |= 0x28;        // mask bit5(RC5) and bit3(RC6) of first data byte
                    }

                    Led_Off();

                    // some remote such as RC5 repeat sending while the button is pressed,
                    // we only leave when there is no IR data received in 0.2s
                    // to prevent the switch being toggled by repeating signals.
                    do
                    {
                        bc7215_clr_data();        // clear data
                        HAL_Delay(200);
                    } while (bc7215_data_ready());        // if there is no new data in 200ms
                    RcvdData = (bc7215DataVarPkt_t*)((uint8_t*)IRData[1] + Len);
                    WorkingMode = 0;        // switch back to normal mode
                }
                else
                {
                    bc7215_clr_data();        // if data will overflow, discard received data
                }
            }
            break;
        default:
            break;
        }
        Counter++;
        HAL_Delay(100);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_RESET);

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
}

/* USER CODE BEGIN 4 */
uint8_t read_MOD(void)        // function to read MOD pin
{
    return 1;
}

void UART_INT_en(void)        // function to enable UART RX interrupt
{
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void UART_INT_dis(void)        // function to disable UART RX interrupt
{
    HAL_NVIC_DisableIRQ(USART1_IRQn);
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

void Led_On(void)
{
    HAL_GPIO_WritePin(LED_PIN_PORT, LED_PIN_NUM, GPIO_PIN_RESET);        // LED on
}

void Led_Off(void)
{
    HAL_GPIO_WritePin(LED_PIN_PORT, LED_PIN_NUM, GPIO_PIN_SET);        // LED off
}

void Toggle_Sw1(void)
{
    if (HAL_GPIO_ReadPin(SW1_PIN_PORT, SW1_PIN_NUM))        // if output is high
    {
        HAL_GPIO_WritePin(SW1_PIN_PORT, SW1_PIN_NUM, GPIO_PIN_RESET);        // SET output LOW
    }
    else
    {
        HAL_GPIO_WritePin(SW1_PIN_PORT, SW1_PIN_NUM, GPIO_PIN_SET);        // SET output PIN HIGH
    }
}

void Toggle_Sw2(void)
{
    if (HAL_GPIO_ReadPin(SW2_PIN_PORT, SW2_PIN_NUM))        // if output is high
    {
        HAL_GPIO_WritePin(SW2_PIN_PORT, SW2_PIN_NUM, GPIO_PIN_RESET);        // SET output LOW
    }
    else
    {
        HAL_GPIO_WritePin(SW2_PIN_PORT, SW2_PIN_NUM, GPIO_PIN_SET);        // SET output PIN HIGH
    }
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
