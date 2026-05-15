/*
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

#include "stm32f10x.h"
#include "bc7215_lib.h"

uint8_t IRRam[50];		// this is memory pool for IR data packets
bc7215DataVarPkt_t* IRData[2];		// 2 pointers for saved IR data for each controlled switch
bc7215DataVarPkt_t* RcvdData;		// This is the received IR data

uint8_t Sig;
uint8_t RxByte;
uint8_t WorkingMode;
uint8_t KeyValue;
uint8_t Counter;
uint32_t DownCounter;
uint16_t Len;

/* USER CODE END PV */

#define MOD_PIN_PORT		GPIOB
#define MOD_PIN_NUM			15
#define BUSY_PIN_PORT		GPIOB
#define BUSY_PIN_NUM		14
#define LED_PIN_PORT		GPIOC
#define LED_PIN_NUM			13
#define SW1_PIN_PORT		GPIOA
#define SW1_PIN_NUM			3
#define SW2_PIN_PORT		GPIOA
#define SW2_PIN_NUM			2

/* Private function prototypes -----------------------------------------------*/
void Clock_Config(void);
void GPIO_Init(void);
void UART_Init(void);
void NVIC_Config(void);		// Config NVIC
void delay_ms(uint16_t Ms);

// In this application, BC7215 is hardwared to receiving mode,
// so only these 3 functions are needed.
uint8_t read_MOD(void);			// function to read MOD pin
void UART_INT_en(void);			// function to enable UART RX interrupt
void UART_INT_dis(void);		// function to disable UART RX interrupt

uint8_t Read_Keypad(void);
void Led_On(void);
void Led_Off(void);
void Toggle_Sw1(void);
void Toggle_Sw2(void);


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
  NVIC_Config();	// enable interrupt

  // bc7215 related init
  bc7215_config_uart_int_en_funct(UART_INT_en);
  bc7215_config_uart_int_dis_funct(UART_INT_dis);
  bc7215_config_read_mod_funct(read_MOD);

  
  Led_Off();

  IRData[0] = (bc7215DataVarPkt_t*)&IRRam[0];	// store IRData[0] at the beginning of IRRam;
  IRData[1] = IRData[0]+1;	// set IRData[1] to the address follow IRData[0]
  RcvdData = IRData[1]+1;

  while (1)
  {
	  KeyValue = Read_Keypad();		// read if there is key press
	  switch (WorkingMode)
	  {
	  case 0:		// WorkingMode 0 is normal mode
		  if ((Counter&0x1f) == 0)		// flash LED every 3200ms to show it's in working mode
		  {
			  Led_On();
		  }
		  else
		  {
			  Led_Off();
		  }

		  switch (KeyValue)
		  {
		  case 0x01:	// button 1 pressed
			  Led_On();		// turn on LED to indicate it's in setting mode
			  WorkingMode = 1;	// Mode1 is to set Sw1
			  break;
		  case 0x02:	// button 2 pressed
			  Led_On();		// turn on LED to indicate it's in setting mode
			  WorkingMode = 2;	// Mode2 is to set Sw2
			  break;
		  default:
			  break;
		  }

		  if (bc7215_data_ready())		// if remote data is received
		  {
		  	Len = bc7215_dpkt_size();
		  	if (bc7215_cal_size(IRData[0])+bc7215_cal_size(IRData[1])+Len < sizeof(IRRam))	// if data will not overflow the IR RAM
		  	{
			  	Sig = bc7215_get_data(RcvdData); // read data and clear status
			    // some PPM remotes such as RC5 and RC6 have a toggle bit in data, which will toggle each time
			    // the button is pressed. To make comparasion easier, we mask these bits if the signal is PPM
			    if ((Sig & 0x30) != 0x30) 	// if TP1:TP0 != 11 (PPM signal), mask possible toggle bits in RC5 and RC6
			    {
			  	  RcvdData->data[0] |= 0x28;		// mask bit5(RC5) and bit3(RC6)
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
						bc7215_clr_data();	// clear any received data packet
						delay_ms(200);
					} while (bc7215_data_ready());
		  		}
				else if ((RcvdData->bitLen == IRData[1]->bitLen) && (bc7215_compare_dpkt(Sig, RcvdData, IRData[1]) == 1))
		  		// if received IR is same as 2nd stored data packet
		  		{
					Toggle_Sw2();

					// some remote such as RC5 repeat sending while the button is pressed,
					// we only leave when there is no IR data received in 0.2s
					// to prevent the switch being toggled by repeating signals.
					do
					{
						bc7215_clr_data();	// clear any received data packet
						delay_ms(200);
					} while (bc7215_data_ready());
		  		}
			}
			else
			{
				bc7215_clr_data();
			}
		  }

		  break;
	  case 1:		// WorkingMode 1 is Sw1 setting mode
		  if (bc7215_data_ready())
		  {
		  	  Len = bc7215_dpkt_size();
		  	  if (Len+bc7215_cal_size(IRData[1]) < sizeof(IRRam))
		  	  {
		  	  	  bc7215_copy_dpkt((bc7215DataVarPkt_t*)&IRRam[Len], IRData[1]);	// move IRData[1] to get space for IRData[0]
		  	  	  IRData[1] = (bc7215DataVarPkt_t*)&IRRam[Len];
		  		  Sig = bc7215_get_data(IRData[0]);
				  // save data in IRData[0], in real product the data should be saved in EEPROM
				  
				  // some PPM remotes such as RC5 and RC6 have a toggle bit in data, which will toggle each time
				  // the button is pressed. To make comparasion easier, we mask these bits if the signal is PPM
				  if ((Sig & 0x30) != 0x30) 	// if TP1:TP0 != 11 (PPM signal), mask possible toggle bits in RC5 and RC6
				  {
				  	IRData[0]->data[0] |= 0x28;		// mask bit5(RC5) and bit3(RC6)
				  }
			  	
				  Led_Off();

					// some remote such as RC5 repeat sending while the button is pressed,
					// we only leave when there is no IR data received in 0.2s
					// to prevent the switch being toggled by repeating signals.
					do
					{
						bc7215_clr_data();	// clear data
						delay_ms(200);
					} while (bc7215_data_ready());	// if there is no new data in 200ms
					RcvdData = (bc7215DataVarPkt_t*)((uint8_t*)IRData[1]+bc7215_cal_size(IRData[1]));
				    WorkingMode = 0;	// switch back to normal mode
			  }
			  else
			  {
			  	bc7215_clr_data();		// if data will overflow, discard received data
			  }
		  }
		  break;
	  case 2:		// WorkingMode 2 is Sw2 setting mode
		  if (bc7215_data_ready())
		  {
		  	  Len = bc7215_dpkt_size();
		  	  if (bc7215_cal_size(IRData[0])+Len < sizeof(IRRam))	// if data will not overflow the IR RAM
		  	  {
			  	  Sig = bc7215_get_data(IRData[1]);
				  // save data in IRData[1], in real product the data should be saved in EEPROM

				  // some PPM remotes such as RC5 and RC6 have a toggle bit in data, which will toggle each time
				  // the button is pressed. To make comparasion easier, we mask these bits if the signal is PPM
				  if ((Sig & 0x30) != 0x30) 	// if TP1:TP0 != 11 (PPM signal), mask possible toggle bits in RC5 and RC6
				  {
				  	IRData[1]->data[0] |= 0x28;		// mask bit5(RC5) and bit3(RC6) of first data byte
				  }

				  Led_Off();

					// some remote such as RC5 repeat sending while the button is pressed,
					// we only leave when there is no IR data received in 0.2s
					// to prevent the switch being toggled by repeating signals.
					do
					{
						bc7215_clr_data();	// clear data
						delay_ms(200);
					} while (bc7215_data_ready());	// if there is no new data in 200ms
					RcvdData = (bc7215DataVarPkt_t*)((uint8_t*)IRData[1]+Len);
				  WorkingMode = 0;	// switch back to normal mode
			  }
			  else
			  {
			  	bc7215_clr_data();		// if data will overflow, discard received data
			  }
		  }
		  break;
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
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN \
				| RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;
    SysTick_Config(SystemCoreClock / 1000);	// set SysTick period to 1ms
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void UART_Init(void)
{
  USART1->BRR = SystemCoreClock/19200;	// set baud rate to 19200
    USART1->CR2 = USART_CR2_STOP_1;		// set to 2 stop bits
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

    GPIOA->CRL = 0x44442288; // PA0-PA3 set as input with pullup(key pad input)
    GPIOA->CRH = 0x444448A4; // PA10(USART1 RX) set as input, A9(UART1 TX) set as output
    GPIOA->ODR = 0x0000040F; // RX(PA10) and PA0-PA3 set pull-up

	//GPIOB->CRH = 0x24444444;  // PB15 set as output(MOD), PB14 set as input (BUSY)

	GPIOC->CRH = 0x44244444; // PC13 set as output (LED)
}


uint8_t read_MOD(void)			// function to read MOD pin
{
	return 1;
}


void UART_INT_en(void)			// function to enable UART RX interrupt
{
	NVIC_EnableIRQ(USART1_IRQn);
}

void UART_INT_dis(void)		// function to disable UART RX interrupt
{
	NVIC_DisableIRQ(USART1_IRQn);
}


void NVIC_Config(void)		// Config NVIC
{
    NVIC_EnableIRQ(USART1_IRQn);         // Enable interrupt
}


void USART1_IRQHandler(void)
{
    if (USART1->SR & USART_SR_RXNE)    //if the	interrupt source is RXNE                            
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
	uint8_t	ButtonMap;
	if ((GPIOA->IDR&0x03) == 0x03)		// if no button is pressed
	{
		return 0;
	}
	else
	{
		ButtonMap = (~GPIOA->IDR)&0x03;
		while ((GPIOA->IDR&0x0003) != 0x0003);	// wait for button to be released
		return ButtonMap;
	}
}


void Led_On(void)
{
	LED_PIN_PORT->BRR = 1UL<<LED_PIN_NUM;	// SET LED PIN LOW (ON)
}

void Led_Off(void)
{
	LED_PIN_PORT->BSRR = 1UL<<LED_PIN_NUM;	// SET LED PIN HIGH (OFF)
}

void Toggle_Sw1(void)
{
	if (SW1_PIN_PORT->IDR & (1UL<<SW1_PIN_NUM))		// if output is high
	{
		SW1_PIN_PORT->BRR = 1UL<<SW1_PIN_NUM;	// SET output LOW
	}
	else
	{
		SW1_PIN_PORT->BSRR = 1UL<<SW1_PIN_NUM;	// SET output PIN HIGH
	}
}					

void Toggle_Sw2(void)
{
	if (SW2_PIN_PORT->IDR & (1UL<<SW2_PIN_NUM))		// if output is high
	{
		SW2_PIN_PORT->BRR = 1UL<<SW2_PIN_NUM;	// SET output LOW
	}
	else
	{
		SW2_PIN_PORT->BSRR = 1UL<<SW2_PIN_NUM;	// SET output PIN HIGH
	}
}					
