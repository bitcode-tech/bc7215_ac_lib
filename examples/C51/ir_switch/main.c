/*
 * BC7215 Example Application :
 *  IR remote control power switch
 *
 * Target platform: STC8G1K08
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

#include "stc8g.h"
#include "./bc7215_lib/bc7215_lib.h"


/*       BC7215 connections       */
/*   P3.0 (MCU RX) --> BC7215 TX  */

#define RXD P3_0
#define LED P3_1
#define SW1 P3_3
#define SW2 P3_2
#define S1  P5_4
#define S2  P5_5

#define TIMER0_RELOAD 56320		//(65536-(10*1000/(1000000/11059200)/12)) , timer0 reload period 10ms


__idata uint8_t IRRam[48];		// this is the memory pool for IR data packets
bc7215DataVarPkt_t* IRData[2];		// 2 pointers for saved IR data for each controlled switch
bc7215DataVarPkt_t* RcvdData;		// This is the received IR data

uint8_t Sig;
uint8_t WorkingMode;
uint8_t KeyValue;
uint8_t Counter;
uint16_t DownCounter;
uint16_t Len;


/* Private function prototypes -----------------------------------------------*/
void Chip_Init(void);
void GPIO_Init(void);
void UART_Init(void);
void Timer_Init(void);
void Interrupt_Init(void);
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



int main(void)
{
  // System Init.
  Chip_Init();
  GPIO_Init();
  UART_Init();
  Timer_Init();
  Interrupt_Init();

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

// Initialize STC8G chip
void Chip_Init(void)
{
    /*	M1:M0 settings:
		0:0  traditional mode
		0:1  hi current mode
		1:0  hi-impedence
		1:1  open drain  */

    // set I/O mode, all set to lagacy MSC51 mode
    P0M0 = 0;
    P0M1 = 0;
    P1M0 = 0;
    P1M1 = 0;
    P2M0 = 0;
    P2M1 = 0;
    P3M0 = 0;
    P3M1 = 0;
    P4M0 = 0;
    P4M1 = 0;
    P5M0 = 0;
    P5M1 = 0;
    
    P5PU = 0xFF;

    P_SW2  = 0x80; // set clock source, set bit7 of P_SW2 to 1 to access SFR in XDATA
    CKSEL  = 0x00; // use internal high speed RC (default 22MHz)
    CLKDIV = 2;    // SYSCLK = MCLK/2 = 11.0592MHz
    P_SW2  = 0x00; 
    AUXR   = 0x00;    // set timer 0，1，2 to use 12T mode, enable internal EXTRAM
}

// initialize UART I/O related
void GPIO_Init(void)
{
    S1 = 1;
    S2 = 1;
    SW1 = 0;
    SW2 = 0;
}

// set UART baud rate
void UART_Init(void)
{
    PCON = 0x80;    // double baud rate, no frame error detection
    SCON = 0xD8; // serial mode 3, 9bit, 9th bit set as 1 to meet 2 stop bit requirement of BC7215
    TH1  = 0xFD; // set baud rate to 19200 @11.0592MHz
    TR1  = 1;    // start Timer1 
}

// timer setup
void Timer_Init(void)
{
    TMOD = 0x21; // timer0 working at mode1 (16bit), timer1 working at mode2 (auto reload)
    TH0  = TIMER0_RELOAD / 0x100;
    TL0  = TIMER0_RELOAD & 0xFF;
    TR0  = 1;
}

void Interrupt_Init(void)
{
    ES  = 1;
    ET0 = 1;
    EA  = 1;
}

void UART_INT_dis(void)
{
    ES = 0; // disable UART interrupt
}

void UART_INT_en(void)
{
    ES = 1; // enable UART interrupt
}


uint8_t read_MOD(void)			// function to read MOD pin
{
	return 1;
}


// UART interrupt ISR
void UART_ISR(void) __interrupt(4)
{
    if (RI) // if it's RX interrupt
    {
        RI = 0;
        bc7215_process_uart_data(SBUF);
    }
    if (TI) // if it's TX interrupt
    {
        TI = 0;
    }
}

// Timer0 interrupt ISR
void TIMER0_ISR(void) __interrupt(1)
{
    TH0 = TIMER0_RELOAD / 0x100; // reload
    TL0 = TIMER0_RELOAD & 0xFF;
    if (0 != DownCounter) // decrease DownCounter each time (10ms)
    {
        --DownCounter;
    }
}

void delay_ms(uint16_t Ms)
{
//	for (uint16_t j=0; j<Ms; j++)
//	{
//		for (uint16_t i=0; i<0x200; i++);
//	}
    DownCounter = (Ms+5)/10;	// convert to nearist value by 10ms
    while (DownCounter != 0)
        ;
}

uint8_t Read_Keypad(void)
{
   if (S1 == 0)
   { 
      while (S1 == 0);
      return 1;
   }
   if (S2 == 0)
   {
   	  while (S2 == 0);
      return 2;
   }
   return 0;
}

		

void Led_On(void)
{
	LED = 0;	// SET LED PIN LOW (ON)
}

void Led_Off(void)
{
	LED = 1;	// SET LED PIN HIGH (OFF)
}

void Toggle_Sw1(void)
{
	if (SW1)		// if output is high
	{
		SW1 = 0;	// SET output LOW
	}
	else
	{
		SW1 = 1;	// SET output PIN HIGH
	}
}					

void Toggle_Sw2(void)
{
	if (SW2)		// if output is high
	{
		SW2 = 0;	// SET output LOW
	}
	else
	{
		SW2 = 1;	// SET output PIN HIGH
	}
}					
