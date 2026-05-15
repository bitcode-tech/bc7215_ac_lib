/*
 * BC7215 Example Application :
 *  Programmable 4-key remote controller
 *
 * Target platform: STC8G1K08
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
 *   Library file bc7215_lib.c has been modified (adding __idata key word
 *   in variable declaration) to work around the limitation of the SDCC
 *   compiler.
 */

#include "./bc7215_lib/bc7215_lib.h"
#include "stc8g.h"

#define RXD  P3_0
#define TXD  P3_1
#define MOD  P3_2
#define BUSY P3_3
#define LED  P5_5
#define S1   P3_4
#define S2   P3_5
#define S3   P3_6
#define S4   P3_7

#define TIMER0_RELOAD 64614        //(65536-(1*1000/(1000000/11059200)/12)) , timer0 reload period 1ms

__xdata bc7215DataMaxPkt_t IRData[4];        // For simplicity, we use 4 max sized array to save data
                                             // instead of variable length
__xdata bc7215FormatPkt_t IRFormat[4];
// ***NOTE***: '__xdata' key word was added due to compiler limitation of SDCC for MCS51
//             May not be necessary if other C comipler is used.

uint8_t  RxByte;
uint8_t  WorkingMode;
uint8_t  KeyValue;
uint8_t  Counter;
uint16_t DownCounter;

/* Private function prototypes -----------------------------------------------*/
void Chip_Init(void);
void GPIO_Init(void);
void UART_Init(void);
void Timer_Init(void);
void Interrupt_Init(void);
void delay_ms(uint16_t Ms);

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
    bc7215_config_uart_send_byte(UART_send_1_byte);
    bc7215_config_mod_set_high_funct(set_MOD_high);
    bc7215_config_mod_set_low_funct(set_MOD_low);
    bc7215_config_read_mod_funct(read_MOD);
    bc7215_config_read_busy_funct(read_BUSY);

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

    P3PU = 0xFF;
    P5PU = 0xFF;

    P_SW2 = 0x80;        // set clock source, set bit7 of P_SW2 to 1 to access SFR in XDATA
    CKSEL = 0x00;        // use internal high speed RC (default 22MHz)
    CLKDIV = 2;          // SYSCLK = MCLK/2 = 11.0592MHz
    P_SW2 = 0x00;
    AUXR = 0x00;        // set timer 0，1，2 to use 12T mode, enable internal EXTRAM
}

// initialize UART I/O related
void GPIO_Init(void)
{
    S1 = 1;
    S2 = 1;
    S3 = 1;
    S4 = 1;
    BUSY = 1;        // input mode
}

// set UART baud rate
void UART_Init(void)
{
    PCON = 0x80;        // double baud rate, no frame error detection
    SCON = 0xD8;        // serial mode 3, 9bit, 9th bit set as 1 to meet 2 stop bit requirement of BC7215
    TH1 = 0xFD;         // set baud rate to 19200 @11.0592MHz
    TR1 = 1;            // start Timer1
}

// timer setup
void Timer_Init(void)
{
    TMOD = 0x21;        // timer0 working at mode1 (16bit), timer1 working at mode2 (auto reload)
    TH0 = TIMER0_RELOAD / 0x100;
    TL0 = TIMER0_RELOAD & 0xFF;
    TR0 = 1;
}

void Interrupt_Init(void)
{
    ES = 1;         // enable UART interrupt
    ET0 = 1;        // enable Timer0 interrupt
    EA = 1;         // enable global interrupt
}

void UART_INT_dis(void)
{
    ES = 0;        // disable UART interrupt
}

void UART_INT_en(void)
{
    ES = 1;        // enable UART interrupt
}

uint8_t read_MOD(void)        // function to read MOD pin
{
    return MOD;
}

void set_MOD_high(void) { MOD = 1; }

void set_MOD_low(void) { MOD = 0; }

uint8_t read_BUSY(void) { return BUSY; }

void UART_send_1_byte(uint8_t data)
{
    SBUF = data;
    while (!TI)
        ;
    TI = 0;
    return;
}

// UART interrupt ISR
void UART_ISR(void) __interrupt(4)
{
    if (RI)        // if it's RX interrupt
    {
        RI = 0;
        bc7215_process_uart_data(SBUF);
    }
}

// Timer0 interrupt ISR
void TIMER0_ISR(void) __interrupt(1)
{
    TH0 = TIMER0_RELOAD / 0x100;        // reload
    TL0 = TIMER0_RELOAD & 0xFF;
    if (0 != DownCounter)        // decrease DownCounter each time (1ms)
    {
        --DownCounter;
    }
}

void delay_ms(uint16_t Ms)
{
    DownCounter = Ms;
    while (DownCounter != 0)
        ;
}

uint8_t Read_Keypad(void)
{
    uint8_t Value;
    Value = 0;
    if ((S1 == 0) || (S2 == 0) || (S3 == 0) || (S4 == 0))
    {
        if (S1 == 0)
        {
            Value = 1;
        }
        else if (S2 == 0)
        {
            Value = 2;
        }
        else if (S3 == 0)
        {
            Value = 4;
        }
        else if (S4 == 0)
        {
            Value = 8;
        }
        while ((S1 == 0) || (S2 == 0) || (S3 == 0) || (S4 == 0))
        {
            if ((S1 == 0) && (S2 == 0))
            {
                Value = 3;
            }
        }
    }
    return Value;
}

void Led_On(void)
{
    LED = 0;        // SET LED PIN LOW (ON)
}

void Led_Off(void)
{
    LED = 1;        // SET LED PIN HIGH (OFF)
}
