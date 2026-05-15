/*
 * BC7215 Example Application :
 *  Infrared Data Communication
 *
 * Target platform: STC8K1G08
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
 *   Library file bc7215_lib.c has been changed (adding key word in variable
 *   declaration) to make the SDCC compiler work correctly.
 *
 */

#include "./bc7215_lib/bc7215_lib.h"
#include "stc8g.h"

#define RXD  P3_0
#define TXD  P3_1
#define MOD  P3_2
#define BUSY P3_3
#define LED1 P1_0 
#define LED2 P1_1
#define LED3 P1_2
#define S1   P5_4
#define S2   P5_5 

#define TIMER0_RELOAD 64614        //(65536-(1*1000/(1000000/11059200)/12)) , timer0 reload period 1ms

const bc7215FormatPkt_t NECFormat = { { { 0x34 } },
    { 0x14, 0x5D, 0x0D, 0x5D, 0x14, 0x3D, 0x3D, 0x1D, 0x1C, 0x9C, 0x62, 0xA0, 0x29, 0xB2, 0x99, 0x44, 0x00, 0x00, 0xC2,
        0x36, 0x9F, 0xF7, 0xFA, 0xB8, 0xE2, 0x9A, 0xA3, 0x26, 0xEA, 0x90, 0x87, 0x30 } };

const uint8_t BinData[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20 };
const char    TxtData[] = { "Hello World" };

__idata uint8_t BufData[32];
// ***NOTE***: '__idata' key word was added due to compiler limitation of SDCC for MCS51
//             May not be necessary if other C comipler is used.
uint8_t         crc;

uint8_t  i, Len;
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
void    Led1_On(void);
void    Led1_Off(void);
void    Led2_On(void);
void    Led2_Off(void);
void    Led3_On(void);
void    Led3_Off(void);

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

    bc7215_set_rx();        // bc7215 set to transmitting mode
    delay_ms(20);

    Led1_Off();
    Led2_Off();
    Led3_Off();

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
            // User must make sure sizeof(TxtData) is not larger than sizeof(BufData)
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
            bc7215_send_raw(TxtData, sizeof(TxtData));
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
    P3PU = 0XFF;

    P_SW2 = 0x80;        // set clock source, set bit7 of P_SW2 to 1 to access SFR in XDATA
    CKSEL = 0x00;        // use internal high speed RC (default 22MHz)
    CLKDIV = 2;          // SYSCLK = MCLK/2 = 11.0592MHz
    P_SW2 = 0x00;
    AUXR = 0x00;        // set timer 0，1，2 to use 12T mode, enable internal EXTRAM
}

// initialize UART I/O related
void GPIO_Init(void)
{
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
    if ((S1 == 0) || (S2 == 0))
    {
        if (S1 == 0)
        {
            Value = 1;
        }
        else if (S2 == 0)
        {
            Value = 2;
        }
        while ((S1 == 0) || (S2 == 0))
        {
            //	   	  if ((S1==0) && (S2==0))
            //	   	  {
            //	   	  	 Value = 3;
            //	   	  }
        }
    }
    return Value;
}

void Led1_On(void)
{
    LED1 = 0;        // SET LED PIN LOW (ON)
}

void Led1_Off(void)
{
    LED1 = 1;        // SET LED PIN HIGH (OFF)
}

void Led2_On(void)
{
    LED2 = 0;        // SET LED PIN LOW (ON)
}

void Led2_Off(void)
{
    LED2 = 1;        // SET LED PIN HIGH (OFF)
}

void Led3_On(void)
{
    LED3 = 0;        // SET LED PIN LOW (ON)
}

void Led3_Off(void)
{
    LED3 = 1;        // SET LED PIN HIGH (OFF)
}
