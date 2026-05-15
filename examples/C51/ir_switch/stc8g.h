/*-------------------------------------------------------------------------
   stc8g.h - register declarations for STC8G series (mainly 8pin) modified from stc12.h
   ************************************************************************

   stc12.h - Register Declarations for STC10/11/12 Series
   Based on 8051.h and compiler.h

   Copyright (c) 2012, intron@intron.ac

   This library is free software; you can redistribute it and/or modify it
   under the terms of the GNU General Public License as published by the
   Free Software Foundation; either version 2, or (at your option) any
   later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License 
   along with this library; see the file COPYING. If not, write to the
   Free Software Foundation, 51 Franklin Street, Fifth Floor, Boston,
   MA 02110-1301, USA.

   As a special exception, if you link this library with other files,
   some of which are compiled with SDCC, to produce an executable,
   this library does not by itself cause the resulting executable to
   be covered by the GNU General Public License. This exception does
   not however invalidate any other reasons why the executable file
   might be covered by the GNU General Public License.
-------------------------------------------------------------------------*/

/*
   Brief:

       STC10/11/12 series are 8051-compatible MCU's. The "official" website
   is http://www.stcmcu.com/ (In Chinese Han only), and datasheets in Chinese
   Han and English can be downloaded there.

   Reference:

   1. The "official" C header file (written for another C51 compiler):
        http://www.stcmcu.com/datasheet/stc/STC-AD-PDF/STC_NEW_8051.H
   2. Datasheets for STC12(C/LE)5Axx(S2/AD) series:
      Chinese Han:
        http://www.stcmcu.com/datasheet/stc/STC-AD-PDF/STC12C5A60S2.pdf
      English:
        http://www.stcmcu.com/datasheet/stc/STC-AD-PDF/STC12C5A60S2-english.pdf
   3. Datasheets for STC12(C/LE)52xxAD series:
      Chinese Han:
        http://www.stcmcu.com/datasheet/stc/STC-AD-PDF/STC12C5201AD.pdf
      English:
        http://www.stcmcu.com/datasheet/stc/STC-AD-PDF/STC12C5201AD-english.pdf
   4. Datasheets for STC11/10 series:
      Chinese Han:
        http://www.stcmcu.com/datasheet/stc/STC-AD-PDF/STC11F-10Fxx.pdf
      English:
        http://www.stcmcu.com/datasheet/stc/STC-AD-PDF/STC11F-10Fxx-english.pdf
 */

#ifndef _STC8G_H_
#define _STC8G_H_

#include <8051.h>
#include <compiler.h>

#ifdef REG8051_H
#undef REG8051_H
#endif

/*
 * Auxiliary Register
 *   Bit Mapping: T0x12 T1x12 UART_M0x6 T2R T2_C/T T2x12 EXTRAM SIST2
 *   Reset Value: 0000,0001
 */
SFR(AUXR, 0x8E);

/*
 * Interrupt and clock output Register
 *   Bit Mapping: - EX4 EX3 EX2 - T2CLKO T1CLKO T0CLKO
 *   Reset Value: x000,x000
 */
SFR(INTCLKO, 0x8F);

/*
 * P1 Mode Register 1
 *   Bit Mapping: P17M1 P16M1 P15M1 P14M1 P13M1 P12M1 P11M1 P10M1
 *   Reset Value: 1111,1111
 */
SFR(P1M1, 0x91);

/*
 * P1 Mode Register 0
 *   Bit Mapping: P17M0 P16M0 P15M0 P14MO P13M0 P12M0 P11M0 P10M0
 *   Reset Value: 0000,0000
 */
SFR(P1M0, 0x92);

/*
 * P0 Mode Register 1
 *   Bit Mapping: P07M1 P06M1 P05M1 P04M1 P03M1 P02M1 P01M1 P00M1
 *   Reset Value: 1111,1111
 */
SFR(P0M1, 0x93);

/*
 * P0 Mode Register 0
 *   Bit Mapping: P07M0 P06M0 P05M0 P04MO P03M0 P02M0 P01M0 P00M0
 *   Reset Value: 0000,0000
 */
SFR(P0M0, 0x94);

/*
 * P2 Mode Register 1
 *   Bit Mapping: P27M1 P26M1 P25M1 P24M1 P23M1 P22M1 P21M1 P20M1
 *   Reset Value: 1111,1111
 */
SFR(P2M1, 0x95);

/*
 * P2 Mode Register 0
 *   Bit Mapping: P27M0 P26M0 P25M0 P24MO P23M0 P22M0 P21M0 P20M0
 *   Reset Value: 0000,0000
 */
SFR(P2M0, 0x96);

/*
 * Serial 2 Control Register
 *   Bit Mapping: S2SM0 - S2SM2 S2REN S2TB8 S2RB8 S2T1 S2R1 
 *   Reset Value: 0x00,0000
 */
SFR(S2CON, 0x9A);

/*
 * Serial 2 Buffer Register
 *   Reset Value: 0000,0000
 */
SFR(S2BUF, 0x9B);

/*
 * IRC Band Register
 *   Bit Mapping: - - - - - - - SEL 
 *   Reset Value: xxxx,xxx0
 */
SFR(IRCBAND, 0x9D);

/*
 * Low IRC Trim Register
 *   Bit Mapping: - - - - - - LIRTRIM1 LIRTRIM0
 *   Reset Value: xxxx,xxnn
 */
SFR(LIRTRIM, 0x9E);

/*
 * IRC Trim Register
 *   Bit Mapping: IRTRIM7 IRTRIM6 IRTRIM5 IRTRIM4 IRTRIM3 IRTRIM2 IRTRIM1 IRTRIM0
 *   Reset Value: nnnn,nnnn
 */
SFR(IRTRIM, 0x9F);

/*
 * Bus Speed register
 *   Bit Mapping: RW_S1 RW_S0 - - - - SPEED2 SPEED1 SPEED0
 *   Reset Value: 00xx,x000
 */
SFR(BUS_SPEED, 0xA1);

/*
 * Peripheral Switch register
 *   Bit Mapping: S1_S1 S1_S0 CCP_S1 CCP_S0 SPI_S1 SPI_S0 0 - 
 *   Reset Value: nn00,000x
 */
SFR(P_SW1, 0xA2);

/* Two extended bits in IE */
SBIT(ELVD, 0xA8, 6); /* Enable Low Voltage Detection Interrupt */
SBIT(EADC, 0xA8, 5); /* Enable ADC Interrupt */

/*
 * Serial 1 Slave Address register
 *   Reset Value: 0000,0000
 */
SFR(SADDR, 0xA9);

/*
 * Wakeup Timer Control register Lower Byte
 *   Reset Value: 1111,1111
 */
SFR(WKTCL, 0xAA);

/*
 * Wakeup Timer Control register Higher Byte
 *   Reset Value: 0111,1111
 */
SFR(WKTCH, 0xAB);

/*
 * Auxiliary Interrupt Register
 *   Bit Mapping: - ET4 ET3 ES4 ES3 ET2 ESPI ES2
 *   Reset Value: x000,0000
 */
SFR(IE2, 0xAF);

/*
 * P3 Mode Register 1
 *   Bit Mapping: P37M1 P36M1 P35M1 P34M1 P33M1 P32M1 P31M1 P30M1
 *   Reset Value: 1111,1111
 */
SFR(P3M1, 0xB1);

/*
 * P3 Mode Register 0
 *   Bit Mapping: P37M0 P36M0 P35M0 P34MO P33M0 P32M0 P31M0 P30M0
 *   Reset Value: 0000,0000
 */
SFR(P3M0, 0xB2);

/*
 * Interrupt Priority Register 2
 *   Bit Mapping: PPWM2FD PI2C PCMP PX4 PPWM0FD PPWM0 PSP1 PS2
 *   Reset Value: 0000,0000
 */
SFR(IP2, 0xB5);

/*
 * Interrupt Priority Register 2 High
 *   Bit Mapping: PPWM2FDH PI2CH PCMPH PX4H PPWM0FDH PPWM0H PSP1H PS2H
 *   Reset Value: 0000,0000
 */
SFR(IP2H, 0xB6);

/*
 * Interrupt Priority Register High
 *   Bit Mapping: PCCAH PLVDH PADCH PSH PT1H PX1H PT0H PX0H
 *   Reset Value: 0000,0000
 */
SFR(IPH, 0xB7);

/* Three extended bits in IP */
SBIT(PPCA, 0xB8, 7); /* Interrupt Priority for PCA */
SBIT(PLVD, 0xB8, 6); /* Interrupt Priority for Low Voltage Detection */
SBIT(PADC, 0xB8, 5); /* Interrupt Priority for ADC */

/*
 * Two Extended GPIO Ports: P4 and P5
 * - For DIP-40 and QFN-40 packages, only higher 4 bits of P4 are available.
 * - For PLCC-44 and LQFP-44 packages, only all 8 bits of P4 are available.
 * - For LQFP-48 package, all 8 bits of P4 and lower 4 bits of P5 are
 *   available.
 */
SFR(P4, 0xC0);
    SBIT(P4_0, 0xC0, 0);
    SBIT(P4_1, 0xC0, 1);
    SBIT(P4_2, 0xC0, 2);
    SBIT(P4_3, 0xC0, 3);
    SBIT(P4_4, 0xC0, 4);
    SBIT(P4_5, 0xC0, 5);
    SBIT(P4_6, 0xC0, 6);
    SBIT(P4_7, 0xC0, 7);
SFR(P5, 0xC8); /* Only lower 6 bits */
    SBIT(P5_0, 0xC8, 0);
    SBIT(P5_1, 0xC8, 1);
    SBIT(P5_2, 0xC8, 2);
    SBIT(P5_3, 0xC8, 3);
    SBIT(P5_4, 0xC8, 4);
    SBIT(P5_5, 0xC8, 5);

/* Working Mode Registers for P0, P1, P2, P3, P4 and P5 */
SFR(P4M0, 0xB4);
SFR(P4M1, 0xB3);
SFR(P5M0, 0xCA);
SFR(P5M1, 0xC9);

/* Slave Address Mask for Serial Communication */
SFR(SADEN, 0xB9);

/*
 * Peripheral Switch register 2
 *   Bit Mapping: EAXFR - I2C_S1 I2C_S0 CMPO_S S4_S S3_S S2_S 
 *   Reset Value: 0x00,0000
 */
SFR(P_SW2, 0xBA);

/*
 * Watchdog Timer Control Register
 *   Bit Mapping: WDT_FLAG - EN_WDT CLR_WDT IDLE_WDT PS2 PS1 PS0
 *   Reset Value: 0xn0,0000
 */
SFR(WDT_CONTR, 0xC1);

/*
 * PCA Control Register
 *   Bit Mapping: CF CR - - - CCF2 CCF1 CCF0
 *   Reset Value: 00xx,x000
 */
SFR(CCON, 0xD8);
    SBIT(CF, 0xD8, 7);
    SBIT(CR, 0xD8, 6);
    SBIT(CCF2, 0xD8, 2);
    SBIT(CCF1, 0xD8, 1);
    SBIT(CCF0, 0xD8, 0);

/*
 * PCA Mode Register
 *   Bit Mapping: CIDL - - - CPS2 CPS1 CPS0 ECF
 *   Reset Value: 0xxx,x000
 */
SFR(CMOD, 0xD9);

/* PCA Counter Registers */
SFR(CL, 0xE9); /* Lower 8 bits */
SFR(CH, 0xF9); /* Higher 8 bits */

/*
 * PCA Module 0 PWM Register
 *   Bit Mapping: - ECOM0 CAPP0 CAPN0 MAT0 TOG0 PWM0 ECCF0
 *   Reset Value: x000,0000
 */
SFR(CCAPM0, 0xDA);

/*
 * PCA Module 1 PWM Register
 *   Bit Mapping: - ECOM1 CAPP1 CAPN1 MAT1 TOG1 PWM1 ECCF1
 *   Reset Value: x000,0000
 */
SFR(CCAPM1, 0xDB);

/* PCA Module 0/1 Capture/Comparison Registers */
SFR(CCAP0L, 0xEA);
SFR(CCAP0H, 0xFA);
SFR(CCAP1L, 0xEB);
SFR(CCAP1H, 0xFB);

/*
 * PCA Module 0 PWM Auxiliary Register
 *   Bit Mapping: - - - - - - EPC0H EPC0L
 *   Reset Value: xxxx,xx00
 */
SFR(PCA_PWM0, 0xF2);

/*
 * PCA Module 1 PWM Auxiliary Register
 *   Bit Mapping: - - - - - - EPC1H EPC1L
 *   Reset Value: xxxx,xx00
 */
SFR(PCA_PWM1, 0xF3);

/*
 * ADC Control Register
 *   Bit Mapping: ADC_POWER SPEED1 SPEED0 ADC_FLAG ADC_START CHS2 CHS1 CHS0
 *   Reset Value: 0000,0000
 */
SFR(ADC_CONTR, 0xBC);

/* ADC Converting Result Registers */
SFR(ADC_RES, 0xBD); /* Higher Bits */
SFR(ADC_RESL, 0xBE); /* Lower Bits */

/*
 * SPI Control Register
 *   Bit Mapping: SSIG SPEN DORD MSTR CPOL CPHA SPR1 SPR0
 *   Reset Value: 0000,0100
 */
SFR(SPCTL, 0xCE);

/*
 * SPI Status Register
 *   Bit Mapping: SPIF WCOL - - - - - -
 *   Reset Value: 00xx,xxxx
 */
SFR(SPSTAT, 0xCD);

/* SPI Data Register */
SFR(SPDAT, 0xCF);

/* In-Application-Programming Data Register */
SFR(IAP_DATA, 0xC2);

/* In-Application-Programming Address Registers */
SFR(IAP_ADDRH, 0xC3); /* Higher 8 bits */
SFR(IAP_ADDRL, 0xC4); /* Lower 8 bits */

/*
 * In-Application-Programming Address Registers
 *   Bit Mapping: - - - - - - MS1 MS0
 *   Reset Value: xxxx,xx00
 */
SFR(IAP_CMD, 0xC5);

/* In-Application-Programming Trigger Registers */
SFR(IAP_TRIG, 0xC6);

/*
 * In-Application-Programming Control Register
 *   Bit Mapping: IAPEN SWBS SWRST CFAIL - WT2 WT1 WT0
 *   Reset Value: 0000,x000
 */
SFR(IAP_CONTR, 0xC7);

/*
 * In-Application-Programming Wait Time Registers
 *   Bit Mapping: - - TPS5 TPS4 TPS3 TPS2 TPS1 TPS0
 *   Reset Value: xx00,0000
 */
SFR(IAP_TPS, 0xF5);

/*
 * DPTR Select Registers
 *   Bit Mapping: ID1 ID0 TSL AU1 AU0 - - SEL
 *   Reset Value: 0000,0xx0
 */
SFR(DPS, 0xE3);

/*
 * Second DPTR Lower Byte
 *   Reset Value: 0000,0000
 */
SFR(DPL1, 0xE4);

/*
 * Second DPTR Higher Byte
 *   Reset Value: 0000,0000
 */
SFR(DPH1, 0xE5);

/*
 * Auxiliary INT flag Register
 *   Bit Mapping: - INT4IF INT3IF INT2IF - - - T2IF
 *   Reset Value: x000,xxx0
 */
SFR(AUXINTIF, 0xEF);

/*
 * Reset Configuration Register
 *   Bit Mapping: - ENLVR - P54RST - - LVDS1 LVDS0
 *   Reset Value: xnxn,xxnn
 */
SFR(RSTCFG, 0xFF);


SFRX(CKSEL, 0xFE00);
SFRX(CLKDIV, 0xFE01);
SFRX(HIRCCR, 0xFE02);
SFRX(IRC32KCR, 0xFE04);
SFRX(MCLKOCR, 0xFE05);
SFRX(IRCDB, 0xFE06);
SFRX(P1PU, 0xFE11);
SFRX(P3PU, 0xFE13);
SFRX(P5PU, 0xFE15);
SFRX(P3NCS, 0xFE1B);
SFRX(P5NCS, 0xFE1D);
SFRX(P3SR, 0xFE23);
SFRX(P5SR, 0xFE25);
SFRX(P3DR, 0xFE2B);
SFRX(P5DR, 0xFE2D);
SFRX(I2CCFG, 0xFE80);
SFRX(I2CMSCR, 0xFE81);
SFRX(I2CMSST, 0xFE82);
SFRX(I2CSLCR, 0xFE83);
SFRX(I2CSLST, 0xFE84);
SFRX(I2CSLADR, 0xFE85);
SFRX(I2CTXD, 0xFE86);
SFRX(I2CRXD, 0xFE87);
SFRX(I2CMSAUX, 0xFE88);
SFRX(MD3, 0xFCF0);
SFRX(MD2, 0xFCF1);
SFRX(MD1, 0xFCF2);
SFRX(MD0, 0xFCF3);
SFRX(MD5, 0xFCF4);
SFRX(MD4, 0xFCF5);
SFRX(ARCON, 0xFCF6);
SFRX(OPCON, 0xFCF7);

#endif /* _STC8G_H_ */

