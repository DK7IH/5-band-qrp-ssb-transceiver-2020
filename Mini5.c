///////////////////////////////////////////////////////////////////  
/*     TRX Mini5 5 band with ATMega328, AD9850, Si5351,          */
/*                    ST7735 LCD and MCP4725 DAC                 */
/*                  Bands: 80, 40, 20, 17 and 15m                */
/*            http://radiotransmitter.wordpress.com              */
///////////////////////////////////////////////////////////////////
/*                                                               */
/*  Compiler:         GCC (GNU AVR C-Compiler)                   */
/*  Author:           Peter Rachow  DK7IH                        */
/*  Last update:      2020-05-08                                 */
///////////////////////////////////////////////////////////////////

  ////////////
 // PORTS  //
////////////

//O U T P U T 
//TWI:          PC4, PC5
//relay decoder PD0, PD1, PD2 
//LCD:          PD3, PD4, PD5, PD6, PD7
//AD9850 DDS:   PB2..PB5

//I N P U T
//---
//PC0: ADC0 user keys
//PC1: ADC1: S-Value
//PC2: ADC2 TX Power
//     ADC6 DC Voltage
//     ADC7 TX/RX detect

//PB0, PB1: Rotary encoder

//See: https://radiotransmitter.files.wordpress.com/2020/04/mini5_qrp_ssb_trx_dk7ih_mini5_port_usage-1.png

////////////////////////////////////////////////////////////////////////

//Si5351 oscillators usage
//OSC0 = LO 9MHz +/- sidenband offset
//OSC1 = not used
//OSC2: not used

//EEPROM structure
//Byte 0:  Last Band used
//Byte 1..4 NOT USED
//byte 5:  Last VFO used
//Byte 6:  Last sideband used
//Byte 7:  Last VFO on 80
//Byte 8:  Last VFO on 40
//Byte 9:  Last VFO on 20
//Byte 10: Last VFO on 15
//Byte 11: Last VFO on 20

//Byte 64..67: VFOA on 80
//Byte 68..71: VFOB on 80
//Byte 72..75: VFOA on 40
//Byte 76..79: VFOB on 40
//Byte 80..83: VFOA on 20
//Byte 84..87: VFOB on 20
//Byte 88..91: VFOA on 15
//Byte 92..95: VFOB on 15
//Byte 96..99: VFOA on 10
//Byte 100..103: VFOB on 10
//VFO data offset = 32

//Bytes 128..137: Two bytes for each band TX gain preset defintion
//138: Scan threshold

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

#define OFF_LAST_BAND_USED 0
#define OFF_LAST_VFO_USED 5
#define OFF_LAST_SIDEBAND_USED 6
#define OFF_VFO_DATA 7
#define OFF_FREQ_DATA 64

//Modes and Bands
#define MAXMODES 2
#define MAXBANDS 5

//MENU
#define MENUSTRINGS 6
#define MENUITEMS 5

  //////////////////////
 //  SPI DDS AD9850  //
//////////////////////
#define DDS_PORT PORTB
#define DDS_DDR  DDRB
#define DDS_FQ_UD 2 //green PB0
#define DDS_SDATA 3 //white PB1
#define DDS_W_CLK 4  //blue PB3
#define DDS_RESETPORT PORTB
#define DDS_RESETDDR DDRB
#define DDS_RESETPIN 5

/////////////////////////////////////
//   Global constants & variables
/////////////////////////////////////

//VFO data
int cur_vfo;
long f_vfo[MAXBANDS][2] = {{3650000, 3650000}, 
	                       {7120000, 7120000}, 
	                       {14200000, 14280000}, 
	                       {18080000, 18150000}, 
	                       {21290000, 21390000}};

//Tuning steps
int laststate = 0; //Last state of rotary encoder
int tuningknob = 0;//Value for rotation detection
long runseconds10 = 0; 
long runseconds10msg = 0;
long tuningcount = 0;

//LO
long f_lo[] = {8998660, 9001800, 9000000}; //LSB/USB LO FREQUENCIES for 9MHz filter

int sideband = 1; //Current sideband USB
int std_sideband [] = {0, 0, 1, 1, 1}; //Standard sideband for each rf band
long c_freq[] =  {3650000, 7120000, 14180000, 18100000, 21290000};
long band_f0[] = {3490000, 6990000, 13990000, 18060000, 20990000};  //Edge frequency I
long band_f1[] = {3810000, 7210000, 14360000, 18170000, 21460000};  //Edge frequency II 

//Band data
int cur_band;

//STRING HANDLING
char *oldbuf;

//S-Meter
int smax = 0;

//Menu
int menu_items[MENUSTRINGS] =  {4, 1, 1, 2, 1, 2}; 

//TX amplifier preset values
int tx_preset[6] = {0, 0, 0, 0, 0, 0};

//MISC
int msgstatus = 0;

//Scan
int thresh = 5;

//Splitmode
int split = 0;

/////////////////////
//Defines for Si5351
/////////////////////
#define SI5351_ADDRESS 0xC0 // 0b11000000 for my module. Others may vary! The 0x60 did NOT work with my module!
#define PLLRATIO 36
#define CFACTOR 1048575

//Set of Si5351A register addresses
#define CLK_ENABLE_CONTROL       3
#define PLLX_SRC				15
#define CLK0_CONTROL            16 
#define CLK1_CONTROL            17
#define CLK2_CONTROL            18
#define SYNTH_PLL_A             26
#define SYNTH_PLL_B             34
#define SYNTH_MS_0              42
#define SYNTH_MS_1              50
#define SYNTH_MS_2              58
#define PLL_RESET              177
#define XTAL_LOAD_CAP          183

//SI5351 Declarations & frequency
void si5351_write(int, int);
void si5351_start(void);
void si5351_set_freq(int, long);

////////////////
// TRX Control
///////////////
void set_band(int);

#undef F_CPU 
#define F_CPU 16000000

  /////////////////////////
 ////   LCD           ////
/////////////////////////
 
//SPI LCD PORT defines
#define LCDPORT PORTD
#define LCD_CLOCK 128 //gray
#define LCD_DATA 64   //violet
#define LCD_DC_A0 32  //green
#define LCD_RST 16    //white
#define LCD_CS 8      //blue

//LCD dimensions
#define LCDHEIGHT 128
#define LCDWIDTH 128

//LCD ST7735 contants
#define ST7735_NOP     0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID   0x04
#define ST7735_RDDST   0x09
#define ST7735_SLPIN   0x10
#define ST7735_SLPOUT  0x11
#define ST7735_PTLON   0x12
#define ST7735_NORON   0x13
#define ST7735_INVOFF  0x20
#define ST7735_INVON   0x21
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_RAMRD   0x2E
#define ST7735_PTLAR   0x30
#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36
#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR  0xB4
#define ST7735_DISSET5 0xB6
#define ST7735_PWCTR1  0xC0
#define ST7735_PWCTR2  0xC1
#define ST7735_PWCTR3  0xC2
#define ST7735_PWCTR4  0xC3
#define ST7735_PWCTR5  0xC4
#define ST7735_VMCTR1  0xC5
#define ST7735_RDID1   0xDA
#define ST7735_RDID2   0xDB
#define ST7735_RDID3   0xDC
#define ST7735_RDID4   0xDD
#define ST7735_PWCTR6  0xFC
#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

//Some sample colors
//USeful website http://www.barth-dev.de/online/rgb565-color-picker/
#define WHITE        0xFFFF
#define BLACK        0x0000
#define GRAY         0x94B2
#define LIGHTGRAY    0xC5D7

#define LIGHTBLUE    0x755C
#define BLUE         0x3C19
#define DARKBLUE     0x0A73
#define DARKBLUE2    0x208C

#define LIGHTRED     0xE882 // 0xEB2D //0xDAAB
#define LIGHTRED2    0xFA00 // 0xEB2D //
#define RED          0xB1A7
#define DARKRED      0x80C3

#define LIGHTGREEN   0x27E0 //0x6E84
#define GREEN        0x6505
#define DARKGREEN    0x3B04

#define LIGHTVIOLET  0xAC19
#define LIGHTVIOLET2 0x9BD9
#define VIOLET       0x71B6
#define DARKVIOLET   0x48AF

#define DARKYELLOW   0xB483
#define YELLOW       0xE746
#define LIGHTYELLOW  0xF752  //0xF7AF

#define LIGHTBROWN   0xF64F
#define BROWN        0x9323
#define DARKBROWN    0x6222

// FONT: 8x14_horizontal_LSB_1
#define FONTWIDTH 8
#define FONTHEIGHT 14
#define CHAROFFSET 0x20

const unsigned char xchar[][FONTHEIGHT] PROGMEM={
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x20
{0x00,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x00,0x08,0x08,0x00,0x00,0x00},	// 0x21
{0x24,0x24,0x24,0x24,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x22
{0x00,0x90,0x90,0x48,0xFE,0x48,0x24,0xFF,0x24,0x12,0x12,0x00,0x00,0x00},	// 0x23
{0x08,0x3C,0x0A,0x0A,0x0A,0x0C,0x18,0x28,0x28,0x28,0x1E,0x08,0x00,0x00},	// 0x24
{0x00,0x86,0x49,0x29,0x29,0x16,0x68,0x94,0x94,0x92,0x61,0x00,0x00,0x00},	// 0x25
{0x00,0x18,0x24,0x24,0x14,0x0C,0x8B,0x91,0x71,0x63,0xFE,0x00,0x00,0x00},	// 0x26
{0x08,0x08,0x08,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x27
{0x60,0x30,0x08,0x08,0x04,0x04,0x04,0x04,0x04,0x08,0x08,0x30,0x60,0x00},	// 0x28
{0x06,0x0C,0x10,0x10,0x20,0x20,0x20,0x20,0x20,0x10,0x10,0x0C,0x06,0x00},	// 0x29
{0x00,0x08,0x08,0x66,0x14,0x2C,0x24,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x2A
{0x00,0x00,0x00,0x00,0x08,0x08,0x08,0x7F,0x08,0x08,0x08,0x00,0x00,0x00},	// 0x2B
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x10,0x08,0x00},	// 0x2C
{0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x2D
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00},	// 0x2E
{0x40,0x20,0x20,0x20,0x10,0x10,0x18,0x08,0x08,0x04,0x04,0x04,0x02,0x00},	// 0x2F
{0x00,0x18,0x24,0x42,0x42,0x42,0x42,0x42,0x42,0x24,0x18,0x00,0x00,0x00},	// 0x30
{0x00,0x10,0x1E,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0xFE,0x00,0x00,0x00},	// 0x31
{0x00,0x1E,0x20,0x20,0x20,0x10,0x10,0x08,0x04,0x02,0x3E,0x00,0x00,0x00},	// 0x32
{0x00,0x3C,0x40,0x40,0x40,0x38,0x40,0x40,0x40,0x40,0x3C,0x00,0x00,0x00},	// 0x33
{0x00,0x20,0x30,0x28,0x28,0x24,0x22,0x7E,0x20,0x20,0x20,0x00,0x00,0x00},	// 0x34
{0x00,0x7C,0x04,0x04,0x04,0x1C,0x60,0x40,0x40,0x40,0x3C,0x00,0x00,0x00},	// 0x35
{0x00,0x38,0x04,0x02,0x02,0x3A,0x46,0x42,0x42,0x44,0x38,0x00,0x00,0x00},	// 0x36
{0x00,0x7E,0x40,0x20,0x20,0x10,0x08,0x08,0x08,0x04,0x04,0x00,0x00,0x00},	// 0x37
{0x00,0x3C,0x42,0x42,0x22,0x1C,0x24,0x42,0x42,0x42,0x3C,0x00,0x00,0x00},	// 0x38
{0x00,0x1C,0x22,0x42,0x42,0x62,0x5C,0x40,0x40,0x20,0x1C,0x00,0x00,0x00},	// 0x39
{0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00},	// 0x3A
{0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x18,0x18,0x10,0x08,0x00},	// 0x3B
{0x00,0x00,0x00,0x00,0x40,0x30,0x08,0x06,0x08,0x30,0x40,0x00,0x00,0x00},	// 0x3C
{0x00,0x00,0x00,0x00,0x00,0x7E,0x00,0x00,0x7E,0x00,0x00,0x00,0x00,0x00},	// 0x3D
{0x00,0x00,0x00,0x00,0x02,0x0C,0x10,0x60,0x10,0x0C,0x02,0x00,0x00,0x00},	// 0x3E
{0x00,0x3E,0x42,0x40,0x20,0x10,0x08,0x08,0x00,0x08,0x08,0x00,0x00,0x00},	// 0x3F
{0x00,0x3C,0x46,0x72,0x49,0x49,0x49,0x69,0xDB,0x22,0x3C,0x00,0x00,0x00},	// 0x40
{0x00,0x00,0x10,0x18,0x28,0x24,0x24,0x42,0x7E,0x42,0x81,0x00,0x00,0x00},	// 0x41
{0x00,0x00,0x3E,0x42,0x42,0x22,0x1E,0x22,0x42,0x42,0x3E,0x00,0x00,0x00},	// 0x42
{0x00,0x00,0x7C,0x02,0x01,0x01,0x01,0x01,0x01,0x02,0x7C,0x00,0x00,0x00},	// 0x43
{0x00,0x00,0x1F,0x21,0x41,0x41,0x41,0x41,0x41,0x21,0x1F,0x00,0x00,0x00},	// 0x44
{0x00,0x00,0x7E,0x02,0x02,0x02,0x3E,0x02,0x02,0x02,0x7E,0x00,0x00,0x00},	// 0x45
{0x00,0x00,0x7E,0x02,0x02,0x02,0x3E,0x02,0x02,0x02,0x02,0x00,0x00,0x00},	// 0x46
{0x00,0x00,0x7C,0x02,0x01,0x01,0x01,0x71,0x41,0x42,0x7C,0x00,0x00,0x00},	// 0x47
{0x00,0x00,0x42,0x42,0x42,0x42,0x7E,0x42,0x42,0x42,0x42,0x00,0x00,0x00},	// 0x48
{0x00,0x00,0x3E,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x3E,0x00,0x00,0x00},	// 0x49
{0x00,0x00,0x3C,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x1E,0x00,0x00,0x00},	// 0x4A
{0x00,0x00,0x42,0x22,0x12,0x0A,0x06,0x0A,0x12,0x22,0x42,0x00,0x00,0x00},	// 0x4B
{0x00,0x00,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x7E,0x00,0x00,0x00},	// 0x4C
{0x00,0x00,0x63,0x63,0x63,0x55,0x55,0x55,0x49,0x41,0x41,0x00,0x00,0x00},	// 0x4D
{0x00,0x00,0x42,0x46,0x46,0x4A,0x4A,0x52,0x52,0x62,0x42,0x00,0x00,0x00},	// 0x4E
{0x00,0x00,0x1C,0x22,0x41,0x41,0x41,0x41,0x41,0x22,0x1C,0x00,0x00,0x00},	// 0x4F
{0x00,0x00,0x3E,0x42,0x42,0x42,0x62,0x1E,0x02,0x02,0x02,0x00,0x00,0x00},	// 0x50
{0x00,0x00,0x1C,0x22,0x41,0x41,0x41,0x41,0x41,0x22,0x1C,0x60,0xC0,0x00},	// 0x51
{0x00,0x00,0x1E,0x22,0x22,0x22,0x1E,0x12,0x32,0x22,0x42,0x00,0x00,0x00},	// 0x52
{0x00,0x00,0x7C,0x02,0x02,0x06,0x18,0x60,0x40,0x40,0x3E,0x00,0x00,0x00},	// 0x53
{0x00,0x00,0x7F,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x00,0x00,0x00},	// 0x54
{0x00,0x00,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x3C,0x00,0x00,0x00},	// 0x55
{0x00,0x00,0x81,0x42,0x42,0x22,0x24,0x14,0x14,0x08,0x08,0x00,0x00,0x00},	// 0x56
{0x00,0x00,0x81,0x81,0xC9,0x59,0x5A,0x5A,0x66,0x66,0x24,0x00,0x00,0x00},	// 0x57
{0x00,0x00,0x81,0x42,0x24,0x18,0x18,0x18,0x24,0x42,0x81,0x00,0x00,0x00},	// 0x58
{0x00,0x00,0x81,0x42,0x22,0x14,0x18,0x08,0x08,0x08,0x08,0x00,0x00,0x00},	// 0x59
{0x00,0x00,0x7F,0x40,0x20,0x10,0x08,0x04,0x02,0x01,0x7F,0x00,0x00,0x00},	// 0x5A
{0x78,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x78,0x00},	// 0x5B
{0x02,0x04,0x04,0x04,0x08,0x08,0x18,0x10,0x10,0x20,0x20,0x20,0x40,0x00},	// 0x5C
{0x1E,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x1E,0x00},	// 0x5D
{0x00,0x10,0x10,0x18,0x28,0x28,0x24,0x44,0x42,0x00,0x00,0x00,0x00,0x00},	// 0x5E
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00},	// 0x5F
{0x10,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x60
{0x00,0x00,0x00,0x00,0x3C,0x40,0x40,0x7C,0x42,0x42,0xFC,0x00,0x00,0x00},	// 0x61
{0x02,0x02,0x02,0x02,0x3A,0x46,0x42,0x42,0x42,0x46,0x3A,0x00,0x00,0x00},	// 0x62
{0x00,0x00,0x00,0x00,0x78,0x04,0x02,0x02,0x02,0x04,0x78,0x00,0x00,0x00},	// 0x63
{0x40,0x40,0x40,0x40,0x5C,0x62,0x42,0x42,0x42,0x62,0x5C,0x00,0x00,0x00},	// 0x64
{0x00,0x00,0x00,0x00,0x3C,0x44,0x42,0x7E,0x02,0x02,0x7C,0x00,0x00,0x00},	// 0x65
{0x70,0x08,0x08,0x08,0x7E,0x08,0x08,0x08,0x08,0x08,0x08,0x00,0x00,0x00},	// 0x66
{0x00,0x00,0x00,0x00,0x7C,0x62,0x42,0x42,0x42,0x62,0x5C,0x40,0x40,0x3C},	// 0x67
{0x02,0x02,0x02,0x02,0x3A,0x46,0x42,0x42,0x42,0x42,0x42,0x00,0x00,0x00},	// 0x68
{0x18,0x18,0x00,0x00,0x1E,0x10,0x10,0x10,0x10,0x10,0x10,0x00,0x00,0x00},	// 0x69
{0x30,0x30,0x00,0x00,0x3C,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x1E},	// 0x6A
{0x02,0x02,0x02,0x02,0x22,0x12,0x0A,0x06,0x1A,0x22,0x42,0x00,0x00,0x00},	// 0x6B
{0x1E,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x00,0x00,0x00},	// 0x6C
{0x00,0x00,0x00,0x00,0x6D,0x5B,0x49,0x49,0x49,0x49,0x49,0x00,0x00,0x00},	// 0x6D
{0x00,0x00,0x00,0x00,0x3A,0x46,0x42,0x42,0x42,0x42,0x42,0x00,0x00,0x00},	// 0x6E
{0x00,0x00,0x00,0x00,0x1C,0x22,0x41,0x41,0x41,0x22,0x1C,0x00,0x00,0x00},	// 0x6F
{0x00,0x00,0x00,0x00,0x3A,0x46,0x42,0x42,0x42,0x46,0x3A,0x02,0x02,0x02},	// 0x70
{0x00,0x00,0x00,0x00,0x5C,0x62,0x42,0x42,0x42,0x62,0x5C,0x40,0x40,0x40},	// 0x71
{0x00,0x00,0x00,0x00,0x3A,0x26,0x02,0x02,0x02,0x02,0x02,0x00,0x00,0x00},	// 0x72
{0x00,0x00,0x00,0x00,0x3C,0x02,0x06,0x18,0x20,0x20,0x1E,0x00,0x00,0x00},	// 0x73
{0x00,0x00,0x04,0x04,0x7F,0x04,0x04,0x04,0x04,0x04,0x78,0x00,0x00,0x00},	// 0x74
{0x00,0x00,0x00,0x00,0x42,0x42,0x42,0x42,0x42,0x62,0x5C,0x00,0x00,0x00},	// 0x75
{0x00,0x00,0x00,0x00,0x81,0x42,0x42,0x22,0x14,0x14,0x08,0x00,0x00,0x00},	// 0x76
{0x00,0x00,0x00,0x00,0x81,0x91,0x5A,0x5A,0x66,0x24,0x24,0x00,0x00,0x00},	// 0x77
{0x00,0x00,0x00,0x00,0x42,0x24,0x18,0x18,0x18,0x24,0x42,0x00,0x00,0x00},	// 0x78
{0x00,0x00,0x00,0x00,0x81,0x42,0x42,0x24,0x24,0x18,0x18,0x08,0x0C,0x07},	// 0x79
{0x00,0x00,0x00,0x00,0x7F,0x20,0x10,0x08,0x04,0x02,0x7F,0x00,0x00,0x00},	// 0x7A
{0x70,0x08,0x08,0x08,0x08,0x08,0x06,0x08,0x08,0x08,0x08,0x08,0x70,0x00},	// 0x7B
{0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x00},	// 0x7C
{0x0E,0x10,0x10,0x10,0x10,0x10,0x60,0x10,0x10,0x10,0x10,0x10,0x0E,0x00},	// 0x7D
{0x00,0x00,0x00,0x00,0x00,0x86,0x99,0x61,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x7E
{0x00,0x00,0x00,0x08,0x1C,0x22,0x41,0x41,0x41,0x41,0x7F,0x00,0x00,0x00},	// 0x7F
{0x00,0x00,0x7C,0x02,0x01,0x01,0x01,0x01,0x01,0x02,0x7C,0x10,0x20,0x30},	// 0x80
{0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00},	// 0x81 S-Meter bar block
{0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10},	// 0x82 |
{0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x83 -
{0x10,0x10,0x10,0x10,0x10,0x10,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x84 '-
{0x10,0x10,0x10,0x10,0x10,0x10,0x1F,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x85 -'
{0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0x10,0x10,0x10,0x10,0x10,0x10,0x10},	// 0x86 ;-
{0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0x10,0x10,0x10,0x10,0x10,0x10,0x10},	// 0x87 -;
{0x00,0x08,0x14,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x88 ° 
};

unsigned int backcolor;

///////////////////////////
//     DECLARATIONS
///////////////////////////
//
//Radio
void set_lo(int);
long set_lo_frequencies(int);
int calc_tuningfactor(void);
int get_adc(int);
int get_pa_temp(void);
int is_mem_freq_ok(long, int);
int get_s_value(void);
long tune_frequency(long);

//I²C
void twi_init(void);
void twi_start(void);
void twi_stop(void);
void twi_write(uint8_t u8data);

//String
int int2asc(long num, int dec, char *buf, int buflen);

//Data display functions
void show_all_data(long, int, int, int, int, int, int);
void show_frequency1(long, int);
void show_frequency2(long);
void show_sideband(int, int);
void show_voltage(int);
void store_vfo(int, int);
void show_band(int, int);
void show_txrx(int);
void show_vfo(int, int);
void show_split(int, int);
void show_pa_temp(void);
void show_msg(char*);
void show_meter(int);
void draw_meter_scale(int meter_type);
void clear_smax(void);

//EEPROM
long load_frequency(int, int);
int load_band(void);
int load_vfo(int);
void store_frequency(int, int, long);
void store_current_operation(int, int, int, long);

//ADC
int get_keys(void);

//MENU
void lcd_drawbox(int, int, int, int);
long menux(long, int);
int navigate_thru_item_list(int, int, int);
void print_menu_head(char*, int);
void print_menu_item_list(int, int, int);
void print_menu_item(char*, int, int);

//MCP4725
#define MCP4725_ADDR 0xC2 //Chinese board with A0 to VCC
void tx_preset_adjust(void);
void mcp4725_set_value(int);
void store_tx_preset(int, int);
int load_tx_preset(int);

//METER
long runseconds10s = 0;
int sv_old = 0;

//SPI DDS AD9850
void spi_send_bit(int);
void set_frequency_ad9850(long); 

//ST7735 LCD
void lcd_init(void);
void lcd_reset(void);                                    //Reset LCD
void lcd_write(int, int);                                //Write a byte of data via SPI
void lcd_write_command(int);                             //Send a command to LCD
void lcd_write_data(int);                                //Send data to LCD
void lcd_setwindow(int, int, int, int);                  //Define output window on LCD
void lcd_setpixel(int, int, unsigned int);               //Set 1 Pixel
void lcd_cls(unsigned int);                              //Clear LCD
void lcd_putchar(int, int, unsigned char, unsigned int, unsigned int, int, int); //Write one char to LCD (double size, variable height)
void lcd_putstring(int, int, char*, unsigned int, unsigned int, int, int);       //Write \0 terminated string to LCD (double size, variable height)
int lcd_putnumber(int, int, long, int, int, int, int, int);                     //Write a number (int or long) to LCD (double size, variable height)

//Scanning QRG
long scan_f0_f1(void);
long scan_vfoa_vfob(void);
void set_scan_threshold(void);

int main(void);

  ////////////////////////
 //   TX gain control  //
////////////////////////
//Send comand to MCP4725
void mcp4725_set_value(int v)
{
    int value = v;
    char *s1, *s2;
    
    s1 = malloc(16);
    s2 = malloc(16);
    
    twi_start();
    twi_write(MCP4725_ADDR); //Device address
    twi_write(64);       		    	
    twi_write(value >> 4); //8 MSBs
    twi_write((value & 0x0F)); //4LSBs
    twi_stop();			
	
	strcpy(s1, "TX PRESET:");
	int2asc(v, -1, s2, 15);	
	strcat(s1, s2);
	//nt xpos = 0, ypos = 8 * FONTHEIGHT;	
	show_msg(s1);
	free(s1);
	free(s2);
} 

void tx_preset_adjust(void)
{
	int key = 0;
	int v1 = tx_preset[cur_band];
			
    show_msg("TX PRESET=    ");
    mcp4725_set_value(v1);
    
	while(get_keys());
	key = get_keys();
			
	while(!key)
	{
		if(tuningknob >= 1)  //Turn CW
		{
		    if(v1 < 4095)
		    {
				v1 += calc_tuningfactor();
			}
			else
			{
				v1 = 4095;
			}			
		    tuningknob = 0;
		    mcp4725_set_value(v1);
		} 

		if(tuningknob <= -1) //Turn CCW
		{    
		    if(v1 > 0)
		    {
				v1 -= calc_tuningfactor();
			}
			else
			{
				v1 = 0;
			}
			
			
		    tuningknob = 0;
		    mcp4725_set_value(v1);
		}	
		key = get_keys();
	}	
	
	if(key == 2)
	{
		tx_preset[cur_band] = v1;
		store_tx_preset(v1, cur_band);
	}	
	
	
}

void store_tx_preset(int value, int band)
{
	//MSB first
    int adr = 128 + band * 2;
    
    while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)adr++, (value >> 8) & 0x0f);
    while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)adr, value & 0xff);
    show_msg("TX preset stored.");
}	

int load_tx_preset(int band)
{
	//MSB first
	int adr = 128 + band * 2;
    int v = 0;
    
    while(!eeprom_is_ready());
    v = eeprom_read_byte((uint8_t*)adr++) << 8;
    while(!eeprom_is_ready());
    v += eeprom_read_byte((uint8_t*)adr);
        
    if(v >= 0 && v <= 4095)
    {
       return v;
    }
    else   
    {
       return 2048; //Set to medium value if result invalid
    }
}	

//Set LO freq to correct sideband
void set_lo(int sb)
{
    si5351_set_freq(SYNTH_MS_0, f_lo[sb]);	
}	

////////////////////////////////
//
// Si5351A commands
//
///////////////////////////////
void si5351_write(int reg_addr, int reg_value)
{
   	 
   twi_start();
   twi_write(SI5351_ADDRESS);
   twi_write(reg_addr);
   twi_write(reg_value);
   twi_stop();
} 

// Set PLLs (VCOs) to internal clock rate of 900 MHz
// Equation fVCO = fXTAL * (a+b/c) (=> AN619 p. 3
void si5351_start(void)
{
  unsigned long a, b, c;
  unsigned long p1, p2;//, p3;
  
  
  // Init clock chip
  si5351_write(XTAL_LOAD_CAP, 0xD2);      // Set crystal load capacitor to 10pF (default), 
                                          // for bits 5:0 see also AN619 p. 60
  si5351_write(CLK_ENABLE_CONTROL, 0x00); // Enable all outputs
  si5351_write(CLK0_CONTROL, 0x0F);       // Set PLLA to CLK0, 8 mA output
  si5351_write(CLK1_CONTROL, 0x2F);       // Set PLLB to CLK1, 8 mA output
  si5351_write(CLK2_CONTROL, 0x2F);       // Set PLLB to CLK2, 8 mA output
  si5351_write(PLL_RESET, 0xA0);          // Reset PLLA and PLLB

  // Set VCOs of PLLA and PLLB to 650 MHz
  a = PLLRATIO;     // Division factor 650/25 MHz !!!!
  b = 0;            // Numerator, sets b/c=0
  c = CFACTOR;      //Max. resolution, but irrelevant in this case (b=0)

  //Formula for splitting up the numbers to register data, see AN619
  p1 = 128 * a + (unsigned long) (128 * b / c) - 512;
  p2 = 128 * b - c * (unsigned long) (128 * b / c);
  //p3  = c;

  
  //Write data to registers PLLA and PLLB so that both VCOs are set to 900MHz intermal freq
  si5351_write(SYNTH_PLL_A, 0xFF);
  si5351_write(SYNTH_PLL_A + 1, 0xFF);
  si5351_write(SYNTH_PLL_A + 2, (p1 & 0x00030000) >> 16);
  si5351_write(SYNTH_PLL_A + 3, (p1 & 0x0000FF00) >> 8);
  si5351_write(SYNTH_PLL_A + 4, (p1 & 0x000000FF));
  si5351_write(SYNTH_PLL_A + 5, 0xF0 | ((p2 & 0x000F0000) >> 16));
  si5351_write(SYNTH_PLL_A + 6, (p2 & 0x0000FF00) >> 8);
  si5351_write(SYNTH_PLL_A + 7, (p2 & 0x000000FF));

  si5351_write(SYNTH_PLL_B, 0xFF);
  si5351_write(SYNTH_PLL_B + 1, 0xFF);
  si5351_write(SYNTH_PLL_B + 2, (p1 & 0x00030000) >> 16);
  si5351_write(SYNTH_PLL_B + 3, (p1 & 0x0000FF00) >> 8);
  si5351_write(SYNTH_PLL_B + 4, (p1 & 0x000000FF));
  si5351_write(SYNTH_PLL_B + 5, 0xF0 | ((p2 & 0x000F0000) >> 16));
  si5351_write(SYNTH_PLL_B + 6, (p2 & 0x0000FF00) >> 8);
  si5351_write(SYNTH_PLL_B + 7, (p2 & 0x000000FF));

}

void si5351_set_freq(int synth, long freq)
{
  unsigned long  a, b, c = CFACTOR; 
  unsigned long f_xtal = 25000000;
  double fdiv = (double) (f_xtal * PLLRATIO) / freq; //division factor fvco/freq (will be integer part of a+b/c)
  double rm; //remainder
  unsigned long p1, p2;
  
  a = (unsigned long) fdiv;
  rm = fdiv - a;  //(equiv. to fractional part b/c)
  b = rm * c;
  p1  = 128 * a + (unsigned long) (128 * b / c) - 512;
  p2 = 128 * b - c * (unsigned long) (128 * b / c);
    
  //Write data to multisynth registers of synth n
  si5351_write(synth, 0xFF);      //1048575 MSB
  si5351_write(synth + 1, 0xFF);  //1048575 LSB
  si5351_write(synth + 2, (p1 & 0x00030000) >> 16);
  si5351_write(synth + 3, (p1 & 0x0000FF00) >> 8);
  si5351_write(synth + 4, (p1 & 0x000000FF));
  si5351_write(synth + 5, 0xF0 | ((p2 & 0x000F0000) >> 16));
  si5351_write(synth + 6, (p2 & 0x0000FF00) >> 8);
  si5351_write(synth + 7, (p2 & 0x000000FF));
}


  ////////////////////////
 //    SPI DDS AD9850  //
////////////////////////
void spi_send_bit(int sbit)
{
    if(sbit)
	{
		DDS_PORT |= (1 << DDS_SDATA);  
	}
	else
	{
		DDS_PORT &= ~(1 << DDS_SDATA);
	}
	
	DDS_PORT |= (1 << DDS_W_CLK);  
    DDS_PORT &= ~(1 << DDS_W_CLK);
    
}


//Set AD9850 to desired frequency
void set_frequency_ad9850(long fx) 
{
    long clk = 125000000;
    long x;
	long hiword, loword;
	int t1;

    double fword0;
    long fword1;
    		
	//Split multiplication by 2^32 into 2 parts to avoid wrong 
	//calculation (reason: 2^32 = 0 in 32 bit number!)
    fword0 = (double) fx / clk * 65536;
    fword1 = (long) (fword0 * 65536);
	hiword = fword1 >> 16;
    loword = fword1 - (hiword << 16);

    //Send 32 frequency bits + 8 additional bits to DDS
	//Start sequence
	DDS_PORT &= ~(1 << DDS_FQ_UD); 
	
	//W0...W15
	x = 1;
	for(t1 = 0; t1 < 16; t1++)
    {
       spi_send_bit(loword & x);
       x <<= 1;
    }
	
	//W16...W31
	x = 1;
	for(t1 = 0; t1 < 16; t1++)
    { 
       spi_send_bit(hiword & x);
       x <<= 1;
    }
	
	//W32...W39
	for(t1 = 0; t1 < 8; t1++)
	{
	    spi_send_bit(0);
	}	
	
	//Stop  sequence
	DDS_PORT |= (1 << DDS_FQ_UD);
	       
}

//////////////////////
//
// SCANNING
//
/////////////////////
//Scan from VFOA to VFOB or vice versa
long scan_f0_f1(void)
{
	
	int t1;
	int key = 0;
	int sval = 0;
	long fx, f[2], ftmp;
	char *s = "SCANNING...";
	char *bstr;
	int xpos0 = (16 - strlen(s)) / 2;
	int ypos0 = 1;
	int wait;
	long runsecs10thresh;
	int msg_sent; 	
	bstr = malloc(17);
	for(t1 = 0; t1 < 17; t1++)
	{
		bstr[t1] = 0;
	}	
	for(t1 = 0; t1 < 16; t1++)
	{
		bstr[t1] = 32;
	}	
	

	for(t1 = 0; t1 < 2; t1++)
	{
	    f[t1] = f_vfo[cur_band][t1];
	}    
	
	if(f[0] > f[1])
	{
		ftmp = f[0];
		f[0] = f[1];
		f[1] = ftmp;
	}	
	
	lcd_cls(backcolor);
	lcd_putstring(0, ypos0 * FONTHEIGHT, bstr, WHITE, LIGHTBLUE, 1, 1);	
	lcd_putstring(xpos0 * FONTWIDTH, ypos0 * FONTHEIGHT, s, WHITE, LIGHTBLUE, 1, 1);	
	free(bstr);
	
	while(get_keys());
	
	draw_meter_scale(0);
		
	while(key != 2)
	{
	    for(fx = f[0]; fx < f[1]; fx += 100)
	    {
			key = get_keys();
			
			set_frequency_ad9850(fx + f_lo[sideband]);
			lcd_putnumber(5 * FONTWIDTH, 4 * FONTHEIGHT, fx / 100, 1, WHITE, backcolor, 1, 1);
			sval = get_s_value();
			show_meter(sval);
			
			msg_sent = 0;
			while(sval > thresh && !key)
			{
				if(!msg_sent) //Just post msg once!
				{
				   show_msg("Stopped.");
				   msg_sent = 1;
				}   
			    sval = get_s_value();
			    show_meter(sval);
			    key = get_keys();
			    wait = 1;
			    
			    //Manual tuning	
			    ftmp = tune_frequency(fx);
			    if(ftmp)
			    {
				    fx = ftmp;
				    set_frequency_ad9850(fx + f_lo[sideband]);
			        lcd_putnumber(5 * FONTWIDTH, 4 * FONTHEIGHT, fx / 100, 1, WHITE, backcolor, 1, 1);
			     }
			}
			
			
			//Wait if strong sig detected
			runsecs10thresh = runseconds10;
			if(wait)
			{
				msg_sent = 0;
			    while(runsecs10thresh + 30 > runseconds10)
			    {
					if(!msg_sent) //Just post msg once!
				    {
				        show_msg("Waiting...");
			            msg_sent = 1;
			        }   
			      
			        key = get_keys();
			        
			        if(key) //Fast out!
			        {
						runsecs10thresh = 0;
					}	
					
					//Manual tuning	
					ftmp = tune_frequency(fx);
					if(ftmp)
					{
						fx = ftmp;
						set_frequency_ad9850(fx + f_lo[sideband]);
						lcd_putnumber(5 * FONTWIDTH, 4 * FONTHEIGHT, fx / 100, 1, WHITE, backcolor, 1, 1);
					}
			    }	
			}   
			 
			if(wait)
			{
				show_msg("Scanning...");
			    wait = 0;
			}   
			
			//Manual tuning	
			ftmp = tune_frequency(fx);
			if(ftmp)
			{
				fx = ftmp;
				set_frequency_ad9850(fx + f_lo[sideband]);
			    lcd_putnumber(5 * FONTWIDTH, 4 * FONTHEIGHT, fx / 100, 1, WHITE, backcolor, 1, 1);
			}

			
			//Key handler
			switch(key)
			{
				    case 2: return fx;
				            break;
				    case 3: return 0;
				            break;        
			}	   
			key = 0;
			
		}
	}	
		
	return 0;
}	

long scan_vfoa_vfob(void)
{
	
	int t1;
	int key = 0;
	int sval = 0;
	long ftmp;
	char *s = "SCANNING...";
	char *bstr;
	int xpos0 = (16 - strlen(s)) / 2;
	int ypos0 = 1;
	
	long runsecs10thresh;
	int msg_sent; 	
	bstr = malloc(17);
	for(t1 = 0; t1 < 17; t1++)
	{
		bstr[t1] = 0;
	}	
	for(t1 = 0; t1 < 16; t1++)
	{
		bstr[t1] = 32;
	}	
	
	lcd_cls(backcolor);
	lcd_putstring(0, ypos0 * FONTHEIGHT, bstr, WHITE, LIGHTBLUE, 1, 1);	
	lcd_putstring(xpos0 * FONTWIDTH, ypos0 * FONTHEIGHT, s, WHITE, LIGHTBLUE, 1, 1);	
	free(bstr);
	
	while(get_keys());
	
	draw_meter_scale(0);
		
	while(key != 2)
	{
	    for(t1  = 0; t1 < 2; t1++)
	    {
			key = get_keys();
			
			//Display frequency
			set_frequency_ad9850(f_vfo[cur_band][t1] + f_lo[sideband]);
			lcd_putnumber(5 * FONTWIDTH, 4 * FONTHEIGHT, f_vfo[cur_band][t1] / 100, 1, WHITE, backcolor, 1, 1);
			sval = get_s_value();
			show_meter(sval);
			
			//Display VFO letter
			lcd_putchar(1 * FONTWIDTH, 4 * FONTHEIGHT, t1 + 65, LIGHTYELLOW, backcolor, 1, 1);
			
			msg_sent = 0;
			while(sval > thresh && !key)
			{
				if(!msg_sent) //Just post msg once!
				{
				   show_msg("Stopped.");
				   msg_sent = 1;
				}   
			    sval = get_s_value();
			    show_meter(sval);
			    key = get_keys();
			    
			    //Manual tuning	
			    ftmp = tune_frequency(f_vfo[cur_band][t1]);
			    if(ftmp)
			    {
				    f_vfo[cur_band][t1] = ftmp;
				    set_frequency_ad9850(f_vfo[cur_band][t1] + f_lo[sideband]);
			        lcd_putnumber(5 * FONTWIDTH, 4 * FONTHEIGHT, f_vfo[cur_band][t1] / 100, 1, WHITE, backcolor, 1, 1);
			        runsecs10thresh = runseconds10;
			     }
			}
		
			
			//Wait 3 seconds
			runsecs10thresh = runseconds10;
			msg_sent = 0;
		    while(runsecs10thresh + 30 > runseconds10)
		    {
				if(!msg_sent) //Just post msg once!
			    {
			        show_msg("Waiting...");
		            msg_sent = 1;
		        }   
		      
		        key = get_keys();
		        
		        if(key) //Fast out!
		        {
					runsecs10thresh = 0;
				}	
					
				//Manual tuning	
				ftmp = tune_frequency(f_vfo[cur_band][t1]);
				if(ftmp)
				{
					f_vfo[cur_band][t1] = ftmp;
					set_frequency_ad9850(f_vfo[cur_band][t1] + f_lo[sideband]);
					lcd_putnumber(5 * FONTWIDTH, 4 * FONTHEIGHT, f_vfo[cur_band][t1] / 100, 1, WHITE, backcolor, 1, 1);
					runsecs10thresh = runseconds10;
				}	
			}   
	 
			
			//Manual tuning	
			ftmp = tune_frequency(f_vfo[cur_band][t1]);
			if(ftmp)
			{
				f_vfo[cur_band][t1] = ftmp;
				set_frequency_ad9850(f_vfo[cur_band][t1] + f_lo[sideband]);
			    lcd_putnumber(5 * FONTWIDTH, 4 * FONTHEIGHT, f_vfo[cur_band][t1] / 100, 1, WHITE, backcolor, 1, 1);
			}

			
			//Key handler
			switch(key)
			{
				    case 2: return ((long)t1 << 28) + f_vfo[cur_band][t1]; //f=Bi0..Bit27; VFO=Bit28
				            break;
				    case 3: return 0;
				            break;        
			}	   
			key = 0;
			
		}
	}
		
	return 0;
}	

	
//Scans 16 memories
void set_scan_threshold(void)
{
	int key = 0;
    char *s = "SCAN THRESH...";
	char *bstr;
	int xpos0 = (16 - strlen(s)) / 2;
	int ypos0 = 1;
	int t1;
    
    bstr = malloc(17);
	for(t1 = 0; t1 < 17; t1++)
	{
		bstr[t1] = 0;
	}	
	for(t1 = 0; t1 < 16; t1++)
	{
		bstr[t1] = 32;
	}	
	lcd_cls(backcolor);
	lcd_putstring(0, ypos0 * FONTHEIGHT, bstr, WHITE, LIGHTBLUE, 1, 1);	
	lcd_putstring(xpos0 * FONTWIDTH, ypos0 * FONTHEIGHT, s, WHITE, LIGHTBLUE, 1, 1);	
	free(bstr);
	
	show_meter(thresh);
    draw_meter_scale(0);
        
    lcd_putnumber(5 * FONTWIDTH, 4 * FONTHEIGHT, thresh, -1, WHITE, backcolor, 1, 1);
	show_meter(thresh);
        	
    while(!key)
    {
        if(tuningknob > 2) //Turn CW
		{
			if(thresh < 12)
			{
				thresh++;
			}
			show_meter(thresh);
	
	        lcd_putstring(5 * FONTWIDTH, 4 * FONTHEIGHT, "    ", WHITE, backcolor, 1, 1);
            lcd_putnumber(5 * FONTWIDTH, 4 * FONTHEIGHT, thresh, -1, WHITE, backcolor, 1, 1);
    
			tuningknob = 0;
		}

		if(tuningknob < -2)  //Turn CCW
		{    
			if(thresh > 0)
			{
				thresh--;
			}
			show_meter(thresh);
			 
			tuningknob = 0;
		}		
		key = get_keys();
	}
	
	if(key == 2)
	{
		eeprom_write_byte((uint8_t*)138, thresh);
	}	
	
}

//////////////////////
//
// RADIO CONTROl
//
/////////////////////
void set_band(int bcode)
{
	int t1 = 0;
    
    //RESET PORTB relay bcd decoder
    PORTD &= ~(1 << PD0);
    PORTD &= ~(1 << PD1);
    PORTD &= ~(1 << PD2);
	
	if(bcode < 0) //Return if RESET-Mode has been selected
	{
		return;
	}
		
	for(t1 = 0; t1 < 3; t1++)
	{
	   if(bcode & (1 << t1))
	   {
		    PORTD |= (1 << t1);  
	   }	 
	   else
	   {	 
           PORTD &= ~(1 << t1);
       }
    }   
    
    //Set LO to preferred sideband of new ham band
    set_lo(std_sideband[bcode]);
    show_sideband(std_sideband[bcode], 0);
}	

//Check if freq is in 20m-band
int is_mem_freq_ok(long f, int cband)
{
	
	if(f >= band_f0[cband] && f <= band_f1[cband])
	{
	    return 1;
	}	
	else
	{
	    return 0;
	}		
	
}

//Calc new frequency from rotary encoder
long tune_frequency(long fx)
{
	long f = fx;
	
	//Manual tuning	
	if(tuningknob > 2)  
	{    
		f += calc_tuningfactor();
		tuningknob = 0;
		return f;
	}
		
	if(tuningknob < -2)
	{
		f -= calc_tuningfactor();
		tuningknob = 0;
		return f;
	}
	
	return 0;
}	

///////////////////////////
//
//         TWI
//
///////////////////////////
void twi_init(void)
{
    //set SCL to 400kHz
    TWSR = 0x00;
    TWBR = 0x0C;
	
    //enable TWI
    TWCR = (1<<TWEN);
}

//Send start signal
void twi_start(void)
{
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while ((TWCR & (1<<TWINT)) == 0);
}

//send stop signal
void twi_stop(void)
{
    TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}

void twi_write(uint8_t u8data)
{
    TWDR = u8data;
    TWCR = (1<<TWINT)|(1<<TWEN);
    while ((TWCR & (1<<TWINT)) == 0);
}



//Perform hardware reset to LCD
void lcd_reset(void)
{
	LCDPORT |= LCD_RST;  
	_delay_ms(100);
	LCDPORT &= ~(LCD_RST);  
	_delay_ms(100);
	LCDPORT |= LCD_RST;  
	_delay_ms(100);
}	

//Write command to LCD
void lcd_write_command(int cmd)
{
	lcd_write(cmd, 0);
}	

//Write data to LCD
void lcd_write_data(int dvalue)
{
	lcd_write(dvalue, 1);
}	

//SPI write data or command to LCD
void lcd_write(int v, int data)
{
	int t1;
	
	if(data) //Data or command?
	{
	    LCDPORT |= LCD_DC_A0;     //Data
	}
	else
	{
	    LCDPORT &= ~(LCD_DC_A0);  //Command
	}
	 
    LCDPORT &= ~(LCD_CS);     //CS=0
	
	for(t1 = 7; t1 >= 0; t1--)
    {
	    LCDPORT &= ~(LCD_CLOCK);  //SCL=0	
	    if(v & (1 << t1))
	    {
		    LCDPORT |= LCD_DATA;
	    }
	    else
	    {
		    LCDPORT &= ~(LCD_DATA); 
	    }
	    LCDPORT |= LCD_CLOCK;  //SCL=1		
	}	
	LCDPORT |= LCD_CS;        //CS=1
}	

//Init LCD to vertical alignement and 16-bit color mode
void lcd_init(void)
{

	lcd_write_command(ST7735_SWRESET); // software reset
	_delay_ms(5);

	lcd_write_command(ST7735_SLPOUT);  // out of sleep mode
	_delay_ms(5);

	lcd_write_command(ST7735_COLMOD);  // set color mode
	lcd_write_data(0x05);              // 16-bit color
	_delay_ms(10);

	lcd_write_command(ST7735_FRMCTR1); // frame rate control
	lcd_write_data(0x00);              // fastest refresh
	lcd_write_data(0x06);              // 6 lines front porch
	lcd_write_data(0x03);              // 3 lines backporch
	_delay_ms(1);

	lcd_write_command(ST7735_MADCTL);  // memory access control (directions)
	lcd_write_data(0xC8);              // row address/col address, bottom to top refresh

	lcd_write_command(ST7735_DISSET5); // display settings #5
	lcd_write_data(0x15);              // 1 clock cycle nonoverlap, 2 cycle gate rise, 3 cycle oscil. equalize
	lcd_write_data(0x02);              // fix on VTL

	lcd_write_command(ST7735_INVCTR);  // display inversion control
	lcd_write_data(0x0);               // line inversion

	lcd_write_command(ST7735_GMCTRP1);
	lcd_write_data(0x09);
	lcd_write_data(0x16);
	lcd_write_data(0x09);
	lcd_write_data(0x20);
	lcd_write_data(0x21);
	lcd_write_data(0x1B);
	lcd_write_data(0x13);
	lcd_write_data(0x19);
	lcd_write_data(0x17);
	lcd_write_data(0x15);
	lcd_write_data(0x1E);
	lcd_write_data(0x2B);
	lcd_write_data(0x04);
	lcd_write_data(0x05);
	lcd_write_data(0x02);
	lcd_write_data(0x0E);

	lcd_write_command(ST7735_GMCTRN1);
	lcd_write_data(0x0B);
	lcd_write_data(0x14);
	lcd_write_data(0x08);
	lcd_write_data(0x1E);
	lcd_write_data(0x22);
	lcd_write_data(0x1D);
	lcd_write_data(0x18);
	lcd_write_data(0x1E);
	lcd_write_data(0x1B);
	lcd_write_data(0x1A);
	lcd_write_data(0x24);
	lcd_write_data(0x2B);
	lcd_write_data(0x06);
	lcd_write_data(0x06);
	lcd_write_data(0x02);
	lcd_write_data(0x0F);
	_delay_ms(10);
	
	lcd_write_command(ST7735_NORON);   // Normal display on
	_delay_ms(10);

	lcd_write_command(ST7735_DISPON);  //Display ON
	
	
}	

//Define window area for next graphic operation
void lcd_setwindow(int x0, int y0, int x1, int y1)
{
	lcd_write_command(ST7735_CASET);   //Coloumn address set
	lcd_write_data(0x00);
	lcd_write_data(x0);          
	lcd_write_data(0x00);
	lcd_write_data(x1);          

	lcd_write_command(ST7735_RASET);   //Row address set
	lcd_write_data(0x00);
	lcd_write_data(y0);        
	lcd_write_data(0x00);
	lcd_write_data(y1);        
}

//Set a pixel (Not used, just for academic purposes!)
void lcd_setpixel(int x, int y, unsigned int color)
{
	lcd_setwindow(x, y, x, y);
	lcd_write_command(ST7735_RAMWR);		// RAM access set
	lcd_write_data(color >> 8);
	lcd_write_data(color);
}

//Clear LCD with background color
void lcd_cls(unsigned int bgcolor)
{
	int x, y;
	
	lcd_setwindow(0, 0, 132, 132);
	lcd_write_command(ST7735_RAMWR);		// RAM access set
		
    for(x = 0; x <= 132; x++)
	{
	    for(y = 0; y <= 132; y++)
	    {
            lcd_write_data(bgcolor >> 8);
	        lcd_write_data(bgcolor);
        }    
    }   
}	

//Print one character to given coordinates to the screen
//sx and sy define "stretch factor"
void lcd_putchar(int x0, int y0, unsigned char ch0, unsigned int fcol, unsigned int bcol, int sx, int sy)
{
	int x, y, t1, t2;
	unsigned char ch;
	
    lcd_setwindow(x0 + 2, y0 + 2, x0 + FONTWIDTH * sx + 1, y0 + FONTHEIGHT * sy);
	lcd_write_command(ST7735_RAMWR);
	
	for(y = 0; y < FONTHEIGHT - 1; y++)
	{
		ch = pgm_read_byte(&xchar[ch0 - CHAROFFSET][y]); 
	    for(t1 = 0; t1 < sy; t1++)
	    {
	        for(x = 0; x < FONTWIDTH; x++)
	        {
		        if((1 << x) & ch)
		        {
					for(t2 = 0; t2 < sx; t2++)
					{
			            lcd_write_data(fcol >> 8);
			            lcd_write_data(fcol);
			        }    
			    }
	   	        else	
		        {
					for(t2 = 0; t2 < sx; t2++)
					{
			            lcd_write_data(bcol >> 8);
			            lcd_write_data(bcol);
			        }    
			    }   
		    }
	    }	
	}
}	

//Print one \0 terminated string to given coordinates to the screen
//xf and yf define "stretch factor"
void lcd_putstring(int x0, int y0, char *s, unsigned int fcol, unsigned int bcol, int xf, int yf)
{
	int x = 0;
	
	while(*s)
	{
		lcd_putchar(x + x0, y0, *(s++), fcol, bcol, xf, yf);
		x += (FONTWIDTH * xf);
	}	
}

//Print a number
//xf and yf define "stretch factor"
int lcd_putnumber(int col, int row, long num, int dec, int fcolor, int bcolor, int xf, int yf)
{
    char *s = malloc(16);
    int slen = 0;
	if(s != NULL)
	{
	    int2asc(num, dec, s, 16);
	    lcd_putstring(col, row, s, fcolor, bcolor, xf, yf);
	    slen = strlen(s);
	    free(s);
	}	
	return slen;
}


/////////////////////////////////
//
// STRING FUNCTIONS
//
////////////////////////////////
//INT 2 ASC
int int2asc(long num, int dec, char *buf, int buflen)
{
    int i, c, xp = 0, neg = 0;
    long n, dd = 1E09;

    if(!num)
	{
	    *buf++ = '0';
		*buf = 0;
		return 1;
	}	
		
    if(num < 0)
    {
     	neg = 1;
	    n = num * -1;
    }
    else
    {
	    n = num;
    }

    //Fill buffer with \0
    for(i = 0; i < 12; i++)
    {
	    *(buf + i) = 0;
    }

    c = 9; //Max. number of displayable digits
    while(dd)
    {
	    i = n / dd;
	    n = n - i * dd;
	
	    *(buf + 9 - c + xp) = i + 48;
	    dd /= 10;
	    if(c == dec && dec)
	    {
	        *(buf + 9 - c + ++xp) = '.';
	    }
	    c--;
    }

    //Search for 1st char different from '0'
    i = 0;
    while(*(buf + i) == 48)
    {
	    *(buf + i++) = 32;
    }

    //Add minus-sign if neccessary
    if(neg)
    {
	    *(buf + --i) = '-';
    }

    //Eleminate leading spaces
    c = 0;
    while(*(buf + i))
    {
	    *(buf + c++) = *(buf + i++);
    }
    *(buf + c) = 0;
	
	return c;
}

//////////////////////////////////
//
//  DATA DISPLAY FUNCTIONS
//
//////////////////////////////////

void show_all_data(long f, int cband, int s, int vfo, int volts, int mtr_scale, int split_state)
{
	int t1;
	int y0 = 36;
	int y1 = y0 + 10 + 2 * FONTHEIGHT;
	
	lcd_cls(backcolor);	
	
	for(t1 = 0; t1 < 16; t1++)
	{
		lcd_putchar(t1 * FONTWIDTH, y0, 0x83, YELLOW, backcolor, 1, 1);
		lcd_putchar(t1 * FONTWIDTH, y1, 0x83, YELLOW, backcolor, 1, 1);
	}	
	show_frequency1(f, 2); 
	show_band(cband, 0);
	show_sideband(s, 0);
	show_vfo(vfo, 0);
	show_voltage(volts);
	show_txrx(0);
	draw_meter_scale(mtr_scale);
	show_split(split_state, backcolor);
	
}   

void show_frequency1(long f, int csize)
{
	int x;
	int y = 48;
	
	if(f < 10000000)
	{
		x = 128 - FONTWIDTH * 12 - 5;
	}
	else
	{
		x = 128 - FONTWIDTH * 14 - 5;
	}	
	
	if(f == 0)
	{
	    lcd_putstring(0, y, "       ", backcolor, backcolor, csize, csize);
	}
	else
	{
		if(csize == 1)
		{
		    lcd_putnumber(x, y, f, 3, LIGHTYELLOW, backcolor, csize, csize);
		}    
		else
		{
		    lcd_putnumber(x, y, f / 100, 1, LIGHTYELLOW, backcolor, csize, csize);
		}
	}	    

}


void show_frequency2(long f)
{
	int xpos, ypos = 2 * FONTHEIGHT;
	
	if(f < 10000000)
	{
		xpos = 10 * FONTWIDTH;
	}
	else
	{
		xpos = 9 * FONTWIDTH;
	}	
	
	lcd_putstring(9 * FONTWIDTH, ypos, "       ", WHITE, backcolor, 1, 1);
	lcd_putnumber(xpos, ypos, f / 100, 1, WHITE, backcolor, 1, 1);
}

void show_band(int band, int invert)
{
	char *band_str[MAXBANDS] = {"80m", "40m", "20m", "17m", "15m"};
	int xpos = 0, ypos = 0;	 
	int forecolor = WHITE;
	
	switch(band)
	{
		case 0: forecolor = LIGHTBLUE;
		        break;
		case 1: forecolor = LIGHTBROWN;
		        break;
		case 2: forecolor = LIGHTGREEN;
		        break;
		case 3: forecolor = LIGHTGRAY;
		        break;
		case 4: forecolor = LIGHTVIOLET2;
		        break;
	}	        
	
	if(invert)
	{	
	     lcd_putstring(xpos, ypos, band_str[band], backcolor, forecolor, 1, 1);
	}
	else     
	{	
	     lcd_putstring(xpos, ypos, band_str[band], forecolor, backcolor, 1, 1);
	}
}

void show_sideband(int sb, int invert)
{
	int xpos = 4 * FONTWIDTH, ypos = 0;
	char *sidebandstr[MAXMODES + 1] = {"LSB", "USB"};
	
	if(invert)
	{
		//Write string to position
	    lcd_putstring(xpos * 6, ypos, sidebandstr[sb], backcolor, LIGHTBLUE, 1, 1);
	}
	else
	{
		//Write string to position
	    lcd_putstring(xpos, ypos, sidebandstr[sb], LIGHTBLUE, backcolor, 1, 1);
	}
	   
}

void show_split(int splt, int invert)
{
	int xpos = 0 * FONTWIDTH, ypos = 2 * FONTHEIGHT;
	char *spltstr[] = {"SPLT OFF", "SPLT ON "};
	int splitcolor[] = {LIGHTRED2, LIGHTGREEN};
	//Write string to position
	lcd_putstring(xpos, ypos, spltstr[splt], splitcolor[splt], backcolor, 1, 1);
	   
}

void show_vfo(int vfo, int invert)
{
	int xpos = 8 * FONTWIDTH, ypos = 0;
	char *vfostr[] = {"VFOA", "VFOB"};
	
	//Show frequency of other VFO in d
	if(!vfo)	
	{
		show_frequency2(f_vfo[cur_band][1]);
	}
	else	
	{
		show_frequency2(f_vfo[cur_band][0]);
	}
	
	//Write string to position
	lcd_putstring(xpos, ypos, vfostr[vfo], YELLOW, backcolor, 1, 1);
}


void show_pa_temp(void)
{
	int xpos = 12 * FONTWIDTH, ypos = FONTHEIGHT;
	int tmp = get_pa_temp();
	int fcolor = LIGHTGREEN;
	
	if(tmp > 40)
	{
		fcolor = LIGHTYELLOW;
	}	
	
	if(tmp > 60)
	{
		fcolor = LIGHTRED;
	}	
	
	xpos = (12 + lcd_putnumber(xpos, ypos, tmp, -1, fcolor, backcolor, 1, 1)) * FONTWIDTH;
	lcd_putchar(xpos, ypos, 0x88, fcolor, backcolor, 1, 1); //°-sign
	xpos += FONTWIDTH;
	lcd_putchar(xpos, ypos, 'C', fcolor, backcolor, 1, 1); //C
}
	
void show_voltage(int v1)
{
    char *buffer;
	int t1, p;
	int xpos = 0, ypos = FONTHEIGHT;
	int fcolor;
		
	buffer= malloc(0x10);
	//Init buffer string
	for(t1 = 0; t1 < 0x10; t1++)
	{
	    *(buffer + t1) = 0;
	}
    
    p = int2asc(v1, 1, buffer, 6) * FONTWIDTH + xpos;
    
    if(v1 < 10)
    {
		fcolor = RED;
	}
	
	if(v1 >= 10 && v1 < 11)
    {
		fcolor = LIGHTRED;
	}	
	
	if(v1 >= 11 && v1 < 13)
    {
		fcolor = GREEN;
	}	
	
	if(v1 >= 13)
    {
		fcolor = LIGHTGREEN;
	}	
	
    lcd_putstring(xpos, ypos, buffer, fcolor, backcolor, 1, 1);
	lcd_putstring(p, ypos, "V ", fcolor, backcolor, 1, 1);
	free(buffer);
}

void show_txrx(int status)
{
	int xpos = 14 * FONTWIDTH, ypos = 0;
		
	//Write string to position
	if(status)
	{
	    lcd_putstring(xpos, ypos, "TX", backcolor, LIGHTRED, 1, 1);;
	}
	else    
	{
	    lcd_putstring(xpos, ypos, "RX", LIGHTGREEN, backcolor, 1, 1);
	}
}	

void show_msg(char *msg)
{	
	int xpos = 0, ypos = 8 * FONTHEIGHT;
	lcd_putstring(xpos, ypos, "                ", WHITE, backcolor, 1, 1);
	lcd_putstring(xpos, ypos, msg, WHITE, backcolor, 1, 1);
	runseconds10msg = runseconds10;
	msgstatus = 1;
}	

//S-Meter bargraph 
void show_meter(int sv0)
{
    int t1, y = 6 * FONTHEIGHT;
    int fcolor;
    int sv = sv0;
    
    if(sv > 16)
    {
		sv = 16;
	}	
				    
	//Clear bar graph
	for(t1 = sv; t1 < smax - 1; t1++)
	{
	   lcd_putchar(t1 * FONTWIDTH, y, ' ', LIGHTGREEN, backcolor, 1, 1);
	}	
    
	//Draw new bar graph
	fcolor = LIGHTGREEN;
	for(t1 = 0; t1 < sv; t1++)
	{
	    if(t1 > 7)
	    {
			fcolor = LIGHTYELLOW;
		}	
	    if(t1 > 10)
	    {
			fcolor = LIGHTRED2;
		}	
		lcd_putchar(t1 * FONTWIDTH, y, 0x81, fcolor, backcolor, 1, 1);
	}	
	
	if(sv > smax)
	{
		smax = sv;
		runseconds10s = runseconds10;
	}	
	
	sv_old = sv;
    
}

void clear_smax(void)
{
	int t1, y = 6 * FONTHEIGHT;
			    
	//Clear bar graph
	for(t1 = smax - 1; t1 < 16; t1++)
	{
	   lcd_putchar(t1 * FONTWIDTH, y, ' ', LIGHTGREEN, backcolor, 1, 1);
	}
	smax = 0;
}
	
void draw_meter_scale(int meter_type)
{
	int y = 7 * FONTHEIGHT;
	
	if(!meter_type)
    {
        lcd_putstring(0, y, "S1 3 5 7 9 +10dB", LIGHTGREEN, backcolor, 1, 1);
    }
    else
    {
        lcd_putstring(0, y, "0 2  4  6  8 10W", LIGHTYELLOW, backcolor, 1, 1);
    }
}

////////////////////////////////////////////////////
//               INTERRUPT HANDLERS
////////////////////////////////////////////////////
//Rotary encoder
ISR(PCINT0_vect)
{ 
    int gray = (PINB & 0x03);           // Read PB0 and PB1
	
    int state = (gray >> 1) ^ gray;         // Convert from Gray code to binary

    if (state != laststate)                //Compare states
    {        
        tuningknob += ((laststate - state) & 0x03) - 2; // Results in -1 or +1
        laststate = state;
        tuningcount++;
    } 
	PCIFR |=  (1 << PCIF0); // Clear pin change interrupt flag.
}

//Timer 1 seconds counter
ISR(TIMER1_COMPA_vect)
{
    runseconds10++; 
    tuningcount = 0;
}

//Calculating tuningrate from rev/time unit
int calc_tuningfactor(void)
{
	return (tuningcount * (tuningcount >> 1)); 
}	

//////////////////////
//
//   A   D   C   
//
/////////////////////
//Read ADC value
int get_adc(int adc_channel)
{
	
	int adc_val = 0;
	
	ADMUX = (1<<REFS0) + adc_channel;     // Kanal adcmode aktivieren
    _delay_ms(1);
	
    ADCSRA |= (1<<ADSC);
    
	_delay_ms(1);
	
	adc_val = ADCL;
    adc_val += ADCH * 256;   
	
	return adc_val;
	
}	

int get_s_value(void)
{
	int adcv = get_adc(1); 
		
	//lcd_putnumber(0 * FONTWIDTH, 2 * FONTHEIGHT, adcv, -1, WHITE, BLACK, 1, 1);	
	adcv -= 360; //360 = minimum AGC voltage when no sig present (band noise 20m)
	
	if(adcv < 10)
	{
		adcv = 10;
	}	
	return  (adcv >> 2);
}	

int get_pa_temp(void)
{
	int adc = get_adc(3);
	double ux = (double) (5 * adc) / 1023;
    double rx = 1000 / (5 / ux - 1);
	double temp = (rx - 815) / 8.81;
	
	return (int) temp;
    
	
}	
//////////////////////////////
//
//    EEPROM-Functions
//
//////////////////////////////
/*
//Byte 64..67: VFOA on 80
//Byte 68..71: VFOB on 80
//Byte 72..75: VFOA on 40
//Byte 76..79: VFOB on 40
//Byte 80..83: VFOA on 20
//Byte 84..87: VFOB on 20
//Byte 88..91: VFOA on 15
//Byte 92..95: VFOB on 15
//Byte 96..99: VFOA on 10
//Byte 100..103: VFOB on 10
//VFO data offset = 32
*/
//Load and store frequency from EEPROM based on VFO
long load_frequency(int vfo, int band)
{
    long rf;
    unsigned char hmsb, lmsb, hlsb, llsb;
    int start_adr = vfo * 4 + band * 8 + OFF_FREQ_DATA;
		
    cli();
    hmsb = eeprom_read_byte((uint8_t*)start_adr);
    hlsb = eeprom_read_byte((uint8_t*)start_adr + 1);
    lmsb = eeprom_read_byte((uint8_t*)start_adr + 2);
    llsb = eeprom_read_byte((uint8_t*)start_adr + 3);
	sei();
	
    rf = (long) 16777216 * hmsb + (long) 65536 * hlsb + (unsigned int) 256 * lmsb + llsb;
		
	return rf;
}

void store_frequency(int vfo, int band, long f)
{
    long hiword, loword;
    unsigned char hmsb, lmsb, hlsb, llsb;
	
    int start_adr = vfo * 4 + band * 8 + OFF_FREQ_DATA;
    
	cli();
    hiword = f >> 16;
    loword = f - (hiword << 16);
    hmsb = hiword >> 8;
    hlsb = hiword - (hmsb << 8);
    lmsb = loword >> 8;
    llsb = loword - (lmsb << 8);

    while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)start_adr, hmsb);

    while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)start_adr + 1, hlsb);

    while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)start_adr + 2, lmsb);

    while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)start_adr + 3, llsb);
    
    sei();	
	
}

/*
Store VFOA (0) or VFOB(1) for given band
//Byte 7:  Last VFO on 80
//Byte 8:  Last VFO on 40
//Byte 9:  Last VFO on 20
//Byte 10: Last VFO on 15
//Byte 11: Last VFO on 20
*/
int load_vfo(int xband)
{
	int start_adr = xband + OFF_VFO_DATA;
	int r = eeprom_read_byte((uint8_t*)start_adr);
	
	if((r == 0) || (r == 1))
	{	
		return(r);
	}
	return(-1);	
}	

void store_vfo(int xband, int xvfo)
{
	int start_adr = xband + OFF_VFO_DATA;
	while(!eeprom_is_ready());
    eeprom_write_byte((uint8_t*)start_adr, xvfo);
}

int load_band(void)
{
	int start_adr = OFF_LAST_BAND_USED;
	int r = eeprom_read_byte((uint8_t*)start_adr);
	
	if((r >= 0) && (r <= 4))
	{	
		return(r);
	}
	return(-1);	
}	


//Save frequency, band, VFO and sideband
void store_current_operation(int cband, int cvfo, int sband, long frequency)
{
    eeprom_write_byte(OFF_LAST_BAND_USED, cband); //Store current band
	store_vfo(cband, cvfo); //Store current VFO
	eeprom_write_byte((uint8_t*)OFF_LAST_SIDEBAND_USED, (uint8_t)sband); //Store current sideband
	store_frequency(cvfo, cband, frequency);	
}	


//////////////////////////////
//
//    M   E   N   U
//
//////////////////////////////
//Read keys via ADC0
int get_keys(void)
{

    int key_value[] = {39, 76, 103, 135};
    int t1;
    int adcval = get_adc(0);
        
    //TEST display of ADC value 
    /*
    lcd_putstring(0, 5, "    ", 0, 0);    
    oled_putnumber(0, 5, adcval, -1, 0, 0);    
   	*/
    for(t1 = 0; t1 < 4; t1++)
    {
        if(adcval > key_value[t1] - 10 && adcval < key_value[t1] + 10)
        {
             return t1 + 1;
        }
    }
    return 0;
}


  //////////
 // MENU //
//////////
void lcd_drawbox(int x0, int y0, int x1, int y1)
{
	int t1;
	
	for(t1 = y0; t1 < y1 + 1; t1++)
	{
		lcd_putchar(x0 * FONTWIDTH, t1 * FONTHEIGHT, 0x82, WHITE, backcolor, 1, 1);
		lcd_putchar(x1 * FONTWIDTH, t1 * FONTHEIGHT, 0x82, WHITE, backcolor, 1, 1);
	}
	
	for(t1 = x0 + 1; t1 < x1; t1++)
	{
		lcd_putchar(t1 * FONTWIDTH, (y0 - 1) * FONTHEIGHT, 0x83, WHITE, backcolor, 1, 1);
		lcd_putchar(t1 * FONTWIDTH, (y1 + 1)  * FONTHEIGHT, 0x83, WHITE, backcolor, 1, 1);
	}
	
	//The "Cornerstones"
	lcd_putchar(x0 * FONTWIDTH, (y1 + 1) * FONTHEIGHT, 0x84, WHITE, backcolor, 1, 1);
	lcd_putchar(x1 * FONTWIDTH, (y1 + 1) * FONTHEIGHT, 0x85, WHITE, backcolor, 1, 1);
	lcd_putchar(x0 * FONTWIDTH, (y0 - 1) * FONTHEIGHT, 0x86, WHITE, backcolor, 1, 1);
	lcd_putchar(x1 * FONTWIDTH, (y0 - 1) * FONTHEIGHT, 0x87, WHITE, backcolor, 1, 1);
}
		
void print_menu_head(char *head_str0, int m_items)
{	
    int xpos0 = (16 - strlen(head_str0)) / 2;
	int ypos0 = 1;
	char *bstr;
	int t1;
	
	bstr = malloc(17);
	for(t1 = 0; t1 < 17; t1++)
	{
		bstr[t1] = 0;
	}	
	for(t1 = 0; t1 < 16; t1++)
	{
		bstr[t1] = 32;
	}	
			
	lcd_cls(backcolor);	
	lcd_drawbox(4, 3, 13, 3 + m_items);
	lcd_putstring(0, ypos0 * FONTHEIGHT, bstr, WHITE, LIGHTBLUE, 1, 1);	
	lcd_putstring(xpos0 * FONTWIDTH, ypos0 * FONTHEIGHT, head_str0, WHITE, LIGHTBLUE, 1, 1);	
	
	
	free(bstr);
}

void print_menu_item(char *m_str, int ypos, int invert)
{
	int xpos1 = 40;
		
	if(invert)
	{
		lcd_putstring(xpos1, (ypos + 3) * FONTHEIGHT, m_str, DARKBLUE, LIGHTYELLOW, 1, 1);
	}
	else
	{
		lcd_putstring(xpos1, (ypos + 3) * FONTHEIGHT, m_str, WHITE, backcolor, 1, 1);
	}	
}
	
//Print the itemlist or single item
void print_menu_item_list(int m, int item, int invert)
{
	
	char *menu_str[MENUSTRINGS][MENUITEMS] =  {{"80m    ", "40m    ", "20m    ", "17m    ", "15m    "},
		                                       {"VFO A  ", "VFO B  ", "       ", "       ", "       "}, 
	                                           {"LSB    ", "USB    ", "       ", "       ", "       "},
	                                           {"f0..f1 ", "VFO A/B", "THRESH ", "       ", "       "},
	                                           {"OFF    ", "ON     ", "       ", "       ", "       "},
	                                           {"SET LSB", "SET USB", "TX GAIN", "       ", "       "}};
	int t1;
    
    for(t1 = 0; t1 < menu_items[m] + 1; t1++)
	{
	    if(t1 == item)
		{
		    print_menu_item(menu_str[m][t1], t1, 1);   
		}
		else
		{
		    print_menu_item(menu_str[m][t1], t1, 0);   
		}   
	}
}

//Returns menu_pos if OK or -1 if aborted
int navigate_thru_item_list(int m, int maxitems, int menu_pos)
{
	int mpos = menu_pos;
	
	print_menu_item_list(m, mpos, 1);     //Write 1st entry in inverted color
	
	int key = get_keys();
	
    while(key == 0)
	{
		if(tuningknob > 2) //Turn CW
		{
			print_menu_item_list(m, mpos, 0); //Write old entry in normal color
		    if(mpos < maxitems)
		    {
				mpos++;
			}
			else	
			{
				mpos = 0;
			}
			print_menu_item_list(m, mpos, 1); //Write new entry in reverse color
		    tuningknob = 0;
		}

		if(tuningknob < -2)  //Turn CCW
		{    
		    print_menu_item_list(m, mpos, 0); //Write old entry in normal color
		    if(mpos > 0)
		    {
				mpos--;
			}
			else	
			{
				mpos = maxitems;
			}
			print_menu_item_list(m, mpos, 1); //Write new entry in reverse color
		    tuningknob = 0;
		}	
		
		//Preview settings
		if(m == 1)
		{
			set_frequency_ad9850(f_vfo[cur_band][mpos] + f_lo[sideband]);
			lcd_putnumber(FONTWIDTH * 4, FONTHEIGHT * 6, f_vfo[cur_band][mpos] / 100, 1, WHITE, backcolor, 1, 1);
		}	
		
		if(m == 2)
		{
			set_lo(mpos);
		}	
		
			
				
		key = get_keys();
	}
	
	set_lo(sideband);
		
	while(get_keys());
	
	switch(key)
	{   case 1: return -1;       //Next menu!
		        break;
	    case 2: return mpos; //OK
	            break;
	    case 3: return -3;       //Quit menu!
	            break;
	}
	
	return -1;
}	
			
long menux(long f, int c_vfo)
{
	
	int result = 0;
	int menu;
		
	////////////////
	//  BAND SEL  //
	////////////////
	while(get_keys());
	menu = 0;
	print_menu_head("BAND SET", menu_items[menu]);	//Head outline of menu
	print_menu_item_list(menu, -1, cur_band);              //Print item list in full
	
	//Navigate thru item list
	result = navigate_thru_item_list(menu, menu_items[menu], cur_band);
	if(result > -1)
	{
		return(menu * 10 + result);
	}
	else
	{
		switch(result)
		{				
		    case -3: return -3; //Quit menu         
		             break;
		    case -1: break;
		}
    }		
    
	
	////////////////
	// VFO FUNCS  //
	////////////////
	while(get_keys());
	menu = 1;
	print_menu_head("VFO SELECT", menu_items[menu]);	//Head outline of menu
	print_menu_item_list(menu, -1, cur_vfo);              //Print item list in full
	
	//Navigate thru item list
	result = navigate_thru_item_list(menu, menu_items[menu], cur_vfo);
	if(result > -1)
	{
		return(menu * 10 + result);
	}
	else
	{
		switch(result)
		{				
		    case -3: return -3; //Quit menu         
		             break;
		    case -1: break;
		}
    }		
    
    ////////////////
	// SIDEBAND  //
	////////////////
	while(get_keys());
	menu = 2;
	print_menu_head("SIDEBAND", menu_items[menu]);	//Head outline of menu
	print_menu_item_list(menu, -1, sideband);              //Print item list in full
	
	//Navigate thru item list
	result = navigate_thru_item_list(menu, menu_items[menu], sideband);
	if(result > -1)
	{
		return(menu * 10 + result);
	}
	else
	{
		switch(result)
		{				
		    case -3: return -3; //Quit menu         
		             break;
		    case -1: break;
		} 
    }		
    
      ////////////////
	 //    SCAN    //
	////////////////
	while(get_keys());
	menu = 3;
	print_menu_head("SCAN FUNC", menu_items[menu]);	//Head outline of menu
	print_menu_item_list(menu, 0, 1);              //Print item list in full
	
	//Navigate thru item list
	result = navigate_thru_item_list(menu, menu_items[menu], 0);
	if(result > -1)
	{
		return(menu * 10 + result);
	}
	else
	{
		switch(result)
		{				
		    case -3: return -3; //Quit menu         
		             break;
		    case -1: break;
		} 
    }	
    
      ////////////////
	 //    SPLIT    //
	////////////////
	while(get_keys());
	menu = 4;
	print_menu_head("SPLIT", menu_items[menu]);	//Head outline of menu
	print_menu_item_list(menu, 0, 1);              //Print item list in full
	
	//Navigate thru item list
	result = navigate_thru_item_list(menu, menu_items[menu], split);
	if(result > -1)
	{
		return(menu * 10 + result);
	}
	else
	{
		switch(result)
		{				
		    case -3: return -3; //Quit menu         
		             break;
		    case -1: break;
		} 
    }	
    
	 ///////////////////////////////////
	//   LO SET MODE AND TX PRESET   //
	//////////////////////////////////
	while(get_keys());
	menu = 5;
	print_menu_head("ADJUST", menu_items[menu]);	//Head outline of menu
	print_menu_item_list(menu, -1, 0);              //Print item list in full
	   
	//Navigate thru item list
	result = navigate_thru_item_list(menu, menu_items[menu], 0);
					
	if(result > -1)
	{
		return(menu * 10 + result);
	}
	else
	{
		switch(result)
		{	
		    case -3: return -3; //Quit menu         
		             break;
		    case -1: break;
		}
    }
    
	return -2; //Nothing to do in main()
}

long set_lo_frequencies(int sb)
{
	int key = 0, t1;
		
	lcd_cls(backcolor);	
    
	key = 0;	
		
	for(t1 = 0; t1 < 16; t1++)
	{
	   lcd_putchar(t1 * FONTWIDTH, 1 * FONTHEIGHT, 32, WHITE, LIGHTBLUE, 1, 1);	
	}   
	lcd_putstring(2 * FONTWIDTH, 1 * FONTHEIGHT, "LO SET MODE", WHITE, LIGHTBLUE, 1, 1);	
	
	if(!sb)
	{
	    lcd_putstring(3 * FONTWIDTH, 2 * FONTHEIGHT, " fLO LSB ", WHITE, BLUE, 1, 1);
	}
	else    
	{
	    lcd_putstring(3 * FONTWIDTH, 2 * FONTHEIGHT, " fLO USB ", WHITE, BLUE, 1, 1);
	}
	
	while(get_keys());
	
	set_frequency_ad9850(f_vfo[cur_band][cur_vfo] + f_lo[sb]);   
	si5351_set_freq(SYNTH_MS_0, f_lo[sb]);
	show_frequency1(f_lo[sb], 1);
	
	while(!key)
	{
		if(tuningknob > 2)  
		{    
		    f_lo[sb] += 10;
		    tuningknob = 0;
		    si5351_set_freq(SYNTH_MS_0, f_lo[sb]);
			show_frequency1(f_lo[sb], 1);
		}	
		
		if(tuningknob < -2)  
		{    
		    f_lo[sb] -= 10;
			tuningknob = 0;
			si5351_set_freq(SYNTH_MS_0, f_lo[sb]);
			show_frequency1(f_lo[sb], 1);
		}	
		
		
		key = get_keys();    
	}	
	
	while(get_keys());
	
	if(key != 2)
	{
        return 0;
    }
    else
    {
        return f_lo[sb];
    }    
}


int main(void)
{
	int t1;
	int m;
	long runseconds10s2 = runseconds10;
	long ftmp;
		
	backcolor = BLACK;
        		
	//Variables runtime
	int key = 0;
		    
    //Volts measurement
    int adc_v;
    int adc_v_old = 0;
    long runseconds10volts = -50;
    double v1;
    
   	//Meter
    int sval0 = 0;
    int sval1 = 0;    
    
    //TX/RX indicator
	int txrx = 0;
	
    //Output ports set        
    //SPI DDS
    DDS_DDR = (1 << PB2)|(1 << PB3)| (1 << PB4)|(1 << PB5); //SPI (PB2..PB5)         
    DDS_RESETPORT |= (1 << DDS_RESETPIN);       //Bit set
    _delay_ms(1);       //wait for > 20ns i. e. 1ms minimum time with _delay_s()
	DDS_RESETPORT &= ~(1 << DDS_RESETPIN);  //Bit erase        

    DDRD = 0xFF;   //Relay driver 0:2, LCD 3:7
             
	//Pull-up Rs
	PORTC = (1 << PC0); //Keys
	PORTB = (1 << PB0)|(1 << PB1); //rotary encoder
	
	//ADC config and ADC init
    ADCSRA = (1<<ADPS0) | (1<<ADPS1) | (1<<ADEN); //Prescaler 64 and ADC on
	get_adc(0); //One dummy conversion

	//Interrupt definitions for rotary encoder  
	PCICR |= (1 << PCIE0);                     // enable pin change interupt
	PCMSK0 |= ((1 << PCINT0) | (1<<PCINT1));  //enable encoder pins as interrupt source
	
	
	//Timer 1 as counter for 10th of seconds
    TCCR1A = 0;             // normal mode, no PWM
    TCCR1B = (1 << CS10) | (1 << CS12) | (1<<WGM12);   // Prescaler = 1/1024 based on system clock 16MHz
                                                       // 15625 incs/sec
                                                       // and enable reset of counter register
	OCR1AH = (1563 >> 8);                             //Load compare values to registers
    OCR1AL = (1563 & 0x00FF);
	TIMSK1 |= (1<<OCIE1A);
				
    //RESET PORTD D0:D2 relay bcd decoder
    set_band(-1);
    
    //Allocate display buffer for text output
    oldbuf = malloc(0x10);	
    for(t1 = 0; t1 < 0x10; t1++)
	{
		*(oldbuf + t1) = 0;
	}	
		
    //TWI
	twi_init();
	
	//LCD
	lcd_reset();
	lcd_init();
	//lcd_cls(backcolor);	

    //VFO and LO start
    si5351_start();
    
    //Load start values
    //BAND
    cur_band = load_band();
    if(cur_band == -1)
    {
		cur_band = 2; //Default 20m
	}
	//VFO	
	cur_vfo = load_vfo(cur_band);	
	if(cur_vfo == -1)
	{
        cur_vfo = 0;
    }
    //QRG
	f_vfo[cur_band][cur_vfo] = load_frequency(cur_vfo, cur_band);  
	if(!is_mem_freq_ok(f_vfo[cur_band][cur_vfo], cur_band))
	{
	    f_vfo[cur_band][cur_vfo] = c_freq[cur_band];
	}
	sideband = std_sideband[cur_band];
	set_band(cur_band);
	
	//Load scan threshold
    thresh = eeprom_read_byte((uint8_t*)138);          	
    if(thresh < 0 || thresh > 12)
    {
		thresh = 8;
	}
			
	//Voltage
	v1 = (double) get_adc(6) * 5 / 1024 * 6 * 10;
	adc_v = (int) v1;	
				
    //AD9850 radio oscillator ON
    set_frequency_ad9850(f_vfo[cur_band][cur_vfo] + f_lo[sideband]);    
    set_frequency_ad9850(f_vfo[cur_band][cur_vfo] + f_lo[sideband]);    
    
    //Si5351 LO on
    set_lo(sideband);
    si5351_set_freq(SYNTH_MS_1, 0); 
    si5351_set_freq(SYNTH_MS_2, 0); 
       
    show_all_data(f_vfo[cur_band][cur_vfo], cur_band, sideband, cur_vfo, adc_v, 0, split);   
    
    //Load TX preset values
    for(t1 = 0; t1 < 5; t1++)
    {
		tx_preset[t1] = load_tx_preset(t1);
		if(tx_preset[t1] < 0 || tx_preset[t1] > 4096)
		{
			tx_preset[t1] = 2048;
		}	
	}	
    mcp4725_set_value(tx_preset[cur_band]); 
             
    sei();    
    
    show_msg("Mini5 DK7IH 2020");    
        
    for(;;) 
	{
	       
        ftmp = tune_frequency(f_vfo[cur_band][cur_vfo]);
        if(ftmp)
        {
			f_vfo[cur_band][cur_vfo] = ftmp;
		    set_frequency_ad9850(f_vfo[cur_band][cur_vfo] + f_lo[sideband]);    
			show_frequency1(f_vfo[cur_band][cur_vfo], 2);
		}	
        key = get_keys();    
        
        if(key == 1)
        {
			//Save current VFO,frequency etc.
			store_frequency(cur_vfo, cur_band, f_vfo[cur_band][cur_vfo]);
			store_current_operation(cur_band, cur_vfo, sideband, f_vfo[cur_band][cur_vfo]);
			
			while(get_keys());
			m = menux(f_vfo[cur_band][cur_vfo], cur_vfo);
			switch(m)
			{
				case 0:    
	            case 1: 
	            case 2: 
	            case 3:
	            case 4:     cur_band = m;
	                        set_band(cur_band); //Band changed
				            if(is_mem_freq_ok(load_frequency(cur_vfo, cur_band), cur_band))
				            {
						        f_vfo[cur_band][cur_vfo] = load_frequency(cur_vfo, cur_band); 	 
						    }    
						    else
						    {
						        f_vfo[cur_band][cur_vfo] = c_freq[cur_band]; 	 
						    }
			                set_frequency_ad9850(f_vfo[cur_band][cur_vfo] + f_lo[sideband]);   
                            eeprom_write_byte((uint8_t*)OFF_LAST_BAND_USED, cur_band); //Store current band
                            //Load TX gain preset value
                            mcp4725_set_value(tx_preset[cur_band]);
    
			                break;
			            
				  
	            case 10: 
	            case 11:    cur_vfo = m - 10;
	                        show_vfo(cur_vfo, 0);
	                        f_vfo[cur_band][cur_vfo] = load_frequency(cur_vfo, cur_band); //VFO changed
                            if(!is_mem_freq_ok(f_vfo[cur_band][cur_vfo], cur_band))	            
                            {
						        f_vfo[cur_band][cur_vfo] = c_freq[cur_band];
						    }  
						    set_frequency_ad9850(f_vfo[cur_band][cur_vfo] + f_lo[sideband]);   
						    
			                eeprom_write_byte((uint8_t*)OFF_LAST_VFO_USED, (uint8_t)cur_vfo); //Store current VFO
			                break;
			     
				case 20: 
	            case 21:    sideband = m - 20;     
	                        set_frequency_ad9850(f_vfo[cur_band][cur_vfo] + f_lo[sideband]);    			
	                        set_lo(sideband);
	                        break;
	                        
	            case 30:    ftmp = scan_f0_f1();
	                        if(is_mem_freq_ok(ftmp, cur_band)) //Freq OK => set VFO
	                        {
								f_vfo[cur_band][cur_vfo] = ftmp;
								set_frequency_ad9850(f_vfo[cur_band][cur_vfo] + f_lo[sideband]);   
							}	
							break;         
							
				case 31:    ftmp = scan_vfoa_vfob();
				            if(ftmp & (0xFFFFFFF))
				            {
								cur_vfo = (ftmp >> 28);
								show_vfo(cur_vfo, backcolor);
								f_vfo[cur_band][cur_vfo] = ftmp & 0xFFFFFFF;
								set_frequency_ad9850(f_vfo[cur_band][cur_vfo] + f_lo[sideband]);   
							}	
							break;
	                        
	            case 32:    set_scan_threshold();
	                        break;              
	            case 40:    
	            case 41:    split = m - 40;
	                        show_split(split, backcolor);
	                        break;
	            
	            case 50: 
	            case 51:    ftmp = set_lo_frequencies(m - 50);
	                        if(ftmp > 0)
	                        {
								f_lo[m - 50] = ftmp;     
								set_lo(sideband);
	                        }
	                        break;
	            case 52:    tx_preset_adjust();
	                        break;
	       
	                    
			    
		    }       
		    show_all_data(f_vfo[cur_band][cur_vfo], cur_band, sideband, cur_vfo, adc_v, 0, split);     
        }     
        
        //Store current frequency setting
        if(key == 2)		
        {
			while(get_keys());
			key = 0;
			store_current_operation(cur_band, cur_vfo, sideband, f_vfo[cur_band][cur_vfo]);
			show_msg("Storing OK.");
	    }	

        if(key == 3)		
        {
			while(get_keys());
			key = 0;
			tx_preset_adjust();
	    }	
	    	    	
        //Fast QSY to next band
        if(key == 4)		
        {
			key = 0;
			while(get_keys());
			
			if(cur_band < MAXBANDS -1)
			{
				cur_band++;
			}
			else
			{
				cur_band = 0;
			}
			
			show_band(cur_band, 0);
			set_band(cur_band); //Band changed
		    
		    for(t1 = 0; t1 < 0x10; t1++)
			{
				*(oldbuf + t1) = 0;
			}	
			f_vfo[cur_band][cur_vfo] = load_frequency(cur_vfo, cur_band); 
			
			if(!is_mem_freq_ok(f_vfo[cur_band][cur_vfo], cur_band))
			{
				f_vfo[cur_band][cur_vfo] = c_freq[cur_band];
			}	
				
			set_frequency_ad9850(f_vfo[cur_band][cur_vfo] + f_lo[sideband]);    
			show_frequency1(0, 2);
			show_frequency1(f_vfo[cur_band][cur_vfo], 2);
			show_vfo(cur_vfo, backcolor);
			eeprom_write_byte((uint8_t*)OFF_LAST_BAND_USED, cur_band); //Store current band
			//Load TX gain preset value
            mcp4725_set_value(tx_preset[cur_band]);
	    }		
		
        //VOLTS and TEMPERATURE measurement
		if(runseconds10 > runseconds10volts + 50)
		{
		    v1 = (double) get_adc(6) * 5 / 1024 * 6 * 10;
		    adc_v = (int) v1;
   		    if(adc_v != adc_v_old)
		    {
    	        show_voltage(adc_v);
	     		adc_v_old = adc_v;
		    }	
		    
		    show_pa_temp();
		    runseconds10volts = runseconds10;
	    }
	    
	    //After 10th second check S-Val resp. PWR value
	    if(runseconds10 > runseconds10s)
		{
			//TX/RX switching check
		    if(get_adc(7) > 1000) //TX, cause ADC7 is hi
		    {
			    if(!txrx)
			    {
					txrx = 1;
				    draw_meter_scale(1);	 
			        show_txrx(1);
			        			        
			        if(split)
			        {
						if(cur_vfo == 0)
						{
			                set_frequency_ad9850(f_vfo[cur_band][1] + f_lo[sideband]);    
			                show_frequency1(f_vfo[cur_band][1], 2);
			            }    
			            else
			            {
			                set_frequency_ad9850(f_vfo[cur_band][0] + f_lo[sideband]);    
			                show_frequency1(f_vfo[cur_band][0], 2);
			            }    
			        }
			     }       
			    
		    }	 
		
		    if(!(get_adc(7) > 1000)) //RX, cause ADC7 lo
		    {
			    if(txrx)
			    {
				    txrx = 0;
				    draw_meter_scale(0);
				    show_txrx(0);
			        
			        if(split)
			        {
						if(cur_vfo == 0)
						{
			                set_frequency_ad9850(f_vfo[cur_band][0] + f_lo[sideband]);    
			                show_frequency1(f_vfo[cur_band][0], 2);
			            }    
			            else
			            {
			                set_frequency_ad9850(f_vfo[cur_band][1] + f_lo[sideband]);    
			                show_frequency1(f_vfo[cur_band][1], 2);
			            }    
			        }
			    }	 
		    }

			if(!txrx)
	        {
				sval0 = get_s_value(); //ADC voltage on ADC1 SVAL 
	            show_meter(sval0); //S-Meter		
			}
			else
			{
			    sval1 = get_adc(2); //TX PWR voltage on ADC2
				show_meter(sval1 >> 6); //S-Meter		
			}
		    runseconds10s = runseconds10;
		}    
		
		
		if(runseconds10 > runseconds10s2 + 20)
		{
			clear_smax();
			runseconds10s2 = runseconds10;
		}	
		
		if(runseconds10 > runseconds10msg + 60 && msgstatus)
		{
			show_msg("Mini5 DK7IH 2020");    
			runseconds10msg = runseconds10;
			msgstatus = 0;
		}	
        
	}
	
    return 0;
}
