/*
 * AMG8833.h
 *
 *  Created on: 22Jan.,2020
 *      Author: jingchen
 */

#ifndef AMG8833_H_
#define AMG8833_H_

#include <stdint.h>
enum AMG88xx_register
{
	AMG88xx_PCTL = 0x00,     //Power Control Register
	AMG88xx_RST  = 0x01,     //Reset Register
	AMG88xx_FPSC = 0x02,     //Frame Rate Register
	AMG88xx_INTC = 0x03,     //Interrupt Control Register
	AMG88xx_STAT = 0x04,
	AMG88xx_SCLR = 0x05,    //Status Clear Register  bit1: INTCLR
	//0x06 reserved
	AMG88xx_AVE = 0x07,     //setting moving average Output Mode
	AMG88xx_INTHL = 0x08,    //  INT_LVL_H[7:0]  upper limit of value to produce INT
	AMG88xx_INTHH = 0x09,    //  INT_LVL_H[11:8] upper limit of value to produce INT
	AMG88xx_INTLL = 0x0A,    //  INT_LVL_L[7:0]  lower limit of value to produce INT
	AMG88xx_INTLH = 0x0B,    //  INT_LVL_L11:8]  lower limit of value to produce INT
	AMG88xx_IHYSL = 0x0C,    //  Hysteresis Level [7:0]
	AMG88xx_IHYSH = 0x0D,    //  Hysteresis Level [11:8]
	AMG88xx_TTHL = 0x0E,
	AMG88xx_TTHH = 0x0F,
	AMG88xx_INT_OFFSET = 0x010,
	AMG88xx_PIXEL_OFFSET = 0x80
};

/***
 * Read Only register
 *0x10  INT0     PIX08 PIX07 PIX06 PIX05 PIX04 PIX03 PIX02 PIX01
 *0x11  INT1     PIX16 PIX15 PIX14 PIX13 PIX12 PIX11 PIX10 PIX09
 * .....
 *0x17  INT7     PIX64 PIX63 PIX62 PIX61 PIX60 PIX59 PIX58 PIX57
 *1 --> INT
 *0 --> No INT
 * ***/



enum power_modes{
	AMG88xx_NORMAL_MODE = 0x00,
	AMG88xx_SLEEP_MODE = 0x01,
	AMG88xx_STAND_BY_60 = 0x20,
	AMG88xx_STAND_BY_10 = 0x21
};

enum sw_resets {
	AMG88xx_FLAG_RESET = 0x30,
	AMG88xx_INITIAL_RESET = 0x3F
};

enum frame_rates {
	AMG88xx_FPS_10 = 0x00,
	AMG88xx_FPS_1 = 0x01
};

enum int_enables{
	AMG88xx_INT_DISABLED = 0x00,
	AMG88xx_INT_ENABLED = 0x01
};

enum int_modes {
	AMG88xx_DIFFERENCE = 0x00,
	AMG88xx_ABSOLUTE_VALUE = 0x01
};


void AMG8833_Init(void);
void AMG8833_Read_pixdels(float *buf);
uint16_t AMG8833_Read_thermistor(void);
void Set_interrupt(float upper, float  lower);
void Enable_interrupt(void);
void Disable_interrupt(void);
void Clear_interrupt(void);
void getInterrupt(uint8_t *buf);

#endif /* AMG8833_H_ */
