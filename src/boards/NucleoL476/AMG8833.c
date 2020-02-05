/*
 * AMG8833.c
 *
 *  Created on: 22Jan.,2020
 *      Author: jingchen
 */

#include "AMG8833.h"
#include "i2c-board.h"
#include <stdio.h>
#include "stm32l4xx.h"
#include "gpio.h"

#include "stm32l4xx_hal.h"

uint16_t DevAddress = 0x69<<1;
uint8_t  MemAddress = 0x00;

uint8_t  Txbuff[2] = {0x00};
uint16_t thermistor;

uint16_t quarter_deg = 0x004;  //0.25 Celsius degree

Gpio_t I2C2_SDA ;
Gpio_t I2C2_SCL ;
I2C_HandleTypeDef i2c2;

static void I2C2_initaliztion(void);
static float int12ToFloat(uint16_t val);
//static void Error_Handler(void);
static void Read(uint8_t *buf);

void  AMG8833_Init(void)
{
	I2C2_initaliztion();
	MemAddress = AMG88xx_PCTL;    //PCTL
	Txbuff[0]=0x00;               //cmd to send
    HAL_I2C_Mem_Write(&i2c2,DevAddress,MemAddress, sizeof(MemAddress),Txbuff, sizeof(Txbuff[0]) , HAL_MAX_DELAY); //AMG88xx_NORMAL_MODE;
	MemAddress = AMG88xx_RST;    //RST
	Txbuff[0]=0x3F;
	HAL_I2C_Mem_Write(&i2c2,DevAddress,MemAddress, sizeof(MemAddress),Txbuff, sizeof(Txbuff[0]) , HAL_MAX_DELAY);  //AMG88xx_INITIAL_RESET;
	MemAddress = AMG88xx_INTC;    //INTC
	Txbuff[0]=AMG88xx_INT_DISABLED;
	HAL_I2C_Mem_Write(&i2c2,DevAddress,MemAddress, sizeof(MemAddress),Txbuff, sizeof(Txbuff[0]) , HAL_MAX_DELAY); //disable INTC;
	MemAddress = AMG88xx_FPSC;    //FRR
	Txbuff[0]=AMG88xx_FPS_10;
	HAL_I2C_Mem_Write(&i2c2,DevAddress,MemAddress, sizeof(MemAddress),Txbuff, sizeof(Txbuff[0]) , HAL_MAX_DELAY); //AMG88xx_FPS_10;
	HAL_Delay(100);
}


void AMG8833_Read_pixdels(float *buf)
{
	uint16_t recast;
	float converted;
	const uint8_t bytesToRead = 128;
	uint8_t rawArray[bytesToRead];
	Read(rawArray);

	while((rawArray[0]==0)&&(rawArray[1]==0)&&(rawArray[2]==0)&&(rawArray[3]==0))
	{
	  Read(rawArray);
	}

	//puts("\n rawArry data is \n ");
	for(int i=0; i<64; i++){
		uint8_t pos = i << 1;
		//printf("L(%d)C(%d)-%02X %02X\t",i/8,i%8,rawArray[pos + 1],rawArray[pos]);
		recast = ((uint16_t)rawArray[pos + 1] << 8) | ((uint16_t)rawArray[pos]);
		converted = int12ToFloat(recast) * 0.25;
		buf[(8*((63-i)/8))+(i%8)] = converted;
	}
	//puts("\n");
}

uint16_t AMG8833_Read_thermistor(void )
{
	 MemAddress = 0x0E;    //AMG88xx_TTHL
	 thermistor=0;
	 HAL_I2C_Mem_Read(&i2c2, DevAddress, MemAddress,  sizeof(MemAddress) , (uint8_t*)&thermistor, sizeof(thermistor), HAL_MAX_DELAY);
	 return thermistor;
}

void Set_interrupt(float upper, float  lower)
{

	volatile int upperlimit = (upper * 4);//*quarter_deg;
	volatile int lowerlimit = (lower * 4);//*quarter_deg;
	//volatile int hys_level  = ((upper*0.95)* 4);

	Txbuff[0]=upperlimit&0xFF;
	Txbuff[1]=((upperlimit&0x0F00) >> 8);

    MemAddress = AMG88xx_INTHL;
	HAL_I2C_Mem_Write(&i2c2,DevAddress,MemAddress, sizeof(MemAddress), &Txbuff[0], 1, HAL_MAX_DELAY);
	MemAddress = AMG88xx_INTHH;
	HAL_I2C_Mem_Write(&i2c2,DevAddress,MemAddress, sizeof(MemAddress), &Txbuff[1], 1, HAL_MAX_DELAY);

	Txbuff[0]=lowerlimit&0xFF;
	Txbuff[1]=((lowerlimit&0x0F00) >> 8);
	MemAddress = AMG88xx_INTLL;
	HAL_I2C_Mem_Write(&i2c2,DevAddress,MemAddress, sizeof(MemAddress), &Txbuff[0], 1, HAL_MAX_DELAY);
	MemAddress = AMG88xx_INTLH;
	HAL_I2C_Mem_Write(&i2c2,DevAddress,MemAddress, sizeof(MemAddress), &Txbuff[1], 1, HAL_MAX_DELAY);

	// Txbuff[0]=hys_level&0xFF;
	// Txbuff[1]=((hys_level&0x0F00) >> 8);
	// MemAddress = AMG88xx_IHYSL;
	//HAL_I2C_Mem_Write(&i2c2,DevAddress,MemAddress, sizeof(MemAddress), &Txbuff[0], 1, HAL_MAX_DELAY);
	//MemAddress = AMG88xx_IHYSH;
	//HAL_I2C_Mem_Write(&hi2c3,DevAddress,MemAddress, sizeof(MemAddress), &Txbuff[1], 1, HAL_MAX_DELAY);
}

void Enable_interrupt(void)
{
	MemAddress = AMG88xx_INTC;
	Txbuff[0]=0x03;//AMG88xx_ABSOLUTE_VALUE<<1;  // enable INT output and Absolute INT mode
	HAL_I2C_Mem_Write(&i2c2,DevAddress,MemAddress, sizeof(MemAddress), Txbuff, sizeof(Txbuff[0]) , HAL_MAX_DELAY);
}

void Clear_interrupt(void)
{
	MemAddress = AMG88xx_RST;
    Txbuff[0]  =  AMG88xx_FLAG_RESET ;
    HAL_I2C_Mem_Write(&i2c2,DevAddress,MemAddress, sizeof(MemAddress), Txbuff, sizeof(Txbuff[0]) , HAL_MAX_DELAY);
}

void Disable_interrupt(void)
{
	MemAddress = AMG88xx_INTC;
	Txbuff[0]=AMG88xx_INT_DISABLED;  // enable INT output and Absolute INT mode
	HAL_I2C_Mem_Write(&i2c2,DevAddress,MemAddress, sizeof(MemAddress), Txbuff, sizeof(Txbuff[0]) , HAL_MAX_DELAY);
}

void getInterrupt(uint8_t *buf)
{
	uint8_t bytesToRead = 8;
	MemAddress=AMG88xx_INT_OFFSET;
	HAL_I2C_Mem_Read(&i2c2, DevAddress, MemAddress,  sizeof(MemAddress) , buf,  bytesToRead , HAL_MAX_DELAY);//sizeof(pixdata)
}

/**************************************************************************/
/*!
    @brief  convert a 12-bit integer two's complement value to a floating point number
    @param  val the 12-bit integer  two's complement value to be converted
    @returns the converted floating point value
*/
/**************************************************************************/

static float int12ToFloat(uint16_t val)
{
	int16_t sVal = (val << 4); //shift to left so that sign bit of 12 bit integer number is placed on sign bit of 16 bit signed integer number
	return sVal >> 4;          //shift back the signed number, return converts to float
}

static void Read(uint8_t *buf)
{
	MemAddress = 0x80;    //AMG88xx_T01L
	HAL_I2C_Mem_Read(&i2c2, DevAddress, MemAddress,  sizeof(MemAddress) , (uint8_t*)buf,  128 , HAL_MAX_DELAY);//sizeof(pixdata)
	
}

static void I2C2_initaliztion(void)
{

    GpioInit( &I2C2_SDA, PB_11, PIN_ALTERNATE_FCT, PIN_OPEN_DRAIN, PIN_NO_PULL, GPIO_AF4_I2C2 );
    GpioInit( &I2C2_SCL, PB_10, PIN_ALTERNATE_FCT, PIN_OPEN_DRAIN, PIN_NO_PULL, GPIO_AF4_I2C2 );
    __HAL_RCC_I2C2_FORCE_RESET( );
    __HAL_RCC_I2C2_RELEASE_RESET( );
	__HAL_RCC_I2C2_CLK_ENABLE( );
    i2c2.Instance=I2C2;
    i2c2.Init.Timing=0x0010061A;
    i2c2.Init.OwnAddress1 = 0;
    i2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    i2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    i2c2.Init.OwnAddress2 = 0;
	i2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    i2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
    i2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
    HAL_I2C_Init( &i2c2 );
}




