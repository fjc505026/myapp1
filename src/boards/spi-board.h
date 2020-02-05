/*!
 * \file      spi-board.h
 *
 * \brief     Target board SPI driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#ifndef __SPI_BOARD_H__
#define __SPI_BOARD_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "spi.h"

// An Spi.c file has to be implmented under system directory.

void writeDatBytes(uint8_t* pDat, uint16_t count);
void writeCmd(uint8_t cmd);
void writeDat(uint8_t dat);
#ifdef __cplusplus
}
#endif

#endif // __SPI_BOARD_H__
