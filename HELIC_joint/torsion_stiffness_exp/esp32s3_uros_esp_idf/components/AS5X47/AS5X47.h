/** @file AS5X47.h
 *
 * @brief A library for Arduino boards that reads angles from AS5047 and AS5147 sensors.
 * 		  Also support configuration of the sensor parameters.
 *
 * @par
 * COPYRIGHT NOTICE: MIT License
 *
 * 	Copyright (c) 2020 Adrien Legrand <contact@adrien-legrand.com>
 *
 * 	Permission is hereby granted, free of charge, to any person obtaining a copy
 * 	of this software and associated documentation files (the "Software"), to deal
 * 	in the Software without restriction, including without limitation the rights
 * 	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * 	copies of the Software, and to permit persons to whom the Software is
 * 	furnished to do so, subject to the following conditions:
 *
 * 	The above copyright notice and this permission notice shall be included in all
 * 	copies or substantial portions of the Software.
 *
 * 	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * 	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * 	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * 	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * 	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * 	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * 	SOFTWARE.
 *
*/


#ifndef AS5X47_H
#define AS5X47_H

#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "AS5X47Spi.h"



// Volatile Registers Addresses
#define NOP_REG           0x0000
#define ERRFL_REG         0x0001
#define PROG_REG          0x0003
#define DIAGAGC_REG       0x3FFC
#define MAG_REG           0x3FFD
#define ANGLE_REG         0x3FFE
#define ANGLECOM_REG      0x3FFF

// Non-Volatile Registers Addresses
#define ZPOSM_REG         0x0016
#define ZPOSL_REG         0x0017
#define SETTINGS1_REG     0x0018
#define SETTINGS2_REG     0x0019

#define WRITE             0
#define READ              1

#define _2PI 6.28318530718f


// Error Flags Register Definition
typedef union {
    uint16_t raw;
    struct __attribute__ ((packed)) {
        uint16_t frerr:1;
        uint16_t invcomm:1;
        uint16_t parerr:1;
        uint16_t unused:13;
    } values;
} Errfl;

// Program Register Definition
typedef union {
    uint16_t raw;
    struct __attribute__ ((packed)) {
        uint16_t progen:1;
        uint16_t unused:1;
        uint16_t otpref:1;
        uint16_t progotp:1;
        uint16_t unused1:2;
        uint16_t progver:1;
        uint16_t unused2:9;
    } values;
} Prog;

// DIAGAGC Register Definition
typedef union {
    uint16_t raw;
    struct __attribute__ ((packed)) {
        uint16_t agc:8;
        uint16_t lf:1;
        uint16_t cof:1;
        uint16_t magh:1;
        uint16_t magl:1;
        uint16_t unused:4;
    } values;
} Diaagc;

// Magnetic Register Definition
typedef union {
    uint16_t raw;
    struct __attribute__ ((packed)) {
        uint16_t cmag:14;
        uint16_t unused:2;
    } values;
} Mag;

// Angle Register Definition
typedef union {
    uint16_t raw;
    struct __attribute__ ((packed)) {
        uint16_t cordicang:14;
        uint16_t unused:2;
    } values;
} Angle;

// Angle Communication Register Definition
typedef union {
    uint16_t raw;
    struct __attribute__ ((packed)) {
        uint16_t daecang:14;
        uint16_t unused:2;
    } values;
} Anglecom;

// Zero Position Register (MSB) Definition
typedef union {
    uint8_t raw;
    struct __attribute__ ((packed)) {
        uint8_t zposm;
    } values;
} Zposm;

// Zero Position Register (LSB) Definition
typedef union {
    uint8_t raw;
    struct __attribute__ ((packed)) {
        uint8_t zposl:6;
        uint8_t compLerrorEn:1;
        uint8_t compHerrorEn:1;
    } values;
} Zposl;

// Settings 1 Register Definition
typedef union {
    uint8_t raw;
    struct __attribute__ ((packed)) {
        uint8_t factorySetting:1;
        uint8_t noiseset:1;
        uint8_t dir:1;
        uint8_t uvw_abi:1;
        uint8_t daecdis:1;
        uint8_t abibin:1;
        uint8_t dataselect:1;
        uint8_t pwmon:1;
    } values;
} Settings1;

// Settings 2 Register Definition
typedef union {
    uint8_t raw;
    struct __attribute__ ((packed)) {
        uint8_t uvwpp:3;
        uint8_t hys:2;
        uint8_t abires:3;
    } values;
} Settings2;

// Command Frame Definition
typedef union {
    uint16_t raw;
    struct __attribute__ ((packed)) {
        uint16_t commandFrame:14;
        uint16_t rw:1;
        uint16_t parc:1;
    } values;
} CommandFrame;

// Read Data Frame Definition
typedef union {
    uint16_t raw;
    struct __attribute__ ((packed)) {
        uint16_t data:14;
        uint16_t ef:1;
        uint16_t pard:1;
    } values;
} ReadDataFrame;

// Write Data Frame Definition
typedef union {
    uint16_t raw;
    struct __attribute__ ((packed)) {
        uint16_t data:14;
        uint16_t low:1;
        uint16_t pard:1;
    } values;
} WriteDataFrame;


// Function Prototypes
esp_err_t AS5X47_init(spi_t *device, uint8_t spi, int mosiPin, int misoPin, int clkPin, int chipSelectPin, int clk_Hz);

void AS5X47_deinit(spi_t *device);
ReadDataFrame AS5X47_readRegister(spi_t *device, uint16_t registerAddress);
void AS5X47_writeRegister(spi_t *device, uint16_t registerAddress, uint16_t registerValue);
void AS5X47_writeSettings1(spi_t *device, Settings1 values);
void AS5X47_writeSettings2(spi_t *device, Settings2 values);
void AS5X47_writeZeroPosition(spi_t *device, Zposm zposm, Zposl zposl);

float AS5X47_readAngle(spi_t *device);
bool isEven(uint16_t data);

#endif // AS5X47_H
