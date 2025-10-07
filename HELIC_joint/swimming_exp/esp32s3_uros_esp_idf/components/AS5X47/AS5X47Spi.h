/** @file spi_t.h
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

#ifndef AS5X47_SPI_H
#define AS5X47_SPI_H

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"


// SPI handle structure
// typedef struct {
//     gpio_num_t cs_pin;
//     spi_device_handle_t spi_handle;
// } spi_t;

typedef struct {
  spi_device_handle_t spiHandle;
  gpio_num_t csPin;
  spi_device_interface_config_t devCfg;
  spi_host_device_t spi;   // Add this member
  spi_bus_config_t busCfg;
} spi_t;

// Function prototypes
// void Spi_init(spi_t* spi, spi_host_device_t spi_host, gpio_num_t chip_select);
void Spi_writeData(spi_t* spi, uint16_t command, uint16_t value);
uint16_t Spi_readData(spi_t* spi, uint16_t command, uint16_t nopCommand);

#endif  // AS5X47_SPI_H
