#include "AS5X47.h"
#include "AS5X47Spi.h"
#include <stdbool.h>

/** @file AS5X47.cpp
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


// Initialize the AS5X47 device with the provided chip select pin.
esp_err_t AS5X47_init(spi_t *device, uint8_t spi, int mosiPin, int misoPin, int clkPin, int chipSelectPin, int clk_Hz) 
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << chipSelectPin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    device->spi = spi;
    device->csPin = chipSelectPin;
    spi_bus_config_t busCfg = {
        .miso_io_num = misoPin,
        .mosi_io_num = mosiPin,
        .sclk_io_num = clkPin,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .flags = SPICOMMON_BUSFLAG_MASTER ,     /*| SPICOMMON_BUSFLAG_IOMUX_PINS ;*/
        .intr_flags = 0,
        .max_transfer_sz = 1,

    };
    spi_device_interface_config_t devCfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = (uint8_t) 1,                //SPI_MODE;
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = (uint8_t) 0,    //CS_ENA_POSTTRANS;
        .clock_speed_hz = clk_Hz,       //SPI_HZ;
        .input_delay_ns = 0,               // INPUT_DELAY_NS;
        .spics_io_num = chipSelectPin,
        .flags = SPI_DEVICE_NO_DUMMY,
        .queue_size = 1,
        .pre_cb = 0,
        .post_cb = 0,
    };



    spi_bus_initialize(device->spi, &busCfg, 0);
    spi_bus_add_device(device->spi, &devCfg, &(device->spiHandle));
    gpio_config(&io_conf);
    gpio_set_level(chipSelectPin, 1);  
    return ESP_OK;
}

// De-initialize the AS5X47 device and free the SPI resources.
void AS5X47_deinit(spi_t *device) {
    spi_bus_remove_device(device->spiHandle);
    spi_bus_free(device->spi);
}

// Read a register from the AS5X47 device.
ReadDataFrame AS5X47_readRegister(spi_t *device, uint16_t registerAddress) 
{
    CommandFrame command;
    command.values.rw = READ;
    command.values.commandFrame = registerAddress;
    command.values.parc = isEven(command.raw);

    CommandFrame nopCommand;
    nopCommand.values.rw = READ;
    nopCommand.values.commandFrame = NOP_REG;
    nopCommand.values.parc = isEven(nopCommand.raw);

    ReadDataFrame receivedFrame;
    receivedFrame.raw = Spi_readData(device, command.raw, nopCommand.raw);
    return receivedFrame;
}

// Write a value to a register on the AS5X47 device.
void AS5X47_writeRegister(spi_t *device, uint16_t registerAddress, uint16_t registerValue) {
    CommandFrame command;
    command.values.rw = WRITE;
    command.values.commandFrame = registerAddress;
    command.values.parc = isEven(command.raw);

    WriteDataFrame contentFrame;
    contentFrame.values.data = registerValue;
    contentFrame.values.low = 0;
    contentFrame.values.pard = isEven(contentFrame.raw);

    Spi_writeData(device, command.raw, contentFrame.raw);
}

// Read the angle value from the AS5X47 device.
float AS5X47_readAngle(spi_t *device) {
    ReadDataFrame readDataFrame = AS5X47_readRegister(device, ANGLE_REG);
    Angle angle;
    angle.raw = readDataFrame.values.data;
    return angle.values.cordicang / 16384.0f * 360.0f;
}

// Write the SETTINGS1 register.
void AS5X47_writeSettings1(spi_t *device, Settings1 values) {
    AS5X47_writeRegister(device, SETTINGS1_REG, values.raw);
}

// Write the SETTINGS2 register.
void AS5X47_writeSettings2(spi_t *device, Settings2 values) {
    AS5X47_writeRegister(device, SETTINGS2_REG, values.raw);
}

// Write zero position data.
void AS5X47_writeZeroPosition(spi_t *device, Zposm zposm, Zposl zposl) {
    AS5X47_writeRegister(device, ZPOSM_REG, zposm.raw);
    AS5X47_writeRegister(device, ZPOSL_REG, zposl.raw);
}



// Check if the data is even.
bool isEven(uint16_t data) {
    int count = 0;
    for (unsigned int i = 0; i < 15; i++) 
    {
        if (data & (1 << i)) 
        {
            count++;
        }
    }
    return count % 2 != 0;
}
