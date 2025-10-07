#include "AS5X47Spi.h"
#define TAG "SPI"

void Spi_writeData(spi_t* spi, uint16_t command, uint16_t value) {
    spi_transaction_t t = {};
    t.length = 16;  // Sending 16 bits
    t.tx_buffer = &command;

    // Send command
    gpio_set_level(spi->csPin, 0); // CS LOW
    ESP_ERROR_CHECK(spi_device_transmit(spi->spiHandle, &t));
    gpio_set_level(spi->csPin, 1); // CS HIGH

    vTaskDelay(pdMS_TO_TICKS(1));

    // Send value
    
    t.tx_buffer = &value;
    gpio_set_level(spi->csPin, 0); // CS LOW
    ESP_ERROR_CHECK(spi_device_transmit(spi->spiHandle, &t));
    gpio_set_level(spi->csPin, 1); // CS HIGH

    vTaskDelay(pdMS_TO_TICKS(1));
}

uint16_t Spi_readData(spi_t* spi, uint16_t command, uint16_t nopCommand) {
    spi_transaction_t t = {};
    uint16_t receivedData = 0;

    // Send Read Command
    t.length = 16;
    command = (command<<8) | (command>>8);
    t.tx_buffer = &command;
    gpio_set_level(spi->csPin, 0);
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi->spiHandle, &t));
    gpio_set_level(spi->csPin, 1);

    // vTaskDelay(pdMS_TO_TICKS(1));

    // Send NOP Command while receiving data
    nopCommand = (nopCommand<<8) | (nopCommand>>8);
    t.tx_buffer = &nopCommand;
    t.rx_buffer = &receivedData;
    gpio_set_level(spi->csPin, 0);
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi->spiHandle, &t));
    gpio_set_level(spi->csPin, 1);

    // vTaskDelay(pdMS_TO_TICKS(1));

    int freq;
    spi_device_get_actual_freq(spi->spiHandle, &freq);

    return (receivedData<<8) | (receivedData>>8);
}
