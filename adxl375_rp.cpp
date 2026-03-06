#include "adxl375_rp.h"
#include <cstdint>

ADXL375_RP::ADXL375_RP(SPIClass &spi, uint8_t cs, DeviceFrequency device_frequency)
    : _spi(&spi), _cs(cs), _device_frequency_code(device_frequency) {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);

    switch (_device_frequency_code) {
    // NOTE: the way we are setting this causes some accuracy to be lost with timestamps when using
    // 3200Hz, but we dont use that
    case ADXL375_RATE_3200HZ:
        _micros_between_entries = 1E6 / 3200;
        break;
    case ADXL375_RATE_1600HZ:
        _micros_between_entries = 1E6 / 1600;
        break;
    case ADXL375_RATE_800HZ:
        _micros_between_entries = 1E6 / 800;
        break;
    case ADXL375_RATE_400HZ:
        _micros_between_entries = 1E6 / 400;
        break;
    case ADXL375_RATE_200HZ:
        _micros_between_entries = 1E6 / 200;
        break;
    case ADXL375_RATE_100HZ:
        _micros_between_entries = 1E6 / 100;
        break;
    }
}

bool ADXL375_RP::begin() {
    // check that device ID is correct as a check that SPI is working
    if (_read_register_single(ADXL375_REG_DEVID) != ADXL375_DEVICE_ID) {
        return false;
    }

    // configure device frequency
    _write_register(ADXL375_REG_BW_RATE, _device_frequency_code);
    if (_read_register_single(ADXL375_REG_BW_RATE) != _device_frequency_code) {
        return false;
    }

    // configure the FIFO to use stream mode
    _write_register(ADXL375_REG_FIFO_CTL, ADXL375_FIFO_MODE_STREAM);
    if (_read_register_single(ADXL375_REG_FIFO_CTL) != ADXL375_FIFO_MODE_STREAM) {
        return false;
    }

    _write_register(ADXL375_REG_POWER_CTL, ADXL375_POWER_CTL_MEASURE);
    if (_read_register_single(ADXL375_REG_POWER_CTL) != ADXL375_POWER_CTL_MEASURE) {
        return false;
    }

    return true;
}

size_t ADXL375_RP::read(ADXL375_RP_Reading read_buf[], int64_t current_time) {
    // first get the number of entires in the FIFO
    uint8_t num_entries =
        (_read_register_single(ADXL375_REG_FIFO_STATUS) & ADXL375_FIFO_ENTRIES_MASK);

    if (num_entries > ADXL375_FIFO_MAX_ENTRIES) {
        num_entries = ADXL375_FIFO_MAX_ENTRIES;
    }

    int64_t start_timestamp =
        current_time - (static_cast<int64_t>(num_entries) * _micros_between_entries);
    for (uint8_t i = 0; i < num_entries; i++) {
        _spi->beginTransaction(_spi_settings);
        digitalWrite(_cs, LOW);

        _spi->transfer(ADXL375_CMD_READ | ADXL375_MULTIBYTE | ADXL375_REG_DATAX0);
        uint8_t dx0 = _spi->transfer(0x0);
        uint8_t dx1 = _spi->transfer(0x0);
        uint8_t dy0 = _spi->transfer(0x0);
        uint8_t dy1 = _spi->transfer(0x0);
        uint8_t dz0 = _spi->transfer(0x0);
        uint8_t dz1 = _spi->transfer(0x0);

        digitalWrite(_cs, HIGH);
        _spi->endTransaction();

        // NOTE: here, the bit shift operation always promotes the result to an int
        // As much as possible, always cast back to an int16_t so that the bit pattern
        // is interpreted properly
        int16_t x_raw = (static_cast<uint16_t>(dx1) << 8 | static_cast<uint16_t>(dx0));
        int16_t y_raw = (static_cast<uint16_t>(dy1) << 8 | static_cast<uint16_t>(dy0));
        int16_t z_raw = (static_cast<uint16_t>(dz1) << 8 | static_cast<uint16_t>(dz0));

        read_buf[i] = {
            .x = x_raw * ADXL375_MULTIPLICATION_FACTOR,
            .y = y_raw * ADXL375_MULTIPLICATION_FACTOR,
            .z = z_raw * ADXL375_MULTIPLICATION_FACTOR,
            .timestamp = static_cast<int64_t>(start_timestamp)
        };

        start_timestamp += _micros_between_entries;
    }

    return num_entries;
}

void ADXL375_RP::read_single(ADXL375_RP_Reading &reading, int32_t time_offset) {
    _spi->beginTransaction(_spi_settings);
    digitalWrite(_cs, LOW);

    _spi->transfer(ADXL375_CMD_READ | ADXL375_MULTIBYTE | ADXL375_REG_DATAX0);
    uint8_t dx0 = _spi->transfer(0x0);
    uint8_t dx1 = _spi->transfer(0x0);
    uint8_t dy0 = _spi->transfer(0x0);
    uint8_t dy1 = _spi->transfer(0x0);
    uint8_t dz0 = _spi->transfer(0x0);
    uint8_t dz1 = _spi->transfer(0x0);

    digitalWrite(_cs, HIGH);
    _spi->endTransaction();

    reading.x = static_cast<int16_t>(static_cast<int16_t>(dx1) << 8 | static_cast<int16_t>(dx0)) *
                ADXL375_MULTIPLICATION_FACTOR;
    reading.y = static_cast<int16_t>(static_cast<int16_t>(dy1) << 8 | static_cast<int16_t>(dy0)) *
                ADXL375_MULTIPLICATION_FACTOR;
    reading.z = static_cast<int16_t>(static_cast<int16_t>(dz1) << 8 | static_cast<int16_t>(dz0)) *
                ADXL375_MULTIPLICATION_FACTOR;

    // this is not super accurate, but its just for a sample, so I think it does not matter
    reading.timestamp += micros() + time_offset;

    return;
}

void ADXL375_RP::configure_offsets(float x, float y, float z) {
    int8_t x_offset = x / ADXL375_OFFSET_PER_LSB;
    _write_register(ADXL375_REG_OFSX, x_offset);

    int8_t y_offset = y / ADXL375_OFFSET_PER_LSB;
    _write_register(ADXL375_REG_OFSY, y_offset);

    int8_t z_offset = z / ADXL375_OFFSET_PER_LSB;
    _write_register(ADXL375_REG_OFSZ, z_offset);
}

uint8_t ADXL375_RP::_read_register_single(uint8_t address) {
    // datasheet says you need 5ns of time, but the core's speed causes consecutive instructions to
    // execute > 5ns apart, so we are good
    _spi->beginTransaction(_spi_settings);
    digitalWrite(_cs, LOW);

    _spi->transfer(ADXL375_CMD_READ | ADXL375_NON_MULTIBYTE | address);
    uint8_t ret = _spi->transfer(0x0);
    digitalWrite(_cs, HIGH);
    _spi->endTransaction();

    return ret;
}

void ADXL375_RP::_write_register(uint8_t address, uint8_t value) {
    // datasheet says you need 5ns of time, but the core's speed causes consecutive instructions to
    // execute > 5ns apart, so we are good
    _spi->beginTransaction(_spi_settings);
    digitalWrite(_cs, LOW);

    _spi->transfer(ADXL375_CMD_WRITE | ADXL375_NON_MULTIBYTE | address);
    _spi->transfer(value);
    digitalWrite(_cs, HIGH);
    _spi->endTransaction();
}