#include "adxl375_rp.h"

ADXL375_RP::ADXL375_RP(SPIClass &spi, uint8_t cs, DeviceFrequency device_frequency)
    : _spi(&spi), _cs(cs), _device_frequency_code(device_frequency) {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);

    switch (_device_frequency_code) {
    case ADXL375_RATE_3200HZ:
        _millis_between_entries = static_cast<float>(1E6) / 3200;
        break;
    case ADXL375_RATE_1600HZ:
        _millis_between_entries = static_cast<float>(1E6) / 1600;
        break;
    case ADXL375_RATE_800HZ:
        _millis_between_entries = static_cast<float>(1E6) / 800;
        break;
    case ADXL375_RATE_400HZ:
        _millis_between_entries = static_cast<float>(1E6) / 400;
        break;
    case ADXL375_RATE_200HZ:
        _millis_between_entries = static_cast<float>(1E6) / 200;
        break;
    case ADXL375_RATE_100HZ:
        _millis_between_entries = static_cast<float>(1E6) / 100;
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

    // right justify data, so that data is aligned to LSB
    _write_register(
        ADXL375_REG_DATA_FORMAT, ADXL375_SELF_TEST_OFF | ADXL375_SPI_4_WIRE | ADXL375_JUSTIFY
    );
    if (_read_register_single(ADXL375_REG_DATA_FORMAT) !=
        (ADXL375_SELF_TEST_OFF | ADXL375_SPI_4_WIRE | ADXL375_JUSTIFY)) {
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

size_t ADXL375_RP::read(ADXL375_RP_Reading read_buf[], int32_t time_offset) {
    // first get the number of entires in the FIFO
    uint8_t num_entries =
        (_read_register_single(ADXL375_REG_FIFO_STATUS) & ADXL375_FIFO_ENTRIES_MASK);

    if (num_entries > ADXL375_FIFO_MAX_ENTRIES) {
        num_entries = ADXL375_FIFO_MAX_ENTRIES;
    }

    float start_timestamp = micros() - num_entries * _millis_between_entries + time_offset;
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

        read_buf[i] = {
            .x = (static_cast<int16_t>(dx1 << 8) | dx0) * ADXL375_MG2G_MULTIPLIER *
                 SENSORS_GRAVITY_STANDARD,
            .y = (static_cast<int16_t>(dy1 << 8) | dy0) * ADXL375_MG2G_MULTIPLIER *
                 SENSORS_GRAVITY_STANDARD,
            .z = (static_cast<int16_t>(dz1 << 8) | dz0) * ADXL375_MG2G_MULTIPLIER *
                 SENSORS_GRAVITY_STANDARD,
            .timestamp = static_cast<unsigned long>(start_timestamp)
        };

        start_timestamp += _millis_between_entries;
    }

    return num_entries;
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