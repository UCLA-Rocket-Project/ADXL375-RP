#ifndef ADXL_375_H
#define ADXL_375_H

#include <Arduino.h>
#include <SPI.h>
#include <stdint.h>

#define ADXL375_REG_DEVID            0x00 // Device ID
#define ADXL375_REG_THRESH_SHOCK     0x1D // Shock threshold
#define ADXL375_REG_OFSX             0x1E // X-axis offset
#define ADXL375_REG_OFSY             0x1F // Y-axis offset
#define ADXL375_REG_OFSZ             0x20 // Z-axis offset
#define ADXL375_REG_DUR              0x21 // Shock duration
#define ADXL375_REG_LATENT           0x22 // Shock latency
#define ADXL375_REG_WINDOW           0x23 // Shock window
#define ADXL375_REG_THRESH_ACT       0x24 // Activity threshold
#define ADXL375_REG_THRESH_INACT     0x25 // Inactivity threshold
#define ADXL375_REG_TIME_INACT       0x26 // Inactivity time
#define ADXL375_REG_ACT_INACT_CTL    0x27 // Axis enable control for activity/inactivity detection
#define ADXL375_REG_SHOCK_AXES       0x2A // Axis control for single shock/double shock
#define ADXL375_REG_ACT_SHOCK_STATUS 0x2B // Source of single shock/double shock
#define ADXL375_REG_BW_RATE          0x2C // Data rate and power mode control
#define ADXL375_REG_POWER_CTL        0x2D // Power saving features control
#define ADXL375_REG_INT_ENABLE       0x2E // Interrupt enable control
#define ADXL375_REG_INT_MAP          0x2F // Interrupt mapping control
#define ADXL375_REG_INT_SOURCE       0x30 // Interrupt source
#define ADXL375_REG_DATA_FORMAT      0x31 // Data format control
#define ADXL375_REG_DATAX0           0x32 // X-Axis Data 0
#define ADXL375_REG_DATAX1           0x33 // X-Axis Data 1
#define ADXL375_REG_DATAY0           0x34 // Y-Axis Data 0
#define ADXL375_REG_DATAY1           0x35 // Y-Axis Data 1
#define ADXL375_REG_DATAZ0           0x36 // Z-Axis Data 0
#define ADXL375_REG_DATAZ1           0x37 // Z-Axis Data 1
#define ADXL375_REG_FIFO_CTL         0x38 // FIFO control
#define ADXL375_REG_FIFO_STATUS      0x39 // FIFO status

#define ADXL375_DEVICE_ID            0xE5
#define SPI_FREQUENCY                5000000 // max frequency is 5Mhz

#define ADXL375_CMD_READ             (1UL << 7)
#define ADXL375_CMD_WRITE            (0UL << 7)
#define ADXL375_MULTIBYTE            (1UL << 6)
#define ADXL375_NON_MULTIBYTE        (0UL << 6)

#define ADXL375_SELF_TEST_OFF        (0UL << 7)
#define ADXL375_SPI_4_WIRE           (0UL << 6)
#define ADXL375_JUSTIFY              (0UL << 2)

#define ADXL375_FIFO_MODE_STREAM     (0b10 << 6)

#define ADXL375_FIFO_ENTRIES_MASK    (0b111111)

#define ADXL375_FIFO_MAX_ENTRIES     33

#define ADXL375_POWER_CTL_MEASURE    (1 << 3)

#define ADXL375_MG2G_MULTIPLIER      (0.049F)
#define SENSORS_GRAVITY_EARTH        (9.80665F)
#define SENSORS_GRAVITY_STANDARD     (SENSORS_GRAVITY_EARTH)

struct ADXL375_RP_Reading {
    float x;
    float y;
    float z;
    unsigned long timestamp;
};

class ADXL375_RP {
  public:
    enum DeviceFrequency : uint8_t {
        ADXL375_RATE_3200HZ = 0x0F,
        ADXL375_RATE_1600HZ = 0x0E,
        ADXL375_RATE_800HZ = 0x0D,
        ADXL375_RATE_400HZ = 0x0C,
        ADXL375_RATE_200HZ = 0x0B,
        ADXL375_RATE_100HZ = 0x0A,
    };

    ADXL375_RP(SPIClass &spi, uint8_t cs, DeviceFrequency device_frequency);

    bool begin();

    size_t read(ADXL375_RP_Reading read_buf[], int32_t time_offset = 0);

    void read_single(ADXL375_RP_Reading &reading, int32_t time_offset = 0);

  private:
    SPIClass *_spi;
    SPISettings _spi_settings{SPI_FREQUENCY, MSBFIRST, SPI_MODE3};

    uint8_t _cs;
    uint8_t _device_frequency_code;
    float _millis_between_entries;

    uint8_t _read_register_single(uint8_t address);
    void _write_register(uint8_t address, uint8_t value);
};

#endif