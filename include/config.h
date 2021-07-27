/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <Arduino.h>
#include <crc16.h>
#include "main.h"

namespace Config {

    static constexpr uint32_t kHeaderMagic = 0x8586c953;

    struct __attribute__((__packed__)) Header {

        uint16_t crc;
        uint32_t magic;

        Header() : crc(0), magic(0) {}
    };

    struct __attribute__((__packed__)) Arrays {
        uint8_t cycles[TinyPwm::kPinCount];
        uint8_t pinModes[TinyPwm::kPinCount];
        uint8_t values[TinyPwm::kPinCount];
    };

    struct __attribute__((__packed__)) Data {

        uint32_t version;
        uint8_t address;
        int8_t adcOffset;
        int16_t adcGainError;
        Arrays arrays;

        Data() :
            version(1),
            address(TINYPWM_I2C_SLAVE_ADDRESS),
            adcOffset(TINYPWM_ADC_OFFSET),
            adcGainError(TINYPWM_GAIN_ERROR),
            arrays({{ TINYPWM_ANALOG_CONVERSIONS }, { TINYPWM_PIN_MODES }, { TINYPWM_VALUES }})
        {}
    };

    static constexpr auto kHeaderSize = sizeof(Header);
    static constexpr auto kDataSize = sizeof(Data);

    class __attribute__((__packed__)) Config {
    public:

        void clear();

        uint16_t crc() const {
            return  crc16_update(&_data, sizeof(_data));
        }

        bool read(uint16_t &position);
        void write(uint16_t &position);

        const Data &data() const {
            return _data;
        }

        Data &data() {
            return _data;
        }

    private:
        Header _header;
        Data _data;
    };

}