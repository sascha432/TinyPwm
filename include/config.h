// /**
//   Author: sascha_lammers@gmx.de
// */

// #pragma once

// #include <Arduino.h>
// #include <EEPROM.h>
// #include <crc16.h>
// #include "reg_mem.h"

// namespace Config {

//     static constexpr uint32_t kHeaderMagic = 0x8586c952;

//     struct __attribute__((__packed__)) Config {

//         uint32_t version;
//         uint32_t magic;
//         uint8_t cycles[TinyPwm::kAnalogPinCount];
//         uint8_t pwmStartValue;
//         int8_t adcOffset;
//         int16_t adcGainError;

//         Config() : version(0), magic(kHeaderMagic), cycles{}, pwmStartValue(0), adcOffset(0), adcGainError(0) {}
//     };

//     class Data {
//     public:
//         Data() : _crc(~0), _config() {}

//     private:
//         struct __attribute__((__packed__)) {
//             uint16_t _crc;
//             Config _config;
//         };
//     };

//     class Header {
//     public:
//         Header() : _magic(kHeaderMagic), _length(sizeof(Data))
//         {
//             _crc = crc();
//         }

//         uint16_t crc() const {
//             uint16_t crc =  crc16_update(&_magic, sizeof(_magic));
//             crc =  crc16_update(crc, _length);
//             return crc;
//         }

//     private:
//         struct __attribute__((__packed__)) {
//             uint16_t _crc;
//             uint32_t _magic;
//             uint8_t _length;
//         };
//     };

//     static constexpr auto kHeaderSize = sizeof(Header);

//     class Base {
//     public:
//         Base() :
//             _headers{0, EEPROM.length() / 3, EEPROM.length() * 2 / 3, EEPROM.length() - sizeof(Header)}
//         {
//         }

//         void format();

//     protected:
//         uint16_t getOffset() {}

//     protected:
//         uint16_t _headers[4];
//     };
// }