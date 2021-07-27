/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <Arduino.h>

// I2C protocol
// ------------------------------------------------------------------------
//
// to slave
//
// <slave-address> <Command> <Command-struct> [<Command> <Command-struct> ...]
//
// from slave
//
// <Response-struct> [<Response-struct> ...]
//
//
// Default pins
// ------------------------------------------------------------------------
//
// PB0 / pin 5 = SDA
// PB2 / pin 7 = SCL
// PB1 / pin 6 = PWM output
// PB3 / pin 2 = ADC #1
// PB4 / pin 3 = ADC #2
//
//

/*
// examples for KFC firmware
// ------------------------------------------------------------------------

scan i2c bus for devices

+i2cscan

set pwm on pin 0 (PB1)

+i2ctm=0x60,0x10,0x00,0x40
+i2ctm=0x60,0x10,0x00,0x20
+i2ctm=0x60,0x10,0x00,0x00

set analog reference 2.56V

+i2ctm=0x60,0x62,0x06

set analog reference 1.1V

+i2ctm=0x60,0x62,0x02

analog read pin 1 (PB3)

+i2ctm=0x60,0x11,0x01;+i2crq=0x60,0x02

analog read pin 2 (PB4)

+i2ctm=0x60,0x11,0x02;+i2crq=0x60,0x02

set timer prescaler to 1, clock prescaler to 1 and normal mode (488Hz PWM)

+i2ctm=0x60,0x50,0x01,0x03,0

set timer prescaler to 1, clock prescaler to 1, CTC mode and OCR1B = 127

analogWrite range is always mapped to 0-255

+i2ctm=0x60,0x50,0x01,0x01,0x7f

set pwm frequeny to 25kHz

+i2ctm=0x60,0x51,0xa8,0x61

read sfr TCCR0A

+i2ctm=0x60,0x80,0x2a;+i2crq=0x60,1

request info

+i2ctm=0x60,0xf0;+i2crq=0x60,0x08

*/

#include "main.h"

namespace TinyPwm {

    struct Pins {
        constexpr uint8_t size() const {
            return sizeof(*this);
        }
        uint8_t pins[kPinCount];
        int operator[](int index) const {
            return pins[index];
        }
    };

    static constexpr Pins kPins = { TINYPWM_PINS };

    struct __attribute__((__packed__)) Version {
        uint16_t major: 4;
        uint16_t minor: 5;
        uint16_t revision: 7;
        constexpr Version(uint16_t _major, uint16_t _minor, uint16_t _revision) :
            major(_major),
            minor(_minor),
            revision(_revision)
        {
        }
    };

    static constexpr auto kVersion = Version(TINYPWM_VERSION_MAJOR, TINYPWM_VERSION_MINOR, TINYPWM_VERSION_REVISION);

    struct __attribute__((__packed__)) SerialDebug {
        uint8_t txPin: 4;
        uint8_t rxPin: 4;
    };

#if DEBUG
    static constexpr SerialDebug kSerialDebug = { DEBUG_SERIAL_TX_PIN, DEBUG_SERIAL_RX_PIN };
#else
    static constexpr SerialDebug kSerialDebug = { 0xf, 0xf };
#endif

    struct __attribute__((__packed__)) Info {
        Version version;
        uint16_t pwmFrequency;
        Pins pins;
        SerialDebug serialDebug;

        Info();
    };

    static constexpr size_t kInfoSize = sizeof(Info);

    enum class Commands : uint8_t {
        INVALID = 0,
        ANALOG_READ = 0x11,
        ANALOG_WRITE = 0x12,
        DIGITAL_READ = 0x13,
        DIGITAL_WRITE = 0x14,
        PIN_MODE = 0x15,
        WRITE_EEPROM = 0x41,        // stores current settings in EEPROM
        RESET_EEPROM = 0x42,        // restores firmware defaults
        SET_PWM_PARAMS = 0x50,
        SET_PWM_FREQUENCY = 0x51,
        // ADC_SET_GAIN =  0x61,
        ADC_SET_AREF = 0x62,
        ADC_SET_READ_CYCLES = 0x63,
        READ_SFR = 0x80,
        WRITE_SFR = 0x81,
        MASK_SFR = 0x82,            // SFR = (SFR & mask) | data
        OR_SFR = 0x83,              // SFR |= data
        AND_SFR = 0x84,             // SFR &= data
        XOR_SFR = 0x85,             // SFR ^= data
        CLI = 0x90,
        SEI = 0x91,
        INFO = 0xf0,
        END = 0xff
    };

    enum class AnalogReference : uint8_t {
        INTERNAL_1V1 = INTERNAL1V1, // 2
        INTERNAL_2V56 = INTERNAL2V56_NO_CAP, // 6
    };

    enum class TimerPrescaler : uint8_t {
        FCLK_1 = _BV(CS00), // 1
        FCLK_8 = _BV(CS01), // 2
        FCLK_64 = _BV(CS01)|_BV(CS00), // 3
        FCLK_256 = _BV(CS02), // 4
        FCLK_1024 = _BV(CS02)|_BV(CS00), // 5
        MASK = 0b11111000
    };

    inline static uint16_t getTimerPrescaler(uint8_t mode) {
        mode &= ~static_cast<uint8_t>(TinyPwm::TimerPrescaler::MASK);
        return mode == 0 ? 0 : mode == 2 ? 8 : mode == 3 ? 64 : mode == 4 ? 256 : mode == 5 ? 1024 : 1;
    }

    inline static uint16_t getTimerPrescaler(TimerPrescaler _mode) {
        uint8_t mode = static_cast<uint8_t>(_mode);
        return mode == 0 ? 0 : mode == 2 ? 8 : mode == 3 ? 64 : mode == 4 ? 256 : mode == 5 ? 1024 : 1;
    }

    #define isPwmModeFastPwm() ((TCCR0A & (_BV(WGM01)|_BV(WGM00))) == (_BV(WGM01)|_BV(WGM00)))
    #define isPwmModeCTC() ((TCCR0A & (_BV(WGM01)|_BV(WGM00))) == _BV(WGM01))
    #define isPwmPhaseCorrect() ((TCCR0A & (_BV(WGM01)|_BV(WGM00))) == _BV(WGM00))

    enum class ClockPrescaler : uint8_t {
        PCK_NONE = 0,
        PCK_1 = _BV(CS10),                                      // 31250Hz @ 8Mhz
        PCK_2 = _BV(CS11),                                      // 16125Hz
        PCK_4 = _BV(CS11)|_BV(CS10),
        PCK_8 = _BV(CS12),
        PCK_16 = _BV(CS12)|_BV(CS10),
        PCK_32 = _BV(CS12)|_BV(CS11),
        PCK_64 = _BV(CS12)|_BV(CS11)|_BV(CS10),                 // 488Hz @ 8Mhz
        PCK_128 = _BV(CS13),
        PCK_256 = _BV(CS13)|_BV(CS10),
        PCK_512 = _BV(CS13)|_BV(CS11),
        PCK_1024 = _BV(CS13)|_BV(CS11)|_BV(CS10),
        //TODO...
        PCK_16384 = _BV(CS13)|_BV(CS12)|_BV(CS11)|_BV(CS10),
        MASK = 0b11110000
    };

    inline uint16_t getClockPrescaler(uint8_t mode) {
        return _BV((mode - 1) & ~static_cast<uint8_t>(TinyPwm::ClockPrescaler::MASK));
    }

    inline uint16_t getClockPrescaler(ClockPrescaler mode) {
        return _BV(static_cast<uint8_t>(mode) - 1);
    }

    enum class PwmFrequency : uint8_t {
        F_31250 = static_cast<uint8_t>(ClockPrescaler::PCK_1),
        F_15625 = static_cast<uint8_t>(ClockPrescaler::PCK_2),
        F_7812 = static_cast<uint8_t>(ClockPrescaler::PCK_4),
        F_3906 = static_cast<uint8_t>(ClockPrescaler::PCK_8),
        F_1953 = static_cast<uint8_t>(ClockPrescaler::PCK_16),
        F_976 = static_cast<uint8_t>(ClockPrescaler::PCK_32),
        F_488 = static_cast<uint8_t>(ClockPrescaler::PCK_64),
        F_244 = static_cast<uint8_t>(ClockPrescaler::PCK_128),
        F_122 = static_cast<uint8_t>(ClockPrescaler::PCK_256),
        F_61 = static_cast<uint8_t>(ClockPrescaler::PCK_512),
        F_30 = static_cast<uint8_t>(ClockPrescaler::PCK_1024),
    };

    struct PwmFrequencyMode {
        const ClockPrescaler prescaler;
        const uint8_t overflow;
        constexpr PwmFrequencyMode(const ClockPrescaler _prescaler, const uint8_t _overflow) : prescaler(_prescaler), overflow(_overflow) {
        }
    };

    static constexpr uint8_t getOverflow(uint16_t f, uint16_t bf) {
        return f == bf ? 0 : (bf * 256UL + 128UL) / f - 1;
    }

    template <unsigned long f>
    static constexpr PwmFrequencyMode kGetPwmFrequency() {
        static_assert(f >= 488, "must be greater or equal 488");
        static_assert(f <= 0xffff, "16 bit overflow");
        return f < 976 ? PwmFrequencyMode(ClockPrescaler::PCK_64, getOverflow(f, 488)) :
            f < 1953  ? PwmFrequencyMode(ClockPrescaler::PCK_32, getOverflow(f, 976)) :
            f < 3906 ? PwmFrequencyMode(ClockPrescaler::PCK_16, getOverflow(f, 1953)) :
            f < 7812 ? PwmFrequencyMode(ClockPrescaler::PCK_8, getOverflow(f, 3906)) :
            f < 15625 ? PwmFrequencyMode(ClockPrescaler::PCK_4, getOverflow(f, 7812)) :
            f < 31250 ? PwmFrequencyMode(ClockPrescaler::PCK_2, getOverflow(f, 15625)) :
            PwmFrequencyMode(ClockPrescaler::PCK_1, getOverflow(f, 31250));
    }

    inline static PwmFrequencyMode getPwmFrequency(uint16_t f) {
        return f < 976 ? PwmFrequencyMode(ClockPrescaler::PCK_64, getOverflow(f, 488)) :
            f < 1953  ? PwmFrequencyMode(ClockPrescaler::PCK_32, getOverflow(f, 976)) :
            f < 3906 ? PwmFrequencyMode(ClockPrescaler::PCK_16, getOverflow(f, 1953)) :
            f < 7812 ? PwmFrequencyMode(ClockPrescaler::PCK_8, getOverflow(f, 3906)) :
            f < 15625 ? PwmFrequencyMode(ClockPrescaler::PCK_4, getOverflow(f, 7812)) :
            f < 31250 ? PwmFrequencyMode(ClockPrescaler::PCK_2, getOverflow(f, 15625)) :
            PwmFrequencyMode(ClockPrescaler::PCK_1, getOverflow(f, 31250));
    }

    struct Command {
        union {
            struct {
                uint8_t length;
                Commands command;
                union {
                    struct {
                        ClockPrescaler prescaler;
                        uint8_t overflow;   // 0/255 = Fast PWM, >0 CTC mode. in CTC mode, delay is not working
                    } SET_PWM_PARAMS;
                    struct {
                        uint16_t frequency;
                    } SET_PWM_FREQUENCY;
                    struct {
                        uint8_t pin;            // internal pin number
                        uint8_t pwmValue;
                    } ANALOG_WRITE;
                    struct {
                        uint8_t pin;            // internal pin number
                    } ANALOG_READ;
                    struct {
                        uint8_t pin;            // internal pin number
                    } DIGITAL_READ;
                    struct {
                        uint8_t pin;            // internal pin number
                        uint8_t value;
                    } DIGITAL_WRITE;
                    struct {
                        uint8_t pin;            // internal pin number
                        uint8_t mode;
                    } PIN_MODE;
                    struct {
                        uint8_t address;
                        uint8_t data;
                    } WRITE_SFR;
                    struct {
                        uint8_t address;
                    } READ_SFR;
                    struct {
                        uint8_t address;
                        uint8_t mask;
                        uint8_t data;
                    } MASK_SFR;
                    struct {
                        uint8_t address;
                        uint8_t data;
                    } OR_SFR;
                    struct {
                        uint8_t address;
                        uint8_t data;
                    } AND_SFR;
                    struct {
                        uint8_t address;
                        uint8_t data;
                    } XOR_SFR;
                    struct {
                        AnalogReference mode;
                    } ADC_SET_AREF;
                    struct {
                        uint8_t pin;
                        uint8_t cycles;
                    } ADC_SET_READ_CYCLES;
                };
            };
            uint8_t raw[32];
            struct {
                uint8_t __length;
                uint8_t rawCommand[31];
            };
        };
    };

    static constexpr size_t kCommandSize = sizeof(Command);

    struct ResponseAnalogRead {
        int16_t value;
        uint16_t reading;
    };

    struct ResponseReadSFR {
        uint8_t data;
    };

    struct Response {
        union {
            ResponseAnalogRead ANALOG_READ;
            ResponseReadSFR READ_SFR;
        };
    };

}