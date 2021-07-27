/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <Arduino.h>

#ifndef DEBUG
#define DEBUG 0
#endif

#if DEBUG
#define DBG_PRINTF(message, ...) Serial.printf(F(message "\n"), ##__VA_ARGS__)
#define DBG_PRINTF_P(message, ...) Serial.printf(F(message "\n"), ##__VA_ARGS__)
#else
#define DBG_PRINTF(message, ...)
#define DBG_PRINTF_P(message, ...)
#endif

#if 1   // i2c related
#define DBG1_PRINTF(message, ...) DBG_PRINTF(message, ##__VA_ARGS__)
#else
#define DBG1_PRINTF(...)
#endif
#if 1   // PWM related1
#define DBG2_PRINTF(message, ...) DBG_PRINTF(message, ##__VA_ARGS__)
#else
#define DBG2_PRINTF(...)
#endif
#if 1   // ADC related
#define DBG3_PRINTF(message, ...) DBG_PRINTF(message, ##__VA_ARGS__)
#else
#define DBG3_PRINTF(...)
#endif

#define TINYPWM_VERSION_MAJOR 0
#define TINYPWM_VERSION_MINOR 0
#define TINYPWM_VERSION_REVISION 3

// default I2C slave address
#ifndef TINYPWM_I2C_SLAVE_ADDRESS
#define TINYPWM_I2C_SLAVE_ADDRESS 0x60
#endif

#if TINYPWM_VALUES_FREQUENCY
#define TINYPWM_CLOCK_PRESCALER TinyPwm::kGetPwmFrequency<TINYPWM_VALUES_FREQUENCY>().prescaler
#define TINYPWM_OVERFLOW TinyPwm::kGetPwmFrequency<TINYPWM_VALUES_FREQUENCY>().overflow
#endif

// clock prescaler for PWM
#ifndef TINYPWM_CLOCK_PRESCALER
#define TINYPWM_CLOCK_PRESCALER TinyPwm::ClockPrescaler::PCK_64
#endif

// decreasing overflow reduces the resolution of the PWM (1 / TINYPWM_OVERFLOW)
// default 0 = (1 step / 255) = 0.392157%
#ifndef TINYPWM_OVERFLOW
#define TINYPWM_OVERFLOW 0
#endif

// analog reference for the ADC
#ifndef TINYPWM_ANALOG_REFERENCE
#define TINYPWM_ANALOG_REFERENCE TinyPwm::AnalogReference::INTERNAL_2V56
#endif

// list of digital pins
#ifndef TINYPWM_PINS
#define TINYPWM_PINS PB1, PB2, PB3
#endif

#define ANALOG_INPUT 0x80

// set modes at startup
#ifndef TINYPWM_PIN_MODES
#define TINYPWM_PIN_MODES INPUT, ANALOG_INPUT, ANALOG_INPUT
#endif

// default values for PWM
// the pin mode must be set to output (TINYPWM_PIN_MODES)
#ifndef TINYPWM_VALUES
#define TINYPWM_VALUES 0, 0, 0
#endif

// get an average over N conversions before storing the final value
// each analog pin can have its own cycle count
// possible values are 1-63
#ifndef TINYPWM_ANALOG_CONVERSIONS
#define TINYPWM_ANALOG_CONVERSIONS 1, 1, 1
#endif

#ifndef TINYPWM_ADC_OFFSET
#define TINYPWM_ADC_OFFSET 0
#endif

#ifndef TINYPWM_GAIN_ERROR
#define TINYPWM_GAIN_ERROR 0
#endif

#define _STRINGIFY(...) ___STRINGIFY(__VA_ARGS__)
#define ___STRINGIFY(...) #__VA_ARGS__

namespace TinyPwm {

    static constexpr const uint8_t _kPins[] = { TINYPWM_PINS };
    static constexpr const uint8_t kPinCount = sizeof(_kPins);

}
