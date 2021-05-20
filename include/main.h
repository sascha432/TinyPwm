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

// default I2C slave address
#ifndef DEFAULT_I2C_SLAVE_ADDRESS
#define DEFAULT_I2C_SLAVE_ADDRESS 0x60
#endif

// #ifndef DEFAULT_TIMER_PRESCALER
// #define DEFAULT_TIMER_PRESCALER TinyPwm::TimerPrescaler::FCLK_64
// #endif

#if DEFAULT_PWM_FREQUENCY
#define DEFAULT_CLOCK_PRESCALER TinyPwm::kGetPwmFrequency<DEFAULT_PWM_FREQUENCY>().prescaler
#define DEFAULT_OVERFLOW TinyPwm::kGetPwmFrequency<DEFAULT_PWM_FREQUENCY>().overflow
#endif

// clock prescaler for PWM
#ifndef DEFAULT_CLOCK_PRESCALER
#define DEFAULT_CLOCK_PRESCALER TinyPwm::ClockPrescaler::PCK_64
#endif

// decreasing overflow reduces the resolution the he PWM (1 / DEFAULT_OVERFLOW)
// default 0 = (1 step / 255) = 0.392157%
#ifndef DEFAULT_OVERFLOW
#define DEFAULT_OVERFLOW 0
#endif

// default value for PWM
#ifndef DEFAULT_PWM
#define DEFAULT_PWM 0
#endif

// analog reference for the ADC
#ifndef DEFAULT_ANALOG_REFERENCE
#define DEFAULT_ANALOG_REFERENCE TinyPwm::AnalogReference::INTERNAL_2V56
#endif

// list of analog inputs to constantly read the ADC values from
#ifndef DEFAULT_ANALOG_PINS
#define DEFAULT_ANALOG_PINS A3,A2
#endif

// get an average over N conversions before storing the final value
// each analog pin can have its own cycle count
// possible values are 1-63
#ifndef DEFAULT_ANALOG_CONVERSIONS
#define DEFAULT_ANALOG_CONVERSIONS 1,1
#endif

static constexpr const uint8_t kAnalogPins[] = { DEFAULT_ANALOG_PINS };

#define _STRINGIFY(...) ___STRINGIFY(__VA_ARGS__)
#define ___STRINGIFY(...) #__VA_ARGS__
