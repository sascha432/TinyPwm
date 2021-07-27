/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <Wire.h>
#include <time.h>
#include "tinypwm.h"
#include "config.h"

#if !defined(HAVE_ADC) || HAVE_ADC==0
static_assert(TinyPwm::kAnalogPinCount == 0, "no ADC present");
#endif


struct RegMem {
    union {
        struct  {
            uint8_t address;
            int8_t adcOffset;
            int16_t adcGainError;
            TinyPwm::ResponseAnalogRead analog[TinyPwm::kPinCount];     // stores average value and number of readings
            uint8_t analogPins[TinyPwm::kPinCount];
            uint8_t analogPinCount;
            uint8_t analogPinSelected;
            uint8_t cycleCounter;
            int16_t analogValue;
            Config::Arrays arrays;
        };
        uint8_t raw[1];
    };

    static constexpr uint16_t kAdcDivider = 1024;

    RegMem() {}
    RegMem(const Config::Data &defaultConfig);

    void updateConfig(Config::Data &config);

    // setup
    void begin();

    // read available data from Wire into cmd struct and fill th rest with 0xff
    // extra data will be discarded
    // the available data is stored in length
    TinyPwm::Command readCommand(uint8_t count);

    // set pwm timer paramters
    void setPwmParams(TinyPwm::ClockPrescaler _clockPrescaler, uint8_t overflow);

    // set pwm frequency
    void setPwmFrequency(uint16_t frequency);

    // calulate PWM frequency
    uint16_t getPwmFrequency() const;

    // set analog reference
    void setAnalogReference(TinyPwm::AnalogReference _analogReference);

    // number of ADC cycles before switching to the next input channel
    // pin is the internal pin number, not the digital pin or input channel
    void setAnalogCycles(uint8_t pin, uint8_t cycles);

    // select analog pin to read and start conversion
    // pin is the internal pin number, not the digital pin or input channel
    void selectAnalogPin(uint8_t pin);

    // select next analog pin to read and start conversion
    void selectNextAnalogPin();

    // read analog value from ADC
    void readAnalogValue();

    // analogWrite with mapped value from 0 - 255
    // pin is the internal pin number, not the digital pin or input channel
    // mode OUTPUT
    void analogWrite(uint8_t pin, uint8_t value);

    // digitalWrite
    // mode OUTPUT
    void digitalWrite(uint8_t pin, bool state);

    // digitalRead
    bool digitalRead(uint8_t pin);

    // set pin mode
    // pin is the internal pin number, not the digital pin or input channel
    // mode INPUT, INPUT_PULLUP, OUTPUT, ANALOG_INPUT
    void pinMode(uint8_t pin, uint8_t mode);

private:
    void _resetAnalogPins();
};

static constexpr size_t kRegMemSize = sizeof(RegMem);

#ifndef DISABLEMILLIS

inline static void testDelay(uint16_t time) {
    DBG_PRINTF("delay(%u) start", time);
    time_t start = millis();
    delay(time);
    time_t end = millis();
    DBG_PRINTF("delay(%u)=%u", time, (unsigned)(end - start));
}

#endif

inline void RegMem::setAnalogReference(TinyPwm::AnalogReference analogReference)
{
    ::analogReference(static_cast<uint8_t>(analogReference));
}

inline void RegMem::setAnalogCycles(uint8_t pin, uint8_t _cycles)
{
    if (pin >= TinyPwm::kPinCount) {
        DBG2_PRINTF("setAnalogCycles pin #%u not valid", pin);
        return;
    }
    arrays.cycles[pin] = max(1, min(63, _cycles));
    if (analogPins[analogPinSelected] == pin) { // restart current measurement
        cycleCounter = 0;
    }
    DBG3_PRINTF("pin=%u analog_cycles=%u", (unsigned)pinNo, (unsigned)cycles[pinNo]);
}

inline void RegMem::setPwmFrequency(uint16_t frequency)
{
    auto data = TinyPwm::getPwmFrequency(frequency);
    DBG2_PRINTF("frequency=%u PCK=%u overflow=%u", frequency, TinyPwm::getClockPrescaler(data.prescaler), data.overflow);
    setPwmParams(data.prescaler, data.overflow);
}

class ADCHelper {
public:
    static void enable() {
        asm volatile ("sbi %0, %1" :: "I" ( _SFR_IO_ADDR(ADCSRA)), "I" (ADEN));
    }

    static void disable() {
        asm volatile ("cbi %0, %1" :: "I" ( _SFR_IO_ADDR(ADCSRA)), "I" (ADEN));
    }

    static void startConversion() {
        asm volatile ("sbi %0, %1" :: "I" ( _SFR_IO_ADDR(ADCSRA)), "I" (ADSC));
    }

    static bool ready() {
        return (ADCSRA & _BV(ADSC)) == 0;
    }
};
