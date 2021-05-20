/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <Wire.h>
#include <time.h>
#include "tinypwm.h"
#include "main.h"

#if !defined(HAVE_ADC) || HAVE_ADC==0
static_assert(TinyPwm::kAnalogPinCount == 0, "no ADC present");
#endif

struct RegMem {
    union {
        struct  {
            TinyPwm::ResponseAnalogRead analog[TinyPwm::kAnalogPinCount];
            uint8_t analogPinNo;
            uint16_t analogValue;
            int8_t adcOffset;
            int16_t adcGainError;
            uint8_t cycles[TinyPwm::kAnalogPinCount];
            int8_t cycleCounter;
            uint8_t pwmStartValue;
            uint8_t pwmValue;
            const uint8_t pwmPin;
        };
        uint8_t raw[1];
    };

    static constexpr uint16_t kAdcDivider = 1024;

    RegMem();

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
    void setAnalogCycles(uint8_t pinNo, uint8_t cycles);

    // select analog pin to read and start conversion
    // pinNo is the internal pin number, not the digital pin or input channel
    void selectAnalogPin(uint8_t pinNo);

    // select next analog pin to read and start conversion
    void selectNextAnalogPin();

    // read analog value from ADC
    void readAnalogValue();

    // analogWrite with mapped value from 0 - 255
    void analogWrite(uint8_t pin, uint8_t value);
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

inline void RegMem::setAnalogCycles(uint8_t pinNo, uint8_t _cycles)
{
    cycles[pinNo] = max(1, min(63, _cycles));
    if (pinNo == analogPinNo) { // restart current measurement
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
