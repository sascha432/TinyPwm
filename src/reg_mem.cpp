/**
  Author: sascha_lammers@gmx.de
*/

#include <util/atomic.h>
#include <wiring_private.h>
#include "reg_mem.h"
#include "config.h"

#if defined(REFS1)
#define ADMUX_REFS_MASK (0x03)
#else
#define ADMUX_REFS_MASK (0x01)
#endif

#if defined(MUX5)
#define ADMUX_MUX_MASK (0x3f)
#elif defined(MUX4)
#define ADMUX_MUX_MASK (0x1f)
#elif defined(MUX3)
#define ADMUX_MUX_MASK (0x0f)
#else
#define ADMUX_MUX_MASK (0x07)
#endif

extern uint8_t analog_reference;

RegMem::RegMem(const Config::Data &defaultConfig) :
    address(defaultConfig.address),
    adcOffset(defaultConfig.adcOffset),
    adcGainError(defaultConfig.adcGainError),
    analog{},
    analogPinCount(0),
    analogPinSelected(0),
    cycleCounter(-1),
    arrays(defaultConfig.arrays)
{
}

void RegMem::updateConfig(Config::Data &config)
{
    config.version++;
    config.address = address;
    config.adcOffset = adcOffset;
    config.adcGainError = adcGainError;
    config.arrays = arrays;
}

void RegMem::begin()
{
#if DEBUG
    Serial = TinySoftwareSerial(Serial._rx_buffer, DEBUG_SERIAL_TX_PIN, DEBUG_SERIAL_RX_PIN);
    Serial.begin(9600);
    DBG_PRINTF("DEBUG MODE");
    DBG_PRINTF("analog pins=%u", TinyPwm::kAnalogPinCount);
#endif

    ADCHelper::disable();

    setPwmParams(TINYPWM_CLOCK_PRESCALER, TINYPWM_OVERFLOW);

    DBG_PRINTF("pwm pin=%u value=%u frequency=%u", pwmPin, pwmStartValue, getPwmFrequency());

    uint8_t n = 0;
    for(auto mode: arrays.pinModes) {
        auto pin = TinyPwm::kPins[n];
        // store internal number for analogWrite
#if DEBUG
        // skip serial pins in debug mode
        if (pin == DEBUG_SERIAL_TX_PIN || pin == DEBUG_SERIAL_RX_PIN) {
            arrays.pinModes[n] = 0xff;
        }
        else
#endif
        if (mode & ANALOG_INPUT) {
            digitalWrite(pin, LOW);
            ::pinMode(pin, INPUT);
            arrays.pinModes[n] = ANALOG_INPUT;
            analogPins[analogPinCount++] = n;
        }
        else if (mode == OUTPUT) {
            // analogWrite sets pin to output
            analogWrite(pin, arrays.values[n]);
            arrays.pinModes[n] = OUTPUT;
        }
        else {
            pinMode(pin, mode);
            arrays.pinModes[n] = mode;
        }
    }

    if (analogPinCount) {
        // enable ADC and reset to first pin
        ADCHelper::enable();
        setAnalogReference(TINYPWM_ANALOG_REFERENCE);
        selectAnalogPin(0);
    }
}

TinyPwm::Command RegMem::readCommand(uint8_t count)
{
    TinyPwm::Command cmd;
    DBG_PRINTF("wire available=%u count=%u", Wire.available(), count);

    if (Wire.available()) {
        auto dst = reinterpret_cast<uint8_t *>(&cmd.command);
        auto dataEnd = dst + min(count, sizeof(cmd) - (dst - reinterpret_cast<uint8_t *>(&cmd)));
        auto end = cmd.raw + sizeof(cmd);
        // read data
        while(dst < dataEnd && Wire.available()) {
            *dst++ = Wire.read();
        }
        cmd.length = dst - reinterpret_cast<const uint8_t *>(&cmd.command);
        // discard extra data
        while(Wire.available()) {
            Wire.read();
        }
        // fill missing data with 0xff
        while(dst < end) {
            *dst++ = 0xff;
        }
#if DEBUG
        Serial.printf(F("cmd length %u data="), (unsigned)cmd.length);
        dst = reinterpret_cast<uint8_t *>(&cmd);
        end = dst + sizeof(cmd);
        while(dst < end) {
            Serial.printf("%02x ", (unsigned)*dst);
        }
        Serial.println();
#endif
    }
    else {
        cmd.command = TinyPwm::Commands::INVALID;
        DBG_PRINTF("Wire.available=0 count=%u", count);
    }
    return cmd;
}

void RegMem::setPwmParams(TinyPwm::ClockPrescaler clockPrescaler, uint8_t overflow)
{
    // defaults
    // TCCR0A 00000111 (???/WGM01/WGM00)
    // TCCR0B 00000111 (CS02/CS01/CS00) timer prescaler 64
    // TCCR1 11000111 (CTC1/PWM1A/CS12/CS11/CS10) clock prescaler 64
    // OCR1C 255

    // DBG2_PRINTF("setTimerParams before TCCR0A=%u TCCR0B=%u TCCR1=%u OCR1C=%u FCLK=%u PCK=%u", TCCR0A, TCCR0B, TCCR1, OCR1C, TinyPwm::getTimerPrescaler(TCCR0B), TinyPwm::getClockPrescaler(TCCR1));
    // testDelay(10000);

    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        // TCCR0B = static_cast<uint8_t>(timerPrescaler);

        if (overflow && overflow != 255) {
            OCR1C = overflow; // set overflow counter
            TCCR0A = (TCCR0A & 0b11111100) | _BV(WGM01); // CTC mode
        }
        else {
            OCR1C = 255;
            TCCR0A = (TCCR0A & 0b11111100) | (_BV(WGM01)|_BV(WGM00)); // Fast PWM
        }
        TCCR1 = (TCCR1 & static_cast<uint8_t>(TinyPwm::ClockPrescaler::MASK)) | static_cast<uint8_t>(clockPrescaler); // set prescaler
    }

    DBG2_PRINTF("TCCR0A=%u TCCR0B=%u TCCR1=%u OCR1C=%u FCLK=%u PCK=%u", TCCR0A, TCCR0B, TCCR1, OCR1C, TinyPwm::getTimerPrescaler(TCCR0B), TinyPwm::getClockPrescaler(TCCR1));
    // testDelay(10000);
}

uint16_t RegMem::getPwmFrequency() const
{
    DBG2_PRINTF("F_CPU=%lu OCR1C=%u pre=%u clk=%u mode=%s", F_CPU, OCR1C, TinyPwm::getTimerPrescaler(TCCR0B), TinyPwm::getClockPrescaler(TCCR1), isPwmModeFastPwm() ? "FastPWM" : (isPwmModeCTC() ? "CTC" : (isPwmPhaseCorrect() ? "PhaseCorrect" : "N/A")));

    if (isPwmModeFastPwm()) {
        return (F_CPU / 256UL) / TinyPwm::getClockPrescaler(TCCR1);
    }
    return F_CPU / (TinyPwm::getClockPrescaler(TCCR1) * (1UL + OCR1C));
}

void RegMem::selectAnalogPin(uint8_t pin)
{
    // translate to internal number
    pin = analogPins[pin];
    if ((arrays.pinModes[pin] & ANALOG_INPUT) == 0) {
        DBG2_PRINTF("selectAnalogPin pin #%u not in analog input mode", pin);
        return;
    }
    analogPinSelected = pin;
    cycleCounter = -1; // skip first reading after selecting a new pin
    // get digital pin
    auto digitalPin = TinyPwm::_kPins[pin];

    DBG3_PRINTF("select_analog_pin %u %u", (unsigned)pin, (unsigned)analogInputToDigitalPin(digitalPin));
#if defined(ADMUX)
    ADMUX = ((analog_reference & ADMUX_REFS_MASK) << REFS0) | ((digitalPin & ADMUX_MUX_MASK) << MUX0); // select the channel and reference
#if defined(REFS2)
    ADMUX |= (((analog_reference & 0x04) >> 2) << REFS2); // some have an extra reference bit in a weird position.
#endif
#endif
    ADCHelper::startConversion();
}

void RegMem::selectNextAnalogPin()
{
    if (analogPinCount == 0) {
        return;
    }
    if (++analogPinSelected >= analogPinCount) {
        analogPinSelected = 0;
    }
    if (analogPinSelected == 0) {
        // start new conversion, only one analog pin is available
        cycleCounter = 0;
        ADCHelper::startConversion();
    }
    else {
        // select next pin and start conversion
        selectAnalogPin(analogPinSelected);
    }
}

void RegMem::analogWrite(uint8_t pin, uint8_t value)
{
    if (pin >= TinyPwm::kPinCount) {
        DBG2_PRINTF("analogWrite pin #%u not valid", pin);
        return;
    }
#if DEBUG
    if (arrays.pinModes[pin] == 0xff) {
        DBG2_PRINTF("analogWrite pin #%u used for debugging", pin);
        return;
    }
#endif
    if (arrays.pinModes[pin] != OUTPUT) {
        DBG2_PRINTF("analogWrite pin #%u not in OUTPUT mode", pin);
        return;
    }
    DBG2_PRINTF("analogWrite pin #%u=%u (scaled %u)", pin, value, value * OCR1C / 255);
    ::analogWrite(pin, value * OCR1C / 255);
    arrays.values[pin] = value;
}

void RegMem::digitalWrite(uint8_t pin, bool state)
{
    if (pin >= TinyPwm::kPinCount) {
        DBG2_PRINTF("digitalWrite pin #%u not valid", pin);
        return;
    }
#if DEBUG
    if (arrays.pinModes[pin] == 0xff) {
        DBG2_PRINTF("digitalWrite pin #%u used for debugging", pin);
        return;
    }
#endif
    if (arrays.pinModes[pin] != OUTPUT) {
        DBG2_PRINTF("digitalWrite pin #%u not in OUTPUT mode", pin);
        return;
    }
    DBG2_PRINTF("analogWrite pin #%u=%u (scaled %u)", pin, value, value * OCR1C / 255);
    ::digitalWrite(pin, state);
    arrays.values[pin] = state;
}

bool RegMem::digitalRead(uint8_t pin)
{
    if (pin >= TinyPwm::kPinCount) {
        DBG2_PRINTF("digitalRead pin #%u not valid", pin);
        return false;
    }
    return ::digitalRead(pin);
}

void RegMem::_resetAnalogPins()
{
    analogPinCount = 0;
    for(uint8_t i = 0; i < TinyPwm::kPinCount; i++) {
        if (arrays.pinModes[i] == ANALOG_INPUT) {
            analogPins[analogPinCount++] = i;
        }
    }
    // start new cycle
    selectNextAnalogPin();
    // we don't know if resetting selected the same pin or a new one
    // skip first reading
    cycleCounter = -1;
    DBG2_PRINTF("analogPinCount %u", analogPinCount);
}

void RegMem::pinMode(uint8_t pin, uint8_t mode)
{
    if (pin >= TinyPwm::kPinCount) {
        DBG2_PRINTF("pinMode pin #%u not valid", pin);
        return;
    }
#if DEBUG
    if (arrays.pinModes[pin] == 0xff) {
        DBG2_PRINTF("pinMode pin #%u used for debugging", pin);
        return;
    }
#endif
    bool isAnalog = arrays.pinModes[pin] & ANALOG_INPUT;
    auto digitalPin = TinyPwm::_kPins[pin];

    if (mode & ANALOG_INPUT) {
        arrays.pinModes[pin] = ANALOG_INPUT;
        mode = INPUT;
        isAnalog = true;
    }
    else {
        arrays.pinModes[pin] = mode;
    }
    ::pinMode(digitalPin, mode);

    // number of analog pins has changed
    if (isAnalog) {
        _resetAnalogPins();
    }
}

void RegMem::readAnalogValue()
{
    if (cycleCounter <= 0) {
        // reset average for new pin or new cycle
        analogValue = ADC;
    }
    else {
        analogValue += ADC;
    }

    // DBG3_PRINTF("adc_read=%u ADC=%u cycles=%u/%u", (unsigned)analogPinNo, (unsigned)ADC, (unsigned)cycleCounter, (unsigned)cycles[analogPinNo]);

    // translate to internal pin number
    uint8_t analogPinNo = analogPins[analogPinSelected];

    if (++cycleCounter >= arrays.cycles[analogPinNo]) {
        analog[analogPinNo].value = analogValue / cycleCounter;
        analog[analogPinNo].reading++;

#if DEBUG
        if (analog[analogPinNo].reading % 8192 == 0) {
            unsigned long mul = (analog_reference == INTERNAL2V56_NO_CAP ? 2560UL : analog_reference == INTERNAL1V1 ? 1100UL : 0);
            DBG3_PRINTF("adc_value pin_no=%u value=%u(%u) %lumV", (unsigned)analogPinNo, analog[analogPinNo].value, analog[analogPinNo].reading, (unsigned long)((analog[analogPinNo].value * mul) >> 10));
        }
#endif

        selectNextAnalogPin();
    }
}
