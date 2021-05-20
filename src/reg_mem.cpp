/**
  Author: sascha_lammers@gmx.de
*/

#include <util/atomic.h>
#include <wiring_private.h>
#include "reg_mem.h"

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

RegMem::RegMem() :
    analog{},
    analogPinNo(0),
    cycles{DEFAULT_ANALOG_CONVERSIONS},
    pwmStartValue(DEFAULT_PWM),
    pwmValue(0),
    pwmPin(1)
{
}

void RegMem::begin()
{
#if DEBUG
    Serial = TinySoftwareSerial(Serial._rx_buffer, DEBUG_SERIAL_TX_PIN, DEBUG_SERIAL_RX_PIN);
    Serial.begin(9600);
    DBG_PRINTF("DEBUG MODE");
    DBG_PRINTF("analog pins=%u", TinyPwm::kAnalogPinCount);
#endif

    if (sizeof(kAnalogPins)) {
        ADCHelper::enable();
        setAnalogReference(DEFAULT_ANALOG_REFERENCE);
        for(auto pin: kAnalogPins) {
            digitalWrite(pin, LOW);
#if DEBUG
            if (pin != PB3) {
                pinMode(pin, INPUT);
            }
#else
            pinMode(pin, INPUT);
#endif

        }
        selectAnalogPin(analogPinNo);

#if DEBUG
        static uint8_t cycles[] = {DEFAULT_ANALOG_CONVERSIONS};
        for(uint8_t i = 0; i < sizeof(kAnalogPins); i++) {
            setAnalogCycles(i, cycles[i]);
        }
#endif

    }
    else {
        // disable ADC without any pins to read
        ADCHelper::disable();
    }

    setPwmParams(DEFAULT_CLOCK_PRESCALER, DEFAULT_OVERFLOW);

    DBG_PRINTF("pwm pin=%u value=%u frequency=%u", pwmPin, pwmStartValue, getPwmFrequency());

    pinMode(pwmPin, OUTPUT);
    analogWrite(pwmPin, pwmStartValue);
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

void RegMem::selectAnalogPin(uint8_t pinNo)
{
    DBG3_PRINTF("select_analog_pin %u %u %u", (unsigned)pinNo, (unsigned)(kAnalogPins[pinNo]&0x7f), (unsigned)analogInputToDigitalPin(kAnalogPins[pinNo]));
#if defined(ADMUX)
    ADMUX = ((analog_reference & ADMUX_REFS_MASK) << REFS0) | ((kAnalogPins[pinNo] & ADMUX_MUX_MASK) << MUX0); // select the channel and reference
#if defined(REFS2)
    ADMUX |= (((analog_reference & 0x04) >> 2) << REFS2); // some have an extra reference bit in a weird position.
#endif
#endif

    cycleCounter = -1; // skip first result
    ADCHelper::startConversion();
}

void RegMem::selectNextAnalogPin()
{
    if (sizeof(kAnalogPins) > 1) {
        analogPinNo++;
        analogPinNo %= sizeof(kAnalogPins);
        selectAnalogPin(analogPinNo);
    }
    else if (sizeof(kAnalogPins) == 1) {
        cycleCounter = 0;
        ADCHelper::startConversion();
    }
}

void RegMem::analogWrite(uint8_t pin, uint8_t value)
{
    DBG2_PRINTF("analogWrite PB%u=%u (scaled %u)", pin, value, value * OCR1C / 255);
    ::analogWrite(pin, value * OCR1C / 255);
    pwmValue = value;
}

void RegMem::readAnalogValue()
{
    if (cycleCounter <= 0) {
        analogValue = ADC;
    }
    else {
        analogValue += ADC;
    }

    // DBG3_PRINTF("adc_read=%u ADC=%u cycles=%u/%u", (unsigned)analogPinNo, (unsigned)ADC, (unsigned)cycleCounter, (unsigned)cycles[analogPinNo]);

    if (++cycleCounter >= cycles[analogPinNo]) {
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
