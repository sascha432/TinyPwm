/**
  Author: sascha_lammers@gmx.de
*/

#include <Arduino.h>
#include <wiring_private.h>
#include <time.h>
#include "twi_buffer.h"
#include "reg_mem.h"
#include "config.h"

RegMem regMem;
TwiBuffer twiBuffer;

TinyPwm::Info::Info() :
    version(kVersion),
    pwmPin(regMem.pwmPin),
    pwmFrequency(regMem.getPwmFrequency()),
#if DEBUG
    serialDebug({DEBUG_SERIAL_TX_PIN, DEBUG_SERIAL_RX_PIN})
#else
    serialDebug({0xf, 0xf})
#endif
{
}

void requestEvent()
{
    twiBuffer.sendTo(Wire);
}

static bool twiLengthError(const char *name, int length, int expected)
{
    DBG1_PRINTF("%s got=%u expected=%u", name, length, expected);
    return false;
}

#define DBG_TWI_LENGTH_ERROR(command) twiLengthError(_STRINGIFY(command), cmd.length, sizeof(cmd.command))
#define CMD_CHECK_LENGTH(command) ((cmd.length < sizeof(cmd.command)) ? DBG_TWI_LENGTH_ERROR(command) : true)

void receiveEvent(int count)
{
    if (!count) {
        DBG1_PRINTF("no data received");
        return;
    }
    auto cmd = regMem.readCommand(count);
    if (cmd.command == TinyPwm::Commands::INVALID) {
        DBG1_PRINTF("Wire.available() = 0");
        return;
    }

    DBG1_PRINTF("receive event count=%u command=%u data=%02x %02x", count, cmd.command, cmd.raw[1], cmd.raw[2]); // sizeof(TinyPwm::Command);

    twiBuffer.clear();

    switch(cmd.command) {
        case TinyPwm::Commands::ANALOG_READ:
            if (CMD_CHECK_LENGTH(ANALOG_READ)) {
                if (cmd.ANALOG_READ.pinNo < sizeof(kAnalogPins)) {
                    twiBuffer.send(regMem.analog[cmd.ANALOG_READ.pinNo]);
                }
                else {
                    DBG1_PRINTF("ANALOG_READ pin=%u out of range (0-%u)", cmd.ANALOG_READ.pinNo, sizeof(kAnalogPins) - 1);
                }
            }
            break;
        case TinyPwm::Commands::ANALOG_WRITE:
            if (CMD_CHECK_LENGTH(ANALOG_WRITE)) {
                regMem.analogWrite(regMem.pwmPin, cmd.ANALOG_WRITE.pwmValue);
            }
            break;
        case TinyPwm::Commands::READ_EEPROM:
        case TinyPwm::Commands::WRITE_EEPROM:
        // case TinyPwm::Commands::ADC_SET_GAIN:
            //TODO
            break;
        case TinyPwm::Commands::SET_PWM_PARAMS:
            if (CMD_CHECK_LENGTH(SET_PWM_PARAMS)) {
                regMem.setPwmParams(cmd.SET_PWM_PARAMS.prescaler, cmd.SET_PWM_PARAMS.overflow);
            }
            break;
        case TinyPwm::Commands::SET_PWM_FREQUENCY:
            if (CMD_CHECK_LENGTH(SET_PWM_FREQUENCY)) {
                regMem.setPwmFrequency(cmd.SET_PWM_FREQUENCY.frequency);
            }
            break;
        case TinyPwm::Commands::ADC_SET_AREF:
            if (CMD_CHECK_LENGTH(ADC_SET_AREF)) {
                regMem.setAnalogReference(cmd.ADC_SET_AREF.mode);
            }
            break;
        case TinyPwm::Commands::ADC_SET_READ_CYCLES:
            if (CMD_CHECK_LENGTH(ADC_SET_READ_CYCLES)) {
                regMem.setAnalogCycles(cmd.ADC_SET_READ_CYCLES.pinNo, cmd.ADC_SET_READ_CYCLES.cycles);
            }
            break;
        case TinyPwm::Commands::READ_SFR:
            if (CMD_CHECK_LENGTH(READ_SFR)) {
                twiBuffer.send(_SFR_IO8(cmd.READ_SFR.address));
            }
            break;
        case TinyPwm::Commands::WRITE_SFR:
            if (CMD_CHECK_LENGTH(WRITE_SFR)) {
                _SFR_IO8(cmd.WRITE_SFR.address) = cmd.WRITE_SFR.data;
            }
            break;
        case TinyPwm::Commands::MASK_SFR:
            if (CMD_CHECK_LENGTH(MASK_SFR)) {
                _SFR_IO8(cmd.MASK_SFR.address) = (_SFR_IO8(cmd.MASK_SFR.address) & cmd.MASK_SFR.mask) | cmd.MASK_SFR.data;
            }
            break;
        case TinyPwm::Commands::OR_SFR:
            if (CMD_CHECK_LENGTH(OR_SFR)) {
                _SFR_IO8(cmd.OR_SFR.address) |= cmd.OR_SFR.data;
            }
            break;
        case TinyPwm::Commands::AND_SFR:
            if (cmd.length >= sizeof(cmd.AND_SFR)) {
                _SFR_IO8(cmd.AND_SFR.address) &= cmd.AND_SFR.data;
            }
            break;
        case TinyPwm::Commands::CLI:
            cli();
            break;
        case TinyPwm::Commands::SEI:
            sei();
            break;
        case TinyPwm::Commands::INFO:
            twiBuffer.send(TinyPwm::Info());
            break;
        default:
            DBG1_PRINTF("invalid command=%u", cmd.command);
            break;
    }
}

void setup()
{
    regMem.begin();

    Wire.begin(DEFAULT_I2C_SLAVE_ADDRESS);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
}

void loop()
{
    // read the ADC pins constantly without blocking
    if (sizeof(kAnalogPins) && ADCHelper::ready()) {
        regMem.readAnalogValue();
    }
}
