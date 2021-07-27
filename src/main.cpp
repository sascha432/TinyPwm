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
    pwmFrequency(regMem.getPwmFrequency()),
    pins(kPins),
    serialDebug(kSerialDebug)
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

//TODO review function
void removeCommand(TinyPwm::Command &cmd, uint8_t cmdStructLength)
{
    cmdStructLength += sizeof(cmd.command);
    // move to next command
    memmove(cmd.rawCommand, cmd.rawCommand + cmdStructLength, sizeof(cmd.rawCommand) - cmdStructLength);
    // fill the rest with 0xff
    memset(cmd.rawCommand + sizeof(cmd.rawCommand) - cmdStructLength, 0xff, cmdStructLength);
    // update length
    cmd.length -= cmdStructLength;
}

void receiveEvent(int count)
{
    if (!count) {
        DBG1_PRINTF("no data received");
        return;
    }

    // all data is read from the buffer while being processed
    auto cmd = regMem.readCommand(count);
    if (cmd.command == TinyPwm::Commands::INVALID) {
        DBG1_PRINTF("Wire.available() = 0");
        return;
    }

    DBG1_PRINTF("receive event count=%u command=%u data=%02x %02x %02x %02x", count, cmd.command, cmd.raw[1], cmd.raw[2], cmd.raw[3], cmd.raw[4]); // sizeof(TinyPwm::Command);

    twiBuffer.clear();

    while(cmd.command != TinyPwm::Commands::END) {
        switch(cmd.command) {
            case TinyPwm::Commands::ANALOG_READ:
                if (CMD_CHECK_LENGTH(ANALOG_READ)) {
                    if (cmd.ANALOG_READ.pin < sizeof(TinyPwm::kPinCount)) {
                        twiBuffer.send(regMem.analog[cmd.ANALOG_READ.pin]);
                    }
                    else {
                        DBG1_PRINTF("ANALOG_READ pin=%u out of range (0-%u)", cmd.ANALOG_READ.pinNo, sizeof(kAnalogPins) - 1);
                    }
                    removeCommand(cmd, sizeof(cmd.ANALOG_READ));
                }
                break;
            case TinyPwm::Commands::ANALOG_WRITE:
                if (CMD_CHECK_LENGTH(ANALOG_WRITE)) {
                    regMem.analogWrite(cmd.ANALOG_WRITE.pin, cmd.ANALOG_WRITE.pwmValue);
                    removeCommand(cmd, sizeof(cmd.ANALOG_WRITE));
                }
                break;
            case TinyPwm::Commands::DIGITAL_READ:
                if (CMD_CHECK_LENGTH(DIGITAL_READ)) {
                    uint8_t state = regMem.digitalRead(cmd.DIGITAL_READ.pin);
                    twiBuffer.send(state);
                    removeCommand(cmd, sizeof(cmd.DIGITAL_READ));
                }
                break;
            case TinyPwm::Commands::DIGITAL_WRITE:
                if (CMD_CHECK_LENGTH(DIGITAL_WRITE)) {
                    regMem.digitalWrite(cmd.DIGITAL_WRITE.pin, cmd.DIGITAL_WRITE.value);
                    removeCommand(cmd, sizeof(cmd.DIGITAL_WRITE));
                }
                break;
            case TinyPwm::Commands::PIN_MODE:
                if (CMD_CHECK_LENGTH(PIN_MODE)) {
                    regMem.pinMode(cmd.PIN_MODE.pin, cmd.PIN_MODE.mode);
                    removeCommand(cmd, sizeof(cmd.PIN_MODE));
                }
                break;
            case TinyPwm::Commands::WRITE_EEPROM: {
                    uint16_t position = 0;
                    Config::Config config;
                    if (!config.read(position)) {
                        config.clear();
                    }
                    regMem.updateConfig(config.data());
                    position = 0;
                    config.write(position);
                    // backup copy in case the MCU resets during write
                    config.write(position);
                    removeCommand(cmd, 1);
                }
                break;
            case TinyPwm::Commands::RESET_EEPROM: {
                    // write default settings to EEPROM
                    Config::Config config;
                    config.clear();
                    uint16_t position = 0;
                    config.write(position);
                    config.write(position);

                    position = 0;
                    if (!config.read(position)) {
                        config.clear();
                    }
                    // reset device
                    regMem = RegMem(config.data());
                    regMem.begin();
                    return;
                }
                break;

            case TinyPwm::Commands::SET_PWM_PARAMS:
                if (CMD_CHECK_LENGTH(SET_PWM_PARAMS)) {
                    regMem.setPwmParams(cmd.SET_PWM_PARAMS.prescaler, cmd.SET_PWM_PARAMS.overflow);
                    removeCommand(cmd, sizeof(cmd.SET_PWM_PARAMS));
                }
                break;
            case TinyPwm::Commands::SET_PWM_FREQUENCY:
                if (CMD_CHECK_LENGTH(SET_PWM_FREQUENCY)) {
                    regMem.setPwmFrequency(cmd.SET_PWM_FREQUENCY.frequency);
                    removeCommand(cmd, sizeof(cmd.SET_PWM_FREQUENCY));
                }
                break;
            case TinyPwm::Commands::ADC_SET_AREF:
                if (CMD_CHECK_LENGTH(ADC_SET_AREF)) {
                    regMem.setAnalogReference(cmd.ADC_SET_AREF.mode);
                    removeCommand(cmd, sizeof(cmd.ADC_SET_AREF));
                }
                break;
            case TinyPwm::Commands::ADC_SET_READ_CYCLES:
                if (CMD_CHECK_LENGTH(ADC_SET_READ_CYCLES)) {
                    regMem.setAnalogCycles(cmd.ADC_SET_READ_CYCLES.pin, cmd.ADC_SET_READ_CYCLES.cycles);
                    removeCommand(cmd, sizeof(cmd.ADC_SET_READ_CYCLES));
                }
                break;
            case TinyPwm::Commands::READ_SFR:
                if (CMD_CHECK_LENGTH(READ_SFR)) {
                    twiBuffer.send(_SFR_IO8(cmd.READ_SFR.address));
                    removeCommand(cmd, sizeof(cmd.READ_SFR));
                }
                break;
            case TinyPwm::Commands::WRITE_SFR:
                if (CMD_CHECK_LENGTH(WRITE_SFR)) {
                    _SFR_IO8(cmd.WRITE_SFR.address) = cmd.WRITE_SFR.data;
                    removeCommand(cmd, sizeof(cmd.WRITE_SFR));
                }
                break;
            case TinyPwm::Commands::MASK_SFR:
                if (CMD_CHECK_LENGTH(MASK_SFR)) {
                    _SFR_IO8(cmd.MASK_SFR.address) = (_SFR_IO8(cmd.MASK_SFR.address) & cmd.MASK_SFR.mask) | cmd.MASK_SFR.data;
                    removeCommand(cmd, sizeof(cmd.MASK_SFR));
                }
                break;
            case TinyPwm::Commands::OR_SFR:
                if (CMD_CHECK_LENGTH(OR_SFR)) {
                    _SFR_IO8(cmd.OR_SFR.address) |= cmd.OR_SFR.data;
                    removeCommand(cmd, sizeof(cmd.OR_SFR));
                }
                break;
            case TinyPwm::Commands::AND_SFR:
                if (cmd.length >= sizeof(cmd.AND_SFR)) {
                    _SFR_IO8(cmd.AND_SFR.address) &= cmd.AND_SFR.data;
                    removeCommand(cmd, sizeof(cmd.AND_SFR));
                }
                break;
            case TinyPwm::Commands::XOR_SFR:
                if (cmd.length >= sizeof(cmd.XOR_SFR)) {
                    _SFR_IO8(cmd.XOR_SFR.address) ^= cmd.XOR_SFR.data;
                    removeCommand(cmd, sizeof(cmd.XOR_SFR));
                }
                break;
            case TinyPwm::Commands::CLI:
                cli();
                removeCommand(cmd, 0);
                break;
            case TinyPwm::Commands::SEI:
                sei();
                removeCommand(cmd, 0);
                break;
            case TinyPwm::Commands::INFO:
                twiBuffer.send(TinyPwm::Info());
                break;
            default:
                DBG1_PRINTF("invalid command=%u", cmd.command);
                return;
        }
    }
}

void setup()
{
    Config::Config config;
    uint16_t position = 0;
    if (!config.read(position)) {
        // try backup copy
        if (!config.read(position)) {
            // use default values
            config.clear();
        }

    }
    regMem = RegMem(config.data());
    regMem.begin();

    Wire.begin(regMem.address);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
}

void loop()
{
    // read the ADC pins constantly without blocking
    if (regMem.analogPinCount && ADCHelper::ready()) {
        regMem.readAnalogValue();
    }
}
