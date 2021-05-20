# TinyPWM IO Expander

IO Expander for ATTiny45/85 that offers one PWM output channel and up to 3 analog input channels using the I2C bus

# I2C Slave

The default address is 0x60
I2C has been tested with Arduino UNO, Nano and the ESP8266
Examples of I2C commands can be found in [tinypwm.h](tinypwm.h)

## PWM Output Channel

Pin 1 can be used to output a PWM signal with a given frequency from 488 to 65535Hz. For different frequencies timer1 can be programmed directly

### PWM frequency

The frequency can be set with the `SET_PWM_FREQUENCY` command. The data for the command is the frequency as uint16.

Example for 20kHz

0x60(slave address) 0x51(SET_PWM_FREQUENCY) 0x20 0x4e(PWM frequency)

### PWM Duty Cycle

The duty cycle is set with the `ANALOG_WRITE` command and the data is the PWM value as uint8_t

Example for 50% duty cycle

`0x60 0x10(ANALOG_WRITE) 0x7f(127)`

The max. duty cycle is 255, but not all frequencies support the full range. For these cases the values are mapped

## ADC Input Channels

Pin 3, 4 and 5 can be used as ADC input channels. If Pin5 is used, the reset pin must be disabled by programming the Fuse High Byte (see Table 20-4, [https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-2586-AVR-8-bit-Microcontroller-ATtiny25-ATtiny45-ATtiny85_Datasheet.pdf](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-2586-AVR-8-bit-Microcontroller-ATtiny25-ATtiny45-ATtiny85_Datasheet.pdf))

### Reading Analog Channels

The selected channels are constantly read and the values or averages value stored. Requesting a reading does not block and returns the last stored value. By default, the internal 2.56V reference is set

`0x60 0x11(ANALOG_READ) 0x00(pin number)`

The response is an uint16 with the value.

The pin number is a sequential number in the order of the analog pins specified during compilation

Set voltage reference to 1.1V

`0x60 0x62(ADC_SET_AREF) 0x02(internal 1.1V)`


# Changelog

## 0.0.2

 - up to 3 analog input channels

## 0.0.1

- Initial version