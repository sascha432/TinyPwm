/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "main.h"

#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE ( 16 )
#endif

class TwiBuffer {
public:
    static constexpr size_t kBufferSize = 32;

    TwiBuffer();

    template<typename _Ta>
    void send(const _Ta &t) {
        send(&t, sizeof(t));
    }

    void send(const void *data, size_t length);
    void send(uint8_t data);
    void clear();
    void sendTo(TwoWire &wire);

private:
    uint8_t _buffer[kBufferSize];
    uint8_t _position;
    uint8_t _length;
};

inline TwiBuffer::TwiBuffer() :
        _position(0),
        _length(0)
{
}

inline void TwiBuffer::send(const void *data, size_t length)
{
    auto ptr = reinterpret_cast<const uint8_t *>(data);
    auto end = ptr + length;
    while(ptr < end) {
        if (_length >= kBufferSize) {
            return;
        }
        _buffer[_length++] = *ptr++;
    }
}

inline void TwiBuffer::send(uint8_t data)
{
    if (_length >= kBufferSize) {
        return;
    }
    _buffer[_length++] = data;
}

inline void TwiBuffer::clear()
{
    _length = 0;
    _position = 0;
}

inline void TwiBuffer::sendTo(TwoWire &wire)
{
    DBG1_PRINTF("request event pos=%u length=%u", _position, _length);
    auto ptr = _buffer + _position;
    auto end = _buffer + _length;
    while (ptr < end) {
        wire.write(*ptr++);
        _position++;
    }
    clear();
}

