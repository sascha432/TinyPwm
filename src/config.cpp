/**
  Author: sascha_lammers@gmx.de
*/

#include <Arduino.h>
#include <EEPROM.h>
#include "config.h"

namespace Config {

    void Config::clear()
    {
        *this = Config();
    }

    bool Config::read(uint16_t &position)
    {
        EEPROM.get(position, _header);
        position += sizeof(_header);
        if (_header.magic != kHeaderMagic) {
            position += sizeof(_data);
            return false;
        }
        EEPROM.get(position, _data);
        position += sizeof(_data);
        if (_header.crc != crc()) {
            return false;
        }
        return false;
    }

    void Config::write(uint16_t &position)
    {
        _header.magic = kHeaderMagic;
        _header.crc = crc();
        EEPROM.put(position, _header);
        position += sizeof(_header);
        EEPROM.put(position, _data);
        position += sizeof(_data);
    }

}