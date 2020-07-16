#pragma once

#include <Arduino.h>
#include <EEPROM.h>

boolean writeStringToEEPROM(int addressOffset, const String &str, int maxLength = -1);

void readStringFromEEPROM(int addressOffset, String *strRead, int maxLength = -1);
