#include "EEPROMString.h"

boolean writeStringToEEPROM(int addressOffset, const String &str, int maxLength) {

  int len = (maxLength>=0 && maxLength<str.length()) ? maxLength : str.length();
  //Serial.println("Writing String '" + strToWrite.substring(0,len) + "' to EEPROM. (Length: " + len + ")");
  EEPROM.write(addressOffset, len);
  for (int i=0; i<len; i++) {
    EEPROM.write(addressOffset+1+i, str.charAt(i));
  }
  boolean committed = EEPROM.commit();
  //Serial.println(committed ? "EEPROM successfully committed\n" : "ERROR! EEPROM commit failed\n");

  return committed;
}

void readStringFromEEPROM(int addressOffset, String *strRead) {
  int len = EEPROM.read(addressOffset);
  char data[len + 1];

  for (int i=0; i<len; i++) {
    data[i] = EEPROM.read(addressOffset+1+i);
  }
  data[len] = '\0';
  *strRead = String(data);
}
