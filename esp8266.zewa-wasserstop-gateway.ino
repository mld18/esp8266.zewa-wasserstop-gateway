#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <ESP8266WebServer.h> .   //https://github.com/esp8266/Arduino/tree/master/libraries
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include "Pushover.h"             //https://github.com/ArduinoHannover/Pushover
#include <SoftwareSerial.h>       //https://github.com/plerup/espsoftwareserial
#include <ArduinoJson.h>
#include <NTPClient.h>            //https://github.com/arduino-libraries/NTPClient
#include <WiFiUdp.h>              //for NTPClient
#include <time.h>                 //for localtime
#include "EEPROMString.h"         //own module with more convenient EEPROM functions
#include <pgmspace.h>
extern "C" {
  #include "user_interface.h"     //used to set hostname
}


// ----------------------------------------------------------
// Function style macros
// ----------------------------------------------------------
#define DEBUG true
#define debugf(fmt_s, ...)  if (DEBUG) { Serial.printf_P((PGM_P) PSTR(fmt_s), ## __VA_ARGS__); }
#define debugfln(fmt_s, ...) if (DEBUG) { Serial.printf_P((PGM_P) PSTR(fmt_s"\n"), ## __VA_ARGS__); }


// ----------------------------------------------------------
// Constants
// ----------------------------------------------------------
/*
 *  Connect the pin WIFI_RESET_OPERATE_MODE to GND in order to trigger the
 *  start of the configuration console.
 */
#define WIFI_RESET_OPERATE_MODE D7

/*
 * Name of the access point when
 */
#define WIFI_CONFIG_ACCESS_POINT_NAME "WasserstopGateway"

#define REST_SERVER_HTTP_PORT 80                   /* TCP port to serve HTTP GET and POST requests */

#define SERIAL_TX_PIN      D6                      /* UART/serial to the Wasserstop unit send pin */
#define SERIAL_RX_PIN      D5                      /* UART/serial to the Wasserstop unit receive pin */
#define SERIAL_BAUD_RATE   9600                    /* Baud rate to the Wasserstop unit */
#define SERIAL_UART_MODE   SWSERIAL_8N1            /* UART mode: 8 data bits, no parity bit, 1 stop bit */
#define SERIAL_RX_BUF_SIZE 90                      /* 01h messages return 81 bytes, so we have a few bytes more as buffer */

#define WASSERSTOP_READ_TIMEOUT_MS 1000            /* Max. wait time between data request and response. */
#define WASSERSTOP_DEFAULT_POLLING_INTERVAL 2000   /* Request data from Wasserstop about every x milliseconds - provided that a WiFi connection is present */

#define NTP_TIME_OFFSET 2*60*60                    /* 2h offset to UTC */

#define NOTIFICATION_PAUSE_INTERVAL 15*60*1000L    /* pause sending push notifications for the same event for at least X milliseconds */

#define EEPROM_ADDR_PUSHOVER_ALREADY_SET  0        /* EEPROM (0): will hold length byte plus strings 'YES' or '-NO' */
#define EEPROM_ADDR_PUSHOVER_APP_TOKEN    5        /* EEPROM (5): will hold length byte plus 30 character long alphanumeric character array containing the Pushover app token */
#define EEPROM_ADDR_PUSHOVER_USER_TOKEN   40       /* EEPROM (40): will hold length byte plus 30 character long alphanumeric character array containing the Pushover user/group token */
#define EEPROM_ADDR_OFFSET_VERBRAUCH_SET  75       /* EEPROM (75): will hold length byte plus strings 'YES' or '-NO' indicating if the offset water comsumption has been set */
#define EEPROM_ADDR_OFFSET_VERBRAUCH      80       /* EEPROM (80): will hold 4 bytes with the offset consumption if EEPROM_ADDR_OFFSET_VERBRAUCH_SET is set to YES */


#define WEBAPP_POLLING_INTERVAL 2500               /* Request data from this gateway about every x milliseconds */

#define JSON_DOCUMENT_CAPACITY  200                /* Max size for payload in POST calls */
#define BODY_MIN_LENGTH 12                         /* Minimum size of payload */

#define CT_TEXT_HTML F("text/html")                /* Return content type for web server response */
#define CT_APPLICATION_JSON F("application/json")  /* Return content type for web server response */

// ----------------------------------------------------------
// Global objects
// ----------------------------------------------------------
// Taking commands / serving data
ESP8266WebServer httpRestServer(REST_SERVER_HTTP_PORT);
boolean restServerRunning = false;

// sending notifications
Pushover* po = NULL;
String poAppToken;
String poUserToken;
unsigned long lastNotifiedKugelventilGeschlossen = 0;    /* Timestamp, when a Pushover notification was send. 0 if no notification has been sent yet. ULONG_MAX if you don't want a refresh */
unsigned long lastNotifiedStoerung = 0;
unsigned long lastNotifiedBatterieSchwach = 0;
unsigned long lastNotifiedBatterieBetrieb = 0;

// Reading data from Wasserstop
SoftwareSerial swSer(SERIAL_RX_PIN, SERIAL_TX_PIN);

struct ResponseBuffer
{
   uint8_t* buf = new uint8_t[95];
   time_t queryTime = 0;
   boolean valid = false;
   size_t len = 0;
   uint16_t crcSum = 0;
};
ResponseBuffer rb[2];
int lastValidBufferIdx = 0;
unsigned long lastPollingTime = 0;

// NTP Client
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

// Waterconsumption offset (liters)
uint32_t waterconsumptionOffset = 0xFFFFFFFF;  /* Init to its max value. Once read, the value is 0 <= v < 0xFFFFFFFF */



// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// Helper functions
// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
boolean waitUntilDataAvailable() {
  for (int i=0; i<WASSERSTOP_READ_TIMEOUT_MS/50 && swSer.available()==0; i++) {
    yield();   // use the waiting time and let the ESP handle other stuff
    delay(50);
  }
  return swSer.available()>0;
}

boolean canNotify (unsigned long notificationTimestamp) {
  unsigned long now = millis();
  return (notificationTimestamp == 0) ||            // unset yet
         ( NOTIFICATION_PAUSE_INTERVAL <= now &&    // watch out for overruns, we're dealing with unsigned here
           notificationTimestamp < now-NOTIFICATION_PAUSE_INTERVAL );
}


template<size_t N> void getFullFormattedTime(char(&result)[N], time_t rawtime) {
   struct tm* ti;
   ti = localtime (&rawtime);

   strftime(result, 26, "%Y-%m-%d %H:%M:%S", ti);
}

boolean isKugelventilGeschlossen() {
  uint8_t sb0 = rb[lastValidBufferIdx].buf[2]; // Get status byte 0 (byte 3) from last valid response string
  return (sb0 & 0x01) != 0;
}

boolean isKugelventilOffen() {
  return !isKugelventilGeschlossen();
}

template<size_t N> void getFormattedDuration(char (&result)[N], unsigned long from, unsigned long till = 0) {
  if (till == 0) {
    till = millis();
  }
  
  if (till < from) {
    strncpy(result, "?", N);
  } else {
    unsigned long durationSec = (till - from) / 1000L;
    if (durationSec <= 120) {
      snprintf_P(result, N, PSTR("%ld Sekunden"), durationSec);
    } else {
      snprintf_P(result, N, PSTR("%ld Minuten"), durationSec/60);
    }
  }
}

bool parseHttpBodyToJson(StaticJsonDocument<JSON_DOCUMENT_CAPACITY>& jsonDocument) {
  bool result = false;

  // Read HTTP request body
  String body = httpRestServer.arg("plain");
  body.trim();

  if (body.length() >= BODY_MIN_LENGTH) {
    auto deserializationError = deserializeJson(jsonDocument, body);

    if (deserializationError) {
      debugf("deserializeJson() failed with code ");
      if (DEBUG) { Serial.println(deserializationError.c_str()); }
    } else {
      debugfln("Parsed JSON is valid.");
      result = true;
    }
  }

  return result;
}


boolean hasWaterconsumptionOffsetInEEPROM() {
  String wmOffsetYesNo;
  readStringFromEEPROM(EEPROM_ADDR_OFFSET_VERBRAUCH_SET, &wmOffsetYesNo, 4);

  debugfln("Read from EEPROM, if water consumption offset is saved: '%s'", wmOffsetYesNo.c_str());

  return wmOffsetYesNo.length() == 3 && wmOffsetYesNo.equals("YES"); // only return true, if set to YES
}

void readWaterconsumptionOffsetFromEEPROM() {
 if (hasWaterconsumptionOffsetInEEPROM()) {
    uint8_t bytes[4];
    for (int i=0; i<4; i++) {
      bytes[i] = EEPROM.read(EEPROM_ADDR_OFFSET_VERBRAUCH+i);
    }
    debugfln("readWaterconsumptionOffsetFromEEPROM: bytes[0]=0x%02X, bytes[1]=0x%02X, bytes[2]=0x%02X, bytes[3]=0x%02X", bytes[0], bytes[1], bytes[2], bytes[3]);
    waterconsumptionOffset = bytes[0] + (bytes[1]<<8) + (bytes[2]<<16) + (bytes[3]<<24);
    debugfln("Water consumption offset read from EEPROM is: %d", waterconsumptionOffset);
  } else {
    waterconsumptionOffset = 0;
    debugfln("No offset in EEPROM => set water consumption offset to 0");
  } 
}

void writeWaterconsumptionOffsetToEEPROM() {

  String wmOffsetYesNo = F("-NO");  // -NO, when commit failed or offset <= 0
  if (0<waterconsumptionOffset && waterconsumptionOffset<0xFFFFFFFF) {
    uint8_t bytes[4] = { waterconsumptionOffset & 0x000000FF,
                         (waterconsumptionOffset & 0x0000FF00) >> 8,
                         (waterconsumptionOffset & 0x00FF0000) >> 16,
                         (waterconsumptionOffset & 0xFF000000) >> 24 };

    debugfln("writeWaterconsumptionOffsetToEEPROM: bytes[0]=0x%02X, bytes[1]=0x%02X, bytes[2]=0x%02X, bytes[3]=0x%02X", bytes[0], bytes[1], bytes[2], bytes[3]);
    for (int i=0; i<4; i++) {
      EEPROM.write(EEPROM_ADDR_OFFSET_VERBRAUCH+i, bytes[i]);
    }
    boolean committed = EEPROM.commit();

    if (committed) {
      wmOffsetYesNo = F("YES");
    } else {
      debugfln("Commit of water consumption offset failed!.");
    }
  } else {
    debugfln("Water consumption value of 0 or 0xFFFFFFFF leads to reset of EEPROM storage for water consumption value to -NO.");
  }
  writeStringToEEPROM(EEPROM_ADDR_OFFSET_VERBRAUCH_SET, wmOffsetYesNo);
}

// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// Business Logic Layer
// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

// ----------------------------------------------------------
// Exchange data with Wasserstop unit
// ----------------------------------------------------------
void readBetriebsdatenFromWasserstop(ResponseBuffer& resBuf) {
  byte idx = 0;

  digitalWrite(LED_BUILTIN, LOW);
  swSer.write(0xAA);
  swSer.write(0x01);

  delay(200);

  if ( waitUntilDataAvailable() ) {
    while (swSer.available() > 0) {
      uint8_t b = swSer.read();

      if (b == 0xEE) { // Read start byte of response. Invalidate buffer.
        resBuf.valid = false;
        idx = 0;
        resBuf.len = 0;
        resBuf.crcSum = 0;
      } else if (resBuf.len == 0 && idx == 1) {
        // Number of data bytes including checksum byte excluding first two bytes.
        // Is always(?) 0x2e (=46)
        resBuf.len = b+2;
        resBuf.crcSum = 0;
      }

      resBuf.buf[idx] = b; // save read byte
      idx++;
      resBuf.crcSum += (idx < resBuf.len) ? b : 0; // exclude last byte from checksum calculation

      if (idx < resBuf.len) {
        waitUntilDataAvailable();
      }
    }
    resBuf.crcSum = resBuf.crcSum & 0xFF;
    resBuf.valid = (idx > 0) && (idx == resBuf.len) && (resBuf.crcSum == resBuf.buf[idx-1]);
    resBuf.queryTime = timeClient.getEpochTime();

    for (int i=0; i<resBuf.len; i++) {
      debugf("%02X ", resBuf.buf[i]);
    }

    debugfln("\nbufferValid: %i", resBuf.valid);

    digitalWrite(LED_BUILTIN, HIGH);
  }
}

boolean sendOpenCloseSignalToWasserstop() {
  digitalWrite(LED_BUILTIN, LOW);
  swSer.write(0xAA);
  swSer.write(0x02);

  delay(WASSERSTOP_READ_TIMEOUT_MS);

  uint8_t response[2];
  for (int i=0; i<2 && waitUntilDataAvailable(); i++) {
    response[i] = swSer.read();
  }

  boolean success = (response[0] == 0xee && response[1] == 0x99);

  digitalWrite(LED_BUILTIN, HIGH);

  return success;
}


// ----------------------------------------------------------
// Notifications and Pushover
// ----------------------------------------------------------
void sendNotification(const String msg, unsigned long* notificationTimestamp = NULL, boolean resetTimestamp = false) {
  po->setMessage(msg);
  boolean result = po->send();
  debugfln("Notification sent (success: %d): %s", result, msg.c_str());

  if (notificationTimestamp != NULL) {
    *notificationTimestamp = resetTimestamp ? 0 : millis();
  }
}

void unsetPushoverSettings() {
  String pushoverSet = "-NO";
  debugfln("Unsetting/initializing Pushover settings in EEPROM...");
  writeStringToEEPROM(EEPROM_ADDR_PUSHOVER_ALREADY_SET, pushoverSet);
}

boolean hasPushoverTokenSettings() {
  String pushoverSettings;
  readStringFromEEPROM(EEPROM_ADDR_PUSHOVER_ALREADY_SET, &pushoverSettings, 4);

  debugfln("Read from EEPROM: '%s'", pushoverSettings.c_str());

  boolean result = pushoverSettings.length() == 3 && pushoverSettings.equals("YES");
  if (!result && !pushoverSettings.equals("-NO")) {
    unsetPushoverSettings(); // upon first read -> invalid/uninitialized field in memory
  }
  return result;
}

boolean readPushoverSettings(String* pushoverAppToken, String* pushoverUserToken, boolean ignoreHasSettings = false) {
  boolean hasSettings = hasPushoverTokenSettings();

  if (hasSettings || ignoreHasSettings) {
    readStringFromEEPROM(EEPROM_ADDR_PUSHOVER_APP_TOKEN, pushoverAppToken);
    readStringFromEEPROM(EEPROM_ADDR_PUSHOVER_USER_TOKEN, pushoverUserToken);
    debugfln("Read Pushover settings from EEPROM. App token: '%s'. User token: '%s'.", pushoverAppToken->c_str(), pushoverUserToken->c_str());
  }

  return hasSettings;
}

boolean savePushoverSettings(const String& pushoverAppToken, const String& pushoverUserToken) {
  boolean result = false;
  boolean appTokenSuccess = writeStringToEEPROM(EEPROM_ADDR_PUSHOVER_APP_TOKEN, pushoverAppToken, 30);
  boolean userTokenSuccess = writeStringToEEPROM(EEPROM_ADDR_PUSHOVER_USER_TOKEN, pushoverUserToken, 30);

  if (appTokenSuccess && userTokenSuccess) {
    debugfln("Pushover app Token successfully written to EEPROM. Validating...");
    
    String readPushoverAppToken, readPushoverUserToken;
    readPushoverSettings(&readPushoverAppToken, &readPushoverUserToken, /*ignoreHasSettings=*/ true);
    
    boolean appTokenValidated = pushoverAppToken.equals(readPushoverAppToken);
    debugfln("Pushover app token: '%s' equals '%s'? -> %d", pushoverAppToken.c_str(), readPushoverAppToken.c_str(), appTokenValidated);
    boolean userTokenValidated = pushoverUserToken.equals(readPushoverUserToken);
    debugfln("Pushover user token: '%s' equals '%s'? -> %d", pushoverUserToken.c_str(), readPushoverUserToken.c_str(), userTokenValidated);

    if (appTokenValidated && userTokenValidated) {
      String pushoverSet = "YES";
      writeStringToEEPROM(EEPROM_ADDR_PUSHOVER_ALREADY_SET, pushoverSet);
      result = true;
    }
  }

  if (!result && hasPushoverTokenSettings()) {    // Unset Pushover settings
    String pushoverSet = "-NO";
    writeStringToEEPROM(EEPROM_ADDR_PUSHOVER_ALREADY_SET, pushoverSet);
  }

  return result;
}

void initializePushover() {
  debugfln("Initializing Pushover...");

  readPushoverSettings(&poAppToken, &poUserToken);

  if (po != NULL) {
    delete po;
  }
  po = new Pushover(poAppToken, poUserToken, UNSAFE);
}

void sendPushNotificationOnError(ResponseBuffer& resBuf) {

  if (!hasPushoverTokenSettings()) {
    return;   // Doesn't make sense to check if a notification should be sent, if sending won't be possible
  }

  uint8_t* b = resBuf.buf;

  // 3. Byte: Statusbyte 0
  uint8_t sb0 = b[2];
  // 4. Byte: Statusbyte 1
  uint8_t sb1 = b[3];

  boolean kugelventilGeschlossen = (sb0 & 0x01) != 0;
  boolean stoerung = (sb0 & 0x40);

  boolean abschaltungsgrundWassermenge   = (sb0 & 0x02) != 0;
  boolean abschaltungsgrundDurchfluss    = (sb0 & 0x04) != 0;
  boolean abschaltungsgrundEntnahmedauer = (sb0 & 0x08) != 0;
  boolean abschaltungsgrundLeckagesensor = (sb1 & 0x08) != 0;

  boolean batterieOk = (sb1 & 0x02) == 0;
  boolean batterieBetrieb ((sb1 & 0x01) != 0);

  char durationStr[20];
  char msg[100];

  // Wasserstop is closed
  if (kugelventilGeschlossen) {
    // Wait at least the given amount of time before another push message is sent
    if ( canNotify(lastNotifiedKugelventilGeschlossen) ) {
      #define STR_WASSERSTOP_IST_GESCHLOSSEN_WEGEN "Wasserstop ist geschlossen wegen "
      
      if (abschaltungsgrundWassermenge) {
        uint8_t limit = (b[28] + (b[29] << 8));
        sprintf_P(msg, PSTR(STR_WASSERSTOP_IST_GESCHLOSSEN_WEGEN"Überschreitung der Wassermenge (Gesetztes Limit: %d l)."), limit);
      } else if (abschaltungsgrundDurchfluss) {
        uint8_t limit = (b[30] + (b[31] << 8));
        sprintf_P(msg, PSTR(STR_WASSERSTOP_IST_GESCHLOSSEN_WEGEN"Überschreitung des Durchflusses (Gesetztes Limit: %d l/Std.)."), limit);
      } else if (abschaltungsgrundEntnahmedauer) {
        uint8_t limit = (b[32] + (b[33] << 8)) / 120;
        sprintf_P(msg, PSTR(STR_WASSERSTOP_IST_GESCHLOSSEN_WEGEN"Überschreitung der Entnahmedauer (Gesetztes Limit: %d min)."), limit);
      } else if (abschaltungsgrundLeckagesensor) {
        sprintf_P(msg, PSTR(STR_WASSERSTOP_IST_GESCHLOSSEN_WEGEN"Überschwemmungsanzeige durch Leckagesensor."));
      } else {
        sprintf_P(msg, PSTR(STR_WASSERSTOP_IST_GESCHLOSSEN_WEGEN"unbekanntem Grund (z.B. manuell geschlossen)."));
      }

      sendNotification(msg, &lastNotifiedKugelventilGeschlossen);
    }
  } else {
    if (lastNotifiedKugelventilGeschlossen != 0) {
      getFormattedDuration(durationStr, lastNotifiedKugelventilGeschlossen);
      sprintf_P(msg, PSTR("Wasserstop nach %s wieder geöffnet."), durationStr);
      sendNotification(msg, &lastNotifiedKugelventilGeschlossen, /*resetTimestamp=*/ true);
    }
  }

  // Wasserstop signals a failure / erroneous state
  if (stoerung) {
    // Wait at least the given amount of time before another push message is sent
    if ( canNotify(lastNotifiedStoerung) ) {
      sendNotification(F("Wasserstop signalisiert eine Störung."), &lastNotifiedStoerung);
    }
  } else {
    if (lastNotifiedStoerung != 0) {
      getFormattedDuration(durationStr, lastNotifiedStoerung);
      sprintf_P(msg, PSTR("Störung nach %s wieder beseitigt."), durationStr);
      sendNotification(msg, &lastNotifiedStoerung, /*resetTimestamp=*/ true);
    }
  }

  // Battery is weak warning
  if (!batterieOk) {
    // Wait at least the given amount of time before another push message is sent
    if ( canNotify(lastNotifiedBatterieSchwach) ) {
      sendNotification(F("Wasserstop signalisiert, dass die Not-Batterien schwach sind."), &lastNotifiedBatterieSchwach);
    }
  } else {
    if (lastNotifiedBatterieSchwach != 0) {
      getFormattedDuration(durationStr, lastNotifiedBatterieSchwach);
      sprintf_P(msg, PSTR("Schwacher Batteriezustand nach %s wieder beseitigt."), durationStr);
      sendNotification(msg, &lastNotifiedBatterieSchwach, /*resetTimestamp=*/ true);
    }
  }

  // Runs on battery warning
  if (batterieBetrieb) {
    // Wait at least the given amount of time before another push message is sent
    if ( canNotify(lastNotifiedBatterieBetrieb) ) {
      sendNotification(F("Wasserstop ist in den Batteriebetrieb gewechselt."), &lastNotifiedBatterieBetrieb);
    }
  } else {
    if (lastNotifiedBatterieBetrieb != 0) {
      getFormattedDuration(durationStr, lastNotifiedBatterieBetrieb);
      sprintf_P(msg, PSTR("Nach %s wieder zurück im regulären Stromnetzbetrieb."), durationStr);
      sendNotification(msg, &lastNotifiedBatterieBetrieb, /*resetTimestamp=*/ true);
    }
  }
}


// ----------------------------------------------------------
// Create JSON response with all data
// ----------------------------------------------------------
void decodeRawBetriebsdatenResponse(ResponseBuffer& resBuf, char *output, size_t outputSize) {
  DynamicJsonDocument doc(2048);
  uint8_t* b = resBuf.buf;

  // 3. Byte: Statusbyte 0
  uint8_t sb0 = b[2];
  // 4. Byte: Statusbyte 1
  uint8_t sb1 = b[3];

  char ts[20];
  getFullFormattedTime(ts, resBuf.queryTime);
  doc["Timestamp"] = ts;

  JsonObject kugelventil     = doc.createNestedObject("Kugelventil");
  kugelventil["Geschlossen"] = (sb0 & 0x01) != 0;
  kugelventil["Motor_an"]    = (sb0 & 0x80) != 0;

  JsonObject abschaltungsgrund = doc.createNestedObject("Abschaltungsgrund");
  abschaltungsgrund["Wassermenge"]   = (sb0 & 0x02) != 0;
  abschaltungsgrund["Durchfluss"]    = (sb0 & 0x04) != 0;
  abschaltungsgrund["Entnahmedauer"] = (sb0 & 0x08) != 0;
  abschaltungsgrund["Leckagesensor"] = (sb1 & 0x08) != 0;

  JsonObject stat = doc.createNestedObject("Status");
  JsonObject statGesamtwassermenge = stat.createNestedObject("Gesamtwassermenge");
  uint32_t calcWm = 100 * (b[40] + (b[41] << 8) + (b[42] << 16) + (b[43] << 24));
  statGesamtwassermenge["intern"] = calcWm;
  statGesamtwassermenge["offset"] = waterconsumptionOffset;
  statGesamtwassermenge["total"]  = calcWm + waterconsumptionOffset;
  stat["Urlaubsmodus"] = (sb0 & 0x10) != 0;
  stat["Standbymodus"] = (sb0 & 0x20) != 0;

  JsonObject stoerung = doc.createNestedObject("Stoerung");
  stoerung["Aktuell_gestoert"] = (sb0 & 0x40) != 0;
  stoerung["Anzahl_Motor_Nocken_Defekt"] = b[26];
  stoerung["Anzahl_Verbindung_Notstrom"] = b[27];
  stoerung["Datenverbindung_gestoert"] = (sb1 & 0x04) != 0;

  JsonObject stromversorgung = doc.createNestedObject("Stromversorgung");
  stromversorgung["Quelle"] = ((sb1 & 0x01) != 0) ? "Batterie" : "Stromnetz";
  stromversorgung["Batterie_ok"] = (sb1 & 0x02) == 0;
  stromversorgung["Batteriespannung"] = 0.07906f * b[4];
  stromversorgung["Notstrommodulspannung"] = 0.1556f * b[5];

  JsonObject wassermenge = doc.createNestedObject("Wassermenge");
  wassermenge["Aktuell"] = (b[34] + (b[35] << 8));
  wassermenge["Grenzwert"] = (b[28] + (b[29] << 8));
  JsonObject wassermengeAbschaltungen = wassermenge.createNestedObject("Abschaltungen");
  wassermengeAbschaltungen["100l"] = b[6];
  wassermengeAbschaltungen["200l"] = b[7];
  wassermengeAbschaltungen["500l"] = b[8];
  wassermengeAbschaltungen["1000l"] = b[9];
  wassermengeAbschaltungen["2000l"] = b[10];
  wassermengeAbschaltungen["3000l"] = b[11];

  JsonObject durchfluss = doc.createNestedObject("Durchfluss");
  durchfluss["Aktuell"] = (b[36] + (b[37] << 8));
  durchfluss["Grenzwert"] = (b[30] + (b[31] << 8));
  JsonObject durchflussAbschaltungen = durchfluss.createNestedObject("Abschaltungen");
  durchflussAbschaltungen["500lh"] = b[12];
  durchflussAbschaltungen["1000lh"] = b[13];
  durchflussAbschaltungen["2000lh"] = b[14];
  durchflussAbschaltungen["3000lh"] = b[15];
  durchflussAbschaltungen["4000lh"] = b[16];
  durchflussAbschaltungen["5000lh"] = b[17];

  JsonObject entnahmezeit = doc.createNestedObject("Entnahmezeit");
  entnahmezeit["Aktuell"] = (b[38] + (b[39] << 8)) / 2; // in seconds
  entnahmezeit["Grenzwert"] = (b[32] + (b[33] << 8)) / 2; // in seconds
  JsonObject entnahmezeitAbschaltungen = entnahmezeit.createNestedObject("Abschaltungen");
  entnahmezeitAbschaltungen["6m"] = b[18];
  entnahmezeitAbschaltungen["12m"] = b[19];
  entnahmezeitAbschaltungen["18m"] = b[20];
  entnahmezeitAbschaltungen["30m"] = b[21];
  entnahmezeitAbschaltungen["60m"] = b[22];
  entnahmezeitAbschaltungen["120m"] = b[23];

  JsonObject externeAnschluesse = doc.createNestedObject("Externe_Anschluesse");
  externeAnschluesse["Eingangssignal_an"] = (sb1 & 0x10) != 0;
  externeAnschluesse["Ausgang_Kugelventil_geschlossen"] = (sb1 & 0x20) != 0;
  externeAnschluesse["Ausgang_ohne_Stoerung_in_Betrieb"] = (sb1 & 0x40) != 0;
  externeAnschluesse["Ausgang_100l_Impuls"] = (sb1 & 0x80) != 0;

  serializeJson(doc, output, outputSize);
}

// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// Connection Layer
// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

// ----------------------------------------------------------
// WIFI related functions
// ----------------------------------------------------------
void displayWifiManagerConfigSite() {
    WiFiManager wifiManager;

    if (!wifiManager.startConfigPortal(WIFI_CONFIG_ACCESS_POINT_NAME)) {
      debugf("Failed to connect to WiFi\n");
      delay(3000);
      ESP.restart();
    }

    debugf("Connected to WiFi: %s\nIP address is: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());

    if (digitalRead(WIFI_RESET_OPERATE_MODE) == LOW) {
      debugf("Please set config pin back, so the %s can restart", WIFI_CONFIG_ACCESS_POINT_NAME);

      while (digitalRead(WIFI_RESET_OPERATE_MODE) == LOW) {
        debugf(".");
        yield();     // let the ESP execute background tasks and reset watchdog
        delay(1000);
      }
    }
    debugf("\nRestarting...");
    ESP.restart();
}

// ----------------------------------------------------------
// HTTP server
// (dispatching and request/response handling)
// ----------------------------------------------------------
void restGetRoot() {
  debugfln("GET /");

  PGM_P usage = PSTR("\n\
<h1>Wasserstop-REST-Schnittstelle</h1> \n\
<h2>API-Doku</h2> \n\
<h3>Nutzungshinweise</h3> \n\
HTTP-Aufruf: <pre>GET /</pre> \n\
CURL-Beispiel: <pre>curl -i --request GET http://%s/</pre> \n\
<p>Aktion: Gibt diese API-Doku zur&uuml;ck.</p> \n\
<h2>Benutzerschnittstelle</h2> \n\
<h3>HTML-WebApp</h3> \n\
HTTP-Aufruf: <pre>GET <a href=\"http://%s/webapp\">http://%s/webapp</a></pre> \n\
<br /> \n\
<h2>Auslesen und Steuern des Wasserstops</h2> \n\
<h3>Abruf Betriebsdaten</h3> \n\
HTTP-Aufruf: <pre>GET <a href=\"http://%s/all-data\">http://%s/all-data</a></pre> \n\
CURL-Beispiel: <pre>curl -i --request GET http://%s/all-data</pre> \n\
<p>Aktion: Gibt ein JSON-Objekt zur&uuml;ck mit den zuletzt gelesenen Einstellungen sowie dem Zeitstempel des Abrufs.</p> \n\
<h3>Ventil &ouml;ffnen/schlie&szlig;en</h3> \n\
HTTP-Aufruf: <pre>POST http://%s/ventil-auf-zu</pre> \n\
CURL-Beispiel: <pre>curl -i --request POST http://%s/ventil-auf-zu</pre> \n\
<p>Aktion: Sendet den Befehl zum &Ouml;ffnen/Schlie&szlig;en an den Wasserstop. Der aktuelle Status wird nicht ber&uuml;cksichtigt.</p> \n\
<h3>Ventil &ouml;ffnen</h3> \n\
HTTP-Aufruf: <pre>POST http://%s/ventil-auf</pre> \n\
CURL-Beispiel: <pre>curl -i --request POST http://%s/ventil-auf</pre> \n\
<p>Aktion: Sendet den Befehl zum &Ouml;ffnen an den Wasserstop, falls das Ventil geschlossen ist. Der Befehl hat nur einen Effekt, wenn sich das Kugelventil in offener Endlage befindet und der Motor nicht in Bewegung ist.</p> \n\
<h3>Ventil schlie&szlig;en</h3> \n\
HTTP-Aufruf: <pre>POST http://%s/ventil-zu</pre> \n\
CURL-Beispiel: <pre>curl -i --request POST http://%s/ventil-zu</pre> \n\
<p>Aktion: Sendet den Befehl zum Schlie&szlig;en an den Wasserstop, falls das Ventil geschlossen ist. Der Befehl hat nur einen Effekt, wenn sich das Kugelventil in geschlossener Endlage befindet und der Motor nicht in Bewegung ist.</p> \n\
<br /> \n\
<h2>Setzen der Einstellungen</h2> \n\
<h3>Pushover Tokens setzen</h3> \n\
<div style=\"border:2px solid red; display:%s\">Pushover-Einstellungen nicht gesetzt. Bitte zun&auml;chst einstellen.</div>\n\
HTTP-Aufruf: <pre>POST http://%s/set-pushover-tokens</pre> \n\
CURL-Beispiel: <pre>curl -i --request POST --data '{\"app-token\":\"alwucdbvppok4i9g7a44lnvvv3o8qo\", \"user-token\":\"v4kg1pnw7i9hbpvuqap4q96h0krxe5\"}' http://%s/set-pushover-tokens</pre> \n\
<p>Aktion: Speichert die Pushover-Einstellungen im nicht-fl&uuml;chtigen Speicher.</p> \n\
Parameter: <em>app-token</em> \n\
<div style=\"padding-left:4em;\">30 Zeichen langer alphanumerischer String</div><br /> \n\
<em>user-token</em> \n\
<div style=\"padding-left:4em;\">30 Zeichen langer alphanumerischer String</div> \n\
<h3>Pushover-Benachrichtigung testen</h3> \n\
HTTP-Aufruf: <pre>POST http://%s/test-notification</pre> \n\
CURL-Beispiel: <pre>curl -i --request POST http://%s/test-notification</pre> \n\
<p>Aktion: Sendet eine Test-Benachrichtigung, wenn Pushover richtig konfiguriert ist..</p> \n\
<h3>Wasserz&auml;hler-Offset</h3> \n\
HTTP-Aufruf: <pre>POST http://%s/set-wasserzaehler-offset</pre> \n\
CURL-Beispiel: <pre>curl -i --request POST --data '{\"value\":120435}' http://%s/set-wasserzaehler-offset</pre> \n\
<p>Aktion: Speichert die in value in Litern angegebene Wassermenge als Offset-Wert ab. In <pre>/all-data</pre> wird hiermit die Gesamtwassermenge, wie sie \n\
auch dem Wasserz&auml;hler entnommen werden kann, korrekt angegeben.</p> \n\
Parameter: <em>value</em> \n\
<div style=\"padding-left:4em;\">Ganzzahliger Wert gr&ouml;&szlig;er oder gleich Null, der den aktuellen Stand laut Wasserz&auml;hler in Litern (Kubikmeter x 1000) angibt. Intern wird von \n\
diesem der aktuelle Z&auml;hlerstand (im Judo ZEWA Wasserstop) abgezogen, um daraus den Offset zu errechnen.</div> \n\
<h3>Wasserstop-Gateway neu starten</h3> \n\
HTTP-Aufruf: <pre>POST http://%s/restart</pre> \n\
CURL-Beispiel: <pre>curl -i --request POST http://%s/restart</pre> \n\
<p>Aktion: Versucht den ESP8266 neu zu starten.</p> \n");

  char ip[16];
  WiFi.localIP().toString().toCharArray(ip, 16);

  char *htmlOutputBuffer = new char[10000];
  snprintf_P(htmlOutputBuffer, 10000, usage, ip, ip, ip, ip, ip, ip, ip, ip, ip, ip, ip, ip, hasPushoverTokenSettings() ? "none" : "block", ip, ip, ip, ip, ip, ip, ip, ip);

  httpRestServer.send(200, CT_TEXT_HTML, htmlOutputBuffer);
  free(htmlOutputBuffer);
}

void restAllData() {
  debugfln("GET /all-data");
  char output[2048];
  decodeRawBetriebsdatenResponse(rb[lastValidBufferIdx], output, 2048);
  httpRestServer.send(200, CT_APPLICATION_JSON, output);
}

void restVentilAufZu() {
  debugfln("POST /ventil-auf-zu");
  boolean success = sendOpenCloseSignalToWasserstop();

  int responseCode = success ? 200 : 503;
  httpRestServer.send(responseCode, CT_TEXT_HTML, "");
}

void restVentilAuf() {
 debugfln("POST /ventil-auf");
 int responseCode = 200;
 if (isKugelventilGeschlossen()) {
  restVentilAufZu();
 } else {
  httpRestServer.send(424, CT_TEXT_HTML, F("Valve already open")); // Failed Dependency
 }
}

void restVentilZu() {
 debugfln("POST /ventil-zu");
 int responseCode = 200;
 if (isKugelventilOffen()) {
  restVentilAufZu();
 } else {
  httpRestServer.send(424, CT_TEXT_HTML, F("Valve already closed")); // Failed Dependency
 }
}

void restSetPushoverTokens() {
  debugfln("POST /set-pushover-tokens");

  StaticJsonDocument<JSON_DOCUMENT_CAPACITY> json;
  bool validJson = parseHttpBodyToJson(json);

  if (validJson) {
    String pushoverAppToken = json["app-token"].as<String>();
    String pushoverUserToken = json["user-token"].as<String>();

    if (pushoverAppToken!=NULL && pushoverAppToken.length()==30) {
      if (pushoverUserToken!=NULL && pushoverUserToken.length()==30) {
        boolean success = savePushoverSettings(pushoverAppToken, pushoverUserToken);
        if (success) {
          poAppToken = pushoverAppToken;
          poUserToken = pushoverUserToken;
          initializePushover();
          httpRestServer.send(200, CT_TEXT_HTML, F("Ok"));
        } else {
          httpRestServer.send(500, CT_TEXT_HTML, F("Unable to save Pushover tokens"));
        }
      } else {
        httpRestServer.send(422, CT_TEXT_HTML, F("Pushover user token not valid. Must be 30 characters long.")); 
      }
    } else {
      httpRestServer.send(422, CT_TEXT_HTML, F("Pushover app token not valid. Must be 30 characters long.")); 
    }
  } else {
    httpRestServer.send(422, CT_TEXT_HTML, F("JSON in HTTP header invalid")); 
  }  
}

void restSendTestNotification() {
  debugfln("POST /test-notification");

  if (hasPushoverTokenSettings()) {
    sendNotification(F("Test-Benachrichtigung"));
    httpRestServer.send(200, CT_TEXT_HTML, "Ok");
  } else {
    httpRestServer.send(423, CT_TEXT_HTML, F("Pushover app tokens are not configured properly.")); 
  }
}

void restRestart() {
  debugfln("POST /restart");
  httpRestServer.send(202, CT_TEXT_HTML, F("Restarting..."));
  ESP.restart();
}

void restWebapp() {
  debugfln("GET /webapp");
  
  PGM_P htmlTemplate = PSTR("<!DOCTYPE html><html lang=\"de\"> <head> <title>Wasserstop</title> <meta http-equiv=\"Content-Type\" content=\"text/html; charset=utf-8\"/> <meta http-equiv=\"cache-control\" content=\"no-cache\"> <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"> <meta name=\"apple-mobile-web-app-capable\" content=\"yes\"> <meta name=\"apple-mobile-web-app-status-bar-style\" content=\"black-translucent\"> <link rel=\"apple-touch-icon\" href=\"https://upload.wikimedia.org/wikipedia/commons/thumb/8/83/Stop_hand_nuvola_blue.svg/240px-Stop_hand_nuvola_blue.svg.png\"/> <style>BODY{font-family: calibri,sans-serif; margin: 0;}#heading{border-bottom: 1px solid #333; background-color: #000; color: #ddd; text-align: center; font-weight: bold; height: 1.5em; padding-top: 0.5em;}DIV.topic-box{border: 1px solid gray; border-radius: 5px; margin: 2em 3vw 0 3vw;}DIV.topic-box P.caption{margin: -0.7em 1em 0.5em 1em; padding-left: 0.5em; background: white; display: block; width: 82vw; white-space: nowrap; font-weight: bold;}DIV.topic-box TABLE{width: 100%%; padding-left: 1em; padding-bottom: 0.5em;}DIV.topic-box TABLE TD P{margin: 0.1em 0 0.1em 0; display: inline-grid;}DIV.topic-box TABLE TD P.messwert{width: 3.6em;}DIV.topic-box TABLE TD P.von{padding-left: 1em; padding-right: 1em;}DIV.topic-box TABLE TD:first-child{width: 40%%;}DIV.buttonbar{display: flex; flex-wrap: nowrap; align-items: center; justify-content: center;}DIV.buttonbar BUTTON{display: grid; align-items: center; justify-content: center; width: 30vw; height: 10vw; border-radius: 5px; margin-bottom: 1em;}DIV.buttonbar BUTTON.open{background-color: green; margin-right: 10vw;}DIV.buttonbar BUTTON.close{background-color: #a00; margin-left: 10vw;}.good{color: green;}.bad{color: red;}.neutral{color: black;}.pulsating{animation: pulsating 1.5s infinite;}@keyframes pulsating{0%%{opacity: 0.2;}50%%{opacity: 1.0;}100%%{opacity: 0.2;}}#motorOeffnet, #motorSchliesst{padding-left: 1em;}.hidden{display: none !important;}</style> <script type='text/javascript'> const pollingInterval=%ld; function setText(id, text, cssClass='neutral'){var e=document.getElementById(id); e.innerText=text; e.className=cssClass;}function setGoodBadText(id, isGood, goodText, badText){if (isGood){setText(id, goodText, 'good');}else{setText(id, badText, 'bad');}}function refresh(){var xhr=new XMLHttpRequest(); xhr.open('GET', '/all-data', true); xhr.timeout=pollingInterval - 100; xhr.overrideMimeType('application/json'); xhr.setRequestHeader('Content-Type', 'application/json'); xhr.onload=function (){var data=JSON.parse(this.responseText); var kvGeschlossen=data['Kugelventil']['Geschlossen']; setGoodBadText('kugelventil', !kvGeschlossen, 'offen', 'geschlossen'); var motorAn=data['Kugelventil']['Motor_an']; var eAuf=document.getElementById('motorOeffnet'); var eZu=document.getElementById('motorSchliesst'); eAuf.className='pulsating good ' + ((!motorAn || (motorAn && !kvGeschlossen)) ? 'hidden' : ''); eZu.className='pulsating bad ' + ((!motorAn || (motorAn && kvGeschlossen)) ? 'hidden' : ''); var quelle=(data['Stromversorgung']['Quelle']); setText('stromversorgung', quelle, (quelle=='Batterie') ? 'bad' : 'good'); setGoodBadText('stoerung', !data['Stoerung']['Aktuell_gestoert'], 'keine', 'gestört'); setText('verbrauchszaehler', (data['Status']['Gesamtwassermenge']['total']/1000).toFixed(1)+' m³'); var spannung=' ('+data['Stromversorgung']['Batteriespannung']+' V)'; setGoodBadText('batteriezustand', data['Stromversorgung']['Batterie_ok'], 'ok'+spannung, 'schwach'+spannung); var wmAus=data['Abschaltungsgrund']['Wassermenge']; setText('messungWassermenge', data['Wassermenge']['Aktuell']+' l', 'messwert ' + (wmAus ? 'bad' : 'neutral')); setText('grenzwertWassermenge', data['Wassermenge']['Grenzwert']+' l', 'neutral maxwert'); var dfAus=data['Abschaltungsgrund']['Durchfluss']; setText('messungDurchfluss', data['Durchfluss']['Aktuell']+' l/h', 'messwert ' + (dfAus ? 'bad' : 'neutral')); setText('grenzwertDurchfluss', data['Durchfluss']['Grenzwert']+' l/h', 'neutral maxwert'); var entAus=data['Abschaltungsgrund']['Entnahmedauer']; var entnahmedauer=data['Entnahmezeit']['Aktuell']; setText('messungEntnahmezeit', (entnahmedauer<120 ? entnahmedauer+' s' : (entnahmedauer/60).toFixed(0)+' min'), 'messwert ' + (entAus ? 'bad' : 'neutral')); setText('grenzwertEntnahmezeit', data['Entnahmezeit']['Grenzwert']+' min', 'neutral maxwert'); var nass=data['Abschaltungsgrund']['Leckagesensor']; setText('messungLeckagesensor', nass ? 'nass' : 'trocken', 'messwert ' + (nass ? 'bad' : 'neutral')); setText('urlaubsmodus', data['Status']['Urlaubsmodus'] ? 'an' : 'aus'); setText('standbymodus', data['Status']['Standbymodus'] ? 'an' : 'aus');}; xhr.send(null); setTimeout(refresh, pollingInterval);}refresh(); setTimeout(refresh, pollingInterval); </script> </head> <body> <div id=\"heading\"> Wasserstop-Steuerung </div><div class=\"topic-box\"> <p class=\"caption\">Betriebsstatus</p><table> <tr> <td><label>Kugelventil</label></td><td><p id=\"kugelventil\" class=\"good\"></p><p id=\"motorOeffnet\" class=\"pulsating hidden good\">(&ouml;ffnet)</p><p id=\"motorSchliesst\" class=\"pulsating hidden bad\">(schlie&szlig;t)</p></td></tr><tr> <td><label>Stromversorgung</label></td><td><p id=\"stromversorgung\" class=\"bad\"></p></td></tr><tr> <td><label>St&ouml;rung</label></td><td><p id=\"stoerung\" class=\"good\"></p></td></tr><tr> <td><label>Verbrauchsz&auml;hler</label></td><td><p id=\"verbrauchszaehler\" class=\"neutral\"></p></td></tr><tr> <td><label>Batteriezustand</label></td><td><p id=\"batteriezustand\" class=\"bad\"></p></td></tr></table> </div><div class=\"topic-box\"> <p class=\"caption\">Messung / Abschaltgrund</p><table> <tr> <td><label>Wassermenge</label></td><td><p id=\"messungWassermenge\" class=\"neutral messwert\"></p><p class=\"von\">von</p><p id=\"grenzwertWassermenge\" class=\"neutral maxwert\"></p></td></tr><tr> <td><label>Durchfluss</label></td><td><p id=\"messungDurchfluss\" class=\"neutral messwert\"></p><p class=\"von\">von</p><p id=\"grenzwertDurchfluss\" class=\"neutral maxwert\"></p></td></tr><tr> <td><label>Entnahmezeit</label></td><td><p id=\"messungEntnahmezeit\" class=\"bad messwert\"></p><p class=\"von\">von</p><p id=\"grenzwertEntnahmezeit\" class=\"neutral maxwert\"></p></td></tr><tr> <td><label>Leckagesensor</label></td><td><p id=\"messungLeckagesensor\" class=\"neutral messwert\"></p></td></tr></table> </div><div class=\"topic-box\"> <p class=\"caption\">Betriebsmodi</p><table> <tr> <td><label>Urlaubsmodus</label></td><td><p id=\"urlaubsmodus\" class=\"neutral\"></p></td></tr><tr> <td><label>Stand-by-Modus</label></td><td><p id=\"standbymodus\" class=\"neutral\"></p></td></tr></table> </div><div class=\"topic-box\"> <p class=\"caption\">Ventilsteuerung</p><div class=\"buttonbar\"> <form method=\"POST\" action=\"http://%s/ventil-auf\" name=\"formVentilOeffnen\" target=\"hiddenFrame\" onsubmit=\"alert('Wasserstop wird ge&ouml;ffnet.')\"> <button type=\"submit\" name=\"ventilOeffnen\" class=\"open\">&Ouml;ffnen</button> </form> <form method=\"POST\" action=\"http://%s/ventil-zu\" name=\"formVentilSchliessen\" target=\"hiddenFrame\" onsubmit=\"alert('Wasserstop wird geschlossen.')\"> <button type=\"submit\" name=\"ventilSchliessen\" class=\"close\">Schlie&szlig;en</button> </form> </div><iframe name=\"hiddenFrame\" width=\"0\" height=\"0\" border=\"0\" style=\"display: none;\"></iframe> </div></body></html>");

  char ip[16];
  WiFi.localIP().toString().toCharArray(ip, 16);

  char *htmlOutputBuffer = new char[10000];
  snprintf_P(htmlOutputBuffer, 10000, htmlTemplate, WEBAPP_POLLING_INTERVAL, ip, ip);
  
  httpRestServer.send(200, CT_TEXT_HTML, htmlOutputBuffer);
  free(htmlOutputBuffer);
}

void restSetWasserzaehlerOffset() {
  debugfln("POST /set-wasserzaehler-offset");

  StaticJsonDocument<JSON_DOCUMENT_CAPACITY> json;
  bool validJson = parseHttpBodyToJson(json);
  
  if (validJson) {
    uint32_t value = json["value"];

    if (0<=value && value<0xFFFFFFFF) { 
      uint8_t* b = rb[lastValidBufferIdx].buf;
      uint32_t currentWaterconsumption = 100 * (b[40] + (b[41] << 8) + (b[42] << 16) + (b[43] << 24));
      waterconsumptionOffset = value - currentWaterconsumption;
      
      writeWaterconsumptionOffsetToEEPROM();
      httpRestServer.send(200, CT_TEXT_HTML, F("Water consumption offset value set."));
    } else {
      httpRestServer.send(400, CT_TEXT_HTML, F("Setting the offset consumption only works with a 'value' greater than 0 liters and smaller than 4294967295 liters.")); 
    }
  } else {
    httpRestServer.send(422, CT_TEXT_HTML, F("JSON in HTTP header invalid")); 
  }  
}

void startRestService() {
  debugfln("Starting REST server");
  
  httpRestServer.on("/", HTTP_GET, restGetRoot);
  httpRestServer.on("/all-data", HTTP_GET, restAllData);
  httpRestServer.on("/ventil-auf-zu", HTTP_POST, restVentilAufZu);
  httpRestServer.on("/ventil-auf", HTTP_POST, restVentilAuf);
  httpRestServer.on("/ventil-zu", HTTP_POST, restVentilZu);
  httpRestServer.on("/set-pushover-tokens", HTTP_POST, restSetPushoverTokens);
  httpRestServer.on("/test-notification", HTTP_POST, restSendTestNotification);
  httpRestServer.on("/restart", HTTP_POST, restRestart);
  httpRestServer.on("/webapp", HTTP_GET, restWebapp);
  httpRestServer.on("/set-wasserzaehler-offset", HTTP_POST, restSetWasserzaehlerOffset);
  
  httpRestServer.begin();
  restServerRunning = true;
}



// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// Control Flow Core
// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

// ----------------------------------------------------------
// Arduino entry functions
// ----------------------------------------------------------
void setup() {
  Serial.begin(115200);
  EEPROM.begin(128); 
  debugfln("\n Starting");

  // init serial connection to Wasserstop
  swSer.begin(SERIAL_BAUD_RATE,
              SERIAL_UART_MODE,
              SERIAL_RX_PIN,
              SERIAL_TX_PIN,
              /*invert=*/ false,
              /*bufCapacity=*/ SERIAL_RX_BUF_SIZE);

  // Initialize pins
  pinMode(WIFI_RESET_OPERATE_MODE, INPUT_PULLUP);

  // Start NTP Client
  timeClient.begin();
  timeClient.setTimeOffset(NTP_TIME_OFFSET);

  // Initialize hostname of ESP8266
  wifi_station_set_hostname(WIFI_CONFIG_ACCESS_POINT_NAME);
}


void loop() {
  // Check if WiFiManagers config portal is requested
  if ( digitalRead(WIFI_RESET_OPERATE_MODE) == LOW ) {
    debugfln("WiFi Manager config requested");
    // Stop our web server in order to allow WiFiManager's web server to start up
    httpRestServer.stop();
    restServerRunning = false;
    displayWifiManagerConfigSite();
  } else {
    if (!restServerRunning) {
      startRestService();
    }
  }

  // Initialize Pushover if not initialized yet
  if (po == NULL) {
    initializePushover();
  }

  if (waterconsumptionOffset == 0xFFFFFFFF) {
    readWaterconsumptionOffsetFromEEPROM();
  }

  if (WiFi.status() == WL_CONNECTED) {
    if (lastPollingTime < millis()-WASSERSTOP_DEFAULT_POLLING_INTERVAL) {
      debugfln("Request data from Wasserstop...");
      int nextBufferIdx = lastValidBufferIdx ? 0 : 1;
      readBetriebsdatenFromWasserstop(rb[nextBufferIdx]);

      if (rb[nextBufferIdx].valid) {
        lastValidBufferIdx = nextBufferIdx;
        lastPollingTime = millis();
      }

      yield();

      sendPushNotificationOnError(rb[lastValidBufferIdx]);
    }

    yield();

    if (restServerRunning) {
      httpRestServer.handleClient();
    }
  } else {
    debugfln("Not connected to WiFi");
    delay(1000);
  }

  timeClient.update();
}
