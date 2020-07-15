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


// ----------------------------------------------------------
// Function style macros
// ----------------------------------------------------------
#define DEBUG true
#define debug(...)   if (DEBUG) { Serial.print(__VA_ARGS__); }
#define debugln(...) if (DEBUG) { Serial.println(__VA_ARGS__); }

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

#define JSON_DOCUMENT_CAPACITY  200                /* Max size for payload in POST calls */
#define BODY_MIN_LENGTH 12                         /* Minimum size of payload */

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


String getFullFormattedTime(time_t rawtime) {
   struct tm* ti;
   ti = localtime (&rawtime);

   char buffer[26];
   strftime(buffer, 26, "%Y-%m-%d %H:%M:%S", ti);

   return String(buffer);
}

boolean isKugelventilGeschlossen() {
  uint8_t sb0 = rb[lastValidBufferIdx].buf[2]; // Get status byte 0 (byte 3) from last valid response string
  return (sb0 & 0x01) != 0;
}

boolean isKugelventilOffen() {
  return !isKugelventilGeschlossen();
}




/* Assumes the given durationString buffer to be 20 bytes large */
void getFormattedDuration(char* durationString, unsigned long from, unsigned long till = 0) {
  if (till == 0) {
    till = millis();
  } else if (till < from) {
    durationString[0] = '?';
    durationString[1] = '\0';
  }

  unsigned long durationSec = (till - from) / 1000L;
  String result = String((durationSec <= 120) ? (durationSec + " Sekunden") : (durationSec/60 + " Minuten"));
  result.toCharArray(durationString, 20);
}

bool parseHttpBodyToJson(StaticJsonDocument<JSON_DOCUMENT_CAPACITY>& jsonDocument) {
  bool result = false;

  // Read HTTP request body
  String body = httpRestServer.arg("plain");
  body.trim();
  debug("Request body (trimmed length: ");
  debug(body.length());
  debugln("):");
  debugln(body);

  if (body.length() >= BODY_MIN_LENGTH) {
    auto deserializationError = deserializeJson(jsonDocument, body);

    if (deserializationError) {
      debug("deserializeJson() failed with code ");
      debugln(deserializationError.c_str());
    } else {
      debugln("Parsed JSON is valid.");
      result = true;
    }
  }

  return result;
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
      debug(resBuf.buf[i], HEX);
      debug(" ");
    }

    debug("\nbufferValid: ");
    debugln(resBuf.valid);

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
  debug("Notification sent (success: ");
  debug(result);
  debug("): ");
  debugln(msg);

  if (notificationTimestamp != NULL) {
    *notificationTimestamp = resetTimestamp ? 0 : millis();
  }
}

boolean hasPushoverTokenSettings() {
  String pushoverSettings;
  readStringFromEEPROM(EEPROM_ADDR_PUSHOVER_ALREADY_SET, &pushoverSettings);

  debug("Read from EEPROM: '");
  debug(pushoverSettings);
  debugln("'");

  return pushoverSettings.length() == 3 && pushoverSettings.equals("YES");
}

boolean readPushoverSettings(String* pushoverAppToken, String* pushoverUserToken) {
  boolean hasSettings = hasPushoverTokenSettings();

  if (hasSettings) {
     readStringFromEEPROM(EEPROM_ADDR_PUSHOVER_APP_TOKEN, pushoverAppToken);
     readStringFromEEPROM(EEPROM_ADDR_PUSHOVER_USER_TOKEN, pushoverUserToken);
  }

  return hasSettings;
}

boolean savePushoverSettings(const String& pushoverAppToken, const String& pushoverUserToken) {
  boolean result = false;
  boolean appTokenSuccess = writeStringToEEPROM(EEPROM_ADDR_PUSHOVER_APP_TOKEN, pushoverAppToken, 30);
  boolean userTokenSuccess = writeStringToEEPROM(EEPROM_ADDR_PUSHOVER_USER_TOKEN, pushoverUserToken, 30);

  if (appTokenSuccess && userTokenSuccess) {
    debugln("Pushover app Token successfully written to EEPROM. Validating...");
    
    String readPushoverAppToken, readPushoverUserToken;
    readPushoverSettings(&readPushoverAppToken, &readPushoverUserToken);
    
    boolean appTokenValidated = pushoverAppToken.equals(readPushoverAppToken);
    debug("Pushover app token: '" + pushoverAppToken + "' equals '" + readPushoverAppToken + "'? -> " + appTokenValidated);
    boolean userTokenValidated = pushoverAppToken.equals(readPushoverAppToken);
    debug("Pushover app token: '" + pushoverAppToken + "' equals '" + readPushoverAppToken + "'? -> " + userTokenValidated);

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
  debugln("Initializing Pushover...");

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

  // Wasserstop is closed
  if (kugelventilGeschlossen) {
    // Wait at least the given amount of time before another push message is sent
    if ( canNotify(lastNotifiedKugelventilGeschlossen) ) {
      String msg = String("Wasserstop ist geschlossen wegen ");
      if (abschaltungsgrundWassermenge) {
        uint8_t limit = (b[28] + (b[29] << 8));
        msg += "Überschreitung der Wassermenge (Gesetztes Limit: ";
        msg += limit;
        msg += "L).";
      } else if (abschaltungsgrundDurchfluss) {
        uint8_t limit = (b[30] + (b[31] << 8));
        msg += "Überschreitung des Durchflusses (Gesetztes Limit: ";
        msg += limit;
        msg += "L/Std).";
      } else if (abschaltungsgrundEntnahmedauer) {
        uint8_t limit = (b[32] + (b[33] << 8)) / 120;
        msg += "Überschreitung der Entnahmedauer (Gesetztes Limit: ";
        msg += limit;
        msg += "min).";
      } else if (abschaltungsgrundLeckagesensor) {
        msg += "Überschwemmungsanzeige durch Leckagesensor.";
      } else {
        msg += "unbekanntem Grund (z.B. manuell geschlossen).";
      }

      sendNotification(msg, &lastNotifiedKugelventilGeschlossen);
    }
  } else {
    if (lastNotifiedKugelventilGeschlossen != 0) {
      getFormattedDuration(durationStr, lastNotifiedKugelventilGeschlossen);
      String msg = "Wasserstop nach " + String(durationStr) + " wieder geöffnet.";
      sendNotification(msg, &lastNotifiedKugelventilGeschlossen, /*resetTimestamp=*/ true);
    }
  }

  // Wasserstop signals a failure / erroneous state
  if (stoerung) {
    // Wait at least the given amount of time before another push message is sent
    if ( canNotify(lastNotifiedStoerung) ) {
      sendNotification("Wasserstop signalisiert eine Störung.", &lastNotifiedStoerung);
    }
  } else {
    if (lastNotifiedStoerung != 0) {
      getFormattedDuration(durationStr, lastNotifiedStoerung);
      String msg = "Störung nach " + String(durationStr) + " wieder beseitigt.";
      sendNotification(msg, &lastNotifiedStoerung, /*resetTimestamp=*/ true);
    }
  }

  // Battery is weak warning
  if (!batterieOk) {
    // Wait at least the given amount of time before another push message is sent
    if ( canNotify(lastNotifiedBatterieSchwach) ) {
      sendNotification("Wasserstop signalisiert, dass die Not-Batterien schwach sind.", &lastNotifiedBatterieSchwach);
    }
  } else {
    if (lastNotifiedBatterieSchwach != 0) {
      getFormattedDuration(durationStr, lastNotifiedBatterieSchwach);
      String msg = "Schwacher Batteriezustand nach " + String(durationStr) + " wieder beseitigt.";
      sendNotification(msg, &lastNotifiedBatterieSchwach, /*resetTimestamp=*/ true);
    }
  }

  // Runs on battery warning
  if (batterieBetrieb) {
    // Wait at least the given amount of time before another push message is sent
    if ( canNotify(lastNotifiedBatterieBetrieb) ) {
      sendNotification("Wasserstop ist in den Batteriebetrieb gewechselt.", &lastNotifiedBatterieBetrieb);
    }
  } else {
    if (lastNotifiedBatterieBetrieb != 0) {
      getFormattedDuration(durationStr, lastNotifiedBatterieBetrieb);
      String msg = "Nach " + String(durationStr) + " wieder zurück im regulären Stromnetzbetrieb.";
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

  doc["Timestamp"] = getFullFormattedTime(resBuf.queryTime);

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
  uint32_t calcWmOffset = 0; // TODO: Implement me
  statGesamtwassermenge["intern"] = calcWm;
  statGesamtwassermenge["offset"] = calcWmOffset;
  statGesamtwassermenge["total"]  = calcWm + calcWmOffset;
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

  serializeJsonPretty(doc, output, outputSize);
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
      debugln("Failed to connect to WiFi");
      delay(3000);
      ESP.restart();
    }

    debug("Connected to WiFi: ");
    debugln(WiFi.SSID());
    debug("IP address is: ");
    debugln(WiFi.localIP());

    if (digitalRead(WIFI_RESET_OPERATE_MODE) == LOW) {
      debug("Please set config pin back, so the ");
      debug(WIFI_CONFIG_ACCESS_POINT_NAME);
      debug(" can restart ");

      while (digitalRead(WIFI_RESET_OPERATE_MODE) == LOW) {
        debug(".");
        yield();     // let the ESP execute background tasks and reset watchdog
        delay(1000);
      }
    }
    debug("\nRestarting...");
    ESP.restart();
}

// ----------------------------------------------------------
// HTTP server
// (dispatching and request/response handling)
// ----------------------------------------------------------
void restGetRoot() {
  debugln("GET /");

  String usage = "\n\
  <h1>Wasserstop-REST-Schnittstelle</h1> \n\
  <h2>API-Doku</h2> \n\
  <h3>Nutzungshinweise</h3> \n\
  HTTP-Aufruf: <pre>GET /</pre> \n\
  CURL-Beispiel: <pre>curl -i --request GET http://ip-adresse/</pre> \n\
  <p>Aktion: Gibt diese API-Doku zur&uuml;ck.</p> \n\
  <h2>Auslesen und Steuern des Wasserstops</h2> \n\
  <h3>Abruf Betriebsdaten</h3> \n\
  HTTP-Aufruf: <pre>GET <a href=\"http://ip-adresse/all-data\">http://ip-adresse/all-data</a></pre> \n\
  CURL-Beispiel: <pre>curl -i --request GET http://ip-adresse/all-data</pre> \n\
  <p>Aktion: Gibt ein JSON-Objekt zur&uuml;ck mit den zuletzt gelesenen Einstellungen sowie dem Zeitstempel des Abrufs.</p> \n\
  <h3>Ventil &ouml;ffnen/schlie&szlig;en</h3> \n\
  HTTP-Aufruf: <pre>POST http://ip-adresse/ventil-auf-zu</pre> \n\
  CURL-Beispiel: <pre>curl -i --request POST http://ip-adresse/ventil-auf-zu</pre> \n\
  <p>Aktion: Sendet den Befehl zum &Ouml;ffnen/Schlie&szlig;en an den Wasserstop. Der aktuelle Status wird nicht ber&uuml;cksichtigt.</p> \n\
  <h3>Ventil &ouml;ffnen</h3> \n\
  HTTP-Aufruf: <pre>POST http://ip-adresse/ventil-auf</pre> \n\
  CURL-Beispiel: <pre>curl -i --request POST http://ip-adresse/ventil-auf</pre> \n\
  <p>Aktion: Sendet den Befehl zum &Ouml;ffnen an den Wasserstop, falls das Ventil geschlossen ist. Der Befehl hat nur einen Effekt, wenn sich das Kugelventil in offener Endlage befindet und der Motor nicht in Bewegung ist.</p> \n\
  <h3>Ventil schlie&szlig;en</h3> \n\
  HTTP-Aufruf: <pre>POST http://ip-adresse/ventil-zu</pre> \n\
  CURL-Beispiel: <pre>curl -i --request POST http://ip-adresse/ventil-zu</pre> \n\
  <p>Aktion: Sendet den Befehl zum Schlie&szlig;en an den Wasserstop, falls das Ventil geschlossen ist. Der Befehl hat nur einen Effekt, wenn sich das Kugelventil in geschlossener Endlage befindet und der Motor nicht in Bewegung ist.</p> \n\
  <br /> \n\
  <h2>Setzen der Einstellungen</h2> \n\
  <h3>Pushover Tokens setzen</h3> \n";

  if (!hasPushoverTokenSettings()) {
    usage += "<div style=\"border:2px solid red;\">Pushover-Einstellungen nicht gesetzt. Bitte zun&auml;chst einstellen.</div>\n";
  }

  usage += "HTTP-Aufruf: <pre>POST http://ip-adresse/set-pushover-tokens</pre> \n\
  CURL-Beispiel: <pre>curl -i --request POST --data '{\"app-token\":\"alwucdbvppok4i9g7a44lnvvv3o8qo\", \"user-token\":\"v4kg1pnw7i9hbpvuqap4q96h0krxe5\"}' http://ip-adresse/set-pushover-tokens</pre> \n\
  <p>Aktion: Speichert die Pushover-Einstellungen im nicht-fl&uuml;chtigen Speicher.</p> \n\
  Parameter: <em>app-token</em> \n\
  <div style=\"padding-left:4em;\">30 Zeichen langer alphanumerischer String</div><br /> \n\
  <em>user-token</em> \n\
  <div style=\"padding-left:4em;\">30 Zeichen langer alphanumerischer String</div> \n\
  <h3>Wasserstop-Gateway neu starten</h3> \n\
  HTTP-Aufruf: <pre>POST http://ip-adresse/restart</pre> \n\
  CURL-Beispiel: <pre>curl -i --request POST http://ip-adresse/restart</pre> \n\
  <p>Aktion: Versucht den ESP8266 neu zu starten.</p> \n";

  usage.replace("ip-adresse", WiFi.localIP().toString());
  httpRestServer.send(200, "text/html", usage);
}

void restAllData() {
  debugln("GET /all-data");
  char output[2048];
  decodeRawBetriebsdatenResponse(rb[lastValidBufferIdx], output, 2048);
  httpRestServer.send(200, "text/html", output);
}

void restVentilAufZu() {
  debugln("POST /ventil-auf-zu");
  boolean success = sendOpenCloseSignalToWasserstop();

  int responseCode = success ? 200 : 503;
  httpRestServer.send(responseCode, "text/html", "");
}

void restVentilAuf() {
 debugln("POST /ventil-auf");
 int responseCode = 200;
 if (isKugelventilGeschlossen()) {
  restVentilAufZu();
 } else {
  httpRestServer.send(424, "text/html", "Valve already open"); // Failed Dependency
 }
}

void restVentilZu() {
 debugln("POST /ventil-zu");
 int responseCode = 200;
 if (isKugelventilOffen()) {
  restVentilAufZu();
 } else {
  httpRestServer.send(424, "text/html", "Valve already closed"); // Failed Dependency
 }
}

void restSetPushoverTokens() {
  debugln("POST /set-pushover-tokens");

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
          httpRestServer.send(200, "text/html", "Ok");
        } else {
          httpRestServer.send(500, "text/html", "Unable to save Pushover tokens");
        }
      } else {
        httpRestServer.send(422, "text/html", "Pushover user token not valid. Must be 30 characters long."); 
      }
    } else {
      httpRestServer.send(422, "text/html", "Pushover app token not valid. Must be 30 characters long."); 
    }
  } else {
    httpRestServer.send(422, "text/html", "JSON in HTTP header invalid"); 
  }  
}

void restRestart() {
  httpRestServer.send(202, "text/html", "Restarting...");
  ESP.restart();
}

void startRestService() {
  debugln("Starting REST server");
  //httpRestServer = new ESP8266WebServer(WiFi.localIP(), REST_SERVER_HTTP_PORT));
  httpRestServer.on("/", HTTP_GET, restGetRoot);
  httpRestServer.on("/all-data", HTTP_GET, restAllData);
  httpRestServer.on("/ventil-auf-zu", HTTP_POST, restVentilAufZu);
  httpRestServer.on("/ventil-auf", HTTP_POST, restVentilAuf);
  httpRestServer.on("/ventil-zu", HTTP_POST, restVentilZu);
  httpRestServer.on("/set-pushover-tokens", HTTP_POST, restSetPushoverTokens);


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
  debugln("\n Starting");

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
}


void loop() {
  // Check if WiFiManagers config portal is requested
  if ( digitalRead(WIFI_RESET_OPERATE_MODE) == LOW ) {
    debugln("WiFi Manager config requested");
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

  if (WiFi.status() == WL_CONNECTED) {
    if (lastPollingTime < millis()-WASSERSTOP_DEFAULT_POLLING_INTERVAL) {
      debugln("Request data from Wasserstop...");
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
    debugln("Not connected to WiFi");
    delay(1000);
  }

  timeClient.update();
}
