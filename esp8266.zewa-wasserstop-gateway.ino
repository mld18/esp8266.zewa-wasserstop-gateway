#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <ESP8266WebServer.h> .   //https://github.com/esp8266/Arduino/tree/master/libraries
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include "Pushover.h"             //https://github.com/ArduinoHannover/Pushover
#include <SoftwareSerial.h>       //https://github.com/plerup/espsoftwareserial
#include <ArduinoJson.h>
#include <NTPClient.h>            //https://github.com/arduino-libraries/NTPClient
#include <WiFiUdp.h>              //for NTPClient
#include <time.h>                 //for localtime


// ----------------------------------------------------------
// Function style macros
// ----------------------------------------------------------
#define DEBUG true
#define debug(S)   if (DEBUG) { Serial.print(S); }
#define debugln(S) if (DEBUG) { Serial.println(S); }

// ----------------------------------------------------------
// Constants
// ----------------------------------------------------------
/*
 *  Connect the pin WIFI_RESET_OPERATE_MODE to GND before startup in order to trigger the
 *  start of the configuration console. For normal operation connect it to 3V3.
 */
#define WIFI_RESET_OPERATE_MODE D7

#define WIFI_CONFIG_ACCESS_POINT_NAME "WasserstopGateway"

#define REST_SERVER_HTTP_PORT 80

#define SERIAL_TX_PIN      D6
#define SERIAL_RX_PIN      D5
#define SERIAL_BAUD_RATE   9600
#define SERIAL_UART_MODE   SWSERIAL_8N1
#define SERIAL_RX_BUF_SIZE 90           /* 01h messages return 81 bytes, so we have a few bytes more as buffer */

#define WASSERSTOP_READ_TIMEOUT_MS 1000
#define WASSERSTOP_DEFAULT_POLLING_INTERVAL 2000

#define NTP_TIME_OFFSET 2*60*60         /* 2h offset */

#define NOTIFICATION_PAUSE_INTERVAL 15*60*1000L    /* pause sending push notifications for the same event for at least X milliseconds */

// ----------------------------------------------------------
// Global objects
// ----------------------------------------------------------
// Taking commands / serving data
ESP8266WebServer httpRestServer(REST_SERVER_HTTP_PORT);
boolean restServerRunning = false;

// sending notifications
Pushover* po = NULL;
unsigned long lastNotifiedKugelventilGeschlossen = 0;
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

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);


// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// Helper functions
// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
void initializePushover() {
  // TODO: load configuration from EEPROM
  debugln("Initializing Pushover...");

  String pushoverAppToken = "/* TODO: Should be set via REST call */";
  String pushoverUserToken = "/* TODO: Should be set via REST call */";

  po = new Pushover(pushoverAppToken, pushoverUserToken, UNSAFE);
}

boolean waitUntilDataAvailable() {
  for (int i=0; i<WASSERSTOP_READ_TIMEOUT_MS/50 && swSer.available()==0; i++) {
    yield();
    delay(50);
  }
  return swSer.available()>0;
}

boolean canNotify (unsigned long notificationTimestamp) {
  unsigned long now = millis();
  return (notificationTimestamp == 0) ||                // unset yet
         ( NOTIFICATION_PAUSE_INTERVAL <= now &&    // watch out for overruns, we're dealing with unsigned here
           notificationTimestamp < now-NOTIFICATION_PAUSE_INTERVAL );
}

void sendNotification(String msg, unsigned long* notificationTimestamp = NULL, boolean resetTimestamp = false) {
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

String getFormattedDuration(unsigned long from, unsigned long till = 0) {
  if (till == 0) {
    till = millis();
  } else if (till < from) {
    return "?";
  }

  unsigned long durationSec = (till - from) / 1000L;
  return String((durationSec <= 120) ? (durationSec + " Sekunden") : (durationSec/60 + " Minuten"));
}

// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// Business Logic Layer
// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
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
    debugln("\n----");
    for (int i=0; i<resBuf.len; i++) {
      if (DEBUG) Serial.print(resBuf.buf[i], HEX);
      debug(" ");
    }

    debug("\nbufferValid: ");
    debugln(resBuf.valid);
    if (!resBuf.valid) {
      debug("idx > 0: ");
      debugln(idx > 0);
      debug("idx == resBuf.len: ");
      debugln(idx == resBuf.len);
      debug("resBuf.crcSum == resBuf.buf[idx-1]: ");
      debugln(resBuf.crcSum == resBuf.buf[idx-1]);
      debug("resBuf.len: ");
      debugln(resBuf.len);
    }

    digitalWrite(LED_BUILTIN, HIGH);
  }
}

boolean sendOpenCloseSignalToWasserstop() {
  digitalWrite(LED_BUILTIN, LOW);
  swSer.write(0xAA);
  swSer.write(0x02);

  delay(1000);

  uint8_t response[2];
  for (int i=0; i<2 && waitUntilDataAvailable(); i++) {
    response[i] = swSer.read();
  }

  boolean success = (response[0] == 0xee && response[1] == 0x99);

  digitalWrite(LED_BUILTIN, HIGH);

  return success;
}

void sendPushNotificationOnError(ResponseBuffer& resBuf) {

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
      String msg = "Wasserstop nach " + getFormattedDuration(lastNotifiedKugelventilGeschlossen) + " wieder geöffnet.";
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
      String msg = "Störung nach " + getFormattedDuration(lastNotifiedStoerung) + " wieder beseitigt.";
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
      String msg = "Schwacher Batteriezustand nach " + getFormattedDuration(lastNotifiedBatterieSchwach) + " wieder beseitigt.";
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
      String msg = "Nach " + getFormattedDuration(lastNotifiedBatterieBetrieb) + " wieder zurück im regulären Stromnetzbetrieb.";
      sendNotification(msg, &lastNotifiedBatterieBetrieb, /*resetTimestamp=*/ true);
    }
  }

}

void decodeRawBetriebsdatenResponse(ResponseBuffer& resBuf, char *output, size_t outputSize) {
  DynamicJsonDocument doc(2048);
  uint8_t* b = resBuf.buf;

  Serial.print("==> resBuf.buf: ");
  for (int i=0; i<resBuf.len; i++) {
    Serial.print(b[i], HEX);
    Serial.print(" ");
  }

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

  // TODO: use minified JSON
  //serializeJson(doc, output, outputSize);
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
      Serial.println("Failed to connect to WiFi");
      delay(3000);
      //reset and try again, or maybe put it to deep sleep
      ESP.reset();  // TODO: Checkout reboot or maybe forward wait endlessly until the config pin is put to HIGH again
      delay(5000);
    }

    Serial.print("Connected to WiFi: ");
    Serial.println(WiFi.SSID());
    Serial.print("IP address is: ");
    Serial.println(WiFi.localIP());
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
  <p>Aktion: Sendet den Befehl zum &Ouml;ffnen an den Wasserstop, falls das Ventil geschlossen ist. Wenn das Ventil in Bewegung ist, wird bis zum Erreichen der Endlage gewartet.</p> \n\
  <h3>Ventil schlie&szlig;en</h3> \n\
  HTTP-Aufruf: <pre>POST http://ip-adresse/ventil-zu</pre> \n\
  CURL-Beispiel: <pre>curl -i --request POST http://ip-adresse/ventil-zu</pre> \n\
  <p>Aktion: Sendet den Befehl zum Schlie&szlig;en an den Wasserstop, falls das Ventil geschlossen ist. Wenn das Ventil in Bewegung ist, wird bis zum Erreichen der Endlage gewartet.</p> \n\
  <br /> \n\
  <h2>Setzen der Einstellungen</h2> \n\
  ";

  /* TODO: We need to have setters for the difference configuration settings. First should be Pushover.
     After Pushover settings are given, we can send secret tokens to the user that can be used to securely
     set stuff. */

  usage.replace("ip-adresse", WiFi.localIP().toString());
  httpRestServer.send(200, "text/html", usage);
}

void restAllData() {
  char output[2048];
  decodeRawBetriebsdatenResponse(rb[lastValidBufferIdx], output, 2048);
  httpRestServer.send(200, "text/html", output);
}

void restVentilAufZu() {
  boolean success = sendOpenCloseSignalToWasserstop();

  int responseCode = success ? 200 : 503;
  httpRestServer.send(responseCode, "text/html", "");
}

void restVentilAuf() {
 int responseCode = 200;
 if (isKugelventilGeschlossen()) {
  restVentilAufZu();
 } else {
  // TODO: Check if motor is already closing the valve, if so, schedule valve for re-opening
  httpRestServer.send(424, "text/html", "Valve already open"); // Failed Dependency
 }
}

void restVentilZu() {
 int responseCode = 200;
 if (isKugelventilOffen()) {
  restVentilAufZu();
 } else {
  // TODO: Check if motor is already opening the valve, if so, schedule valve for re-closing
  httpRestServer.send(424, "text/html", "Valve already closed"); // Failed Dependency
 }
}


void startRestService() {
  debugln("Starting REST server");
  //httpRestServer = new ESP8266WebServer(WiFi.localIP(), REST_SERVER_HTTP_PORT));
  httpRestServer.on("/", HTTP_GET, restGetRoot);
  httpRestServer.on("/all-data", HTTP_GET, restAllData);
  httpRestServer.on("/ventil-auf-zu", HTTP_POST, restVentilAufZu);
  httpRestServer.on("/ventil-auf", HTTP_POST, restVentilAuf);
  httpRestServer.on("/ventil-zu", HTTP_POST, restVentilZu);


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
  Serial.println("\n Starting");

  // init serial connection to Wasserstop
  swSer.begin(SERIAL_BAUD_RATE,
              SERIAL_UART_MODE,
              SERIAL_RX_PIN,
              SERIAL_TX_PIN,
              /*invert=*/ false,
              /*bufCapacity=*/ SERIAL_RX_BUF_SIZE);

  // Initialize pins
  pinMode(WIFI_RESET_OPERATE_MODE, INPUT);

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
      Serial.println("Request data from Wasserstop...");
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
