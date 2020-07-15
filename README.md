# esp8266.zewa-wasserstop-gateway

## What's this project about?
This project aims to integrate the [Judo ZEWA Wasserstop](https://judo.eu/produkt/judo-zewa-wasserstop-jzw-%C2%BE-1%C2%BC/) in smart home systems. Therefore, an ESP8266 based WeMos D1 mini serves as a gateway. Using a RS-323 to TTL converter the ESP8266 is connected to the RS-323 serial interface of the Judo ZEWA Wasserstop on one side. On the other side, it connects to a WiFi and provides several HTTP endpoints. Using REST calls these endpoints can be used to read data and control the valve of the Wasserstop remotely.

## Disclaimer
### No Warranty / No Liability Statement
Although already stated in the appended [License file](./LICENSE) let's be crystal clear: Use this software and the described hardware setup *AT YOUR OWN RISK*. Be aware, if you screw up something in your setup you may destroy the Wasserstop unit or any other part involved.
### Personal Statement
* I'm *not* associated in any kind with the vendor of the Judo ZEWA Wasserstop system, i.e. [Judo Wasseraufbereitung GmbH](https://judo.eu)
* Please do *not* contact me or any other committer of this project for sales, consulting or advice like "Where can I buy XYZ?", "Can I purchase a cable from you?", etc.
### Collaboration and Ideas are welcome
After all the DON'Ts above let's also be clear: if you have specific questions, ideas or a feature request you are highly welcome to join and contribute.


## Usage

### Setup
Please refer to section [Hardware Setup](#hardware-setup) to learn how to wire up things. Section [Development](#development) will tell you how to compile the sources and upload it to your ESP8266 board.

### First Start
Upon first start you can see a new Wifi access point. Please connect to it. You'll automatically be redirected to a captive portal. Select the Wifi and enter the access code.

### Endpoints
TODO: describe all endpoints


## Hardware Setup
### Schematic
![Schematic for wiring](doc/wemos-d1-rs323-converter-wiring.png "Wiring of WeMos D1 mini and the RS-323 to TTL converter")
(Please be aware that the pin order on your RS-232 to TTL converter might be different, look at the pin labels)

### Used Parts
* RS-232 to TTL converter with MAX3232 chip that can deal with 3,3V TTL level (~2 Euro, [Aliexpress](https://de.aliexpress.com/item/33024283173.html))
* ESP-8266 (WeMos D1 mini) (~3 Euro, [Aliexpress](https://de.aliexpress.com/item/4000420770002.html))
* Do it yourself: RS-232 cable (or buy it from Judo for about 150 Euro)
  * Header connector: 3-pin MAS-CON PANCON, IDC, 3.96mm (CE156F24-3-D) (~5-10 Euro for end customers, you may be lucky on Ebay)
  * Basic old-style D-Sub 9 serial cable where you cut off one end (~4 Euro, [Amazon](https://www.amazon.de/ASSMANN-AK-610202-020-9-Pol-Stecker-Kupplung/dp/B007PO8BTQ/))

### Cable Wiring
The RS-232 to TTL converter probably has a *female* D-Sub 9 connector. Hence, you will need to plug in the *male* connector of the cable. Cut off the female connector and remove the outer part of the cable's insulation.

In the ZEWA Wasserstop unit the RS-323 pin header order is as follows:
![RS-232 pin header order in the Wasserstop unit](doc/rs323-header-pin-order.jpg)

When you've bought IDC header connectors (IDC = insulation-displacement contact) you may just use a screw driver to squeeze the litz wires into the header connector. Make sure the header connector is in the right orientation. Connect the litz wires behind the D-Sub connector pins as follows:
* Pin 2 -> RX
* Pin 3 -> TX
* Pin 5 -> GND

### Connect to Wasserstop Unit
According to [Beiblatt für RS-232-Kabel zum Judo Zewa Wasserstop](https://judo.eu/app/themes/judo-2019//data/8140001/manuals/1701936.pdf) you first need to remove the power plug. Then you can connect your DIY gateway with the RS-232 header pins of the Wasserstop unit. Eventually, insert the unit's power plug again.


## Development

### TODOs
For the beginning all open to-dos are listed here:

#### Miscellaneous
* New endpoint `/set-pushover-tokens` not properly implemented yet (revisit)
* Scan code for potential memory leaks
* Time for the timestamp is not adjusted between DST and normal time (inspiration can be found [here](https://forum.arduino.cc/index.php?topic=172044.msg1278536#msg1278536))
* Soft-limits: if none of the fixed settings suits your needs a soft limit allows for choosing different trigger limits.

#### Missing endpoints
* Add `/set-wasserzaehler-offset` POST endpoint to map a value of the internal volume counter to the physical water meter.
* Add `/wasserzaehler` GET endpoint to retrieve the value on the water meter (if offset has been set properly)
* If possible, add `/webapp` GET endpoint with a simple AJAX based single page web application that can be used on mobile device (with a home screen icon)

#### Security
Currently none of the calls is secured. Hence, every client in the WiFi may control the Wasserstop unit. A shared key that is transferred in an HTTP header *similar* to JWT tokens may be a viable solution.


### Used Tooling
* Arduino IDE, 1.8.13, https://www.arduino.cc/en/Main/Software

### Library dependencies
| Library                                                                      	| Version                    	| Source                                         	| Used for                                                                                 	|
|------------------------------------------------------------------------------	|----------------------------	|------------------------------------------------	|------------------------------------------------------------------------------------------	|
| SoftwareSerial                                                               	| 2.8.1                      	| https://github.com/plerup/espsoftwareserial/   	| Reading and writing serial data from/to the ZEWA Wasserstop                              	|
| WiFiManager                                                                  	| 0.15.0                     	| https://github.com/tzapu/WiFiManager           	| Convenient user interface to capture and store WiFi access data and manage auto connect. 	|
| Pushover                                                                     	| master  (date: 2020-07-12) 	| https://github.com/ArduinoHannover/Pushover/   	| Sending push notifications to mobile devices                                             	|
| NTPClient                                                                    	| 3.2.0                      	| https://github.com/arduino-libraries/NTPClient 	| Retrieving the actual time in order to add human readable timestamps to responses.       	|
| ArduinoJson                                                                  	| 6.15.2                     	| https://arduinojson.org/v6/                    	| Exchanging data via HTTP REST with a client                                              	|
| Arduino core for ESP8266 WiFi chip  (ESP8266WiFi, ESP8266WebServer, WiFiUdp) 	| 2.7.1                      	| https://github.com/esp8266/Arduino             	| Basic WiFi functionality, web server for serving HTTP REST, UDP for NTP requests.        	|

### Design considerations
* For the sake of simplicity, a passive HTTP REST interface has been chosen over an active MQTT.

## Links / References
* [Beiblatt für RS-232-Kabel zum Judo Zewa Wasserstop](https://judo.eu/app/themes/judo-2019//data/8140001/manuals/1701936.pdf)
* [Judo ZEWA Wasserstop Einbau- und Betriebsanleitung](https://judo.eu/app/downloads/files/de/8140001/manuals/Art-Nr_1701862_ZEWA-WASSERSTOP.pdf)
