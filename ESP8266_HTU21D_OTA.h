//
// Created by Adam Howell on 2022-11-29.
//

#ifndef ESP8266_HTU21D_OTA_ESP8266_HTU21D_OTA_H
#define ESP8266_HTU21D_OTA_ESP8266_HTU21D_OTA_H


#ifdef ESP8266
// These headers are installed when the ESP8266 is installed in board manager.
#include "ESP8266WiFi.h"  // ESP8266 WiFi support.  https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WiFi
#include <ESP8266mDNS.h>  // OTA - mDNSResponder (Multicast DNS) for the ESP8266 family.
#elif ESP32
// These headers are installed when the ESP32 is installed in board manager.
#include "WiFi.h"     // ESP32 WiFi support.  https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi/src/WiFi.h
#include <ESPmDNS.h>  // OTA - Multicast DNS for the ESP32.
#else
// This block is meant for other devices, like the Wi-Fi capable Uno.
#include "WiFi.h"  // Arduino Wifi support.  This header is part of the standard library.  https://www.arduino.cc/en/Reference/WiFi

#endif

#include <Wire.h>          // This header is part of the standard library.  https://www.arduino.cc/en/reference/wire
#include <PubSubClient.h>  // PubSub is the MQTT API.  Author: Nick O'Leary  https://github.com/knolleary/pubsubclient
#include <ArduinoJson.h>   // The JSON parsing library used.  Author: Benoît Blanchon  https://arduinojson.org/
#include <ArduinoOTA.h>    // OTA - The Arduino OTA library.  Specific version of this are installed along with specific boards in board manager.
#include <SHT2x.h>         // Rob Tillaart's SHT20-series library: https://github.com/RobTillaart/SHT2x
#include "privateInfo.h"   // I use this file to hide my network information from random people browsing my GitHub repo.


/**
 * @brief Declare network variables.
 * Adjust the commented-out variables to match your network and broker settings.
 * The commented-out variables are stored in "privateInfo.h", which I do not upload to GitHub.
 */
// const char * wifiSsidArray[4] = { "Network1", "Network2", "Network3", "Syrinx" };			// Typically kept in "privateInfo.h".
// const char * wifiPassArray[4] = { "Password1", "Password2", "Password3", "By-Tor" };		// Typically kept in "privateInfo.h".
// const char * mqttBrokerArray[4] = { "Broker1", "Broker2", "Broker3", "192.168.0.2" };		// Typically kept in "privateInfo.h".
// int const mqttPortArray[4] = { 1883, 1883, 1883, 2112 };												// Typically kept in "privateInfo.h".

const char *notes = "Adam's ESP8266 with HTU21D and OTA";       // These notes are published via MQTT and printed to the serial port.
const char *hostname = "adam-8266-htu21";                       // The network hostname for this device.  Used by OTA and general networking.
const char *mqttCommandTopic = "AdamsDesk/8266/command";        // The topic used to subscribe to update commands.  Commands: publishTelemetry, changeTelemetryInterval, publishStatus.
const char *sketchTopic = "AdamsDesk/8266/sketch";              // The topic used to publish the sketch name.
const char *macTopic = "AdamsDesk/8266/mac";                    // The topic used to publish the MAC address.
const char *ipTopic = "AdamsDesk/8266/ip";                      // The topic used to publish the IP address.
const char *rssiTopic = "AdamsDesk/8266/rssi";                  // The topic used to publish the WiFi Received Signal Strength Indicator.
const char *publishCountTopic = "AdamsDesk/8266/publishCount";  // The topic used to publish the loop count.
const char *notesTopic = "AdamsDesk/8266/notes";                // The topic used to publish notes relevant to this project.
const char *tempCTopic = "AdamsDesk/8266/HTU21D/tempC";         // The topic used to publish the temperature in Celsius.
const char *tempFTopic = "AdamsDesk/8266/HTU21D/tempF";         // The topic used to publish the temperature in Fahrenheit.
const char *humidityTopic = "AdamsDesk/8266/HTU21D/humidity";   // The topic used to publish the humidity.
const char *mqttTopic = "espWeather";                           // The topic used to publish a single JSON message containing all data.
const char *mqttStatsTopic = "espStats";                        // The topic this device will publish to upon connection to the broker.
const int BUFFER_SIZE = 512;                                    // The maximum packet size MQTT should transfer.
const int MILLIS_IN_SEC = 1000;                                 // The number of milliseconds in one second.
const int LED_PIN = 2;                                          // The blue LED on the Freenove devkit.
unsigned int consecutiveBadTemp = 0;                            // Holds the current number of consecutive invalid temperature readings.
unsigned int consecutiveBadHumidity = 0;                        // Holds the current number of consecutive invalid humidity readings.
unsigned int networkIndex = 2112;                               // Holds the correct index for the network arrays: wifiSsidArray[], wifiPassArray[], mqttBrokerArray[], and mqttPortArray[].
unsigned long publishInterval = 60 * MILLIS_IN_SEC;             // The delay in milliseconds between MQTT publishes.  This prevents "flooding" the broker.
unsigned long telemetryInterval = 10 * MILLIS_IN_SEC;           // The delay in milliseconds between polls of the sensor.  This should be greater than 100 milliseconds.
unsigned long ledBlinkInterval = 200;                           // The interval between telemetry processing times.
unsigned long lastPublishTime = 0;                              // The time of the last MQTT publish.
unsigned long lastPollTime = 0;                                 // The time of the last sensor poll.
unsigned long lastLedBlinkTime = 0;                             // The time of the last telemetry process.
unsigned long publishCount = 0;                                 // A count of how many publishes have taken place.
unsigned long wifiConnectionTimeout = 10 * MILLIS_IN_SEC;       // The maximum amount of time in milliseconds to wait for a WiFi connection before trying a different SSID.
unsigned long mqttReconnectDelay = 5 * MILLIS_IN_SEC;           // When mqttMultiConnect is set to try multiple times, this is how long to delay between each attempt.
unsigned int mqttReconnectCooldown = 20000;                     // Set the minimum time between calls to mqttMultiConnect() to 20 seconds.
unsigned long lastMqttConnectionTime = 0;                       // The last time a MQTT connection was attempted.
float tempC;                                                    // A global to hold the temperature in Celsius.
float tempF;                                                    // A global to hold the temperature in Fahrenheit.
float humidity;                                                 // A global to hold the relative humidity reading.
long rssi;                                                      // A global to hold the Received Signal Strength Indicator.
char macAddress[18];                                            // The MAC address of the WiFi NIC.
char ipAddress[16];                                             // The IP address given to the device.


// Create class objects.
WiFiClient espClient;                  // Network client.
PubSubClient mqttClient( espClient );  // MQTT client.
SHT2x htu21d;                          // Environmental sensor.


void onReceiveCallback( char *topic, byte *payload, unsigned int length );
void configureOTA();
void setupHTU21D();
void wifiMultiConnect();
int checkForSSID( const char *ssidName );
void mqttMultiConnect( int maxAttempts );
void readTelemetry();
void printTelemetry();
void lookupWifiCode( int code, char *buffer );
void lookupMQTTCode( int code, char *buffer );
void publishStats();
void publishTelemetry();
void toggleLED();

#endif //ESP8266_HTU21D_OTA_ESP8266_HTU21D_OTA_H
