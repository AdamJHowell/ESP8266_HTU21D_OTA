/**
 * This sketch will use a HTU21D (SHT20/SHT21 compatible) sensor to measure temperature and humidity.
 * The HTU21D uses address 0x40.
 * My topic formats are:
 *    <location>/<device>/<device reading>
 *    <location>/<device>/<sensor type>/<sensor reading>
 * @copyright   Copyright © 2022 Adam Howell
 * @licence     The MIT License (MIT)
 */
#ifdef ESP8266
// These headers are installed when the ESP8266 is installed in board manager.
#include "ESP8266WiFi.h"	// ESP8266 Wifi support.  https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WiFi
#include <ESP8266mDNS.h>	// OTA - Multicast DNS for the ESP8266.
#elif ESP32
// These headers are installed when the ESP32 is installed in board manager.
#include "WiFi.h"				// ESP32 Wifi support.  https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi/src/WiFi.h
#include <ESPmDNS.h>			// OTA - Multicast DNS for the ESP32.
#else
#include "WiFi.h"				// Arduino Wifi support.  This header is part of the standard library.  https://www.arduino.cc/en/Reference/WiFi
#endif
#include <Wire.h>				// This header is part of the standard library.  https://www.arduino.cc/en/reference/wire
#include <PubSubClient.h>	// PubSub is the MQTT API.  Author: Nick O'Leary  https://github.com/knolleary/pubsubclient
#include <ArduinoJson.h>	// The JSON parsing library used.  Author: Benoît Blanchon  https://arduinojson.org/
#include <ArduinoOTA.h>		// OTA - The Arduino OTA library.  Specific version of this are installed along with specific boards in board manager.
#include "privateInfo.h"	// I use this file to hide my network information from random people browsing my GitHub repo.
#include "SHT2x.h"			// Rob Tillaart's excellent SHT20-series library: https://github.com/RobTillaart/SHT2x


/*
 * Declare network variables.
 * Adjust the commented-out variables to match your network and broker settings.
 * The commented-out variables are stored in "privateInfo.h", which I do not upload to GitHub.
 */
// const char * wifiSsidArray[4] = { "Network1", "Network2", "Network3", "Syrinx" };			// Typically kept in "privateInfo.h".
// const char * wifiPassArray[4] = { "Password1", "Password2", "Password3", "By-Tor" };		// Typically kept in "privateInfo.h".
// const char * mqttBrokerArray[4] = { "Broker1", "Broker2", "Broker3", "192.168.0.2" };		// Typically kept in "privateInfo.h".
// int const mqttPortArray[4] = { 1883, 1883, 1883, 2112 };												// Typically kept in "privateInfo.h".

const char *notes = "adamsDesk/8266/htu21d ESP8266 with HTU21D and OTA";
const char *sketchName = "ESP8266_HTU21D_OTA";						// The sketch name to report.
const char *hostname = "8266-htu21";									// The network hostname for this device.  Used by OTA and general networking.
const char *mqttCommandTopic = "adamsDesk/8266/command";			// The topic used to subscribe to update commands.  Commands: publishTelemetry, changeTelemetryInterval, publishStatus.
const char *sketchTopic = "adamsDesk/8266/sketch";					// The topic used to publish the sketch name.
const char *macTopic = "adamsDesk/8266/mac";							// The topic used to publish the MAC address.
const char *ipTopic = "adamsDesk/8266/ip";							// The topic used to publish the IP address.
const char *rssiTopic = "adamsDesk/8266/rssi";						// The topic used to publish the WiFi Received Signal Strength Indicator.
const char *publishCountTopic = "adamsDesk/8266/publishCount"; // The topic used to publish the loop count.
const char *notesTopic = "adamsDesk/8266/notes";					// The topic used to publish notes relevant to this project.
const char *tempCTopic = "adamsDesk/8266/sht40/tempC";			// The topic used to publish the temperature in Celcius.
const char *tempFTopic = "adamsDesk/8266/sht40/tempF";			// The topic used to publish the temperature in Fahrenheit.
const char *humidityTopic = "adamsDesk/8266/sht40/humidity";	// The topic used to publish the humidity.
const char *mqttTopic = "espWeather";									// The topic used to publish a single JSON message containing all data.
const char *mqttStatsTopic = "espStats";								// The topic this device will publish to upon connection to the broker.
const int BUFFER_SIZE = 512;												// The maximum packet size MQTT should transfer.
const int LED_PIN = 2;														// The blue LED on the Freenove devkit.
unsigned int consecutiveBadTemp = 0;									// Holds the number of consecutive invalid temperature readings.
unsigned int consecutiveBadHumidity = 0;								// Holds the number of consecutive invalid humidity readings.
unsigned int publishDelay = 60000;										// How long to delay between publishes.
unsigned int networkIndex = 2112;										// An unsigned integer (0-65535) to hold the correct index for the network arrays: wifiSsidList[], wifiPassArray[], mqttBrokerArray[], and mqttPortArray[].
unsigned long lastPublishTime = 0;										// This is used to determine the time since last MQTT publish.
unsigned long sensorPollDelay = 10000;									// This is the delay between polls of the sensor.  This should be greater than 100 milliseconds.
unsigned long lastPollTime = 0;											// This is used to determine the time since last sensor poll.
unsigned long lastPublish = 0;											// In milliseconds, this sets a limit at 49.7 days of time.
unsigned long publishCount = 0;											// A count of how many publishes have taken place.
unsigned long wifiConnectionTimeout = 10000;							// The maximum amount of time to wait for a WiFi connection before trying a different SSID.
unsigned long mqttReconnectDelay = 5000;								// How long to delay between MQTT broker connection attempts.
float temperature;															// A global to hold the temperature reading.
float humidity;																// A global to hold the humidity reading.
char macAddress[18];															// The MAC address of the WiFi NIC.
char ipAddress[16];															// The IP address given to the device.
char *activeWifiSsid;
char *activeMqttBroker;
int activeMqttPort;

// Create class objects.
WiFiClient espClient;					  // Network client.
PubSubClient mqttClient( espClient ); // MQTT client.
SHT2x htu21d;								  // Environmental sensor.


void onReceiveCallback( char *topic, byte *payload, unsigned int length )
{
	char str[length + 1];
	Serial.printf( "Message arrived [%s] ", topic );
	unsigned int i = 0;
	for( i = 0; i < length; i++ )
	{
		Serial.print( ( char )payload[i] );
		str[i] = ( char )payload[i];
	}
	Serial.println();
	// Add the null terminator.
	str[i] = 0;
	StaticJsonDocument<256> doc;
	deserializeJson( doc, str );

	// The command can be: publishTelemetry, changeTelemetryInterval, or publishStatus.
	const char *command = doc["command"];
	if( strcmp( command, "publishTelemetry" ) == 0 )
	{
		Serial.println( "Reading and publishing sensor values." );
		// Poll the sensor.
		readTelemetry();
		// Publish the sensor readings.
		publishTelemetry();
		Serial.println( "Readings have been published." );
	}
	else if( strcmp( command, "changeTelemetryInterval" ) == 0 )
	{
		Serial.println( "Changing the publish interval." );
		unsigned long tempValue = doc["value"];
		// Only update the value if it is greater than 4 seconds.  This prevents a seconds vs. milliseconds mixup.
		if( tempValue > 4000 )
			publishDelay = tempValue;
		Serial.print( "MQTT publish interval has been updated to " );
		Serial.println( publishDelay );
		lastPublishTime = 0;
	}
	else if( strcmp( command, "publishStatus" ) == 0 )
	{
		Serial.println( "publishStatus is not yet implemented." );
	}
	else
	{
		Serial.print( "Unknown command: " );
		Serial.println( command );
	}
} // End of onReceiveCallback() function.


/**
 * The setup() function runs once when the device is booted, and then loop() takes over.
 */
void setup()
{
	// Start the Serial communication to send messages to the computer.
	Serial.begin( 115200 );
	delay( 1000 );

	pinMode( LED_PIN, OUTPUT );
	digitalWrite( LED_PIN, LOW );

	if( !Serial )
		delay( 1000 );

	Serial.println( "\nSetup is initiating..." );
	Wire.begin();

	Serial.println( __FILE__ );

	// Set up the sensor.
	setupHTU21D();

	// Set the ipAddress char array to a default value.
	snprintf( ipAddress, 16, "127.0.0.1" );

	// Get the MAC address and store it in macAddress.
	snprintf( macAddress, 18, "%s", WiFi.macAddress().c_str() );

	Serial.println( "Connecting WiFi..." );
	wifiMultiConnect();

	// The networkIndex variable is initialized to 2112.  If it is still 2112 at this point, then WiFi failed to connect.
	if( networkIndex != 2112 )
	{
		const char *mqttBroker = mqttBrokerArray[networkIndex];
		const int mqttPort = mqttPortArray[networkIndex];
		// Set the MQTT client parameters.
		mqttClient.setServer( mqttBroker, mqttPort );
		// Assign the onReceiveCallback() function to handle MQTT callbacks.
		mqttClient.setCallback( onReceiveCallback );
		Serial.printf( "Using MQTT broker: %s\n", mqttBroker );
		Serial.printf( "Using MQTT port: %d\n", mqttPort );
	}

	/*
	 * This section contains the Over The Air (OTA) update code.
	 * An excellent OTA guide can be found here:
	 * https://randomnerdtutorials.com/esp8266-ota-updates-with-arduino-ide-over-the-air/
	 * The setPort(), setHostname(), setPassword() function are all optional.
	 */
	// Port defaults to 8266
	// ArduinoOTA.setPort( 8266 );

	// Hostname defaults to esp8266-[ChipID]
	ArduinoOTA.setHostname( hostname );
	Serial.printf( "Using hostname '%s'\n", hostname );

	// No authentication by default.  Usage:
	// ArduinoOTA.setPassword( ( const char * )"abc123" );

	ArduinoOTA.onStart( []() { Serial.println( "Starting OTA communication." ); } );
	ArduinoOTA.onEnd( []() { Serial.println( "\nTerminating OTA communication." ); } );
	ArduinoOTA.onProgress( []( unsigned int progress, unsigned int total ) { Serial.printf( "OTA progress: %u%%\r", ( progress / ( total / 100 ) ) ); } );
	ArduinoOTA.onError( []( ota_error_t error ) {
		Serial.printf( "Error[%u]: ", error );
		if( error == OTA_AUTH_ERROR ) Serial.println( "OTA authentication failed!" );
		else if( error == OTA_BEGIN_ERROR ) Serial.println( "OTA transmission failed to initiate properly!" );
		else if( error == OTA_CONNECT_ERROR ) Serial.println( "OTA connection failed!" );
		else if( error == OTA_RECEIVE_ERROR ) Serial.println( "OTA client was unable to properly receive data!" );
		else if( error == OTA_END_ERROR ) Serial.println( "OTA transmission failed to terminate properly!" ); } );
	ArduinoOTA.begin();
	Serial.println( "OTA is configured and ready." );
	Serial.printf( "IP address: %s\n", ipAddress );

	// Connect to the MQTT broker.
	mqttMultiConnect( 10 );

	// Take the first temperature and humidity readings.
	readTelemetry();

	// Print the first readings to the terminal.
	printTelemetry();

	// Publish the first readings to the MQTT broker.
	publishTelemetry();

	Serial.println( "Setup has completed.\n" );
} // End of setup() function.


void setupHTU21D()
{
	Serial.println( "Initializing the HTU21D sensor." );
	Serial.print( "SHT2x_LIB_VERSION: \t" );
	Serial.println( SHT2x_LIB_VERSION );

	htu21d.begin();

	uint8_t stat = htu21d.getStatus();
	Serial.printf( "HTU21D status code: %x\n", stat );

	Serial.println( "The HTU21D has been configured." );
} // End of setupHTU21D function.


/*
 * wifiMultiConnect() will iterate through 'wifiSsidArray[]', attempting to connect with the password stored at the same index in 'wifiPassArray[]'.
 *
 */
void wifiMultiConnect()
{
	digitalWrite( LED_PIN, HIGH ); // Turn the LED off to show a connection is being made.

	Serial.println( "\nEntering wifiMultiConnect()" );
	for( size_t networkArrayIndex = 0; networkArrayIndex < sizeof( wifiSsidArray ); networkArrayIndex++ )
	{
		// Get the details for this connection attempt.
		const char *wifiSsid = wifiSsidArray[networkArrayIndex];
		const char *wifiPassword = wifiPassArray[networkArrayIndex];

		// Announce the WiFi parameters for this connection attempt.
		Serial.print( "Attempting to connect to to SSID \"" );
		Serial.print( wifiSsid );
		Serial.println( "\"" );

		// Don't even try to connect if the SSID cannot be found.
		if( checkForSSID( wifiSsid ) )
		{
			// Attempt to connect to this WiFi network.
			Serial.printf( "Wi-Fi mode set to WIFI_STA %s\n", WiFi.mode( WIFI_STA ) ? "" : "Failed!" );
			WiFi.begin( wifiSsid, wifiPassword );

			unsigned long wifiConnectionStartTime = millis();
			// Wait up to 10 seconds for a connection.
			Serial.print( "Waiting up to " );
			Serial.print( wifiConnectionTimeout / 1000 );
			Serial.print( " seconds for a connection" );
			/*
			WiFi.status() return values:
			WL_IDLE_STATUS      = 0,
			WL_NO_SSID_AVAIL    = 1,
			WL_SCAN_COMPLETED   = 2,
			WL_CONNECTED        = 3,
			WL_CONNECT_FAILED   = 4,
			WL_CONNECTION_LOST  = 5,
			WL_WRONG_PASSWORD   = 6,
			WL_DISCONNECTED     = 7
			*/
			while( WiFi.status() != WL_CONNECTED && ( millis() - wifiConnectionStartTime < wifiConnectionTimeout ) )
			{
				Serial.print( "." );
				delay( 1000 );
			}
			Serial.println( "" );

			if( WiFi.status() == WL_CONNECTED )
			{
				digitalWrite( LED_PIN, LOW ); // Turn the LED on to show the connection was successfull.
				Serial.print( "IP address: " );
				snprintf( ipAddress, 16, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
				Serial.println( ipAddress );
				networkIndex = networkArrayIndex;
				// Print that WiFi has connected.
				Serial.println( "\nWiFi connection established!" );
				return;
			}
			else
				Serial.println( "Unable to connect to WiFi!" );
		}
		else
			Serial.println( "That network was not found!" );
	}
	Serial.println( "Exiting wifiMultiConnect()\n" );
} // End of wifiMultiConnect() function.


/*
 * checkForSSID() is used by wifiMultiConnect() to avoid attempting to connect to SSIDs which are not in range.
 * Returns 1 if 'ssidName' can be found.
 * Returns 0 if 'ssidName' cannot be found.
 */
int checkForSSID( const char *ssidName )
{
	byte networkCount = WiFi.scanNetworks();
	if( networkCount == 0 )
		Serial.println( "No WiFi SSIDs are in range!" );
	else
	{
		Serial.print( networkCount );
		Serial.println( " networks found." );
		for( int i = 0; i < networkCount; ++i )
		{
			// Check to see if this SSID matches the parameter.
			if( strcmp( ssidName, WiFi.SSID( i ).c_str() ) == 0 )
				return 1;
			// Alternately, the String compareTo() function can be used like this: if( WiFi.SSID( i ).compareTo( ssidName ) == 0 )
		}
	}
	Serial.print( "SSID '" );
	Serial.print( ssidName );
	Serial.println( "' was not found!" );
	return 0;
} // End of checkForSSID() function.


/*
 * mqttMultiConnect() will:
 * 1. Check the WiFi connection, and reconnect WiFi as needed.
 * 2. Attempt to connect the MQTT client designated in 'mqttBrokerArray[networkIndex]' up to 'maxAttempts' number of times.
 * 3. Subscribe to the topic defined in 'mqttCommandTopic'.
 * If the broker connection cannot be made, an error will be printed to the serial port.
 */
bool mqttMultiConnect( int maxAttempts )
{
	Serial.println( "Function mqttMultiConnect() has initiated.\n" );
	if( WiFi.status() != WL_CONNECTED )
		wifiMultiConnect();

	digitalWrite( LED_PIN, HIGH ); // Turn the LED off to show a connection is being made.

	/*
	 * The networkIndex variable is initialized to 2112.
	 * If it is still 2112 at this point, then WiFi failed to connect.
	 * This is only needed to display the name and port of the broker being used.
	 */
	if( networkIndex != 2112 )
	{
		Serial.print( "Attempting to connect to the MQTT broker at '" );
		Serial.print( mqttBrokerArray[networkIndex] );
		Serial.print( ":" );
		Serial.print( mqttPortArray[networkIndex] );
		Serial.print( "' up to " );
		Serial.print( maxAttempts );
		Serial.println( " times." );
	}
	else
	{
		Serial.print( "Attempting to connect to the MQTT broker up to " );
		Serial.print( maxAttempts );
		Serial.println( " times." );
	}


	int attemptNumber = 0;
	// Loop until MQTT has connected.
	while( !mqttClient.connected() && attemptNumber < maxAttempts )
	{
		// Put the macAddress and randNum into clientId.
		char clientId[22];
		snprintf( clientId, 22, "%s-%03ld", macAddress, random( 999 ) );
		// Connect to the broker using the MAC address for a clientID.  This guarantees that the clientID is unique.
		Serial.printf( "Connecting with client ID '%s'.\n", clientId );
		Serial.printf( "Attempt # %d....", ( attemptNumber + 1 ) );
		if( mqttClient.connect( clientId ) )
		{
			digitalWrite( LED_PIN, LOW ); // Turn the LED on to show the connection was successfull.
			Serial.println( " connected." );
			if( !mqttClient.setBufferSize( BUFFER_SIZE ) )
			{
				Serial.printf( "Unable to create a buffer %d bytes long!\n", BUFFER_SIZE );
				Serial.println( "Restarting the device!" );
				ESP.restart();
			}
			publishStats();
			// Subscribe to the command topic.
			if( mqttClient.subscribe( mqttCommandTopic ) )
				Serial.printf( "Succsefully subscribed to topic '%s'.\n", mqttCommandTopic );
			else
				Serial.printf( "Failed to subscribe to topic '%s'!\n", mqttCommandTopic );
		}
		else
		{
			int mqttState = mqttClient.state();
			/*
				Possible values for client.state():
				#define MQTT_CONNECTION_TIMEOUT     -4		// Note: This also comes up when the clientID is already in use.
				#define MQTT_CONNECTION_LOST        -3
				#define MQTT_CONNECT_FAILED         -2
				#define MQTT_DISCONNECTED           -1
				#define MQTT_CONNECTED               0
				#define MQTT_CONNECT_BAD_PROTOCOL    1
				#define MQTT_CONNECT_BAD_CLIENT_ID   2
				#define MQTT_CONNECT_UNAVAILABLE     3
				#define MQTT_CONNECT_BAD_CREDENTIALS 4
				#define MQTT_CONNECT_UNAUTHORIZED    5
			*/
			Serial.printf( " failed!  Return code: %d", mqttState );
			if( mqttState == -4 )
				Serial.println( " - MQTT_CONNECTION_TIMEOUT" );
			else if( mqttState == 2 )
				Serial.println( " - MQTT_CONNECT_BAD_CLIENT_ID" );
			else
				Serial.println( "" );

			Serial.printf( "Trying again in %lu seconds.\n\n", mqttReconnectDelay / 1000 );
			delay( mqttReconnectDelay );
		}
		attemptNumber++;
	}

	if( !mqttClient.connected() )
	{
		Serial.println( "Unable to connect to the MQTT broker!" );
		return false;
	}

	Serial.println( "Function mqttMultiConnect() has completed.\n" );
	return true;
} // End of mqttMultiConnect() function.


/*
 * readTelemetry() will:
 * 1. Read from all available sensors.
 * 2. Store legitimate values in global variables.
 * 3. Increment a counter if any value is invalid.
 */
void readTelemetry()
{
	// Read fresh data from the sensor.
	uint32_t start = micros();
	htu21d.read();
	uint32_t stop = micros();

	float temporaryTemperature = htu21d.getTemperature();
	float temporaryHumidity = htu21d.getHumidity();

	// Print the sensor data to the serial port.
	Serial.printf( "Measurement took %u milliseconds.\n", ( stop - start ) / 1000 );

	// Define the valid temperature range (in Celsius) for this sensor.
	if( temporaryTemperature > -30 || temporaryTemperature < 90 )
	{
		temperature = temporaryTemperature;
		// Clear the consecutive bad count.
		consecutiveBadTemp = 0;
	}
	else
		consecutiveBadTemp++;

	// Define the valid humidity range for this sensor.
	if( temporaryHumidity >= 0 || temporaryHumidity <= 100 )
	{
		humidity = temporaryHumidity;
		// Clear the consecutive bad count.
		consecutiveBadHumidity = 0;
	}
	else
		consecutiveBadHumidity++;

	if( consecutiveBadTemp > 2 || consecutiveBadHumidity > 2 )
	{
		Serial.println( "\n\n\n\n" );
		Serial.printf( "%u consecutive bad temperature readings!\n", consecutiveBadTemp );
		Serial.printf( "%u consecutive bad humidity readings!\n", consecutiveBadHumidity );
		Serial.println( "Resetting the device!\n\n\n" );
		ESP.restart(); // Reset the device.
	}
} // End of readTelemetry() function.


/*
 * printTelemetry() will print the sensor and device data to the serial port.
 */
void printTelemetry()
{
	// Print the signal strength:
	long rssi = WiFi.RSSI();
	Serial.printf( "Temperature: %.2f C\n", temperature );
	float tempF = ( temperature * 9 / 5 ) + 32;
	Serial.printf( "Temperature: %.2f F\n", tempF );
	Serial.printf( "Humidity: %.2f %%\n", humidity );
	Serial.printf( "WiFi RSSI: %ld\n", rssi );
} // End of printTelemetry() function.


/*
 * publishStats() is called by mqttConnect() every time the device (re)connects to the broker, and every publishDelay milliseconds thereafter.
 * It is also called by the callback when the "publishStats" command is received.
 */
void publishStats()
{
	// Read the RSSI.
	long rssi = WiFi.RSSI();
	// This needs to be large enough to hold the String used by snprintf().
	char mqttStatsString[256];
	// Create a JSON Document on the stack.
	StaticJsonDocument<BUFFER_SIZE> doc;
	// Add data: sketchName, macAddress, ipAddress, rssi, publishCount
	doc["sketch"] = sketchName;
	doc["mac"] = macAddress;
	doc["ip"] = ipAddress;
	doc["rssi"] = rssi;
	doc["publishCount"] = publishCount;

	// Serialize the JSON into mqttStatsString, with indentation and line breaks.
	serializeJsonPretty( doc, mqttStatsString );

	Serial.printf( "Publishing stats to the '%s' topic.", mqttStatsTopic );

	// snprintf( mqttStatsString, 256, "{\n\t\"sketch\": \"%s\",\n\t\"mac\": \"%s\",\n\t\"ip\": \"%s\",\n\t\"rssi\": \"%ld\",\n\t\"publishCount\": \"%ld\"\n}", sketchName, macAddress, ipAddress, rssi, publishCount );
	if( mqttClient.connected() )
	{
		if( mqttClient.connected() && mqttClient.publish( mqttStatsTopic, mqttStatsString ) )
			Serial.printf( "Published to this broker and port: %s:%d, and to this topic '%s':\n%s\n", mqttBrokerArray[networkIndex], mqttPortArray[networkIndex], mqttStatsTopic, mqttStatsString );
		else
			Serial.println( "\n\nPublish failed!\n\n" );
	}
} // End of publishStats() function.


/*
 * publishTelemetry() will publish the sensor and device data over MQTT.
 */
void publishTelemetry()
{
	long rssi = WiFi.RSSI();
	float tempF = ( temperature * 9 / 5 ) + 32;
	char mqttString[BUFFER_SIZE]; // A String to hold the JSON.

	// Create a JSON Document on the stack.
	StaticJsonDocument<BUFFER_SIZE> doc;
	// Add data: sketchName, macAddress, ipAddress, temperature, tempF, humidity, rssi, publishCount, notes
	doc["sketch"] = sketchName;
	doc["mac"] = macAddress;
	doc["ip"] = ipAddress;
	doc["tempC"] = temperature;
	doc["tempF"] = tempF;
	doc["humidity"] = humidity;
	doc["rssi"] = rssi;
	doc["publishCount"] = publishCount;
	doc["notes"] = notes;

	// Serialize the JSON into mqttString, with indentation and line breaks.
	serializeJsonPretty( doc, mqttString );

	// Publish the JSON to the MQTT broker.
	bool success = mqttClient.publish( mqttTopic, mqttString, false );
	if( success )
	{
		Serial.println( "Succsefully published to:" );
		char buffer[20];
		// New format: <location>/<device>/<sensor>/<metric>
		if( mqttClient.publish( sketchTopic, sketchName, false ) )
			Serial.printf( "\t%s\n", sketchTopic );
		if( mqttClient.publish( macTopic, macAddress, false ) )
			Serial.printf( "\t%s\n", macTopic );
		if( mqttClient.publish( ipTopic, ipAddress, false ) )
			Serial.printf( "\t%s\n", ipTopic );
		if( mqttClient.publish( rssiTopic, ltoa( rssi, buffer, 10 ), false ) )
			Serial.printf( "\t%s\n", rssiTopic );
		if( mqttClient.publish( publishCountTopic, ltoa( publishCount, buffer, 10 ), false ) )
			Serial.printf( "\t%s\n", publishCountTopic );
		if( mqttClient.publish( notesTopic, notes, false ) )
			Serial.printf( "\t%s\n", notesTopic );
		// Convert the temperature in Celsius from a float to a char array.
		dtostrf( temperature, 1, 3, buffer );
		if( mqttClient.publish( tempCTopic, buffer, false ) )
			Serial.printf( "\t%s\n", tempCTopic );
		dtostrf( ( tempF ), 1, 3, buffer );
		if( mqttClient.publish( tempFTopic, buffer, false ) )
			Serial.printf( "\t%s\n", tempFTopic );
		dtostrf( ( humidity ), 1, 3, buffer );
		if( mqttClient.publish( humidityTopic, buffer, false ) )
			Serial.printf( "\t%s\n", humidityTopic );

		Serial.printf( "Successfully published to '%s', this JSON:\n", mqttTopic );
	}
	else
		Serial.printf( "Failed to publish to '%s', this JSON:\n", mqttTopic );
	// Print the JSON to the Serial port.
	Serial.println( mqttString );
	lastPublishTime = millis();
} // End of publishTelemetry() function.


void loop()
{
	// Check the mqttClient connection state.
	if( !mqttClient.connected() )
		mqttMultiConnect( 10 );
	// The loop() function facilitates the receiving of messages and maintains the connection to the broker.
	mqttClient.loop();

	ArduinoOTA.handle();

	unsigned long time = millis();
	if( lastPollTime == 0 || ( ( time > sensorPollDelay ) && ( time - sensorPollDelay ) > lastPollTime ) )
	{
		readTelemetry();
		printTelemetry();
		lastPollTime = millis();
		Serial.printf( "Next telemetry poll in %lu seconds\n\n", sensorPollDelay / 1000 );
	}

	time = millis();
	if( ( time > publishDelay ) && ( time - publishDelay ) > lastPublish )
	{
		publishCount++;
		Serial.println();
		Serial.println( sketchName );
		Serial.println( __FILE__ );

		// Populate temperature and humidity variables with fresh data.
		readTelemetry();

		// Print the readings to the terminal.
		printTelemetry();

		// Publish the readings to the MQTT broker.
		publishTelemetry();

		Serial.printf( "publishCount: %lu\n", publishCount );

		lastPublish = millis();
		Serial.printf( "Next MQTT publish in %u seconds.\n\n", publishDelay / 1000 );
	}
} // End of loop() function.
