/**
 * This sketch will use a HTU21D (SHT20/SHT21 compatible) sensor to measure temperature and humidity.
 * The HTU21D uses address 0x40.
 * One unconventional thing about this devkit is the LED is on when the pin is set to low, rather than high.
 * My topic formats are:
 *    <location>/<device>/<device reading>
 *    <location>/<device>/<sensor type>/<sensor reading>
 * @copyright   Copyright © 2022 Adam Howell
 * @license     The MIT License (MIT)
 */
#ifdef ESP8266
// These headers are installed when the ESP8266 is installed in board manager.
#include "ESP8266WiFi.h" // ESP8266 WiFi support.  https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WiFi
#include <ESP8266mDNS.h> // OTA - mDNSResponder (Multicast DNS) for the ESP8266 family.
#elif ESP32
// These headers are installed when the ESP32 is installed in board manager.
#include "WiFi.h"		// ESP32 WiFi support.  https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi/src/WiFi.h
#include <ESPmDNS.h> // OTA - Multicast DNS for the ESP32.
#else
#include "WiFi.h" // Arduino Wifi support.  This header is part of the standard library.  https://www.arduino.cc/en/Reference/WiFi
#endif
#include <Wire.h>			  // This header is part of the standard library.  https://www.arduino.cc/en/reference/wire
#include <PubSubClient.h> // PubSub is the MQTT API.  Author: Nick O'Leary  https://github.com/knolleary/pubsubclient
#include <ArduinoJson.h>  // The JSON parsing library used.  Author: Benoît Blanchon  https://arduinojson.org/
#include <ArduinoOTA.h>	  // OTA - The Arduino OTA library.  Specific version of this are installed along with specific boards in board manager.
#include "privateInfo.h"  // I use this file to hide my network information from random people browsing my GitHub repo.
#include "SHT2x.h"		  // Rob Tillaart's excellent SHT20-series library: https://github.com/RobTillaart/SHT2x


/**
 * @brief Declare network variables.
 * Adjust the commented-out variables to match your network and broker settings.
 * The commented-out variables are stored in "privateInfo.h", which I do not upload to GitHub.
 */
// const char * wifiSsidArray[4] = { "Network1", "Network2", "Network3", "Syrinx" };			// Typically kept in "privateInfo.h".
// const char * wifiPassArray[4] = { "Password1", "Password2", "Password3", "By-Tor" };		// Typically kept in "privateInfo.h".
// const char * mqttBrokerArray[4] = { "Broker1", "Broker2", "Broker3", "192.168.0.2" };		// Typically kept in "privateInfo.h".
// int const mqttPortArray[4] = { 1883, 1883, 1883, 2112 };												// Typically kept in "privateInfo.h".

const char *notes = "adamsDesk/8266/htu21d ESP8266 with HTU21D and OTA";
const char *hostname = "8266-htu21";											// The network hostname for this device.  Used by OTA and general networking.
const char *mqttCommandTopic = "adamsDesk/8266/command";					// The topic used to subscribe to update commands.  Commands: publishTelemetry, changeTelemetryInterval, publishStatus.
const char *sketchTopic = "adamsDesk/8266/sketch";							// The topic used to publish the sketch name.
const char *macTopic = "adamsDesk/8266/mac";									// The topic used to publish the MAC address.
const char *ipTopic = "adamsDesk/8266/ip";									// The topic used to publish the IP address.
const char *rssiTopic = "adamsDesk/8266/rssi";								// The topic used to publish the WiFi Received Signal Strength Indicator.
const char *publishCountTopic = "adamsDesk/8266/publishCount";			// The topic used to publish the loop count.
const char *notesTopic = "adamsDesk/8266/notes";							// The topic used to publish notes relevant to this project.
const char *tempCTopic = "adamsDesk/8266/sht40/tempC";					// The topic used to publish the temperature in Celsius.
const char *tempFTopic = "adamsDesk/8266/sht40/tempF";					// The topic used to publish the temperature in Fahrenheit.
const char *humidityTopic = "adamsDesk/8266/sht40/humidity";			// The topic used to publish the humidity.
const char *mqttTopic = "espWeather";											// The topic used to publish a single JSON message containing all data.
const char *mqttStatsTopic = "espStats";										// The topic this device will publish to upon connection to the broker.
const int BUFFER_SIZE = 512;														// The maximum packet size MQTT should transfer.
const int MILLIS_IN_SEC = 1000;													// The number of milliseconds in one second.
const int MCU_LED = 2;																// The blue LED on the Freenove devkit.
unsigned int consecutiveBadTemp = 0;											// Holds the current number of consecutive invalid temperature readings.
unsigned int consecutiveBadHumidity = 0;										// Holds the current number of consecutive invalid humidity readings.
unsigned int networkIndex = 2112;												// Holds the correct index for the network arrays: wifiSsidArray[], wifiPassArray[], mqttBrokerArray[], and mqttPortArray[].
unsigned long publishInterval = 60 * MILLIS_IN_SEC;						// The delay in milliseconds between MQTT publishes.  This prevents "flooding" the broker.
unsigned long telemetryInterval = 10 * MILLIS_IN_SEC;						// The delay in milliseconds between polls of the sensor.  This should be greater than 100 milliseconds.
unsigned long ledBlinkInterval = 200;								// The interval between telemetry processing times.
unsigned long lastPublishTime = 0;												// The time of the last MQTT publish.
unsigned long lastPollTime = 0;													// The time of the last sensor poll.
unsigned long lastLedBlinkTime = 0;									// The time of the last telemetry process.
unsigned long publishCount = 0;													// A count of how many publishes have taken place.
unsigned long wifiConnectionTimeout = 10 * MILLIS_IN_SEC;				// The maximum amount of time in milliseconds to wait for a WiFi connection before trying a different SSID.
unsigned long mqttReconnectDelay = 5 * MILLIS_IN_SEC;						// How long in milliseconds to delay between MQTT broker connection attempts.
float tempC;																			// A global to hold the temperature in Celsius.
float tempF;																			// A global to hold the temperature in Fahrenheit.
float humidity;																		// A global to hold the relative humidity reading.
long rssi;																				// A global to hold the Received Signal Strength Indicator.
char macAddress[18];																	// The MAC address of the WiFi NIC.
char ipAddress[16];																	// The IP address given to the device.


// Create class objects.
WiFiClient espClient;					  // Network client.
PubSubClient mqttClient( espClient ); // MQTT client.
SHT2x htu21d;								  // Environmental sensor.


/**
 * @brief onReceiveCallback() handles callback operations for MQTT.
 * This function will react to JSON messages containing the "command" property.
 * The "publishTelemetry" and "publishStatus" commands will immediately perform those operations.
 * The "changeTelemetryInterval" expects another property named "value" to have an integer value, which represents the new update interval in milliseconds.
 */
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
	StaticJsonDocument<BUFFER_SIZE> doc;
	deserializeJson( doc, str );

	// The command can be: publishTelemetry, publishStatus, changeTelemetryInterval, or changePublishInterval.
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
		// Only update the value if it is greater than 4 seconds.  This prevents a seconds vs. milliseconds mix-up.
		if( tempValue > 4 * MILLIS_IN_SEC )
			publishInterval = tempValue;
		Serial.print( "MQTT publish interval has been updated to " );
		Serial.println( publishInterval );
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
	delay( MILLIS_IN_SEC );

	pinMode( MCU_LED, OUTPUT );
	digitalWrite( MCU_LED, 0 );

	if( !Serial )
		delay( MILLIS_IN_SEC );

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

	configureOTA();

	Serial.printf( "IP address: %s\n", ipAddress );

	// Connect to the MQTT broker.
	mqttMultiConnect( 5 );

	// Take the first temperature and humidity readings.
	//	readTelemetry();

	// Print the first readings to the terminal.
	//	printTelemetry();

	// Publish the first readings to the MQTT broker.
	//	publishTelemetry();

	Serial.println( "Setup has completed.\n" );
} // End of setup() function.


/**
 * @brief configureOTA() will configure and initiate Over The Air (OTA) updates for this device.
 */
void configureOTA()
{

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

	ArduinoOTA.onStart( []()
							  { Serial.println( "Starting OTA communication." ); } );
	ArduinoOTA.onEnd( []()
							{ Serial.println( "\nTerminating OTA communication." ); } );
	ArduinoOTA.onProgress( []( unsigned int progress, unsigned int total )
								  { Serial.printf( "OTA progress: %u%%\r", ( progress / ( total / 100 ) ) ); } );
	ArduinoOTA.onError( []( ota_error_t error )
							  {
								Serial.printf( "Error[%u]: ", error );
								if( error == OTA_AUTH_ERROR ) Serial.println( "OTA authentication failed!" );
								else if( error == OTA_BEGIN_ERROR ) Serial.println( "OTA transmission failed to initiate properly!" );
								else if( error == OTA_CONNECT_ERROR ) Serial.println( "OTA connection failed!" );
								else if( error == OTA_RECEIVE_ERROR ) Serial.println( "OTA client was unable to properly receive data!" );
								else if( error == OTA_END_ERROR ) Serial.println( "OTA transmission failed to terminate properly!" ); } );
	ArduinoOTA.begin();
	Serial.println( "OTA is configured and ready." );
} // End of the configureOTA() function.


/**
 * @brief setupHTU21D() will initialize the sensor and check its status.
 */
void setupHTU21D()
{
	Serial.println( "Initializing the HTU21D sensor." );

	if( htu21d.begin() )
	{
		if( htu21d.isConnected() )
		{
			/*
				getStatus() 2-bit values:
				bits  value  meaning
				00    0      open circuit
				01    1      temperature reading
				10    2      humidity reading
				11    3      closed circuit
			*/
			uint8_t stat = htu21d.getStatus();
			Serial.printf( "HTU21D status code: %x\n", stat );

			stat = htu21d.getFirmwareVersion();
			Serial.printf( "HTU21D firmware version (hex): %x\n", stat );
			Serial.printf( "HTU21D firmware version (dec): %d\n", stat );

			Serial.printf( "HTU21D resolution (hex): %x\n", stat );

			if( htu21d.isHeaterOn() )
				htu21d.heatOff();
			Serial.println( "The HTU21D has been configured." );
		}
	}
} // End of setupHTU21D function.


/**
 * @brief wifiMultiConnect() will iterate through 'wifiSsidArray[]', attempting to connect with the password stored at the same index in 'wifiPassArray[]'.
 */
void wifiMultiConnect()
{
	digitalWrite( MCU_LED, 1 ); // Turn the LED off to show a connection is being made.

	Serial.println( "\nEntering wifiMultiConnect()" );
	for( size_t networkArrayIndex = 0; networkArrayIndex < sizeof( wifiSsidArray ); networkArrayIndex++ )
	{
		// Get the details for this connection attempt.
		const char *wifiSsid = wifiSsidArray[networkArrayIndex];
		const char *wifiPassword = wifiPassArray[networkArrayIndex];

		// Announce the WiFi parameters for this connection attempt.
		Serial.print( "Attempting to connect to SSID \"" );
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
			Serial.print( wifiConnectionTimeout / MILLIS_IN_SEC );
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
				delay( MILLIS_IN_SEC );
			}
			Serial.println( "" );

			if( WiFi.status() == WL_CONNECTED )
			{
				digitalWrite( MCU_LED, 0 ); // Turn the LED on to show the connection was successful.
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


/**
 * @brief checkForSSID() is used by wifiMultiConnect() to avoid attempting to connect to SSIDs which are not in range.
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


/**
 * @brief mqttMultiConnect() will:
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

	digitalWrite( MCU_LED, 1 ); // Turn the LED off to show a connection is being made.

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
		// Put the macAddress and random number into clientId.
		char clientId[22];
		snprintf( clientId, 22, "%s-%03ld", macAddress, random( 999 ) );
		// Connect to the broker using the MAC address for a clientID.  This guarantees that the clientID is unique.
		Serial.printf( "Connecting with client ID '%s'.\n", clientId );
		Serial.printf( "Attempt # %d....", ( attemptNumber + 1 ) );
		if( mqttClient.connect( clientId ) )
		{
			digitalWrite( MCU_LED, 0 ); // Turn the LED on to show the connection was successful.
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
				Serial.printf( "Successfully subscribed to topic '%s'.\n", mqttCommandTopic );
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

			Serial.printf( "Trying again in %lu seconds.\n\n", mqttReconnectDelay / MILLIS_IN_SEC );
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


/**
 * @brief readTelemetry() will:
 * 1. Read from all available sensors.
 * 2. Store legitimate values in global variables.
 * 3. Increment a counter if any value is invalid.
 */
void readTelemetry()
{
	rssi = WiFi.RSSI();
	// Read fresh data from the sensor.
	if( htu21d.read() )
	{
		// Get the results from the .read().
		float temporaryTemperature = htu21d.getTemperature();
		float temporaryHumidity = htu21d.getHumidity();

		// Define the valid temperature range (in Celsius) for this sensor.
		if( temporaryTemperature > -30 || temporaryTemperature < 90 )
		{
			tempC = temporaryTemperature;
			tempF = ( tempC * 9 / 5 ) + 32;
			// Clear the consecutive bad count.
			consecutiveBadTemp = 0;
		}
		else
		{
			Serial.printf( "Temperature reading of %.2f C is out of bounds and is being marked as invalid!\n", temporaryTemperature );
			consecutiveBadTemp++;
		}

		// Define the valid humidity range for this sensor.
		if( temporaryHumidity >= 0 || temporaryHumidity <= 100 )
		{
			humidity = temporaryHumidity;
			// Clear the consecutive bad count.
			consecutiveBadHumidity = 0;
		}
		else
		{
			Serial.printf( "Humidity reading of %.2f is out of bounds and is being marked as invalid!\n", temporaryHumidity );
			consecutiveBadHumidity++;
		}

		if( consecutiveBadTemp > 4 || consecutiveBadHumidity > 4 )
		{
			Serial.println( "\n\n\n\n" );
			Serial.printf( "%u consecutive bad temperature readings!\n", consecutiveBadTemp );
			Serial.printf( "%u consecutive bad humidity readings!\n", consecutiveBadHumidity );
			Serial.println( "Resetting the device!\n\n\n" );
			ESP.restart(); // Reset the device.
		}
	}
	lastPollTime = millis();
} // End of readTelemetry() function.


/**
 * @brief printTelemetry() will print the sensor and device data to the serial port.
 */
void printTelemetry()
{
	Serial.printf( "WiFi SSID: %s\n", wifiSsidArray[networkIndex] );
	Serial.printf( "Broker: %s:%d\n", mqttBrokerArray[networkIndex], mqttPortArray[networkIndex] );
	Serial.printf( "Temperature: %.2f C\n", tempC );
	Serial.printf( "Temperature: %.2f F\n", tempF );
	Serial.printf( "Humidity: %.2f %%\n", humidity );
	Serial.printf( "WiFi RSSI: %ld\n", rssi );
	Serial.printf( "Publish count: %lu\n", publishCount );
	Serial.printf( "File name: %s\n", __FILE__ );
	Serial.printf( "Next telemetry poll in %lu seconds\n\n", telemetryInterval / MILLIS_IN_SEC );
} // End of printTelemetry() function.


/**
 * @brief publishStats() is called by mqttConnect() every time the device (re)connects to the broker, and every publishInterval milliseconds thereafter.
 * It is also called by the callback when the "publishStats" command is received.
 */
void publishStats()
{
	char mqttStatsString[BUFFER_SIZE];
	// Create a JSON Document on the stack.
	StaticJsonDocument<BUFFER_SIZE> doc;
	// Add data: __FILE__, macAddress, ipAddress, rssi, publishCount
	doc["sketch"] = __FILE__;
	doc["mac"] = macAddress;
	doc["ip"] = ipAddress;
	doc["rssi"] = rssi;
	doc["publishCount"] = publishCount;

	// Serialize the JSON into mqttStatsString, with indentation and line breaks.
	serializeJsonPretty( doc, mqttStatsString );

	Serial.printf( "Publishing stats to the '%s' topic.\n", mqttStatsTopic );

	if( mqttClient.connected() )
	{
		if( mqttClient.connected() && mqttClient.publish( mqttStatsTopic, mqttStatsString ) )
			Serial.printf( "Published to this broker and port: %s:%d, and to this topic '%s':\n%s\n", mqttBrokerArray[networkIndex], mqttPortArray[networkIndex], mqttStatsTopic, mqttStatsString );
		else
			Serial.println( "\n\nPublish failed!\n\n" );
	}
} // End of publishStats() function.


/**
 * @brief publishTelemetry() will publish the sensor and device data over MQTT.
 */
void publishTelemetry()
{
	char mqttString[BUFFER_SIZE]; // A String to hold the JSON.

	// Create a JSON Document on the stack.
	StaticJsonDocument<BUFFER_SIZE> doc;
	// Add data: __FILE__, macAddress, ipAddress, temperature, tempF, humidity, rssi, publishCount, notes
	doc["sketch"] = __FILE__;
	doc["mac"] = macAddress;
	doc["ip"] = ipAddress;
	doc["tempC"] = tempC;
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
		Serial.println( "Successfully published to:" );
		char buffer[20];
		// Device topic format: <location>/<device>/<metric>
		// Sensor topic format: <location>/<device>/<sensor>/<metric>
		if( mqttClient.publish( sketchTopic, __FILE__, false ) )
			Serial.printf( "  %s\n", sketchTopic );
		if( mqttClient.publish( macTopic, macAddress, false ) )
			Serial.printf( "  %s\n", macTopic );
		if( mqttClient.publish( ipTopic, ipAddress, false ) )
			Serial.printf( "  %s\n", ipTopic );
		if( mqttClient.publish( rssiTopic, ltoa( rssi, buffer, 10 ), false ) )
			Serial.printf( "  %s\n", rssiTopic );
		if( mqttClient.publish( publishCountTopic, ltoa( publishCount, buffer, 10 ), false ) )
			Serial.printf( "  %s\n", publishCountTopic );
		if( mqttClient.publish( notesTopic, notes, false ) )
			Serial.printf( "  %s\n", notesTopic );
		// Convert the temperature in Celsius from a float to a char array.
		dtostrf( tempC, 1, 3, buffer );
		if( mqttClient.publish( tempCTopic, buffer, false ) )
			Serial.printf( "  %s\n", tempCTopic );
		dtostrf( ( tempF ), 1, 3, buffer );
		if( mqttClient.publish( tempFTopic, buffer, false ) )
			Serial.printf( "  %s\n", tempFTopic );
		dtostrf( ( humidity ), 1, 3, buffer );
		if( mqttClient.publish( humidityTopic, buffer, false ) )
			Serial.printf( "  %s\n", humidityTopic );

		Serial.printf( "Successfully published to '%s', this JSON:\n", mqttTopic );
	}
	else
		Serial.printf( "Failed to publish to '%s', this JSON:\n", mqttTopic );
	// Print the JSON to the Serial port.
	Serial.println( mqttString );
	lastPublishTime = millis();

	Serial.printf( "Next MQTT publish in %lu seconds.\n\n", publishInterval / MILLIS_IN_SEC );
} // End of publishTelemetry() function.


/**
 * @brief toggleLED() will change the state of the LED.
 * This function does not manage any timings.
 */
void toggleLED()
{
	if( digitalRead( MCU_LED ) != 1 )
		digitalWrite( MCU_LED, 1 );
	else
		digitalWrite( MCU_LED, 0 );
} // End of toggleLED() function.


/**
 * @brief The main loop function.
 */
void loop()
{
	// Check the WiFi and MQTT client connection state.
	if( !mqttClient.connected() )
		mqttMultiConnect( 5 );
	// The MQTT loop() function facilitates the receiving of messages and maintains the connection to the broker.
	mqttClient.loop();
	// The OTA handle() function broadcasts this device's presence to compatible clients on the same Wi-Fi network.
	ArduinoOTA.handle();

	unsigned long time = millis();
	// Print the first time.  Avoid subtraction overflow.  Print every interval.
	if( lastPollTime == 0 || ( time > telemetryInterval && ( time - telemetryInterval ) > lastPollTime ) )
	{
		readTelemetry();
		printTelemetry();
	}

	time = millis();
	// Publish the first time.  Avoid subtraction overflow.  Publish every interval.
	if( lastPublishTime == 0 || ( time > publishInterval && ( time - publishInterval ) > lastPublishTime ) )
	{
		publishCount++;
		readTelemetry();
		printTelemetry();
		publishTelemetry();
	}

	time = millis();
	// Process the first time.  Avoid subtraction overflow.  Process every interval.
	if( lastLedBlinkTime == 0 || ( ( time > ledBlinkInterval ) && ( time - ledBlinkInterval ) > lastLedBlinkTime ) )
	{
		lastLedBlinkTime = millis();

		// If Wi-Fi is connected, but MQTT is not, blink the LED.
		if( WiFi.status() == WL_CONNECTED )
		{
			if( mqttClient.state() != 0 )
				toggleLED();
			else
				digitalWrite( MCU_LED, 0 );   // Turn the LED on to show both Wi-Fi and MQTT are connected.
		}
	}
} // End of loop() function.
