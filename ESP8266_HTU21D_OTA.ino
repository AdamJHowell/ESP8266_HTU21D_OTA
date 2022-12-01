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
#include "ESP8266_HTU21D_OTA.h"


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
		Serial.print(( char ) payload[ i ] );
		str[ i ] = ( char ) payload[ i ];
	}
	Serial.println();
	// Add the null terminator.
	str[ i ] = 0;
	StaticJsonDocument <BUFFER_SIZE> doc;
	deserializeJson( doc, str );

	// The command can be: publishTelemetry, publishStatus, changeTelemetryInterval, or changePublishInterval.
	const char *command = doc[ "command" ];
	if( strcmp( command, "publishTelemetry" ) == 0 )
	{
		Serial.println( "Reading and publishing sensor values because MQTT received the 'publishTelemetry' command." );
		// Poll the sensor.
		readTelemetry();
		// Publish the sensor readings.
		publishTelemetry();
		Serial.println( "Readings have been published." );
	}
	else if( strcmp( command, "changeTelemetryInterval" ) == 0 )
	{
		Serial.println( "Changing the publish interval because MQTT received the 'changeTelemetryInterval' command." );
		unsigned long tempValue = doc[ "value" ];
		// Only update the value if it is greater than 4 seconds.  This prevents a seconds vs. milliseconds mix-up.
		if( tempValue > 4 * MILLIS_IN_SEC )
			publishInterval = tempValue;
		Serial.printf( "MQTT publish interval has been updated to %lu\n", publishInterval );
		lastPublishTime = 0;
	}
	else if( strcmp( command, "publishStats" ) == 0 )
	{
		Serial.println( "Publishing stats because MQTT received the 'publishStats' command." );
		readTelemetry();
		publishStats();
	}
	else
		Serial.printf( "Unknown command: %s\n", command );
} // End of onReceiveCallback() function.


/**
 * The setup() function runs once when the device is booted, and then loop() takes over.
 */
void setup()
{
	// Wait one second before starting serial communication, to give the user time to connect the terminal.
	delay( MILLIS_IN_SEC );
	Serial.begin( 115200 );
	// If the serial port has not connected yet, wait one more second before printing that this function has begun.
	if( !Serial )
		delay( MILLIS_IN_SEC );
	Serial.println( "\nSetup is initiating..." );

	// Start I2C communication.
	Wire.begin();

	// Set up the onboard LED, and turn it on (this devkit uses 0, or LOW, to turn the LED on).
	pinMode( LED_PIN, OUTPUT );
	digitalWrite( LED_PIN, 0 );

	Serial.println( __FILE__ );

	// Set up the HTU21D sensor.
	setupHTU21D();

	// Set ipAddress to a default value.
	snprintf( ipAddress, 16, "127.0.0.1" );

	// Get the MAC address and store it in macAddress.
	snprintf( macAddress, 18, "%s", WiFi.macAddress().c_str());

	Serial.println( "Connecting WiFi..." );
	wifiMultiConnect();

	// The networkIndex variable is initialized to 2112.  If it is still 2112 at this point, then WiFi failed to connect.
	if( networkIndex != 2112 )
	{
		const char *mqttBroker = mqttBrokerArray[ networkIndex ];
		const int mqttPort = mqttPortArray[ networkIndex ];
		// Set the MQTT client parameters.
		mqttClient.setServer( mqttBroker, mqttPort );
		// Assign the onReceiveCallback() function to handle MQTT callbacks.
		mqttClient.setCallback( onReceiveCallback );
		Serial.printf( "Using MQTT broker: %s\n", mqttBroker );
		Serial.printf( "Using MQTT port: %d\n", mqttPort );
	}

	// Configure Over-The-Air update functionality.
	configureOTA();

	Serial.printf( "IP address: %s\n", ipAddress );

	Serial.println( "Setup has completed.\n" );
} // End of setup() function.


/**
 * @brief configureOTA() will configure and initiate Over The Air (OTA) updates for this device.
 * An excellent OTA guide can be found here:
 * https://randomnerdtutorials.com/esp8266-ota-updates-with-arduino-ide-over-the-air/
 * The setPort(), setHostname(), and setPassword() functions are optional.
 */
void configureOTA()
{
	// Port defaults to 8266, but can be overridden here:
	// ArduinoOTA.setPort( 8266 );

	// Hostname defaults to esp8266-[ChipID], but can be overridden here:
	ArduinoOTA.setHostname( hostname );
	Serial.printf( "Using hostname '%s'\n", hostname );

	// No authentication by default, but a password can be set here:
	// ArduinoOTA.setPassword( ( const char * )"abc123" );

	// OTA callbacks are required:
	ArduinoOTA.onStart( []()
	                    { Serial.println( "Starting OTA communication." ); } );
	ArduinoOTA.onEnd( []()
	                  { Serial.println( "\nTerminating OTA communication." ); } );
	ArduinoOTA.onProgress( []( unsigned int progress, unsigned int total )
	                       { Serial.printf( "OTA progress: %u%%\r", ( progress / ( total / 100 ))); } );
	ArduinoOTA.onError( []( ota_error_t error )
	                    {
		                    Serial.printf( "Error[%u]: ", error );
		                    if( error == OTA_AUTH_ERROR ) Serial.println( "OTA authentication failed!" );
		                    else if( error == OTA_BEGIN_ERROR ) Serial.println( "OTA transmission failed to initiate properly!" );
		                    else if( error == OTA_CONNECT_ERROR ) Serial.println( "OTA connection failed!" );
		                    else if( error == OTA_RECEIVE_ERROR ) Serial.println( "OTA client was unable to properly receive data!" );
		                    else if( error == OTA_END_ERROR ) Serial.println( "OTA transmission failed to terminate properly!" );
	                    } );
	ArduinoOTA.begin();
	Serial.println( "OTA is configured and ready." );
} // End of the configureOTA() function.


/**
 * @brief setupHTU21D() will initialize the sensor and check its status.
 */
void setupHTU21D()
{
	Serial.println( "Initializing the HTU21D sensor." );

	if( htu21d.begin())
	{
		if( htu21d.isConnected())
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

			stat = htu21d.getResolution();
			Serial.printf( "HTU21D resolution (hex): %x\n", stat );

			if( htu21d.isHeaterOn())
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
	digitalWrite( LED_PIN, 1 ); // Turn the LED off to show a connection is being made.

	Serial.println( "\nEntering wifiMultiConnect()" );
	for( size_t networkArrayIndex = 0; networkArrayIndex < sizeof( wifiSsidArray ); networkArrayIndex++ )
	{
		// Get the details for this connection attempt.
		const char *wifiSsid = wifiSsidArray[ networkArrayIndex ];
		const char *wifiPassword = wifiPassArray[ networkArrayIndex ];

		// Announce the WiFi parameters for this connection attempt.
		Serial.print( "Attempting to connect to SSID \"" );
		Serial.print( wifiSsid );
		Serial.println( "\"" );

		// Don't even try to connect if the SSID cannot be found.
		if( checkForSSID( wifiSsid ))
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
			while( WiFi.status() != WL_CONNECTED && ( millis() - wifiConnectionStartTime < wifiConnectionTimeout ))
			{
				Serial.print( "." );
				delay( MILLIS_IN_SEC );
			}
			Serial.println( "" );

			if( WiFi.status() == WL_CONNECTED )
			{
				digitalWrite( LED_PIN, 0 ); // Turn the LED on to show the connection was successful.
				Serial.print( "IP address: " );
				snprintf( ipAddress, 16, "%d.%d.%d.%d", WiFi.localIP()[ 0 ], WiFi.localIP()[ 1 ], WiFi.localIP()[ 2 ], WiFi.localIP()[ 3 ] );
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
			if( strcmp( ssidName, WiFi.SSID( i ).c_str()) == 0 )
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
void mqttMultiConnect( int maxAttempts )
{
	unsigned long time = millis();
	// Connect the first time.  Avoid subtraction overflow.  Connect every interval.
	if( lastMqttConnectionTime == 0 || (( time > mqttReconnectCooldown ) && ( time - mqttReconnectCooldown ) > lastMqttConnectionTime ))
	{
		Serial.print( "Current time: " );
		Serial.println( time );
		Serial.print( "Last MQTT connection time: " );
		Serial.println( lastMqttConnectionTime );
		Serial.print( "MQTT boolean: " );
		Serial.println((( time > mqttReconnectCooldown ) && ( time - mqttReconnectCooldown ) > lastMqttConnectionTime ));

		Serial.println( "Function mqttMultiConnect() has initiated.\n" );
		if( WiFi.status() != WL_CONNECTED )
			wifiMultiConnect();

		digitalWrite( LED_PIN, 1 ); // Turn the LED off to show a connection is being made.

		/*
		 * The networkIndex variable is initialized to 2112.
		 * If it is still 2112 at this point, then WiFi failed to connect.
		 * This is only needed to display the name and port of the broker being used.
		 */
		if( networkIndex != 2112 )
		{
			Serial.print( "Attempting to connect to the MQTT broker at '" );
			Serial.print( mqttBrokerArray[ networkIndex ] );
			Serial.print( ":" );
			Serial.print( mqttPortArray[ networkIndex ] );
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
			snprintf( clientId, 22, "%s-%03ld", macAddress, random( 999 ));
			// Connect to the broker using the MAC address for a clientID.  This guarantees that the clientID is unique.
			Serial.printf( "Connecting with client ID '%s'.\n", clientId );
			Serial.printf( "Attempt # %d....", ( attemptNumber + 1 ));
			if( mqttClient.connect( clientId ))
			{
				lastMqttConnectionTime = millis();
				digitalWrite( LED_PIN, 0 ); // Turn the LED on to show the connection was successful.
				Serial.println( " connected." );
				if( !mqttClient.setBufferSize( BUFFER_SIZE ))
				{
					Serial.printf( "Unable to create a buffer %d bytes long!\n", BUFFER_SIZE );
					Serial.println( "Restarting the device!" );
					ESP.restart();
				}

				// Subscribe to the command topic.
				if( mqttClient.subscribe( mqttCommandTopic ))
					Serial.printf( "Successfully subscribed to topic '%s'.\n", mqttCommandTopic );
				else
					Serial.printf( "Failed to subscribe to topic '%s'!\n", mqttCommandTopic );
			}
			else
			{
				lastMqttConnectionTime = millis();
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

				if( maxAttempts > 1 )
				{
					Serial.printf( "Trying again in %lu seconds.\n\n", mqttReconnectDelay / MILLIS_IN_SEC );
					delay( mqttReconnectDelay );
				}
			}
			attemptNumber++;
		}

		if( !mqttClient.connected())
		{
			Serial.println( "\n" );
			Serial.println( "*************************************" );
			Serial.println( "Unable to connect to the MQTT broker!" );
			Serial.println( "*************************************" );
			Serial.println( "\n" );
			delay( MILLIS_IN_SEC );
		}
	}
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
	if( htu21d.read())
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
	Serial.println();
	Serial.printf( "MAC address: %s\n", macAddress );
	int wifiStatusCode = WiFi.status();
	char buffer[29];
	lookupWifiCode( wifiStatusCode, buffer );
	Serial.printf( "Wi-Fi status: %d, %s\n", wifiStatusCode, buffer );
	int mqttStateCode = mqttClient.state();
	lookupMQTTCode( mqttStateCode, buffer );
	Serial.printf( "MQTT state: %d, %s\n", mqttStateCode, buffer );
	if( wifiStatusCode == 3 )
	{
		Serial.printf( "Wi-Fi SSID: %s\n", WiFi.SSID() );
		Serial.printf( "IP address: %s\n", ipAddress );
		Serial.printf( "RSSI: %ld\n", rssi );
	}
	if( networkIndex != 2112 )
		Serial.printf( "Broker: %s:%d\n", mqttBrokerArray[ networkIndex ], mqttPortArray[ networkIndex ] );

	Serial.printf( "Hostname: %s\n", hostname );
	Serial.printf( "Sketch file name: %s\n", __FILE__ );
	Serial.printf( "Notes: %s\n", notes );

	Serial.printf( "Temperature: %.2f C\n", tempC );
	Serial.printf( "Temperature: %.2f F\n", tempF );
	Serial.printf( "Humidity: %.2f %%\n", humidity );
	Serial.printf( "Publish count: %lu\n", publishCount );
	Serial.printf( "Next telemetry poll in %lu seconds\n\n", telemetryInterval / MILLIS_IN_SEC );
} // End of printTelemetry() function.


/**
 * @brief lookupWifiCode() will return the string for an integer code.
 */
void lookupWifiCode( int code, char *buffer )
{
	switch( code )
	{
		case 0:
			snprintf( buffer, 26, "%s", "Idle" );
			break;
		case 1:
			snprintf( buffer, 26, "%s", "No SSID" );
			break;
		case 2:
			snprintf( buffer, 26, "%s", "Scan completed" );
			break;
		case 3:
			snprintf( buffer, 26, "%s", "Connected" );
			break;
		case 4:
			snprintf( buffer, 26, "%s", "Connection failed" );
			break;
		case 5:
			snprintf( buffer, 26, "%s", "Connection lost" );
			break;
		case 6:
			snprintf( buffer, 26, "%s", "Disconnected" );
			break;
		default:
			snprintf( buffer, 26, "%s", "Unknown Wi-Fi status code" );
	}
} // End of lookupWifiCode() function.


/**
 * @brief lookupMQTTCode() will return the string for an integer state code.
 */
void lookupMQTTCode( int code, char *buffer )
{
	switch( code )
	{
		case -4:
			snprintf( buffer, 29, "%s", "Connection timeout" );
			break;
		case -3:
			snprintf( buffer, 29, "%s", "Connection lost" );
			break;
		case -2:
			snprintf( buffer, 29, "%s", "Connect failed" );
			break;
		case -1:
			snprintf( buffer, 29, "%s", "Disconnected" );
			break;
		case 0:
			snprintf( buffer, 29, "%s", "Connected" );
			break;
		case 1:
			snprintf( buffer, 29, "%s", "Bad protocol" );
			break;
		case 2:
			snprintf( buffer, 29, "%s", "Bad client ID" );
			break;
		case 3:
			snprintf( buffer, 29, "%s", "Unavailable" );
			break;
		case 4:
			snprintf( buffer, 29, "%s", "Bad credentials" );
			break;
		case 5:
			snprintf( buffer, 29, "%s", "Unauthorized" );
			break;
		default:
			snprintf( buffer, 29, "%s", "Unknown MQTT state code" );
	}
}


/**
 * @brief publishStats() is called by mqttConnect() every time the device (re)connects to the broker, and every publishInterval milliseconds thereafter.
 * It is also called by the callback when the "publishStats" command is received.
 */
void publishStats()
{
	char mqttStatsString[BUFFER_SIZE];
	// Create a JSON Document on the stack.
	StaticJsonDocument <BUFFER_SIZE> doc;
	// Add data: __FILE__, macAddress, ipAddress, rssi, publishCount
	doc[ "sketch" ] = __FILE__;
	doc[ "mac" ] = macAddress;
	doc[ "ip" ] = ipAddress;
	doc[ "rssi" ] = rssi;
	doc[ "notes" ] = notes;
	doc[ "publishCount" ] = publishCount;

	// Serialize the JSON into mqttStatsString, with indentation and line breaks.
	serializeJsonPretty( doc, mqttStatsString );

	Serial.printf( "Publishing stats to the '%s' topic.\n", mqttStatsTopic );

	if( mqttClient.connected())
	{
		if( mqttClient.connected() && mqttClient.publish( mqttStatsTopic, mqttStatsString ))
			Serial.printf( "Published to this broker and port: %s:%d, and to this topic '%s':\n%s\n", mqttBrokerArray[ networkIndex ], mqttPortArray[ networkIndex ], mqttStatsTopic, mqttStatsString );
		else
			Serial.println( "\n\nPublish failed!\n\n" );
	}
} // End of publishStats() function.


/**
 * @brief publishTelemetry() will publish the sensor and device data over MQTT.
 * It is also called by the callback when the "publishTelemetry" command is received.
 */
void publishTelemetry()
{
	char mqttString[BUFFER_SIZE]; // A String to hold the JSON.

	// Create a JSON Document on the stack.
	StaticJsonDocument <BUFFER_SIZE> doc;
	// Add data: __FILE__, macAddress, ipAddress, temperature, tempF, humidity, rssi, publishCount, notes
	doc[ "sketch" ] = __FILE__;
	doc[ "mac" ] = macAddress;
	doc[ "ip" ] = ipAddress;
	doc[ "rssi" ] = rssi;
	doc[ "notes" ] = notes;
	doc[ "tempC" ] = tempC;
	doc[ "tempF" ] = tempF;
	doc[ "humidity" ] = humidity;
	doc[ "publishCount" ] = publishCount;

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
		if( mqttClient.publish( sketchTopic, __FILE__, false ))
			Serial.printf( "  %s\n", sketchTopic );
		if( mqttClient.publish( macTopic, macAddress, false ))
			Serial.printf( "  %s\n", macTopic );
		if( mqttClient.publish( ipTopic, ipAddress, false ))
			Serial.printf( "  %s\n", ipTopic );
		if( mqttClient.publish( rssiTopic, ltoa( rssi, buffer, 10 ), false ))
			Serial.printf( "  %s\n", rssiTopic );
		if( mqttClient.publish( publishCountTopic, ltoa( publishCount, buffer, 10 ), false ))
			Serial.printf( "  %s\n", publishCountTopic );
		if( mqttClient.publish( notesTopic, notes, false ))
			Serial.printf( "  %s\n", notesTopic );
		// Convert the temperature in Celsius from a float to a char array.
		dtostrf( tempC, 1, 3, buffer );
		if( mqttClient.publish( tempCTopic, buffer, false ))
			Serial.printf( "  %s\n", tempCTopic );
		dtostrf(( tempF ), 1, 3, buffer );
		if( mqttClient.publish( tempFTopic, buffer, false ))
			Serial.printf( "  %s\n", tempFTopic );
		dtostrf(( humidity ), 1, 3, buffer );
		if( mqttClient.publish( humidityTopic, buffer, false ))
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
	if( digitalRead( LED_PIN ) != 1 )
		digitalWrite( LED_PIN, 1 );
	else
		digitalWrite( LED_PIN, 0 );
} // End of toggleLED() function.


/**
 * @brief The main loop function.
 */
void loop()
{
	// Check the WiFi and MQTT client connection state.
	if( !mqttClient.connected())
		mqttMultiConnect( 1 );
	// The MQTT loop() function facilitates the receiving of messages and maintains the connection to the broker.
	mqttClient.loop();
	// The OTA handle() function broadcasts this device's presence to compatible clients on the same Wi-Fi network.
	ArduinoOTA.handle();

	unsigned long currentTime = millis();
	// Print the first currentTime.  Avoid subtraction overflow.  Print every interval.
	if( lastPollTime == 0 || ( currentTime > telemetryInterval && ( currentTime - telemetryInterval ) > lastPollTime ))
	{
		readTelemetry();
		printTelemetry();
	}

	currentTime = millis();
	// Publish the first currentTime.  Avoid subtraction overflow.  Publish every interval.
	if( lastPublishTime == 0 || ( currentTime > publishInterval && ( currentTime - publishInterval ) > lastPublishTime ))
	{
		publishCount++;
		readTelemetry();
		printTelemetry();
		publishTelemetry();
	}

	currentTime = millis();
	// Process the first currentTime.  Avoid subtraction overflow.  Process every interval.
	if( lastLedBlinkTime == 0 || (( currentTime > ledBlinkInterval ) && ( currentTime - ledBlinkInterval ) > lastLedBlinkTime ))
	{
		lastLedBlinkTime = millis();

		// If Wi-Fi is connected, but MQTT is not, blink the LED.
		if( WiFi.status() == WL_CONNECTED )
		{
			if( mqttClient.state() != 0 )
				toggleLED();
			else
				digitalWrite( LED_PIN, 0 );   // Turn the LED on to show both Wi-Fi and MQTT are connected.
		}
		else
			digitalWrite( LED_PIN, 1 );  // Turn the LED off to show that Wi-Fi is not connected.
	}
} // End of loop() function.
