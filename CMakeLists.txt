cmake_minimum_required( VERSION 3.23 )
project( ESP8266_HTU21D_OTA )

set( CMAKE_CXX_STANDARD 14 )

include_directories( .
		C:/Users/Adam/AppData/Local/Arduino15/packages/arduino/hardware/avr/1.8.5/cores/arduino/
		C:/Code/Arduino/libraries/libraries/
		C:/Code/Arduino/libraries/libraries/PubSubClient/src/
		C:/Code/Arduino/libraries/ArduinoJson/
		C:/Code/Arduino/libraries/PubSubClient/src/
		"C:/Program Files (x86)/Arduino/hardware/arduino/avr/cores/arduino/"
		"C:/Program Files (x86)/Arduino/hardware/tools/avr/lib/gcc/avr/7.3.0/include/"
		C:/Users/Adam/AppData/Local/Arduino15/packages/arduino/tools/avr-gcc/7.3.0-atmel3.6.1-arduino7/avr/include/ )

add_executable( ESP8266_HTU21D_OTA
		ESP8266_HTU21D_OTA.ino
		ESP8266_HTU21D_OTA.h
		privateInfo.h )

set_target_properties( ESP8266_HTU21D_OTA PROPERTIES LINKER_LANGUAGE CXX )
