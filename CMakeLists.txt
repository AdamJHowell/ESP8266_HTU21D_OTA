cmake_minimum_required( VERSION 3.23 )
project( ESP8266_HTU21D_OTA )

set( CMAKE_CXX_STANDARD 14 )

include_directories( . )

add_executable( ESP8266_HTU21D_OTA
		ESP8266_HTU21D_OTA.ino
		privateInfo.h )
