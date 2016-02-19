//******************************************************************************************
//  File: PS_TemperatureHumidity.cpp
//  Authors: Dan G Ogorchock & Daniel J Ogorchock (Father and Son)
//
//  Summary:  PS_TemperatureHumidity is a class which implements both the SmartThings "Temperature Measurement" 
//			  and "Relative Humidity Measurement" device capabilities.
//			  It inherits from the st::PollingSensor class.  The current version uses a digital input to measure the 
//			  temperature and humidity from a DHT series sensor.  This was tested with both the DHT11 and DHT22.  
//
//			  Create an instance of this class in your sketch's global variable section
//			  For Example:  st::PS_TemperatureHumidity sensor2("temphumid", 120000, 3000, PIN_TEMPERATUREHUMIDITY, st::PS_TemperatureHumidity::DHT22);
//
//			  st::PS_TemperatureHumidity() constructor requires the following arguments
//				- String &name - REQUIRED - the name of the object - must match the Groovy ST_Anything DeviceType tile name
//				- long interval - REQUIRED - the polling interval in seconds
//				- long offset - REQUIRED - the polling interval offset in seconds - used to prevent all polling sensors from executing at the same time
//				- byte pin - REQUIRED - the Arduino Pin to be used as a digital output
//				- DHT_SENSOR DHTSensorType - REQUIRED - the type of DHT sensor (DHT11, DHT21, DHT22, DHT33, or DHT44)
//				- String strTemp - OPTIONAL - name of temperature sensor to send to ST Cloud (defaults to "temperature")
//				- String strHumid - OPTIONAL - name of humidity sensor to send to ST Cloud (defaults to "humidity")
//
//			  This class supports receiving configuration data from the SmartThings cloud via the ST App.  A user preference
//			  can be configured in your phone's ST App, and then the "Configure" tile will send the data for all sensors to 
//			  the ST Shield.  For PollingSensors, this data is handled in the beSMart() function.
//
//			  TODO:  Determine a method to persist the ST Cloud's Polling Interval data
//
//  Change History:
//
//    Date        Who            What
//    ----        ---            ----
//    2015-01-03  Dan & Daniel   Original Creation
//	  2015-01-17  Dan Ogorchock	 Added optional temperature and humidity device names in constructor to allow multiple Temp/Humidity sensors
//    2015-03-29  Dan Ogorchock	 Optimized use of the DHT library (made it static) to reduce SRAM memory usage at runtime.
//
//
//******************************************************************************************

#include "PS_TemperatureHumidity.h"

#include "Constants.h"
#include "Everything.h"
#include <OneWire.h>

namespace st
{
//private
	

//public
	//constructor - called in your sketch's global variable declaration section
	PS_TemperatureOneWire::PS_TemperatureOneWire(const __FlashStringHelper *name, unsigned int interval, int offset, byte digitalInputPin, ONEWIRE_SENSOR OneWireSensorType, String strTemp) :
		PollingSensor(name, interval, offset),
		m_nTemperatureSensorValue(0),
		m_bOneWireSensorType(OneWireSensorType),
		m_strTemperature(strTemp)
	{
		setPin(digitalInputPin);
	}
	
	//destructor
	PS_TemperatureOneWire::~PS_TemperatureOneWire()
	{
		
	}

	//SmartThings Shield data handler (receives configuration data from ST - polling interval, and adjusts on the fly)
	void PS_TemperatureOneWire::beSmart(const String &str)
	{
		String s = str.substring(str.indexOf(' ') + 1);

		if (s.toInt() != 0) {
			st::PollingSensor::setInterval(s.toInt() * 1000);
			if (st::PollingSensor::debug) {
				Serial.print(F("PS_TemperatureOneWire::beSmart set polling interval to "));
				Serial.println(s.toInt());
			}
		}
		else {
			if (st::PollingSensor::debug) 
			{
				Serial.print(F("PS_TemperatureOneWire::beSmart cannot convert "));
				Serial.print(s);
				Serial.println(F(" to an Integer."));
			}
		}
	}

	//initialization routine - get first set of readings and send to ST cloud
	void PS_TemperatureOneWire::init()
	{
		delay(1500);		//Needed to prevent "Unknown Error" on first read of DHT Sensor
		getData();
	}
	
	//function to get data from sensor and queue results for transfer to ST Cloud 
	void PS_TemperatureOneWire::getData()
	{
		// READ DATA
		
	byte i;
    byte present = 0;
	byte type_s;
	byte data[12];
    byte addr[8];
    float celsius, fahrenheit;
    
    if ( !ds.search(addr)) {
    	ds.reset_search();
    	delay(250);
    	return;
    }
    if (OneWire::crc8(addr, 7) != addr[7]) {
    	
		if (st::PollingSensor::debug) {
      		Serial.println("PS_TemperatureOneWire: CRC is not valid!");
      	}
      return;
    }
    // the first ROM byte indicates which chip
  	switch (addr[0]) {
    	case 0x10: // or old DS1820
      		type_s = 1;
      		break;
    	case 0x28:
      		type_s = 0;
      		break;
    	case 0x22:
      		type_s = 0;
      		break;
    	default:
    		
			if (st::PollingSensor::debug) {
      			Serial.println("PS_TemperatureOneWire: Device is not a DS18x20 family device.");
      		}
      return;
  	} 
  	
  	ds.reset();
  	ds.select(addr);
  	ds.write(0x44);
  	
  	delay(1000);
  	
  	present = ds.reset();
  	ds.select(addr);
  	ds.write(0xBE):
  	for ( i = 0; i < 9; i++ ) {
  	 	data[i] = ds.read();
  	}
  	
  	// Convert the data to actual temperature
 	// because the result is a 16 bit signed integer, it should
	// be stored to an "int16_t" type, which is always 16 bits
  	// even when compiled on a 32 bit processor.
  	int16_t raw = (data[1] << 8) | data[0];
  	if (type_s) {
    	raw = raw << 3; // 9 bit resolution default
    	if (data[7] == 0x10) {
      		// "count remain" gives full 12 bit resolution
      		raw = (raw & 0xFFF0) + 12 - data[6];
    	}
 	} else {
    	byte cfg = (data[4] & 0x60);
    	// at lower res, the low bits are undefined, so let's zero them
    	if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    	else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    	else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    	//// default is 12 bit resolution, 750 ms conversion time
  	}
  	celsius = (float)raw / 16.0;
  	fahrenheit = celsius * 1.8 + 32.0;
  	m_nTemperatureSensorValue = fahrenheit;




	
		Everything::sendSmartString(m_strTemperature + " " + String(m_nTemperatureSensorValue));
	}
	
	void PS_TemperatureOneWire::setPin(byte pin)
	{
		m_nDigitalInputPin=pin;
		ds(m_nDigitalInputPin);
	}


	//initialize static members
	ds PS_TemperatureOneWire::OneWire;					//DHT library object
}