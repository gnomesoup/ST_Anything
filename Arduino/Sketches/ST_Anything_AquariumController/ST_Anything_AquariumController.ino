//******************************************************************************************
//  File: ST_Anything.ino
//  Authors: Dan G Ogorchock & Daniel J Ogorchock (Father and Son)
//
//  Summary:  This Arduino Sketch, along with the ST_Anything library and the revised SmartThings 
//            library, demonstrates the ability of one Arduino + SmartThings Shield to 
//            implement a multi input/output custom device for integration into SmartThings.
//            The ST_Anything library takes care of all of the work to schedule device updates
//            as well as all communications with the SmartThings Shield.
//
//            During the development of this re-usable library, it became apparent that the 
//            Arduino UNO R3's very limited 2K of SRAM was very limiting in the number of 
//            devices that could be implemented simultaneously.  A tremendous amount of effort
//            has gone into reducing the SRAM usage, including siginificant improvements to
//            the SmartThings Arduino library.  The SmartThings library was also modified to
//            include support for using Hardware Serial port(s) on the UNO, MEGA, and Leonardo.
//            During testing, it was determined that the Hardware Serial ports provide much
//            better performance and reliability versus the SoftwareSerial library.  Also, the
//            MEGA 2560's 8K of SRAM is well worth the few extra dollars to save your sanity
//            versus always running out of SRAM on the UNO R3.  The MEGA 2560 also has 4 Hardware
//            serial ports (i.e. UARTS) which makes it very easy to use Hardware Serial instead 
//            of SoftwareSerial, while still being able to see debug data on the USB serial 
//            console port (pins 0 & 1).  
//
//            Note: We did not have a Leonardo for testing, but did fully test on UNO R3 and 
//            MEGA 2560 using both SoftwareSerial and Hardware Serial communications to the 
//            Thing Shield.
//    
//  Change History:
//
//    Date        Who            What
//    ----        ---            ----
//    2015-01-03  Dan & Daniel   Original Creation
//    2015-03-28  Dan Ogorchock  Removed RCSwitch #include now that the libraries are split up
//    2015-03-31  Daniel O.      Memory optimizations utilizing progmem
//
//******************************************************************************************
//******************************************************************************************
// SmartThings Library for Arduino Shield
//******************************************************************************************
#include <SoftwareSerial.h> //Arduino UNO/Leonardo uses SoftwareSerial for the SmartThings Library
#include <SmartThings.h>    //Library to provide API to the SmartThings Shield
#include <dht.h>            //DHT Temperature and Humidity Library 
#include <avr/pgmspace.h>
#include <OneWire.h> //Maxum OneWire Library for DS18S20 temperature sensor

//******************************************************************************************
// ST_Anything Library 
//******************************************************************************************
#include <Constants.h>       //Constants.h is designed to be modified by the end user to adjust behavior of the ST_Anything library
#include <Device.h>          //Generic Device Class, inherited by Sensor and Executor classes
#include <Sensor.h>          //Generic Sensor Class, typically provides data to ST Cloud (e.g. Temperature, Motion, etc...)
#include <Executor.h>        //Generic Executor Class, typically receives data from ST Cloud (e.g. Switch)
#include <InterruptSensor.h> //Generic Interrupt "Sensor" Class, waits for change of state on digital input 
#include <PollingSensor.h>   //Generic Polling "Sensor" Class, polls Arduino pins periodically
#include <Everything.h>      //Master Brain of ST_Anything library that ties everything together and performs ST Shield communications

#include <PS_Illuminance.h>  //Implements a Polling Sensor (PS) to measure light levels via a photo resistor

#include <PS_TemperatureHumidity.h>  //Implements a Polling Sensor (PS) to measure Temperature and Humidity via DHT library
#include <PS_TemperatureOneWire.h>
#include <EX_Switch.h>       //Implements an Executor (EX) via a digital output to a relay

//******************************************************************************************
//Define which Arduino Pins will be used for each device
//  Notes: -Serial Communications Pins are defined in Constants.h (avoid pins 0,1,2,3
//          for inputs and output devices below as they may be used for communications)
//         -Always avoid Pin 6 as it is reserved by the SmartThings Shield
//
//******************************************************************************************
//"RESERVED" pins for SmartThings ThingShield - best to avoid
#define PIN_O_RESERVED               0  //reserved by ThingShield for Serial communications OR USB Serial Monitor
#define PIN_1_RESERVED               1  //reserved by ThingShield for Serial communications OR USB Serial Monitor
#define PIN_2_RESERVED               14  //reserved by ThingShield for Serial communications
#define PIN_3_RESERVED               15  //reserved by ThingShield for Serial communications
#define PIN_6_RESERVED               6  //reserved by ThingShield (possible future use?)

#define PIN_TEMP_AQUARIUM            4
#define PIN_TEMPHUMID_AIR            5
#define PIN_SWITCH                   13
OneWire ds(PIN_TEMP_AQUARIUM);
int aquaHigh;
int aquaLow;

//******************************************************************************************
//Arduino Setup() routine
//******************************************************************************************
void setup()
{
  //******************************************************************************************
  //Declare each Device that is attached to the Arduino
  //  Notes: - For each device, there is typically a corresponding "tile" defined in your 
  //           SmartThings DeviceType Groovy code
  //         - For details on each device's constructor arguments below, please refer to the 
  //           corresponding header (.h) and program (.cpp) files.
  //         - The name assigned to each device (1st argument below) must match the Groovy
  //           DeviceType Tile name.  (Note: "temphumid" below is the exception to this rule
  //           as the DHT sensors produce both "temperature" and "humidity".  Data from that
  //           particular sensor is sent to the ST Shield in two separate updates, one for 
  //           "temperature" and one for "humidity")
  //******************************************************************************************
  //Polling Sensors
  static st::PS_TemperatureOneWire sensor1(F("o_Aquarium"), 30, 3, PIN_TEMP_AQUARIUM, st::PS_TemperatureOneWire::DS18B20, "t_Aquarium");
  static st::PS_TemperatureHumidity sensor2(F("th_Air"), 30, 5, PIN_TEMPHUMID_AIR, st::PS_TemperatureHumidity::DHT22, "t_Air", "h_Air");

  /*
  int t_Aquarium;
  getTemp(t_Aquarium);
  if( !aquaHigh || aquaHigh < t_Aquarium ) { aquaHigh = t_Aquarium; }
  if( !aquaLow || aquaLow > t_Aquarium ) { aquaLow = t_Aquarium; }
  String sc="t_Aquarium " + (String)t_Aquarium;
  st::Everything::sendSmartString(sc);
  String sc1="aquaHigh " + (String)aquaHigh;
  //st::Everything::sendSmartString(sc1);
  String sc2="aquaLow " + (String)aquaLow;
  //st::Everything::sendSmartString(sc2);
  */
  
  //Executors
  static st::EX_Switch executor1(F("switch"), PIN_SWITCH, LOW, false);
  
  //*****************************************************************************
  //  Configure debug print output from each main class 
  //  -Note: Set these to "false" if using Hardware Serial on pins 0 & 1
  //         to prevent communication conflicts with the ST Shield communications
  //*****************************************************************************
  st::Everything::debug=true;
  st::Executor::debug=true;
  st::Device::debug=true;
  st::PollingSensor::debug=true;
  st::InterruptSensor::debug=true;
  
  //*****************************************************************************
  //Initialize the "Everything" Class
  //*****************************************************************************
  st::Everything::init();
  
  //*****************************************************************************
  //Add each sensor to the "Everything" Class
  //*****************************************************************************
  st::Everything::addSensor(&sensor1);
  
  st::Everything::addSensor(&sensor2);
  
  //*****************************************************************************
  //Add each executor to the "Everything" Class
  //*****************************************************************************
  st::Everything::addExecutor(&executor1);
  
  //*****************************************************************************
  //Initialize each of the devices which were added to the Everything Class
  st::Everything::initDevices();
  //*****************************************************************************
}

//******************************************************************************************
//Arduino Loop() routine
//******************************************************************************************
void loop()
{
  //*****************************************************************************
  //Execute the Everything run method which takes care of "Everything"
  //*****************************************************************************
  /*
  int t_Aquarium;
  getTemp(t_Aquarium);
  if( !aquaHigh || aquaHigh < t_Aquarium ) { aquaHigh = t_Aquarium; }
  if( !aquaLow || aquaLow > t_Aquarium ) { aquaLow = t_Aquarium; }
  String sc="t_Aquarium " + (String)t_Aquarium;
  st::Everything::sendSmartString(sc);
  String sc1="aquaHigh " + (String)aquaHigh;
  //st::Everything::sendSmartString(sc1);
  String sc2="aquaLow " + (String)aquaLow;
  //st::Everything::sendSmartString(sc2);
 */
  st::Everything::run();

  
}

void getTemp(int &fahrenheit)
{
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius;

  if( !ds.search(addr))
  {
    ds.reset_search();
    delay(250);
    return;
  }

  if( OneWire::crc8(addr, 7) != addr[7])
  {
    if( st::PollingSensor::debug ) {
      Serial.print("OneWire: CRC is not valid");
    }
    return;
  }

  switch (addr[0])
  {
    case 0x10:
      type_s = 1;
      break;
    case 0x28:
      type_s = 0;
      break;
    case 0x22:
      type_s = 0;
      break;
    default:
      if( st::PollingSensor::debug ) {
        Serial.print("OneWire: Device is not a DS18*20 family device");
      }
      return;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44);

  delay(1000);
  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);
  for( i=0; i<9; i++)
  {
    data[i] = ds.read();
  }

  int16_t raw = (data[1] << 8) | data[0];
  if( type_s )
  {
    raw = raw << 3;
    if( data[7] == 0x10)
    {
      raw = ( raw & 0xFFF0 ) + 12 - data[6];
    }
  } else {
    byte cfg = ( data[4] & 0x60 );
    if (cfg == 0x00) raw = raw & ~7;
    else if (cfg == 0x20) raw = raw & ~3;
    else if (cfg == 0x40) raw = raw & ~1;
  }
  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
}


