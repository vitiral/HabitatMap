//Control code for M=pod
//Revision 1: Omkar Mansata, omkar@umich.edu   Yun Xiang
//Revison  1.5: Yun Xiang, xiangyun@umich.edu
//Revision 2: Yun Xiang
//Revision 3: Yun Xiang
//Revision 4: Garrett Berg for use with Aircasting App
//********* WARNING: Bluetooth Module needs to be Initialized using the epodBluetoothInit Code the first time ever
//After doing this once, you DO NOT need to do this again ever unless you think something is wrong*************/


//I2C based Components
//S100 CO2 Sensor IC10
//Humidity Sensor U16
//Temperature Sensor U17
//light sensor
//EEPROM memory

//WT11 Bluetooth Module
//RX - RXD
//TX - TXD
//WT11_RESET - PD7

//Fan Controller - PB1

#include <Wire.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include <EEPROM.h>
#include <Time.h>
//#include <avr/pgmspace.h>
//#include <avr/wdt.h>

#include <MemoryFree.h>

//#define DEBUG
#include <SoftwareSerial.h>
//SoftwareSerial dSerial(MOSI, MISO); // RX, TX

#include <errorhandling.h>



//This is the buffer size!!
#define Buffer_size 1
#define Buffer_length 12 // should be Data_length/2
#define Print_length 10 //length of contents to be printed
#define Data_length 24  //the length of bytes stored in the mem //*important: it must be a multiple of 8!!!!!
#define Update_interval 6 //sampling every seconds

//new pins: 3, 10
#define LED_CONTROL_PIN 8   //PIN 12
//#define VDD_SENSE_PIN A0    // 5V supply voltage sensing

#define CO_NO_CONTROL_PIN 10     //PIN 14 1 when only one sensor works, 0 means they work simultaneously, should always be 0
#define PGGOOD_PIN 3       //PIN 1 not used anmore
#define NO_SENSE_PIN A1  //PIN 24

#define CO_CONTROL_PIN 6        //PIN 10
#define VOC_SENSE_ON_PIN 2       //PIN 32
#define EXT_CO_PIN A0
#define VOC_SENSE_PIN A3   //PIN 26  // VOC = Volotile Organic Compounds
#define BAT_SENSE_PIN A2         //PIN 25 
#define NO_CONTROL_PIN 4     //PIN 2
#define OZONE_CONTROL_PIN 5      //PIN 9
#define CO_SENSE_PIN A7
#define OZONE_SENSE_PIN A6

#define FAN_CONTROL_PIN 9            //PIN 13

// I2C address:
// Light_address 1000100
// SHT21_address 1000000
// Mem_address 101000x

//define commands
#define POWER_OFF   100   //character d
#define POWER_ON    101   //character e
#define STOP        999
#define START       99
#define SEND_DATA   102  //character f
#define STATE       103  //character g
#define SYNC_header   'T'  //starting with character T
#define UP_interval   'U'  //starting with character U
#define STREAM_MODE  'X'
#define BUF_MODE     'B'
#define ERASE_MEM    'E'
#define RESEND_DATA  'R'

//define sync parameters
#define TIME_MSG_LEN  10   // time sync to PC is HEADER followed by Unix time_t as ten ASCII digits
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 
//T1262347200 //noon Jan 1 2010


// ****************************************************************************************8
// ************   SENSORS

uint16_t ReadHumiditySensor()
{
  uint16_t humidityValue = 0;

  //Humidity Sensor START
  Wire.beginTransmission(0x40);
  //Wire.write(0xF5);
  Wire.write(0x05);
  Wire.endTransmission();   
  delay (100);
  Wire.requestFrom(0x40,3);        //Read 2 bytes from the Humidity Sensor with read address  
  if( Wire.available() >= 2 )
  {
    humidityValue = Wire.read();            //Convert ASCII to decimal: 0x32 converts to 0x02
    humidityValue = humidityValue << 8;        //Upper High Byte (Most significant byte received first)     
    humidityValue |= Wire.read();           //Convert to ASCII and append to end of Data structure
    
    //humidityValue = humidityValue >> 2;        // get rid of the unwanted data
    humidityValue &= 0xFFF;
  }    
  return humidityValue;
  //Humidity Sensor END 
}

uint16_t ReadTempSensor()
{
  uint16_t tempValue = 0;

  //Temperature Sensor START
  Wire.beginTransmission(0x40);
  //Wire.write(0xF3);
  Wire.write(0x03);
  Wire.endTransmission();   
  delay (100);
  Wire.requestFrom(0x40,3);        //Read 2 bytes from the Temperature Sensor with read address  
  if( Wire.available() >= 2 )
  {
    tempValue = Wire.read();            //Convert ASCII to decimal: 0x32 converts to 0x02
    tempValue = tempValue << 8;        //Upper High Byte (Most significant byte received first)     
    tempValue |= Wire.read();           //Convert to ASCII and append to end of Data structure
    //tempValue = tempValue >> 2;					//get rid of the unwanted data
    tempValue &= 0x3FFF;
  }
  return tempValue;
  //Humidity Sensor END 
}

unsigned int ReadCO2Sensor()
{
  unsigned int CO2Value = 0;
  //S100 CO2 Sensor START
  Wire.beginTransmission(0x31);    //Write to Write Data Address of S100
  Wire.write(0x52);                 //Send Command to Read
  Wire.endTransmission();
  //delay (200);   
  delay (200);
  Wire.requestFrom(0x31,3);        //Read 7 bytes from the S100 with read address
  if(Wire.available() >= 3)
  { 
    Wire.read();//configuration byte
    CO2Value = Wire.read();    //first byte  
    CO2Value = CO2Value << 8;
    CO2Value += (Wire.read());          //second byte
  }

  return CO2Value;
}

void LightBegin()
{

  Wire.beginTransmission(0x44);    //write 
  Wire.write(0x00);                 //Send to Command register
  Wire.write(0x88);                   //10001001    I used 12 bits resolution rather than 16 bit.
  Wire.endTransmission();
}

void LightInit()
{   
  Wire.beginTransmission(0x44);    //write
  Wire.write(0x01);                 //Send to control register
  Wire.write(0x06);                 //00001100    I used 12 bits resolution rather than 16 bit.
  Wire.endTransmission();
  LightBegin();
}

unsigned int ReadLightSensor()
{
  unsigned int lightValue = 0;
  //LightBegin();
  //delay(100);
  Wire.beginTransmission(0x44);    //Write to Write Data Address
  Wire.write(0x05);                 //Send Command to Read MSB,should be 05
  Wire.endTransmission();
  //Wire.requestFrom(0x44,1);
  Wire.requestFrom(0x44,1);        //Read 7 bytes from the S100 with read address
  if (Wire.available())
  { 
    lightValue = Wire.read();    //first byte
    lightValue = lightValue << 8;  
  }
  Wire.beginTransmission(0x44);    
  Wire.write(0x04);                 //Send Command to Read LSB, should be 04
  Wire.endTransmission();
  Wire.requestFrom(0x44,1);      
  //Wire.requestFrom(0x44,1);   
  if (Wire.available())
  { 
    lightValue += Wire.read();    //first byte  
  }
  return lightValue;
}

void TempSensorsInit_TI()
{

  //Set up one shot modes for both TMP100 U17 and U18 chips
  Wire.beginTransmission(0x48);    //Write to Write Data Address of U29
  Wire.write(0x01);                 //Send command to write to pointer register 0, i.e. configuration register
  Wire.write(0x60);                 //Write to configuration register OS/ALERT = 0; R1/R0 = 11 (12 bits); F1/F0 = 00 (fault condition, don't care in our case)
  //POL = 0 (Don't care); TM = 0 (Don't care); SD = 0 (set up to continuous conversion mode)
  //Can set up in one-shot mode to consume 0.1uA current vs. 75uA current in continuous mode
  Wire.endTransmission();
  delay(100);

  Wire.beginTransmission(0x48);    //Write to Write Data Address of U29
  Wire.write(0x00);                 //Send command to set up pointer register to point to the temperature read register
  Wire.endTransmission();
}

//float ReadTempSensor_TI()
uint16_t ReadTempSensor_TI()
{
  unsigned int tempValue = 0;
  unsigned int interValue = 0;

  /*
  Wire.beginTransmission(0x48);    //Write to Write Data Address of U17
  Wire.write(0x00);
  Wire.endTransmission();  
  Wire.requestFrom(0x48,2);        //Read 2 bytes from the TEMP Sensor 2 with read address
  */
  Wire.beginTransmission(0x48);    //Write to Write Data Address of U17
  Wire.requestFrom(0x48,2);        //Read 2 bytes from the TEMP Sensor 2 with read address
  Wire.endTransmission(); 
  
  if( 2==Wire.available() )
  {
    tempValue = Wire.read();          //Recieve High Byte
    tempValue = tempValue << 4;          //Shift Left by 4 since, it is a complete 12 bit number
    interValue =  Wire.read();        //Receive Lower byte, first 4 MSBs are relevant, last 4 MSBs are 0 and irrelevant
    interValue = interValue >> 4;          //Shift Lower byte by 4
    //tempValue |= interValue;             //OR the two values to form complete temperature reading, Actual Value is in Two's Complement Form
    tempValue += interValue;
  }
  else return -1;
  
  if(tempValue & 0x0800){
    // it is negative
    tempValue &= ~0x0800; // get rid of - bit
    tempValue = -tempValue;
  }
  
  return tempValue;
  /*
  //if(0xF7FF & tempValue)
  //  tempValue |= 0xF000;                 //Bring it to the real two's complement form

  delay(50);  
  //return tempValue;
  double T = tempValue;
  Logger.println();
  debug(T);
  // v = 16T + .15
  // T = (V - .15) / 16
  //T = T * 25.0;
  //T = T / 400.0;
  T = (T - .15) / 16.0;
  
  return T;
  */
}


// ****************************************************************************************8
// ************   MAIN FUNCTIONS

void setup()
{
  Serial.begin(115200);                          // Initialize serial communication with the Bluetooth module
  //dSerial.begin(19200);
  //L_config_soft(&dSerial);
  
  Wire.begin();                                  // Initialize I2C Bus
  
  pinMode(LED_CONTROL_PIN,OUTPUT);
  pinMode(EXT_CO_PIN,INPUT);
  pinMode(NO_CONTROL_PIN,OUTPUT);
  pinMode(CO_CONTROL_PIN,OUTPUT);
  pinMode(PGGOOD_PIN,INPUT);
  pinMode(NO_SENSE_PIN,INPUT);
  pinMode(VOC_SENSE_ON_PIN,OUTPUT);
  pinMode(VOC_SENSE_PIN,INPUT);
  pinMode(BAT_SENSE_PIN,INPUT);
  pinMode(CO_NO_CONTROL_PIN,OUTPUT);
  pinMode(OZONE_CONTROL_PIN,OUTPUT);
  pinMode(CO_SENSE_PIN,INPUT);
  pinMode(OZONE_SENSE_PIN,INPUT);
  pinMode(FAN_CONTROL_PIN, OUTPUT);
  
  digitalWrite(LED_CONTROL_PIN, HIGH);      //Initialize LED to OFF state, the rest should be on
  
  LightInit();  
  
  TempSensorsInit_TI();
}

void write_data(){
  /*
  CO2     ppm
  CO      ppb
  VOC     ppb
  03      ppb
  NO      ppb
  light   raw
  */
  float tempC;
  //Display of humidity
  Serial.print(ReadHumiditySensor());
  Serial.println(F(";InsertSensorPackageName;SHT21-H;Humidity;RH;response indicator;RI;0;1000;2000;3000;4096"));
  
  Serial.print(ReadTempSensor());
  //Serial.print(42);
  Serial.println(F(";InsertSensorPackageName;SHT21-T;Temperature;Temp;response indicator;RI;0;1000;2000;3000;4096"));

  Serial.print(ReadTempSensor_TI());
  Serial.println(F(";InsertSensorPackageName;TMP175;Temperature;Temp;response indicator;RI;0;1000;2000;3000;4096"));
  
  Serial.print(ReadCO2Sensor());
  Serial.println(F(";InsertSensorPackageName;S100;CO2 Gas;CO;;RI;0;1250;2500;3750;5000"));
  
  Serial.print(analogRead(CO_SENSE_PIN));
  Serial.println(F(";InsertSensorPackageName;MiCS-5525;CO Gas;CO;Analog Value;AV;0;250;500;750;1000"));
  
  Serial.print(analogRead(NO_SENSE_PIN));
  Serial.println(F(";InsertSensorPackageName;MiCS-2710;N02 Gas;NO2;Analog Value;AV;0;250;500;750;1000"));
  
  Serial.print(analogRead(OZONE_SENSE_PIN));
  Serial.println(F(";InsertSensorPackageName;MiCS-2611;Ozone;O3;Analog Value;AV;0;250;500;750;1000"));
  
  Serial.print(analogRead(VOC_SENSE_PIN));
  Serial.println(F(";InsertSensorPackageName;MiCS-5121;Volotile Organic Compounds;VOC;Analog Value;AV;0;250;500;750;1000"));
}

//int test_count = 0;
void loop()
{ 
  /*
  float out;
  String outs;
  
  uint16_t time = millis();
  dSerial.print(F("M:"));
  dSerial.println(freeMemory());
  
  dSerial.print(F("Hum:"));
  dSerial.print(ReadHumiditySensor());
  dSerial.println("%");
  
  dSerial.print(F("CO2:"));
  dSerial.println(ReadCO2Sensor());
  
  dSerial.print(F("Light:"));
  dSerial.println(ReadLightSensor());
  
  dSerial.print(F("Temp:"));
  dSerial.println(ReadTempSensor());
  
  dSerial.print(F("Temp_T1:"));
  dSerial.println(ReadTempSensor_TI());

  dSerial.println();
  */
  write_data();
  delay(3000);
  
}



