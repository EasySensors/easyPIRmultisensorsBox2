/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.  
 * 
**/

// Enable debug prints to serial monitor
#define MY_DEBUG

#include <avr/wdt.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif



// ------------------------------------------ The node settings start

// Define it if only the magnet sensor used and no PIR sensor installed
#define NO_PIR_SENSOR_INSTALLED


// Comment it out for Auto Node ID #
#define MY_NODE_ID 217 // 7D

int relayNodeIDPIRSensor  = 0; // Relay addressess to send switch ON\OFF states. Can be any address; 0 is SmartHome controller address.
int relayNodeIDmagSensor  = 0; // Relay addressess to send switch ON\OFF states. Can be any address; 0 is SmartHome controller address.

// Uncomment either one. dpending on the Temperature and Humidity sensor installed on the board:
#define TEMP_HUM_SENSOR_SHTC3
//#define TEMP_HUM_SENSOR_Si7021

#if defined (TEMP_HUM_SENSOR_SHTC3) &&  defined (TEMP_HUM_SENSOR_Si7021)
#error Only one temperature and humidity sensor type can be activated
#endif


//  Lux light level threshold. if Lux level redings below  or equal LUXTHRESHOLD PIR_sensor will be reported to controller if motion detected
//  If Lux level redings above LUXTHRESHOLD PIR_aboveLUXthreshold_sensor will be reported to controller if motion detected
//  PIR_aboveLUXthreshold_sensor is useful when lights set to ON but you still need motion detection.

#define LUXTHRESHOLD 0 //  0xFFFF = always report PIR triggered sensor

//  it only sends Humidity report when there has been some changes in humidity. If changes exceeds set % value the node will report it.
#define HUMIDITY_HYSTERESIS_PERCENTS 10


// Define miliseconds interval between desired sensors report times. Set 0 to report each PIRor  door sensor triggered wakeup. 
#define SENSORS_REPORT_TIME_MS 0 //  0 sleep forever 1hr = 3600000 5hrs = 18000000


// Assign numbers for all sensors to be reported to the gateway\controller (they will be created as child devices)

#define PIR_sensor 1  
#define PIR_aboveLUXthreshold_sensor 11 
#define MAG_sensor 2  
#define HUM_sensor 3 //3 If sensor # is 0 the sensor will not report any values to the controller
#define TEMP_sensor 4 //4 If sensor # is 0 the sensor will not report any values to the controller
#define VIS_sensor 5//5 If sensor # is 0 the sensor will not report any values to the controller


//#define MY_RFM69_NETWORKID 111

// Enable and select radio type attached MY_RADIO_RFM69
#define MY_RADIO_RFM69
#define MY_IS_RFM69HW
#define MY_RFM69_TX_POWER_DBM (13u)
#define  WAIT_RFM69_RETRY_TIMEOUT_MS 1000 //uint8_t retries=5, uint8_t retryWaitTime=200)


// if you use MySensors 2.0 use this style 
//#define MY_RFM69_FREQUENCY   RFM69_433MHZ
//#define MY_RFM69_FREQUENCY   RFM69_868MHZ
#define MY_RFM69_FREQUENCY   RFM69_915MHZ

// Enable and select radio type attached MY_RADIO_RFM95
//#define MY_RADIO_RFM95
//#define MY_RFM95_MODEM_CONFIGRUATION  RFM95_BW125CR45SF128
#define MY_RFM95_MODEM_CONFIGRUATION RFM95_BW_500KHZ | RFM95_CODING_RATE_4_5, RFM95_SPREADING_FACTOR_2048CPS | RFM95_RX_PAYLOAD_CRC_ON, RFM95_AGC_AUTO_ON // 
#define MY_RFM95_TX_POWER_DBM (14u)
//#define MY_RFM95_MODEM_CONFIGRUATION RFM95_BW125CR48SF4096
//RFM95_BW125CR45SF128

// air-time approximation for timeout, 1 hop ~15 bytes payload - adjust if needed
// BW125/SF128: 50ms
// BW500/SF128: 15ms
// BW31.25/SF512: 900ms
// BW125/SF4096: 1500ms

#define RFM95_RETRY_TIMEOUT_MS      (2500ul)      //!< Timeout for ACK, adjustments needed if modem configuration changed (air time different)

//#define   MY_RFM95_FREQUENCY RFM95_915MHZ
//#define   MY_RFM95_FREQUENCY RFM95_868MHZ
//#define   MY_RFM95_FREQUENCY RFM95_433MHZ


// Avoid battery drain if Gateway disconnected and the node sends more than MY_TRANSPORT_STATE_RETRIES times message.
#define MY_TRANSPORT_UPLINK_CHECK_DISABLED
#define MY_PARENT_NODE_IS_STATIC
#define MY_PARENT_NODE_ID  0x0




//Enable Crypto Authentication to secure the node
//#define MY_SIGNING_ATSHA204
//#define  MY_SIGNING_REQUEST_SIGNATURES


// ------------------------------------------ The node settings end


// Written by Christopher Laws, March, 2013.
// https://github.com/claws/BH1750
#include <BH1750.h>
BH1750 lightMeter;



#ifdef  TEMP_HUM_SENSOR_Si7021
  #include "SparkFun_Si7021_Breakout_Library.h"
  //Create Instance of HTU21D or SI7021 temp and humidity sensor and MPL3115A2 barrometric sensor
  Weather sensor;
#endif
 
#ifdef   TEMP_HUM_SENSOR_SHTC3
  #include "SparkFun_SHTC3.h" // Click here to get the library: http://librarymanager/All#SparkFun_SHTC3
  SHTC3 SHTC3;              // Declare an instance of the SHTC3 class
#endif

#include <MySensors.h>

// Redefining write codes for JDEC FLASH used in the node
// These two defines should always be after #include <MySensors.h> declaration
#define SPIFLASH_BLOCKERASE_32K   0xD8
#define SPIFLASH_CHIPERASE        0x60

#include <stdlib.h>

#define MAGNET_PIN 3
#define PIR_PIN 7
#define RED_LED_PIN 6
#define GREEN_LED_PIN 5


uint16_t LUXthreshold;
  

// Create MyMessage Instance for sending readins from sensors to gateway\controller (they will be created as child devices)

MyMessage msg_PIR(PIR_sensor, V_LIGHT);
MyMessage msg_PIR_aboveLUXthreshold(PIR_aboveLUXthreshold_sensor, V_LIGHT);
MyMessage msg_mag(MAG_sensor, V_LIGHT);
MyMessage msg_hum(HUM_sensor, V_HUM);
MyMessage msg_temp(TEMP_sensor, V_TEMP);
MyMessage msg_vis(VIS_sensor, V_LEVEL); //V_LIGHT_LEVEL


int BATTERY_SENSE_PIN = A6;  // select the input pin for the battery sense point

uint16_t oldLux = 0, lux;
int16_t oldhumidity = 1, humidity;
int16_t oldTemp = 0, temp;
uint8_t batteryPcnt, oldBatteryPcnt = 0; 


volatile bool flagIntPIR = false, flagIntMagnet = false;
uint8_t prevoiusMagnetPinValue = 0;




//#define G_VALUE 16380
//#define G_VALUE2 268304400 //G_VALUE * G_VALUE

void pinsIntEnable(){  
  //   Pcint 23 d7 D0-D7 = PCINT 16-23 = PCIR2 = PD = PCIE2 = pcmsk2
  // PCINT23 (PCMSK2 / PCIF2 / PCIE2)
  
  PCMSK2 |= bit (PCINT23);
  PCIFR  |= bit (PCIF2);   // clear any outstanding interrupts
  PCICR  |= bit (PCIE2);   // Pcint 23 d7 D0-D7 = PCINT 16-23 = PCIR2 = PD = PCIE2 = pcmsk2
}

ISR (PCINT2_vect){
  flagIntPIR = true;
}

void magnetSensorInterruptHandler(){
  flagIntMagnet = true; 
}

void blinkSensorLed(int  i){
  batteryPcnt > 10 ?  blinkGreenSensorLed(i): blinkRedSensorLed(i);
}

void blinkGreenSensorLed(int  i){
  //return;
  for (;i>0;i--){
    digitalWrite(GREEN_LED_PIN, HIGH);
    wait(50);
    digitalWrite(GREEN_LED_PIN, LOW);
    if (i > 1) {wait(50);}
  }
}
void blinkRedSensorLed(int  i){
  for (;i>0;i--){
  digitalWrite(RED_LED_PIN, HIGH);
  wait(50);
  digitalWrite(RED_LED_PIN, LOW);
  if (i > 1) {wait(50);}
  }
}




int  batteryLevelRead(){
  

  // Get the battery Voltage
  int sensorValue = analogRead(BATTERY_SENSE_PIN);
  /* 1M, 470K divider across batteries
   *  Vsource = Vout * R2 / (R2+R1)   = 7,383 * Vout;
   *  we use internal refference voltage of 1.1 Volts. Means 1023 Analg Input values  = 1.1Volts
   *  2 v for batteries in series is close to dead bateries. analog Read value for 2 V is  595.  100% = 960 or more 
   *  something in between is working range.
   */
  float batteryPcntValue = (sensorValue - 595)  / 3.64;

  #ifdef MY_DEBUG
    Serial.print("battery sensorValue   : ");  Serial.println(sensorValue);
  #endif
  
  if (batteryPcntValue < 0 ) {
    // Cut down negative values. Just in case the battery goes below 4V and the node still working. 
    batteryPcnt = 0; }
  else if (batteryPcntValue > 100){
     // Cut down more than "100%" values. In case of ADC fluctuations. 
     batteryPcnt = 100; }
  else  batteryPcnt = (int)batteryPcntValue;   

  // check if BATTERY_SENSE_PIN readings close to 0 (external power source). 
  return  batteryPcntValue < 100 ? 100 : (int) batteryPcntValue;  
}


void batteryReport(){
  if ( (abs(oldBatteryPcnt - batteryPcnt) > 5 ) || batteryPcnt == 0 ) {
    sendBatteryLevel(batteryPcnt);
    // this wait(); is 2.0 and up RFM69 specific. Hope to get rid of it soon
    // TSF:MSG:SEND,238-238-0-0,s=255,c=3,t=0,pt=1,l=1,sg=0,ft=0,st=OK:100
   
    // wait for ACK signal up to RFM95_RETRY_TIMEOUT_MS or 50ms for rfm miliseconds
    #ifdef  MY_RADIO_RFM95
      wait(RFM95_RETRY_TIMEOUT_MS, 3, 0);
    #endif
    #ifdef  MY_RADIO_RFM69
     // waiting up to xxx millis ACK of type 2 message t=2
      wait(WAIT_RFM69_RETRY_TIMEOUT_MS, 3, 0);
    #endif
    oldBatteryPcnt = batteryPcnt;
  }
}


void lightReport()
{

  char visualLight[10];
 
  lightMeter.begin(BH1750::ONE_TIME_LOW_RES_MODE); // need for correct wake up
  lux = lightMeter.readLightLevel(true);// Get Lux value
  #ifdef MY_DEBUG
    Serial.print("visualLight LUX : ");  Serial.println(lux);
  #endif
  // dtostrf(); converts float into string
  dtostrf(lux,5,0,visualLight);
  
  // Sensor # is 0, no need to report anything
  if (0 != VIS_sensor) { //  If sensor # is 0 the sensor will not report any values to the controller 
    send(msg_vis.set(visualLight), true);  // Send LIGHT BH1750     sensor readings
    #ifdef  MY_RADIO_RFM95
      // TSF:MSG:SEND,209-209-0-0,s=5,c=1,t=37,pt=0,l=5,sg=0,ft=0,st=OK: 
      // waiting up to xxx millis ACK of type 37 message t=37
      wait(RFM95_RETRY_TIMEOUT_MS, 1, 37);
    #endif
    #ifdef  MY_RADIO_RFM69
      wait(WAIT_RFM69_RETRY_TIMEOUT_MS, 1, 37);
    #endif 

           
    oldLux = lux;
  }
}

void TempHumReport()
{
    char humidityStr[10];
    char tempStr[10];

  #ifdef  TEMP_HUM_SENSOR_Si7021
    // Measure Relative Humidity from the Si7021
    // Temperature is measured every time RH is requested.
    // It is faster, therefore, to read it from previous RH
    // measurement with getTemp() instead with readTemp()
    humidity = sensor.getRH();
    temp = sensor.getTemp();
  #endif    
  
  #ifdef  TEMP_HUM_SENSOR_SHTC3
    SHTC3_Status_TypeDef result = SHTC3.update(); 
    // Measure Relative Humidity from the SHTC3 
    humidity = SHTC3.toPercent();
    temp = SHTC3.toDegC();
   #endif
   
    dtostrf(humidity,0,2,humidityStr);
    dtostrf(temp,0,2,tempStr);  
   
    float oldhumidityPercValue =  (float)oldhumidity/100;
    float humidityDeltaPercents =   abs( (humidity - oldhumidity)/oldhumidityPercValue );
    #ifdef MY_DEBUG
          Serial.print("humidity  ");   Serial.println(humidity);
          Serial.print("oldhumidity  ");   Serial.println(oldhumidity);
          Serial.print("humidityDeltaPercents  ");   Serial.println(humidityDeltaPercents);
    #endif 
    
    if ( humidityDeltaPercents > HUMIDITY_HYSTERESIS_PERCENTS  && 0 != HUM_sensor) {  //  If sensor # is 0 the sensor will not report any values to the controller 
      send(msg_hum.set(humidityStr), true); // Send humiditySHTC3     sensor readings
      // this wait(); is 2.0 and up RFM69 specific. Hope to get rid of it soon
      // TSF:MSG:READ,0-0-209,s=4,c=1,t=0,pt=0,l=5,sg=0:22.00
      // waiting up to xxx millis ACK of type 0 message t=0
      #ifdef  MY_RADIO_RFM95
        wait(RFM95_RETRY_TIMEOUT_MS, 1, 0);
      #endif
      #ifdef  MY_RADIO_RFM69
       // waiting up to xxx millis ACK of type 0 message t=0
        wait(WAIT_RFM69_RETRY_TIMEOUT_MS, 1, 0);
      #endif
      oldhumidity = humidity; 
    }
  


    if (temp != oldTemp && 0 != TEMP_sensor) { //  If sensor # is 0 the sensor will not report any values to the controller 
      send(msg_temp.set(tempStr), true); // Send tempSi7021 temp sensor readings
      // this wait(); is 2.0 and up RFM69 specific. Hope to get rid of it soon
      // TSF:MSG:READ,0-0-209,s=3,c=1,t=1,pt=0,l=5,sg=0:34.00
      // waiting up to xxx millis ACK of type 1 message t=0
      #ifdef  MY_RADIO_RFM95
        wait(RFM95_RETRY_TIMEOUT_MS, 1, 0);
      #endif
      #ifdef  MY_RADIO_RFM69
       // waiting up to xxx millis ACK of type 2 message t=0
        wait(WAIT_RFM69_RETRY_TIMEOUT_MS, 1, 0);
      #endif 
      oldTemp = temp;
   }
 
}


void before() {
    //No need watch dog enabled in case of battery power.
    //wdt_enable(WDTO_4S);
    wdt_disable();

    #ifdef  TEMP_HUM_SENSOR_SHTC3
      errorDecoder(SHTC3.begin());  // SHTC3 start
      SHTC3.sleep(true);
    #endif

    Serial.println();  
    
    #ifdef NO_PIR_SENSOR_INSTALLED
      Serial.println("The node is configured as Door sensor node. If there is PIR sensor soldered, please comment out line #define NO_PIR_SENSOR_INSTALLED ");  
      pinMode(PIR_PIN, INPUT_PULLUP);
      digitalWrite(PIR_PIN,HIGH);
    #else if
      Serial.println("The node is configured as PIR sensor node. If there is no PIR sensor soldered, please define #define NO_PIR_SENSOR_INSTALLED ");  
    #endif
    
    #ifdef MY_RADIO_RFM69
      Serial.println(F("The node is RFM69 configured."));
      Serial.print(F("MY_RFM69_NETWORKID: "));  Serial.println(MY_RFM69_NETWORKID);
      Serial.print(F("MY_RFM69_FREQUENCY: ")); 
      if (MY_RFM69_FREQUENCY == RFM69_433MHZ)   Serial.println(F("RFM69_433MHZ"));
      else if (MY_RFM69_FREQUENCY == RFM69_915MHZ)  Serial.println(F("RFM69_915MHZ"));
      else Serial.println(F("RFM69_868MHZ"));
          
      Serial.print(F("MY_RFM69_TX_POWER_DBM "));  Serial.println(MY_RFM69_TX_POWER_DBM);
      #ifdef MY_IS_RFM69HW
        Serial.println(F("RFM69HCW selected."));
      #endif
      
    #endif    

    #ifdef MY_RADIO_RFM95
      Serial.println(F("The node is RFM65 configured."));
      Serial.print(F("MY_RFM95_FREQUENCY: "));  Serial.println(MY_RFM95_FREQUENCY);
      Serial.print(F("RFM95_RETRY_TIMEOUT_MS: "));  Serial.println(RFM95_RETRY_TIMEOUT_MS);  
      Serial.print(F("MY_RFM95_TX_POWER_DBM "));  Serial.println(MY_RFM95_TX_POWER_DBM);
      Serial.print(F("MY_RFM95_MODEM_CONFIGRUATION "));  Serial.println(MY_RFM95_MODEM_CONFIGRUATION);
    #endif
    

    Serial.print(F("MY_NODE_ID Hex: "));  Serial.println(MY_NODE_ID,HEX);
    Serial.print(F("The node will send PIR sensor statuses to address Hex: "));  Serial.println(relayNodeIDPIRSensor,HEX);
    Serial.print(F("The node will send Magnet  sensor statuses to address Hex: "));  Serial.println(relayNodeIDmagSensor,HEX);

    Serial.print(F("LUXthreshold: "));  Serial.println(LUXthreshold); 
    Serial.print(F("PIR_sensor #: "));  Serial.println(PIR_sensor);
    Serial.print(F("PIR_aboveLUXthreshold_sensor #: "));  Serial.println(PIR_aboveLUXthreshold_sensor);
    Serial.print(F("MAG_sensor #: "));  Serial.println(MAG_sensor); 
    Serial.print(F("HUM_sensor #: "));  Serial.println(HUM_sensor); 
    Serial.print(F("TEMP_sensor #: "));  Serial.println(TEMP_sensor); 
    Serial.print(F("VIS_sensor #: "));  Serial.println(VIS_sensor); 
    Serial.print(F("Node sensor's report interval miliseconds: "));  Serial.println(SENSORS_REPORT_TIME_MS); ;
    Serial.print(F("Humidity report histeresys %: "));  Serial.println(HUMIDITY_HYSTERESIS_PERCENTS); 
    Serial.println();  
    
    
    analogReference(INTERNAL); //     DEFAULT

    
    //reading unused pin - init ADC 
    analogRead(A7);

  
    /*  RFM reset pin is 9
     *  A manual reset of the RFM69HCW\CW is possible even for applications in which VDD cannot be physically disconnected.
     *  Pin RESET should be pulled high for a hundred microseconds, and then released. The user should then wait for 5 ms
     *  before using the module.
     */

    pinMode(9, INPUT);
    delay(10);
    pinMode(9, OUTPUT);
    //reset RFM module
    digitalWrite(9, 1);
    delay(1);
    digitalWrite(9, 0);
    // set Pin 9 to high impedance
    pinMode(9, INPUT);
    delay(10);

  
    pinMode(RED_LED_PIN, OUTPUT);
    digitalWrite(RED_LED_PIN,0);
    pinMode(GREEN_LED_PIN, OUTPUT);
    digitalWrite(GREEN_LED_PIN,0);

    if (batteryLevelRead() < 1)  {
     // The battery level is critical. To avoid rebooting and flooding radio with presentation messages
     // turn the device into hwSleep mode.
  
     #ifdef MY_DEBUG
        Serial.println("Voltage is way below 2 Volts. Halting boot"); 
     #endif
     
     blinkSensorLed(10);
     hwSleep2(0, &flagIntPIR, &flagIntMagnet); 
    }
   

    lightMeter.begin(BH1750::ONE_TIME_LOW_RES_MODE);
    lightMeter.readLightLevel();// Get Lux value
    
    pinsIntEnable();     
    attachInterrupt(digitalPinToInterrupt(MAGNET_PIN), magnetSensorInterruptHandler, CHANGE);

    LUXthreshold = LUXTHRESHOLD;
    blinkGreenSensorLed(2);

    
    
}

void setup() {

}

void presentation() 
{  
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("PIR node", "1.0");

  // Register all sensors to gw (they will be created as child devices)
  present(PIR_sensor, S_BINARY);
  present(PIR_aboveLUXthreshold_sensor, S_BINARY);
  present(MAG_sensor, S_BINARY);
  if (0 != HUM_sensor) present(HUM_sensor, S_HUM);
  if (0 != TEMP_sensor) present(TEMP_sensor, S_TEMP);
  if (0 != VIS_sensor) present(VIS_sensor, S_LIGHT_LEVEL);
}

int PIRValue = 0, PIR_aboveLUXthresholdValue = 0, MagSensorValue = 0;

void loop()
{
  int sendStatus;
  uint8_t magnetPinValue = digitalRead(MAGNET_PIN);
  bool PIRPinValue; 

  batteryLevelRead();
  if ( flagIntPIR && digitalRead(PIR_PIN) == HIGH )  {
      flagIntPIR = false;
      // uncomment if you like report Light sensor reading before reporting PIR sensror
      // lightReport();
      msg_PIR.setDestination(relayNodeIDPIRSensor); 
      lightMeter.begin(BH1750::ONE_TIME_LOW_RES_MODE); // need for correct wake up
      if (lightMeter.readLightLevel(true) <= LUXthreshold){
        sendStatus =  send(msg_PIR.set(PIRValue),true);
        if (!sendStatus) {
         hwSleep2(5000, &flagIntPIR, &flagIntMagnet);  
         if (flagIntMagnet ) magnetPinValue = digitalRead(MAGNET_PIN); // saving the value if wake up happend after magnet sensor trigger 
         sendStatus =  send(msg_PIR.set(PIRValue),true); //Resendig PIR event anyways
        }
        PIRValue ?  PIRValue  = 0: PIRValue = 1;  // inverting the value each time
      } else {
        sendStatus =  send(msg_PIR_aboveLUXthreshold.set(PIR_aboveLUXthresholdValue),true);
        if (!sendStatus) {
         hwSleep2(5000, &flagIntPIR, &flagIntMagnet);
         if (flagIntMagnet ) magnetPinValue = digitalRead(MAGNET_PIN); // saving the value if wake up happend after magnet sensor trigger 
         sendStatus =  send(msg_PIR_aboveLUXthreshold.set(PIR_aboveLUXthresholdValue),true); //Resendig PIR event anyways
        }        
        PIR_aboveLUXthresholdValue ?  PIR_aboveLUXthresholdValue  = 0: PIR_aboveLUXthresholdValue = 1;  // inverting the value each time
      }
      // wait for ACK signal up to RFM95_RETRY_TIMEOUT_MS or 50ms for rfm miliseconds
      #ifdef  MY_RADIO_RFM95
        wait(RFM95_RETRY_TIMEOUT_MS, 1, 2);
      #endif
      #ifdef  MY_RADIO_RFM69
       // TSF:MSG:READ,0-0-209,s=1,c=1,t=2,pt=2,l=2,sg=0:1
       // waiting up to xxx millis ACK of type 2 message t=2
      wait(WAIT_RFM69_RETRY_TIMEOUT_MS, 1, 2);
      #endif
      // Blink  respective LED's once if message delivered to controller. 3 times if failed
      if (sendStatus) {
         blinkSensorLed(1);
      } else {
         blinkSensorLed(3);
      }

  } else if (flagIntPIR) {flagIntPIR = false;}

  
  if ( flagIntMagnet && magnetPinValue != prevoiusMagnetPinValue )  {
      flagIntMagnet  = false;
      prevoiusMagnetPinValue = magnetPinValue;
      //MagSensorValue ?  MagSensorValue  = 0: MagSensorValue = 1;  // inverting the value each time
      MagSensorValue = digitalRead(MAGNET_PIN);
      msg_mag.setDestination(relayNodeIDmagSensor); 
      // Blink  respective LED's once if message delivered to controller. 3 times if failed
      sendStatus =  send(msg_mag.set(MagSensorValue),true) ;
      if (!sendStatus) {
       hwSleep2(5000, &flagIntPIR, &flagIntMagnet);
       flagIntMagnet  = false;
       if (digitalRead(PIR_PIN) == HIGH ) PIRPinValue = HIGH; // saving the value if wake up happend after magnet sensor trigger 
       sendStatus =  send(msg_mag.set(MagSensorValue),true); //Resendig MagSensor event anyways
      }              
      // wait for ACK signal up to RFM95_RETRY_TIMEOUT_MS or 50ms for rfm miliseconds
      // this wait(); is 2.0 and up RFM69 specific. Hope to get rid of it soon
      #ifdef  MY_RADIO_RFM95
       // TSF:MSG:READ,0-0-209,s=1,c=1,t=2,pt=2,l=2,sg=0:1
       // waiting up to xxx millis ACK of type 2 message t=2
        wait(RFM95_RETRY_TIMEOUT_MS, 1, 2);
      #endif
      #ifdef  MY_RADIO_RFM69
       // TSF:MSG:READ,0-0-209,s=1,c=1,t=2,pt=2,l=2,sg=0:1
       // waiting up to xxx millis ACK of type 2 message t=2
       wait(WAIT_RFM69_RETRY_TIMEOUT_MS, 1, 2);
      #endif
      
      if (sendStatus) {
         blinkSensorLed(1);
      } else {
         blinkSensorLed(3);
      }

  }  else if (flagIntMagnet){flagIntMagnet = false;}


  // Report all sensors
  lightReport(); 
  TempHumReport();
  batteryReport();
  



  if ( !flagIntMagnet && (!flagIntPIR || (flagIntPIR && digitalRead(PIR_PIN) == LOW )) ){ // make sure no changes in PIR and magnet sensor happened while we were sending reports. 
      // hwSleep2 is rework of MySensors native sleep function to be able to sleep for some time and  wakeup on interrups CHANGE 
      // Doesnot support smart sleep, FOTA nor TransportReady. just sleeps miliseconds provided with first parameter.
      //sleep(0); 
      hwSleep2(SENSORS_REPORT_TIME_MS, &flagIntPIR, &flagIntMagnet); 
  }
}



uint32_t hwInternalSleep2(uint32_t ms, volatile bool *flag1, volatile bool *flag2  )
{
  // Sleeping with watchdog only supports multiples of 16ms.
  // Round up to next multiple of 16ms, to assure we sleep at least the
  // requested amount of time. Sleep of 0ms will not sleep at all!
  ms += 15u;


  while (!*flag1 && !*flag2 && ms >= 16) {
    for (uint8_t period = 9u; ; --period) {
      const uint16_t comparatorMS = 1 << (period + 4);
      if ( ms >= comparatorMS) {
        hwPowerDown(period); // 8192ms => 9, 16ms => 0
        ms -= comparatorMS;
        break;
      }
    }
  }

  
  if ( *flag1 or *flag2 ) {
    return ms;
  }
  return 0ul;
}

int8_t hwSleep2(uint32_t ms, volatile bool* flag1, volatile bool* flag2)
{
  transportDisable();
  // Return what woke the mcu.
  // Default: no interrupt triggered, timer wake up
  int8_t ret = MY_WAKE_UP_BY_TIMER;
  sleepRemainingMs = 0ul;
  if (ms > 0u) {
    // sleep for defined time
    sleepRemainingMs = hwInternalSleep2(ms, flag1, flag2);
  } else {
    // sleep until ext interrupt triggered
    hwPowerDown(WDTO_SLEEP_FOREVER);
  }
  transportReInitialise();
  return 0;
}



#ifdef  TEMP_HUM_SENSOR_SHTC3

  void errorDecoder(SHTC3_Status_TypeDef message)                             // The errorDecoder function prints "SHTC3_Status_TypeDef" resultsin a human-friendly way
  {
    switch(message)
    {
      case SHTC3_Status_Nominal : Serial.println("SHTC3 Status Nominal"); break;
      case SHTC3_Status_Error : Serial.println("SHTC3 Status Error"); break;
      case SHTC3_Status_CRC_Fail : Serial.println("SHTC3 Status CRC Fail"); break;
      default : Serial.println("SHTC3 Status Unknown return code"); break;
    }
  }
#endif
