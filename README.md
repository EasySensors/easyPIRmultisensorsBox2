![Arduino PIR](https://github.com/EasySensors/easyPIRmultisensorsBox2/blob/master/pics/Easy_PIR_MAIN.jpg?raw=true)
![Arduino PIR](https://github.com/EasySensors/easyPIRmultisensorsBox2/blob/master/pics/Easy_PIR_PCB_TOP.jpg?raw=true)
![Arduino PIR](https://github.com/EasySensors/easyPIRmultisensorsBox2/blob/master/pics/Easy_PIR_BOTTOM.jpg?raw=true)


**The easyPIRmultisensorsBox is a low cost wireless Arduino IDE compatible (the Atmel ATMega328P 8MHz) microcontroller with  RFM 69 CW or RFM 69 HCW or RFM 95 LoRa  radios on board and few other nice additions.** 
------------------------------------------------------------------------

Best sutable for Home Automation, IOT.  You may think of it as Arduino Pro Mini plus all the items in the picture below:

![Arduino PIR](https://github.com/EasySensors/easyPIRmultisensorsBox2/blob/master/pics/replcePIR2.jpg?raw=true)

## Specification: ##

 - MCU Atmel ATMega328P 8MHz) microcontroller
 - Radio- HopeRF RFM 69 CW or RFM 69 HCW or RFM 95 LoRa  (915, 868 0r 433 MHz) radio on board
 - Enclosure dimensions 95mm*40mm*20mm 
 - Powered by two AA batteries
 - Booster converter alows to work from batteries drained as low as 0.7V combined
 - Wide operating temperature range. Tested -20 +40 Celsius
 - PIR sensor Am312 
 - Temperature and humidity sensor Si7021 
 - High Accuracy Temperature Sensor ±0.4 °C (max), –10 to 85 °C
 - Precision Relative Humidity Sensor ± 3% RH (max), 0–80% RH
 - Light sensor BH1750,  spectral responsibility is approximately human eye response.
 - Authentication security - Atmel ATSHA204A Crypto Authentication Chip
 - Dualoptiboot bootloader. Implements over the air (OTA) firmware update ability
 - FTDI  header for programming
 - Reverse polarity protection.
 - Arduino pins A0 A1 have pads on PCB for any special needs.
 - PIR triggers hardware interrupt 1

**Pin out:** 


Arduino Pins|	Description
------------|--------------
A0, A1 |	Available ARDUINO analog GPIO / DIGITAL GPIO as PCB pads close to radio module
A6 |	Connected to Battery voltage sensor (via divider) 3M/470k 
A4 |	Connected to sensors i2c
A5 |	Connected to sensors i2c
A3 |	Connected to  ATSHA204A
D3 | Connected to  PIR sensor
D4 | Connected to  LED connected
D5 | Connected to  LED connected
D8 |	Connected to CS FLASH chip (OTA) M25P40
D2 |	Connected to RFM 69 DIO0 
D9 | Connected to RFM 69 Reset pin 
D10 |	Connected to RFM 69 CS/NSS
D11 |	Connected to  MOSI
D12 |	Connected to  MISO
D13 |	Connected to  SCK


**Arduino IDE Settings**

![Arduino IDE Settings](https://github.com/EasySensors/ButtonSizeNode/blob/master/pics/IDEsettings.jpg?raw=true)


**programming FTDI adapter connection**

![enter image description here](https://github.com/EasySensors/ButtonSizeNode/blob/master/pics/FTDIvcc5-3.jpg?raw=true)


How to use it as home automation (IOT) node controller
------------------------------------------------------

easyPIRmultisensorsBox.ino is the Arduino example sketch using [MySensors](https://www.mysensors.org/) API. 


Connect the Node to FTDI USB adaptor, Select Pro Mini 8MHz board in Arduino IDE and upload the sketch.
Connect the Node to FTDI USB adaptor, Select Pro Mini 8MHz board in Arduino IDE and upload the easyPIRmultisensorsBoxWithTempHumSensor.ino. The skecth will create node fith fixed address in Mysensors network.



**Done**

[Schematics](https://github.com/EasySensors/easyPIRmultisensorsBox2/blob/master/Schematics_pir_magnet_window_sensor.PDF)

The board designed by  [Koresh](https://www.openhardware.io/user/143/projects/Koresh)

