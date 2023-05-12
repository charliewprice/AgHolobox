# AgHolobox
An engineer and part-time hand on the family's farm - this is my hardware/firmware implementation for AgOpenGPS:

https://github.com/farmerbriantee/AgOpenGPS

https://discourse.agopengps.com/

Heavily based on mtz8302's ESP-32 implementation --> https://github.com/mtz8302/AOG_Autosteer_ESP32 whose work I found before I knew very much about this.  I'm glad that I did!  The ESP-32 with its' low-cost, built-in WiFi, and high-speed dual core architecture just made more sense to me than the more well worn Teensy path taken by most.

I've eliminated some of the features of mtz8302's software stack - primarily so I could better understand it:  The code in the AgSteer component of this repo uses:

 * UDP over WiFi
 * Bosch BNO085 as the IMU
 * On/Off switch as the Steer Enable
 * ADS1115 as the A/D convertor for the wheel angle sensor

### agPureAp 
for the ESP-32, the agPureAp functions solely as the WiFi access point.

### agRtkBase  & agRtkRover
for the *SparkFun LoRa Thing Plus(expLoRaBLE) & uBlock ZED-F9P* module in both the agRtkBase (fixed station) and agRtkRover (vehicle). 
The correction data is being sent from the base station to the vehicle over a LoRa P2P link.  This is not the most cost effective solution but this was my first step so I went with Sparkfun because of their very nice tutorial on RTK.  The base and rover have a minimal serial terminal interface over Bluetooth.  I use the Android Serial Terminal app from my phone to start the survey-in process, check on status of the fix, etc. 

Just fyi here, I've been impressed with the performance of the Signalplus 12dbi Omni-Directional 824-960MHZ Outdoor LoRa Antenna for the base station which travels around the farm in my vehicle.  I raise a short mast with the SignalPlus from the back of my vehicle, start the survey-in process, and within 30minutes I'm getting RTK fix in the tractor.  The rover side of LoRa uses a full length 915MHz dipole of the *rubber ducky* type.

### agSteer
for the ESP-32. I've chopped out many of the options from mtz8302's original, primarily to make it easier for me to understand and modify.

### agIMU (aka IMUCaster)
for the *Particle Photon*.  The Bosch BNO085 is wildly at odds with the I2C spec and the ESP-32 doesn't support clock stretching.  And I had many lockups reading the BNO0xx devices with the ESP-32. All of that has pushed me to using the Particle Photon processor to read the IMU and send the data over the WiFi network.  

### agGnssBridge
for the ESP-32, this component receives data from the ZED-F9P over a serial link and sends it over the WiFi network.

### agDru (aka Drive Response Unit)
for Arduino UNO, this component processes the Steer Enable switch, monitors motor current, and activates relays to connect the driver (I'm using the Cytron)

## Hardware Packaging
Isn't this the hardest piece? For now, there are three  pieces of hardware:

[Rover Unit](https://www.dropbox.com/s/5r3vjf7hzsla30m/agRover.png?dl=0 "Rover")

1. GNSS RTK Base Station
2. Rover (containing)
  * GNSS RTK mobile station
  * WiFi Access Point 
  * GNSS UDP Bridge   
  * IMUCaster
3. Motor Driver
  * Drive Response Unit
  * 12-24v Boost Convertor
  * Cytron MD30C Driver
