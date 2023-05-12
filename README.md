# AgHolobox
An engineer and part-time hand on the family's farm - this is my hardware/firmware implementation for AgOpenGPS.

Heavily based on mtz8302's ESP-32 implementation --> https://github.com/mtz8302/AOG_Autosteer_ESP32 whose work I found before I knew very much about this.  I'm glad that I did!  The ESP-32 with its' high-speed dual core architecture just made more sense to me than the more well worn Teensy path taken by most.

The flexibility of the software stack has been reduced considerably here:

 * UDP over WiFi
 * Bosch BNO085 as the IMU
 * On/Off switch as the Steer Enable
 * ADS1115 as the A/D convertor for the wheel angle sensor

### agPureAp 
for the ESP-32, the agPureAp functions solely as the WiFi access point.

### agRtkBase  & agRtkRover
for the *SparkFun LoRa Thing Plus(expLoRaBLE) & uBlock ZED-F9P* module in both the agRtkBase (fixed station) and agRtkRover (vehicle). 
The correction data is being sent from the base station to the vehicle over LoRa P2P link.  This is not the most cost effective solution here, but this was my first step so I went with what I could find good documentation on.  Thanks Sparkfun!

### agSteer
for the ESP-32. I've chopped out many of the options from mtz8302's original, primarily to make it easier for me to understand and modify.

### agIMU
for the *Particle Photon*.  The Bosch BNO085 is wildly at odds with the I2C spec and the ESP-32 doesn't support clock stretching.  And I had many lockups reading the BNO0xx devices with the ESP-32. 
All of that has pushed me to using the Particle Photon processor to read the IMU and send the data over the WiFi network.  

### agGnssBridge
for the ESP-32, this component receives data from the ZED-F9P over a serial link and sends it over the WiFi network.

###agDru
for Arduino UNO, this component processes the Steer Enable switch, monitors motor current, and activates relays to connect the driver (I'm using the Cytron)


