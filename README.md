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

### agPureAp (depr 6/23)
|  |  |
| ------------- | ------------- |
| <img src="/images/block-diagram_4.png" alt="BlockDiagram_4" title="Rover Unit" align="left" width=640 style="display: inline-block; margin: 0 auto;">  | AgOpenGPS interacts with the other components in the solution via UDP packets. The agPureAp acts as a cheap and stealthy WiFi access point for a WiFi network in the tractor's cab using the ESP-32 SoftAp library.|

The agPureAp function has been deprecated in favor of a standalone WiFi Router installed in the cab of the tractor. 


### agRtkBase
|  |  |
| ------------- | ------------- |
| <img src="/images/block-diagram_5.png" alt="BlockDiagram_5" title="GNSS Base" align="left" width=1080 style="display: inline-block; margin: 0 auto;">  | The GNSS Base Station uses a *SparkFun LoRa Thing Plus(expLoRaBLE) & uBlock ZED-F9P* module to acquire the GPS fix, and to generate and send RTCM correction data to the tractor over a point-to-point LoRa network.  A Bluetooth serial terminal interface is included to start and stop the survey-in procedure, configure the LoRa channel, check fix status, etc. I use the Android Serial Terminal app from my phone to control the base station.  Just fyi here, I've been impressed with the performance of the Signalplus 12dbi Omni-Directional 824-960MHZ Outdoor LoRa Antenna for the base station which travels around the farm in my vehicle.  I raise a short mast with the SignalPlus from the back of my vehicle, start the survey-in process, and within 30minutes I'm getting RTK fix in the tractor.|
### agRtkRover
|  |  |
| ------------- | ------------- |
| <img src="/images/block-diagram_1A.png" alt="BlockDiagram_1" title="GNSS Rover" align="left" width=640 style="display: inline-block; margin: 0 auto;">  | The GNSS Rover Unit also uses a *SparkFun LoRa Thing Plus(expLoRaBLE) & uBlock ZED-F9P* module to acquire the GPS fix, and to receive RTCM correction data from the base station.  A Bluetooth serial terminal interface is included to configure the LoRa channel, check fix status, etc. The corrected NMEA sentences (GGA, VTG) are streamed from the ZED-F9P over a high-speed serial (UART) connection to the GNSS UDP which broadcasts the packets over UDP network. The rover uses a full length 915MHz vertical dipole in a *rubber ducky* package.| 

The agPandaBridge is now contained in the GNSS Rover Unit where it receives GNSS data from the ZED-F9P and IMU data from the BNO085. These are combined in the new PANDA sentence which is sent via UDP. 

### agIMU (aka IMUCaster, depr 6/23)
|  |  |
| ------------- | ------------- |
| <img src="/images/block-diagram_2.png" alt="BlockDiagram_2" title="IMU" align="left" width=640 style="display: inline-block; margin: 0 auto;">  | The Bosch BNO085 is wildly at odds with the I2C spec and the ESP-32 doesn't support clock stretching.  I've seen many lockups reading the BNO0xx devices with the ESP-32. I've decided to locate the IMU on a separate processor.  For now I'm using a Particle Photon which has built-in WiFi.  At some point I want to receive the GGA messages here and use the IMU roll, pitch, heading, to adjust the coordinates and generate the custom $PANDA sentence which AgOpenGPS supports.  I'm fuzzy on all of that at present.

The agIMU component has been deprecated. the BNO085 is now with the agPandaBridge which is installed in the RtkRover package.

### agSteer
|  |  |
| ------------- | ------------- |
| <img src="/images/block-diagram_3.png" alt="BlockDiagram_3" title="Steer" align="left" width=640 style="display: inline-block; margin: 0 auto;">  | The steer unit interfaces with the wheel angle sensor, motor drive unit, and the AgOpenGPS application. In the driver unit is packaged separately I've chopped out many of the options from mtz8302's original, primarily to make it easier for me to understand and modify. |

### agDru (aka Drive Response Unit)
for Arduino UNO, this component processes the Steer Enable switch, monitors motor current, and activates relays to connect the driver (I'm using the Cytron) output to the motor terminals. This allows relatively free operator control of the wheel without having to mechanically disengage the gears.

### agPandaBridge
This component is managed by a Gen 1 Particle Photon MCU and receives GNSS data from the ZED-F9P over serial port (UART).  The BNO085 is connected via I2C.  GNSS and IMU data are combined in the new PANDA sentence and broadcast over UDP to the agOpenGPS application.

## Hardware Packaging
Isn't this the hardest piece? For now, there are three  pieces of hardware:

1. GNSS RTK Base Station
2. Rover (containing)
  * GNSS RTK Rover Station
  * ~~ WiFi Access Point ~~ 
  * ~~ GNSS UDP Bridge ~~   
  * ~~ IMUCaster ~~
  * Panda Bridge
3. Motor Driver
  * Drive Response Unit
  * 12-24v Boost Convertor
  * Cytron MD30C Driver

### The Rover in the John Deere 6420
|  |  |
| ------------- | ------------- |
| <img src="/images/agRover.png" alt="agRover" title="Rover Package" align="left" width=640 style="display: inline-block; margin: 0 auto;">  | I've mounted the package in the JD6420 on the left side window of the cab to keep the IMU in a stable location. It drops in from the top and is easily removed when needed.|There's a buzzer on the front panel for general use -- it beeps a few times on boot-up and whenever a parameter in the EEPROM is changed.   

## Drive System
I'm using a Phidget motor with a gear drive system similar to others.
