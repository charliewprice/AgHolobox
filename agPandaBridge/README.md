# agPandaBridge

This folder contains source files for the Particle Photon microcontroller which acts as the UDP bridge for the GPS stream coming from the ZED-F9P uBlox GNSS module.  This connection is over the UART2 of the ZED-F9P to the Serial1 (RX) pin of the Photon.
The BNO085 IMU is also connected to the Photon over the I2C interface.  The Photon connects to the WiFi network (RoverNet) which is hosted by the agPureAp module.

## $PANDA sentence

The Photon is digesting GGA and VTG sentences from the F9P and merging that data with measurements from the IMU.  When the Photon receives the GGA sentence it constructs and sends the custom $PANDA sentence over UDP to AOG using 192.168.1.255:9999.  This is the only sentence needed in AoG.

## Compiling

The files in this project are compiled with the Particle toolset as follows:

>particle compile photon --saveTo particle_panda.bin
