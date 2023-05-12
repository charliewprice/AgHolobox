/*
  Using the BNO080 IMU
  By: Nathan Seidle
  SparkFun Electronics
  Date: December 21st, 2017
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14586

  This example shows how to output the parts of the magnetometer.

  It takes about 1ms at 400kHz I2C to read a record from the sensor, but we are polling the sensor continually
  between updates from the sensor. Use the interrupt pin on the BNO080 breakout to avoid polling.

  Hardware Connections:
  Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the sensor onto the shield
  Serial.print it out at 9600 baud to serial monitor.
*/
#include "application.h"
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"

SYSTEM_MODE(MANUAL);
SYSTEM_THREAD(ENABLED);

#define TEST
#if defined(TEST)
uint16_t packetCount = 0; 
#endif

#define CONST_180_DIVIDED_BY_PI 57.2957795130823    // 180/pi

UDP Udp;

//byte buffer[] = {1,2,3,4,5,6};
//int remPort = 9999;
uint16_t portDestination = 9990;  
//IPAddress remServer(192,168,1,255);
static uint8_t ipDestination[] = {192,168,1,255};

//hello packet that we'll send
//IMU Reply 	                     121  121  5 * 	* 	* 	* 	* 	CRC
uint8_t helloFromIMU[] = { 128, 129, 121, 121, 5, 0, 0, 0, 0, 0, 71 };
//loop time variables in microseconds  
const uint16_t LOOP_TIME = 100;  //10Hz    
uint32_t lastTime = LOOP_TIME;
uint32_t currentTime = LOOP_TIME;

BNO080 bno08x;

float bno08xHeading = 0;
double bno08xRoll = 0, bno08xPitch = 0;
int16_t bno08xHeading10x = 0, bno08xRoll10x = 0, bno08xPitch10x = 0;

//the default network address
struct ConfigIP {
        uint8_t ipOne = 192;
        uint8_t ipTwo = 168;
        uint8_t ipThree = 1;
};  ConfigIP networkAddress;   //4 bytes

static uint8_t myip[] = { 0,0,0,0 };
unsigned int localPort = 8888;
uint16_t portMy = 5121; 

uint8_t data[] = {128, 129, 121, 211, 8, 0,0,0,0, 0,0,0,0, 15};
int16_t dataSize = sizeof(data); 

//Program counter reset
void(*resetFunc) (void) = 0;
    
void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("BNO080/UDP IMUCaster");
  delay(5000);
  Wire.begin();
  Wire.setClock(400000); //Increase I2C data rate to 400kHz
  delay(50);
  boolean imuFound = false;
  do {
    imuFound = bno08x.begin();
    if(imuFound)
      Serial.println("BNO085 found!");      
    else
      Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    delay(1000);
  } while(!imuFound);  
    
  bno08x.enableRotationVector(50); //Send data update every 50ms

  Serial.println(F("Rotation vector enabled"));
  Serial.println(F("Output in form i, j, k, real, accuracy"));
  
  //WiFi.on();
  WiFi.connect();
  
  int connectTimeSeconds = 0;
  while(!WiFi.ready()) {
      Serial.print(".");
      delay(1000);
      connectTimeSeconds += 1;
      if (connectTimeSeconds==20) {
        Serial.println("Forcing a hardware reset in 2seconds...");
        delay(2000);
        System.reset();
      }
  }
  
  Serial.print("\nConnected to "); Serial.println(WiFi.SSID());
  Serial.print("\nIP = "); Serial.println(WiFi.localIP().toString().c_str());
  delay(2000); 
  
  myip[0] = networkAddress.ipOne;
  myip[1] = networkAddress.ipTwo;
  myip[2] = networkAddress.ipThree;
  myip[3] = WiFi.localIP()[3];
   
  Udp.begin(localPort);
  delay(500);
}

void sendUdp(uint8_t* data, uint16_t len, uint16_t srcPort, uint8_t* destIp, uint16_t destPort) {
      Udp.beginPacket(destIp, 9999);
      Udp.write(data,len);
      Udp.endPacket();
}

void parseAogPacket(byte* udpData, uint16_t len, IPAddress srcIp) {      
        /*
        for (int16_t i = 0; i < len; i++) {
          Serial.print(udpData[i],HEX); Serial.print("\t"); } Serial.println(len);
        */      
// Hello 			7F 	127 	C8 	200 	3 	Module ID 	0 	0 	CRC
// Scan Request		7B 	123 	CA 	202 	3 	202 	202 	5 	CRC
// Subnet Change 	7F 	127 	C9 	201 	5 	201 	201 	IP_One 	IP_Two 	IP_Three 	CRC
        if (udpData[0] == 0x80 && udpData[1] == 0x81) {
            if ((udpData[2] == 0x7F) && (udpData[3] == 200)) {
			  // Hello from AgIO
              //Serial.println("Hello from AgIO!");
              static uint8_t ipDest[] = { 255,255,255,255 };
              uint16_t portDest = 9999; //AOG port that listens
              sendUdp(helloFromIMU, sizeof(helloFromIMU), portMy, ipDest, portDest);			  
            } else if ((udpData[2] == 0x7F) && (udpData[3] == 201)) {
			  // Subnet Change			  
              if (udpData[4] == 5 && udpData[5] == 201 && udpData[6] == 201) {
				  Serial.println("Subnet Change");
                  networkAddress.ipOne = udpData[7];
                  networkAddress.ipTwo = udpData[8];
                  networkAddress.ipThree = udpData[9];
                  //save in EEPROM and restart
                  EEPROM.put(4, networkAddress);
                  resetFunc();
              }
            } else if ((udpData[2] == 0x7F) && (udpData[3] == 202)) {
                // Scan Request
				if (udpData[4] == 3 && udpData[5] == 202 && udpData[6] == 202) {
					//Serial.println("Scan Request");					
					uint8_t scanReply[] = { 128, 129, 121, 203, 7, 
                        myip[0], myip[1], myip[2], myip[3], 
                        srcIp[0], srcIp[1], srcIp[2], 23};
                    // calculate the checksum
                    int16_t CK_A = 0;
                    for (uint8_t i = 2; i < sizeof(scanReply) - 1; i++) {
                      CK_A = (CK_A + scanReply[i]);
                    }
                    scanReply[sizeof(scanReply)-1] = CK_A;

                    static uint8_t ipDest[] = { 255,255,255,255 };
                    uint16_t portDest = 9999; //AOG port that listens
                    //off to AOG
                    sendUdp(scanReply, sizeof(scanReply), portMy, ipDest, portDest);
					//sendUdp(helloFromIMU, sizeof(helloFromIMU), portMy, ipDestination, portDestination);
                    
                }
            } else {
			  /*
			  Serial.println("unhandled packet:");
			  for(uint8_t n=0; n<len; n++) {
				  Serial.print(udpData[n]); Serial.print(" ");
			  }
			  Serial.println();
			  */
			}
        }
}


void loop() {
  if (!WiFi.ready()) {
    Serial.println("Network connection lost, resetting in 2seconds...");
    delay(2000);
    System.reset();        
  }
  // Loop triggers every 100 msec and sends back  heading, and roll
  currentTime = millis();
  Particle.process();
  if (currentTime - lastTime >= LOOP_TIME) {
    
    lastTime = currentTime;
    if (bno08x.dataAvailable() == true) {
      bno08xHeading = (bno08x.getYaw()) * CONST_180_DIVIDED_BY_PI; // Convert yaw / heading to degrees
      bno08xHeading = -bno08xHeading; //BNO085 counter clockwise data to clockwise data

      if (bno08xHeading < 0 && bno08xHeading >= -180) {//Scale BNO085 yaw from [-180°;180°] to [0;360°]
        bno08xHeading = bno08xHeading + 360;
      }
                    
      bno08xRoll = (bno08x.getRoll()) * CONST_180_DIVIDED_BY_PI; //Convert roll to degrees
      bno08xPitch = (bno08x.getPitch()) * CONST_180_DIVIDED_BY_PI; // Convert pitch to degrees

      bno08xHeading10x = (int16_t)(bno08xHeading * 10);
      bno08xRoll10x = (int16_t)(bno08xRoll * 10);
      bno08xPitch10x = (int16_t) (bno08xPitch * 10);

      //the heading x10
      data[5] = (uint8_t)bno08xHeading10x;
      data[6] = bno08xHeading10x >> 8;
      //the roll x10 - use pitch angle instead
      //data[7] = (uint8_t)bno08xRoll10x;
      //data[8] = bno08xRoll10x >> 8;
      data[7] = (uint8_t)bno08xPitch10x;
      data[8] = bno08xPitch10x >> 8;
    }
    
    //add the packetCount for testing
    #if defined(TEST)
    data[11] = (uint8_t) packetCount;
    data[12] = packetCount >>8;
    Serial.print("hdg: "); Serial.print(bno08xHeading);
    Serial.print(" ");
    Serial.print("roll: "); Serial.print(bno08xRoll);
    Serial.print(" ");
    Serial.print("pitch: "); Serial.print(bno08xPitch);
    Serial.print("   "); Serial.println(packetCount);
    packetCount += 1;
    #endif    
    //checksum
    int16_t CK_A = 0;
    for (int16_t i = 2; i < dataSize - 1; i++) {
      CK_A = (CK_A + data[i]);
    }

    data[dataSize - 1] = CK_A;
    //Serial.println("Sending IMU data");
    sendUdp(data, sizeof(data), portMy, ipDestination, portDestination);
  } //end of timed loop

  //delay(1);

  int packetLength = Udp.parsePacket();
  if (packetLength > 0) {
	byte fromAog[packetLength+1];
    Udp.read(fromAog, packetLength);			    
    parseAogPacket(fromAog, packetLength, Udp.remoteIP());
    //Serial.println("packet recv");
  }
} // end of main loop

/*
void mloop()
{
  
  if (myIMU.dataAvailable() == true) {
    float quatI = myIMU.getQuatI();
    float quatJ = myIMU.getQuatJ();
    float quatK = myIMU.getQuatK();
    float quatReal = myIMU.getQuatReal();
    float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();

    Serial.print(quatI, 2);
    Serial.print(F(","));
    Serial.print(quatJ, 2);
    Serial.print(F(","));
    Serial.print(quatK, 2);
    Serial.print(F(","));
    Serial.print(quatReal, 2);
    Serial.print(F(","));
    Serial.print(quatRadianAccuracy, 2);
    Serial.print(F(","));

    Serial.println();
    udp.beginPacket(remServer, remPort);
    udp.write(buffer,6);
    udp.endPacket();    
  }
  delay(500);
}

void oloop() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
}
*/