/*
  Receives corrected GPS data from the ZED9-FP over Serial
  Reads the IMU (BNO085)
  Creates the custom $PANDA message and sends it to AgOpenGPS over UDP
  
  By: Charlie Price
  
  Hardware Connections:
  ZED9-FP is connected to the Rx & Tx pins of the Photon
  Using a serial buffer to grab the bytes

  06-01-2023 VTG sentence is being appended to the $PANDA message
  	  	  	 -looks like the Serial buffer is overflowing. Change
  	  	  	  processSerial to read characters until the buffer is
  	  	  	  empty or it sees the newline character.
*/



#include "application.h"
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include "SerialBufferRK.h"

#define PANDA_MODE	1
#define NMEA_MODE	2

#define MODE_PIN 6
uint8_t sendMode;

#define SERIAL1_BAUD 230400  //this is max that the Photon (Gen2) can handle
SerialBuffer<4096> serBuf(Serial1);

SYSTEM_MODE(MANUAL);
SYSTEM_THREAD(ENABLED);

#define CONST_180_DIVIDED_BY_PI 57.2957795130823    // 180/pi
#define NEWLINE 10
#define COMMA 44 

char panda[100];

char packetBuffer[255]; //buffer to hold incoming serial data
int packetPtr;
char c;

//fields used in the $PANDA sentence
char imuHeading[6];
char imuRoll[6];
char imuPitch[6];
char imuYawRate[6];
char speedKnots[10];

// Conversion to Hexidecimal
const char* asciiHex = "0123456789ABCDEF";

UDP Udp;

uint16_t portDestination = 9990;  
static uint8_t ipDestination[] = {192,168,1,255};

//hello packet 
//IMU Reply 	                     121  121  5 * 	* 	* 	* 	* 	CRC
uint8_t helloFromIMU[] = { 128, 129, 121, 121, 5, 0, 0, 0, 0, 0, 71 };
//loop time variables in microseconds

const uint16_t IMU_PANDALOOP_MILLIS = 20;  //50Hz
const uint16_t IMU_NMEALOOP_MILLIS = 100; //10Hz

uint32_t lastImuTimeMillis = millis();
uint32_t currentTimeMillis = millis();

BNO080 bno08x;

float bno08xHeading = 0;
double bno08xRoll = 0;
double bno08xPitch = 0;
int16_t bno08xHeading10x = 0, bno08xRoll10x = 0, bno08xPitch10x = 0;

//the default network address
struct ConfigIP {
        uint8_t ipOne = 192;
        uint8_t ipTwo = 168;
        uint8_t ipThree = 1;
};  ConfigIP networkAddress;   //4 bytes

// circular buffer for the IMU readings
#define IMU_BUFFER_SIZE 10
#define IMU_BUFFER_LAG 1
struct ImuData {
  float heading = 0;
  double roll = 0;
  double pitch = 0;   
};

ImuData imuBuffer[IMU_BUFFER_SIZE];
uint8_t imuBufferPtr = 0;
 
static uint8_t myip[] = { 0,0,0,0 };
unsigned int localPort = 8888;
uint16_t portMy = 5121;
static uint8_t ipDest[] = { 192,168,1,255 };
uint16_t portDest = 9999; //AOG listens here

//uint8_t data[] = {128, 129, 121, 121, 8, 0,0,0,0, 0,0,0,0, 15};
uint8_t data[] = {128, 129, 121, 211, 8, 0,0,0,0, 0,0,0,0, 15};
int16_t dataSize = sizeof(data); 

//Program counter reset
void(*resetFunc) (void) = 0;
  
#define NMEAFIELDS 16
#define NMEAFIELDLENGTH 16
#define NMEA_UNRECOGNIZEDTYPE -1
#define NMEA_GGATYPE 1
#define NMEA_VTGTYPE 2

char nmea[NMEAFIELDS * NMEAFIELDLENGTH]; 
/*
 * the comment delimited message is passed in and each field is 
 * extracted into the character array referenced as "nmea"
 * returns the message type if recognized.
 */
byte parseNmeaSentence(char* nmea, char* serialBuffer, uint8_t len) {
  char c;
  uint8_t nBuffer = 0;      //position in buffer
  uint8_t nField = 0;       //position in field
  uint8_t idField = 0;      //each field comma delimited
  do {
    c = serialBuffer[nBuffer++];
    if (c==COMMA) {
      nmea[idField * NMEAFIELDLENGTH + nField++] = 0;  
      idField += 1;
      nField = 0;
    } else if (c!=NEWLINE) {
      nmea[idField * NMEAFIELDLENGTH + nField++] = c;
    }
  } while ((c!=NEWLINE) && (idField<NMEAFIELDS) && (nBuffer<len));
  
  nmea[idField * NMEAFIELDLENGTH + nField++] = 0;
  
  //first field contains the message type
  if ( (nmea[0]=='$') && (nmea[3]=='G') && (nmea[4]=='G') && (nmea[5]=='A'))
    return NMEA_GGATYPE;
  else if ( (nmea[0]=='$') && (nmea[3]=='V') && (nmea[4]=='T') && (nmea[5]=='G'))
    return NMEA_VTGTYPE;
  else
    return NMEA_UNRECOGNIZEDTYPE; 
}

/*
  $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M ,  ,*47
   0     1      2      3    4      5 6  7  8   9    10 11  12 13  14
        Time      Lat       Lon     FixSatsOP Alt
  Where:
     GGA          Global Positioning System Fix Data
     123519       Fix taken at 12:35:19 UTC
     4807.038,N   Latitude 48 deg 07.038' N
     01131.000,E  Longitude 11 deg 31.000' E
     1            Fix quality: 0 = invalid
                               1 = GPS fix (SPS)
                               2 = DGPS fix
                               3 = PPS fix
                               4 = Real Time Kinematic
                               5 = Float RTK
                               6 = estimated (dead reckoning) (2.3 feature)
                               7 = Manual input mode
                               8 = Simulation mode
     08           Number of satellites being tracked
     0.9          Horizontal dilution of position
     545.4,M      Altitude, Meters, above mean sea level
     46.9,M       Height of geoid (mean sea level) above WGS84
                      ellipsoid
     (empty field) time in seconds since last DGPS update
     (empty field) DGPS station ID number
      47          the checksum data, always begins with
*/

/*
0      1         2            3 4             5 6 7  8    9       10 11    12  13 14   15
$PANDA,202133.60,3516.4704286,N,08038.8960440,W,2,12,0.46,210.819,,0.036,26,6,-265,0*4D
  $PANDA
  (1) Time of fix

  position
  (2,3) 4807.038,N Latitude 48 deg 07.038' N
  (4,5) 01131.000,E Longitude 11 deg 31.000' E

  (6) 1 Fix quality:
    0 = invalid
    1 = GPS fix(SPS)
    2 = DGPS fix
    3 = PPS fix
    4 = Real Time Kinematic
    5 = Float RTK
    6 = estimated(dead reckoning)(2.3 feature)
    7 = Manual input mode
    8 = Simulation mode
  (7) Number of satellites being tracked
  (8) 0.9 Horizontal dilution of position
  (9) 545.4 Altitude (ALWAYS in Meters, above mean sea level)
  (10) 1.2 time in seconds since last DGPS update
  (11) Speed in knots

  FROM IMU:
  (12) Heading in degrees x10
  (13) Roll angle in degrees x10 (positive roll = right leaning - right down, left up)

  (14) Pitch angle in degrees x10 (Positive pitch = nose up)
  (15) Yaw Rate in Degrees / second

  CHKSUM
*/

void buildPanda(char* panda, const char* nmea) {
    strcpy(panda, "$PANDA,");
    strcat(panda, nmea + 1*NMEAFIELDLENGTH); //fixTime); Field #1
    strcat(panda, ",");
    strcat(panda, nmea + 2*NMEAFIELDLENGTH); //latitude); Field #2
    strcat(panda, ",");
    strcat(panda, nmea + 3*NMEAFIELDLENGTH); //latNS); Field #3
    strcat(panda, ",");
    strcat(panda, nmea + 4*NMEAFIELDLENGTH); //longitude); Field #4
    strcat(panda, ",");
    strcat(panda, nmea + 5*NMEAFIELDLENGTH); //lonEW); Field #5
    strcat(panda, ",");
    // 6
    strcat(panda, nmea + 6*NMEAFIELDLENGTH); //fixQuality); Field #6
    strcat(panda, ",");
    strcat(panda, nmea + 7*NMEAFIELDLENGTH); //numSats); Field #7
    strcat(panda, ",");
    strcat(panda, nmea + 8*NMEAFIELDLENGTH); //HDOP); Field #8
    strcat(panda, ",");
    strcat(panda, nmea + 9*NMEAFIELDLENGTH); //altitude); Field #9
    strcat(panda, ",");
    //10
    strcat(panda, nmea + 13*NMEAFIELDLENGTH); //ageDGPS); Field #13
    strcat(panda, ",");
    //11
    strcat(panda, speedKnots);   // from VTG message
    strcat(panda, ",");
    //12
    // generate string data for $PANDA sentence
    // we're using the circular buffer
    int8_t idx = (int8_t) imuBufferPtr - IMU_BUFFER_LAG;
    if (idx<0)
      idx = IMU_BUFFER_LAG + idx;
    itoa((int16_t)(imuBuffer[idx].heading * 10), imuHeading, 10);
    // roll & pitch are switched due to sensor orientation
    itoa((int16_t) (imuBuffer[idx].roll * 10), imuPitch, 10);
    itoa((int16_t)(imuBuffer[idx].pitch * 10), imuRoll, 10);
    itoa(0, imuYawRate, 10);        // YawRate - 0 for now
      
    strcat(panda, imuHeading);
    strcat(panda, ",");
    //13
    strcat(panda, imuRoll);
    strcat(panda, ",");
    //14
    strcat(panda, imuPitch);
    strcat(panda, ",");
    //15
    strcat(panda, imuYawRate);
    strcat(panda, "*");
    
    //Calculate Checksum;
    int16_t sum = 0;
    int16_t inx = 1;
    char tmp;
    while(1) {
      tmp = panda[inx++];      
      if (tmp == '*') { // * Indicates end of data and start of checksum
        break;
      }
      sum ^= tmp;    // Build checksum  
    }
    byte chk = (sum >> 4);
    char hex[2] = { asciiHex[chk], 0 };
    strcat(panda, hex);
    chk = (sum % 16);
    char hex2[2] = { asciiHex[chk], 0 };
    strcat(panda, hex2);    
    strcat(panda, "\r\n");    
}
  
void setup()
{

  pinMode(MODE_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  Serial.println();
  Serial.println("Particle Panda");
  delay(5000);
  
  //ZED-9FP is connected over Serial1
  Serial1.begin(SERIAL1_BAUD);
  serBuf.setup();
  
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

/*
 * remove bytes from the serial buffer into the buffer
 * when the NL character is received we want to indicate
 * back to the caller so that the buffer can be processed
 */
uint8_t processSerialBuffer() {
  while (serBuf.available()) {
    c = serBuf.read();
    packetBuffer[packetPtr++] = c;
    if (c == NEWLINE)  {
      //NL marks end of message from ZED9-FP
      return 1;
    }
  }
  return 0;
}

void processZedMessage() {
  uint8_t nmeaType = parseNmeaSentence(nmea, packetBuffer, packetPtr);
  if (nmeaType==NMEA_GGATYPE) {
    if (sendMode == PANDA_MODE) {
      buildPanda(panda, nmea);
      sendUdp((uint8_t*)panda, sizeof(panda), portMy, ipDestination, portDestination);
      memset(panda, 0, sizeof(panda));
    } else
      sendUdp((uint8_t*)packetBuffer, packetPtr, portMy, ipDestination, portDestination);

  } else if (nmeaType==NMEA_VTGTYPE) {  
    // vtg Speed knots parser.getArg(4, speedKnots);
    if (sendMode == PANDA_MODE)
      strcpy(speedKnots, nmea + 5*NMEAFIELDLENGTH);
    else
      sendUdp((uint8_t*)packetBuffer, packetPtr, portMy, ipDestination, portDestination);
  }
  //Serial.println(packetBuffer);
  c=0;
  packetPtr=0;
  memset(packetBuffer, 0, sizeof packetBuffer); 
}

/*
 * this is the non-PANDA behavior, sending the IMU directly
 * to AOG.
 */
void sendImuData() {
    int8_t idx = (int8_t) imuBufferPtr - IMU_BUFFER_LAG;
    if (idx<0)
      idx = IMU_BUFFER_LAG + idx;
    /*
    data[5] = (uint8_t)(imuBuffer[idx].heading * 10);
    data[6] = ((int16_t)(imuBuffer[idx].heading * 10)) >> 8;
    //the roll x10 - use pitch angle instead
    //data[7] = (uint8_t)bno08xRoll10x;
    //data[8] = bno08xRoll10x >> 8;
    data[7] = (uint8_t)(imuBuffer[idx].pitch * 10);
    data[8] = ((uint16_t)(imuBuffer[idx].pitch * 10)) >> 8;
    */
    //the heading x10
          data[5] = (uint8_t)bno08xHeading10x;
          data[6] = bno08xHeading10x >> 8;
          //the roll x10 - use pitch angle instead
          //data[7] = (uint8_t)bno08xRoll10x;
          //data[8] = bno08xRoll10x >> 8;
          data[7] = (uint8_t)bno08xPitch10x;
          data[8] = bno08xPitch10x >> 8;
    //add the checksum
    int16_t CK_A = 0;
    for (int16_t i = 2; i < dataSize - 1; i++) {
      CK_A = (CK_A + data[i]);
    }
    data[dataSize - 1] = CK_A;
    Serial.println("Sending IMU data");
    for(uint8_t k=0; k<sizeof(data); k++) {
    	Serial.print(data[k]); Serial.print(" ");
    }
    Serial.println();
    sendUdp(data, sizeof(data), portMy, ipDestination, portDestination);
    
}

void loop() {
  if (digitalRead(MODE_PIN)==0)
    sendMode = PANDA_MODE;
  else
	sendMode = NMEA_MODE;

  //Particle.process();  //causes Serial1 problems
  
  /*
   * make sure we still have the network
   */
  /* 
  if (!WiFi.ready()) {
    Serial.println("Network connection lost, resetting in 2seconds...");
    delay(2000);
    System.reset();        
  }
  */
  
  /*
   * process messages from the ZED9-FP on the Serial port
   */
  if (processSerialBuffer()==1) {
    processZedMessage();  
  }
  
  /*
   *  read the IMU data into the circular buffer
   */
  currentTimeMillis = millis();
  if ( ((sendMode==PANDA_MODE)
	     && (currentTimeMillis - lastImuTimeMillis >= IMU_PANDALOOP_MILLIS)) ||
	   ((sendMode==NMEA_MODE)
	     && (currentTimeMillis - lastImuTimeMillis >= IMU_NMEALOOP_MILLIS))	) {
    lastImuTimeMillis = currentTimeMillis;
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
      /*
       * only sending this in non-PANDA mode
       */
      if (sendMode == NMEA_MODE)
        sendImuData();

      //stuff it into the circular buffer
      imuBuffer[imuBufferPtr].heading = bno08xHeading;
      imuBuffer[imuBufferPtr].roll = bno08xRoll;
      imuBuffer[imuBufferPtr].pitch = bno08xPitch;
      if (imuBufferPtr<IMU_BUFFER_SIZE)
        imuBufferPtr += 1;
      else
        imuBufferPtr = 0;


    }    
  } //end of timed loop  

  /*
   *  check to see if AOG is sending something
   */
  
  // call to parsePacket(0) is non-blocking
  int packetLength = Udp.parsePacket(0);
  if (packetLength > 0) {
	byte fromAog[packetLength+1];
    Udp.read(fromAog, packetLength);			    
    parseAogPacket(fromAog, packetLength, Udp.remoteIP());
    //Serial.println("packet recv");
  }  
} // end of main loop
