/*
  Send UBX binary commands to enable RTCM sentences on u-blox ZED-F9P module
  By: Nathan Seidle / Charlie Price
  SparkFun Electronics
  Date: January 9th, 2019
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example does all steps to configure and enable a ZED-F9P as a base station:
    Set static position
    Enable six RTCM messages
    Begin outputting RTCM bytes over LoRa channel

  RadioLib SX126x Transmit Example:
  This example transmits packets using SX1262 LoRa radio module.

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a ThingPlus ExpLORAble
  Open the serial monitor at 115200 baud to see the output

  05-28-2022 new project from Git Repo RtkBase
*/

//#define _DEBUG
//#define BARE_METAL_BUILD

#include <Wire.h>
#include <RadioLib.h>
#include "Watchdog.h"
Watchdog watchdog;

//WS2812B DEFS - START
#include <Adafruit_NeoPixel.h>
#define PIN            	7
#define NUMPIXELS      	2
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ400);
//WS2812B DEFS - END

//CRAFTPORT DEFS - START
#include "ErriezSerialTerminal.h"
#define CMD_PROMPT "> "
boolean hasConfigChanged;
char *arg;
char param[10];
char newlineChar = '\r';
char delimiterChar = ' ';
SerialTerminal term(newlineChar, delimiterChar);
//CRAFTPORT DEFS - END

//EEPROM - START
#include "I2cSerialEeprom.h"
I2cSerialEeprom  eeprom;
boolean updateEeprom;
//EEPROM - END

/*  DEVICE CONFIG - START
 * the structure defining the configuration of this device
 */
struct device_parms {
	int8_t channel;
	uint8_t usbProtocolFlag;
	uint16_t rtcmSendIntervalMillis;
	boolean pushRtcmPackets;
	int32_t lat;
	int8_t latHp;
	int32_t lon;
	int8_t lonHp;
	int32_t alt;
	int8_t altHp;
	char callsign[16];
};
typedef struct device_parms Config;
Config deviceConfig;

byte devEui[8];
long upTimeSecs;
long rtcmPacketCount;
long rtcmPacketQueuedCount;

/*
 * the default device configuration is built here
 */
void bareMetal(byte* b) {
	Serial.println("Bare Metal");
    Config cfg;
    cfg.channel = 63;
    sprintf(cfg.callsign,"APL\0");
    cfg.rtcmSendIntervalMillis = 1000;
    cfg.lat= 0;
    cfg.latHp= 0;
    cfg.lon= 0;
    cfg.lonHp= 0;
    cfg.alt= 0;
    cfg.altHp= 0;
    memcpy(b, &cfg, sizeof(cfg));
}

/*
 * the bytes are copied over the device configuration structure
 */
void initialize(byte* b) {
  memcpy(&deviceConfig, b, sizeof(deviceConfig));
#if defined(_DEBUG)
  Serial.print("channel: "); Serial.println(deviceConfig.channel);
  Serial.print("callsign: "); Serial.println(deviceConfig.callsign);
#endif
}
//DEVICE CONFIG - END

//LORA STUFF - START
#define US915_STARTFREQ  902.3				// 902.3 + 0.2 * CHANNEL_NUMBER (0-63)
SX1262 radio = new Module(D36, D40, D44, D39, SPI1);	// SX1262 has the following connections:  NSS pin:   D36   DIO1 pin:  D40  NRST pin:  D44   BUSY pin:  D39

int loraPacketsSent  = 0;

#define BUFLEN 1024  //max size packet we can handle
uint8_t    buf[BUFLEN];
uint16_t  bufptr = 0;
int loraState;
long lastByteReceivedMillis;

#define _RTCM_BUFFERS 16
byte rtcmBuffer[_RTCM_BUFFERS][256];
uint8_t rtcmBufferIdx;
uint8_t rtcmBufferIdxPrev;

long lastRtcmQueuePollMillis;
#define _RTCM_QUEUE_MILLIS 100

/* int init_SX1262(uint8_t channel) *
 *  Carrier frequency: 434.0 MHz
 *  Bandwidth: 125.0 kHz (dual-sideband)
 * Spreading factor: 9
 * Coding rate: 4/7
 *  Sync word: SX126X_SYNC_WORD_PRIVATE (0x12)
 *  Output power: 10 dBm
 *  Preamble length: 8 symbols
 *  TCXO reference voltage: 1.6 V (SX126x module with TCXO)
 *  LDO regulator mode: disabled (SX126x module with DC-DC power supply)
 *  e.g.
 *  SX126x::begin(434.0, 125.0, 9, 7, SX126X_SYNC_WORD_PRIVATE, 10, 8, 1.6, false);
 */

int init_SX1262(uint8_t channel) {
	int state = radio.begin(US915_STARTFREQ + channel * 0.2, 500.0, 7, 5, 0x34, 20, 10, 0, false);
	//int state = radio.begin(914.9, 250.0, 7, 5, 0x34, 20, 10, 0, false);
	return state;
}
//LORA STUFF - END

//GNSS DEFS - START
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
SFE_UBLOX_GNSS gnssRcvr;
#define _UBLOX_CHECK_MILLIS 1000
#define _MAX_WAIT 1000
long lastUbloxCheckMillis;

/*
 * holds important information about the ZED-F9P
 */
struct fix_status {
         uint8_t fixType;
         uint8_t rtkSoln;
};
typedef struct fix_status FixStats;

//GNSS DEFS - END

#define PRINT_FIX_TYPE_MILLIS 5000
long lastPrintFixTypeMillis;

boolean sendRtcm = false;

/*  CRAFTPORT FUNCTIONS - START
 * these are the functions that are called when the user connects over Bluetooth
 */
void getHelp() {
	// Print usage details
#if not defined(_ROVER)
	Serial1.println(F("\nFlyBy Terminal    GNSS Base Station"));
#else
	Serial1.println(F("\nFlyBy Terminal    GNSS Rover"));
#endif
	Serial1.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~"));
	Serial1.println(F("commands:"));
	Serial1.println(F("  help            Print usage info"));
	Serial1.println(F("  config        Config settings"));
	Serial1.println(F("  start <secs> <m.m>  Start survey-in"));
	Serial1.println(F("  stop         Stop survey-in"));
	Serial1.println(F("  status          Show GNSS receiver status"));
	Serial1.println(F("  reboot       Reboot device"));
	Serial1.println(F("  info            System information"));
	Serial1.println(F("  callsign  <chars>       Set base call-sign"));
	Serial1.println(F("  channel   <0..63>       Set LoRa channel"));;
	Serial1.println(F("  usbprot  <nurs>         Set USB protocols"));
#if not defined(_ROVER)
	Serial1.println(F("  msecs    <nnnn>        Set RTCM interval"));;
	Serial1.println(F("  static      <>...<>          Set static position"));
#else
	Serial1.println(F(" rtcmpush <y|n>  Push RTCM packets"));
#endif

	if (hasConfigChanged) {
	  Serial1.println(F("  save          Save changes"));
	  Serial1.println(F("  discard       Discard changes"));
	}
	Serial1.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~"));
}

void getInfo() {
	char durations[64];
	long seconds;
	Serial1.println("\nDevice Information:");
	Serial1.println(F("~~~~~~~~~~~~~~~~~"));
#if not defined(_ROVER)
	Serial1.print("GNSS Base EUI:");
#else
	Serial1.print("GNSS Rover EUI:");
#endif
	char hexdigit[2];
	for(int n=0; n<8; n++) {
      sprintf(hexdigit,"%02x",devEui[n]);
	  Serial1.print(hexdigit);
	}
	for(uint8_t n=0; n<1; n++) {
	    switch(n) {
	       case 0: seconds = upTimeSecs; Serial1.print("\nupTime   : ");  break;
	    }
	    int hrs = seconds/3600;                                                        //Number of seconds in an hour
	    int mins = (seconds-hrs*3600)/60;                                     //Remove the number of hours and calculate the minutes.
	    int secs = seconds-hrs*3600-mins*60;                                //Remove the number of hours and minutes, leaving only seconds.
	    sprintf(durations, "%ih %im %is", hrs, mins, secs);
	    Serial1.println(durations);
	 }
	Serial1.println(F("~~~~~~~~~~~~~~~~~"));
#if defined(_ROVER)
	 Serial1.print("RTCM Packets Sent: "); Serial1.println(rtcmPacketCount);
#endif
}
void getConfig() {
	Serial1.println("\nDevice Configuration:");
	Serial1.println(F("~~~~~~~~~~~~~~~~~"));
	Serial1.print("Callsign: "); Serial1.println(deviceConfig.callsign);
	Serial1.print("Channel: "); Serial1.print(deviceConfig.channel);Serial1.print(" (");	Serial1.print(US915_STARTFREQ + deviceConfig.channel * 0.2);Serial1.println("MHz)");
#if not defined(_ROVER)
	Serial1.print("RTCM send interval: "); Serial1.print(deviceConfig.rtcmSendIntervalMillis); Serial1.println("\u33B3");
#else
	Serial1.print("RTCM packets enabled: ");
	if (deviceConfig.pushRtcmPackets == true)
	  Serial1.println("Yes");
   else
	  Serial1.println("No");
#endif
	Serial1.print("USB protocols: ");
	if ((deviceConfig.usbProtocolFlag & COM_TYPE_NMEA) ==  COM_TYPE_NMEA)
		Serial1.print("NMEA ");
	if ((deviceConfig.usbProtocolFlag &  COM_TYPE_UBX) == COM_TYPE_UBX)
		Serial1.print("UBX ");
	if ((deviceConfig.usbProtocolFlag &  COM_TYPE_RTCM3) == COM_TYPE_RTCM3)
		Serial1.print("RTCM3 ");
	if ((deviceConfig.usbProtocolFlag &  COM_TYPE_SPARTN) == COM_TYPE_SPARTN)
		Serial1.print("SPARTN ");
	if (deviceConfig.usbProtocolFlag ==0)
		Serial1.print("none configured");
	Serial1.println();
	Serial1.println(F("~~~~~~~~~~~~~~~~~"));
}

void setChannel() {
  char* channel = term.getNext();
  deviceConfig.channel = atof(channel);
  Serial1.println(F("Channel change Ok. (save/reboot)"));
  hasConfigChanged = true;
}
void setCallSign() {
  char* callsign = term.getNext();
  char c;
  uint8_t n=0;
  do {
	  c = callsign[n];
	  deviceConfig.callsign[n] = c;
	  n++;
  } while (c!=0);
  //memcpy(deviceConfig.callsign, callsign, 3);
  Serial1.println(F("Callsign change Ok. (save/reboot)"));
  hasConfigChanged = true;
}

void setRtcmSendIntervalMillis() {
	char* secs = term.getNext();
	deviceConfig.rtcmSendIntervalMillis = atoi(secs);
	Serial1.println(F("RTCM send interval change Ok. (save/reboot)"));
	hasConfigChanged = true;
}

void setRtcmPush() {
	char* yorn = term.getNext();
	switch (yorn[0]) {
	  case 'y' : deviceConfig.pushRtcmPackets = true; break;
	  default : deviceConfig.pushRtcmPackets = false; break;
	}
	Serial1.println(F("RTCM push enable change Ok. (save/reboot)"));
	hasConfigChanged = true;
}

void setUsbProtocols() {
	char* protocols = term.getNext();
	char  protocol;
	uint8_t n=0;
	uint8_t flag = 0;
	do {
		protocol = protocols[n++];
	    if (protocol!=0) {
		  switch (protocol) {
		    case 'n' : flag |= COM_TYPE_NMEA; break;
		    case 'u' : flag |= COM_TYPE_UBX; break;
		    case 'r'  : flag |= COM_TYPE_RTCM3; break;
		    case 's'  : flag |= COM_TYPE_SPARTN;
		  }
	    }
    } while (protocol!=0);
	deviceConfig.usbProtocolFlag = flag;
	gnssRcvr.setUSBOutput(deviceConfig.usbProtocolFlag);
	Serial1.println(F("USB protocols change Ok. (save/reboot)"));
	hasConfigChanged = true;
}

void setStaticPosition() {
	int32_t lat       = atol(term.getNext());
	int8_t latHp    = atol(term.getNext());
	int32_t lon      = atol(term.getNext());
    int8_t lonHp   = atol(term.getNext());
    int32_t alt      = atol(term.getNext());
    int8_t altHp   = atol(term.getNext());
	// For Lat/Lon/Alt the units are: degrees^-7, degrees^-9, degrees^-7, degrees^-9, cm, 0.1mm
   if (gnssRcvr.setStaticPosition(lat, latHp,  lon, lonHp, alt,  altHp, true, _MAX_WAIT)) {
	  deviceConfig.lat = lat;
	  deviceConfig.latHp = latHp;
	  deviceConfig.lon = lon;
	  deviceConfig.lonHp = lonHp;
	  deviceConfig.alt = alt;
	  deviceConfig.altHp = altHp;
	  Serial1.println(F("Static position change Ok. (save/reboot)"));
   }  else {
	  Serial1.println(F("Static position change failed."));
   }
}

void reboot() {
	Serial1.println("\nRebooting now....");
	NVIC_SystemReset();
}

void saveConfigChanges() {
  byte b[sizeof(deviceConfig)];
  memcpy(b, &deviceConfig, sizeof(deviceConfig));
  eeprom.saveBytes(b, sizeof(device_parms));
  eeprom.readBytes(b, sizeof(device_parms));
  initialize(b);
  Serial1.println("Config changes saved");
}

void discardConfigChanges() {
  byte b[sizeof(deviceConfig)];
  eeprom.readBytes(b, sizeof(device_parms));
  initialize(b);
  Serial1.println("Changes discarded");
}

/*
 * get Survey -In information
 * bool getSurveyInActive(uint16_t maxWait = defaultMaxWait);
  bool getSurveyInValid(uint16_t maxWait = defaultMaxWait);
  uint16_t getSurveyInObservationTime(uint16_t maxWait = defaultMaxWait);     // Truncated to 65535 seconds
  uint32_t getSurveyInObservationTimeFull(uint16_t maxWait = defaultMaxWait); // Return the full uint32_t
  double getSurveyInMeanAccuracy(uint16_t maxWait = defaultMaxWait);
 */

void getSvin() {
	Serial1.print("\n\nSurvey-In Status: ");
	if (gnssRcvr.getSurveyInActive(_MAX_WAIT))
		Serial1.println(F("In-progress"));
	else
		Serial1.println(F("Completed"));

	Serial1.print("Mean position valid: ");
		if (gnssRcvr.getSurveyInValid(_MAX_WAIT))
			Serial1.println(F("Yes."));
		else
			Serial1.println(F("No"));

	Serial1.print("Observation time: ");
	Serial1.println(gnssRcvr.getSurveyInObservationTime(_MAX_WAIT));

	Serial1.print("Mean 3D StdDev: ");
	Serial1.print(gnssRcvr.getSurveyInMeanAccuracy(_MAX_WAIT)); Serial1.println( "m");
}

void startSurveyIn() {
	float randomizer = random(0, 100) / 500.0;
	 int periodSecs =  atoi(term.getNext());
	 arg = term.getNext();

	 if (gnssRcvr.getSurveyInActive() == true)   {
	    Serial1.println(F("Survey-in already in progress."));
	  } else	  {
	    boolean ok = gnssRcvr.enableSurveyMode(periodSecs, randomizer + atof(arg));
	    if (ok) {
		   Serial1.println(F("Survey-in start Ok."));
	    } else {
		   Serial1.println(F("Survey-in start failed."));
	    }
	  }
}

void haltSurveyIn() {
	 if (gnssRcvr.getSurveyInActive() == true)   {
	    Serial1.println(F("Halting survey-in..."));
	    boolean ok = gnssRcvr.disableSurveyMode(_MAX_WAIT);
	    if (ok)
	    	Serial1.println(F("Halted Ok."));
	    else
	    	Serial1.println(F("Halt failed."));
	  } else	  {
	       Serial1.println(F("No survey-in currently running."));
	  }
}

String double2string(double n, int ndec) {
    String r = "";
    int v = n;
    r += v;     // whole number part
    r += '.';   // decimal point
    int i;
    for (i=0; i<ndec; i++) {  // iterate through each decimal digit for 0..ndec
        n -= v;
        n *= 10;
        v = n;
        r += v;
    }
    return r;
}


void degrees2dms(float angle, int* deg, unsigned int* min, float *sec) {
  float t;
  unsigned int d, m;
  float s;

  if (angle < 0.0) {
    angle = - angle;
  }

  d = (unsigned int)angle;
  t = (angle - (float )d)*60.0;
  m = (unsigned int)(t);
  s = (t - (float )m)*60.0;

  /* Now some rounding issues */
  if (s >= 59.995) {
    s = 0.0;
    m ++;
    if (m == 60) {
      m = 0;
      d++;
    }
  }
  *deg = d;
  *min = m;
  *sec = s;

}

void getStatus() {
	Serial1.println(F("\nGNSS Receiver Status:"));
	Serial1.println(F("~~~~~~~~~~~~~~~~~"));
    FixStats fs;
    fs.fixType = gnssRcvr.getFixType();
    if( fs.fixType == 0) Serial1.print(F("No fix"));
    else if( fs.fixType == 1) Serial1.print(F("Dead reckoning"));
    else if( fs.fixType == 2) Serial1.print(F("2D Fix"));
    else if( fs.fixType == 3) Serial1.print(F("3D Fix"));
    else if( fs.fixType == 4) Serial1.print(F("GNSS Fix + Dead reckoning"));
    else if( fs.fixType == 5) Serial1.print(F("Time only Fix"));

    fs.rtkSoln = gnssRcvr.getCarrierSolutionType();
    if (fs.rtkSoln == 0) Serial1.println(F(" / No RTK solution"));
    else if (fs.rtkSoln == 1) Serial1.println(F("/ RTK High precision floating fix"));
    else if (fs.rtkSoln == 2) Serial1.println(F("/ RTK High precision fix"));
	Serial1.print(F("GNSS satellites used:        ")); Serial1.println(gnssRcvr.getSIV(_MAX_WAIT));		// Returns number of satellites in view
	boolean fixOk = gnssRcvr.getGnssFixOk(_MAX_WAIT); 		// Get whether we have a valid fix (i.e within DOP & accuracy masks)
	boolean diffSoln = gnssRcvr.getDiffSoln(_MAX_WAIT);
	Serial1.print(F("Fix Ok: "));
	if (fixOk)
		Serial1.println(F("Yes"));
	else
	    Serial1.println(F("No"));
	Serial1.print(F("Differential solution: "));
	if (diffSoln)
		Serial1.println(F("Yes"));
	else
	    Serial1.println(F("No"));

	double longitude = gnssRcvr.getLongitude(_MAX_WAIT)/10000000.0;
	double latitude    =  gnssRcvr.getLatitude(_MAX_WAIT)/10000000.0;

	 int deg;
	 unsigned int  min;
	float secs;

	Serial1.print(F("Longitude (deg):     "));
	if (longitude>0.0)
		 Serial1.print(double2string(longitude, 8));
	else {
		Serial1.print(F("- "));
		Serial1.print(double2string(-longitude, 8));
	}
	Serial1.println("\u00B0");
	degrees2dms(longitude, &deg, &min, &secs);
	if (longitude<0.0)
		deg = -deg;
	Serial1.print(F("                   (dms):     ")); Serial1.print(deg); Serial1.print("\u00B0"); Serial1.print(" ");  Serial1.print(min); Serial1.print("' "); Serial1.print(secs); Serial1.println("\"");

	Serial1.print(F("Latitude    (deg):      "));
	if (latitude>0.0)
	  Serial1.print(double2string(latitude, 8));
	else {
	  Serial1.print(F("- "));
	  Serial1.print(double2string(-latitude, 8));
	}
	Serial1.println("\u00B0");
	degrees2dms(latitude, &deg, &min, &secs);
	if (latitude<0.0)
		deg = -deg;
	Serial1.print(F("                   (dms):     ")); Serial1.print(deg); Serial1.print("\u00B0"); Serial1.print(" ");  Serial1.print(min); Serial1.print("' "); Serial1.print(secs); Serial1.println("\"");

	Serial1.print(F("Elevation  (MSL):     ")); Serial1.print(gnssRcvr.getAltitudeMSL(_MAX_WAIT)/1000.0); Serial1.println("\u004D");
	Serial1.print(F("Position accuracy:  ")); Serial1.print(gnssRcvr.getPositionAccuracy(_MAX_WAIT)/10.0); Serial1.println("\u339D");

	char datetime[24];
	//uint8_t dow =  gnssRcvr.getTimeOfWeek(_MAX_WAIT);
    uint16_t yyyy = gnssRcvr.getYear(_MAX_WAIT);
	uint8_t   MM =gnssRcvr.getMonth(_MAX_WAIT);
	uint8_t   dd = gnssRcvr.getDay(_MAX_WAIT);
	uint8_t   hh = gnssRcvr.getHour(_MAX_WAIT);
	uint8_t   mm = gnssRcvr.getMinute(_MAX_WAIT);
	uint8_t   ss  = gnssRcvr.getSecond(_MAX_WAIT);
	uint16_t msecs = gnssRcvr.getMillisecond(_MAX_WAIT);
	sprintf(datetime, "%02u:%02u:%02u.%03u  UTC", hh, mm, ss, msecs);
	Serial1.print(F("\nTime:    ")); Serial1.println(datetime);
	sprintf(datetime, "%02u/%02u/%02u", MM, dd, yyyy);
    Serial1.print(F("Date:    ")); Serial1.println(datetime);

#if not defined(_ROVER)
	Serial1.print("\nSurvey-In Status: ");
	if (gnssRcvr.getSurveyInActive(_MAX_WAIT))
		Serial1.println(F("In-progress"));
	else
		Serial1.println(F("Completed"));

	Serial1.print("Mean position valid: ");
		if (gnssRcvr.getSurveyInValid(_MAX_WAIT))
			Serial1.println(F("Yes"));
		else
			Serial1.println(F("No"));

	Serial1.print("Observation time: ");
	Serial1.print(gnssRcvr.getSurveyInObservationTime(_MAX_WAIT));Serial1.println(F("seconds"));

	Serial1.print("Mean 3d \u03c3: ");
	Serial1.print(gnssRcvr.getSurveyInMeanAccuracy(_MAX_WAIT)); Serial1.println("\u004D");
#endif
	Serial1.println(F("~~~~~~~~~~~~~~~~~"));
}

void postCommandHandler()
{
    Serial1.print(F(CMD_PROMPT));
}
void defaultHandler(const char *cmd) {
	Serial1.print("Unrecognized command \""); Serial1.print(cmd); Serial1.println("\"");
}
// CRAFTPORT FUNCTIONS - END

void rtcmQueue() {
	if (rtcmBufferIdxPrev!=rtcmBufferIdx) {
	   loraState = radio.transmit(&rtcmBuffer[rtcmBufferIdxPrev][1], rtcmBuffer[rtcmBufferIdxPrev][0]);
	   rtcmPacketCount += 1;
	   if (loraState == RADIOLIB_ERR_NONE) {
	 	   	     //do nothing
	   } else if (loraState == RADIOLIB_ERR_PACKET_TOO_LONG) {
	    // the supplied packet was longer than 256 bytes
	    Serial.println(F("LoRa too long!"));
	   } else if (loraState == RADIOLIB_ERR_TX_TIMEOUT) {
	    // timeout occured while transmitting packet
	 	Serial.println(F("LoRa timeout!"));
	   }
	   rtcmBufferIdxPrev += 1;
	   if(rtcmBufferIdxPrev>=_RTCM_BUFFERS)
	      rtcmBufferIdxPrev = 0;
	   //Serial.print("#");
	}
}

FixStats  printFixType() {
	    FixStats fs;
	    fs.fixType = gnssRcvr.getFixType();
	    Serial.print(" \n\r");
	    Serial.print(gnssRcvr.getUnixEpoch()); Serial.print(" Fix: ");
	    if( fs.fixType == 0) Serial.print(F("No fix"));
	    else if( fs.fixType == 1) Serial.print(F("Dead reckoning"));
	    else if( fs.fixType == 2) Serial.print(F("2D"));
	    else if( fs.fixType == 3) Serial.print(F("3D"));
	    else if( fs.fixType == 4) Serial.print(F("GNSS + Dead reckoning"));
	    else if( fs.fixType == 5) Serial.print(F("Time only"));

	    fs.rtkSoln = gnssRcvr.getCarrierSolutionType();
	    Serial.print(" RTK: ");
	    Serial.print(fs.rtkSoln);
	    if (fs.rtkSoln == 0) Serial.println(F(" (No solution)"));
	    else if (fs.rtkSoln == 1) Serial.println(F(" (High precision floating fix)"));
	    else if (fs.rtkSoln == 2) Serial.println(F(" (High precision fix)"));

	    Serial.print("RTCM Packets Queued= "); Serial.println( rtcmPacketQueuedCount);
	    Serial.print("RTCM Packets Sent      = "); Serial.println(rtcmPacketCount);

	    return fs;
}

/*
 RTCM 3.2 packets look like this:
 Byte 0: Always 0xD3
 Byte 1: 6-bits of zero
 Byte 2: 10-bits of length of this packet including the first 3 header bytes and last 3 crc bytes, total of 6 extra bytes.
 byte 3 + 4 bits: Msg type 12 bits
 Example: D3 00 7C 43 F0 ... / 0x7C = 124+6 = 130 bytes in this packet, 0x43F = Msg type 1087
*/
struct rtcm_v3_packet {
       uint8_t flag;
       uint8_t len_hi;
       uint8_t len_lo;
       uint8_t msgtype_hi;
       uint8_t msgtype_lo;
       uint8_t payload[1024];
   };
typedef struct rtcm_v3_packet Rtcm;
Rtcm rtcmPacket;

//This function gets called from the SparkFun u-blox Arduino Library.
//As each RTCM byte comes in you can specify what to do with it
//Useful for passing the RTCM correction data to a radio, Ntrip broadcaster, etc.
void SFE_UBLOX_GNSS::processRTCM(uint8_t incoming)  {
   if (!sendRtcm) {
	  bufptr=0;
	  return;
   }

#if defined(_DUMP_RTCH_BYTES)
  //Pretty-print the HEX values to Serial
  if (gnssRcvr.rtcmFrameCounter % 16 == 0) Serial.println();
	Serial.print(F(" "));
  if (incoming < 0x10)
	Serial.print(F("0"));
  Serial.print(incoming, HEX);
#endif

  buf[bufptr++] = incoming;

  if (bufptr==5) {  /* we have enough bytes to calculate packet length */
	 memcpy(&rtcmPacket, buf, 5);
  } else if (bufptr==(256*(rtcmPacket.len_hi & 0x03) + rtcmPacket.len_lo + 6)) {  //3-byte header + 3-byte CRC
	  // we can send 256bytes maximum over LoRa
	  uint16_t msgType = (256* rtcmPacket.msgtype_hi + rtcmPacket.msgtype_lo)>>4;
	  if(bufptr<=256) {
	    boolean sendIt = false;

	    Serial.print(msgType); Serial.print("-"); Serial.println(millis()); Serial.print(" ");
	    switch(msgType) {
	        case   1005: sendIt=true; break;
	        case   1074: sendIt=true; break;
	        case   1084: sendIt=true; break;
	        case   1094: sendIt=true; break;
	        case   1124: sendIt=true; break;
	        case   1230: sendIt=true; Serial.println(); break;
	        default: Serial.print("DISCARDING  MESSAGE type="); Serial.println(msgType); break;
	    }

	    if  (sendIt==true) {
	    	//queue the packet
	    	rtcmBufferIdx += 1;
	    	if (rtcmBufferIdx>=_RTCM_BUFFERS)
	    	   rtcmBufferIdx = 0;
            rtcmBuffer[rtcmBufferIdx][0] =bufptr;
            memcpy(&rtcmBuffer[rtcmBufferIdx][1], buf, bufptr);
            rtcmPacketQueuedCount += 1;
	    }
	  } else {
		  Serial.println(F("Message too big!"));
		  Serial.print("bytes="); Serial.print(bufptr);
		  Serial.print(" type="); Serial.println(msgType);
	  }
	  bufptr=0;

  }
  lastByteReceivedMillis = millis();
  loraPacketsSent++;
}

void setup()
{
  Serial.begin(115200);
#if defined (_DEBUG)
  while (!Serial); 		//Wait for user to open terminal
#endif

/*
  pixels.setBrightness(255);
  pixels.begin();
  pixels.setPixelColor(0, pixels.Color(255,0,0));
  pixels.setPixelColor(1, pixels.Color(255,255,0));

  pixels.show();
*/

  delay(500);
  Serial.println(F("\n\ru-blox Survey-In Base Station"));

  //qwiic and the ZED-F9P are on Wire
  Wire.begin();
  Wire.setClock(100000); //Increase I2C clock speed to 400kHz

  //EEPROM is on Wire1
  Wire1.begin();
  Wire1.setClock(400000); //Increase I2C clock speed to 400kHz

  //LOAD CONFIGURATION - START
  eeprom.i2c_eeprom_read_buffer(_EEPROM_I2C_ADDR, _DEVEUI_ADDR, devEui, 8);

#if defined(_DEBUG)
  Serial.print("devEui: ");
  for(uint8_t n=0; n<8; n++) {
	  if (devEui[n]<0x10)
		  Serial.print(" 0");
	  else
		  Serial.print(" ");
	  Serial.print(devEui[n], HEX);
  }
  Serial.println();
#endif

  byte b[sizeof(deviceConfig)];

#if defined(BARE_METAL_BUILD)
  bareMetal(b);
#if defined(_DEBUG)
  Serial.print("b[]: ");
  for(uint8_t n=0; n<sizeof(deviceConfig); n++) {
	  Serial.print(" x"); Serial.print(b[n], HEX);
  }
  Serial.println();
#endif
  eeprom.saveBytes(b, sizeof(deviceConfig));
#endif

  eeprom.readBytes(b, sizeof(deviceConfig));
#if defined(_DEBUG)
  Serial.print("b[]: ");
  for(uint8_t n=0; n<sizeof(deviceConfig); n++) {
	  Serial.print(" x"); Serial.print(b[n], HEX);
  }
  Serial.println();
#endif
  initialize(b);

#if defined(_DEBUG)
  Serial.print("LoRa channel: "); Serial.println(deviceConfig.channel);
#endif
//LOAD CONFIGURATION - END

  // initialize SX1262 with default settings
  Serial.print(F("SX1262 Radio  Initializing ... "));

  int state = init_SX1262(deviceConfig.channel);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("Radio startup completed!"));
    radio.transmit("Hello World");
  } else {
    Serial.print(F("Radio startup failed with code  "));
    Serial.println(state);
    while (true);
  }

  /*
   *   begin(TwoWire &wirePort = Wire, uint8_t deviceAddress = 0x42, uint16_t maxWait = defaultMaxWait, bool assumeSuccess = false);
   */

//long wireFrequency = 400000;
while( gnssRcvr.begin(Wire, 0x42,  5000,  false) == false) {
 //while (gnssRcvr.begin() == false)  { //Connect to the u-blox module using Wire port
    Serial.println(F("u-blox GNSS not detected at default I2C address.  Massaging Wire...."));
   // gnssRcvr.begin(0x42, 2000)
    //Wire.flush();
    //Wire.clearWriteError();
    //Wire.end();
    //delay(2000);
    // Wire.begin();
    // wireFrequency -= 40000;
    // Serial.print("Wire frequency="); Serial.println(wireFrequency);
    // Wire.setClock(wireFrequency);
     delay(5000);
  }
 Serial.println(F("u-blox GNSS initialized."));
  //gnssRcvr.factoryDefault(); delay(5000);

  gnssRcvr.setI2COutput(COM_TYPE_UBX); 										//Set the I2C port to output UBX only (turn off NMEA noise)
  gnssRcvr.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); 		//Save the communications port settings to flash and BBR

  bool response = true;

  /*  Enable RTCM message types that will be broadcast to the GNSS Rovers over LoRa:
   1005 the ECEF location of the antenna - the antenna reference point (ARP)
   1074 GPS MSM4 type 4 Multiple Signal Message format for the USA’s GPS system.
   1084 GLONASS MSM4 type 4 Multiple Signal Message format for the Russian GLONASS system.
   1094 Galileo MSM4 type 4 Multiple Signal Message format for Europe’s Galileo system.
   1124 BeiDou MSM4 type 4 Multiple Signal Message format for China’s BeiDou system.
   1230 GLONASS L1 and L2 Code-Phase Biases corrections for the inter-frequency bias caused by the different FDMA frequencies

   All messages will be sent over the I2C connection from the Ublox ZED-F9P
  */
  gnssRcvr.setI2COutput(COM_TYPE_UBX | COM_TYPE_RTCM3, 2000); //Set the I2C port to output UBX and RTCM sentences (not really an option, turns on NMEA as well)

  response &= gnssRcvr.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_I2C, 300);
  response &= gnssRcvr.enableRTCMmessage(UBX_RTCM_1074, COM_PORT_I2C, deviceConfig.rtcmSendIntervalMillis/100);
  response &= gnssRcvr.enableRTCMmessage(UBX_RTCM_1084, COM_PORT_I2C, deviceConfig.rtcmSendIntervalMillis/100);
  response &= gnssRcvr.enableRTCMmessage(UBX_RTCM_1094, COM_PORT_I2C, deviceConfig.rtcmSendIntervalMillis/100);
  response &= gnssRcvr.enableRTCMmessage(UBX_RTCM_1124, COM_PORT_I2C, deviceConfig.rtcmSendIntervalMillis/100);
  response &= gnssRcvr.enableRTCMmessage(UBX_RTCM_1230, COM_PORT_I2C, 100); 		//Enable message every 10 seconds



/*
  response &= gnssRcvr.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_I2C, 100);
  response &= gnssRcvr.enableRTCMmessage(UBX_RTCM_1074, COM_PORT_I2C, 10);
  response &= gnssRcvr.enableRTCMmessage(UBX_RTCM_1084, COM_PORT_I2C, 10);
  response &= gnssRcvr.enableRTCMmessage(UBX_RTCM_1094, COM_PORT_I2C, 10);
  response &= gnssRcvr.enableRTCMmessage(UBX_RTCM_1124, COM_PORT_I2C, 10);
  response &= gnssRcvr.enableRTCMmessage(UBX_RTCM_1230, COM_PORT_I2C, 100); 		//Enable message every 10 seconds
*/
  if (response == true)   {
    Serial.println(F("RTCM message types enabled"));
  }
  else  {
    Serial.println(F("One or more RTCM message types could not be enabled.  Startup is Halting."));
    //while (1); //Freeze
  }

  lastUbloxCheckMillis = millis();

   bool success = true;
    //Units are cm so 1234 = 12.34m
    //success &= gnssRcvr.setStaticPosition(-128020831, -471680385, 408666581);
    //Units are cm with a high precision extension so -1234.5678 should be called: (-123456, -78)
    //success &= gnssRcvr.setStaticPosition(-128020830, -80, -471680384, -70, 408666581, 10); //With high precision 0.1mm parts

    //We can also set via lat/long
    //40.09029751,-105.18507900,1560.238
    //success &= gnssRcvr.setStaticPosition(400902975, -1051850790, 156024, true); //True at end enables lat/long input
    //Param	Value	Units
   // Position Latitude, Longitude, Height, MSL	35.2745203, -80.6483179, 180.1, 212.9	[deg,deg,m,m]

    //success &= gnssRcvr.setStaticPosition(400902975, 10, -1051850790, 0, 156023, 80, true);

    if (success == false)   {
      Serial.println(F("Failed to set static position"));
      while (1); 	//Freeze
    } else {
      Serial.print("Base sending RTCM packets over LoRa P2P(PHY) at "); Serial.print(US915_STARTFREQ + deviceConfig.channel * 0.2); Serial.println("MHz");
    }


    //CRAFTPORT SETUP - START
    Serial1.begin(9600);
    term.setSerialEcho(false);
    term.addCommand("help", getHelp);
    term.addCommand("config", getConfig);
    term.addCommand("save", saveConfigChanges);
    term.addCommand("discard", discardConfigChanges);
    term.addCommand("reboot", reboot);
    term.addCommand("info", getInfo);
    term.addCommand("channel",setChannel);
    term.addCommand("callsign",setCallSign);
    term.addCommand("start",startSurveyIn);
    term.addCommand("stop",haltSurveyIn);
    term.addCommand("status",getStatus);
    term.addCommand("usbprot", setUsbProtocols);
#if not defined(_ROVER)
    term.addCommand("start",startSurveyIn);
    term.addCommand("stop",haltSurveyIn);
    term.addCommand("msecs", setRtcmSendIntervalMillis);
    term.addCommand("static", setStaticPosition);
#else
    term.addCommand("rtcmpush",setRtcmPush);
#endif
    term.setPostCommandHandler(postCommandHandler);
    term.setDefaultHandler(defaultHandler);
    delay(100);
    getHelp();
    Serial1.print(F(CMD_PROMPT));
    //CRAFTPORT SETUP - END
}

void loop()   {
  watchdog.poll();
  term.readSerial();												//poll the craftport interface
  if ((millis() - lastRtcmQueuePollMillis) > _RTCM_QUEUE_MILLIS) {
	  rtcmQueue();
	  lastRtcmQueuePollMillis = millis();
  }
  if ((millis() - lastUbloxCheckMillis) > _UBLOX_CHECK_MILLIS) {
	upTimeSecs += _UBLOX_CHECK_MILLIS/1000;
    gnssRcvr.checkUblox(); 								//See if new data is available. Process bytes as they come in.
    lastUbloxCheckMillis = millis();
    //Serial.print(".");
  }
  if ((millis() - lastPrintFixTypeMillis)> 30000) {

	     FixStats fs =  printFixType();
   		  if (fs.fixType==5) {
   			  sendRtcm = true;
   			  //Serial.println("enabling RTCM send.");
   		  } else {
   			  sendRtcm = false;
   			  //Serial.println("inhibiting RTCM send.");
   		  }
   		rtcmPacketCount = 0;
   		rtcmPacketQueuedCount = 0;
   		lastPrintFixTypeMillis = millis();
  }
}
