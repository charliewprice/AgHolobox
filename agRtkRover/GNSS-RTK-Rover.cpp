/*
  Send UBX binary commands to enable RTCM sentences on u-blox ZED-F9P module
  By: Nathan Seidle / Charlie Price
  SparkFun Electronics
  Date: January 9th, 2019
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example does all steps to configure and enable a ZED-F9P as a rover:

  RadioLib SX126x receiver receives RTCM corrections over LoRa

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a ThingPlus ExpLORAble
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#define _ROVER
//#define _DEBUG
//#define _CALLBACK_DEBUG
//#define BARE_METAL_BUILD
//#define DUMP_RTCM_PACKETS

#include <Wire.h>  //Needed for I2C to GNSS
#include <RadioLib.h>

#include "Watchdog.h"
Watchdog watchdog;

//
#include<SPI.h>
#include <ArduinoJson.h>
#define _DASHBOARD_UPDATE_MILLIS 5000
long lastDashboardMillis;
//

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

//GNSS DEFS - START
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include "ublox_structures.h"

#define  _MAX_WAIT 1000
#define _UBLOX_CHECK_MILLIS 1000

SFE_UBLOX_GNSS gnssRcvr;

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

// SI 2 - START

// SI 2 - END

// SPEED INDICATOR - START

// SPEED INDICATOR - END

/*  DEVICE CONFIG - START
 * the structure defining the configuration of this device
 */
struct device_parms {
	int8_t channel;
	uint8_t usbProtocolFlag;
	uint8_t navRateHz;
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
long rtcmPacketCount,  rtcmPacketCountPeriod;
long rtcmPacketPushedCount, rtcmPacketPushedCountPeriod;

/*
 * the default device configuration is built here
 */
void bareMetal(byte* b) {
	Serial.println("Bare Metal");
    Config cfg;
    cfg.channel = 63;
    sprintf(cfg.callsign,"ROVER_1\0");
    cfg.rtcmSendIntervalMillis = 1000;
    cfg.navRateHz = 2;
    cfg.pushRtcmPackets = true;
    cfg.usbProtocolFlag =  COM_TYPE_NMEA;
    memcpy(b, &cfg, sizeof(cfg));
}

/*
 * the bytes are copied over the device configuration structure
 */
void initialize(byte* b) {
  memcpy(&deviceConfig, b, sizeof(deviceConfig));
#if defined(_DEBUG)
  Serial.print("LoRa channel: "); Serial.println(deviceConfig.channel);
  Serial.print("Node callsign: "); Serial.println(deviceConfig.callsign);
#endif
}
//DEVICE CONFIG - END

//LORA STUFF - START
#define US915_STARTFREQ  902.3				// 902.3 + 0.2 * CHANNEL_NUMBER (0-63)
SX1262 radio = new Module(D36, D40, D44, D39, SPI1);	// SX1262 has the following connections:  NSS pin:   D36   DIO1 pin:  D40  NRST pin:  D44   BUSY pin:  D39

int loraPacketCount  = 0;
int loraPacketCountPeriod;
#define _RSSI_AVERAGE_WINDOW  10
uint8_t rssiAvgPtr = 0;
int  rssi[_RSSI_AVERAGE_WINDOW];
long  rssiAverage;

#define BUFLEN 1024  //max size packet we can handle
//uint8_t    buf[BUFLEN];
//uint16_t  bufptr = 0;
int loraState;
//long lastByteReceivedMillis;

#define _RTCM_BUFFERS 16
byte rtcmBuffer[_RTCM_BUFFERS][256];
uint8_t rtcmBufferIdx;
uint8_t rtcmBufferIdxPrev;

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

#define PRINT_FIX_TYPE_MILLIS 15000
long lastPrintFixTypeMillis;

//#define DUMP_RTCM_PACKETS
//#define LORA_DEBUG

/*  CRAFTPORT FUNCTIONS - START
 * these are the functions that are called when the user connects over Bluetooth
 */
void getHelp() {
	// Print usage details
#if not defined(_ROVER)
	Serial1.println(F("\n~ FlyBy Terminal    GNSS Base"));
#else
	Serial1.println(F("\n~ FlyBy Terminal    GNSS Rover"));
#endif
	Serial1.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~"));
	Serial1.println(F("commands:"));
	Serial1.println(F("  help            Print usage info"));
	Serial1.println(F("  config        Config settings"));
	//Serial1.println(F("  survey <secs> <m.m>  Start survey-in"));
	Serial1.println(F("  state          Show GNSS receiver status"));
	Serial1.println(F("  reboot       Reboot device"));
	Serial1.println(F("  info            System information"));
	Serial1.println(F("  callsign  <chars>       Set base call-sign"));
	Serial1.println(F("  channel   <n>              Set LoRa channel"));;
	Serial1.println(F("  usbprot  <nurs>         Set USB protocols"));
#if defined(_ROVER)
	Serial1.println(F("  navrate <hz>"));
	Serial1.println(F("  rtcmpush <y|n>  Push RTCM packets"));
#else
	Serial1.println(F("  rtcmmsecs <nnnn>  Set RTCM interval"));;
	Serial1.println(F("  static <> <> <> <> <> <> Set static position"));
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
	 Serial1.print("RAW   Packets Received: "); Serial1.println( loraPacketCountPeriod);
	 Serial1.print("RTCM Packets Received: "); Serial1.println( rtcmPacketCountPeriod);
	 Serial1.print("RTCM Packets Pushed   : "); Serial1.println( rtcmPacketPushedCountPeriod);
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
	Serial1.print("Nav Rate(Hz): "); Serial1.println(deviceConfig.navRateHz);
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

void setNavRateHz() {
	char* rate = term.getNext();
	deviceConfig.navRateHz = atoi(rate);
	Serial1.println(F("Nav Rate change Ok. (save/reboot)"));
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

void dashboardJson() {
	StaticJsonDocument<200> doc;
    // StaticJsonObject allocates memory on the stack, it can be
	 // replaced by DynamicJsonDocument which allocates in the heap.
	 //
	 // DynamicJsonDocument  doc(200);
	 // "{\"ftype\":\"Rtk\",\"data\":[12.2,48,31,0.049]}";

	FixStats fs;
	fs.fixType = gnssRcvr.getFixType();
    fs.rtkSoln = gnssRcvr.getCarrierSolutionType();
	switch(fs.fixType) {
	  case 0: doc["ftype"] = "NONE  "; break;
	  case 1: doc["ftype"] = "DR      "; break;
	  case 2: doc["ftype"] = "2D      "; break;
	  case 3:  {
		  switch(fs.rtkSoln) {
		    case 0:  doc["ftype"] = "3D      "; break;
		    case 1:  doc["ftype"] = "FRTK   "; break;
		    case 2:  doc["ftype"] = "RTK     "; break;
		  }
		  break;
	  }
	  case 4: doc["ftype"] = "GNSS    "; break;
	  case 5: doc["ftype"] = "TIME    "; break;
	}

	char tod[24];
	int   hh = gnssRcvr.getHour(_MAX_WAIT);
	int   mm = gnssRcvr.getMinute(_MAX_WAIT);
	int   ss  = gnssRcvr.getSecond(_MAX_WAIT);
	sprintf(tod, "%02d:%02d:%02d", hh, mm, ss);

	doc["time"] = tod;

    // Add an array of values/
	JsonArray data = doc.createNestedArray("data");

	int rssiAccumulator = 0.0;
	for (uint8_t n=0; n<_RSSI_AVERAGE_WINDOW; n++) {
	  rssiAccumulator += rssi[n];
	}

	double packetLoss = 0.0;
	if (loraPacketCount>0)
	   packetLoss = 100.0* (1.0 - ((double)rtcmPacketPushedCount/(double)loraPacketCount));

	data.add(packetLoss);																											//packetloss %
	data.add(rssiAccumulator/_RSSI_AVERAGE_WINDOW);													//rssi
	data.add(gnssRcvr.getSIV(_MAX_WAIT));																			//nsats
	data.add(gnssRcvr.getPositionAccuracy(_MAX_WAIT)/10.0);											//haccy cm

	serializeJson(doc, Serial1);
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

// GNSS CALLBACKS - START
// xGPGGA, GPGLL, xGPVTG, xGPRMC

void printGPGGA(NMEA_GGA_data_t *nmeaData) {
//#if defined(_CALLBACK_DEBUG)
    Serial.print(F("\r\nGPGGA: Length: "));
    Serial.print(nmeaData->length);
    Serial.print(F("\tData: "));
    Serial.print((const char *)nmeaData->nmea); // .nmea is printable (NULL-terminated) and already has \r\n on the end
//#endif
//    Serial1.print((const char *)nmeaData->nmea); // .nmea is printable (NULL-terminated) and already has \r\n on the end
}

// Callback: printGNGGA will be called if new GNGGA NMEA data arrives
// See u-blox_structs.h for the full definition of NMEA_GGA_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setNMEAGNGGAcallback
//        /               _____  This _must_ be NMEA_GGA_data_t
//        |              /           _____ You can use any name you like for the struct
//        |              |          /
//        |              |          |
void printGNGGA(NMEA_GGA_data_t *nmeaData) {
#if defined(_CALLBACK_DEBUG)
    Serial.print(F("\r\nGNGGA: Length: "));
    Serial.print(nmeaData->length);
    Serial.print(F("\tData: "));
    Serial.print((const char *)nmeaData->nmea); // .nmea is printable (NULL-terminated) and already has \r\n on the end
#endif
    Serial1.print((const char *)nmeaData->nmea); // .nmea is printable (NULL-terminated) and already has \r\n on the end
}

void printGPVTG(NMEA_VTG_data_t *nmeaData) {
#if defined(_CALLBACK_DEBUG)
	Serial.print(F("\r\nGPVTG: Length: "));
	Serial.print(nmeaData->length);
	Serial.print(F("\tData: "));
	Serial.print((const char *)nmeaData->nmea); // .nmea is printable (NULL-terminated) and already has \r\n on the end
#endif
	Serial1.print((const char *)nmeaData->nmea); // .nmea is printable (NULL-terminated) and already has \r\n on the end
}

void printGPRMC(NMEA_RMC_data_t *nmeaData) {
#if defined(_CALLBACK_DEBUG)
	Serial.print(F("\r\nGPRMC: Length: "));
	    Serial.print(nmeaData->length);
	    Serial.print(F("\tData: "));
	    Serial.print((const char *)nmeaData->nmea); // .nmea is printable (NULL-terminated) and already has \r\n on the end
#endif
	    Serial1.print((const char *)nmeaData->nmea); // .nmea is printable (NULL-terminated) and already has \r\n on the end
}

void pvtCallback(UBX_NAV_PVT_data_t *ubxDataStruct) {
	//Serial.println("pvtCallback");
	/*
	long longitude = ubxDataStruct->lon; // Print the longitude
	Serial.print(F(" Long: "));
	Serial.print(longitude);
	Serial.print(F(" (degrees * 10^-7)"));
	*/

	long groundSpeed = ubxDataStruct->gSpeed; // Print the ground speed
	Serial.print(F(" Speed: "));  Serial.print(groundSpeed); 	Serial.println(F(" mm/sec)"));

	/*
	 *  Raven Speed Sensor  Timer  1 cycle       0.03937inch       1 foot             mm
	 *                                                      ------   x   ------------   x    ---------    x    -------   =  cycles/sec
	 *                                                      foot                mm              12 inch            sec
	 */
#define FREQ_CONVERSION 0.03937/12
}

void newRAWX(UBX_RXM_RAWX_data_t *ubxDataStruct)
{
  Serial.println();

  Serial.print(F("New RAWX data received. It contains "));
  Serial.print(ubxDataStruct->header.numMeas); // Print numMeas (Number of measurements / blocks)
  Serial.println(F(" data blocks:"));

  for (uint8_t block = 0; block < ubxDataStruct->header.numMeas; block++) // For each block
  {
    Serial.print(F("GNSS ID: "));
    if (ubxDataStruct->blocks[block].gnssId < 100) Serial.print(F(" ")); // Align the gnssId
    if (ubxDataStruct->blocks[block].gnssId < 10) Serial.print(F(" ")); // Align the gnssId
    Serial.print(ubxDataStruct->blocks[block].gnssId);
    Serial.print(F("  SV ID: "));
    if (ubxDataStruct->blocks[block].svId < 100) Serial.print(F(" ")); // Align the svId
    if (ubxDataStruct->blocks[block].svId < 10) Serial.print(F(" ")); // Align the svId
    Serial.print(ubxDataStruct->blocks[block].svId);

    if (sizeof(double) == 8) // Check if our processor supports 64-bit double
    {
      // Convert prMes from uint8_t[8] to 64-bit double
      // prMes is little-endian
      double pseudorange;
      memcpy(&pseudorange, &ubxDataStruct->blocks[block].prMes, 8);
      Serial.print(F("  PR: "));
      Serial.print(pseudorange, 3);

      // Convert cpMes from uint8_t[8] to 64-bit double
      // cpMes is little-endian
      double carrierPhase;
      memcpy(&carrierPhase, &ubxDataStruct->blocks[block].cpMes, 8);
      Serial.print(F(" m  CP: "));
      Serial.print(carrierPhase, 3);
      Serial.print(F(" cycles"));
    }
    Serial.println();
  }
}
// GNSS CALLBACKS - END

/*
// RTCM 3.2 bytes look like this:
// Byte 0: Always 0xD3
// Byte 1: 6-bits of zero
// Byte 2: 10-bits of length of this packet including the first two-ish header bytes, + 6.
// byte 3 + 4 bits: Msg type 12 bits
// Example: D3 00 7C 43 F0 ... / 0x7C = 124+6 = 130 bytes in this packet, 0x43F = Msg type 1087
 */
struct rtcm_v3_preamble {
       uint8_t flag;
       uint8_t len_hi;
       uint8_t len_lo;
       uint8_t msgtype_hi;
       uint8_t msgtype_lo;
       uint8_t payload[1024];
       uint8_t crc[3];
   };
typedef struct rtcm_v3_preamble Rtcm;
Rtcm rtcmMsg;

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// disable interrupt when it's not needed
volatile bool enableInterrupt = true;

#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif

 // this function is called when a complete packet  is received by the module
 // IMPORTANT: this function MUST be 'void' type  and MUST NOT have any arguments!
void setFlag(void) {
  // check if the interrupt is enabled
  if(!enableInterrupt) {
    return;
  }
  // we got a packet, set the flag
  receivedFlag = true;
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

	    Serial.print("LoRa Packets  = "); Serial.print(loraPacketCount);
	    Serial.print("  RTCM Packets = "); Serial.print(rtcmPacketCount);
	    Serial.print("  RTCM Pushed = "); Serial.println(rtcmPacketPushedCount);

	    return fs;
}

void rtcmQueue() {
  while(rtcmBufferIdxPrev!=rtcmBufferIdx) {
	  /*
	  uint16_t len = rtcmBuffer[rtcmBufferIdxPrev][0];
	  for (uint8_t n=1; n<len; n++) {   // print data of the packet
	  	     if (rtcmBuffer[rtcmBufferIdxPrev][n]<16)
	  	  	    Serial.print("0");
             Serial.print(rtcmBuffer[rtcmBufferIdxPrev][n], HEX);
	  }
	  Serial.println();
	  */

	  if (deviceConfig.pushRtcmPackets) {
	      boolean ok = gnssRcvr.pushRawData(&rtcmBuffer[rtcmBufferIdxPrev][1], rtcmBuffer[rtcmBufferIdxPrev][0], true);							 //Push RTCM to GNSS module over I2C
	      if (ok==true) {
	         //Serial.print("#");
	    	 rtcmPacketPushedCount += 1;
	      }  else {
	         Serial.println(F("RTCM packet pushed to ZED-F9P with failures."));
	      }
      }
      rtcmBufferIdxPrev += 1;
      if(rtcmBufferIdxPrev>=_RTCM_BUFFERS)
    	  rtcmBufferIdxPrev = 0;
  }
}

void setup()
{
	// BLE SETUP - START
	/*
	if (!BLE.begin()) {
	    Serial.println("starting BLE failed!");
	    while (1);
	}
	BLE.setLocalName("ROVER_2");
	//BLE.setAdvertisedService(batteryService); // add the service UUID
	//batteryService.addCharacteristic(batteryLevelChar); // add the battery level characteristic
	//BLE.addService(batteryService); // Add the battery service
	//batteryLevelChar.writeValue(oldBatteryLevel); // set initial value for this characteristic
    BLE.advertise();
	  */
	// BLE SETUP - END

  Serial.begin(115200);
#if defined (_DEBUG)
  while (!Serial); 		//Wait for user to open terminal
#endif
  Serial.println("u-blox Rover Station");

  Wire.begin();
  Wire.setClock(400000); 								//Increase I2C clock speed to 400kHz

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

  Serial.print(F("SX1262 Radio  Initializing ... "));
  int state = init_SX1262(deviceConfig.channel);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  radio.setDio1Action(setFlag);						// set the function that will be called  when a new LoRa packet is received

  Serial.print("... ");
  state = radio.startReceive();						// start listening for LoRa packets
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("LoRa Receiver started!"));
  } else {
    Serial.print(F("LoRa Receiver failed to start!"));
    Serial.println(state);
    while (true);
  }

  // GNSS SETUP - START
  if (gnssRcvr.begin() == false) 	  {				//Connect to the u-blox module using Wire port
    Serial.println(F("u-blox ZED-F9P not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  //??? gnssRcvr.setNavigationFrequency(deviceConfig.navRateHz); 	//Set Nav Rate in Hz.

  gnssRcvr.setUSBOutput(deviceConfig.usbProtocolFlag);			//Set the protocols on the ZED-F9P USB port

  //gnssRcvr.setAutoPVT(true, false, _MAX_WAIT);                           // Enable/disable automatic PVT reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  //gnssRcvr. setAutoPVTrate(.5,true, _MAX_WAIT);               			 // Set the rate for automatic PVT reports
  // gnssRcvr.setAutoPVTcallbackPtr(&pvtCallback);
  //gnssRcvr.setAutoRXMRAWXcallbackPtr(&newRAWX); // Enable automatic RXM RAWX messages with callback to newRAWX
  //gnssRcvr.setNMEAGPGGAcallbackPtr(&nmeaCallback);

  gnssRcvr.setHighPrecisionMode(true); // Enable High Precision Mode - include extra decimal places in the GGA messages
  gnssRcvr.setI2COutput(COM_TYPE_NMEA | COM_TYPE_UBX); // Turn on both UBX and NMEA sentences on I2C. (Turn off RTCM and SPARTN)
/*
   gnssRcvr.enableNMEAMessage(UBX_NMEA_GSA, COM_PORT_I2C);
   gnssRcvr.enableNMEAMessage(UBX_NMEA_GSV, COM_PORT_I2C);
   gnssRcvr.enableNMEAMessage(UBX_NMEA_RMC, COM_PORT_I2C);
   gnssRcvr.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_I2C);   // Leave only GGA enabled at current navigation rate
   gnssRcvr.enableNMEAMessage(UBX_NMEA_VTG, COM_PORT_I2C);
   gnssRcvr.enableNMEAMessage(UBX_NMEA_GLL, COM_PORT_I2C);     // Several of these are on by default on ublox board so let's disable them
*/
  //we'll send the NMEA messages from UART2 on the ZED-F9P RTK2 board so we don't get bogged down
  //On the NMEA side? xGSA, xGBS, xGST, xGRS perhaps?
  // xGGA,x GST, xRMC, xVTG, ZDA
  /*
  gnssRcvr.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART2);
  gnssRcvr.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART2);
  gnssRcvr.disableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART2);
  gnssRcvr.disableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART2);
  gnssRcvr.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART2);
  gnssRcvr.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART2);
  gnssRcvr.disableNMEAMessage(UBX_NMEA_GBS, COM_PORT_UART2);
  gnssRcvr.disableNMEAMessage(UBX_NMEA_GST, COM_PORT_UART2);
  gnssRcvr.disableNMEAMessage(UBX_NMEA_GRS, COM_PORT_UART2);
  gnssRcvr.disableNMEAMessage(UBX_NMEA_ZDA, COM_PORT_UART2);
  */

  /*  FieldBee Spec: NMEA0183 messages: GGA, GST, RMC, VTG, ZDA. !The GSA message has been planned to be added in Q2 2022!
  To receive correction from FieldBee RTK Base station, the antenna needs to receive NMEA messages in RTCM x3 format, also receive GGA, VTG messages, update rate can be 5 or 10 Hz. For autosteer, it is needed to set up a baud rate for the 115200.

  Possible connection:
  GGA at 5Hz -yes
  VTG at 5Hz – yes
  GSA at 1Hz -yes
  GSV at 1Hz – yes
  GST at 1Hz – yes
  ZDA at 1Hz – yes

  Baud Rate of 115200 – yes
  */

  /* #1 NMEA GGA and VTG sentences only! */
  //first disable ALL
  //gnssRcvr.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART2);
  //gnssRcvr.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART2);
  //gnssRcvr.disableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART2);
  //gnssRcvr.disableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART2);
  //gnssRcvr.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART2);
  //gnssRcvr.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART2);
  //gnssRcvr.disableNMEAMessage(UBX_NMEA_GBS, COM_PORT_UART2);
  //gnssRcvr.disableNMEAMessage(UBX_NMEA_GST, COM_PORT_UART2);
  //gnssRcvr.disableNMEAMessage(UBX_NMEA_GRS, COM_PORT_UART2);
  //gnssRcvr.disableNMEAMessage(UBX_NMEA_ZDA, COM_PORT_UART2);
  //now do the enables
  //gnssRcvr.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART2, 5);
  //gnssRcvr.enableNMEAMessage(UBX_NMEA_GST, COM_PORT_UART2, 1);
  //gnssRcvr.enableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART2, 1);
  //gnssRcvr.enableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART2, 5);
  //gnssRcvr.enableNMEAMessage(UBX_NMEA_ZDA, COM_PORT_UART2, 1);

  //gnssRcvr.enableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART2); 			//2Q 2022?

  //gnssRcvr.enableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART2);
  //gnssRcvr.enableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART2);
  //gnssRcvr.enableNMEAMessage(UBX_NMEA_GBS, COM_PORT_UART2);
  //gnssRcvr.enableNMEAMessage(UBX_NMEA_GRS, COM_PORT_UART2);

  // gnssRcvr.setMainTalkerID(SFE_UBLOX_MAIN_TALKER_ID_GP);					 // Set the Main Talker ID to "GP". The NMEA GGA messages will be GPGGA instead of GNGGA
  gnssRcvr.setMainTalkerID(SFE_UBLOX_MAIN_TALKER_ID_DEFAULT); 				 // Uncomment this line to restore the default main talker ID

  //gnssRcvr.saveConfiguration(VAL_CFG_SUBSEC_IOPORT | VAL_CFG_SUBSEC_MSGCONF); //Optional: Save only the ioPort and message settings to NVM

  Serial.println(F("NMEA messages configured  for output on ZED-F9P UART2/Bluetooth port."));

  //gnssRcvr.setSerialRate(115200, COM_PORT_UART2, _MAX_WAIT);

  //gnssRcvr.setNMEAOutputPort(Serial1); // echo all NMEA data to Serial for debugging. This is slow.

   /* Set up the callbacks
  gnssRcvr.setNMEAGPGGAcallbackPtr(&printGPGGA);
  gnssRcvr.setNMEAGPGGAcallbackPtr(&printGPGGA);
  gnssRcvr.setNMEAGNGGAcallbackPtr(&printGNGGA);
  gnssRcvr.setNMEAGPVTGcallbackPtr(&printGPVTG);
  gnssRcvr.setNMEAGPRMCcallbackPtr(&printGPRMC);
  */

  Serial.print("Rover listening for RTCM packets over LoRa P2P(PHY) at "); Serial.print(US915_STARTFREQ + deviceConfig.channel * 0.2); Serial.println("MHz");
  // GNSS SETUP - END

  //CRAFTPORT SETUP - START
      Serial1.begin(57600);
      term.setSerialEcho(false);
      term.addCommand("help", getHelp);
      term.addCommand("config", getConfig);
      term.addCommand("save", saveConfigChanges);
      term.addCommand("discard", discardConfigChanges);
      term.addCommand("reboot", reboot);
      term.addCommand("info", getInfo);
      term.addCommand("channel",setChannel);
      term.addCommand("callsign",setCallSign);
      term.addCommand("outputs",getStatus);
#if defined( _ROVER)
      term.addCommand("rtcmpush",setRtcmPush);
      term.addCommand("navrate", setNavRateHz);
      term.addCommand("usbprot", setUsbProtocols);
#else
      term.addCommand("rtcmmsecs", setRtcmSendIntervalMillis);
      term.addCommand("survey",startSurveyIn);
      term.addCommand("hsvin",haltSurveyIn);
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
      rtcmQueue();

      //  gnssRcvr.checkCallbacks(); 							// Check if any callbacks are waiting to be processed.

      if ((millis() - lastDashboardMillis) > _DASHBOARD_UPDATE_MILLIS) {
    	  dashboardJson();
    	  lastDashboardMillis = millis();
      }

      if ((millis() - lastUbloxCheckMillis) > _UBLOX_CHECK_MILLIS) {
     	upTimeSecs += _UBLOX_CHECK_MILLIS/1000;
         gnssRcvr.checkUblox(); 								//See if new data is available. Process bytes as they come in.
         lastUbloxCheckMillis = millis();

       }

 	  if(receivedFlag) {							// check if the flag is set
	    enableInterrupt = false;			// disable the interrupt service routine while processing the data
	    receivedFlag = false;				// reset flag
	    loraPacketCount += 1;

	     uint16_t len = radio.getPacketLength();
	     byte byteArr[len];
	     int state = radio.readData(byteArr, len);

		 rssi[rssiAvgPtr++] =  radio.getRSSI();
		 if (rssiAvgPtr ==   _RSSI_AVERAGE_WINDOW)
		 	rssiAvgPtr = 0;

	     radio.startReceive();				// put module back to listen mode
	     enableInterrupt = true;		// we're ready to receive more packets,  enable interrupt service routine
         //memcpy(&rtcmMsg, byteArr, len);

	     if  (byteArr[0]==0xD3) {					 // check that packet is RTCM
	    	//queue the packet
	    	rtcmBufferIdx += 1;
	    	   if (rtcmBufferIdx>=_RTCM_BUFFERS)
	    	      rtcmBufferIdx = 0;

            rtcmBuffer[rtcmBufferIdx][0] = len;
            memcpy(&rtcmBuffer[rtcmBufferIdx][1], byteArr, len);

            rtcmPacketCount += 1;
	     } else {
	    	 Serial.println();
	    	 for (uint8_t n=0; n<len; n++) {   // print data of the packet
	    	 	  	     if (byteArr[n]<16)
	    	 	  	  	    Serial.print("0");
	    	              Serial.print(byteArr[n], HEX);
	    	 	  }
	    	 	  Serial.println();
	     }


	  }

 	 if ((millis() - lastPrintFixTypeMillis)> 30000) {
 		 FixStats fs =  printFixType();
 		rtcmPacketCountPeriod = rtcmPacketCount;
 		 rtcmPacketCount = 0;
 		rtcmPacketPushedCountPeriod = rtcmPacketPushedCount;
 		 rtcmPacketPushedCount = 0;
 		 loraPacketCountPeriod =  loraPacketCount;
 		 loraPacketCount = 0;
 		 lastPrintFixTypeMillis = millis();
 	  }

}
