// ESP32 code for autosteer unit for AgOpenGPS

// ready for AOG V4.3 + V5.x Version
// PINs for GormR central unit PCB (V1.8) see GitHub https://github.com/GormR/HW_for_AgOpenGPS
// by MTZ8302 see GitHub https://github.com/mtz8302 and Youtube Ma Ha MTZ8302 https://www.youtube.com/channel/UCv44DlUXQJKbQjzaOUssVgw

// Simplified by charliewprice

#define AOG_HELLO_TIMEOUT_MILLIS 30000
#define WAS_LOOP_TIME  20  //50Hz
#define DATA_LOOP_TIME 110 //110 //10hz + 10ms to wait for AOG data
#define WATCHDOG_TIMEOUT 200

static uint8_t ipDest[] = { 255,255,255,255 };
uint16_t portDest = 9999; //AOG port that listens

byte vers_nr = 44;
char VersionTXT[120] = " - 16. April 2022 by charliewprice<br>(V5.x, BNO085, WiFi)";

#define useLED_BUILTIN  0            // some ESP board have a build in LED, some not. Here it's the same funtion as the WiFi LED
#define BUZZER_GPIO     25
#define PWMBUZ_Ch       5
#define PWMBUZ_Res      8
#define PWMBUZ_Freq     1440

//#define USE_LCD_DISPLAY
 
int PWMBUZ_DutyCycle = 128;

//#define HOME_NETWORK				  // flag to use home network instead of SoftAP
  
struct Storage {
  //WiFi
  #if defined(HOME_NETWORK)
  char ssid1[24] = "linville5";       // WiFi network that this SteerModule will join
  char password1[24] = "thr3adtr4il"; // WiFi network password
  #else
  char ssid1[24] = "RoverNet";            // WiFi network that this SteerModule will join
  char password1[24] = "74777477";            // WiFi network password
  #endif
  byte timeoutWebIO = 255;
  uint16_t timeoutRouter = 1800;  
  byte WiFi_myip[4] = { 192, 168, 1, 77 };      // autosteer module 
  byte WiFi_gwip[4] = { 192, 168, 1, 1 };       // Gateway IP only used if Accesspoint created
  byte WiFi_ipDest_ending = 255;                  // ending of IP address to send UDP data to
  byte mask[4] = { 255, 255, 255, 0 };
  byte myDNS[4] = { 8, 8, 8, 8 };               //optional

  unsigned int PortAutostToAOG = 5577;          // this is port of this module: Autosteer = 5577 IMU = 5566 GPS = 
  unsigned int PortFromAOG = 8888;              // port to listen for AOG
  unsigned int PortDestination = 9999;          // port of AOG that listens

  //general settings
  uint8_t aogVersion = 20;                      // Version number for version check 4.3.10 = 4+3+10 = 17 
                                                // 5.7.2 14  
  
  uint8_t output_type = 1;      // set to 1  if you want to use Steer Motor + Cytron MD30C Driver
 
  uint16_t PWMOutFrequ = 20000;                 // PWM frequency for motordriver: 1000Hz:for low heat at PWM device 20000Hz: not hearable
  uint8_t MotorDriveDirection = 0;              // 0 = normal, 1 = inverted
  uint8_t MotorSlowDriveDegrees = 5;            // How many degrees before decreasing Max PWM
  uint8_t WASType = 2;          // 0 = No ADS installed, Wheel Angle Sensor connected directly to ESP at GPIO 36 (pin set below) (attention 3,3V only)
                                // 1 = Single Mode of ADS1115 - Sensor Signal at A0 (ADS)
                                // 2 = Differential Mode - Connect Sensor GND to A1, Signal to A0

  //IMU on standalone processor
  uint8_t IMUType = 0;          // 0: none, 3: BNO085

  // BNO08x
  uint8_t bno08xAddresses[2] = { 0x4A,0x4B };   // BNO08x address variables to check where it is
  float BNOHeadingCorrection = 0.0;             // not used at the moment
  float BNORollCorrection = 0.0;                // not used at the moment
  uint8_t BNOUsedAxis = 0;                      // not used at the moment

  uint8_t InvertRoll = 0;                       // 0: no, set to 1 to change roll direction
  uint8_t InvertWAS = 0;                        // set to 1 to Change Direction of Wheel Angle Sensor - to + 

  uint8_t ShaftEncoder = 0;                     // Steering Wheel ENCODER Installed
  uint8_t PressureSensor = 0;                   // (not supported at the moment)
  uint8_t CurrentSensor = 0;                    // (not supported at the moment)
  uint8_t pulseCountMax = 3;                    // Switch off Autosteer after x Pulses from Steering wheel encoder 

  uint16_t WebIOSteerPosZero = 10300;           // first value for steer zero position ADS: 11000 for EPS32 AD PIN: 2048

  uint8_t AckermanFix = 78;                     // if values for left and right are the same: 100 

  uint8_t SteerSwitchType = 1;        // 0 = enable = switch high (3,3V) //1 = enable = switch low(GND) //2 = toggle = button to low(GND)
                                      // 3 = enable = button to high (3,3V), disable = button to low (GND), neutral = 1,65V
                                      // 255 = no steer switch, allways on if AOG steering is active

  uint8_t WorkSW_mode = 2;                      // 0 = disabled // 1 = digital ON/OFF // 2 = analog Value 0...4095 (0 - 3,3V)

  uint8_t Invert_WorkSW = 0;                    // 0 = Hitch raised -> High    // 1 = Hitch raised -> Low

  float autoSteerMinSpeed = 0.2;                // Min speed to use autosteer km/h
  float autoSteerMaxSpeed = 30;                 // Max speed to use autosteer km/h

  uint16_t WorkSW_Threshold = 1600;             // Value for analog hitch level to switch workswitch  (0-4096)

  // IO pins ------------------------------------------------------------------
  // set to 255 for unused !!!!!
  uint8_t SDA = 21;                   // I2C Pins
  uint8_t SCL = 22;

  uint8_t AutosteerLED_PIN = 2;       // light on active autosteer and IBT2
  uint8_t LEDWiFi_PIN = 0;            // light on WiFi connected, flashes on searching Network. If GPIO 0 is used LED must be activ LOW otherwise ESP won't boot
  uint8_t LEDWiFi_ON_Level = LOW;     // HIGH = LED on high, LOW = LED on low

  // (not supported at the moment) relays for section control
  uint8_t Relay_PIN[16] = { 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255 }; 
  uint8_t Tram_PIN[3] = { 255,255,255 };  // (not supported at the moment) relais for tramline control
  uint8_t Relays_ON = HIGH;           // HIGH = Relay on high, LOW = Relay on low

  uint8_t WAS_PIN = 36;               // PIN for Wheel Angle Sensor (none, if ADS used)
  uint8_t WAS_Diff_GND_PIN = 39;
  uint8_t WORKSW_PIN = 33;            // PIN for workswitch (can be analog or on/off switch see WorkSW_mode)
  uint8_t STEERSW_PIN = 34;           // Pin for steer button or switch (see SteerSwitchType)
  uint8_t encA_PIN = 4;               // Pin for steer encoder, to turn off autosteer if steering wheel is used
  uint8_t encB_PIN = 32;              // Pin for steer encoder, to turn off autosteer if steering wheel is used

  uint8_t Servo_PIN = 16;             // (not supported at the moment) Pin for servo to pull motor to steering wheel

  uint8_t PWM_PIN = 27;               // PWM Output to motor controller (IBT2 or cytron)
  uint8_t DIR_PIN = 26;               // direction output to motor controller (IBT2 or cytron)

  uint8_t Current_sens_PIN = 35;      // (not supported at the moment) current sensor for IBT2 to read the force needed to turn steering wheel

  //uint8_t Eth_CS_PIN = 5;             // CS PIN with SPI Ethernet hardware  SPI config: MOSI 23 / MISO 19 / CLK18 / CS5

  uint8_t CAN_RX_PIN = 25;            // (not supported at the moment) CAN bus 
  uint8_t CAN_TX_PIN = 17;            // (not supported at the moment)

  //##########################################################################################################
  //### End of Setup Zone ####################################################################################
  //##########################################################################################################
  //filter variables set by AOG via PGN settings sentence
  float Ko = 0.05f;     //overall gain  
  float Kp = 20.0f;     //proportional gain  
  float Ki = 0.001f;    //integral gain
  float Kd = 1.0f;      //derivative gain 
  float AOGSteerPositionZero = 0;
  float steerSensorCounts = 100;
  uint16_t roll_corr = 200;
  byte minPWM = 40, highPWM = 150, lowPWM = 60;

  bool debugmode = false;
  bool debugmodeDataFromAOG = false;

};  Storage Set;

boolean EEPROM_clear = false;  //set to true when changing settings to write them as default values: true -> flash -> boot -> false -> flash again

// sentences to AOG V4.6 and up only 
const byte FromAOGSentenceHeader[3] = { 0x80,0x81,0x7F };

//the following are the PGN ids:

//From AutoSteer 	7E 	126 	FD 	253 	8 	ActualSteerAngle * 100 	IMU Heading Hi/Lo 	IMU Roll Hi/Lo 	Switch 	PWMDisplay 	CRC
#define steerDataToAOGPGN  			0xFD
//Steer Data 		7F 	127 	FE 	254 	8 	Speed 	Status 	steerAngle 	xte 	SC1to8 	SC9to16 	CRC
#define steerDataFromAOGPGN  		0xFE
//Steer Config 		7F 	127 	FB 	251 	8 	set0 	pulseCount 	minSpeed 	sett1 	*** 	*** 	*** 	*** 	CRC
#define steerArdConfFromAOGPGN 		0xFB
//Steer Settings 	7F 	127 	FC 	252 	8 	gainP 	highPWM 	lowPWM 	minPWM 	countsPerDeg 	steerOffset 	ackermanFix 	CRC
#define steerSettingsFromAOGPGN  	0xFC
//Hello 			7F 	127 	C8 	200 	3 	Module ID 	0 	0 	CRC
#define helloFromAogPGN				      	0xC8
//Scan request 		7F 	123 	CA 	202 	3 	202 	202 	5 	CRC
#define scanFromAogPGN					      0xCA

#define steerDataSentenceToAOGLength  	14

//global
byte incomSentenceDigit = 0,DataToAOGLength;
bool isSteerDataFound = false, isSteerSettingFound = false, isSteerArdConfFound = false, isSteerArdConfigFound = false;
bool isSteerDataFoundV17 = false, isSteerSettingFoundV17 = false, isSteerArdConfFoundV17 = false, isSteerArdConfigFoundV17 = false;

//hello + scan 
boolean helloFromAog = false;
boolean scanFromAog = false;

#define incommingDataArraySize 5
byte incommingBytes[incommingDataArraySize][500];
byte incommingBytesArrayNr = 0;
byte incommingBytesArrayNrToParse = 0;
unsigned int incommingDataLength[incommingDataArraySize] = { 0,0,0,0,0 };
#define SentenceFromAOGMaxLength 14
byte SentenceFromAOG[SentenceFromAOGMaxLength], SentenceFromAOGLength;

byte steerToAOG[14] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

//libraries 
#include "EEPROM.h"
#include <Update.h>
#include "Wire.h"
#include "zADS1115.h"
#include <WiFiUdp.h>
#include <WebServer.h>
#include <WiFiClient.h>
#include <WiFi.h>
//#include "BNO08x_AOG.h"
#if defined(USE_LCD_DISPLAY)
#include <LiquidCrystal_I2C.h>
#endif

// Instances
ADS1115_lite adc(ADS1115_DEFAULT_ADDRESS);     // Use this for the 16-bit version ADS1115
//BNO080 bno08x;

WiFiUDP WiFiUDPFromAOG;
WiFiUDP WiFiUDPToAOG;
WebServer WiFi_Server(80);

#if defined(USE_LCD_DISPLAY)
// set the LCD columns, rows, and I2C address
int lcdColumns = 16;
int lcdRows = 2;
LiquidCrystal_I2C lcd(0x20, lcdColumns, lcdRows);
#define LCD_UPDATE_MILLIS 1000
long lastLcdUpdateMillis;
#endif

TaskHandle_t taskHandle_WiFi_connect;
TaskHandle_t taskHandle_DataFromAOGWiFi; 
TaskHandle_t taskHandle_WebIO;
TaskHandle_t taskHandle_LEDBlink;

bool WiFiDataTaskRunning = false;

// WiFi status LED blink times: searching WIFI: blinking 4x faster; connected: blinking as times set; data available: light on; no data for 2 seconds: blinking
unsigned long LED_WIFI_time = 0, DataFromAOGTime = 0;
#define LED_WIFI_pulse 1000   //light on in ms 
#define LED_WIFI_pause 700    //light off in ms
boolean LED_WIFI_ON = false;

unsigned long WebIOTimeOut = 0, WiFi_network_search_timeout = 0;
byte WiFi_connect_step = 10, WiFi_STA_connect_call_nr = 1, WiFi_netw_nr = 0, my_WiFi_Mode = 0; // WIFI_STA = 1 = Workstation  WIFI_AP = 2  = Accesspoint
IPAddress WiFi_ipDestination; //set in network.ino
bool WebIORunning = true;
bool WiFiUDPRunning = false;
bool newDataFromAOG = false;

//loop time variables in milliseconds

unsigned long lastDataLoopMillis = 100;
unsigned long now = 100;
unsigned long lastWasLoopMillis = 100;
byte watchdogTimer = 0;

//program flow
int AnalogValue = 0;
bool steerEnable = false;
bool toggleSteerEnable = false;
bool SteerButtonPressed = false;
bool steerEnableOld = false;
bool remoteSwitchPressed = false;
byte guidanceStatus = 0;
byte workSwitch = 0;
byte workSwitchOld = 0;
byte steerSwitch = 1;
byte switchByte = 0;
float gpsSpeed = 0;
float distanceFromLine = 0; 

//steering variables
float steerAngleActual = 0;
float steerAngleSetPoint = 0;
float steerAngleError = 0; //setpoint - actual
long steeringPosition = 0;
long actualSteerPosRAW = 0; //from steering sensor steeringPosition_corr = 0,
int  pulseCount = 0;
int prevEncAState = 0;
int prevEncBState = 0;    // Steering Wheel Encoder
bool encDebounce = false; // Steering Wheel Encoder

//IMU, inclinometer variables
int16_t roll16 = 0;
int16_t heading16 = 0;
uint16_t x_;
uint16_t y_;
uint16_t z_;
float roll = 0.0;
float lastRoll = 0.0;
float diff = 0.0;
float heading = 0.0;
float bno08xHeading = 0.0;
double bno08xRoll = 0.0;
double bno08xPitch = 0.0;
int bno08xHeading10x = 0;
int bno08xRoll10x = 0;

// BNO08x address variables
//int nrBNO08xAdresses = sizeof(Set.bno08xAddresses) / sizeof(Set.bno08xAddresses[0]);
//byte bno08xAddress;
byte REPORT_INTERVAL;

//Kalman variables
float Pc = 0.0, G = 0.0, P = 1.0, Xp = 0.0, Zp = 0.0;
float rollMMA = 0;
const float varRoll = 0.1; // variance,
const float varProcess = 0.001; //0.00025 smaller is more filtering

//pwm variables
int pwmDrive = 0;
int pwmDisplay = 0;
int pwmOut = 0;
int errorAbs = 0;
int highLowPerDeg = 0;
float pValue = 0; // iValue = 0, dValue = 0;drive = 0,

//Relays + Tramlines
byte SectGrFromAOG[2] = { 0,0 }, Tram = 0;

//webpage
long argVal = 0;

//status display on LCD
unsigned int status = 0;

//steer status for display
byte steerStat;

long lastAogHelloMillis;


#if defined(USE_LCD_DISPLAY)
void lcdSplash() {
  //a little startup dazzle
  for (uint8_t col=0; col<16; col++){
    lcd.setCursor(col, 0);
    lcd.write(byte(0xFF));
    lcd.setCursor(15-col, 1);
    lcd.write(byte(0xFF));
    delay(75);
  }
  
  lcd.setCursor(0, 0);				// set cursor to first column, first row
  lcd.print("AutoSteer   ");		// print message
  lcd.setCursor(0, 1);
  lcd.print("STARTING....");
  lcd.setCursor(13,0);
  lcd.print("ver");
  lcd.setCursor(13,1);
  lcd.print("1.1");
  delay(1500);
  lcd.clear();
}

void lcdUpdate() {
  if ((millis() - lastLcdUpdateMillis)> LCD_UPDATE_MILLIS) {
      if (status%60==0){  //periodically clear the display
        lcd.clear();
      }       
      char hdg[6];
      //sprintf(hdg,"%03u", (int)(0.5 + BNO.euler.head/16.0));
      lcd.setCursor(0, 1); //col, row
      lcd.print(bno08xHeading);
      char rol[6];
      // #5 Readings on LCD should match those in AOG
      sprintf(rol,"%03i", (int)bno08xRoll);
      lcd.setCursor(4, 1); //col, row
      lcd.print(rol);
      char pit[6];
      // #5 Readings on LCD should match those in AOG
      sprintf(pit,"%03i", (int)bno08xPitch);
      lcd.setCursor(10, 1); //col, row
      lcd.print(pit);      
      char ang[6];
      sprintf(ang,"%3i", (int)steerAngleActual);
      lcd.setCursor(0,0); //col, row
      lcd.print(ang);
      char en[4];
      switch(digitalRead(Set.STEERSW_PIN)) {
        case 0: sprintf(en,"ARM%i", steerStat); break;
        case 1: sprintf(en,"LOCK"); break;
      }      
      lcd.setCursor(5,0); //col, row
      lcd.print(en);
      lcd.setCursor(14,0);
      char net[3];
      switch(WiFi_connect_step) {
        case 0:  sprintf(net, "Ok", WiFi_connect_step); break;
        default: sprintf(net, "%02X", WiFi_connect_step); break;
      }      
      lcd.print(net);
      lcd.setCursor(9,0);
      char lps[3];
      sprintf(lps, "%4.1f", (float)((millis() - lastAogHelloMillis)/1000.0));
      lcd.print(lps);      
      lcd.setCursor(15, 1); //col, row
      switch (status++%2) {
        case 0: lcd.write(0xFE); break;
        case 1: lcd.write(0xFF); break;
      } 
      lastLcdUpdateMillis = millis();
  }
}
#endif

/*
 * Buzzer for operator attention
 */
void beep(uint8_t n) {
  uint8_t nbeeps = 0;
  while(nbeeps++<n) {
    ledcWrite(PWMBUZ_Ch, 128);
    delay(50);
    ledcWrite(PWMBUZ_Ch, 0);
    delay(50);
  }
}

void setup() {  
  Serial.begin(230400); 							//init USB Serial Port
  Serial.println("\n=041023========================\n\rAgOpenGPS Autosteer (-o-)    cp");
  ledcSetup(PWMBUZ_Ch, PWMBUZ_Freq, PWMBUZ_Res);	//init buzzer
  ledcAttachPin(BUZZER_GPIO, PWMBUZ_Ch);  
  beep(3);			//sound buzzer on restart
  restoreEEprom();	//get EEPROM data
  delay(100);
  
  // setup the autosteer packet preamble..  
  steerToAOG[0] = FromAOGSentenceHeader[0];   //0x80
  steerToAOG[1] = FromAOGSentenceHeader[1];   //0x81
  steerToAOG[2] = FromAOGSentenceHeader[2];   //0x7F
  steerToAOG[3] = steerDataToAOGPGN;		  //0xFD
  steerToAOG[4] = steerDataSentenceToAOGLength - 6; //length of data = all - header - length - CRC
  DataToAOGLength = steerDataSentenceToAOGLength;
  incomSentenceDigit = 0;
    
  assignGPIOs_start_extHardware();				//set GPIOs

  #if defined(USE_LCD_DISPLAY)
  lcd.begin(16,2); 		// initialize LCD and activate backlight
  lcd.backlight();
  lcdSplash();
  #endif
      
  WiFi_connect_step = 10;  //step 10 = start WiFi connection
  //create a task to manage the WiFi connection
  xTaskCreate(WiFi_handle_connection, "WiFiConnectHandle", 3072, NULL, 1, &taskHandle_WiFi_connect);
  //create a task to get UDP data from AOG
  xTaskCreate(getDataFromAOGWiFi, "DataFromAOGHandleWiFi", 5000, NULL, 1, &taskHandle_DataFromAOGWiFi);
  //create a task to blink WiFi LED status - who cares?
  xTaskCreate(WiFi_LED_blink, "WiFiLEDBlink", 3072, NULL, 0, &taskHandle_LEDBlink);
  vTaskDelay(1000); //waiting for other tasks to start before starting loop()
  lastAogHelloMillis = millis();  
}

/*
 *  There are several operations in loop() that run periodically on a timer:
 *
 *     the panic handler	  (not critical, 30 seconds)
 *     the LCD update handler (not critical, 5 seconds)
 *     the WAS handler		  (determines the steering angle from the WAS every 20ms )
 *     the data loop          (builds and sends the autosteer packet to AOG every 100ms)*
 *
 *     *the data loop is also triggered if a packet is received from AOG 
 *
 *  Every pass of loop() makes a call to the PID engine and motor driver if steering is enabled.
 *     
 */
void loop() {
	

  if ((millis() - lastAogHelloMillis)> AOG_HELLO_TIMEOUT_MILLIS) {
    Serial.println("Comm timeout - initiating RESET");
    #if defined(USE_LCD_DISPLAY)
    lcd.setCursor(0, 0);
    lcd.print("PANIC"); 
    #endif
    beep(1);
    ESP.restart();
  }
  
  #if defined(USE_LCD_DISPLAY)
  if ((millis() - lastLcdUpdateMillis) > LCD_UPDATE_MILLIS) {
    lcdUpdate();
  }
  #endif

  /*
   * new packets arriving from AOG must be examined
   * - if packet contains instructions for SteerUnit they are performed by the parser
   */
  if (incommingDataLength[incommingBytesArrayNrToParse] != 0) { 
    parseDataFromAOG();    
  } else { 
    vTaskDelay(3);     
  }

  //parsing data from AOG may set some flags
  if (helloFromAog) {
    // Hello from AgIO
    //Serial.println("Hello Response to AgIO!");
    lastAogHelloMillis = millis();
    helloFromAog = false;
    incommingDataLength[incommingBytesArrayNrToParse] = 0;
    //static uint8_t ipDest[] = { 255,255,255,255 };
    //uint16_t portDest = 9999; //AOG port that listens
    int steer100x = (1 * steerAngleActual);  
   
    uint8_t helloReply[] = { 128, 129, 126, 126, 5, 
                             (byte) steer100x, (byte)(steer100x >> 8), 
                             (byte) pulseCount, (byte)(pulseCount >>8 ), 
                             switchByte, 0};
    // calculate the checksum
    int16_t CK_A = 0;
    for (uint8_t i = 2; i < sizeof(helloReply) - 1; i++) {
      CK_A = (CK_A + helloReply[i]);
    }
    helloReply[sizeof(helloReply)-1] = CK_A;  
  
    WiFiUDPToAOG.beginPacket(ipDest, portDest);
    WiFiUDPToAOG.write(helloReply, sizeof(helloReply));
    WiFiUDPToAOG.endPacket();    
  } else if (scanFromAog) {
    //Serial.println("Scan Request");
    scanFromAog = false;
    incommingDataLength[incommingBytesArrayNrToParse] = 0;
    //static uint8_t ipDest[] = { 192,168,1,255 };
    //uint16_t portDest = 9999; //AOG port that listens 
    /*    
    //hello from AgIO
                    uint8_t scanReply[] = { 128, 129, Eth_myip[3], 203, 7,
                        Eth_myip[0], Eth_myip[1], Eth_myip[2], Eth_myip[3], 
                        rem_ip[0],rem_ip[1],rem_ip[2], 23 };
    */        
    uint8_t scanReply[] = { 128, 129, 126, 203, 7, 
                            Set.WiFi_myip[0], Set.WiFi_myip[1], Set.WiFi_myip[2], Set.WiFi_myip[3], 
                            Set.WiFi_myip[0], Set.WiFi_myip[1], Set.WiFi_myip[2], 23};
    //06-07-23 from teensy thread -- IP address never appears in scan
    //uint8_t scanReply[] = { 128, 129, Set.WiFi_myip[3], 203, 7,
    //                        Set.WiFi_myip[0], Set.WiFi_myip[1], Set.WiFi_myip[2], Set.WiFi_myip[3], 
    //                        192,168,1, 23 };
    // calculate the checksum
    int16_t CK_A = 0;
    for (uint8_t i = 2; i < sizeof(scanReply) - 1; i++) {
      CK_A = (CK_A + scanReply[i]);
    }
    scanReply[sizeof(scanReply)-1] = CK_A;
    WiFiUDPToAOG.beginPacket(ipDest, portDest);
    WiFiUDPToAOG.write(scanReply, sizeof(scanReply));
    WiFiUDPToAOG.endPacket();    
  }

  /*
   * Check the steering wheel encoder
   */
  if (Set.ShaftEncoder == 1) {
    if ((digitalRead(Set.encA_PIN) != prevEncAState) && !encDebounce) { 
      pulseCount++; 
      encDebounce = HIGH; 
    }
    if ((digitalRead(Set.encB_PIN) != prevEncBState) && !encDebounce) { 
      pulseCount++; 
      encDebounce = HIGH; 
    }
  }

  /*
   * Check the steer enable contact
   */
  steerEnable = !digitalRead(Set.STEERSW_PIN);
  steerSwitch = steerEnable;
  if (steerEnableOld != steerEnable) {
    if (Set.debugmode) {
      if (steerEnable) { 
        Serial.println("Autosteer ON by Switch"); 
      } else { 
        Serial.println("Autosteer OFF by Switch B"); 
      }
    }      
    steerEnableOld = steerEnable;
  }  
     
  /*
   * Check that it is safe to engage autosteer
   *    - ground speed within range?
   *    - steering wheel encoder is quiet
   */
  if (steerEnable) {
  if ((!bitRead(guidanceStatus, 0)) || 
        (pulseCount >= Set.pulseCountMax) ||
        (gpsSpeed < Set.autoSteerMinSpeed) || 
        (gpsSpeed > Set.autoSteerMaxSpeed)) {
      
      if (!bitRead(guidanceStatus, 0))
        steerStat = 1;
      if (pulseCount >= Set.pulseCountMax)
        steerStat = 2;
      if (gpsSpeed < Set.autoSteerMinSpeed)
        steerStat = 3;
      if (gpsSpeed > Set.autoSteerMaxSpeed)
        steerStat = 4;
      
      steerEnable = false;
      if (steerEnable != steerEnableOld) {
        //Serial.println(" Steer-Break:  AOG not active or Speed too low or Steering Wheel Encoder..");
        if (Set.debugmode) { 
          Serial.println(" Steer-Break:  AOG not active or Speed too low or Steering Wheel Encoder.."); 
        }
        steerEnableOld = steerEnable;
      }
      pulseCount = 0;
      digitalWrite(Set.AutosteerLED_PIN, LOW); //turn LED off
    }
  } 
   
  /*
   * call the PID engine and motor driver
   */
  if ((steerEnable) && (watchdogTimer < WATCHDOG_TIMEOUT)) {
    steerStat = 0;
    digitalWrite(Set.AutosteerLED_PIN, HIGH);  //turn LED on (LED connected to MotorDriver = ON = Motor moves)
    //06-07-23 it seems wrong to calculate PID without a fresh sample of the steering angle
    steerAngleError = steerAngleActual - steerAngleSetPoint;   //calculate the steering error 
    calcSteeringPID();   //do the pid     
    motorDrive();        //output pwm to steer motor 
  } else {
    digitalWrite(Set.AutosteerLED_PIN, LOW);  //turn LED off 
    steerEnable = false;
    if ((steerEnable != steerEnableOld) && (watchdogTimer >= WATCHDOG_TIMEOUT)) {
      //we've lost the comm to AgOpenGPS
      if (Set.debugmode) { 
      Serial.println("Steer-Break: watch dog timer runs out, no Data from AOG"); 
    }
      steerEnableOld = steerEnable;
    }
    pwmDrive = 0; //turn off steering motor
    motorDrive(); //out to motors the pwm value   
    pulseCount = 0; //Reset counters if Autosteer is offline  
  }
   
  /*
   *  The WAS LOOP  
   *  calculate the steering angle
   *  ~20ms
   */   
  now = millis();
  if (now - lastWasLoopMillis >= WAS_LOOP_TIME)  {
    lastWasLoopMillis = now;
    //04-10-23 SetRelays() is causing GPIO error message:
    //E (32005) gpio: gpio_set_level(226): GPIO output gpio_num error
    //cwp-d SetRelays(); //turn on off sections, do in timed loop, if new data comes in
    encDebounce = LOW; //reset steerEncoder debounce
    //If connection lost to AgOpenGPS, the watchdog will count up and turn off steering
    if (watchdogTimer++ > 250) watchdogTimer = 250;
    //steering position and steer angle
    switch (Set.WASType) {
      case 1:  // ADS 1115 single
        adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0);
        steeringPosition = adc.getConversion();
        adc.triggerConversion();
        steeringPosition = steeringPosition >> 1; //divide by 2
        break;
      case 2:  // ADS 1115 differential
        adc.setMux(ADS1115_REG_CONFIG_MUX_DIFF_0_1);
        adc.triggerConversion();
        steeringPosition = adc.getConversion();
        steeringPosition = steeringPosition >> 1; //divide by 2
        //Serial.println(steeringPosition);      
        break;
    }
    actualSteerPosRAW = steeringPosition; // stored for >zero< Funktion
    //center the steering position sensor  
    steeringPosition = steeringPosition - Set.WebIOSteerPosZero - Set.AOGSteerPositionZero;
    //invert position, left must be minus
    if (Set.InvertWAS == 1) 
      steeringPosition *= -1;
    //Ackermann fix: correct non linear values = right side factor
    if (steeringPosition > 0) {
      steeringPosition = long((steeringPosition * Set.AckermanFix) / 100);
    }
    //convert position to steer angle
    steerAngleActual = ((float)(steeringPosition) / Set.steerSensorCounts);
    //Serial.println(steerAngleActual);
  } // WAS loop
  
  /*
   *  The DATA LOOP  ~100ms or on packets from AOG
   *  generate and send the autosteer packet to AOG
   */
  now = millis();  
  if ( ((now - lastDataLoopMillis) >= DATA_LOOP_TIME) || (newDataFromAOG) ) {
    newDataFromAOG = false;
    //Serial.println(now - lastDataLoopMillis); //rock solid at DATA_LOOP_TIME
    lastDataLoopMillis = now;
    
    //workswitch
    switch (Set.WorkSW_mode) {
      case 1:
        if (Set.Invert_WorkSW == 0) workSwitch = digitalRead(Set.WORKSW_PIN);    // read digital work switch
        if (Set.Invert_WorkSW == 1) workSwitch = !digitalRead(Set.WORKSW_PIN);    // read digital work switch
        break;
      case 2:
        AnalogValue = analogRead(Set.WORKSW_PIN);
        delay(2);
        AnalogValue += analogRead(Set.WORKSW_PIN);
        AnalogValue = AnalogValue >> 1;
        if (Set.Invert_WorkSW == 0) {
          if (AnalogValue < Set.WorkSW_Threshold)   
			workSwitch = 1;
          else 
			workSwitch = 0;
        }
        if (Set.Invert_WorkSW == 1) {
          if (AnalogValue > Set.WorkSW_Threshold)   
			workSwitch = 1;
          else 
			workSwitch = 0;
        }
        break;
    }	
    switchByte = 0;
    switchByte |= (!steerSwitch << 1); //put steerswitch status in bit 1 position
    switchByte |= workSwitch;

    //Build the Autosteer Packet
    //actual steer angle
    int temInt = (100 * steerAngleActual);
    //Serial.println(temInt);
    steerToAOG[5] = (byte)(temInt);
    steerToAOG[6] = (byte)(temInt >> 8);
    //Serial.print("SteerAngleActual=");     
	  // thereis no IMU on the Steer Unit, stuff packet with special bytes 
		//roll no hardware = 8888
		steerToAOG[9] = 0xB8;
		steerToAOG[10] = 0x22;
		roll = 0;
		//heading16 no hardware = 9999     
		steerToAOG[7] = 0x0F;
		steerToAOG[8] = 0x27;
		heading = 0;
    //switch byte
    steerToAOG[11] = switchByte;
    //pwm value
    if (pwmDisplay < 0) 
	    pwmDisplay *= -1;  
    steerToAOG[12] = pwmDisplay;
    //add the checksum
    int CRCtoAOG = 0;
    for (byte i = 2; i < sizeof(steerToAOG) - 1; i++) {
      CRCtoAOG = (CRCtoAOG + steerToAOG[i]);
    }	
    steerToAOG[sizeof(steerToAOG) - 1] = CRCtoAOG;
    //Build Autosteer Packet completed, send packet over UDP	
    if (WiFiUDPRunning) {
      WiFiUDPToAOG.beginPacket(WiFi_ipDestination, Set.PortDestination);
      WiFiUDPToAOG.write(steerToAOG, sizeof(steerToAOG));
      WiFiUDPToAOG.endPacket();
    } //send AutoSteer packet to AOG  
	
    vTaskDelay(1);		//all done give time to other tasks   
  } //end of  data loop  
}
