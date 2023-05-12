/* AgGnssBridge - GNSS UDP Bridge
 * for the AgOpenGPS application.
 *
 * Charlie Price 2023 
*/

byte vers_nr = 44;

#define SERIAL_DEBUG

#include <EEPROM.h>
#include <WiFi.h>
#include <WiFiUdp.h>

//Constants
#define EEPROM_SIZE 12
#define LED_BUILTIN 2

struct Storage {
  //WiFi
  char ssid1[24] = "RoverNet";            // WiFi network that this SteerModule will join
  char password1[24] = "74777477";            // WiFi network password
  //char ssid1[24] = "linville5";            // WiFi network that this SteerModule will join
  //char password1[24] = "thr3adtr4il";            // WiFi network password
  char ssid_ap[24] = "Autosteer_Net"; 
  byte WiFi_myip[4] = { 192, 168, 1, 75 };      // gnss module 
  uint16_t timeoutRouter = 1800;          // time (seconds) to wait for ssid1, after that become a standalone Access Point
  byte timeoutWebIO = 255;                // time (min) afterwards webinterface is switched off
  unsigned int PortAutostToAOG = 5577;          // this is port of this module: Autosteer = 5577 IMU = 5566 GPS = 
  unsigned int PortFromAOG = 8888;
  unsigned int PortDestination = 9999;
  byte WiFi_gwip[4] = { 192, 168, 1, 1 };       // Gateway IP only used if Accesspoint created
  byte WiFi_ipDest_ending = 2;                  // ending of IP address to send UDP data to
  byte mask[4] = { 255, 255, 255, 0 };
  byte myDNS[4] = { 8, 8, 8, 8 };               //optional

  byte DataTransVia = 7;                        // transfer data via 0 = USB / 7 = WiFi UDP / 10 = Ethernet UDP

  bool debugmode = true;
  bool debugmodeDataFromAOG = false;

   uint8_t LEDWiFi_PIN = 0;            // light on WiFi connected, flashes on searching Network. If GPIO 0 is used LED must be activ LOW otherwise ESP won't boot
  uint8_t LEDWiFi_ON_Level = LOW;     // HIGH = LED on high, LOW = LED on low

};  Storage Set;

boolean EEPROM_clear = true;  //set to true when changing settings to write them as default values: true -> flash -> boot -> false -> flash again

/*
#define net 4
const char *ssid = "RoverNet";
const char *passphrase = "74777477";

IPAddress local_IP(192,168,1,1);      //IP of the SoftAP
IPAddress gateway(192,168,1,1);       //
IPAddress subnet(255,255,255,0);      //

IPAddress udpRemoteIp(192,168,1,255); //broadcast address for GNSS packets received on Serial2
int       udpRemotePort=9999;         //
int       udpLocalPort=7777;          //
*/
unsigned int now = 100;

char packetBuffer[255]; //buffer to hold outgoing packet
int packetPtr;
char c;
WiFiUDP Udp;

//UART2, Serial2 pins for the GNSS UDP Bridge
#define RXD2 16
#define TXD2 17
#define SERIAL2_BAUD 115200

boolean blink;

TaskHandle_t taskHandle_WiFi_connect;
TaskHandle_t taskHandle_DataFromAOGWiFi; bool WiFiDataTaskRunning = false;

// Variables --------------------------------------------------------------------------------------
// WiFi status LED blink times: searching WIFI: blinking 4x faster; connected: blinking as times set; data available: light on; no data for 2 seconds: blinking
unsigned long LED_WIFI_time = 0, DataFromAOGTime = 0;
#define LED_WIFI_pulse 1000   //light on in ms 
#define LED_WIFI_pause 700    //light off in ms
boolean LED_WIFI_ON = false;

//WIFI+Ethernet
unsigned long WebIOTimeOut = 0, WiFi_network_search_timeout = 0;
byte WiFi_connect_step = 10, WiFi_STA_connect_call_nr = 1, WiFi_netw_nr = 0, my_WiFi_Mode = 0; // WIFI_STA = 1 = Workstation  WIFI_AP = 2  = Accesspoint
IPAddress WiFi_ipDestination; //set in network.ino
bool WiFiUDPRunning = false, newDataFromAOG = false;

WiFiUDP WiFiUDPFromAOG;
WiFiUDP WiFiUDPToAOG;

IPAddress udpRemoteIp(192,168,1,255); //broadcast address for GNSS packets received on Serial2
int udpRemotePort=9999;



void setup() {
  Serial2.begin(SERIAL2_BAUD, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Serial 2 started.");
 
  #if defined(SERIAL_DEBUG)
  Serial.begin(115200);
  //while(!Serial) {
     
  //}
  delay(2000);
  #endif 

  //get EEPROM data
  restoreEEprom();
  delay(100);
 
  WiFi_connect_step = 10;//step 10 = begin of starting a WiFi connection

  //start WiFi
  xTaskCreate(WiFi_handle_connection, "WiFiConnectHandle", 3072, NULL, 1, &taskHandle_WiFi_connect);
  delay(500);

  //handle WiFi LED status
  //xTaskCreate(WiFi_LED_blink, "WiFiLEDBlink", 3072, NULL, 0, &taskHandle_LEDBlink);
  
  vTaskDelay(1000); //waiting for other tasks to start
 
}

void loop() {
    
 int packetSize = Udp.parsePacket();
  if (packetSize) {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());
    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    Serial.println("Contents:");
    Serial.println(packetBuffer);
    // send a reply, to the IP address and port that sent us the packet we received
    //Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    //Udp.write(ReplyBuffer);
    //Udp.endPacket();
  }
  
 if (Serial2.available()) {
    c = Serial2.read();
    packetBuffer[packetPtr++] = c;
    //Serial.print(c);
    //if (c == 10)
    //  packetPtr=0;
 }
 
 if (c == 10) { 
    Udp.beginPacket(udpRemoteIp,udpRemotePort);
    int bytesWritten = Udp.write((byte*)packetBuffer,packetPtr);
    //Serial.print("bytes sent "); Serial.print(bytesWritten); Serial.print(" of "); Serial.println(packetPtr);
    int endCode = Udp.endPacket(); 
    //Serial.println(endCode);
    
    packetBuffer[packetPtr++] = 0;
    if (WiFi_connect_step == 52) {
      Serial.print(packetBuffer);
    }
    
    c=0;
    packetPtr=0;
    memset(packetBuffer, 0, sizeof packetBuffer);     
 }

// if (blink) {
   digitalWrite(LED_BUILTIN, HIGH);
   blink = false;
// } else {
//   digitalWrite(LED_BUILTIN, LOW);
//   blink = true;
// }

 
}
