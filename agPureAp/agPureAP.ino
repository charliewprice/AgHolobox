/* AgPureAp - WiFi Access Point
 * for the AgOpenGPS application.
 *
 * Charlie Price 2023 
*/
#define SERIAL_DEBUG

#include <EEPROM.h>
#include <WiFi.h>
#include <WiFiUdp.h>

//Constants
#define EEPROM_SIZE 12

#define net 4
const char *ssid = "RoverNet";
const char *passphrase = "74777477";

IPAddress local_IP(192,168,1,1);      //IP of the SoftAP
IPAddress gateway(192,168,1,1);       //
IPAddress subnet(255,255,255,0);      //

boolean blink;
//int LED_BUILTIN = 2;
TaskHandle_t taskHandle_LEDBlink;

void osetup() {
 //Serial2.begin(SERIAL2_BAUD, SERIAL_8N1, RXD2, TXD2);
 //Serial.println("Serial 2 started.");
 
  #if defined(SERIAL_DEBUG)
 Serial.begin(230400);
 //while(!Serial) {
     
 //}
 delay(2000);
 #endif 
 
 Serial.println();
 
 WiFi.mode(WIFI_AP);
 
 Serial.print("Setting soft-AP configuration ... ");
 if (WiFi.softAPConfig(local_IP, gateway, subnet)) {
   Serial.println("Ready");
 } else {
   Serial.println("Failed!");  
 }
 
 delay(2000);
 
 Serial.print("Setting soft-AP ... ");
 if (WiFi.softAP(ssid,passphrase,1,0,8)) {
   Serial.println("Ready");
 } else {
   Serial.println("Failed!");  
 }
 delay(2000);
 
 Serial.print("AgPureAp IP address = ");
 Serial.println(WiFi.softAPIP());
 
 pinMode(LED_BUILTIN, OUTPUT);
 //xTaskCreate(WiFi_LED_blink, "WiFiLEDBlink", 3072, NULL, 0, &taskHandle_LEDBlink);
}

void setup() {
 Serial.begin(230400);
 WiFi.softAPConfig(local_IP, gateway, subnet);
 WiFi.softAP(ssid, passphrase,3,0,5);
 delay(2000);
 Serial.println("Ready");
 Serial.print("AgPureAp IP address = ");
 Serial.println(WiFi.softAPIP()); 
}
void loop() { 
  vTaskDelay(3); 
  //flash(WiFi.softAPgetStationNum());  
}
