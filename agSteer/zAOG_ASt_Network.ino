// WIFI handling 7. Maerz 2021 for ESP32  -------------------------------------------

void WiFi_handle_connection(void* pvParameters) {
    //start Ethernet first, if needed for data transfer
    for (;;) {
        if (WiFi_connect_step == 0) {
            if (Set.debugmode) { Serial.println("closing WiFi connection task"); }
            delay(1);
            vTaskDelete(NULL);
            delay(1);
        } else {
            vTaskDelay(500);//do every half second

            now = millis();

            IPAddress gwIP, myIP;

            if (Set.debugmode) { Serial.print("WiFi_connect_step: "); Serial.println(WiFi_connect_step); }
            
                    
            switch (WiFi_connect_step) {
                //WiFi network scan
            case 10:
                WiFi_netw_nr = 0;
                WebIORunning = false;
                WiFiUDPRunning = false;
                if (WiFi_network_search_timeout == 0) {   //first run                 
                    WiFi_network_search_timeout = now + (Set.timeoutRouter * 1000);
                }
                //skip it and just connect WiFi_scan_networks();
                WiFi_connect_step++;
                //timeout?
                if (now > WiFi_network_search_timeout) { 
                  WiFi_connect_step = 50; 
                } else {
                    if (WiFi_netw_nr > 0) {
                        //found network
                        WiFi_connect_step++;
                        WiFi_network_search_timeout = 0;//reset timer
                    }
                }
                break;
                //start WiFi connection
            case 11:
                WiFi.mode(WIFI_STA);   //  Workstation 
                WiFi_connect_step++;
                break;
            case 12:
                if (WiFi_network_search_timeout == 0) {   //first run  
                    WiFi_network_search_timeout = now + (Set.timeoutRouter * 500);//half time
                }
                WiFi_STA_connect_network();
                WiFi_connect_step++;
                break;
            case 13:
                if (WiFi.status() != WL_CONNECTED) {
                    Serial.print(".");
                    if (now > WiFi_network_search_timeout) {
                        //timeout
                        WiFi_STA_connect_call_nr++;
                        WiFi_connect_step = 17;//close WiFi and try again
                        WiFi_network_search_timeout += (Set.timeoutRouter * 500);//add rest of time
                    }
                } else {
                    //connected
                    WiFi_connect_step++;
                    WiFi_network_search_timeout = 0;//reset timer
                }
                break;
                //change IP / DHCP
            case 14:
                //connected
                Serial.println();
                Serial.println("WiFi Client successfully connected");
                Serial.print("Connected IP - Address : ");
                myIP = WiFi.localIP();
                Serial.println(myIP);
                //after connecting get IP from router -> change it to x.x.x.IP Ending (from settings)
                myIP[3] = Set.WiFi_myip[3]; //set ESP32 IP to x.x.x.myIP_ending
                Serial.print("changing IP to: ");
                Serial.println(myIP);
                gwIP = WiFi.gatewayIP();
                if (!WiFi.config(myIP, gwIP, Set.mask, gwIP)) { Serial.println("STA Failed to configure"); }
                WiFi_connect_step++;
                break;
            case 15:
                myIP = WiFi.localIP();
                Serial.print("Connected IP - Address : "); Serial.println(myIP);
                WiFi_ipDestination = myIP;
                WiFi_ipDestination[3] = Set.WiFi_ipDest_ending;
                Serial.print("sending to IP - Address : "); Serial.println(WiFi_ipDestination);
                gwIP = WiFi.gatewayIP();
                Serial.print("Gateway IP - Address : "); Serial.println(gwIP);
                my_WiFi_Mode = 1;// WIFI_STA;
                WiFi_connect_step = 20;
                break;
                //no connection at first try, try again
            case 17:
                if (WiFi_STA_connect_call_nr > 2) { //create access point
                    WiFi_connect_step = 50;
                    WiFi_netw_nr = 0;
                }
                else {
                    WiFi.disconnect();
                    vTaskDelay(2);
                    WiFi_connect_step++;
                    Serial.print("-");
                }
                break;
            case 18:
                WiFi.mode(WIFI_OFF); vTaskDelay(2);
                WiFi_connect_step = 11; //set STA
                break;

                //UDP
            case 20://init WiFi UDP sending to AOG
                if (WiFiUDPToAOG.begin(Set.PortAutostToAOG))
                {
                    Serial.print("UDP writing to IP: ");
                    Serial.println(WiFi_ipDestination);
                    Serial.print("UDP writing to port: ");
                    Serial.println(Set.PortDestination);
                    Serial.print("UDP writing from port: ");
                    Serial.println(Set.PortAutostToAOG);
                }
                else { Serial.println("Error starting UDP"); }
                WiFi_connect_step++;
                break;
            case 21:
                //init WiFi UPD listening to AOG 
                if (WiFiUDPFromAOG.begin(Set.PortFromAOG))
                {
                    Serial.print("WiFi UDP Listening for AOG data to port: ");
                    Serial.println(Set.PortFromAOG);
                    Serial.println();
                    WiFiUDPRunning = true;
                }
                else { Serial.println("Error starting UDP"); }
                delay(2);

                WiFi_connect_step = 100;
                break;
            case 51:
                if (my_WiFi_Mode == 2) { WiFi_connect_step++; }
                break;
            case 52://init WiFi UDP sending to AOG
                WiFiUDPToAOG.begin(Set.PortAutostToAOG);
                Serial.print("UDP writing to IP: ");
                Serial.println(WiFi_ipDestination);
                Serial.print("UDP writing to port: ");
                Serial.println(Set.PortDestination);
                Serial.print("UDP writing from port: ");
                Serial.println(Set.PortAutostToAOG);
                WiFi_connect_step++;
                break;
            case 53:
                //init WiFi UPD listening to AOG 
                WiFiUDPFromAOG.begin(Set.PortFromAOG);
                Serial.print("NTRIP WiFi UDP Listening to port: ");
                Serial.println(Set.PortFromAOG);
                Serial.println();
                delay(2);
                WiFi_connect_step = 100;
                break;

                //Webserver start
            case 100:
                //start Server for Webinterface
                WiFiStartServer();
                WiFi_connect_step++;
                break;

            case 101:
                WebIOTimeOut = millis() + (long(Set.timeoutWebIO) * 60000);
                xTaskCreate(doWebinterface, "WebIOHandle", 5000, NULL, 1, &taskHandle_WebIO);
                delay(300);
                WiFi_connect_step = 0;
                LED_WIFI_ON = true;
                Serial.println(); Serial.println();
                if (WiFi_netw_nr == 0) { myIP = WiFi.softAPIP(); }
                else { myIP = WiFi.localIP(); }

                Serial.print("started settings Webinterface at: ");
                for (byte i = 0; i < 3; i++) {
                    Serial.print(myIP[i]); Serial.print(".");
                }
                Serial.println(myIP[3]);
                Serial.println("type IP in Internet browser to get to webinterface");
                Serial.print("you need to be in WiFi network ");
                Serial.print(Set.ssid1);
                Serial.println(" to get access"); Serial.println(); Serial.println();
#if useLED_BUILTIN
                digitalWrite(LED_BUILTIN, HIGH);
#endif
                digitalWrite(Set.LEDWiFi_PIN, Set.LEDWiFi_ON_Level);
                break;

            default:
                WiFi_connect_step++;
                Serial.print("default called at WiFi_connection_step "); Serial.println(WiFi_connect_step);
                break;
            }
        }
    }
}


//---------------------------------------------------------------------
// scanning for known WiFi networks

void WiFi_scan_networks()
{
    Serial.println("\n===============================\n\rSearching for WiFi network");
    // WiFi.scanNetworks will return the number of networks found
    int WiFi_num_netw_inReach = WiFi.scanNetworks();
    Serial.print("   scan done: ");
    if (WiFi_num_netw_inReach == 0) {
        Serial.println("   no networks found");
    }
    else
    {
        Serial.print(WiFi_num_netw_inReach);
        Serial.println("  network(s) found:");
        for (int i = 0; i < WiFi_num_netw_inReach; ++i) {
            Serial.println("   " + WiFi.SSID(i));
        }
        delay(800);//.SSID gives no value if no delay
        
        for (int i = 0; i < WiFi_num_netw_inReach; ++i) {
            if (WiFi.SSID(i) == Set.ssid1) {
                // network found in list
                Serial.println("Connecting to: " + WiFi.SSID(i));
                WiFi_netw_nr = 1;
                break;
            }
        }
    }
}  //end WiFi_scan_networks()

//-------------------------------------------------------------------------------------------------
//connects to WiFi network

void WiFi_STA_connect_network() {//run WiFi_scan_networks first
   // Serial.print("netwNr: "); Serial.print(WiFi_netw_nr);
    WiFi.begin(Set.ssid1, Set.password1);    
    //set IP to DHCP on first run. call immediately after begin
    if (WiFi_STA_connect_call_nr == 0) { WiFi.config(0U, 0U, 0U); Serial.println("enable DHCP for WiFi"); WiFi_STA_connect_call_nr++; }
    delay(2);
}
