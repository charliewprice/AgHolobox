//getData 7. Maerz 2021
void getDataFromAOGWiFi(void* pvParameters)
{
	byte nextincommingBytesArrayNr;
	unsigned int packetLength;

	WiFiDataTaskRunning = true;

	for (;;) {
		// stay here until everything is running
		if (!WiFiUDPRunning) { 
		  vTaskDelay(3000); 
		} else { 
		  break; 
		}
	}
	
	for (;;) {
		//get UDP packets and stuff them into a circular buffer
		packetLength = WiFiUDPFromAOG.parsePacket();
		if (packetLength > 0) {
			nextincommingBytesArrayNr = (incommingBytesArrayNr + 1) % incommingDataArraySize;
			WiFiUDPFromAOG.read(incommingBytes[nextincommingBytesArrayNr], packetLength);
			incommingBytesArrayNr = nextincommingBytesArrayNr;
			incommingDataLength[incommingBytesArrayNr] = packetLength;
		} else { 
		  vTaskDelay(10); 
		}
	}
}

//-------------------------------------------------------------------------------------------------
//parseData 7. Maerz 2021

void parseDataFromAOG() {
	for (int i = 0; i < incommingDataLength[incommingBytesArrayNrToParse]; i++) {
		//sentence comming? V4.6: 80 81 7F PGN
		if (incomSentenceDigit < 3) {
			if (incommingBytes[incommingBytesArrayNrToParse][i] == FromAOGSentenceHeader[incomSentenceDigit]) {
				//Serial.println("first 3 Bytes fit: sentence");
				SentenceFromAOG[incomSentenceDigit] = incommingBytes[incommingBytesArrayNrToParse][i];
				incomSentenceDigit++;
			} else {
				incomSentenceDigit = 0;
			}
		} else { // matching on 1st three bytes 
			//write incoming Data to sentence array if it fits in
			if (incomSentenceDigit <= SentenceFromAOGMaxLength) {
				SentenceFromAOG[incomSentenceDigit] = incommingBytes[incommingBytesArrayNrToParse][i];
			}

			if (incomSentenceDigit == 3) {  // this is the PGN field
				incomSentenceDigit++;
				//which sentence comming? PGN
				switch (incommingBytes[incommingBytesArrayNrToParse][i]) {
				  case steerDataFromAOGPGN:
					if (Set.aogVersion == 17) { 
                      isSteerDataFoundV17 = true; 
                      SentenceFromAOGLength = 6; //V4.3: no length, no crc but 4 header, starting at 0
					} else { 
					  isSteerDataFound = true; 
					}
					break;
				  case steerSettingsFromAOGPGN:
					if (Set.aogVersion == 17) { isSteerSettingFoundV17 = true; SentenceFromAOGLength = 6; }//V4.3: no length, no crc but 4 header, starting at 0
					else { isSteerSettingFound = true; }
					break;
				  case steerArdConfFromAOGPGN:
					if (Set.aogVersion == 17) { isSteerArdConfFoundV17 = true; SentenceFromAOGLength = 6; }//V4.3: no length, no crc but 4 header, starting at 0
					else { isSteerArdConfFound = true; }
					break;
					
// Hello 			7F 	127 	C8 	200 	3 	Module ID 	0 	0 	CRC
// Scan Request		7F 	123 	CA 	202 	3 	202 	202 	5 	CRC
// Subnet Change 	7F 	127 	C9 	201 	5 	201 	201 	IP_One 	IP_Two 	IP_Three 

                  case helloFromAogPGN:
                    //128 129 127 202 3 202 202 5 71  hello message from AoG looks like this
                    //Serial.println("Hello from AoG");
                    helloFromAog = true;
                    break;
				  case scanFromAogPGN:
                    //128 129 127 202 3 202 202 5 71  hello message from AoG looks like this
                    //Serial.println("Scan Request from AoG");
                    scanFromAog = true;
                    break;
		          default:
		            //Serial.println("no matching PGN");
		            if (Set.aogVersion == 17) { 
					  incomSentenceDigit = 2; 
					} else { 
					  incomSentenceDigit = 0; 
					}
		             break;
		        }//switch
			} else { // >3
				if (incomSentenceDigit == 4) {//length
					if (Set.aogVersion != 17) {
						SentenceFromAOGLength = incommingBytes[incommingBytesArrayNrToParse][i];//+1 for CRC + 1 for Length 
					}
					incomSentenceDigit++;					
				} else {//>4	
					if (incomSentenceDigit == (SentenceFromAOGLength + 5)) { //sentence complete Length: + 4 byte header + 1 length + 1 CRC - 1 (starting at 0) 
						//sentence complete
						if (Set.aogVersion != 17) { //NO checksum in V 4.3
							int CRCDataFromAOG = 0;
							for (byte i = 2; i < sizeof(SentenceFromAOG) - 1; i++)
							{
								CRCDataFromAOG = (CRCDataFromAOG + SentenceFromAOG[i]);
							}
							if (byte(CRCDataFromAOG) == incommingBytes[incommingBytesArrayNrToParse][i]) {
							  //Serial.println("Checksum OK");
							}
							else
							{//checksum error
								if (Set.debugmodeDataFromAOG) { Serial.println("Checksum error"); }
								isSteerDataFound = false;
								isSteerSettingFound = false;
								isSteerArdConfigFound = false;
								incomSentenceDigit = 255;
							}
						}
						if (Set.debugmodeDataFromAOG) {
							for (byte b = 0; b <= (SentenceFromAOGLength + 5); b++) {
								Serial.print(SentenceFromAOG[b]); Serial.print(" ");
							}
							Serial.println();
						}

						if (isSteerDataFound) {
							SectGrFromAOG[0] = SentenceFromAOG[11];   // read Section control from AgOpenGPS 
							SectGrFromAOG[1] = SentenceFromAOG[12];   // read Section control from AgOpenGPS 

							Tram = SentenceFromAOG[10];

							gpsSpeed = ((float)(SentenceFromAOG[6] << 8 | SentenceFromAOG[5])) * 0.1;

							guidanceStatus = SentenceFromAOG[7];

							//Bit 8,9    set point steer angle * 100 is sent
							steerAngleSetPoint = ((float)(SentenceFromAOG[9] << 8 | SentenceFromAOG[8])) * 0.01; //high low bytes
							if (steerAngleSetPoint > 500) { steerAngleSetPoint -= 655.35; }
						//	Serial.print("SteerSetPoint: "); Serial.println(steerAngleSetPoint);

							newDataFromAOG = true;
							isSteerDataFound = false;
							incomSentenceDigit = 255;
							DataFromAOGTime = millis();
							watchdogTimer = 0;
							if (Set.debugmodeDataFromAOG) {
								Serial.print("speed: "); Serial.print(gpsSpeed);
								Serial.print(" GuidStat: "); Serial.print(guidanceStatus);
								Serial.print(" SteerAngSet: "); Serial.print(steerAngleSetPoint);
								Serial.print(" SectGrFromAOG[0]: "); Serial.print(SectGrFromAOG[0]);
								Serial.print(" SectGrFromAOG[1]: "); Serial.println(SectGrFromAOG[1]);
							}
						}
						else {

							if (isSteerDataFoundV17) {
								SectGrFromAOG[0] = SentenceFromAOG[4];   // read Section control from AgOpenGPS 
								SectGrFromAOG[1] = 0;   // not send in V4.3 

								gpsSpeed = ((float)(SentenceFromAOG[5])) * 0.25;

								//distance from the guidance line in mm
								distanceFromLine = (float)(SentenceFromAOG[6] << 8 | SentenceFromAOG[7]);   //high,low bytes     
								if (int(distanceFromLine) == 32020) { guidanceStatus = 0; }
								else { guidanceStatus = 1; }

								//set point steer angle * 10 is sent
								steerAngleSetPoint = ((float)(SentenceFromAOG[8] << 8 | SentenceFromAOG[9])) * 0.01; //high low bytes 

								if (steerAngleSetPoint > 500) { steerAngleSetPoint -= 655.35; }
								//Serial.print("SteerSetPoint V17: "); Serial.println(steerAngleSetPoint);


								newDataFromAOG = true;
								isSteerDataFoundV17 = false;
								incomSentenceDigit = 1;
								DataFromAOGTime = millis();
								watchdogTimer = 0;
								if (Set.debugmodeDataFromAOG) {
									Serial.print("speed: "); Serial.print(gpsSpeed);
									Serial.print(" GuidStat: "); Serial.print(guidanceStatus);
									Serial.print(" SteerAngSet: "); Serial.print(steerAngleSetPoint);
									Serial.print(" distFromLine: "); Serial.print(distanceFromLine);
									Serial.print(" SectGrFromAOG[0]: "); Serial.println(SectGrFromAOG[0]);
								}
							}
							else {

								if (isSteerSettingFound) {
									//PID values
									Set.Kp = ((float)SentenceFromAOG[5]);   // read Kp from AgOpenGPS
									Set.highPWM = SentenceFromAOG[6];
									Set.lowPWM = (float)SentenceFromAOG[7];   // read lowPWM from AgOpenGPS
									Set.minPWM = SentenceFromAOG[8]; //read the minimum amount of PWM for instant on
									Set.steerSensorCounts = float(SentenceFromAOG[9]); //sent as setting displayed in AOG
							//		Set.wasOffset = (SentenceFromAOG[10]);  //read was zero offset Hi
							//		Set.wasOffset |= (SentenceFromAOG[11] << 8);  //read was zero offset Lo
									Set.AckermanFix = SentenceFromAOG[12];

									EEprom_write_all();

									// for PWM High to Low interpolator
									highLowPerDeg = ((float)(Set.highPWM - Set.lowPWM)) / Set.MotorSlowDriveDegrees;

									if (Set.debugmodeDataFromAOG) { Serial.println("got NEW steer settings from AOG"); }
									isSteerSettingFound = false;
									incomSentenceDigit = 255;
								}

								if (isSteerArdConfigFound) {
									if (bitRead(SentenceFromAOG[5], 0)) Set.InvertWAS = 1; else Set.InvertWAS = 0;
									if (bitRead(SentenceFromAOG[5], 1)) Set.Relays_ON = 1; else Set.Relays_ON = 0;
									if (bitRead(SentenceFromAOG[5], 2)) Set.MotorDriveDirection = 1; else Set.MotorDriveDirection = 0;
									//if (bitRead(SentenceFromAOG[5], 3)) Set.SingleInputWAS = 1; else Set.SingleInputWAS = 0;
									//if (bitRead(SentenceFromAOG[5], 4)) Set.CytronDriver = 1; else Set.CytronDriver = 0;
									if (bitRead(SentenceFromAOG[5], 5)) Set.SteerSwitchType = 1;//switch to GND
									if (bitRead(SentenceFromAOG[5], 6)) Set.SteerSwitchType = 2;//button
									else {
										if (!bitRead(SentenceFromAOG[5], 5)) { Set.SteerSwitchType = 255; }//none
									}
									if (bitRead(SentenceFromAOG[5], 7)) Set.ShaftEncoder = 1; else Set.ShaftEncoder = 0;

									Set.pulseCountMax = SentenceFromAOG[6];

									//if (bitRead(SentenceFromAOG[8], 0)) Set.IsDanfoss = 1; else Set.IsDanfoss = 0;
									//if (bitRead(SentenceFromAOG[8], 1)) Set.PressureSensor = 1; else Set.PressureSensor = 0;
									//if (bitRead(SentenceFromAOG[8], 2)) Set.CurrentSensor = 1; else Set.CurrentSensor = 0;

									EEprom_write_all();

									if (Set.debugmodeDataFromAOG) { Serial.println("got NEW Arduino settings from AOG V4.6 or higher"); }

									isSteerArdConfigFound = false;
									incomSentenceDigit = 255;
								}								
							}
						}
					}//sentence complete

					incomSentenceDigit++;

					if (incomSentenceDigit > (SentenceFromAOGLength + 6)) {
						//Serial.println("sentence too long");
						incomSentenceDigit = 0;						
					}
				}//>4
			}//==3
		}//<3
	}//for packetLength

	//this packet is done, prepare for new 
	incommingDataLength[incommingBytesArrayNrToParse] = 0;
	incommingBytesArrayNrToParse = (incommingBytesArrayNrToParse + 1) % incommingDataArraySize;
}

//-------------------------------------------------------------------------------------------------
// 7. Maerz 2021

void AOGDataSend()
{
	WiFiUDPToAOG.beginPacket(WiFi_ipDestination, Set.PortDestination);
	WiFiUDPToAOG.write(steerToAOG, DataToAOGLength);
	WiFiUDPToAOG.endPacket();
}

//-------------------------------------------------------------------------------------------------
//send back checksum and version

void SendTwoThirty(byte check)
{
	if (Set.aogVersion == 17) {
		byte TwoThirtyV17[10] = { FromAOGSentenceHeader[2],230,check,Set.aogVersion,0,0,0,0,0,0 };
		if (WiFiUDPRunning) {
		  WiFiUDPToAOG.beginPacket(WiFi_ipDestination, Set.PortDestination);
		  WiFiUDPToAOG.write(TwoThirtyV17, sizeof(TwoThirtyV17));
		  WiFiUDPToAOG.endPacket();
		}
	} else {
		byte TwoThirtyV20[] = { FromAOGSentenceHeader[0],FromAOGSentenceHeader[1],FromAOGSentenceHeader[2],230,2,check,Set.aogVersion, check + Set.aogVersion };
		//WiFi UDP
		if (WiFiUDPRunning) {
		  WiFiUDPToAOG.beginPacket(WiFi_ipDestination, Set.PortDestination);
		  WiFiUDPToAOG.write(TwoThirtyV20, sizeof(TwoThirtyV20));
		  WiFiUDPToAOG.endPacket();
		}
	}
}
