void assignGPIOs_start_extHardware() {
  Serial.println("   Initializing I/O");
  //delay(50);
	//init wire for ADS and MMA or BNO or CMPS
	//if (!Wire.begin(Set.SDA, Set.SCL, 400000)) {
  if (!Wire.begin(21,22)) {  
    delay(5000);
		Serial.println("error INIT wire, ADS, BNO, CMPS, MMA will not work");
    delay(5000);
	} else {
    Serial.println("   I2C is running");
	}
	//delay(20);

	//init GPIO pins, if 255 = unused/not connected
#if useLED_BUILTIN
	pinMode(LED_BUILTIN, OUTPUT);
#endif	
	if (Set.LEDWiFi_PIN < 255) { pinMode(Set.LEDWiFi_PIN, OUTPUT); }
	if ((Set.WorkSW_mode > 0) && (Set.WORKSW_PIN < 255)) { pinMode(Set.WORKSW_PIN, INPUT_PULLUP); }
	if (Set.Relay_PIN[0] < 255) { pinMode(Set.Relay_PIN[0], OUTPUT); }
	if (Set.Relay_PIN[1] < 255) { pinMode(Set.Relay_PIN[1], OUTPUT); }

	//no check if < 255 as needed for autosteer in every case
	pinMode(Set.AutosteerLED_PIN, OUTPUT);
	pinMode(Set.PWM_PIN, OUTPUT);
	pinMode(Set.DIR_PIN, OUTPUT);
	delay(2);
	ledcSetup(0, Set.PWMOutFrequ, 8);  // PWM Output with channel 0, x kHz, 8-bit resolution (0-255)
	ledcSetup(1, Set.PWMOutFrequ, 8);  // PWM Output with channel 1, x kHz, 8-bit resolution (0-255)
	delay(2);
	ledcAttachPin(Set.PWM_PIN, 0);  // attach PWM PIN to Channel 0
	ledcAttachPin(Set.DIR_PIN, 1);  // attach PWM PIN to Channel 1

	//if (Set.WASType == 0)  Set.WebIOSteerPosZero = 2048;                //Starting Point with ESP ADC 2048 
	//if (Set.WASType > 0 && Set.WASType < 3)  Set.WebIOSteerPosZero = 13000;  //with ADS start with 13000  

  if (Set.SteerSwitchType == 0) { 
	  pinMode(Set.STEERSW_PIN, INPUT_PULLDOWN); 
    Serial.print(Set.STEERSW_PIN); Serial.println(" is INPUT_PULLDOWN type.");
	}
	if (Set.SteerSwitchType > 0) { 
	  pinMode(Set.STEERSW_PIN, INPUT_PULLUP); 
  }

	//Setup Interrupt -Steering Wheel encoder
	if (Set.encA_PIN < 255) { pinMode(Set.encA_PIN, INPUT_PULLUP); }
	if (Set.encB_PIN < 255) { pinMode(Set.encB_PIN, INPUT_PULLUP); }

	delay(50);
	//IMU
	byte error = 0;	
	switch (Set.IMUType) {		
	case 0:
		//roll no hardware = 8888
		steerToAOG[9] = 0xB8;
		steerToAOG[10] = 0x22;
		roll = 0;
		//heading16 no hardware = 9999     
		steerToAOG[7] = 0x0F;
		steerToAOG[8] = 0x27;
		heading = 0;
		break;   
	}//switch IMU	
	//ADS1115
	adc.setSampleRate(ADS1115_REG_CONFIG_DR_128SPS); //128 samples per second
	adc.setGain(ADS1115_REG_CONFIG_PGA_6_144V);
}
