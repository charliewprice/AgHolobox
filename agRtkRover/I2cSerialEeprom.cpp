/*
 * I2cSerialEeprom.cpp
 *
 *  Created on: Apr 28, 2022
 *      Author: Charlie
 */

#include "I2cSerialEeprom.h"

I2cSerialEeprom::I2cSerialEeprom() {
	// TODO Auto-generated constructor stub
}

I2cSerialEeprom::~I2cSerialEeprom() {
	// TODO Auto-generated constructor stub
}

void I2cSerialEeprom::i2c_eeprom_read_buffer( int deviceaddress, unsigned int eeaddress, byte *buffer, int length ) {
    Wire1.beginTransmission(deviceaddress);
    Wire1.write(eeaddress);
    Wire1.endTransmission();
    Wire1.requestFrom(deviceaddress,length);
    int c = 0;
    for ( c = 0; c < length; c++ )
       if (Wire1.available())
         buffer[c] = Wire1.read();
       else
    	 Serial.print("na");
    Wire1.endTransmission();
}

void I2cSerialEeprom::EEPROM_write(uint16_t addr,uint8_t data) {
  //Serial.print("EEPROM_write");
  Wire1.beginTransmission(_EEPROM_I2C_ADDR);
  Wire1.write(addr);
  Wire1.write(data);
  Wire1.endTransmission();    // stop transmitting
}

byte I2cSerialEeprom::EEPROM_read(uint16_t addr) {
  byte data;
  Wire1.beginTransmission(_EEPROM_I2C_ADDR);
  Wire1.write(addr);
  Wire1.endTransmission();
  Wire1.requestFrom(_EEPROM_I2C_ADDR, 1);

  if (Wire1.available()) {
    data = Wire1.read();
  }
  Wire1.endTransmission();

  return data;
}

void I2cSerialEeprom::saveBytes(byte b[], uint16_t size) {
  for(uint16_t  i=0; i<size; i++) {
    EEPROM_write(_EEPROM_CONFIG_ADDRESS + i, b[i]);
    delay(10);
  }
  updateEeprom = false;
}

void I2cSerialEeprom::readBytes(byte b[], uint16_t size) {
  for(uint16_t i=0; i<size; i++) {
    b[i] = EEPROM_read(_EEPROM_CONFIG_ADDRESS + i);
  }
}

