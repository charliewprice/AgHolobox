/*
 * I2cSerialEeprom.h
 *
 *  Created on: Apr 28, 2022
 *      Author: Charlie
 */

#ifndef I2cSerialEeprom_H_
#define I2cSerialEeprom_H_

#include <Wire.h>

#define _EEPROM_I2C_ADDR 				0x50		//the I2C bus address for the EEPROM
#define _DEVEUI_ADDR 							0xF8		//the location in the EEPROM for the DEVEUI
#define _EEPROM_CONFIG_ADDRESS 30			//the location in the EEPROM for the device configuration

class I2cSerialEeprom {
public:
	I2cSerialEeprom();
	virtual ~I2cSerialEeprom();
	void i2c_eeprom_read_buffer( int deviceaddress, unsigned int eeaddress, byte *buffer, int length );
	void saveBytes(byte b[], uint16_t size);
	void readBytes(byte b[], uint16_t size) ;
private:
	void EEPROM_write(uint16_t addr,uint8_t data) ;
	byte EEPROM_read(uint16_t addr);

	boolean updateEeprom;
};

#endif /* I2cSerialEeprom_H_ */
