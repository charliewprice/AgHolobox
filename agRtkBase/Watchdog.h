/*
 * Watchdog.h
 * 2K I2C Serial EEPROMs with EUI-48™ or EUI-64™ Node Identity
 *  Created on: Apr 26, 2022
 *      Author: Charlie
 */

#ifndef WATCHDOG_H_
#define WATCHDOG_H_

//#define _TEST_WATCHDOG
#define _WATCHDOG_DONE 8

class Watchdog {
public:
	Watchdog();
	void poll();
};

#endif /* WATCHDOG_H_ */
