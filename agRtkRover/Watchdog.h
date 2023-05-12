/*
 * Watchdog.h
 *
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
