/*
 * Watchdog.cpp
 *
 *  Created on: Apr 26, 2022
 *      Author: Charlie
 */

#include "Watchdog.h"

/*
 * HeaterController.cpp
 *
 *  Created on:  ~April 2022
 *      Author: Charlie
 */


#include "Arduino.h"

Watchdog::Watchdog() {
  pinMode(_WATCHDOG_DONE, OUTPUT);
}

void Watchdog::poll() {
#if not defined(_TEST_WATCHDOG)
  digitalWrite(_WATCHDOG_DONE, HIGH);				// rising edge pulse on DONE to keep the watchdog happy!
  delay(2);																		// pet the dog before going to sleep.
  digitalWrite(_WATCHDOG_DONE, LOW);
#endif
}
