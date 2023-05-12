/*
 * Headlights.h
 *
 *  Created on: Apr 26, 2022
 *      Author: Charlie
 */

#ifndef HEADLIGHTS_H_
#define HEADLIGHTS_H_

#include <Adafruit_NeoPixel.h>

#define PIN            			11
#define NUMPIXELS      	2

class Headlights {
public:
	Headlights();
	void flash(uint8_t pixel, uint8_t r, uint8_t g, uint8_t b);
private:
    Adafruit_NeoPixel pixels;
};

#endif /* HEADLIGHTS_H_ */
