/*
 * Headlights.cpp
 *
 *  Created on: Apr 26, 2022
 *      Author: Charlie
 */

#include "Headlights.h"

Headlights::Headlights() {
	pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
	pixels.setBrightness(255);
	pixels.begin();
}

void Headlights::flash(uint8_t pixel, uint8_t r, uint8_t g, uint8_t b) {
    pixels.setPixelColor(pixel, pixels.Color(r,g,b));
    pixels.show();
}
