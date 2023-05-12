// NeoPixel Ring simple sketch (c) 2013 Shae Erisson
// released under the GPLv3 license to match the rest of the AdaFruit NeoPixel library

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

boolean steerEnabled;
boolean steerCancelled;

#define STEER_ENABLE_IN_PIN     2
#define MOTOR_RELAY_PIN         3
#define SECOND_RELAY_PIN  4
#define STEER_ENABLE_REMOTE_PIN 5
#define PIXEL_PIN               6

#define RELAY_ON  LOW
#define RELAY_OFF HIGH

/*
 * there are 2 series wired current sensors
 */

#define CURRENT_SAMPLE_MILLIS 100
long lastCurrentSampleMillis;
#define CURRENT_SENSOR_1       A0
#define CURRENT_SENSOR_2       A1 
#define CURRENT_SENSOR_MAX_MILLIAMPS 20000
#define MOTOR_MILLIS_CUTOUT 20000
#define CURRENT_RATIO_MAX  1.2                //magnitude of currents should match
#define CURRENT_RATIO_MIN -0.2                //direction of currents should match
#define CURRENT_AVG_WINDOW_SIZE 20
long motorCurrent[CURRENT_AVG_WINDOW_SIZE];
uint8_t currIdx;

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);
  
  pinMode(STEER_ENABLE_IN_PIN, INPUT_PULLUP);
  pinMode(MOTOR_RELAY_PIN, OUTPUT);
  digitalWrite(MOTOR_RELAY_PIN, RELAY_OFF);
  pinMode(SECOND_RELAY_PIN, OUTPUT);
  digitalWrite(SECOND_RELAY_PIN, RELAY_OFF);
  pinMode(STEER_ENABLE_REMOTE_PIN, OUTPUT);
  digitalWrite(STEER_ENABLE_REMOTE_PIN, RELAY_OFF);

  steerEnabled = false;

  pixels.begin(); // This initializes the NeoPixel library.
}

boolean currentOk() {

  boolean excessiveCurrent = false;
  boolean mismatchCurrents = false;
  uint16_t current_1_raw = analogRead(CURRENT_SENSOR_1);
  uint16_t current_2_raw = analogRead(CURRENT_SENSOR_2);
  long current_1_milliAmps = map(current_1_raw, 0, 1023, -CURRENT_SENSOR_MAX_MILLIAMPS, CURRENT_SENSOR_MAX_MILLIAMPS);
  long current_2_milliAmps = map(current_2_raw, 0, 1023, -CURRENT_SENSOR_MAX_MILLIAMPS, CURRENT_SENSOR_MAX_MILLIAMPS);

  //averaging over a window of motor current samples
  motorCurrent[currIdx++] = (current_1_milliAmps + current_1_milliAmps)/2;
  if (currIdx==CURRENT_AVG_WINDOW_SIZE)
    currIdx = 0;

  long averageMilliAmps = 0;
  for(uint8_t n=0; n<CURRENT_AVG_WINDOW_SIZE; n++)
    averageMilliAmps += motorCurrent[n];
  averageMilliAmps = averageMilliAmps/CURRENT_AVG_WINDOW_SIZE;  
  
  Serial.print(current_1_raw); Serial.print(" "); Serial.print(current_2_raw);
  Serial.print(" --> ");
  Serial.print(current_1_milliAmps); Serial.print(" "); Serial.print(current_2_milliAmps);
  Serial.println(); 
  
  /* Several factors will cause us to return the false condition
   *    averageMilliAmps are above the cutout limit
   *    current sensor values are not the same within a degree of margin
   */
  if (averageMilliAmps > MOTOR_MILLIS_CUTOUT) {
    excessiveCurrent = true;
    Serial.print("current exceeded at "); Serial.print(averageMilliAmps); Serial.println("mAmps");
  }

  double currentRatio = ((double)current_1_raw)/((double)current_2_raw);  
  if ((currentRatio > CURRENT_RATIO_MAX) || (currentRatio < CURRENT_RATIO_MIN)) {
    Serial.print("currentRatio="); Serial.println(currentRatio);  
    mismatchCurrents = true;
  }

  if (excessiveCurrent || mismatchCurrents)
    return false;
  else
    return true;
}

void loop() {

  if ((millis() - lastCurrentSampleMillis) >= CURRENT_SAMPLE_MILLIS) {
    lastCurrentSampleMillis = millis(); 
    if(currentOk()) {
      // nothing to do here!
    } else {
      steerCancelled = true;
      steerEnabled = false;
      digitalWrite(MOTOR_RELAY_PIN, RELAY_OFF);
      //digitalWrite(SECOND_RELAY_PIN, RELAY_OFF);
      digitalWrite(STEER_ENABLE_REMOTE_PIN, RELAY_OFF);
    }    
  }

  if(digitalRead(STEER_ENABLE_IN_PIN) == 0){
    long actCount = 0;
    while(digitalRead(STEER_ENABLE_IN_PIN) == 0) {
      actCount++;
      delay(10);
      if (actCount>10)
        break;
    }
    if (actCount>10) {
      steerCancelled = false;
      steerEnabled = !steerEnabled;
      delay(500);
    }
    if (steerEnabled) {
      digitalWrite(MOTOR_RELAY_PIN, RELAY_ON);
      delay(200);
      //digitalWrite(SECOND_RELAY_PIN, RELAY_ON);
      digitalWrite(STEER_ENABLE_REMOTE_PIN, RELAY_ON); 
    } else {
      digitalWrite(MOTOR_RELAY_PIN, RELAY_OFF);
      //digitalWrite(SECOND_RELAY_PIN, RELAY_OFF);
      digitalWrite(STEER_ENABLE_REMOTE_PIN, RELAY_OFF);
    }
    while(digitalRead(STEER_ENABLE_IN_PIN) == 0) {}
  }    

  if (steerCancelled) 
     pixels.setPixelColor(0, pixels.Color(150,150,0)); // Moderately bright yellow color.
  else if(steerEnabled)
     pixels.setPixelColor(0, pixels.Color(0,150,0));   // Moderately bright green color. 
  else
     pixels.setPixelColor(0, pixels.Color(150,0,0));   // Moderately bright red color.

  pixels.show(); // This sends the updated pixel color to the hardware.
  //delay(delayval); // Delay for a period of time (in milliseconds).
}
