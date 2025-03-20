/* 
 * Project: Automatic Plant Watering System
 * Author: Marlon McPherson
 * Date: 20 MAR 2025
 */

#include "Particle.h"
#include "credentials.h"

SYSTEM_MODE(SEMI_AUTOMATIC);
// SYSTEM_THREAD(ENABLED);
SerialLogHandler logHandler(LOG_LEVEL_INFO);

void setup() {
  
}

void loop() {
  Serial.printf("?");
}
