#pragma once
#include <Arduino.h>
#include <inttypes.h>

#define SONAR_MAX_DIST 400ul    // макс. измеряемое расстояние [см]
#define SONAR_TRIG_PULSE 10     // импульс триггера [мкс]
#define SONAR_TRIG_TOUT 1000ul  // таймаут триггера [мкс]

#define GP_MAX_PULSE (GP_MAX_DIST * 58)

class Sonar {
public:
  Sonar(int trigPin, int echoPin);

  void setup();
  void update();
  int getDistance();

private:
  int _trigPin;
  int _echoPin;
  uint32_t _lastUs;
  int _temp;
  long _lastPing;

  void _ping();
};