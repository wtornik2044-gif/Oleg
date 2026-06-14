#include "sonar.h"

Sonar::Sonar(int trigPin, int echoPin) {
  _trigPin = trigPin;
  _echoPin = echoPin;
  _lastUs = 9999999999;
  _temp = 22;
  _lastPing = 0;
}

void Sonar::setup() {
  pinMode(_trigPin, OUTPUT);
  pinMode(_echoPin, INPUT);
  _ping();
}

void Sonar::update() {
  if (micros() >= _lastPing + SONAR_TRIG_TOUT) {
    _ping();
  }
}

int Sonar::getDistance() {
  if (_lastUs <= 0) return 0;
  return _lastUs * (0.609 * _temp + 330.75) / 2000ul;
}

void Sonar::_ping() {
  digitalWrite(_trigPin, HIGH);
  delayMicroseconds(SONAR_TRIG_PULSE);
  digitalWrite(_trigPin, LOW);

  uint32_t us = pulseIn(_echoPin, HIGH);
  _lastUs = us;
  _lastPing = micros();
}