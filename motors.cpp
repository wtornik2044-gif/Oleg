#include "motors.h"

Motors* Motors::_context = nullptr;

Motors::Motors(MotorPins leftPins, MotorPins rightPins, PIDStruct pid, RobotStruct robot) {
  _context = this;

  _leftMotor = { leftPins, 0, 0, 0, 0, 0 };
  _rightMotor = { rightPins, 0, 0, 0, 0, 0 };

  _pid = pid;
  _robot = robot;

  _pos = { 0, 0, 0 };
  _target = { 0, 0, 0 };

  _mode = MOTION_IDLE;

  _targetTicks = 0;

  _integral = 0;
  _lastError = 0;
}

void setupMotorPins(MotorPins pins) {
  pinMode(pins.PWM, OUTPUT);
  pinMode(pins.IN1, OUTPUT);
  pinMode(pins.IN2, OUTPUT);

  pinMode(pins.ENC_A, INPUT_PULLUP);
  pinMode(pins.ENC_B, INPUT_PULLUP);
}

void Motors::setup() {
  setupMotorPins(_leftMotor.pins);
  setupMotorPins(_rightMotor.pins);

  _leftMotor.lastState = _getState(_leftMotor);
  _rightMotor.lastState = _getState(_rightMotor);

  attachInterrupt(digitalPinToInterrupt(_leftMotor.pins.ENC_A), Motors::_leftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(_leftMotor.pins.ENC_B), Motors::_leftISR, CHANGE);

  attachInterrupt(digitalPinToInterrupt(_rightMotor.pins.ENC_A), Motors::_rightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(_rightMotor.pins.ENC_B), Motors::_rightISR, CHANGE);

  stop();
}

void Motors::update() {
  _updateOdometry();
  _updateMotion();
}

void Motors::moveLinear(float distanceMM) {
  _target.x = _pos.x + distanceMM * cos(radians(_pos.rot));
  _target.y = _pos.y + distanceMM * sin(radians(_pos.rot));
  _target.rot = _pos.rot;

  _mode = MOTION_LINEAR;
  _startLinear(distanceMM);
}

void Motors::turnBy(float angleDeg) {
  _target.x = _pos.x;
  _target.y = _pos.y;
  _target.rot = _normalizeAngle(_pos.rot + angleDeg);

  _mode = MOTION_TURN;
  _startTurn(angleDeg);
}

void Motors::turnTo(float targetRotDeg) {
  _target.x = _pos.x;
  _target.y = _pos.y;
  _target.rot = _normalizeAngle(targetRotDeg);

  float diff = _angleDiff(_target.rot, _pos.rot);

  _mode = MOTION_TURN;
  _startTurn(diff);
}

void Motors::moveTo(float targetX, float targetY) {
  _target.x = targetX;
  _target.y = targetY;

  float dx = _target.x - _pos.x;
  float dy = _target.y - _pos.y;

  float angleToTarget = degrees(atan2(dy, dx));
  _target.rot = _normalizeAngle(angleToTarget);

  float diff = _angleDiff(angleToTarget, _pos.rot);

  _mode = MOTION_GOTO_TURN;
  _startTurn(diff);
}

void Motors::stop() {
  _setMotors(0, 0);
  _mode = MOTION_IDLE;
  _resetPID();
}

bool Motors::isBusy() {
  return _mode != MOTION_IDLE;
}

RobotState Motors::getState() {
  if (_mode == MOTION_IDLE) {
    return ROBOT_IDLE;
  }

  return ROBOT_MOVING;
}

RobotPosition Motors::getPosition() {
  return _pos;
}

RobotPosition Motors::getTarget() {
  return _target;
}

void Motors::resetPosition(float x, float y, float rot) {
  _pos.x = x;
  _pos.y = y;
  _pos.rot = _normalizeAngle(rot);

  _target.x = _pos.x;
  _target.y = _pos.y;
  _target.rot = _pos.rot;

  long l, r;
  _getTicks(l, r);

  _leftMotor.lastOdom = l;
  _rightMotor.lastOdom = r;
}

void Motors::_startLinear(float distanceMM) {
  long l, r;
  _getTicks(l, r);

  _leftMotor.startTicks = l;
  _rightMotor.startTicks = r;

  _targetTicks = _distanceToTicks(distanceMM);

  int dir = distanceMM >= 0 ? 1 : -1;
  _leftMotor.dir = dir;
  _rightMotor.dir = dir;

  _resetPID();
}

void Motors::_startTurn(float angleDeg) {
  angleDeg = constrain(angleDeg, -360, 360);

  long l, r;
  _getTicks(l, r);

  _leftMotor.startTicks = l;
  _rightMotor.startTicks = r;

  _targetTicks = _angleToTicks(angleDeg);

  _leftMotor.dir = angleDeg >= 0 ? 1 : -1;
  _rightMotor.dir = angleDeg >= 0 ? -1 : 1;

  _resetPID();
}

void Motors::_updateMotion() {
  if (_mode == MOTION_IDLE) return;

  long l, r;
  _getTicks(l, r);

  // Serial.print(l);
  // Serial.print(" ");
  // Serial.print(r);
  // Serial.print("\n");

  long leftProgress = abs(l - _leftMotor.startTicks);
  long rightProgress = abs(r - _rightMotor.startTicks);

  long avgProgress = (leftProgress + rightProgress) / 2;

  if (avgProgress >= _targetTicks) {
    _setMotors(0, 0);
    _resetPID();

    if (_mode == MOTION_GOTO_TURN) {
      float dx = _target.x - _pos.x;
      float dy = _target.y - _pos.y;
      float dist = sqrt(dx * dx + dy * dy);

      _mode = MOTION_GOTO_LINEAR;
      _startLinear(dist);
      return;
    }

    _mode = MOTION_IDLE;
    return;
  }

  int currentBasePWM = _robot.BASE_PWM;

  long slowDownTicks = _distanceToTicks(120); // зона замедления 120 мм
  long remainingTicks = _targetTicks - avgProgress;
  if (remainingTicks < slowDownTicks) {
    currentBasePWM = map(remainingTicks, 0, slowDownTicks, _robot.MIN_PWM, _robot.BASE_PWM);
    currentBasePWM = constrain(currentBasePWM, _robot.MIN_PWM, _robot.BASE_PWM);
  }

  int correction = _calculatePID(leftProgress, rightProgress);
  int maxCorrection = currentBasePWM - _robot.MIN_PWM;
  correction = constrain(correction, -maxCorrection, maxCorrection);
  // Serial.print(correction);
  // Serial.print("\n");
  int leftPWM = currentBasePWM - correction;
  int rightPWM = currentBasePWM + correction;

  leftPWM = constrain(leftPWM, _robot.MIN_PWM, _robot.MAX_PWM);
  rightPWM = constrain(rightPWM, _robot.MIN_PWM, _robot.MAX_PWM);

  _setMotors(_leftMotor.dir * leftPWM, _rightMotor.dir * rightPWM);
}

void Motors::_updateOdometry() {
  long l, r;
  _getTicks(l, r);

  long dLticks = l - _leftMotor.lastOdom;
  long dRticks = r - _rightMotor.lastOdom;

  _leftMotor.lastOdom = l;
  _rightMotor.lastOdom = r;

  float mmPerTick = _wheelCircumference() / _robot.COUNTS_PER_REV;

  float dL = dLticks * mmPerTick;
  float dR = dRticks * mmPerTick;

  float dCenter = (dL + dR) / 2.0;
  float dTheta = (dR - dL) / _robot.WHEEL_BASE_MM;

  float currentRotRad = radians(_pos.rot);
  float middleRotRad = currentRotRad + dTheta / 2.0;

  _pos.x += dCenter * cos(middleRotRad);
  _pos.y += dCenter * sin(middleRotRad);

  _pos.rot += degrees(dTheta);
  _pos.rot = _normalizeAngle(_pos.rot);
}

byte Motors::_getState(Motor &motor) {
  return (digitalRead(motor.pins.ENC_A) << 1) | digitalRead(motor.pins.ENC_B);
}

int8_t Motors::_readEncoderStep(byte lastState, byte currentState) {
  if ((lastState == 0b00 && currentState == 0b01) ||
      (lastState == 0b01 && currentState == 0b11) ||
      (lastState == 0b11 && currentState == 0b10) ||
      (lastState == 0b10 && currentState == 0b00)) {
    return 1;
  }

  if ((lastState == 0b00 && currentState == 0b10) ||
      (lastState == 0b10 && currentState == 0b11) ||
      (lastState == 0b11 && currentState == 0b01) ||
      (lastState == 0b01 && currentState == 0b00)) {
    return -1;
  }

  return 0;
}

void Motors::_encoderISR(Motor &motor) {
  byte currentState = _getState(motor);
  int8_t step = _readEncoderStep(motor.lastState, currentState);

  motor.ticks += step;
  motor.lastState = currentState;
}

void Motors::_leftISR() {
  if (_context == nullptr) return;
  _context->_encoderISR(_context->_leftMotor);
}

void Motors::_rightISR() {
  if (_context == nullptr) return;
  _context->_encoderISR(_context->_rightMotor);
}

void Motors::_setMotor(Motor &motor, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(motor.pins.IN1, HIGH);
    digitalWrite(motor.pins.IN2, LOW);
    analogWrite(motor.pins.PWM, speed);
  } else if (speed < 0) {
    digitalWrite(motor.pins.IN1, LOW);
    digitalWrite(motor.pins.IN2, HIGH);
    analogWrite(motor.pins.PWM, -speed);
  } else {
    digitalWrite(motor.pins.IN1, HIGH);
    digitalWrite(motor.pins.IN2, HIGH);
    analogWrite(motor.pins.PWM, 255);
  }
}

void Motors::_setMotors(int leftSpeed, int rightSpeed) {
  _setMotor(_leftMotor, leftSpeed);
  _setMotor(_rightMotor, rightSpeed);
}

void Motors::_getTicks(long &left, long &right) {
  noInterrupts();
  left = _leftMotor.ticks;
  right = _rightMotor.ticks;
  interrupts();
}

float Motors::_wheelCircumference() {
  return PI * _robot.WHEEL_D_MM;
}

long Motors::_distanceToTicks(float distanceMM) {
  return abs(distanceMM) / _wheelCircumference() * _robot.COUNTS_PER_REV;
}

long Motors::_angleToTicks(float angleDeg) {
  float turnCircle = PI * _robot.WHEEL_BASE_MM;
  float wheelDistance = turnCircle * abs(angleDeg) / 360.0;

  return _distanceToTicks(wheelDistance);
}

int Motors::_calculatePID(long leftProgress, long rightProgress) {
  long error = leftProgress - rightProgress;

  _integral += error;
  //_integral = constrain(_integral, -500, 500);

  long derivative = error - _lastError;
  _lastError = error;

  return _pid.Kp * error + _pid.Ki * _integral + _pid.Kd * derivative;
}

void Motors::_resetPID() {
  _integral = 0;
  _lastError = 0;
}

float Motors::_normalizeAngle(float angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

float Motors::_angleDiff(float target, float current) {
  return _normalizeAngle(target - current);
}