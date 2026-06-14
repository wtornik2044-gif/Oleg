#pragma once
#include <Arduino.h>

struct MotorPins {
  uint8_t PWM;
  uint8_t IN1;
  uint8_t IN2;
  uint8_t ENC_A;
  uint8_t ENC_B;
};

struct PIDStruct {
  float Kp;
  float Ki;
  float Kd;
};

struct RobotStruct {
  float WHEEL_D_MM;
  float WHEEL_BASE_MM;
  float COUNTS_PER_REV;

  int BASE_PWM;
  int MIN_PWM;
  int MAX_PWM;
};

struct RobotPosition {
  float x;    // мм
  float y;    // мм
  float rot;  // градусы
};

enum RobotState {
  ROBOT_IDLE,
  ROBOT_MOVING
};

enum MotionMode {
  MOTION_IDLE,
  MOTION_LINEAR,
  MOTION_TURN,
  MOTION_GOTO_TURN,
  MOTION_GOTO_LINEAR
};

struct Motor {
  MotorPins pins;
  volatile long ticks;
  volatile byte lastState;
  long startTicks;
  int dir;
  long lastOdom;
};

class Motors {
public:
  Motors(MotorPins leftPins, MotorPins rightPins, PIDStruct pid, RobotStruct robot);

  void setup();
  void update();

  void moveLinear(float distanceMM);
  void turnBy(float angleDeg);
  void turnTo(float targetRotDeg);
  void moveTo(float targetX, float targetY);

  void stop();

  bool isBusy();
  RobotState getState();

  RobotPosition getPosition();
  RobotPosition getTarget();

  void resetPosition(float x = 0, float y = 0, float rot = 0);

private:
  static Motors* _context;

  Motor _leftMotor;
  Motor _rightMotor;

  PIDStruct _pid;
  RobotStruct _robot;

  RobotPosition _pos;
  RobotPosition _target;

  MotionMode _mode;

  long _targetTicks;

  float _integral;
  long _lastError;

  static void _leftISR();
  static void _rightISR();

  void _encoderISR(Motor &motor);
  byte _getState(Motor &motor);
  int8_t _readEncoderStep(byte lastState, byte currentState);

  void _setMotor(Motor &motor, int speed);
  void _setMotors(int leftSpeed, int rightSpeed);

  void _startLinear(float distanceMM);
  void _startTurn(float angleDeg);

  void _updateMotion();
  void _updateOdometry();

  float _wheelCircumference();
  long _distanceToTicks(float distanceMM);
  long _angleToTicks(float angleDeg);

  void _getTicks(long &left, long &right);

  int _calculatePID(long leftProgress, long rightProgress);
  void _resetPID();

  float _normalizeAngle(float angle);
  float _angleDiff(float target, float current);
};