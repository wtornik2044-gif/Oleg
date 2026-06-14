#include "motors.h"
#include "sonar.h"

#define MOTORS_DEBUG false
#define WALL_MIN_DIST 40
#define LEFT_IR_SENSOR 38
#define RIGHT_IR_SENSOR 32
#define CAMERA_PIN 44

MotorPins leftPins = {
  6,   // PWM
  7,  // IN1
  8,  // IN2
  2,   // ENC_A
  3    // ENC_B
};

MotorPins rightPins = {
  11,   // PWM
  9,  // IN1
  10,  // IN2
  18,  // ENC_A
  19   // ENC_B
};

PIDStruct pid = {
  0.28,
  0.001,
  0.07
};

RobotStruct robot = {
  68.2,    // диаметр колеса, мм
  178.2,   // база робота, мм
  1488.0,   // тиков на оборот
  160,     // BASE_PWM
  75,      // MIN_PWM
  255     // MAX_PWM
};

Motors motors(leftPins, rightPins, pid, robot);

unsigned long lastPrintTime = 0;
int testState = 0;
bool commandStarted = false;

void printPosition() {
  RobotPosition pos = motors.getPosition();
  RobotPosition target = motors.getTarget();

  Serial.print("STATE: ");
  if (motors.getState() == ROBOT_MOVING) {
    Serial.print("MOVING");
  } else {
    Serial.print("IDLE");
  }

  Serial.print(" | POS X: ");
  Serial.print(pos.x);
  Serial.print(" Y: ");
  Serial.print(pos.y);
  Serial.print(" ROT: ");
  Serial.print(pos.rot);

  Serial.print(" | TARGET X: ");
  Serial.print(target.x);
  Serial.print(" Y: ");
  Serial.print(target.y);
  Serial.print(" ROT: ");
  Serial.println(target.rot);
}

void motorsDebug() {
  if (!MOTORS_DEBUG) return;

  if (millis() - lastPrintTime > 300) {
    lastPrintTime = millis();
    printPosition();
  }

  if (testState == 0) {
    Serial.println();
    Serial.println("TEST 1: moveLinear +300 mm");
    Serial.println("Robot must move FORWARD.");
    motors.moveLinear(300);
    testState = 1;
  }

  if (testState == 1 && !motors.isBusy()) {
    Serial.println("TEST 1 DONE");
    delay(1500);

    RobotPosition pos = motors.getPosition();
    Serial.print("Expected: X about 300, Y about 0, ROT about 0. Got: ");
    Serial.print(pos.x);
    Serial.print(", ");
    Serial.print(pos.y);
    Serial.print(", ");
    Serial.println(pos.rot);

    testState = 2;
  }

  if (testState == 2) {
    Serial.println();
    Serial.println("TEST 2: moveLinear -300 mm");
    Serial.println("Robot must move BACKWARD to near start.");
    motors.moveLinear(-300);
    testState = 3;
  }

  if (testState == 3 && !motors.isBusy()) {
    Serial.println("TEST 2 DONE");
    delay(1500);

    RobotPosition pos = motors.getPosition();
    Serial.print("Expected: X about 0, Y about 0, ROT about 0. Got: ");
    Serial.print(pos.x);
    Serial.print(", ");
    Serial.print(pos.y);
    Serial.print(", ");
    Serial.println(pos.rot);

    testState = 4;
  }

  if (testState == 4) {
    Serial.println();
    Serial.println("TEST 3: turnBy +90 deg");
    Serial.println("Robot must turn RIGHT or counter-clockwise depending on your convention.");
    motors.turnBy(90);
    testState = 5;
  }

  if (testState == 5 && !motors.isBusy()) {
    Serial.println("TEST 3 DONE");
    delay(1500);

    RobotPosition pos = motors.getPosition();
    Serial.print("Expected: ROT about +90. Got: ");
    Serial.println(pos.rot);

    testState = 6;
  }

  if (testState == 6) {
    Serial.println();
    Serial.println("TEST 4: moveLinear +300 mm after turn");
    Serial.println("Robot must move along new heading.");
    motors.moveLinear(300);
    testState = 7;
  }

  if (testState == 7 && !motors.isBusy()) {
    Serial.println("TEST 4 DONE");
    delay(1500);

    RobotPosition pos = motors.getPosition();
    Serial.print("Expected if ROT=90: X about 0, Y about 300. Got: ");
    Serial.print(pos.x);
    Serial.print(", ");
    Serial.print(pos.y);
    Serial.print(", ");
    Serial.println(pos.rot);

    testState = 8;
  }

  if (testState == 8) {
    Serial.println();
    Serial.println("TEST 5: moveTo(0, 0)");
    Serial.println("Robot must turn to start point and drive near X=0 Y=0.");
    motors.moveTo(0, 0);
    testState = 9;
  }

  if (testState == 9 && !motors.isBusy()) {
    Serial.println("TEST 5 DONE");
    delay(1500);

    RobotPosition pos = motors.getPosition();
    Serial.print("Expected: X about 0, Y about 0. Got: ");
    Serial.print(pos.x);
    Serial.print(", ");
    Serial.print(pos.y);
    Serial.print(", ");
    Serial.println(pos.rot);

    Serial.println();
    Serial.println("=== ALL TESTS DONE ===");
    motors.stop();

    testState = 10;
  }

  if (testState == 10) {
    motors.stop();
    delay(1000);
  }
}

Sonar sonar(48, 50);

bool getLeftIRStatus() {
  return !digitalRead(LEFT_IR_SENSOR);
}

bool getRightIRStatus() {
  return !digitalRead(RIGHT_IR_SENSOR);
}

bool getCameraStatus() {
  return digitalRead(CAMERA_PIN);
}

enum RobotMainState {
  SEARCH_WALL,
  CREATE_PERIMETER
};

RobotMainState mainState;

void setup() {
  Serial.begin(9600);
  delay(1000);

  Serial.println("=== ROBOT INIT ===");

  motors.setup();
  motors.resetPosition(0, 0, 0);

  sonar.setup();

  pinMode(LEFT_IR_SENSOR, INPUT);
  pinMode(RIGHT_IR_SENSOR, INPUT);
  pinMode(CAMERA_PIN, INPUT);

  mainState = SEARCH_WALL;

  delay(2000);
}

void loop() {
  motors.update();
  motorsDebug();

  sonar.update();

  Serial.print(getCameraStatus());
  Serial.print("\n");

  if (mainState == SEARCH_WALL) {
    searchWall();
  } else if (mainState == CREATE_PERIMETER) {
    createPerimeter();
  }
}

// SEARCH WALL
void searchWall() {
  const int distanceToWall = sonar.getDistance();

  if (distanceToWall > 0 && distanceToWall <= WALL_MIN_DIST) {
    motors.stop();
    mainState = CREATE_PERIMETER;
    prepareCreatePerimeter();
    return;
  }

  if (!motors.isBusy() && distanceToWall > WALL_MIN_DIST) {
    motors.moveLinear(distanceToWall - WALL_MIN_DIST);
  }
}

// PERIMETER

#define PERIMETER_MAX_POINTS 50

#define FRONT_WALL_DIST 40
#define FRONT_DANGER_DIST 15

#define FOLLOW_STEP_MM 300
#define WALL_LOST_FORWARD_MM 90

#define SIDE_ALIGN_TURN_STEP_DEG 6
#define CORNER_TURN_STEP_DEG 12
#define WALL_SEARCH_TURN_STEP_DEG -8

#define RETURN_DISTANCE_MM 180
#define MIN_PERIMETER_POINTS 4
#define MIN_PERIMETER_TIME_MS 8000

bool perimeterActionStarted = false;

struct PerimeterPoint {
  float x;
  float y;
};

PerimeterPoint perimeter[PERIMETER_MAX_POINTS];
int perimeterPointsCount = 0;

RobotPosition perimeterStart;
unsigned long perimeterStartTime = 0;

enum PerimeterState {
  PERIMETER_ALIGN_SIDE,
  PERIMETER_FOLLOW,
  PERIMETER_FRONT_CORNER,
  PERIMETER_WALL_LOST,
  PERIMETER_FINISHED
};

PerimeterState perimeterState = PERIMETER_ALIGN_SIDE;

void addPerimeterPoint() {
  if (perimeterPointsCount >= PERIMETER_MAX_POINTS) return;

  RobotPosition pos = motors.getPosition();

  perimeter[perimeterPointsCount].x = pos.x;
  perimeter[perimeterPointsCount].y = pos.y;
  perimeterPointsCount++;

  Serial.print("PERIMETER POINT ");
  Serial.print(perimeterPointsCount);
  Serial.print(": X=");
  Serial.print(pos.x);
  Serial.print(" Y=");
  Serial.println(pos.y);
}

float distanceToPerimeterStart() {
  RobotPosition pos = motors.getPosition();

  float dx = pos.x - perimeterStart.x;
  float dy = pos.y - perimeterStart.y;

  return sqrt(dx * dx + dy * dy);
}

bool isPerimeterClosed() {
  if (millis() - perimeterStartTime < MIN_PERIMETER_TIME_MS) return false;
  if (perimeterPointsCount < MIN_PERIMETER_POINTS) return false;

  return distanceToPerimeterStart() < RETURN_DISTANCE_MM;
}

void printPerimeter() {
  Serial.println("=== PERIMETER POINTS ===");

  for (int i = 0; i < perimeterPointsCount; i++) {
    Serial.print(i);
    Serial.print(": X=");
    Serial.print(perimeter[i].x);
    Serial.print(" Y=");
    Serial.println(perimeter[i].y);
  }
}

void onPerimeterFinished() {
  Serial.println("NEXT STAGE PLACEHOLDER");
}

void prepareCreatePerimeter() {
  Serial.println("=== PREPARE CREATE PERIMETER ===");

  motors.stop();

  perimeterPointsCount = 0;
  perimeterStartTime = millis();

  perimeterState = PERIMETER_ALIGN_SIDE;
  perimeterActionStarted = false;
}

void finishPerimeter() {
  motors.stop();
  addPerimeterPoint();

  perimeterState = PERIMETER_FINISHED;

  Serial.println("=== PERIMETER FINISHED ===");
  printPerimeter();

  onPerimeterFinished();
}

void changePerimeterState(int newState) {
  if (perimeterState == newState) return;

  perimeterState = newState;
  perimeterActionStarted = false;
}

void createPerimeter() {
  if (perimeterState == PERIMETER_FINISHED) return;

  const int frontDistance = sonar.getDistance();
  const bool leftWall = getLeftIRStatus();

  if (isPerimeterClosed()) {
    finishPerimeter();
    return;
  }

  switch (perimeterState) {

    case PERIMETER_ALIGN_SIDE:
      if (leftWall) {
        motors.stop();

        perimeterStart = motors.getPosition();
        addPerimeterPoint();

        Serial.println("SIDE WALL FOUND. START FOLLOWING.");

        changePerimeterState(PERIMETER_FOLLOW);
        return;
      }

      if (!perimeterActionStarted) {
        motors.turnBy(120);          // один длинный поворот
        perimeterActionStarted = true;
      }

      return;


    case PERIMETER_FOLLOW:
      if (frontDistance > 0 && frontDistance <= FRONT_WALL_DIST) {
        motors.stop();
        addPerimeterPoint();

        Serial.println("FRONT WALL FOUND. TURN RIGHT.");

        changePerimeterState(PERIMETER_FRONT_CORNER);
        return;
      }

      if (!leftWall) {
        motors.stop();
        addPerimeterPoint();

        Serial.println("LEFT WALL LOST. OUTER CORNER.");

        changePerimeterState(PERIMETER_WALL_LOST);
        return;
      }

      if (!motors.isBusy()) {
        motors.moveLinear(FOLLOW_STEP_MM);
      }

      return;


    case PERIMETER_FRONT_CORNER:
      if (!perimeterActionStarted) {
        motors.turnBy(100);          // один нормальный поворот, не по 12°
        perimeterActionStarted = true;
      }

      if ((frontDistance <= 0 || frontDistance > FRONT_WALL_DIST) && leftWall) {
        motors.stop();

        Serial.println("CORNER PASSED. FOLLOW WALL.");

        changePerimeterState(PERIMETER_FOLLOW);
        return;
      }

      if (!motors.isBusy()) {
        changePerimeterState(PERIMETER_FOLLOW);
      }

      return;


    case PERIMETER_WALL_LOST:
      if (!perimeterActionStarted) {
        motors.moveLinear(WALL_LOST_FORWARD_MM);
        perimeterActionStarted = true;
      }

      if (!motors.isBusy()) {
        motors.turnBy(-90);
        changePerimeterState(PERIMETER_ALIGN_SIDE);
      }

      return;


    case PERIMETER_FINISHED:
      return;
  }
}