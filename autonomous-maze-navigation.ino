// Main.ino
#include <Pololu3piPlus32U4.h>
#include <Servo.h>
#include <math.h>            // sin, cos, atan2, hypot

#include "printOLED.h"
#include "odometry.h"
#include "PIDcontroller.h"
#include "sonar.h"
#include "GridMap.h"
#include "Explorer.h"

using namespace Pololu3piPlus32U4;

// Hardware
Servo    scanServo;
Sonar    sonar(4);
PrintOLED oled;

// Pololu3pi hardware is auto-initialized by calling init()
Motors   motors;
Encoders encoders;

// Odometry constants & object
#define WHEEL_DIAMETER_L   3.2f
#define WHEEL_DIAMETER_R   3.2f
#define COUNTS_PER_REV_L   12
#define COUNTS_PER_REV_R   12
#define WHEEL_BASE_WIDTH   9.6f
#define GEAR_RATIO         75
#define USE_DEAD_RECKONING false

Odometry odometry(
  WHEEL_DIAMETER_L,
  WHEEL_DIAMETER_R,
  WHEEL_BASE_WIDTH,
  COUNTS_PER_REV_L,
  COUNTS_PER_REV_R,
  GEAR_RATIO,
  USE_DEAD_RECKONING
);

// PID heading gains & controller
#define minOutputAng -100
#define maxOutputAng 100
#define kpAng 10 //Tune Kp here
#define kdAng 10 //Tune Kd here
#define kiAng 0.05 //Tune Ki here
#define clamp_iAng 100 //Tune ki integral clamp here
#define base_speedAng 50

#define minOutputVel -100
#define maxOutputVel 100
#define kpVel 10 //Tune Kp here
#define kdVel 50 //Tune Kd here
#define kiVel 0.025 //Tune Ki here
#define clamp_iVel 100 //Tune ki integral clamp here
#define base_speedVel 50

PIDcontroller pidAngController(
  kpAng, kiAng, kdAng, minOutputAng, maxOutputAng, clamp_iAng
);

PIDcontroller pidVelController(
  kpVel, kiVel, kdVel, minOutputVel, maxOutputVel, clamp_iVel
);

// Mapping params & objects
#define CELL_SIZE_CM   10.0f
#define DRIVE_SPEED    50

#define PI 3.14159265358979323846f

GridMap  gridMap;
Explorer explorer(gridMap);

// Robot state, pose & encoder totals
enum RobotState { STATE_MAPPING, STATE_CLEANING, STATE_RETURN, STATE_DONE };
RobotState robotState = STATE_MAPPING;

long    encLeftTotal  = 0;
long    encRightTotal = 0;
int16_t deltaL = 0, deltaR = 0;

float poseX     = CELL_SIZE_CM;
float poseY     = CELL_SIZE_CM;
float poseTheta = 0.0f;


void setup() {
  Serial.begin(9600);
  init();
  scanServo.attach(5);

  gridMap.reset();
  scanAndUpdate();

  gridMap.markFree(poseX, poseY);
  delay(500);
}


void updateOdom() {
  deltaL = encoders.getCountsAndResetLeft();
  deltaR = encoders.getCountsAndResetRight();
  encLeftTotal  += deltaL;
  encRightTotal += deltaR;

  odometry.update_odom(encLeftTotal, encRightTotal,
                       poseX, poseY, poseTheta);
  oled.print_odom(poseX, poseY, poseTheta);
}

float readSonarAt(int ang) {
  scanServo.write(ang + 90);
  delay(200);
  return sonar.readDist();
}

void scanAndUpdate() {
  const int angles[3] = {0, -90, +90};
  for (int a : angles) {
    float dist = readSonarAt(a);
    float angWorld = poseTheta + (a * (PI / 180.0f));
    float cx = poseX + CELL_SIZE_CM * cos(angWorld);
    float cy = poseY + CELL_SIZE_CM * sin(angWorld);

    if (dist > 0 && dist < CELL_SIZE_CM) {
      // If there is something within, find the cell that it jits
      float hitX = poseX + dist * cos(angWorld);
      float hitY = poseY + dist * sin(angWorld);
      gridMap.markOcc(hitX, hitY);
    } else {
      gridMap.markFree(cx, cy);
    }
  }
}

void driveToCell(Point target) {
  // Center of the goal cell in cm
  const float tx = (target.col + 0.5f) * CELL_SIZE_CM;
  const float ty = (target.row + 0.5f) * CELL_SIZE_CM;

  // Tolerances
  const float toleranceDist = 1.0f;    // stop within 1 cm
  const float toleranceAng  = 0.05f;   // 0.05 rad ~ 3°

  while (true) {
    // 1) Update pose
    updateOdom();

    // 2) Compute errors
    float dx = tx - poseX;
    float dy = ty - poseY;
    float distance_to_goal = sqrt(dx*dx + dy*dy);

    float goal_angle = atan2(dy, dx);
    float actual_angle = atan2(sin(poseTheta), cos(poseTheta));

    // 3) PID outputs
    double PIDout_theta   = pidAngController.update(actual_angle, goal_angle);
    double PIDout_distance= pidVelController.update(distance_to_goal, 0.0);

    // 4) Mix into left/right speeds
    int left_speed  = base_speedAng;
    int right_speed = base_speedAng;

    if (PIDout_theta >  toleranceAng) {
      left_speed  = -base_speedAng - abs(PIDout_theta);
      right_speed =  base_speedAng + abs(PIDout_theta);
    } 
    else if (PIDout_theta < -toleranceAng) {
      right_speed = -base_speedAng - abs(PIDout_theta);
      left_speed  =  base_speedAng + abs(PIDout_theta);
    }
    else {
      // small heading error then drive forward with velocity PID
      left_speed  = base_speedVel + abs(PIDout_distance);
      right_speed = base_speedVel + abs(PIDout_distance);
    }


    left_speed  = constrain(left_speed,  -base_speedVel,  base_speedVel);
    right_speed = constrain(right_speed, -base_speedVel,  base_speedVel);

    // 5) Stop condition
    if (distance_to_goal < toleranceDist) {
      motors.setSpeeds(0, 0);
      break;
    }

    motors.setSpeeds(left_speed, right_speed);
  }
}

void loop() {
  updateOdom();
  switch (robotState) {
    case STATE_MAPPING: {
      Point me = gridMap.worldToGrid(poseX, poseY);
      Point frontier = explorer.findNearestFrontierCell(me);
      if (frontier.row == me.row && frontier.col == me.col) {
        Serial.println("Mapping complete");
        robotState = STATE_CLEANING;
      } else {
        PointList path = explorer.planPathTo(me, frontier);
        for (int i = 0; i < path.length; ++i) {
          driveToCell(path.data[i]);
          scanAndUpdate();
        }
      }
      break;
    }
    case STATE_CLEANING:
      Serial.println("Cleaning phase (TODO)");
      robotState = STATE_RETURN;
      break;
    case STATE_RETURN: {
      Serial.println("Returning to start");
      Point me   = gridMap.worldToGrid(poseX, poseY);
      Point home = {0, 0};
      PointList path2 = explorer.planPathTo(me, home);
      for (int i = 0; i < path2.length; ++i) {
        driveToCell(path2.data[i]);
        scanAndUpdate();
      }
      robotState = STATE_DONE;
      break;
    }
    case STATE_DONE:
      Serial.println("Task complete");
      motors.setSpeeds(0,0);
      pinMode(6, OUTPUT);
      for (unsigned long t = millis(); millis() - t < 500; ) {
        digitalWrite(6, HIGH);
        delayMicroseconds(568);   // ~440 Hz half-period
        digitalWrite(6, LOW);
        delayMicroseconds(568);
      }
      while (true) {}
  }
}
