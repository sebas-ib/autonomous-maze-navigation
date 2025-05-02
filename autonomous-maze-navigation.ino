
// This is our second solution after we couldn't get our 2d array mapping using BFS to work, this is the wall following approach

#include <Pololu3piPlus32U4.h>
#include <Servo.h>
#include "sonar.h"
#include <math.h>
#include "odometry.h"
#include "PIDcontroller.h"
using namespace Pololu3piPlus32U4;

//Odometry Parameters
#define diaL 3.2
#define diaR  3.2
#define nL 12
#define nR 12
#define w 9.6
#define gearRatio 75
#define DEAD_RECKONING false

//Update kp and kd based on your testing
#define minOutputVel -100
#define maxOutputVel 100
#define kpVel 15 //Tune Kp here
#define kdVel 100 //Tune Kd here
#define kiVel 0.025 //Tune Ki here
#define clamp_iVel 100 //Tune ki integral clamp here
#define base_speed 150

Motors motors;
Encoders encoders;
LineSensors lineSensors;
Servo servo;
Sonar sonar(4);

enum RobotState { RIGHT_FOLLOW, OBSTACLE_AVOIDANCE, LEFT_FOLLOW, COLLECT_TRASH, RETURN_HOME, AT_HOME};
RobotState robot_state = RIGHT_FOLLOW; 

Odometry odometry(diaL, diaR, w, nL, nR, gearRatio, DEAD_RECKONING);
PIDcontroller pid_vel_controller(kpVel, kiVel, kdVel, minOutputVel, maxOutputVel, clamp_iVel); //Uncomment after you import PIDController

int16_t deltaL=0, deltaR=0;
int16_t encCountsLeft = 0, encCountsRight = 0;
float x, y, theta;

float prevx = 0;
float prevy = 0;

float path_distance = 0.0;

const double distFromWall=10.0; 

double wallDist;

// float prev_time = 0;

void calibrateSensors()
{
  //TASK 2.1a
  //Implement calibration for IR Sensors
  //Hint: Have your robot turn to the left and right to calibrate sensors.
  for (int i = 0; i < 100; i++) {
      motors.setSpeeds(50, -50);  // Rotate right
      lineSensors.calibrate();
      delay(25);
  }
  for (int i = 0; i < 100; i++) {
      motors.setSpeeds(-50, 50);  // Rotate left
      lineSensors.calibrate();
      delay(25);
  }
  motors.setSpeeds(0, 0);  // Stop
}

void setup() {
  Serial.begin(9600);
  servo.attach(5);
  calibrateSensors();
  motors.setSpeeds(50, 50);
  delay(1500);
  servo.write(0);
  // prev_time = millis();
}

void updateOdom() {
  deltaL = encoders.getCountsAndResetLeft();
  deltaR = encoders.getCountsAndResetRight();

  // Increment total encoder cound
  encCountsLeft += deltaL;
  encCountsRight += deltaR;  

  odometry.update_odom(encCountsLeft,encCountsRight, x, y, theta);
}

bool avoidObstacle(RobotState current_state){
  if (path_distance >= 20) {
    motors.setSpeeds(0, 0);
    delay(250);
    servo.write(90);  // Look forward
    delay(250);

    float read = sonar.readDist();

    while (read < 10.0) {
        read = sonar.readDist();
        if(current_state == RIGHT_FOLLOW) {
          motors.setSpeeds(-base_speed / 2, base_speed / 2);
        } else if(current_state == LEFT_FOLLOW) {
          read = sonar.readDist();
          motors.setSpeeds(base_speed / 2, -base_speed / 2);
        }
        delay(500);
        motors.setSpeeds(0, 0);
        delay(100);
    }

    // Reset path distance after check
    path_distance = 0.0;
  }
}

bool collectTrash(){
  if (path_distance >= 20) {
    motors.setSpeeds(0, 0);
    delay(250);
    servo.write(90);  // Look forward
    delay(250);

    float read = sonar.readDist();

    while (read < 10.0) {
        read = sonar.readDist();
        if(current_state == RIGHT_FOLLOW) {
          motors.setSpeeds(-base_speed / 2, base_speed / 2);
        } else if(current_state == LEFT_FOLLOW) {
          read = sonar.readDist();
          motors.setSpeeds(base_speed / 2, -base_speed / 2);
        }
        delay(500);
        motors.setSpeeds(0, 0);
        delay(100);
    }

    // Reset path distance after check
    path_distance = 0.0;
  }
}

void loop() {
  updateOdom();
  switch (robot_state){
    case RIGHT_FOLLOW: {
      servo.write(0);
      float dx = x - prevx;
      float dy = y - prevy;
      float delta_dist = sqrt(dx * dx + dy * dy);

      path_distance += delta_dist;

      prevx = x;
      prevy = y;

      avoidObstacle(robot_state);

      wallDist = sonar.readDist();

      double PIDout;

      PIDout = pid_vel_controller.update(wallDist, distFromWall); 
      int left_speed = base_speed;
      int right_speed = base_speed;

      if (PIDout > 0) {
        left_speed = left_speed - abs(PIDout);
        right_speed = right_speed + abs(PIDout);
      }

      if (PIDout < 0) {
        right_speed = right_speed - abs(PIDout);
        left_speed = left_speed + abs(PIDout);
      }

      motors.setSpeeds(left_speed, right_speed);
    }
    case COLLECT_TRASH {
      System.println(lineSensors.)
    }
  }
}
