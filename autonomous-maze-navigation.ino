
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
#define base_speed 120

#define kpAngle 40
#define kdAngle 0
#define kiAngle 0
#define minOutputAng -80
#define maxOutputAng 80
#define clamp_iAng 100

Motors motors;
Encoders encoders;
LineSensors lineSensors;
Servo servo;
OLED disp;

Sonar sonar(4);

enum RobotState { RIGHT_FOLLOW, OBSTACLE_AVOIDANCE, LEFT_FOLLOW, COLLECT_TRASH, RETURN_HOME, AT_HOME};
RobotState robot_state = RIGHT_FOLLOW;
RobotState prev_state = RIGHT_FOLLOW;
RobotState last_state = robot_state;


Odometry odometry(diaL, diaR, w, nL, nR, gearRatio, DEAD_RECKONING);
PIDcontroller pid_vel_controller(kpVel, kiVel, kdVel, minOutputVel, maxOutputVel, clamp_iVel); //Uncomment after you import PIDController
PIDcontroller pid_ang_controller(kpAngle, kiAngle, kdAngle, minOutputAng, maxOutputAng, clamp_iAng);

int16_t deltaL=0, deltaR=0;
int16_t encCountsLeft = 0, encCountsRight = 0;
float x, y, theta;

float prevx = 0;
float prevy = 0;

float path_distance = 0.0;

const double distFromWall=10.0; 
unsigned int lineSensorValues[5];
bool trash_detection_enabled = true;
bool return_turn_complete = false;



bool end = false;

double wallDist;
int trash_count = 0;


// float prev_time = 0;

void calibrateSensors()
{
  //TASK 2.1a
  //Implement calibration for IR Sensors
  //Hint: Have your robot turn to the left and right to calibrate sensors.
  for (int i = 0; i < 100; i++) {
      motors.setSpeeds(150, -150);  // Rotate right
      lineSensors.calibrate();
      delay(2);
  }
  for (int i = 0; i < 100; i++) {
      motors.setSpeeds(-150, 150);  // Rotate left
      lineSensors.calibrate();
      delay(2);
  }
  motors.setSpeeds(0, 0);  // Stop
}

void setup() {
  Serial.begin(9600);
  servo.attach(5);
  calibrateSensors();
  motors.setSpeeds(50, 50);
  delay(3500);
  servo.write(0);

  disp.setLayout21x8();
  disp.clear();
  disp.gotoXY(0, 0);
  disp.print(F("Trash: 0"));
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

bool collectTrash(RobotState current_state){
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

bool blackIrReading(){
  lineSensors.readCalibrated(lineSensorValues);
  int len = sizeof(lineSensorValues) / sizeof(lineSensorValues[0]);

  for (int i = 0; i < len; i++) {
    if (lineSensorValues[i] > 900) {
      return true;
    }
  }
  return false;

}

bool homeSquareReading() {
  lineSensors.readCalibrated(lineSensorValues);
  int len = sizeof(lineSensorValues) / sizeof(lineSensorValues[0]);

  for (int i = 0; i < len; i++) {
    if (lineSensorValues[i] > 110 && lineSensorValues[i] < 600) {
      return true;
    }
  }
  return false;
}

void loop() {
  while (!end) {
    updateOdom();


    // Keeps track of state changes previous and current
    if (robot_state != last_state) {
      prev_state = last_state;
      last_state = robot_state;
    }

    // This is so we dont collect trash until after we have left a black square
    if (!trash_detection_enabled && !blackIrReading()) {
      trash_detection_enabled = true;
    }

    switch (robot_state){
      case RIGHT_FOLLOW: {
        servo.write(0);
        float dx = x - prevx;
        float dy = y - prevy;
        float delta_dist = sqrt(dx * dx + dy * dy);

        path_distance += delta_dist;

        prevx = x;
        prevy = y;

        if (trash_detection_enabled && blackIrReading()) {
          motors.setSpeeds(0, 0);
          delay(250);
          trash_detection_enabled = false;  // disable any trash collection until after leaving black square
          robot_state = COLLECT_TRASH;
          return;
        }

        if (homeSquareReading()) {
          motors.setSpeeds(0, 0);
          delay(500);

          robot_state = AT_HOME;
          return;
        }

        if (path_distance >= 20) {
          motors.setSpeeds(0, 0);
          delay(250);
          servo.write(90);  // Face forward
          delay(250);
          float read = sonar.readDist();
          if (read < 10.0) {
            robot_state = OBSTACLE_AVOIDANCE;
            return;
          }
        }


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
        break;
      }
      case OBSTACLE_AVOIDANCE: {
        motors.setSpeeds(0, 0);
        delay(250);
        servo.write(90);  // Look forward
        delay(250);

        float read = sonar.readDist();

        while (read < 10.0) {
            read = sonar.readDist();

            // Turn away from obstacle based on previous wall-following direction
            if (prev_state == RIGHT_FOLLOW) {
              motors.setSpeeds(-base_speed / 2, base_speed / 2);  // Turn left
            } else if (prev_state == LEFT_FOLLOW) {
              motors.setSpeeds(base_speed / 2, -base_speed / 2);  // Turn right
            }
            delay(500);
            motors.setSpeeds(0, 0);
            delay(100);
        }

        // Reset distance traveled
        path_distance = 0.0;

        // Resume wall following

        robot_state = prev_state;  // Or LEFT_FOLLOW if you use left-side following
        break;
      }
      case COLLECT_TRASH: {
        motors.setSpeeds(0, 0);
        delay(250);

        // Set target theta to add 3 full turns or 6PI radians
        float theta_start = theta;
        float theta_target = theta_start + 6 * M_PI;

        while (abs(theta - theta_target) > 0.05) {
            updateOdom();  // Refresh current theta

            float output = pid_ang_controller.update(theta, theta_target);
            motors.setSpeeds(output, -output);

            delay(10);
        }

        // Stop motors
        motors.setSpeeds(0, 0);

        trash_count++;
        disp.clear();
        disp.gotoXY(0, 0);
        disp.print(F("Trash: "));
        disp.print(trash_count);

        delay(250);

        // Resume previous behavior
        if (trash_count == 3 && prev_state == LEFT_FOLLOW) {
          return_turn_complete = false;  // Reset turn status
          robot_state = RETURN_HOME;
        } else {
          robot_state = prev_state;
        }
        break;
      }
      case LEFT_FOLLOW: {
        servo.write(180);
        float dx = x - prevx;
        float dy = y - prevy;
        float delta_dist = sqrt(dx * dx + dy * dy);

        path_distance += delta_dist;

        prevx = x;
        prevy = y;

        // Trash collection trigger
        if (trash_detection_enabled && blackIrReading()) {
          motors.setSpeeds(0, 0);
          delay(250);
          trash_detection_enabled = false;
          robot_state = COLLECT_TRASH;
          return;
        }


        if (path_distance >= 20) {
          motors.setSpeeds(0, 0);
          delay(250);
          servo.write(90);  // Look forward
          delay(250);
          float read = sonar.readDist();
          if (read < 10.0) {
            robot_state = OBSTACLE_AVOIDANCE;
            return;
          }
        }

        wallDist = sonar.readDist();
        double PIDout = pid_vel_controller.update(wallDist, distFromWall);

        int left_speed = base_speed;
        int right_speed = base_speed;

        if (PIDout > 0) {
          right_speed = right_speed - abs(PIDout);
          left_speed = left_speed + abs(PIDout);
        }

        if (PIDout < 0) {
          left_speed = left_speed - abs(PIDout);
          right_speed = right_speed + abs(PIDout);
        }

        motors.setSpeeds(left_speed, right_speed);
        break;
      }
      case RETURN_HOME: {
        if (!return_turn_complete) {
          // Do 180-degree turn
          float theta_start = theta;
          float theta_target = theta_start + M_PI;

          while (abs(theta - theta_target) > 0.05) {
            updateOdom();
            float output = pid_ang_controller.update(theta, theta_target);
            motors.setSpeeds(output, -output);
            delay(10);
          }

          motors.setSpeeds(0, 0);
          delay(250);
          return_turn_complete = true;
          return;  // Wait for next loop iteration to start wall follow
        }

        // Right wall-following to home
        servo.write(0);
        float dx = x - prevx;
        float dy = y - prevy;
        float delta_dist = sqrt(dx * dx + dy * dy);
        path_distance += delta_dist;
        prevx = x;
        prevy = y;

        if (homeSquareReading()) {
          motors.setSpeeds(0, 0);
          delay(500);
          robot_state = AT_HOME;
          return;
        }

        if (path_distance >= 20) {
          motors.setSpeeds(0, 0);
          delay(250);
          servo.write(90);
          delay(250);
          float read = sonar.readDist();
          if (read < 10.0) {
            robot_state = OBSTACLE_AVOIDANCE;
            return;
          }
        }

        wallDist = sonar.readDist();
        double PIDout = pid_vel_controller.update(wallDist, distFromWall);

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
        break;
      }

      case AT_HOME: {
          if (trash_count >= 3) {
            end = true;
          } else {
            robot_state = LEFT_FOLLOW;
          }
        break;
      }
    }
  }
}
