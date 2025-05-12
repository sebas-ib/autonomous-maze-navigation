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
#define kpVel 10 //Tune Kp here
#define kdVel 0 //Tune Kd here
#define kiVel 0 //Tune Ki here
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
RobotState last_wall_follow_state = RIGHT_FOLLOW;



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
bool home_detection_enabled = true;
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
  for (int i = 0; i < 50; i++) {
      motors.setSpeeds(150, -150);  // Rotate right
      lineSensors.calibrate();
      delay(1);
  }
  for (int i = 0; i < 50; i++) {
      motors.setSpeeds(-150, 150);  // Rotate left
      lineSensors.calibrate();
      delay(1);
  }
  motors.setSpeeds(0, 0);  // Stop
}

void setup() {
  Serial.begin(9600);
  servo.attach(5);
  disp.setLayout21x8();
  disp.clear();
  disp.gotoXY(0, 0);
  disp.print(F("Trash: 0"));
  calibrateSensors();
  motors.setSpeeds(50, 50);
  delay(3500);
  servo.write(0);
}

void updateOdom() {
  deltaL = encoders.getCountsAndResetLeft();
  deltaR = encoders.getCountsAndResetRight();

  // Increment total encoder cound
  encCountsLeft += deltaL;
  encCountsRight += deltaR;  

  odometry.update_odom(encCountsLeft,encCountsRight, x, y, theta);
}



bool blackIrReading(){

  // Read sensor values
  lineSensors.readCalibrated(lineSensorValues);
  int count = 0;

  // Count the number of sensors over 900 (reading a black square)
  for (int i = 0; i < 5; i++) {
    if (lineSensorValues[i] > 900) {
      count++;
    }
  }

  // If more than 2 are getting these readings, a black square has been found
  if (count >= 3) {
    Serial.println("Black square detected.");
    return true;
  }

  return false;
}

// Similar logic from the black square reading, but different range of values
bool homeSquareReading() {
  lineSensors.readCalibrated(lineSensorValues);
  int count = 0;

  for (int i = 0; i < 5; i++) {
    if (lineSensorValues[i] >= 120 && lineSensorValues[i] <= 200) {
      count++;
    }
  }

  if (count >= 3) {
    Serial.println("Home square detected.");
    return true;
  }

  return false;
}

void loop() {

  // If loop has ended play noise
  if (end) {
    motors.setSpeeds(0, 0);
    pinMode(6, OUTPUT);
    for (unsigned long t = millis(); millis() - t < 500;) {
      digitalWrite(6, HIGH);
      delayMicroseconds(568);
      digitalWrite(6, LOW);
      delayMicroseconds(568);
    }
    while (true);  // Halt
  }
  
  // Loop through states
  while (!end) {
    updateOdom();
    Serial.println(robot_state);
    // Prints the current state

    // Keeps track of state changes previous and current
    if (robot_state != last_state) {
      prev_state = last_state;
      last_state = robot_state;
    }

    // This is so we dont collect trash until after we have left a black square
    if (!trash_detection_enabled && !blackIrReading()) {
      trash_detection_enabled = true;
    }

    if (!home_detection_enabled && !homeSquareReading()) {
      home_detection_enabled = true;
    }

    
    switch (robot_state){
      case RIGHT_FOLLOW: {
        // Face servo to the right
        servo.write(0);

        // Calculations done to find the distance traveled
        float dx = x - prevx;
        float dy = y - prevy;
        float delta_dist = sqrt(dx * dx + dy * dy);
        path_distance += delta_dist;
        prevx = x;
        prevy = y;

        // Around every 5 cm check for a black or home square
        if (fmod(path_distance, 5) <= 1) {
          if (trash_detection_enabled && blackIrReading()) {
            motors.setSpeeds(0, 0);
            delay(250);
            trash_detection_enabled = false;  // disable any trash collection until after leaving black square
            robot_state = COLLECT_TRASH;
            break;
          }
          if (home_detection_enabled && homeSquareReading()) {
            motors.setSpeeds(0, 0);
            delay(250);
            home_detection_enabled = false;   // disable any home detection until after leaving home square
            robot_state = AT_HOME;
            break;
          }
        }
       
        // Every 20cm have the servo face forward
        if (path_distance >= 20) {
          motors.setSpeeds(0, 0);
          servo.write(90);
          delay(100);
          float read = sonar.readDist();

          // If a wall is detected within 15cm switch to obstacle avoidance state
          if (read < 15.0) {
            robot_state = OBSTACLE_AVOIDANCE;
            break;
          }
          // Reset distance traveled
          path_distance = 0;
        }

        // Face servo right and start right wall following using PID Controller
        servo.write(0);
        delay(100);
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
        // Stop robot, and face servo forward
        motors.setSpeeds(0, 0);
        delay(250);
        servo.write(90);
        delay(250);

        // Take a reading
        float read = sonar.readDist();
        // while the reading is less than 15cm, turn right or left depending wheter right or left wall following
        while (read < 15.0) {
            read = sonar.readDist();

            // Turn away from obstacle based on previous wall-following direction
            if (prev_state == RIGHT_FOLLOW) {
              motors.setSpeeds(-base_speed , base_speed );  // Turn left
            } else if (prev_state == LEFT_FOLLOW || prev_state == RETURN_HOME) {
              motors.setSpeeds(base_speed , -base_speed );  // Turn right
            }

            delay(250);
            motors.setSpeeds(0, 0);
            delay(100);
        }

        // Reset distance traveled
        path_distance = 0.0;

        // Resume wall following
        robot_state = prev_state;
        break;
      }
      case COLLECT_TRASH: {
        // Stop robot, then do a 360deg turn
        motors.setSpeeds(0, 0);
        delay(250);
        motors.setSpeeds(-base_speed, base_speed);
        delay(2500);

        // Increment the trash count and update the display
        trash_count++;
        disp.clear();
        disp.gotoXY(0, 0);
        disp.print(F("Trash: "));
        disp.print(trash_count);

        // If trash is >= 3 and the prev state was left wall following, then
        // return home, else go back to right or left wall following
        if (trash_count >=  3 && prev_state == LEFT_FOLLOW) {
          return_turn_complete = false;  // Reset turn status
          robot_state = RETURN_HOME;
        } else {
          robot_state = prev_state;
        }
        break;
      }
      case LEFT_FOLLOW: {
        // Same as right, but for left wall
        servo.write(180);
        float dx = x - prevx;
        float dy = y - prevy;
        float delta_dist = sqrt(dx * dx + dy * dy);
        path_distance += delta_dist;
        prevx = x;
        prevy = y;

        
        if (fmod(path_distance, 5) <= 1) {
          if (trash_detection_enabled && blackIrReading()) {
            motors.setSpeeds(0, 0);
            delay(250);
            trash_detection_enabled = false;
            robot_state = COLLECT_TRASH;
            break;
          }
          if (home_detection_enabled && homeSquareReading()) {
            motors.setSpeeds(0, 0);
            delay(500);

            home_detection_enabled = false;
            robot_state = AT_HOME;
            break;
          }
        }
        
        if (path_distance >= 20) {
          motors.setSpeeds(0, 0);
          servo.write(90);  // Look forward
          delay(100);
          float read = sonar.readDist();
          if (read < 15.0) {
            robot_state = OBSTACLE_AVOIDANCE;
            break;
          }
          path_distance = 0;
        }

        servo.write(180);
        delay(200);
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
        // Once found 3rd trash in left wall follow, we do a 180deg turn once, then right wall follow to get back to home
        if (!return_turn_complete) {
        motors.setSpeeds(-base_speed, base_speed);
        delay(1300);
          return_turn_complete = true;
          break;  // Wait for next loop iteration to start wall follow
        }

        // Right wall-following to home
        servo.write(0);
        float dx = x - prevx;
        float dy = y - prevy;
        float delta_dist = sqrt(dx * dx + dy * dy);
        path_distance += delta_dist;
        prevx = x;
        prevy = y;

        // Dont scan for trash anymore, just home
        if (homeSquareReading()) {
          robot_state = AT_HOME; 
          break;
        }

        if (path_distance >= 20) {
          motors.setSpeeds(0, 0);
          delay(250);
          servo.write(90);
          delay(250);
          float read = sonar.readDist();
          if (read < 15.0) {
            robot_state = OBSTACLE_AVOIDANCE;
            break;
          }
        }
        servo.write(0);
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
        // If robot is at home, check for trash count, if greater than 3 stop, else do left wall following to keep looking
        if (trash_count >= 3){
          end = true;
        } else {
          robot_state = LEFT_FOLLOW;
        }
        break;
      }
    }
  }
}
