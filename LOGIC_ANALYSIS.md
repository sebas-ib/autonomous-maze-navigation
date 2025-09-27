# Autonomous Maze Navigation Code Review

## Objective of the System

The project controls a Pololu 3pi+ 32U4 robot equipped with a servo-mounted ultrasonic sensor, line sensors, and encoders. Its finite-state machine follows walls to explore a maze, detects "trash" on black tiles, tracks odometry, and eventually returns to a designated home square after collecting three trash items. 【F:autonomous-maze-navigation/autonomous-maze-navigation.ino†L33-L128】【F:autonomous-maze-navigation/autonomous-maze-navigation.ino†L201-L455】

## Logic Issues Identified

### 1. PID Timing and Division-by-Zero Risks
- **Location:** `PIDcontroller::update` in `PIDcontroller.cpp`. 【F:autonomous-maze-navigation/PIDcontroller.cpp†L38-L69】
- **Issue:** The elapsed time `dt` is measured in raw milliseconds and can be zero. Using millisecond units inflates the integral and derivative terms by ~1000×, and dividing by zero when the loop runs faster than 1 ms will crash the controller.
- **Fix Suggestion:** Convert `dt` to seconds (divide by 1000.0) and guard against values ≤ 0 before using it in the integral and derivative computations.

### 2. Obstacle Avoidance Keeps Turning After Clearance
- **Location:** `case OBSTACLE_AVOIDANCE` in the main FSM. 【F:autonomous-maze-navigation/autonomous-maze-navigation.ino†L273-L304】
- **Issue:** Inside the `while (read < 15.0)` loop the code immediately turns before checking the updated range reading, so the robot executes at least one additional turn even when the obstacle is already gone.
- **Fix Suggestion:** Take a fresh distance measurement *after* the turn (or continue the loop with `continue`/`break` logic) so the robot exits the loop as soon as the path is clear.

### 3. Return-Home Distance Never Resets
- **Location:** `case RETURN_HOME` when scanning ahead after 20 cm. 【F:autonomous-maze-navigation/autonomous-maze-navigation.ino†L391-L444】
- **Issue:** Unlike the left/right wall-follow states, `path_distance` is not reset after the forward sonar scan. Once `path_distance` passes 20 cm, the robot stops every cycle, constantly rescanning and stalling progress home.
- **Fix Suggestion:** Reset `path_distance` to zero after the forward-look block to match the other wall-following behaviors.

### 4. Turn-to-Return Leaves Motors Spinning
- **Location:** First branch of `case RETURN_HOME`. 【F:autonomous-maze-navigation/autonomous-maze-navigation.ino†L391-L399】
- **Issue:** After the 1.3 s blocking delay for the 180° turn, the motors remain commanded at `(-base_speed, base_speed)` until the next loop iteration updates them, causing unnecessary overshoot.
- **Fix Suggestion:** Stop the motors immediately after the delay before breaking out of the case.

### 5. Distance-Based Trigger Fires Repeatedly Near Thresholds
- **Location:** Both wall-follow states use `fmod(path_distance, 5) <= 1` to trigger tile checks. 【F:autonomous-maze-navigation/autonomous-maze-navigation.ino†L214-L230】【F:autonomous-maze-navigation/autonomous-maze-navigation.ino†L339-L356】
- **Issue:** Because the condition remains true for roughly the first centimeter past each 5 cm multiple, the robot keeps re-evaluating the same square multiple times, which wastes time and can re-trigger behaviors.
- **Fix Suggestion:** Track the last trigger distance (e.g., increment a counter or reset `path_distance` after each check) so each 5 cm interval is only processed once.
