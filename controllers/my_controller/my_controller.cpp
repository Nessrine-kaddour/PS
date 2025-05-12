#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>

#define TIME_STEP 64  // Time step for the simulation
#define MAX_SPEED 6.28  // Maximum motor speed
#define OBSTACLE_THRESHOLD 800  // Distance threshold (sensor value) for obstacle detection

using namespace webots;

int main() {
  Robot *robot = new Robot();

  // Initialize distance sensors (for detecting obstacles)
  DistanceSensor *ds[2];  // Assume 2 sensors: right and left
  char dsNames[2][10] = {"ds_right", "ds_left"};  // Adjust sensor names based on your setup
  
  for (int i = 0; i < 2; i++) {
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);  // Enable the sensors
  }

  // Initialize motors (4 wheels)
  Motor *wheels[4];
  char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};  // Adjust motor names
  for (int i = 0; i < 4; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);  // Continuous rotation
    wheels[i]->setVelocity(0.0);  // Initial velocity
  }

  // Initialize motor speeds
  double leftSpeed = MAX_SPEED;
  double rightSpeed = MAX_SPEED;

  // Main control loop
  while (robot->step(TIME_STEP) != -1) {
    // Read distance sensor values
    double rightDistance = ds[0]->getValue();
    double leftDistance = ds[1]->getValue();

    // Inform the user (console output for now)
    if (rightDistance < OBSTACLE_THRESHOLD && leftDistance < OBSTACLE_THRESHOLD) {
      std::cout << "Warning: Obstacles detected on both sides! Turning around..." << std::endl;
    } else if (rightDistance < OBSTACLE_THRESHOLD) {
      std::cout << "Warning: Obstacle detected on the right! Turning left..." << std::endl;
    } else if (leftDistance < OBSTACLE_THRESHOLD) {
      std::cout << "Warning: Obstacle detected on the left! Turning right..." << std::endl;
    } else {
      std::cout << "No obstacles detected. Moving forward..." << std::endl;
    }

    // Obstacle avoidance logic
    if (rightDistance < OBSTACLE_THRESHOLD && leftDistance < OBSTACLE_THRESHOLD) {
      // Obstacles on both sides → Stop and turn around
      leftSpeed = -MAX_SPEED / 2;
      rightSpeed = MAX_SPEED / 2;
    } else if (rightDistance < OBSTACLE_THRESHOLD) {
      // Obstacle on the right → Turn left
      leftSpeed = MAX_SPEED / 2;
      rightSpeed = -MAX_SPEED / 2;
    } else if (leftDistance < OBSTACLE_THRESHOLD) {
      // Obstacle on the left → Turn right
      leftSpeed = -MAX_SPEED / 2;
      rightSpeed = MAX_SPEED / 2;
    } else {
      // No obstacles → Move forward
      leftSpeed = MAX_SPEED;
      rightSpeed = MAX_SPEED;
    }

    // Set wheel velocities
    wheels[0]->setVelocity(leftSpeed);   // Left wheel 1
    wheels[1]->setVelocity(rightSpeed);  // Right wheel 1
    wheels[2]->setVelocity(leftSpeed);   // Left wheel 2
    wheels[3]->setVelocity(rightSpeed);  // Right wheel 2
  }

  delete robot;  // Cleanup
  return 0;
}
