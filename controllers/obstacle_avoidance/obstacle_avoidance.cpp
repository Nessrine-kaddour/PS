#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <iostream> // For debugging output

#define TIME_STEP 64
using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();

  // Initialize distance sensors
  DistanceSensor *ds[2];
  char dsNames[2][10] = {"ds_right", "ds_left"};
  for (int i = 0; i < 2; i++) {
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    if (ds[i] == nullptr) {
      std::cerr << "Error: Distance sensor " << dsNames[i] << " not found!" << std::endl;
      return -1;
    }
    ds[i]->enable(TIME_STEP);
  }

  // Initialize motors
  Motor *wheels[4];
  char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (int i = 0; i < 4; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    if (wheels[i] == nullptr) {
      std::cerr << "Error: Motor " << wheels_names[i] << " not found!" << std::endl;
      return -1;
    }
    wheels[i]->setPosition(INFINITY); // Enable continuous rotation
    wheels[i]->setVelocity(0.0);     // Set initial velocity
  }

  // Obstacle avoidance logic
  int avoidObstacleCounter = 0;

  while (robot->step(TIME_STEP) != -1) {
    double leftSpeed = 1.0;  // Default speed for left wheels
    double rightSpeed = 1.0; // Default speed for right wheels

    if (avoidObstacleCounter > 0) {
      avoidObstacleCounter--;
      leftSpeed = 1.0;
      rightSpeed = -1.0; // Turn right to avoid obstacle
    } else {
      // Check distance sensors
      for (int i = 0; i < 2; i++) {
        if (ds[i]->getValue() < 950.0) { // Detect obstacle
          avoidObstacleCounter = 100; // Trigger obstacle avoidance
        }
      }
    }

    // Set motor velocities
    wheels[0]->setVelocity(leftSpeed);   // Left front
    wheels[1]->setVelocity(rightSpeed);  // Right front
    wheels[2]->setVelocity(leftSpeed);   // Left back
    wheels[3]->setVelocity(rightSpeed);  // Right back
  }

  delete robot; // Cleanup
  return 0;     // EXIT_SUCCESS
}
