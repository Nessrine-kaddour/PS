#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

#define TIME_STEP 64  // Simulation time step in milliseconds
#define MAX_SPEED 6.28  // Maximum motor speed

using namespace webots;

int main(int argc, char **argv) {
    Robot *robot = new Robot();

    // Initialize distance sensors
    DistanceSensor *ds[2];
    char dsNames[2][10] = {"ds_right", "ds_left"};

    for (int i = 0; i < 2; i++) {
        ds[i] = robot->getDistanceSensor(dsNames[i]);
        ds[i]->enable(TIME_STEP);
    }

    // Initialize motors
    Motor *wheels[4];
    char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
    for (int i = 0; i < 4; i++) {
        wheels[i] = robot->getMotor(wheels_names[i]);
        wheels[i]->setPosition(INFINITY);  // Continuous rotation
        wheels[i]->setVelocity(0.0);       // Initial velocity
    }

    // Variables for motor speed
    double leftSpeed = MAX_SPEED;
    double rightSpeed = MAX_SPEED;

    // Main loop: Runs until simulation is terminated
    while (robot->step(TIME_STEP) != -1) {
        // Read distance sensor values
        double rightDistance = ds[0]->getValue();
        double leftDistance = ds[1]->getValue();

        // Obstacle avoidance logic
        if (rightDistance < 800.0 && leftDistance < 800.0) { 
            // Obstacles on both sides → Stop and turn around
            leftSpeed = -MAX_SPEED / 2;
            rightSpeed = MAX_SPEED / 2;
        } else if (rightDistance < 800.0) { 
            // Obstacle on the right → Turn left
            leftSpeed = MAX_SPEED / 2;
            rightSpeed = -MAX_SPEED / 2;
        } else if (leftDistance < 800.0) { 
            // Obstacle on the left → Turn right
            leftSpeed = -MAX_SPEED / 2;
            rightSpeed = MAX_SPEED / 2;
        } else { 
            // No obstacles → Move forward
            leftSpeed = MAX_SPEED;
            rightSpeed = MAX_SPEED;
        }

        // Set wheel velocities
        wheels[0]->setVelocity(leftSpeed);
        wheels[1]->setVelocity(rightSpeed);
        wheels[2]->setVelocity(leftSpeed);
        wheels[3]->setVelocity(rightSpeed);
    }

    delete robot;  // Cleanup
    return 0;  // Exit success
}
