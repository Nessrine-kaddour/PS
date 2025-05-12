#include <stdio.h>
#include <stdlib.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distancesensor.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28  // Maximum motor speed
#define OBSTACLE_THRESHOLD 800.0  // Distance threshold for obstacle detection

// Function to initialize devices
void initialize_devices(WbDeviceTag *ds, WbDeviceTag *wheels[4]) {
    // Initialize distance sensors
    ds[0] = wb_robot_get_device("ds_right");
    ds[1] = wb_robot_get_device("ds_left");
    wb_distance_sensor_enable(ds[0], TIME_STEP);
    wb_distance_sensor_enable(ds[1], TIME_STEP);

    // Initialize motors (4 wheels)
    wheels[0] = wb_robot_get_device("wheel1");
    wheels[1] = wb_robot_get_device("wheel2");
    wheels[2] = wb_robot_get_device("wheel3");
    wheels[3] = wb_robot_get_device("wheel4");

    for (int i = 0; i < 4; i++) {
        wb_motor_set_position(wheels[i], INFINITY);  // Continuous rotation
        wb_motor_set_velocity(wheels[i], 0.0);      // Initial velocity
    }
}

// Function to control the robot movement based on sensor input
void move_robot(WbDeviceTag *ds, WbDeviceTag *wheels[4], double *left_speed, double *right_speed) {
    // Read distance sensor values
    double right_distance = wb_distance_sensor_get_value(ds[0]);
    double left_distance = wb_distance_sensor_get_value(ds[1]);

    // Inform the user via console output
    if (right_distance < OBSTACLE_THRESHOLD && left_distance < OBSTACLE_THRESHOLD) {
        printf("Warning: Obstacles detected on both sides! Turning around...\n");
    } else if (right_distance < OBSTACLE_THRESHOLD) {
        printf("Warning: Obstacle detected on the right! Turning left...\n");
    } else if (left_distance < OBSTACLE_THRESHOLD) {
        printf("Warning: Obstacle detected on the left! Turning right...\n");
    } else {
        printf("No obstacles detected. Moving forward...\n");
    }

    // Obstacle avoidance logic
    if (right_distance < OBSTACLE_THRESHOLD && left_distance < OBSTACLE_THRESHOLD) {
        // Obstacles on both sides → Stop and turn around
        *left_speed = -MAX_SPEED / 2;
        *right_speed = MAX_SPEED / 2;
    } else if (right_distance < OBSTACLE_THRESHOLD) {
        // Obstacle on the right → Turn left
        *left_speed = MAX_SPEED / 2;
        *right_speed = -MAX_SPEED / 2;
    } else if (left_distance < OBSTACLE_THRESHOLD) {
        // Obstacle on the left → Turn right
        *left_speed = -MAX_SPEED / 2;
        *right_speed = MAX_SPEED / 2;
    } else {
        // No obstacles → Move forward
        *left_speed = MAX_SPEED;
        *right_speed = MAX_SPEED;
    }

    // Set motor velocities
    wb_motor_set_velocity(wheels[0], *left_speed);   // Left wheel 1
    wb_motor_set_velocity(wheels[1], *right_speed);  // Right wheel 1
    wb_motor_set_velocity(wheels[2], *left_speed);   // Left wheel 2
    wb_motor_set_velocity(wheels[3], *right_speed);  // Right wheel 2
}

int main() {
    // Initialize Webots and the robot
    wb_robot_init();

    // Device tags
    WbDeviceTag ds[2];  // Distance sensors (right and left)
    WbDeviceTag *wheels[4];  // Motors (4 wheels)

    // Initialize devices
    initialize_devices(ds, wheels);

    // Variables for motor speed
    double left_speed = MAX_SPEED;
    double right_speed = MAX_SPEED;

    // Main loop
    while (wb_robot_step(TIME_STEP) != -1) {
        move_robot(ds, wheels, &left_speed, &right_speed);
    }

    // Cleanup
    wb_robot_cleanup();
