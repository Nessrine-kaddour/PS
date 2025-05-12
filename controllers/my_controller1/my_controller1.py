from controller import Robot, DistanceSensor, Motor

# Time step for the simulation
TIME_STEP = 64
MAX_SPEED = 6.28  # Maximum motor speed
OBSTACLE_THRESHOLD = 800  # Distance threshold for obstacle detection

class ElderlyRobot(Robot):
    def __init__(self):
        super().__init__()

        # Initialize distance sensors (right and left)
        self.ds = [None, None]
        self.ds_names = ["ds_right", "ds_left"]
        for i in range(2):
            self.ds[i] = self.getDistanceSensor(self.ds_names[i])
            self.ds[i].enable(TIME_STEP)

        # Initialize motors (4 wheels)
        self.wheels = [None, None, None, None]
        self.wheel_names = ["wheel1", "wheel2", "wheel3", "wheel4"]
        for i in range(4):
            self.wheels[i] = self.getMotor(self.wheel_names[i])
            self.wheels[i].setPosition(float('inf'))  # Continuous rotation
            self.wheels[i].setVelocity(0.0)  # Initial velocity

        # Initial speed
        self.left_speed = MAX_SPEED
        self.right_speed = MAX_SPEED

    def move(self):
        # Read distance sensor values
        right_distance = self.ds[0].getValue()
        left_distance = self.ds[1].getValue()

        # Inform the user via console output
        if right_distance < OBSTACLE_THRESHOLD and left_distance < OBSTACLE_THRESHOLD:
            print("Warning: Obstacles detected on both sides! Turning around...")
        elif right_distance < OBSTACLE_THRESHOLD:
            print("Warning: Obstacle detected on the right! Turning left...")
        elif left_distance < OBSTACLE_THRESHOLD:
            print("Warning: Obstacle detected on the left! Turning right...")
        else:
            print("No obstacles detected. Moving forward...")

        # Obstacle avoidance logic
        if right_distance < OBSTACLE_THRESHOLD and left_distance < OBSTACLE_THRESHOLD:
            # Obstacles on both sides → Stop and turn around
            self.left_speed = -MAX_SPEED / 2
            self.right_speed = MAX_SPEED / 2
        elif right_distance < OBSTACLE_THRESHOLD:
            # Obstacle on the right → Turn left
            self.left_speed = MAX_SPEED / 2
            self.right_speed = -MAX_SPEED / 2
        elif left_distance < OBSTACLE_THRESHOLD:
            # Obstacle on the left → Turn right
            self.left_speed = -MAX_SPEED / 2
            self.right_speed = MAX_SPEED / 2
        else:
            # No obstacles → Move forward
            self.left_speed = MAX_SPEED
            self.right_speed = MAX_SPEED

        # Set motor velocities
        self.wheels[1].setVelocity(self.left_speed)   # Left wheel 1
        self.wheels[2].setVelocity(self.right_speed)  # Right wheel 1
        self.wheels[3].setVelocity(self.left_speed)   # Left wheel 2
        self.wheels[4].setVelocity(self.right_speed)  # Right wheel 2

def main():
    robot = ElderlyRobot()

    # Main loop: Runs until simulation is terminated
    while robot.step(TIME_STEP) != -1:
        robot.move()

    # Cleanup
    robot.cleanup()

if __name__ == "__main__":
    main()
