# Import libraries
from controller import Robot
import math

# Initialize robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())
robot_name = robot.getName()

# Get motor devices
front_right_motor = robot.getDevice('front_right_wheel_joint')
front_left_motor = robot.getDevice('front_left_wheel_joint')
back_right_motor = robot.getDevice('back_right_wheel_joint')
back_left_motor = robot.getDevice('back_left_wheel_joint')

# Set motors to velocity control mode
front_right_motor.setPosition(float('inf'))
front_left_motor.setPosition(float('inf'))
back_right_motor.setPosition(float('inf'))
back_left_motor.setPosition(float('inf'))

# Get GPS sensor
gps = robot.getDevice('gps')
gps.enable(timestep)

# Get compass sensor
compass = robot.getDevice('compass')
compass.enable(timestep)

# Constants
BASE_SPEED = 15.0
WAYPOINT_THRESHOLD = 0.5  # meters - how close to consider "reached"

# Define a simple waypoint to test
target_waypoint_x = 10.0
target_waypoint_y = 40.0

print(f"{robot_name} starting navigation to waypoint ({target_waypoint_x}, {target_waypoint_y})")

# Main control loop
while robot.step(timestep) != -1:
    # Read GPS position
    gps_values = gps.getValues()
    current_x = gps_values[0]
    current_y = gps_values[1]
    
    # Read compass and calculate heading
    compass_values = compass.getValues()
    compass_x = compass_values[0]
    compass_y = compass_values[1]
    current_heading = math.atan2(compass_y, compass_x)
    
    # Calculate vector to waypoint
    delta_x = target_waypoint_x - current_x
    delta_y = target_waypoint_y - current_y
    distance_to_waypoint = math.sqrt(delta_x * delta_x + delta_y * delta_y)
    
    # Check if waypoint reached
    if distance_to_waypoint < WAYPOINT_THRESHOLD:
        print(f"{robot_name} reached waypoint!")
        # Stop all motors
        front_right_motor.setVelocity(0.0)
        front_left_motor.setVelocity(0.0)
        back_right_motor.setVelocity(0.0)
        back_left_motor.setVelocity(0.0)
        continue
    
    # Calculate desired heading to waypoint
    desired_heading = math.atan2(delta_y, delta_x)
    
    # Calculate heading error (shortest angular path)
    heading_error = desired_heading - current_heading
    # Normalize to [-pi, pi]
    if heading_error > math.pi:
        heading_error = heading_error - 2.0 * math.pi
    if heading_error < -math.pi:
        heading_error = heading_error + 2.0 * math.pi
    
    # Simple proportional control for steering
    steering_gain = 5.0
    angular_velocity_command = steering_gain * heading_error
    
    # Limit angular velocity
    max_angular_velocity = 2.0
    if angular_velocity_command > max_angular_velocity:
        angular_velocity_command = max_angular_velocity
    if angular_velocity_command < -max_angular_velocity:
        angular_velocity_command = -max_angular_velocity
    
    # Convert to differential drive commands
    # For mecanum: left side positive forward, right side positive forward
    # Turning: left side slower/backward, right side faster/forward
    linear_velocity_command = BASE_SPEED * 0.5  # Slower while navigating
    
    left_velocity = linear_velocity_command - angular_velocity_command
    right_velocity = linear_velocity_command + angular_velocity_command
    
    # Apply velocities to motors
    front_left_motor.setVelocity(left_velocity)
    back_left_motor.setVelocity(left_velocity)
    front_right_motor.setVelocity(right_velocity)
    back_right_motor.setVelocity(right_velocity)