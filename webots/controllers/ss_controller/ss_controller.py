from controller import Robot, Supervisor
import math

# Initialize robot as Supervisor (can see other AGVs)
robot = Supervisor()
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

waypoint_goals = {
    'AGV_1': (10.0, 42.0),
    'AGV_2': (10.0, 36.0),
    'AGV_3': (10.0, -0.31),
    'AGV_4': (10.0, -40.0),
    'AGV_5': (10.0, -46.0),
    'AGV_6': (88.0, -10.0),
    'AGV_7': (13.0, -70.0),
    'AGV_8': (32.0, -70.0),
    'AGV_9': (45.0, -70.0),
    'AGV_10': (65.0, -70.0),
    'AGV_11': (83.0, -70.0),
    'AGV_12': (-80.0, 42.0),
    'AGV_13': (-85.0, -46.0),
    'AGV_14': (70.0, 36.0),
    'AGV_15': (-100.0, -57.0),
}

# Get my waypoint
target_waypoint_x, target_waypoint_y = waypoint_goals[robot_name]

print(f"{robot_name} starting at ({gps.getValues()[0]:.2f}, {gps.getValues()[1]:.2f})")
print(f"{robot_name} navigating to waypoint ({target_waypoint_x}, {target_waypoint_y})")

# Main control loop
while robot.step(timestep) != -1:
    # ========== SENSE: Read my own state ==========
    # Read GPS position
    gps_values = gps.getValues()
    current_x = gps_values[0]
    current_y = gps_values[1]
    
    # Read compass and calculate heading
    compass_values = compass.getValues()
    compass_x = compass_values[0]
    compass_y = compass_values[1]
    current_heading = math.atan2(compass_y, compass_x)
    
    # ========== SENSE: Read other AGVs' positions ==========
    other_agvs = []
    for i in range(1, 16):
        agv_name = f"AGV_{i}"
        if agv_name == robot_name:
            continue  # Skip myself
        
        # Get node handle for this AGV
        agv_node = robot.getFromDef(agv_name)
        if agv_node is not None:
            position = agv_node.getPosition()
            other_agvs.append({
                'name': agv_name,
                'x': position[0],
                'y': position[1],
                'z': position[2]
            })
    
    # ========== NAVIGATE: Calculate vector to waypoint ==========
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
    linear_velocity_command = BASE_SPEED * 0.5  # Slower while navigating
    
    # TODO: Add collision detection here (next step)
    # TODO: Add safety shield integration here (next step)
    # For now, just navigate without collision avoidance
    
    left_velocity = linear_velocity_command - angular_velocity_command
    right_velocity = linear_velocity_command + angular_velocity_command
    
    # Apply velocities to motors
    front_left_motor.setVelocity(left_velocity)
    back_left_motor.setVelocity(left_velocity)
    front_right_motor.setVelocity(right_velocity)
    back_right_motor.setVelocity(right_velocity)