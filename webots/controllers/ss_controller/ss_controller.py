from controller import Robot, Supervisor
import math
import sys
import os

# Add shield module path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

# Import shield modules
from shield.risk import assess_collision_risk_2d
from shield.limiter import limit_velocity, limit_angular_velocity
from shield.supervisor import supervise_commands, PASS, GUARDED, EMERGENCY

# Import geometry utilities
from geometry_utils import closest_point_on_rectangle

# Initialize robot
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

# Get sensors
gps = robot.getDevice('gps')
gps.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(timestep)

# Constants
BASE_SPEED = 1.5  # m/s (linear velocity)
WHEEL_RADIUS = 0.125  # meters (Summit XL Steel wheel radius)
WAYPOINT_THRESHOLD = 0.5
SAFETY_RADIUS = 1.5
MIN_SAFE_DISTANCE = 2.5  # meters - emergency stop distance (position-based)
PREDICTION_TIME = 0.5  # seconds - look ahead time
MAX_ANGULAR_VELOCITY = 2.0
STEERING_GAIN = 3.0

# Waypoint goals
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

target_waypoint_x, target_waypoint_y = waypoint_goals[robot_name]

print(f"{robot_name} starting navigation to waypoint ({target_waypoint_x}, {target_waypoint_y})")

# Rack definitions: name, width, length
rack_definitions = [
    # FG_Top RackA (16 racks)
    ("FG_Top_RackRowA_1", 4.35, 37.0),
    ("FG_Top_RackRowA_2", 8.7, 37.0),
    ("FG_Top_RackRowA_3", 8.7, 37.0),
    ("FG_Top_RackRowA_4", 8.7, 37.0),
    ("FG_Top_RackRowA_5", 8.7, 37.0),
    ("FG_Top_RackRowA_6", 8.7, 37.0),
    ("FG_Top_RackRowA_7", 8.7, 37.0),
    ("FG_Top_RackRowA_8", 8.7, 37.0),
    ("FG_Top_RackRowA_9", 8.7, 37.0),
    ("FG_Top_RackRowA_10", 8.7, 37.0),
    ("FG_Top_RackRowA_11", 8.7, 37.0),
    ("FG_Top_RackRowA_12", 8.7, 37.0),
    ("FG_Top_RackRowA_13", 8.7, 37.0),
    ("FG_Top_RackRowA_14", 8.7, 37.0),
    ("FG_Top_RackRowA_15", 8.7, 37.0),
    ("FG_Top_RackRowA_16", 8.7, 37.0),
    # FG_Top RackB (13 racks)
    ("FG_Top_RackRowB_1", 4.35, 31.7),
    ("FG_Top_RackRowB_2", 8.7, 31.7),
    ("FG_Top_RackRowB_3", 8.7, 31.7),
    ("FG_Top_RackRowB_4", 8.7, 31.7),
    ("FG_Top_RackRowB_5", 8.7, 31.7),
    ("FG_Top_RackRowB_6", 8.7, 31.7),
    ("FG_Top_RackRowB_7", 8.7, 31.7),
    ("FG_Top_RackRowB_8", 8.7, 31.7),
    ("FG_Top_RackRowB_9", 8.7, 31.7),
    ("FG_Top_RackRowB_10", 8.7, 31.7),
    ("FG_Top_RackRowB_11", 8.7, 31.7),
    ("FG_Top_RackRowB_12", 8.7, 31.7),
    ("FG_Top_RackRowB_13", 8.7, 31.7),
    # FG_Top RackR (2 racks)
    ("FG_Top_RackRowR_1", 5.1, 15.3),
    ("FG_Top_RackRowR_2", 5.1, 15.3),
    # FG_Middle (14 racks)
    ("FG_Middle_RackRow_1", 8.7, 37.0),
    ("FG_Middle_RackRow_2", 8.7, 37.0),
    ("FG_Middle_RackRow_3", 8.7, 37.0),
    ("FG_Middle_RackRow_4", 8.7, 37.0),
    ("FG_Middle_RackRow_5", 8.7, 37.0),
    ("FG_Middle_RackRow_6", 8.7, 37.0),
    ("FG_Middle_RackRow_7", 8.7, 37.0),
    ("FG_Middle_RackRow_8", 8.7, 37.0),
    ("FG_Middle_RackRow_9", 8.7, 37.0),
    ("FG_Middle_RackRow_10", 8.7, 37.0),
    ("FG_Middle_RackRow_11", 8.7, 37.0),
    ("FG_Middle_RackRow_12", 8.7, 37.0),
    ("FG_Middle_RackRow_13", 8.7, 37.0),
    ("FG_Middle_RackRow_14", 8.7, 37.0),
    # FG_Middle RackR (2 racks)
    ("FG_Middle_RackRowR_1", 5.1, 15.3),
    ("FG_Middle_RackRowR_2", 5.1, 15.3),
    # FG_Bottom (8 racks)
    ("FG_Bottom_RackRow_1", 8.7, 31.7),
    ("FG_Bottom_RackRow_2", 8.7, 31.7),
    ("FG_Bottom_RackRow_3", 8.7, 31.7),
    ("FG_Bottom_RackRow_4", 8.7, 31.7),
    ("FG_Bottom_RackRow_5", 8.7, 31.7),
    ("FG_Bottom_RackRow_6", 8.7, 31.7),
    ("FG_Bottom_RackRow_7", 8.7, 31.7),
    ("FG_Bottom_RackRow_8", 8.7, 31.7),
    # FG_Bottom RackR (2 racks)
    ("FG_Bottom_RackRowR_1", 5.1, 15.3),
    ("FG_Bottom_RackRowR_2", 5.1, 15.3),
]

# Initialize velocity tracking
previous_x = None
previous_y = None
previous_time = 0.0

# Main control loop
while robot.step(timestep) != -1:
    # Read my own state
    gps_values = gps.getValues()
    current_x = gps_values[0]
    current_y = gps_values[1]
    
    compass_values = compass.getValues()
    compass_x = compass_values[0]
    compass_y = compass_values[1]
    current_heading = math.atan2(compass_y, compass_x)
    
    # Calculate my velocity
    current_time = robot.getTime()
    delta_time = current_time - previous_time
    
    if previous_x is not None and delta_time > 0.0:
        my_vx = (current_x - previous_x) / delta_time
        my_vy = (current_y - previous_y) / delta_time
    else:
        my_vx = 0.0
        my_vy = 0.0
    
    my_speed = math.sqrt(my_vx * my_vx + my_vy * my_vy)
    
    # Calculate predicted position (look-ahead)
    predicted_x = current_x + my_vx * PREDICTION_TIME
    predicted_y = current_y + my_vy * PREDICTION_TIME
    
    previous_x = current_x
    previous_y = current_y
    previous_time = current_time
    
    # Read other AGVs' positions
    other_agvs = []
    for i in range(1, 16):
        agv_name = f"AGV_{i}"
        if agv_name == robot_name:
            continue
        
        agv_node = robot.getFromDef(agv_name)
        if agv_node is not None:
            position = agv_node.getPosition()
            other_agvs.append({
                'name': agv_name,
                'x': position[0],
                'y': position[1],
                'z': position[2]
            })
    
    # Assess collision risk to all other AGVs
    worst_risk = {
        "ttc": math.inf,
        "headway": math.inf,
        "gap_m": math.inf,
        "margin_low": False,
        "violation_predicted": False,
    }
    closest_threat = None
    min_distance = math.inf
    
    # Check current position collision risk
    for other in other_agvs:
        risk = assess_collision_risk_2d(
            current_x, current_y, my_vx, my_vy,
            other['x'], other['y'],
            other_vx=0.0, other_vy=0.0,
            safety_radius=SAFETY_RADIUS
        )
        
        if risk["gap_m"] < min_distance:
            min_distance = risk["gap_m"]
        
        if risk["ttc"] < worst_risk["ttc"]:
            worst_risk = risk
            closest_threat = other['name']
    
    # Check predicted position collision risk (look-ahead)
    for other in other_agvs:
        risk_predicted = assess_collision_risk_2d(
            predicted_x, predicted_y, my_vx, my_vy,
            other['x'], other['y'],
            other_vx=0.0, other_vy=0.0,
            safety_radius=SAFETY_RADIUS
        )
        
        if risk_predicted["ttc"] < worst_risk["ttc"]:
            worst_risk = risk_predicted
            closest_threat = other['name'] + " (predicted)"
    
    # Assess collision risk to all racks
    for rack_name, rack_width, rack_length in rack_definitions:
        rack_node = robot.getFromDef(rack_name)
        if rack_node is None:
            continue
        
        rack_position = rack_node.getPosition()
        rack_x = rack_position[0]
        rack_y = rack_position[1]
        
        # Get rack rotation (if any)
        rack_rotation_field = rack_node.getField('rotation')
        if rack_rotation_field is not None:
            rack_rotation_values = rack_rotation_field.getSFRotation()
            rack_rotation = rack_rotation_values[3]
        else:
            rack_rotation = 0.0
        
        # Find closest point on rack rectangle
        closest_x, closest_y = closest_point_on_rectangle(
            current_x, current_y,
            rack_x, rack_y,
            rack_width, rack_length,
            rack_rotation
        )
        
        # Calculate collision risk to this closest point
        risk = assess_collision_risk_2d(
            current_x, current_y, my_vx, my_vy,
            closest_x, closest_y,
            other_vx=0.0, other_vy=0.0,
            safety_radius=SAFETY_RADIUS
        )
        
        if risk["gap_m"] < min_distance:
            min_distance = risk["gap_m"]
        
        if risk["ttc"] < worst_risk["ttc"]:
            worst_risk = risk
            closest_threat = rack_name
    
    # POSITION-BASED SAFETY OVERRIDE
    # If too close to anything, force emergency stop regardless of velocity
    if min_distance < MIN_SAFE_DISTANCE:
        worst_risk["violation_predicted"] = True
        worst_risk["margin_low"] = True
        if min_distance < MIN_SAFE_DISTANCE * 0.7:  # Very close (< 1.75m)
            print(f"{robot_name} EMERGENCY DISTANCE: {min_distance:.2f}m to {closest_threat}")
    
    # Calculate vector to waypoint
    delta_x = target_waypoint_x - current_x
    delta_y = target_waypoint_y - current_y
    distance_to_waypoint = math.sqrt(delta_x * delta_x + delta_y * delta_y)
    
    # Check if waypoint reached
    if distance_to_waypoint < WAYPOINT_THRESHOLD:
        print(f"{robot_name} reached waypoint!")
        front_right_motor.setVelocity(0.0)
        front_left_motor.setVelocity(0.0)
        back_right_motor.setVelocity(0.0)
        back_left_motor.setVelocity(0.0)
        continue
    
    # Calculate desired heading to waypoint
    desired_heading = math.atan2(delta_y, delta_x)
    
    # Calculate heading error
    heading_error = desired_heading - current_heading
    if heading_error > math.pi:
        heading_error = heading_error - 2.0 * math.pi
    if heading_error < -math.pi:
        heading_error = heading_error + 2.0 * math.pi
    
    # Proportional control for steering
    angular_velocity_command = STEERING_GAIN * heading_error
    angular_velocity_command = limit_angular_velocity(angular_velocity_command, MAX_ANGULAR_VELOCITY)
    
    # Base linear velocity command (no warmup - full speed)
    linear_velocity_command = BASE_SPEED
    
    # Apply safety shield (works in m/s)
    state = {"v_cap": BASE_SPEED}
    commanded_velocities = (linear_velocity_command, angular_velocity_command)
    mode, (v_safe, w_safe) = supervise_commands(state, commanded_velocities, worst_risk)
    
    # Debug output (only print when intervening)
    if mode == EMERGENCY:
        print(f"{robot_name} [{mode}] STOPPING for {closest_threat}: TTC={worst_risk['ttc']:.2f}s, gap={worst_risk['gap_m']:.2f}m")
    elif mode == GUARDED:
        # Print every 20 steps to reduce spam
        if int(current_time * 100) % 20 == 0:
            print(f"{robot_name} [{mode}] Slowing for {closest_threat}: TTC={worst_risk['ttc']:.2f}s, v={v_safe:.2f}m/s")
    
    # Convert linear velocity from m/s to wheel rad/s
    wheel_v_safe = v_safe / WHEEL_RADIUS
    
    # Apply differential drive: left/right wheel velocities
    left_velocity = wheel_v_safe - w_safe
    right_velocity = wheel_v_safe + w_safe
    
    front_left_motor.setVelocity(left_velocity)
    back_left_motor.setVelocity(left_velocity)
    front_right_motor.setVelocity(right_velocity)
    back_right_motor.setVelocity(right_velocity)