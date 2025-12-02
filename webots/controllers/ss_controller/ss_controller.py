from controller import Robot, Supervisor
import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

from shield.risk import assess_collision_risk_2d
from shield.limiter import limit_velocity, limit_angular_velocity
from shield.supervisor import supervise_commands, PASS, GUARDED, EMERGENCY
from geometry_utils import closest_point_on_rectangle

# ROBOT INITIALIZATION
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
robot_name = robot.getName()

# MOTOR SETUP
front_right_motor = robot.getDevice('front_right_wheel_joint')
front_left_motor = robot.getDevice('front_left_wheel_joint')
back_right_motor = robot.getDevice('back_right_wheel_joint')
back_left_motor = robot.getDevice('back_left_wheel_joint')

front_right_motor.setPosition(float('inf'))
front_left_motor.setPosition(float('inf'))
back_right_motor.setPosition(float('inf'))
back_left_motor.setPosition(float('inf'))

# SENSOR SETUP
gps = robot.getDevice('gps')
gps.enable(timestep)
compass = robot.getDevice('compass')
compass.enable(timestep)

# CONSTANTS
BASE_SPEED = 1.0
WHEEL_RADIUS = 0.125
WAYPOINT_THRESHOLD = 0.5
CRITICAL_DISTANCE = 2.0
WARNING_DISTANCE = 4.0
MAX_ANGULAR_VELOCITY = 2.0
STEERING_GAIN = 3.0

# WAYPOINT GOALS
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

# RACK DEFINITIONS
rack_definitions = [
    ("FG_Top_RackRowA_1", 4.35, 37.0), ("FG_Top_RackRowA_2", 8.7, 37.0),
    ("FG_Top_RackRowA_3", 8.7, 37.0), ("FG_Top_RackRowA_4", 8.7, 37.0),
    ("FG_Top_RackRowA_5", 8.7, 37.0), ("FG_Top_RackRowA_6", 8.7, 37.0),
    ("FG_Top_RackRowA_7", 8.7, 37.0), ("FG_Top_RackRowA_8", 8.7, 37.0),
    ("FG_Top_RackRowA_9", 8.7, 37.0), ("FG_Top_RackRowA_10", 8.7, 37.0),
    ("FG_Top_RackRowA_11", 8.7, 37.0), ("FG_Top_RackRowA_12", 8.7, 37.0),
    ("FG_Top_RackRowA_13", 8.7, 37.0), ("FG_Top_RackRowA_14", 8.7, 37.0),
    ("FG_Top_RackRowA_15", 8.7, 37.0), ("FG_Top_RackRowA_16", 8.7, 37.0),
    ("FG_Top_RackRowB_1", 4.35, 31.7), ("FG_Top_RackRowB_2", 8.7, 31.7),
    ("FG_Top_RackRowB_3", 8.7, 31.7), ("FG_Top_RackRowB_4", 8.7, 31.7),
    ("FG_Top_RackRowB_5", 8.7, 31.7), ("FG_Top_RackRowB_6", 8.7, 31.7),
    ("FG_Top_RackRowB_7", 8.7, 31.7), ("FG_Top_RackRowB_8", 8.7, 31.7),
    ("FG_Top_RackRowB_9", 8.7, 31.7), ("FG_Top_RackRowB_10", 8.7, 31.7),
    ("FG_Top_RackRowB_11", 8.7, 31.7), ("FG_Top_RackRowB_12", 8.7, 31.7),
    ("FG_Top_RackRowB_13", 8.7, 31.7), ("FG_Top_RackRowR_1", 5.1, 15.3),
    ("FG_Top_RackRowR_2", 5.1, 15.3), ("FG_Middle_RackRow_1", 8.7, 37.0),
    ("FG_Middle_RackRow_2", 8.7, 37.0), ("FG_Middle_RackRow_3", 8.7, 37.0),
    ("FG_Middle_RackRow_4", 8.7, 37.0), ("FG_Middle_RackRow_5", 8.7, 37.0),
    ("FG_Middle_RackRow_6", 8.7, 37.0), ("FG_Middle_RackRow_7", 8.7, 37.0),
    ("FG_Middle_RackRow_8", 8.7, 37.0), ("FG_Middle_RackRow_9", 8.7, 37.0),
    ("FG_Middle_RackRow_10", 8.7, 37.0), ("FG_Middle_RackRow_11", 8.7, 37.0),
    ("FG_Middle_RackRow_12", 8.7, 37.0), ("FG_Middle_RackRow_13", 8.7, 37.0),
    ("FG_Middle_RackRow_14", 8.7, 37.0), ("FG_Middle_RackRowR_1", 5.1, 15.3),
    ("FG_Middle_RackRowR_2", 5.1, 15.3), ("FG_Bottom_RackRow_1", 8.7, 31.7),
    ("FG_Bottom_RackRow_2", 8.7, 31.7), ("FG_Bottom_RackRow_3", 8.7, 31.7),
    ("FG_Bottom_RackRow_4", 8.7, 31.7), ("FG_Bottom_RackRow_5", 8.7, 31.7),
    ("FG_Bottom_RackRow_6", 8.7, 31.7), ("FG_Bottom_RackRow_7", 8.7, 31.7),
    ("FG_Bottom_RackRow_8", 8.7, 31.7), ("FG_Bottom_RackRowR_1", 5.1, 15.3),
    ("FG_Bottom_RackRowR_2", 5.1, 15.3),
]

# NODE SEARCH FUNCTION
def find_node_by_name(supervisor, name):
    root = supervisor.getRoot()
    children_field = root.getField('children')
    
    def search_node(node):
        name_field = node.getField('name')
        if name_field is not None:
            node_name = name_field.getSFString()
            if node_name == name:
                return node
        
        children_field = node.getField('children')
        if children_field is not None:
            for i in range(children_field.getCount()):
                child = children_field.getMFNode(i)
                if child is not None:
                    result = search_node(child)
                    if result is not None:
                        return result
        return None
    
    for i in range(children_field.getCount()):
        child = children_field.getMFNode(i)
        if child is not None:
            result = search_node(child)
            if result is not None:
                return result
    return None

# STATE INITIALIZATION
rack_nodes_cache = {}
racks_initialized = False
previous_x = None
previous_y = None
previous_time = 0.0
step_count = 0

# MAIN CONTROL LOOP
while robot.step(timestep) != -1:
    step_count += 1
    
    # RACK NODE INITIALIZATION (FIRST STEP ONLY)
    if not racks_initialized:
        print(f"{robot_name} initializing rack detection...")
        for rack_name, rack_width, rack_length in rack_definitions:
            rack_node = find_node_by_name(robot, rack_name)
            if rack_node is not None:
                rack_nodes_cache[rack_name] = {
                    'node': rack_node,
                    'width': rack_width,
                    'length': rack_length
                }
        print(f"{robot_name} found {len(rack_nodes_cache)} racks")
        racks_initialized = True
    
    # READ CURRENT STATE
    gps_values = gps.getValues()
    current_x = gps_values[0]
    current_y = gps_values[1]
    
    compass_values = compass.getValues()
    current_heading = math.atan2(compass_values[1], compass_values[0])
    
    # CALCULATE VELOCITY
    current_time = robot.getTime()
    delta_time = current_time - previous_time
    
    if previous_x is not None and delta_time > 0.0:
        my_vx = (current_x - previous_x) / delta_time
        my_vy = (current_y - previous_y) / delta_time
    else:
        my_vx = 0.0
        my_vy = 0.0
    
    previous_x = current_x
    previous_y = current_y
    previous_time = current_time
    
    # FIND MINIMUM DISTANCE TO OBSTACLES
    min_distance = math.inf
    closest_obstacle = None
    
    for i in range(1, 16):
        agv_name = f"AGV_{i}"
        if agv_name == robot_name:
            continue
        
        agv_node = robot.getFromDef(agv_name)
        if agv_node is not None:
            position = agv_node.getPosition()
            dx = position[0] - current_x
            dy = position[1] - current_y
            distance = math.sqrt(dx * dx + dy * dy)
            
            if distance < min_distance:
                min_distance = distance
                closest_obstacle = agv_name
    
    for rack_name, rack_data in rack_nodes_cache.items():
        rack_node = rack_data['node']
        rack_position = rack_node.getPosition()
        
        rack_rotation_field = rack_node.getField('rotation')
        rack_rotation = rack_rotation_field.getSFRotation()[3] if rack_rotation_field else 0.0
        
        closest_x, closest_y = closest_point_on_rectangle(
            current_x, current_y,
            rack_position[0], rack_position[1],
            rack_data['width'], rack_data['length'],
            rack_rotation
        )
        
        dx = closest_x - current_x
        dy = closest_y - current_y
        distance = math.sqrt(dx * dx + dy * dy)
        
        if distance < min_distance:
            min_distance = distance
            closest_obstacle = rack_name
    
    if step_count % 30 == 0:
        print(f"{robot_name} closest: {closest_obstacle} at {min_distance:.2f}m")
    
    # WAYPOINT NAVIGATION
    delta_x = target_waypoint_x - current_x
    delta_y = target_waypoint_y - current_y
    distance_to_waypoint = math.sqrt(delta_x * delta_x + delta_y * delta_y)
    
    if distance_to_waypoint < WAYPOINT_THRESHOLD:
        print(f"{robot_name} reached waypoint!")
        front_right_motor.setVelocity(0.0)
        front_left_motor.setVelocity(0.0)
        back_right_motor.setVelocity(0.0)
        back_left_motor.setVelocity(0.0)
        continue
    
    # STEERING CONTROL
    desired_heading = math.atan2(delta_y, delta_x)
    heading_error = desired_heading - current_heading
    
    if heading_error > math.pi:
        heading_error -= 2.0 * math.pi
    if heading_error < -math.pi:
        heading_error += 2.0 * math.pi
    
    angular_velocity_command = limit_angular_velocity(STEERING_GAIN * heading_error, MAX_ANGULAR_VELOCITY)
    
    # DISTANCE-BASED VELOCITY CONTROL
    if min_distance < CRITICAL_DISTANCE:
        linear_velocity = 0.0
        angular_velocity_command = 0.0
        print(f"{robot_name} [EMERGENCY] STOP! {min_distance:.2f}m to {closest_obstacle}")
    elif min_distance < WARNING_DISTANCE:
        safety_factor = (min_distance - CRITICAL_DISTANCE) / (WARNING_DISTANCE - CRITICAL_DISTANCE)
        linear_velocity = BASE_SPEED * safety_factor
        if step_count % 20 == 0:
            print(f"{robot_name} [SLOWING] {min_distance:.2f}m to {closest_obstacle}")
    else:
        linear_velocity = BASE_SPEED
    
    # MOTOR COMMANDS
    wheel_velocity = linear_velocity / WHEEL_RADIUS
    left_velocity = wheel_velocity - angular_velocity_command
    right_velocity = wheel_velocity + angular_velocity_command
    
    front_left_motor.setVelocity(left_velocity)
    back_left_motor.setVelocity(left_velocity)
    front_right_motor.setVelocity(right_velocity)
    back_right_motor.setVelocity(right_velocity)