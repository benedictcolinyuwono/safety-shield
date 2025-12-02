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

# NAVIGATION CONSTANTS
BASE_SPEED = 1.5
WHEEL_RADIUS = 0.125
WAYPOINT_THRESHOLD = 0.5
SLOWDOWN_DISTANCE = 3.0
MAX_LINEAR_ACCEL = 2.0
MAX_ANGULAR_ACCEL = 3.0
MAX_ANGULAR_VELOCITY = 2.0

# SAFETY CONSTANTS
CRITICAL_DISTANCE = 1.0
WARNING_DISTANCE = 2.0

# DWA PARAMETERS
DWA_DT = 0.3
DWA_PREDICT_TIME = 1.0
DWA_V_SAMPLES = 10
DWA_W_SAMPLES = 11
DWA_ALPHA = 1.5
DWA_BETA = 1.8
DWA_GAMMA = 1.0

# WAYPOINT GOALS
waypoint_goals = {
    'AGV_1': (65.0, 50.0),      # Open area top right
    'AGV_2': (65.0, 25.0),      # Open area middle right
    'AGV_3': (65.0, 5.0),       # Open area between sections
    'AGV_4': (-95.0, 50.0),     # Open area top left
    'AGV_5': (-95.0, 25.0),     # Open area middle left
    'AGV_6': (15.0, -10.0),     # Center aisle
    'AGV_7': (25.0, -65.0),     # Bottom section open area
    'AGV_8': (50.0, -65.0),     # Bottom section middle
    'AGV_9': (75.0, -65.0),     # Bottom section right
    'AGV_10': (-30.0, 50.0),    # Top section middle aisle
    'AGV_11': (-60.0, 50.0),    # Top section left aisle
    'AGV_12': (-30.0, 5.0),     # Middle section aisle
    'AGV_13': (-60.0, -30.0),   # Middle section left
    'AGV_14': (30.0, 30.0),     # Top section right aisle
    'AGV_15': (-80.0, -65.0),   # Bottom section far left
}

target_waypoint_x, target_waypoint_y = waypoint_goals[robot_name]
print(f"{robot_name} starting navigation to waypoint ({target_waypoint_x}, {target_waypoint_y})")

# OBSTACLE DEFINITIONS
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
    ("FG_PalletizingConveyors_1A", 30.1, 2.17),
    ("FG_PalletizingConveyors_1B", 2.17, 2.24),
    ("FG_PalletizingConveyors_2A", 32.19, 2.17),
    ("FG_PalletizingConveyors_2B", 2.17, 2.42),
    ("FG_PalletizingConveyors_3", 32.1, 2.17),
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

# DWA: TRAJECTORY PREDICTION
def predict_trajectory(x, y, theta, v, w, dt, steps):
    trajectory = [(x, y, theta)]
    for _ in range(steps):
        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt
        theta += w * dt
        trajectory.append((x, y, theta))
    return trajectory

# DWA: COLLISION CHECKING
def check_trajectory_collision(trajectory, obstacles, safety_radius=1.0):
    min_clearance = math.inf
    
    for x, y, _ in trajectory:
        for obs in obstacles:
            dx = obs['x'] - x
            dy = obs['y'] - y
            dist = math.sqrt(dx * dx + dy * dy) - safety_radius
            min_clearance = min(min_clearance, dist)
            
            if dist < 0:
                return -1.0
    
    return min_clearance

# DWA: HEADING OBJECTIVE
def heading_objective(trajectory, goal_x, goal_y):
    final_x, final_y, final_theta = trajectory[-1]
    
    dx = goal_x - final_x
    dy = goal_y - final_y
    angle_to_goal = math.atan2(dy, dx)
    
    angle_diff = abs(angle_to_goal - final_theta)
    while angle_diff > math.pi:
        angle_diff -= 2.0 * math.pi
    angle_diff = abs(angle_diff)
    
    return (math.pi - angle_diff) / math.pi

# DWA: CLEARANCE OBJECTIVE
def clearance_objective(clearance, max_clearance=10.0):
    if clearance < 0:
        return 0.0
    return min(clearance / max_clearance, 1.0)

# DWA: VELOCITY OBJECTIVE
def velocity_objective(v, max_v):
    return v / max_v

# DWA: TRAJECTORY EVALUATION
def evaluate_trajectory(trajectory, goal_x, goal_y, obstacles, v, max_v, alpha, beta, gamma):
    clearance = check_trajectory_collision(trajectory, obstacles)
    
    if clearance < 0:
        return -math.inf
    
    h_obj = heading_objective(trajectory, goal_x, goal_y)
    c_obj = clearance_objective(clearance)
    v_obj = velocity_objective(v, max_v)
    
    return alpha * h_obj + beta * c_obj + gamma * v_obj

# DWA: MAIN ALGORITHM
def dynamic_window_approach(current_x, current_y, current_theta, current_v, current_w,
                            goal_x, goal_y, obstacles, dt, predict_time,
                            v_samples, w_samples, max_v, max_w, max_accel_v, max_accel_w,
                            alpha, beta, gamma):
    predict_steps = int(predict_time / dt)
    
    v_min = max(0, current_v - max_accel_v * dt)
    v_max = min(max_v, current_v + max_accel_v * dt)
    
    w_min = max(-max_w, current_w - max_accel_w * dt)
    w_max = min(max_w, current_w + max_accel_w * dt)
    
    best_score = -math.inf
    best_v = 0.0
    best_w = 0.0
    
    v_step = (v_max - v_min) / max(v_samples - 1, 1) if v_samples > 1 else 0
    w_step = (w_max - w_min) / max(w_samples - 1, 1) if w_samples > 1 else 0
    
    for i in range(v_samples):
        v = v_min + i * v_step
        for j in range(w_samples):
            w = w_min + j * w_step
            
            trajectory = predict_trajectory(current_x, current_y, current_theta,
                                          v, w, dt, predict_steps)
            
            score = evaluate_trajectory(trajectory, goal_x, goal_y, obstacles,
                                       v, max_v, alpha, beta, gamma)
            
            if score > best_score:
                best_score = score
                best_v = v
                best_w = w
    
    return best_v, best_w, best_score

# STATE INITIALIZATION
rack_nodes_cache = {}
racks_initialized = False
previous_x = None
previous_y = None
previous_time = 0.0
current_v = 0.0
current_w = 0.0
step_count = 0
waypoint_reached = False

# MAIN CONTROL LOOP
while robot.step(timestep) != -1:
    step_count += 1
    
    # RACK NODE INITIALIZATION
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
        current_v = math.sqrt(my_vx * my_vx + my_vy * my_vy)
    
    previous_x = current_x
    previous_y = current_y
    previous_time = current_time
    
    # BUILD OBSTACLE LIST
    obstacles = []
    
    for i in range(1, 16):
        agv_name = f"AGV_{i}"
        if agv_name == robot_name:
            continue
        
        agv_node = robot.getFromDef(agv_name)
        if agv_node is not None:
            position = agv_node.getPosition()
            obstacles.append({'x': position[0], 'y': position[1]})
    
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
        
        obstacles.append({'x': closest_x, 'y': closest_y})
    
    # WAYPOINT CHECK
    delta_x = target_waypoint_x - current_x
    delta_y = target_waypoint_y - current_y
    distance_to_waypoint = math.sqrt(delta_x * delta_x + delta_y * delta_y)
    
    if distance_to_waypoint < WAYPOINT_THRESHOLD:
        if not waypoint_reached:
            print(f"{robot_name} ✓ REACHED WAYPOINT!")
            waypoint_reached = True
        front_right_motor.setVelocity(0.0)
        front_left_motor.setVelocity(0.0)
        back_right_motor.setVelocity(0.0)
        back_left_motor.setVelocity(0.0)
        continue
    
    # ADAPTIVE SPEED SCALING
    if distance_to_waypoint < SLOWDOWN_DISTANCE:
        speed_scale = distance_to_waypoint / SLOWDOWN_DISTANCE
        adaptive_max_speed = BASE_SPEED * max(speed_scale, 0.5)
    else:
        adaptive_max_speed = BASE_SPEED
    
    # DYNAMIC WINDOW APPROACH
    dwa_v, dwa_w, dwa_score = dynamic_window_approach(
        current_x, current_y, current_heading, current_v, current_w,
        target_waypoint_x, target_waypoint_y, obstacles,
        DWA_DT, DWA_PREDICT_TIME,
        DWA_V_SAMPLES, DWA_W_SAMPLES,
        adaptive_max_speed, MAX_ANGULAR_VELOCITY,
        MAX_LINEAR_ACCEL, MAX_ANGULAR_ACCEL,
        DWA_ALPHA, DWA_BETA, DWA_GAMMA
    )
    
    # SAFETY SHIELD OVERRIDE
    min_distance = math.inf
    
    for obs in obstacles:
        dx = obs['x'] - current_x
        dy = obs['y'] - current_y
        dist = math.sqrt(dx * dx + dy * dy)
        if dist < min_distance:
            min_distance = dist
    
    if min_distance < CRITICAL_DISTANCE:
        linear_velocity = 0.0
        angular_velocity = 0.0
    elif min_distance < WARNING_DISTANCE:
        safety_factor = (min_distance - CRITICAL_DISTANCE) / (WARNING_DISTANCE - CRITICAL_DISTANCE)
        linear_velocity = dwa_v * safety_factor
        angular_velocity = dwa_w * safety_factor
    else:
        linear_velocity = dwa_v
        angular_velocity = dwa_w
    
    current_w = angular_velocity
    
    # MOTOR COMMANDS
    wheel_velocity = linear_velocity / WHEEL_RADIUS
    left_velocity = wheel_velocity - angular_velocity
    right_velocity = wheel_velocity + angular_velocity
    
    front_left_motor.setVelocity(left_velocity)
    back_left_motor.setVelocity(left_velocity)
    front_right_motor.setVelocity(right_velocity)
    back_right_motor.setVelocity(right_velocity)