from controller import Robot, Supervisor
import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

from shield.risk import assess_collision_risk_2d
from shield.limiter import limit_velocity, limit_angular_velocity
from shield.supervisor import supervise_commands, PASS, GUARDED, EMERGENCY
from geometry_utils import closest_point_on_rectangle
from waypoints import warehouse_waypoints
from path_planner import get_path_planner
from obstacles import rack_obstacles, wall_obstacles

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

# ROTATION CONSTANTS
ROTATION_THRESHOLD = 0.4   # 23 degrees
ROTATION_TOLERANCE = 0.15  # 9 degrees
ROTATION_SPEED = 1.2       # rad/s

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
    'AGV_1': ('row_a_aisle_1', 3),
    'AGV_2': ('row_a_aisle_5', 2),
    'AGV_3': ('row_b_aisle_3', 4),
    'AGV_4': ('middle_aisle_7', 5),
    'AGV_5': ('middle_aisle_2', 1),
    'AGV_6': ('bottom_aisle_4', 3),
    'AGV_7': ('conveyor_1a_pickup', 7),
    'AGV_8': ('conveyor_2a_pickup', 10),
    'AGV_9': ('conveyor_3_pickup', 8),
    'AGV_10': ('row_a_aisle_10', 6),
    'AGV_11': ('row_b_aisle_8', 5),
    'AGV_12': ('middle_aisle_12', 4),
    'AGV_13': ('bottom_aisle_2', 2),
    'AGV_14': ('row_a_aisle_15', 6),
    'AGV_15': ('bottom_aisle_6', 5),
}

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

def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def rotate_to_angle(target_angle, tolerance=0.15, timeout=15.0):
    """
    Blocking function: rotate in place until aligned with target angle
    Returns when angle difference < tolerance or timeout
    """
    start_time = robot.getTime()
    consecutive_aligned_count = 0
    required_aligned_steps = 3
    
    while robot.step(timestep) != -1:
        current_time = robot.getTime()
        
        if current_time - start_time > timeout:
            print(f"{robot_name}: Rotation timeout after {timeout}s")
            break
        
        compass_values = compass.getValues()
        current_heading = math.atan2(compass_values[1], compass_values[0])
        
        angle_diff = normalize_angle(target_angle - current_heading)
        
        if abs(angle_diff) < tolerance:
            consecutive_aligned_count += 1
            if consecutive_aligned_count >= required_aligned_steps:
                front_left_motor.setVelocity(0.0)
                back_left_motor.setVelocity(0.0)
                front_right_motor.setVelocity(0.0)
                back_right_motor.setVelocity(0.0)
                print(f"{robot_name}: Rotation complete (error: {math.degrees(angle_diff):.1f}°)")
                return True
        else:
            consecutive_aligned_count = 0
        
        base_rotation_speed = ROTATION_SPEED
        
        if abs(angle_diff) < 0.8:
            slowdown_factor = max(abs(angle_diff) / 0.8, 0.3)
            rotation_speed = base_rotation_speed * slowdown_factor
        else:
            rotation_speed = base_rotation_speed
        
        if angle_diff > 0:
            angular_velocity = rotation_speed
        else:
            angular_velocity = -rotation_speed
        
        left_velocity = -angular_velocity
        right_velocity = angular_velocity
        
        front_left_motor.setVelocity(left_velocity)
        back_left_motor.setVelocity(left_velocity)
        front_right_motor.setVelocity(right_velocity)
        back_right_motor.setVelocity(right_velocity)
    
    return False

def predict_trajectory(x, y, theta, v, w, dt, steps):
    trajectory = [(x, y, theta)]
    for _ in range(steps):
        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt
        theta += w * dt
        trajectory.append((x, y, theta))
    return trajectory

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

def clearance_objective(clearance, max_clearance=10.0):
    if clearance < 0:
        return 0.0
    return min(clearance / max_clearance, 1.0)

def velocity_objective(v, max_v):
    return v / max_v

def evaluate_trajectory(trajectory, goal_x, goal_y, obstacles, v, max_v, alpha, beta, gamma):
    clearance = check_trajectory_collision(trajectory, obstacles)
    
    if clearance < 0:
        return -math.inf
    
    h_obj = heading_objective(trajectory, goal_x, goal_y)
    c_obj = clearance_objective(clearance)
    v_obj = velocity_objective(v, max_v)
    
    return alpha * h_obj + beta * c_obj + gamma * v_obj

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
wall_static_cache = {}
racks_initialized = False
walls_initialized = False
previous_x = None
previous_y = None
previous_time = 0.0
current_v = 0.0
current_w = 0.0
step_count = 0

# PATH PLANNING INITIALIZATION
path_planner = None
waypoint_path = None
current_waypoint_index = 0
path_initialized = False

# MAIN CONTROL LOOP
while robot.step(timestep) != -1:
    step_count += 1
    
    # RACK NODE INITIALIZATION
    if not racks_initialized:
        for rack_data in rack_obstacles:
            rack_name, _, _, rack_width, rack_length = rack_data
            rack_node = find_node_by_name(robot, rack_name)
            if rack_node is not None:
                rack_nodes_cache[rack_name] = {
                    'node': rack_node,
                    'width': rack_width,
                    'length': rack_length
                }
        racks_initialized = True
    
    # WALL STATIC INITIALIZATION
    if not walls_initialized:
        for wall_data in wall_obstacles:
            wall_name, wall_x, wall_y, wall_width, wall_length = wall_data
            wall_static_cache[wall_name] = {
                'x': wall_x,
                'y': wall_y,
                'width': wall_width,
                'length': wall_length
            }
        walls_initialized = True
    
    # READ CURRENT STATE
    gps_values = gps.getValues()
    current_x = gps_values[0]
    current_y = gps_values[1]
    
    compass_values = compass.getValues()
    current_heading = math.atan2(compass_values[1], compass_values[0])
    
    # PATH PLANNING INITIALIZATION
    if not path_initialized and previous_x is not None:
        path_planner = get_path_planner()
        
        aisle_name, position_index = waypoint_goals[robot_name]
        waypoint_path = path_planner.find_path(current_x, current_y, aisle_name, position_index)
        
        if waypoint_path:
            goal_x, goal_y = waypoint_path[-1]
            print(f"{robot_name}: ({current_x:.1f}, {current_y:.1f}) → ({goal_x:.1f}, {goal_y:.1f})")
            current_waypoint_index = 0
            path_initialized = True
        else:
            print(f"{robot_name}: PATH PLANNING FAILED")
            path_initialized = True
            waypoint_path = []
    
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
    
    # CHECK IF PATH IS AVAILABLE
    if not path_initialized or not waypoint_path or current_waypoint_index >= len(waypoint_path):
        front_right_motor.setVelocity(0.0)
        front_left_motor.setVelocity(0.0)
        back_right_motor.setVelocity(0.0)
        back_left_motor.setVelocity(0.0)
        continue
    
    # GET CURRENT TARGET WAYPOINT
    target_waypoint_x, target_waypoint_y = waypoint_path[current_waypoint_index]
    
    # CALCULATE ANGLE TO WAYPOINT
    delta_x = target_waypoint_x - current_x
    delta_y = target_waypoint_y - current_y
    distance_to_waypoint = math.sqrt(delta_x * delta_x + delta_y * delta_y)
    angle_to_waypoint = math.atan2(delta_y, delta_x)
    angle_diff = normalize_angle(angle_to_waypoint - current_heading)
    
    # WAYPOINT CHECK
    if distance_to_waypoint < WAYPOINT_THRESHOLD:
        current_waypoint_index += 1
        
        if current_waypoint_index >= len(waypoint_path):
            aisle_name, position_index = waypoint_goals[robot_name]
            print(f"{robot_name}: ✓ ARRIVED at {aisle_name}[{position_index}]")
            front_right_motor.setVelocity(0.0)
            front_left_motor.setVelocity(0.0)
            back_right_motor.setVelocity(0.0)
            back_left_motor.setVelocity(0.0)
            continue
        else:
            target_waypoint_x, target_waypoint_y = waypoint_path[current_waypoint_index]
            delta_x = target_waypoint_x - current_x
            delta_y = target_waypoint_y - current_y
            distance_to_waypoint = math.sqrt(delta_x * delta_x + delta_y * delta_y)
            angle_to_waypoint = math.atan2(delta_y, delta_x)
            angle_diff = normalize_angle(angle_to_waypoint - current_heading)
    
    # CHECK IF ROTATION NEEDED (BLOCKING)
    if abs(angle_diff) > ROTATION_THRESHOLD:
        rotate_to_angle(angle_to_waypoint, ROTATION_TOLERANCE)
        continue
    
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
    
    for wall_name, wall_data in wall_static_cache.items():
        closest_x, closest_y = closest_point_on_rectangle(
            current_x, current_y,
            wall_data['x'], wall_data['y'],
            wall_data['width'], wall_data['length'],
            0.0
        )
        
        obstacles.append({'x': closest_x, 'y': closest_y})
    
    # NAVIGATION MODE
    if distance_to_waypoint < SLOWDOWN_DISTANCE:
        speed_scale = distance_to_waypoint / SLOWDOWN_DISTANCE
        adaptive_max_speed = BASE_SPEED * max(speed_scale, 0.5)
    else:
        adaptive_max_speed = BASE_SPEED
    
    dwa_v, dwa_w, dwa_score = dynamic_window_approach(
        current_x, current_y, current_heading, current_v, current_w,
        target_waypoint_x, target_waypoint_y, obstacles,
        DWA_DT, DWA_PREDICT_TIME,
        DWA_V_SAMPLES, DWA_W_SAMPLES,
        adaptive_max_speed, MAX_ANGULAR_VELOCITY,
        MAX_LINEAR_ACCEL, MAX_ANGULAR_ACCEL,
        DWA_ALPHA, DWA_BETA, DWA_GAMMA
    )
    
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
    
    wheel_velocity = linear_velocity / WHEEL_RADIUS
    left_velocity = wheel_velocity - angular_velocity
    right_velocity = wheel_velocity + angular_velocity
    
    front_left_motor.setVelocity(left_velocity)
    back_left_motor.setVelocity(left_velocity)
    front_right_motor.setVelocity(right_velocity)
    back_right_motor.setVelocity(right_velocity)