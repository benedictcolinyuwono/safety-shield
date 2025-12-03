"""
A* Path Planning for Warehouse Navigation
Builds waypoint connectivity graph and finds optimal paths
"""

import math
from waypoints import warehouse_waypoints
from obstacles import rack_obstacles, wall_obstacles

class PathPlanner:
    def __init__(self, max_connection_distance=100.0):
        """
        Initialize path planner with waypoint network
        
        Args:
            max_connection_distance: Maximum distance to connect waypoints (meters)
        """
        self.waypoints = warehouse_waypoints
        self.graph = {}
        self.max_connection_distance = max_connection_distance
        self.rack_obstacles = []
        self.wall_obstacles = []
        self._load_obstacles()
        self._build_graph()
    
    def _load_obstacles(self):
        """Load rack and wall obstacle definitions"""
        for rack_data in rack_obstacles:
            name, center_x, center_y, width, length = rack_data
            self.rack_obstacles.append({
                'name': name,
                'x': center_x,
                'y': center_y,
                'width': width,
                'length': length
            })
        
        for wall_data in wall_obstacles:
            name, center_x, center_y, width, length = wall_data
            self.wall_obstacles.append({
                'name': name,
                'x': center_x,
                'y': center_y,
                'width': width,
                'length': length
            })
    
    def _line_too_close_to_obstacle(self, x1, y1, x2, y2, clearance=0.5):
        """Check if line segment gets too close to any rack or wall"""
        for rack in self.rack_obstacles:
            if self._line_too_close_to_rectangle(x1, y1, x2, y2, 
                                                 rack['x'], rack['y'], 
                                                 rack['width'], rack['length'],
                                                 clearance):
                return True
        
        for wall in self.wall_obstacles:
            if self._line_too_close_to_rectangle(x1, y1, x2, y2,
                                                 wall['x'], wall['y'],
                                                 wall['width'], wall['length'],
                                                 clearance):
                return True
        
        return False
    
    def _line_too_close_to_rectangle(self, x1, y1, x2, y2, rect_x, rect_y, rect_width, rect_length, clearance):
        """Check if line segment gets within clearance distance of rectangle"""
        half_w = rect_width / 2.0 + clearance
        half_l = rect_length / 2.0 + clearance
        
        rect_min_x = rect_x - half_w
        rect_max_x = rect_x + half_w
        rect_min_y = rect_y - half_l
        rect_max_y = rect_y + half_l
        
        num_samples = 15
        for i in range(num_samples + 1):
            t = i / num_samples
            px = x1 + t * (x2 - x1)
            py = y1 + t * (y2 - y1)
            
            if (rect_min_x <= px <= rect_max_x and rect_min_y <= py <= rect_max_y):
                return True
        
        return False
    
    def _build_graph(self):
        """Build connectivity graph between waypoints with collision checking"""
        self.waypoint_list = []
        self.waypoint_coords = {}
        
        for aisle_name, positions in self.waypoints.items():
            for i, (x, y) in enumerate(positions):
                wp_id = f"{aisle_name}_{i}"
                self.waypoint_list.append(wp_id)
                self.waypoint_coords[wp_id] = (x, y)
        
        for wp1_id in self.waypoint_list:
            self.graph[wp1_id] = []
            x1, y1 = self.waypoint_coords[wp1_id]
            
            for wp2_id in self.waypoint_list:
                if wp1_id == wp2_id:
                    continue
                
                x2, y2 = self.waypoint_coords[wp2_id]
                distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                
                if distance <= self.max_connection_distance:
                    if not self._line_too_close_to_obstacle(x1, y1, x2, y2, clearance=0.5):
                        self.graph[wp1_id].append((wp2_id, distance))
    
    def _heuristic(self, wp_id, goal_id):
        """A* heuristic: Euclidean distance to goal"""
        x1, y1 = self.waypoint_coords[wp_id]
        x2, y2 = self.waypoint_coords[goal_id]
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def find_path(self, start_x, start_y, goal_aisle, goal_position):
        """
        Find optimal path from start position to goal waypoint using A*
        
        Args:
            start_x, start_y: Starting position
            goal_aisle: Target aisle name (e.g., 'row_a_aisle_1')
            goal_position: Position index in aisle (e.g., 3)
        
        Returns:
            List of (x, y) coordinates representing path
        """
        goal_wp_id = f"{goal_aisle}_{goal_position}"
        
        if goal_wp_id not in self.waypoint_coords:
            print(f"ERROR: Goal {goal_wp_id} not found!")
            return None
        
        goal_coords = self.waypoint_coords[goal_wp_id]
        
        # Find nearest waypoint that's in the direction of the goal
        start_wp_id = self._find_nearest_waypoint_toward_goal(start_x, start_y, goal_coords[0], goal_coords[1])
        start_coords = self.waypoint_coords[start_wp_id]
        
        open_set = {start_wp_id}
        came_from = {}
        
        g_score = {wp_id: math.inf for wp_id in self.waypoint_list}
        g_score[start_wp_id] = 0
        
        f_score = {wp_id: math.inf for wp_id in self.waypoint_list}
        f_score[start_wp_id] = self._heuristic(start_wp_id, goal_wp_id)
        
        while open_set:
            current = min(open_set, key=lambda wp: f_score[wp])
            
            if current == goal_wp_id:
                path = self._reconstruct_path(came_from, current)
                return path
            
            open_set.remove(current)
            
            for neighbor, edge_cost in self.graph[current]:
                tentative_g = g_score[current] + edge_cost
                
                if tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal_wp_id)
                    
                    if neighbor not in open_set:
                        open_set.add(neighbor)
        
        print(f"ERROR: No path found from {start_wp_id} to {goal_wp_id}!")
        return None
    
    def _find_nearest_waypoint_toward_goal(self, x, y, goal_x, goal_y):
        """Find closest waypoint that's generally in the direction of the goal"""
        # Calculate direction vector to goal
        dx_to_goal = goal_x - x
        dy_to_goal = goal_y - y
        
        best_waypoint = None
        best_score = -math.inf
        
        for wp_id, (wx, wy) in self.waypoint_coords.items():
            # Distance to this waypoint
            dx_to_wp = wx - x
            dy_to_wp = wy - y
            dist_to_wp = math.sqrt(dx_to_wp * dx_to_wp + dy_to_wp * dy_to_wp)
            
            if dist_to_wp < 0.1:
                continue
            
            # Dot product: how aligned is this waypoint with goal direction?
            # Normalized to [-1, 1] where 1 = same direction, -1 = opposite
            goal_dist = math.sqrt(dx_to_goal * dx_to_goal + dy_to_goal * dy_to_goal)
            if goal_dist < 0.1:
                alignment = 1.0
            else:
                alignment = (dx_to_wp * dx_to_goal + dy_to_wp * dy_to_goal) / (dist_to_wp * goal_dist)
            
            # Score: prefer waypoints that are close AND aligned with goal
            # Heavily penalize waypoints in the opposite direction
            if alignment < 0:
                continue
            
            score = alignment * 2.0 - (dist_to_wp / 50.0)
            
            if score > best_score:
                best_score = score
                best_waypoint = wp_id
        
        # Fallback: if no good waypoint found, just use nearest
        if best_waypoint is None:
            best_waypoint = self._find_nearest_waypoint(x, y)
        
        return best_waypoint
    
    def _find_nearest_waypoint(self, x, y):
        """Find closest waypoint to given position (fallback method)"""
        min_dist = math.inf
        nearest = None
        
        for wp_id, (wx, wy) in self.waypoint_coords.items():
            dist = math.sqrt((wx - x)**2 + (wy - y)**2)
            if dist < min_dist:
                min_dist = dist
                nearest = wp_id
        
        return nearest
    
    def _reconstruct_path(self, came_from, current):
        """Reconstruct path from A* search"""
        path = [self.waypoint_coords[current]]
        
        while current in came_from:
            current = came_from[current]
            path.append(self.waypoint_coords[current])
        
        path.reverse()
        return path


path_planner = None

def get_path_planner():
    """Get or create global path planner instance"""
    global path_planner
    if path_planner is None:
        path_planner = PathPlanner(max_connection_distance=100.0)
    return path_planner


if __name__ == "__main__":
    planner = PathPlanner()
    
    test_path = planner.find_path(4.89, 41.15, 'row_a_aisle_1', 3)
    
    if test_path:
        print("\nTest path:")
        for i, (x, y) in enumerate(test_path):
            print(f"  {i+1}: ({x:.2f}, {y:.2f})")