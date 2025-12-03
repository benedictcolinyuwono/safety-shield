"""
Waypoint Validation Script
Proves that waypoints are in navigable space (not inside racks/walls)
"""

import math
from waypoints import warehouse_waypoints
from obstacles import rack_obstacles, wall_obstacles

def point_inside_rectangle(px, py, rect_x, rect_y, rect_width, rect_length):
    """Check if point is inside rectangle"""
    half_w = rect_width / 2.0
    half_l = rect_length / 2.0
    
    if (rect_x - half_w <= px <= rect_x + half_w and
        rect_y - half_l <= py <= rect_y + half_l):
        return True
    return False

def validate_waypoints():
    """Validate all waypoints are in navigable space"""
    
    print("=" * 70)
    print("WAYPOINT VALIDATION")
    print("=" * 70)
    
    total_waypoints = 0
    invalid_waypoints = []
    
    # Check each waypoint
    for aisle_name, positions in warehouse_waypoints.items():
        for i, (x, y) in enumerate(positions):
            total_waypoints += 1
            wp_id = f"{aisle_name}_{i}"
            
            # Check against racks
            for rack_data in rack_obstacles:
                rack_name, rack_x, rack_y, rack_width, rack_length = rack_data
                if point_inside_rectangle(x, y, rack_x, rack_y, rack_width, rack_length):
                    invalid_waypoints.append({
                        'wp_id': wp_id,
                        'position': (x, y),
                        'obstacle': rack_name,
                        'reason': 'Inside rack'
                    })
            
            # Check against walls
            for wall_data in wall_obstacles:
                wall_name, wall_x, wall_y, wall_width, wall_length = wall_data
                if point_inside_rectangle(x, y, wall_x, wall_y, wall_width, wall_length):
                    invalid_waypoints.append({
                        'wp_id': wp_id,
                        'position': (x, y),
                        'obstacle': wall_name,
                        'reason': 'Inside wall'
                    })
    
    print(f"\nTotal waypoints: {total_waypoints}")
    print(f"Invalid waypoints: {len(invalid_waypoints)}")
    
    if invalid_waypoints:
        print("\n❌ INVALID WAYPOINTS FOUND:")
        for inv in invalid_waypoints:
            print(f"  - {inv['wp_id']} at {inv['position']} : {inv['reason']} ({inv['obstacle']})")
    else:
        print("\n✅ ALL WAYPOINTS VALID - No collisions with obstacles!")
    
    # Check warehouse bounds
    print("\n" + "=" * 70)
    print("WAREHOUSE BOUNDS CHECK")
    print("=" * 70)
    
    all_x = []
    all_y = []
    for positions in warehouse_waypoints.values():
        for x, y in positions:
            all_x.append(x)
            all_y.append(y)
    
    min_x, max_x = min(all_x), max(all_x)
    min_y, max_y = min(all_y), max(all_y)
    
    print(f"X range: {min_x:.1f} to {max_x:.1f} (span: {max_x - min_x:.1f}m)")
    print(f"Y range: {min_y:.1f} to {max_y:.1f} (span: {max_y - min_y:.1f}m)")
    
    # Expected warehouse dimensions from world file
    expected_x_range = (-106, 106)
    expected_y_range = (-85, 85)
    
    if (min_x >= expected_x_range[0] and max_x <= expected_x_range[1] and
        min_y >= expected_y_range[0] and max_y <= expected_y_range[1]):
        print("✅ All waypoints within warehouse bounds")
    else:
        print("⚠️  Some waypoints outside expected bounds")
    
    # Print sample waypoints from each section
    print("\n" + "=" * 70)
    print("SAMPLE WAYPOINTS BY SECTION")
    print("=" * 70)
    
    samples = [
        ('row_a_aisle_1', 0),
        ('row_a_aisle_1', 3),
        ('row_b_aisle_3', 4),
        ('middle_aisle_7', 5),
        ('bottom_aisle_4', 3),
        ('conveyor_1a_pickup', 7)
    ]
    
    for aisle, idx in samples:
        if aisle in warehouse_waypoints and idx < len(warehouse_waypoints[aisle]):
            x, y = warehouse_waypoints[aisle][idx]
            print(f"{aisle}[{idx}]: ({x:.2f}, {y:.2f})")
    
    print("=" * 70)
    
    return len(invalid_waypoints) == 0


if __name__ == "__main__":
    is_valid = validate_waypoints()
    if is_valid:
        print("\n✅ WAYPOINT NETWORK VALIDATED SUCCESSFULLY!")
    else:
        print("\n❌ WAYPOINT NETWORK HAS ERRORS - FIX REQUIRED")