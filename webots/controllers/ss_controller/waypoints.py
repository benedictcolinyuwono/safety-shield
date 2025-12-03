"""
Complete warehouse waypoint network for FG Warehouse
FULLY CORRECTED: Manual calculation of all aisle positions
"""

import math

warehouse_waypoints = {}

def generate_waypoints(x_start, x_end, y_start, y_end, num_points):
    """Generate evenly spaced waypoints between two points"""
    waypoints = []
    if num_points == 1:
        waypoints.append(((x_start + x_end) / 2, (y_start + y_end) / 2))
    else:
        for i in range(num_points):
            t = i / (num_points - 1)
            x = x_start + t * (x_end - x_start)
            y = y_start + t * (y_end - y_start)
            waypoints.append((x, y))
    return waypoints


# ROW A - Manual calculation of each aisle
# Racks from world file with (center_x, width)
row_a_y_start = 45.5
row_a_y_end = 82.5

# Aisle 1: Between Rack 1 (58.26, 4.35) and Rack 2 (49.84, 8.7)
# Rack 1 left edge: 58.26 - 2.175 = 56.085
# Rack 2 right edge: 49.84 + 4.35 = 54.19
# Center: (56.085 + 54.19) / 2 = 55.14
warehouse_waypoints['row_a_aisle_1'] = generate_waypoints(55.14, 55.14, row_a_y_start, row_a_y_end, 7)

# Aisle 2: Between Rack 2 (49.84, 8.7) and Rack 3 (39.24, 8.7)
# Rack 2 left: 49.84 - 4.35 = 45.49
# Rack 3 right: 39.24 + 4.35 = 43.59
# Center: (45.49 + 43.59) / 2 = 44.54
warehouse_waypoints['row_a_aisle_2'] = generate_waypoints(44.54, 44.54, row_a_y_start, row_a_y_end, 7)

# Aisle 3: Between Rack 3 (39.24, 8.7) and Rack 4 (28.64, 8.7)
warehouse_waypoints['row_a_aisle_3'] = generate_waypoints(33.94, 33.94, row_a_y_start, row_a_y_end, 7)

# Aisle 4: Between Rack 4 (28.64, 8.7) and Rack 5 (18.04, 8.7)
warehouse_waypoints['row_a_aisle_4'] = generate_waypoints(23.34, 23.34, row_a_y_start, row_a_y_end, 7)

# Aisle 5: Between Rack 5 (18.04, 8.7) and Rack 6 (7.44, 8.7)
warehouse_waypoints['row_a_aisle_5'] = generate_waypoints(12.74, 12.74, row_a_y_start, row_a_y_end, 7)

# Aisle 6: Between Rack 6 (7.44, 8.7) and Rack 7 (-3.16, 8.7)
warehouse_waypoints['row_a_aisle_6'] = generate_waypoints(2.14, 2.14, row_a_y_start, row_a_y_end, 7)

# Aisle 7: Between Rack 7 (-3.16, 8.7) and Rack 8 (-13.76, 8.7)
warehouse_waypoints['row_a_aisle_7'] = generate_waypoints(-8.46, -8.46, row_a_y_start, row_a_y_end, 7)

# Aisle 8: Between Rack 8 (-13.76, 8.7) and Rack 9 (-24.36, 8.7)
warehouse_waypoints['row_a_aisle_8'] = generate_waypoints(-19.06, -19.06, row_a_y_start, row_a_y_end, 7)

# Aisle 9: Between Rack 9 (-24.36, 8.7) and Rack 10 (-34.96, 8.7)
warehouse_waypoints['row_a_aisle_9'] = generate_waypoints(-29.66, -29.66, row_a_y_start, row_a_y_end, 7)

# Aisle 10: Between Rack 10 (-34.96, 8.7) and Rack 11 (-45.56, 8.7)
warehouse_waypoints['row_a_aisle_10'] = generate_waypoints(-40.26, -40.26, row_a_y_start, row_a_y_end, 7)

# Aisle 11: Between Rack 11 (-45.56, 8.7) and Rack 12 (-56.16, 8.7)
warehouse_waypoints['row_a_aisle_11'] = generate_waypoints(-50.86, -50.86, row_a_y_start, row_a_y_end, 7)

# Aisle 12: Between Rack 12 (-56.16, 8.7) and Rack 13 (-66.76, 8.7)
warehouse_waypoints['row_a_aisle_12'] = generate_waypoints(-61.46, -61.46, row_a_y_start, row_a_y_end, 7)

# Aisle 13: Between Rack 13 (-66.76, 8.7) and Rack 14 (-77.36, 8.7)
warehouse_waypoints['row_a_aisle_13'] = generate_waypoints(-72.06, -72.06, row_a_y_start, row_a_y_end, 7)

# Aisle 14: Between Rack 14 (-77.36, 8.7) and Rack 15 (-87.96, 8.7)
warehouse_waypoints['row_a_aisle_14'] = generate_waypoints(-82.66, -82.66, row_a_y_start, row_a_y_end, 7)

# Aisle 15: Between Rack 15 (-87.96, 8.7) and Rack 16 (-98.56, 8.7)
warehouse_waypoints['row_a_aisle_15'] = generate_waypoints(-93.26, -93.26, row_a_y_start, row_a_y_end, 7)

# Row A access corridors
warehouse_waypoints['row_a_west_access'] = generate_waypoints(-103.0, -103.0, row_a_y_start, row_a_y_end, 7)
warehouse_waypoints['row_a_east_access'] = generate_waypoints(61.5, 61.5, row_a_y_start, row_a_y_end, 7)


# ROW B - Manual calculation
row_b_y_start = 1.25
row_b_y_end = 32.95

warehouse_waypoints['row_b_aisle_1'] = generate_waypoints(58.78, 58.78, row_b_y_start, row_b_y_end, 6)
warehouse_waypoints['row_b_aisle_2'] = generate_waypoints(49.28, 49.28, row_b_y_start, row_b_y_end, 6)
warehouse_waypoints['row_b_aisle_3'] = generate_waypoints(38.68, 38.68, row_b_y_start, row_b_y_end, 6)
warehouse_waypoints['row_b_aisle_4'] = generate_waypoints(28.08, 28.08, row_b_y_start, row_b_y_end, 6)
warehouse_waypoints['row_b_aisle_5'] = generate_waypoints(17.48, 17.48, row_b_y_start, row_b_y_end, 6)
warehouse_waypoints['row_b_aisle_6'] = generate_waypoints(6.88, 6.88, row_b_y_start, row_b_y_end, 6)
warehouse_waypoints['row_b_aisle_7'] = generate_waypoints(-3.72, -3.72, row_b_y_start, row_b_y_end, 6)
warehouse_waypoints['row_b_aisle_8'] = generate_waypoints(-14.32, -14.32, row_b_y_start, row_b_y_end, 6)
warehouse_waypoints['row_b_aisle_9'] = generate_waypoints(-24.92, -24.92, row_b_y_start, row_b_y_end, 6)
warehouse_waypoints['row_b_aisle_10'] = generate_waypoints(-35.52, -35.52, row_b_y_start, row_b_y_end, 6)
warehouse_waypoints['row_b_aisle_11'] = generate_waypoints(-46.12, -46.12, row_b_y_start, row_b_y_end, 6)
warehouse_waypoints['row_b_aisle_12'] = generate_waypoints(-56.72, -56.72, row_b_y_start, row_b_y_end, 6)

warehouse_waypoints['row_b_west_access'] = generate_waypoints(-67.0, -67.0, row_b_y_start, row_b_y_end, 6)
warehouse_waypoints['row_b_east_access'] = generate_waypoints(66.0, 66.0, row_b_y_start, row_b_y_end, 6)


# MIDDLE SECTION
middle_y_start = -38.65
middle_y_end = -1.65

warehouse_waypoints['middle_aisle_1'] = generate_waypoints(75.48, 75.48, middle_y_start, middle_y_end, 7)
warehouse_waypoints['middle_aisle_2'] = generate_waypoints(64.88, 64.88, middle_y_start, middle_y_end, 7)
warehouse_waypoints['middle_aisle_3'] = generate_waypoints(54.28, 54.28, middle_y_start, middle_y_end, 7)
warehouse_waypoints['middle_aisle_4'] = generate_waypoints(43.68, 43.68, middle_y_start, middle_y_end, 7)
warehouse_waypoints['middle_aisle_5'] = generate_waypoints(33.08, 33.08, middle_y_start, middle_y_end, 7)
warehouse_waypoints['middle_aisle_6'] = generate_waypoints(22.48, 22.48, middle_y_start, middle_y_end, 7)
warehouse_waypoints['middle_aisle_7'] = generate_waypoints(11.88, 11.88, middle_y_start, middle_y_end, 7)
warehouse_waypoints['middle_aisle_8'] = generate_waypoints(1.28, 1.28, middle_y_start, middle_y_end, 7)
warehouse_waypoints['middle_aisle_9'] = generate_waypoints(-9.32, -9.32, middle_y_start, middle_y_end, 7)
warehouse_waypoints['middle_aisle_10'] = generate_waypoints(-19.92, -19.92, middle_y_start, middle_y_end, 7)
warehouse_waypoints['middle_aisle_11'] = generate_waypoints(-30.52, -30.52, middle_y_start, middle_y_end, 7)
warehouse_waypoints['middle_aisle_12'] = generate_waypoints(-41.12, -41.12, middle_y_start, middle_y_end, 7)
warehouse_waypoints['middle_aisle_13'] = generate_waypoints(-51.72, -51.72, middle_y_start, middle_y_end, 7)

warehouse_waypoints['middle_west_access'] = generate_waypoints(-62.0, -62.0, middle_y_start, middle_y_end, 7)
warehouse_waypoints['middle_east_access'] = generate_waypoints(86.0, 86.0, middle_y_start, middle_y_end, 7)


# BOTTOM SECTION
bottom_y_start = -82.55
bottom_y_end = -50.85

warehouse_waypoints['bottom_aisle_1'] = generate_waypoints(-76.1, -76.1, bottom_y_start, bottom_y_end, 6)
warehouse_waypoints['bottom_aisle_2'] = generate_waypoints(-65.5, -65.5, bottom_y_start, bottom_y_end, 6)
warehouse_waypoints['bottom_aisle_3'] = generate_waypoints(-54.9, -54.9, bottom_y_start, bottom_y_end, 6)
warehouse_waypoints['bottom_aisle_4'] = generate_waypoints(-44.3, -44.3, bottom_y_start, bottom_y_end, 6)
warehouse_waypoints['bottom_aisle_5'] = generate_waypoints(-33.7, -33.7, bottom_y_start, bottom_y_end, 6)
warehouse_waypoints['bottom_aisle_6'] = generate_waypoints(-23.1, -23.1, bottom_y_start, bottom_y_end, 6)
warehouse_waypoints['bottom_aisle_7'] = generate_waypoints(-12.5, -12.5, bottom_y_start, bottom_y_end, 6)

warehouse_waypoints['bottom_west_access'] = generate_waypoints(-87.0, -87.0, bottom_y_start, bottom_y_end, 6)
warehouse_waypoints['bottom_east_access'] = generate_waypoints(-2.0, -2.0, bottom_y_start, bottom_y_end, 6)


# RACK R ACCESS
warehouse_waypoints['top_rackr_west_access'] = generate_waypoints(-102.0, -102.0, 5.0, 20.0, 3)
warehouse_waypoints['top_rackr_east_access'] = generate_waypoints(-90.0, -90.0, 5.0, 20.0, 3)

warehouse_waypoints['middle_rackr_west_access'] = generate_waypoints(77.5, 77.5, -59.0, -44.0, 3)
warehouse_waypoints['middle_rackr_east_access'] = generate_waypoints(89.0, 89.0, -59.0, -44.0, 3)

warehouse_waypoints['bottom_rackr_west_access'] = generate_waypoints(-104.0, -104.0, -80.0, -64.0, 3)
warehouse_waypoints['bottom_rackr_east_access'] = generate_waypoints(-92.0, -92.0, -80.0, -64.0, 3)


# CONVEYOR PICKUP (stay well clear of walls)
warehouse_waypoints['conveyor_1a_pickup'] = generate_waypoints(10.0, 36.0, -79.5, -79.5, 15)
warehouse_waypoints['conveyor_2a_pickup'] = generate_waypoints(41.0, 70.0, -79.5, -79.5, 16)
warehouse_waypoints['conveyor_3_pickup'] = generate_waypoints(76.0, 103.0, -79.5, -79.5, 16)


# CROSS-AISLES (stay 3m clear of racks)
warehouse_waypoints['top_north_perimeter'] = generate_waypoints(-100.0, 58.0, 83.0, 83.0, 15)
warehouse_waypoints['top_cross_main'] = generate_waypoints(-100.0, 60.0, 38.0, 38.0, 20)
warehouse_waypoints['row_b_south_perimeter'] = generate_waypoints(-64.0, 64.0, 0.0, 0.0, 15)
warehouse_waypoints['top_middle_transition'] = generate_waypoints(-60.0, 84.0, -1.5, -1.5, 18)
warehouse_waypoints['middle_north_perimeter'] = generate_waypoints(-60.0, 84.0, -2.5, -2.5, 18)
warehouse_waypoints['middle_south_perimeter'] = generate_waypoints(-60.0, 84.0, -39.5, -39.5, 18)
warehouse_waypoints['middle_bottom_transition'] = generate_waypoints(-100.0, 84.0, -46.0, -46.0, 20)
warehouse_waypoints['bottom_north_perimeter'] = generate_waypoints(-86.0, 0.0, -48.0, -48.0, 12)
warehouse_waypoints['bottom_south_perimeter'] = generate_waypoints(-86.0, 0.0, -83.5, -83.5, 12)
warehouse_waypoints['palletizing_access_corridor'] = generate_waypoints(2.0, 102.0, -73.0, -73.0, 15)

warehouse_waypoints['east_perimeter_corridor'] = [
    (61.0, 80.0), (61.0, 60.0), (61.0, 40.0), (61.0, 20.0), (61.0, 5.0),
    (85.5, -5.0), (85.5, -20.0), (85.5, -35.0), (85.5, -48.0),
    (3.0, -55.0), (3.0, -65.0), (3.0, -75.0)
]

warehouse_waypoints['west_perimeter_corridor'] = [
    (-104.0, 80.0), (-104.0, 60.0), (-104.0, 40.0), (-104.0, 20.0), (-104.0, 5.0),
    (-80.0, -5.0), (-80.0, -20.0), (-80.0, -35.0), (-80.0, -48.0),
    (-104.0, -55.0), (-104.0, -65.0), (-104.0, -75.0), (-104.0, -80.0)
]


print("=" * 70)
print("WAREHOUSE WAYPOINT NETWORK (MANUALLY CORRECTED)")
print("=" * 70)
print(f"Total Groups: {len(warehouse_waypoints)}")
total_waypoints = sum(len(wp) for wp in warehouse_waypoints.values())
print(f"Total Waypoints: {total_waypoints}")
print("=" * 70)