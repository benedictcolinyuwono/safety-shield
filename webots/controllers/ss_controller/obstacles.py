"""
Obstacle definitions for FG Warehouse
Single source of truth for all static obstacles (racks and walls)
"""

# RACK OBSTACLES
rack_obstacles = [
    # name, center_x, center_y, width, length
    ("FG_Top_RackRowA_1", 58.26, 64.0, 4.35, 37.0),
    ("FG_Top_RackRowA_2", 49.84, 64.0, 8.7, 37.0),
    ("FG_Top_RackRowA_3", 39.24, 64.0, 8.7, 37.0),
    ("FG_Top_RackRowA_4", 28.64, 64.0, 8.7, 37.0),
    ("FG_Top_RackRowA_5", 18.04, 64.0, 8.7, 37.0),
    ("FG_Top_RackRowA_6", 7.44, 64.0, 8.7, 37.0),
    ("FG_Top_RackRowA_7", -3.16, 64.0, 8.7, 37.0),
    ("FG_Top_RackRowA_8", -13.76, 64.0, 8.7, 37.0),
    ("FG_Top_RackRowA_9", -24.36, 64.0, 8.7, 37.0),
    ("FG_Top_RackRowA_10", -34.96, 64.0, 8.7, 37.0),
    ("FG_Top_RackRowA_11", -45.56, 64.0, 8.7, 37.0),
    ("FG_Top_RackRowA_12", -56.16, 64.0, 8.7, 37.0),
    ("FG_Top_RackRowA_13", -66.76, 64.0, 8.7, 37.0),
    ("FG_Top_RackRowA_14", -77.36, 64.0, 8.7, 37.0),
    ("FG_Top_RackRowA_15", -87.96, 64.0, 8.7, 37.0),
    ("FG_Top_RackRowA_16", -98.56, 64.0, 8.7, 37.0),
    ("FG_Top_RackRowB_1", 62.98, 17.1, 4.35, 31.7),
    ("FG_Top_RackRowB_2", 54.58, 17.1, 8.7, 31.7),
    ("FG_Top_RackRowB_3", 43.98, 17.1, 8.7, 31.7),
    ("FG_Top_RackRowB_4", 33.38, 17.1, 8.7, 31.7),
    ("FG_Top_RackRowB_5", 22.78, 17.1, 8.7, 31.7),
    ("FG_Top_RackRowB_6", 12.18, 17.1, 8.7, 31.7),
    ("FG_Top_RackRowB_7", 1.58, 17.1, 8.7, 31.7),
    ("FG_Top_RackRowB_8", -9.02, 17.1, 8.7, 31.7),
    ("FG_Top_RackRowB_9", -19.62, 17.1, 8.7, 31.7),
    ("FG_Top_RackRowB_10", -30.22, 17.1, 8.7, 31.7),
    ("FG_Top_RackRowB_11", -40.82, 17.1, 8.7, 31.7),
    ("FG_Top_RackRowB_12", -51.42, 17.1, 8.7, 31.7),
    ("FG_Top_RackRowB_13", -62.02, 17.1, 8.7, 31.7),
    ("FG_Top_RackRowR_1", -95.7, 12.63, 11.3, 15.3),
    ("FG_Middle_RackRow_1", 80.78, -20.15, 8.7, 37.0),
    ("FG_Middle_RackRow_2", 70.18, -20.15, 8.7, 37.0),
    ("FG_Middle_RackRow_3", 59.58, -20.15, 8.7, 37.0),
    ("FG_Middle_RackRow_4", 48.98, -20.15, 8.7, 37.0),
    ("FG_Middle_RackRow_5", 38.38, -20.15, 8.7, 37.0),
    ("FG_Middle_RackRow_6", 27.78, -20.15, 8.7, 37.0),
    ("FG_Middle_RackRow_7", 17.18, -20.15, 8.7, 37.0),
    ("FG_Middle_RackRow_8", 6.58, -20.15, 8.7, 37.0),
    ("FG_Middle_RackRow_9", -4.02, -20.15, 8.7, 37.0),
    ("FG_Middle_RackRow_10", -14.62, -20.15, 8.7, 37.0),
    ("FG_Middle_RackRow_11", -25.22, -20.15, 8.7, 37.0),
    ("FG_Middle_RackRow_12", -35.82, -20.15, 8.7, 37.0),
    ("FG_Middle_RackRow_13", -46.42, -20.15, 8.7, 37.0),
    ("FG_Middle_RackRow_14", -57.02, -20.15, 8.7, 37.0),
    ("FG_Middle_RackRowR_1", 83.18, -51.2, 11.3, 15.3),
    ("FG_Bottom_RackRow_1", -81.4, -66.7, 8.7, 31.7),
    ("FG_Bottom_RackRow_2", -70.8, -66.7, 8.7, 31.7),
    ("FG_Bottom_RackRow_3", -60.2, -66.7, 8.7, 31.7),
    ("FG_Bottom_RackRow_4", -49.6, -66.7, 8.7, 31.7),
    ("FG_Bottom_RackRow_5", -39.0, -66.7, 8.7, 31.7),
    ("FG_Bottom_RackRow_6", -28.4, -66.7, 8.7, 31.7),
    ("FG_Bottom_RackRow_7", -17.8, -66.7, 8.7, 31.7),
    ("FG_Bottom_RackRow_8", -7.2, -66.7, 8.7, 31.7),
    ("FG_Bottom_RackRowR_1", -97.75, -71.7, 11.3, 15.3),
    ("FG_PalletizingConveyors_1A", 23.02, -81.28, 30.1, 2.17),
    ("FG_PalletizingConveyors_1B", 9.06, -83.48, 2.17, 2.24),
    ("FG_PalletizingConveyors_2A", 55.75, -81.1, 32.19, 2.17),
    ("FG_PalletizingConveyors_2B", 70.76, -83.39, 2.17, 2.42),
    ("FG_PalletizingConveyors_3", 90.37, -81.1, 32.1, 2.17),
]

# WALL OBSTACLES
wall_obstacles = [
    # name, center_x, center_y, width, length
    # Top Section Walls
    ("FG_Top_Walls_1", -20.22, 84.55, 172.3, 0.1),
    ("FG_Top_Walls_2", 65.95, 42.05, 0.1, 85.1),
    ("FG_Top_Walls_3", -106.35, 42.05, 0.1, 85.1),
    
    # Middle Section Walls
    ("FG_Middle_Walls_1", 78.3, -0.5, 24.6, 0.1),
    ("FG_Middle_Walls_2", 90.53, -30.05, 0.1, 59.1),
    ("FG_Middle_Walls_3", -106.42, -30.05, 0.1, 59.1),
    
    # Bottom Section Walls
    ("FG_Bottom_Walls_1", 98.48, -59.6, 15.9, 0.1),
    ("FG_Bottom_Walls_2", 106.4, -72.1, 0.1, 25.0),
    ("FG_Bottom_Walls_3", -106.42, -72.1, 0.1, 25.0),
    ("FG_Bottom_Walls_4", 0.0, -84.6, 212.8, 0.1),
]