import pandas as pd
import matplotlib.pyplot as plt


def plot_rear_end(csv_path="data/logs/carla_rearend_numeric.csv"):
    df = pd.read_csv(csv_path)
    mode_map = {"PASS": 0, "GUARDED": 1, "EMERGENCY": 2}
    df["mode_int"] = df["mode"].map(mode_map)
    
    plt.figure(figsize=(10, 4))
    plt.plot(df["time_s"], df["ttc_s"])
    plt.xlabel("Time [s]")
    plt.ylabel("TTC [s]")
    plt.title("Time-to-Collision")
    plt.grid(True)
    
    plt.figure(figsize=(10, 4))
    plt.plot(df["time_s"], df["mode_int"])
    plt.xlabel("Time [s]")
    plt.ylabel("Mode (0=PASS, 1=GUARDED, 2=EMERGENCY)")
    plt.title("Shield Mode")
    plt.grid(True)
    
    plt.figure(figsize=(10, 4))
    plt.plot(df["time_s"], df["v_in"], label="Commanded")
    plt.plot(df["time_s"], df["v_out"], label="Limited")
    plt.xlabel("Time [s]")
    plt.ylabel("Speed [m/s]")
    plt.title("Velocity: Command vs Limited")
    plt.legend()
    plt.grid(True)
    
    plt.show()


def plot_agv(csv_path="data/logs/agv_queue_numeric.csv"):
    df = pd.read_csv(csv_path)
    
    plt.figure(figsize=(10, 4))
    plt.step(df["t"], df["a0_pos"], where="post", label="AGV 0 (follower)")
    plt.step(df["t"], df["a1_pos"], where="post", label="AGV 1 (leader)")
    plt.xlabel("Time [step]")
    plt.ylabel("Cell Index")
    plt.title("AGV Positions")
    plt.legend()
    plt.grid(True)
    
    plt.figure(figsize=(10, 4))
    plt.step(df["t"], df["headway_cells"], where="post")
    plt.axhline(y=1, color='r', linestyle='--', label="Minimum (1 cell)")
    plt.xlabel("Time [step]")
    plt.ylabel("Headway [cells]")
    plt.title("Inter-Vehicle Headway")
    plt.legend()
    plt.grid(True)
    
    plt.show()


if __name__ == "__main__":
    try:
        plot_agv()
    except Exception as e:
        print(f"Plotting error: {e}")