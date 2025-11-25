import pandas as pd
import matplotlib.pyplot as plt

def plot(csv_path="data/logs/carla_rearend_numeric.csv"):
    df = pd.read_csv(csv_path)
    # Mode as simple integer for plotting
    mode_map = {"PASS":0, "GUARDED":1, "SAFE":2}
    df["mode_i"] = df["mode"].map(mode_map)

    plt.figure()
    plt.plot(df["time_s"], df["ttc_s"])
    plt.xlabel("time [s]"); plt.ylabel("TTC [s]"); plt.title("Time-to-Collision")

    plt.figure()
    plt.plot(df["time_s"], df["mode_i"])
    plt.xlabel("time [s]"); plt.ylabel("mode (0=PASS,1=GUARDED,2=SAFE)"); plt.title("Shield Mode")

    plt.figure()
    plt.plot(df["time_s"], df["v_in"], label="v_in")
    plt.plot(df["time_s"], df["v_out"], label="v_out")
    plt.xlabel("time [s]"); plt.ylabel("speed [arb]"); plt.title("Command vs Limited")
    plt.legend()
    plt.show()

def plot_agv(csv_path="data/logs/agv_queue_numeric.csv"):
    import pandas as pd
    import matplotlib.pyplot as plt
    df = pd.read_csv(csv_path)

    plt.figure()
    plt.step(df["t"], df["a0_pos"], where="post", label="AGV0 (follower)")
    plt.step(df["t"], df["a1_pos"], where="post", label="AGV1 (leader)")
    plt.xlabel("time [step]"); plt.ylabel("cell index")
    plt.title("AGV positions across junction")
    plt.legend()

    plt.figure()
    plt.step(df["t"], df["headway_cells"], where="post")
    plt.xlabel("time [step]"); plt.ylabel("headway [cells]")
    plt.title("Headway (should be ≥ 1 cell)")
    plt.show()

if __name__ == "__main__":
    try:
        plot_agv()  # AGV positions + headway
    except Exception as e:
        print("AGV plot error:", e)
