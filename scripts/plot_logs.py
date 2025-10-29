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

if __name__ == "__main__":
    plot()