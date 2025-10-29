import csv, math
from shield.risk import pack_risk
from shield.supervisor import supervise, PASS, GUARDED, SAFE
from sims.rear_end_numeric import step_1d

def run(output_csv="data/logs/carla_rearend_numeric.csv"):
    # scenario: ego starts 15 m behind, both at 12 m/s; lead brakes hard at t=3 s
    dt = 0.1
    T  = 10.0
    t  = 0.0

    ego_x,  ego_v  = 0.0, 12.0
    lead_x, lead_v = 15.0, 12.0

    v_cap = 15.0
    mode  = PASS

    with open(output_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time_s","mode","ttc_s","headway_s","v_in","w_in","v_out","w_out","step_ms"])

        while t <= T + 1e-9:
            # lead brakes after t>=3 s
            a_lead = -6.0 if t >= 3.0 else 0.0

            # desired command: keep speed (no steering in 1D)
            v_cmd, w_cmd = ego_v, 0.0

            # state → risk → supervisor
            gap_m = max(lead_x - ego_x, 0.0)
            state = {"ego_x": ego_x, "ego_v": ego_v, "lead_x": lead_x, "lead_v": lead_v, "gap_m": gap_m, "v_cap": v_cap}
            risk  = pack_risk(state)
            mode, (v_out, w_out) = supervise(state, (v_cmd, w_cmd), risk)

            # crude mapping: velocity command → acceleration
            a_ego = (v_out - ego_v) * 2.0  # proportional accel towards commanded speed

            # integrate one step
            ego_x, ego_v, lead_x, lead_v = step_1d(ego_x, ego_v, lead_x, lead_v, a_ego, a_lead, dt)

            # time to collision (for logging) is risk["ttc"]
            w.writerow([round(t,3), mode, risk["ttc"], risk["headway"], v_cmd, w_cmd, v_out, w_out, 0.0])

            # stop early if we would collide (gap ~0)
            if lead_x - ego_x <= 0.0 and not math.isinf(risk["ttc"]):
                break

            t += dt

if __name__ == "__main__":
    run()
    print("Wrote data/logs/carla_rearend_numeric.csv")