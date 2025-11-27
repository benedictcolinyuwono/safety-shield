import csv
import math
from shield.risk import assess_collision_risk
from shield.supervisor import supervise_commands, PASS
from sims.rear_end_numeric import euler_step_1d

DT = 0.1
DURATION = 10.0
BRAKE_TIME = 3.0

EGO_START_X = 0.0
EGO_START_V = 12.0
LEAD_START_X = 15.0
LEAD_START_V = 12.0
LEAD_BRAKE_ACCEL = -6.0

V_CAP = 15.0


def run(output_csv="data/logs/carla_rearend_numeric.csv"):
    t = 0.0
    ego_x, ego_v = EGO_START_X, EGO_START_V
    lead_x, lead_v = LEAD_START_X, LEAD_START_V
    mode = PASS
    
    with open(output_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_s", "mode", "ttc_s", "headway_s", 
                        "v_in", "w_in", "v_out", "w_out", "step_ms"])
        
        while t <= DURATION:
            a_lead = LEAD_BRAKE_ACCEL if t >= BRAKE_TIME else 0.0
            
            v_cmd, w_cmd = ego_v, 0.0
            
            gap_m = max(lead_x - ego_x, 0.0)
            state = {
                "ego_x": ego_x,
                "ego_v": ego_v,
                "lead_x": lead_x,
                "lead_v": lead_v,
                "gap_m": gap_m,
                "v_cap": V_CAP
            }
            
            risk = assess_collision_risk(state)
            mode, (v_out, w_out) = supervise_commands(state, (v_cmd, w_cmd), risk)
            
            a_ego = (v_out - ego_v) * 2.0
            
            ego_x, ego_v, lead_x, lead_v = euler_step_1d(
                ego_x, ego_v, lead_x, lead_v, a_ego, a_lead, DT
            )
            
            writer.writerow([
                round(t, 3), mode, risk["ttc"], risk["headway"],
                v_cmd, w_cmd, v_out, w_out, 0.0
            ])
            
            if lead_x - ego_x <= 0.0 and not math.isinf(risk["ttc"]):
                print(f"Collision at t={t:.2f}s!")
                break
            
            t += DT
    
    print(f"Simulation complete: {output_csv}")


if __name__ == "__main__":
    run()
    print("Wrote data/logs/carla_rearend_numeric.csv")