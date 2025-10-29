def limit_speed(v_cmd, ttc_s, headway_s, v_cap, ttc_thr=1.5, headway_thr=1.5):
    v = min(v_cmd, v_cap)
    if ttc_s < ttc_thr or headway_s < headway_thr:
        v = min(v, 0.0)  # v1: safe stop when margin low
    return max(v, 0.0)

def limit_turn(w_cmd, w_cap):
    return max(min(w_cmd, w_cap), -w_cap)