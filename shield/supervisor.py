from .limiter import limit_speed

PASS, GUARDED, SAFE = "PASS", "GUARDED", "SAFE"

def supervise(state, cmd, risk):
    v_cmd, w_cmd = cmd
    if risk["violation_predicted"]:
        return SAFE, (0.0, 0.0)
    if risk["margin_low"]:
        v_out = limit_speed(v_cmd, risk["ttc"], risk["headway"], state["v_cap"])
        return GUARDED, (v_out, 0.0)  # keep straight for v1
    return PASS, (v_cmd, w_cmd)