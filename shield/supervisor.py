from .limiter import limit_velocity

MODE_PASS = "PASS"
MODE_GUARDED = "GUARDED"
MODE_EMERGENCY = "EMERGENCY"

PASS = MODE_PASS
GUARDED = MODE_GUARDED
SAFE = MODE_EMERGENCY

def supervise_commands(state, commanded_velocities, risk_assessment):
    v_cmd, w_cmd = commanded_velocities
    
    if risk_assessment["violation_predicted"]:
        return MODE_EMERGENCY, (0.0, 0.0)
    
    if risk_assessment["margin_low"]:
        v_safe = limit_velocity(
            v_cmd,
            risk_assessment["ttc"],
            risk_assessment["headway"],
            state["v_cap"]
        )
        return MODE_GUARDED, (v_safe, 0.0)
    
    return MODE_PASS, (v_cmd, w_cmd)


supervise = supervise_commands