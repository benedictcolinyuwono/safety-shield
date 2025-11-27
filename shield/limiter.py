def limit_velocity(v_cmd, ttc_s, headway_s, v_max):
    CRITICAL_THRESHOLD = 1.0
    WARNING_THRESHOLD = 2.0
    
    v_limited = min(v_cmd, v_max)
    min_safety_margin = min(ttc_s, headway_s)

    if min_safety_margin < CRITICAL_THRESHOLD:
        v_limited = 0.0
    elif min_safety_margin < WARNING_THRESHOLD:
        safety_factor = (min_safety_margin - CRITICAL_THRESHOLD) / (WARNING_THRESHOLD - CRITICAL_THRESHOLD)
        v_limited = v_limited * safety_factor
    
    return max(v_limited, 0.0)

def limit_angular_velocity(w_cmd, w_max):
    return max(min(w_cmd, w_max), -w_max)