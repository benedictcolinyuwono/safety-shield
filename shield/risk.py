import math

def time_to_collision(ego_x, ego_v, lead_x, lead_v, epsilon=1e-6):
    gap = lead_x - ego_x
    closing_speed = ego_v - lead_v
    if closing_speed <= 0:
        return math.inf
    ttc_seconds = gap / max(closing_speed, epsilon)
    return max(ttc_seconds, 0.0)

def time_headway(ego_v, gap_m, epsilon=1e-6):
    if abs(ego_v) < epsilon:
        return math.inf
    return gap_m / max(ego_v, epsilon)

def assess_collision_risk(state):
    WARNING_THRESHOLD = 2.0
    CRITICAL_THRESHOLD = 1.0
    ttc = time_to_collision(
        state["ego_x"], state["ego_v"],
        state["lead_x"], state["lead_v"]
    )
    headway = time_headway(state["ego_v"], state["gap_m"])
    return {
        "ttc": ttc,
        "headway": headway,
        "margin_low": (ttc < WARNING_THRESHOLD or headway < WARNING_THRESHOLD),
        "violation_predicted": (ttc < CRITICAL_THRESHOLD or headway < CRITICAL_THRESHOLD),
    }