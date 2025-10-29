import math

def ttc(ego_x, ego_v, lead_x, lead_v, eps=1e-6):
    rel_x = lead_x - ego_x
    rel_v = ego_v - lead_v
    if rel_v <= 0:  # not closing
        return math.inf
    return max(rel_x / max(rel_v, eps), 0.0)

def headway(ego_v, gap_m, eps=1e-6):
    return math.inf if abs(ego_v) < eps else gap_m / max(ego_v, eps)

def pack_risk(state):
    t = ttc(state["ego_x"], state["ego_v"], state["lead_x"], state["lead_v"])
    h = headway(state["ego_v"], state["gap_m"])
    return {
        "ttc": t,
        "headway": h,
        "margin_low": (t < 2.0 or h < 2.0),
        "violation_predicted": (t < 1.0 or h < 1.0),
    }