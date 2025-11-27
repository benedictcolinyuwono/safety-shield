def euler_step_1d(ego_x, ego_v, lead_x, lead_v, a_ego, a_lead, dt=0.1):
    new_ego_v = max(ego_v + a_ego * dt, 0.0)
    new_lead_v = max(lead_v + a_lead * dt, 0.0)
    new_ego_x = ego_x + new_ego_v * dt
    new_lead_x = lead_x + new_lead_v * dt
    return new_ego_x, new_ego_v, new_lead_x, new_lead_v