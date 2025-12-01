import math

def closest_point_on_rectangle(point_x, point_y, rect_center_x, rect_center_y, rect_width, rect_length, rect_rotation=0.0):
    """
    Find closest point on a rectangle to a given point.
    
    Args:
        point_x, point_y: Point position
        rect_center_x, rect_center_y: Rectangle center
        rect_width: Rectangle width (x dimension)
        rect_length: Rectangle length (y dimension)
        rect_rotation: Rectangle rotation in radians (rotation around z-axis)
    
    Returns:
        (closest_x, closest_y): Closest point on rectangle edge
    """
    # Translate point to rectangle's local coordinate system
    dx = point_x - rect_center_x
    dy = point_y - rect_center_y
    
    # Rotate point by negative rectangle rotation
    cos_rot = math.cos(-rect_rotation)
    sin_rot = math.sin(-rect_rotation)
    local_x = dx * cos_rot - dy * sin_rot
    local_y = dx * sin_rot + dy * cos_rot
    
    # Clamp to rectangle bounds
    half_width = rect_width / 2.0
    half_length = rect_length / 2.0
    
    clamped_x = max(-half_width, min(half_width, local_x))
    clamped_y = max(-half_length, min(half_length, local_y))
    
    # Rotate back to world coordinates
    world_x = clamped_x * cos_rot + clamped_y * sin_rot + rect_center_x
    world_y = -clamped_x * sin_rot + clamped_y * cos_rot + rect_center_y
    
    return world_x, world_y


def distance_to_rectangle(point_x, point_y, rect_center_x, rect_center_y, rect_width, rect_length, rect_rotation=0.0):
    """
    Calculate minimum distance from point to rectangle.
    
    Returns:
        Distance in meters
    """
    closest_x, closest_y = closest_point_on_rectangle(
        point_x, point_y,
        rect_center_x, rect_center_y,
        rect_width, rect_length,
        rect_rotation
    )
    
    dx = point_x - closest_x
    dy = point_y - closest_y
    
    return math.sqrt(dx * dx + dy * dy)