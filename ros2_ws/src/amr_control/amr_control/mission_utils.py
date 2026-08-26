"""Pure helpers for route poses and odometry-based reverse motion."""

import math


def validate_route(node_ids, route_x, route_y, route_z):
    """Validate parallel route arrays and return normalized route points."""
    size = len(node_ids)
    coordinate_arrays = (route_x, route_y, route_z)
    if size == 0 or any(
        len(values) != size for values in coordinate_arrays
    ):
        raise ValueError(
            'Route arrays must be non-empty and have equal lengths.'
        )
    return tuple(
        (float(route_x[index]), float(route_y[index]), float(route_z[index]))
        for index in range(size)
    )


def route_yaws(points):
    """Return segment headings, retaining the incoming heading at the end."""
    if not points:
        raise ValueError('Route must contain at least one point.')
    if len(points) == 1:
        return (0.0,)
    headings = tuple(
        math.atan2(end[1] - start[1], end[0] - start[0])
        for start, end in zip(points, points[1:])
    )
    return headings + (headings[-1],)


def navigation_waypoints(points):
    """Skip the current first Node and return XYZ-yaw waypoint tuples."""
    if not points:
        raise ValueError('Route must contain at least one point.')
    if len(points) == 1:
        return ()

    yaws = route_yaws(points)
    return tuple(
        (points[index][0], points[index][1], points[index][2], yaws[index])
        for index in range(1, len(points))
    )


def validate_mission_routes(approach, delivery):
    """Validate that the two AMR route array groups meet at one Node."""
    approach_ids, approach_points = approach
    delivery_ids, delivery_points = delivery
    if int(approach_ids[-1]) != int(delivery_ids[0]):
        raise ValueError(
            'Approach and delivery routes do not meet at Task.start.'
        )
    if approach_points[-1] != delivery_points[0]:
        raise ValueError(
            'Approach and delivery coordinates differ at Task.start.'
        )
    return approach_points, delivery_points


def quaternion_z_w(yaw):
    """Return the non-zero components of a planar yaw quaternion."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quaternion_yaw(x, y, z, w):
    """Return planar yaw from an odometry quaternion."""
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def traveled_distance(start_xy, current_xy):
    """Return planar displacement used by the reverse controller."""
    return math.hypot(
        current_xy[0] - start_xy[0],
        current_xy[1] - start_xy[1],
    )
