"""Tests for mission route orientation and odometry reverse helpers."""

import math

import pytest

from amr_control.mission_utils import (
    navigation_waypoints,
    quaternion_yaw,
    quaternion_z_w,
    route_yaws,
    traveled_distance,
    validate_mission_routes,
    validate_route,
)


@pytest.mark.parametrize(
    ('points', 'expected'),
    [
        (((0.0, 0.0, 0.0), (2.0, 0.0, 0.0)), 0.0),
        (((0.0, 0.0, 0.0), (0.0, 2.0, 0.0)), math.pi / 2.0),
        (((0.0, 0.0, 0.0), (2.0, 2.0, 0.0)), math.pi / 4.0),
        (((0.0, 0.0, 0.0), (-2.0, 0.0, 0.0)), math.pi),
        (((0.0, 0.0, 0.0), (0.0, -2.0, 0.0)), -math.pi / 2.0),
    ],
)
def test_route_heading_directions(points, expected):
    """Horizontal, vertical, diagonal, and negative headings use atan2."""
    assert route_yaws(points)[0] == pytest.approx(expected)


def test_route_yaws_keep_incoming_heading_at_final_node():
    """The final waypoint keeps the last segment heading."""
    points = (
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.0, 2.0, 0.0),
    )
    assert route_yaws(points) == pytest.approx(
        (0.0, math.pi / 2.0, math.pi / 2.0)
    )


def test_navigation_waypoints_skip_first_route_node():
    """The current first Node is omitted from NavigateThroughPoses."""
    points = (
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.1),
        (1.0, 1.0, 0.2),
    )
    waypoints = navigation_waypoints(points)
    assert len(waypoints) == 2
    assert waypoints[0] == pytest.approx((1.0, 0.0, 0.1, math.pi / 2.0))
    assert waypoints[-1] == pytest.approx((1.0, 1.0, 0.2, math.pi / 2.0))


def test_single_node_route_is_already_at_goal():
    """A one-Node route produces no navigation Action poses."""
    assert navigation_waypoints(((1.0, 2.0, 0.0),)) == ()


def test_yaw_quaternion_round_trip():
    """Planar yaw converts to the expected odometry quaternion and back."""
    yaw = -math.pi / 3.0
    z, w = quaternion_z_w(yaw)
    assert quaternion_yaw(0.0, 0.0, z, w) == pytest.approx(yaw)


def test_route_arrays_must_have_equal_lengths():
    """Mismatched route arrays are rejected before navigation."""
    with pytest.raises(ValueError):
        validate_route([1, 2], [0.0], [0.0, 1.0], [0.0, 0.0])


def test_two_routes_must_share_node_and_coordinates():
    """The approach end and delivery start must describe the same Node."""
    approach = ((1, 2), ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)))
    delivery = ((2, 3), ((1.1, 0.0, 0.0), (2.0, 0.0, 0.0)))
    with pytest.raises(ValueError, match='coordinates differ'):
        validate_mission_routes(approach, delivery)


def test_reverse_distance_uses_planar_odometry_displacement():
    """Return completion uses measured planar displacement."""
    assert traveled_distance((1.0, 2.0), (-0.8, -0.4)) == pytest.approx(3.0)
