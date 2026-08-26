"""Pure validation and response mapping for FMS mission routes."""

from dataclasses import dataclass


@dataclass(frozen=True)
class RouteFields:
    """Normalized parallel arrays for one planned NodeMap route."""

    node_ids: tuple[int, ...]
    x: tuple[float, ...]
    y: tuple[float, ...]
    z: tuple[float, ...]
    total_cost: float


def route_fields(route) -> RouteFields:
    """Normalize a PlannedNodeRoute-like object and validate its arrays."""
    node_ids = tuple(int(node_id) for node_id in route.node_ids)
    points = tuple(
        tuple(float(value) for value in point)
        for point in route.points
    )

    if not node_ids:
        raise ValueError('A mission route must contain at least one Node.')
    if len(node_ids) != len(points):
        raise ValueError('Route Node and point arrays have different lengths.')
    if any(len(point) != 3 for point in points):
        raise ValueError('Every route point must contain XYZ coordinates.')

    return RouteFields(
        node_ids=node_ids,
        x=tuple(point[0] for point in points),
        y=tuple(point[1] for point in points),
        z=tuple(point[2] for point in points),
        total_cost=float(route.total_cost),
    )


def validate_mission_routes(
    approach_route,
    delivery_route,
    task_start_node_id: int,
    task_goal_node_id: int,
) -> tuple[RouteFields, RouteFields]:
    """Validate that approach and delivery routes meet at Task.start."""
    approach = route_fields(approach_route)
    delivery = route_fields(delivery_route)

    if approach.node_ids[-1] != int(task_start_node_id):
        raise ValueError('Approach route must end at Task.start.')
    if delivery.node_ids[0] != int(task_start_node_id):
        raise ValueError('Delivery route must start at Task.start.')
    if delivery.node_ids[-1] != int(task_goal_node_id):
        raise ValueError('Delivery route must end at Task.goal.')

    return approach, delivery


def populate_response_routes(
    response,
    approach_route,
    delivery_route,
    task_start_node_id: int,
    task_goal_node_id: int,
) -> tuple[RouteFields, RouteFields]:
    """Map validated mission routes onto a RequestTask response."""
    approach, delivery = validate_mission_routes(
        approach_route,
        delivery_route,
        task_start_node_id,
        task_goal_node_id,
    )

    response.approach_route_node_ids = list(approach.node_ids)
    response.approach_route_x = list(approach.x)
    response.approach_route_y = list(approach.y)
    response.approach_route_z = list(approach.z)
    response.approach_route_total_cost = approach.total_cost

    response.route_node_ids = list(delivery.node_ids)
    response.route_x = list(delivery.x)
    response.route_y = list(delivery.y)
    response.route_z = list(delivery.z)
    response.route_total_cost = delivery.total_cost

    return approach, delivery
