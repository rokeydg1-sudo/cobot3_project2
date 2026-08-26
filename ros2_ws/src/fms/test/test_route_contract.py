"""Tests for approach and delivery NodeMap route contracts."""

from types import SimpleNamespace

import pytest

from fms.NodeMapGraph import EdgeData, NodeData, NodeMapGraphManager
from fms.route_contract import (
    populate_response_routes,
    validate_mission_routes,
)


def make_graph():
    """Create a small deterministic NodeMap for route tests."""
    graph = NodeMapGraphManager()
    nodes = {
        0: NodeData(0, '0', 'junction', 0.0, 0.0, 0.0),
        1: NodeData(1, '1', 'junction', 1.0, 0.0, 0.0),
        2: NodeData(2, '2', 'junction', 2.0, 0.0, 0.0),
        3: NodeData(3, '3', 'junction', 1.0, 1.0, 0.0),
        4: NodeData(4, '4', 'junction', 2.0, 1.0, 0.0),
    }
    edges = [
        EdgeData(0, 0, 1, 1.0),
        EdgeData(1, 1, 2, 1.0),
        EdgeData(2, 1, 3, 1.0),
        EdgeData(3, 3, 4, 1.0),
    ]
    graph.update_nodemap(nodes, edges, 7)
    return graph


def test_shortest_route_uses_existing_node_map_graph():
    """The existing shortest-path implementation supplies Node routes."""
    route = make_graph().create_route(0, 4)
    assert route.node_ids == (0, 1, 3, 4)
    assert route.total_cost == pytest.approx(3.0)
    assert len(route.node_ids) == len(route.points)


def test_approach_and_delivery_route_endpoints_and_costs():
    """Both routes meet at Task.start and delivery ends at Task.goal."""
    graph = make_graph()
    approach = graph.create_route(0, 3)
    delivery = graph.create_route(3, 2)
    approach_fields, delivery_fields = validate_mission_routes(
        approach,
        delivery,
        task_start_node_id=3,
        task_goal_node_id=2,
    )
    assert approach_fields.node_ids == (0, 1, 3)
    assert approach_fields.total_cost == pytest.approx(2.0)
    assert delivery_fields.node_ids == (3, 1, 2)
    assert delivery_fields.total_cost == pytest.approx(2.0)


def test_response_mapping_keeps_parallel_arrays_consistent():
    """The response mapping preserves all Node/XYZ array lengths."""
    graph = make_graph()
    response = SimpleNamespace()
    approach, delivery = populate_response_routes(
        response,
        graph.create_route(0, 3),
        graph.create_route(3, 2),
        task_start_node_id=3,
        task_goal_node_id=2,
    )
    assert response.approach_route_node_ids[-1] == 3
    assert response.route_node_ids[0] == 3
    assert response.route_node_ids[-1] == 2
    approach_size = len(response.approach_route_node_ids)
    delivery_size = len(response.route_node_ids)
    assert approach_size == len(response.approach_route_x)
    assert approach_size == len(response.approach_route_y)
    assert approach_size == len(response.approach_route_z)
    assert delivery_size == len(response.route_x)
    assert delivery_size == len(response.route_y)
    assert delivery_size == len(response.route_z)
    assert response.approach_route_total_cost == approach.total_cost
    assert response.route_total_cost == delivery.total_cost


def test_route_endpoint_mismatch_is_rejected():
    """A delivery route that does not start at Task.start is rejected."""
    graph = make_graph()
    with pytest.raises(ValueError, match='Delivery route must start'):
        validate_mission_routes(
            graph.create_route(0, 3),
            graph.create_route(1, 2),
            task_start_node_id=3,
            task_goal_node_id=2,
        )


def test_cuopt_gpu_runtime_is_explicitly_skipped():
    """GPU solver execution is outside this host-only unit suite."""
    pytest.skip('SKIP: cuOpt GPU runtime unavailable')
