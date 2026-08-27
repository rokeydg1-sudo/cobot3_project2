#!/usr/bin/env python3
"""Run the production FMS cuOpt adapter on a deterministic scene graph."""

from fms.NodeMapGraph import EdgeData, NodeData, NodeMapGraphManager
from fms.TaskManager import AMRState, OptimizationRequest, TaskManager
from fms.cuopt_solver import CuOptSolver


NODE_XY = {
    1: (-42.453572, 0.0),
    8: (-27.495022, 6.610057),
    9: (-27.565342, -7.805492),
    10: (-27.846621, 22.502320),
    11: (-27.354383, -20.814646),
    12: (-42.191850, 22.432000),
    13: (-42.332490, -20.884966),
}

EDGE_PAIRS = (
    (8, 9),
    (10, 8),
    (12, 10),
    (1, 12),
    (13, 1),
    (11, 13),
    (9, 11),
)


def main():
    nodes = {
        node_id: NodeData(
            node_id=node_id,
            name=f"Node_{node_id}",
            node_type="WAYPOINT",
            x=xy[0],
            y=xy[1],
            z=0.15,
        )
        for node_id, xy in NODE_XY.items()
    }
    edges = []
    for edge_id, (start, end) in enumerate(EDGE_PAIRS):
        start_node = nodes[start]
        end_node = nodes[end]
        weight = (
            (start_node.x - end_node.x) ** 2
            + (start_node.y - end_node.y) ** 2
        ) ** 0.5
        edges.append(
            EdgeData(
                edge_id=edge_id,
                start=start,
                end=end,
                weight=weight,
                bidirectional=True,
            )
        )

    graph = NodeMapGraphManager()
    graph.update_nodemap(nodes, edges, revision=1)

    task_manager = TaskManager()
    task_manager.create_task(nodes[10], nodes[11], task_id="task_node10_to_11")
    task_manager.create_task(nodes[12], nodes[13], task_id="task_node12_to_13")
    request = OptimizationRequest(
        tasks=task_manager.get_waiting_tasks(),
        amr_state=AMRState(
            amr_id="amr1",
            state="IDLE",
            x=-30.360045,
            y=17.247943,
            yaw=0.0,
            load_state="EMPTY",
        ),
    )

    solver = CuOptSolver(request, graph, time_limit=2.0)
    result = solver.solve()
    print("PASS: cuOpt production adapter")
    print(f"recovery_node={result.recovery_node_id}")
    print(
        "task_order="
        + ",".join(task.task_id for task in result.ordered_tasks)
    )
    for ordered_task in result.ordered_tasks:
        print(
            f"route[{ordered_task.task_id}]="
            + "->".join(map(str, ordered_task.route.node_ids))
        )
    print(f"total_cost={result.total_cost:.6f}")


if __name__ == "__main__":
    main()
