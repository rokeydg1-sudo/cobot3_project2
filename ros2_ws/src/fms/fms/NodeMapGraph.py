import heapq
import random
from dataclasses import dataclass, field

import numpy as np


@dataclass
class NodeData:
    node_id: int
    name: str
    node_type: str
    x: float
    y: float
    z: float
    available: bool = True


@dataclass
class EdgeData:
    edge_id: int
    start: int
    end: int
    weight: float
    bidirectional: bool = True
    available: bool = True
    path_points: list[tuple[float, float, float]] = field(
        default_factory=list
    )


@dataclass(frozen=True)
class NeighborData:
    node_id: int
    edge_index: int



# CSR 그래프 데이터 구조 (cuOpt에 실제로 들어가는 데이터 형태)
@dataclass
class CsrGraph:
    node_ids: np.ndarray
    offsets: np.ndarray
    indices: np.ndarray
    weights: np.ndarray


@dataclass(frozen=True)
class PlannedNodeRoute:
    start_node_id: int
    goal_node_id: int
    node_ids: tuple[int, ...]
    points: tuple[tuple[float, float, float], ...]
    total_cost: float






class NodeMapGraphManager:
    def __init__(self) -> None:
        self.nodes: dict[int, NodeData] = {}
        self.edges: list[EdgeData] = []
        self.active_edges: list[EdgeData] = []
        self.active_neighbors: dict[int, list[NeighborData]] = {}
        self.csr = CsrGraph(
            node_ids=np.array([], dtype=np.int32),
            offsets=np.array([0], dtype=np.int32),
            indices=np.array([], dtype=np.int32),
            weights=np.array([], dtype=np.float32),
        )
        self.revision = 0


    # 모든 노드와 엣지의 상태를 확인해서 활성노드맵 구성
    def update_nodemap(
        self,
        nodes: dict[int, NodeData],
        edges: list[EdgeData],
        revision: int,
    ) -> None:
        
        node_ids = sorted(
            node_id for node_id, node in nodes.items() if node.available
        )
        node_index = {node_id: index for index, node_id in enumerate(node_ids)}

        active_edges: list[EdgeData] = []
        active_neighbors: dict[int, list[NeighborData]] = {
            node_id: [] for node_id in node_ids
        }

        for edge_index, edge in enumerate(edges):
            if not edge.available:
                continue

            start_node = nodes[edge.start]
            end_node = nodes[edge.end]

            if not start_node.available or not end_node.available:
                continue

            active_edges.append(edge)
            active_neighbors[edge.start].append(
                NeighborData(edge.end, edge_index)
            )

            if edge.bidirectional and edge.start != edge.end:
                active_neighbors[edge.end].append(
                    NeighborData(edge.start, edge_index)
                )

        offsets = [0]
        indices: list[int] = []
        weights: list[float] = []

        for node_id in node_ids:
            neighbors = active_neighbors[node_id]
            neighbors.sort(
                key=lambda neighbor: (
                    neighbor.node_id,
                    neighbor.edge_index,
                )
            )

            for neighbor in neighbors:
                edge = edges[neighbor.edge_index]
                indices.append(node_index[neighbor.node_id])
                weights.append(float(edge.weight))

            offsets.append(len(indices))

        csr = CsrGraph(
            node_ids=np.asarray(node_ids, dtype=np.int32),
            offsets=np.asarray(offsets, dtype=np.int32),
            indices=np.asarray(indices, dtype=np.int32),
            weights=np.asarray(weights, dtype=np.float32),
        )

        self.nodes = dict(nodes)
        self.edges = list(edges)
        self.active_edges = active_edges
        self.active_neighbors = active_neighbors
        self.csr = csr
        self.revision = int(revision)

    def get_active_edges(self) -> list[EdgeData]:
        return self.active_edges

    def get_csr(self) -> CsrGraph:
        return self.csr

    def create_random_route(self) -> PlannedNodeRoute:
        """Select two reachable active nodes and return their shortest route."""
        return self.create_route(*self.choose_random_reachable_nodes())

    def choose_random_reachable_nodes(self) -> tuple[int, int]:
        """검증 작업에 사용할 서로 연결된 활성 Node 두 개를 선택한다."""
        reachable_pairs: list[tuple[int, int]] = []

        for start_node_id, neighbors in self.active_neighbors.items():
            if not neighbors:
                continue

            reachable = self._find_reachable_nodes(start_node_id)
            reachable_pairs.extend(
                (start_node_id, goal_node_id)
                for goal_node_id in reachable
                if goal_node_id != start_node_id
            )

        if not reachable_pairs:
            raise ValueError(
                "NodeMap has no reachable pair of different available nodes."
            )

        return random.choice(reachable_pairs)

    def find_nearest_active_node(self, x: float, y: float) -> NodeData:
        """현재 물리 좌표에서 가장 가까운 활성 복귀 Node를 반환한다."""
        if not self.csr.node_ids.size:
            raise ValueError("NodeMap has no active nodes.")

        return min(
            (self.nodes[int(node_id)] for node_id in self.csr.node_ids),
            key=lambda node: (node.x - x) ** 2 + (node.y - y) ** 2,
        )

    def create_route(
        self,
        start_node_id: int,
        goal_node_id: int,
    ) -> PlannedNodeRoute:
        node_ids, total_cost = self.find_shortest_path(
            start_node_id,
            goal_node_id,
        )
        points = tuple(
            (self.nodes[node_id].x, self.nodes[node_id].y, self.nodes[node_id].z)
            for node_id in node_ids
        )

        return PlannedNodeRoute(
            start_node_id=start_node_id,
            goal_node_id=goal_node_id,
            node_ids=node_ids,
            points=points,
            total_cost=total_cost,
        )

    def find_shortest_path(
        self,
        start_node_id: int,
        goal_node_id: int,
    ) -> tuple[tuple[int, ...], float]:
        if start_node_id not in self.active_neighbors:
            raise ValueError(f"Start Node is not active: {start_node_id}")
        if goal_node_id not in self.active_neighbors:
            raise ValueError(f"Goal Node is not active: {goal_node_id}")

        node_ids = [int(node_id) for node_id in self.csr.node_ids]
        node_index = {node_id: index for index, node_id in enumerate(node_ids)}
        distances = {start_node_id: 0.0}
        previous: dict[int, int] = {}
        queue = [(0.0, start_node_id)]

        while queue:
            distance, node_id = heapq.heappop(queue)
            if distance > distances[node_id]:
                continue
            if node_id == goal_node_id:
                break

            index = node_index[node_id]
            begin = int(self.csr.offsets[index])
            end = int(self.csr.offsets[index + 1])

            for position in range(begin, end):
                neighbor_id = node_ids[int(self.csr.indices[position])]
                weight = float(self.csr.weights[position])
                if weight < 0:
                    raise ValueError("CSR edge weight must be non-negative.")

                next_distance = distance + weight
                if next_distance >= distances.get(neighbor_id, float("inf")):
                    continue

                distances[neighbor_id] = next_distance
                previous[neighbor_id] = node_id
                heapq.heappush(queue, (next_distance, neighbor_id))

        if goal_node_id not in distances:
            raise ValueError(
                f"No active path exists: {start_node_id} -> {goal_node_id}"
            )

        path = [goal_node_id]
        while path[-1] != start_node_id:
            path.append(previous[path[-1]])
        path.reverse()

        return tuple(path), distances[goal_node_id]

    def _find_reachable_nodes(self, start_node_id: int) -> set[int]:
        reachable = {start_node_id}
        pending = [start_node_id]

        while pending:
            node_id = pending.pop()
            for neighbor in self.active_neighbors[node_id]:
                if neighbor.node_id in reachable:
                    continue
                reachable.add(neighbor.node_id)
                pending.append(neighbor.node_id)

        return reachable
