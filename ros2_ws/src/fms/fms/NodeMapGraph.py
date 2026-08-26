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
        
        node_ids = sorted(nodes)
        node_index = {node_id: index for index, node_id in enumerate(node_ids)}

        active_edges: list[EdgeData] = []
        active_neighbors: dict[int, list[NeighborData]] = {
            node_id: [] for node_id in node_ids
        }

        for edge_index, edge in enumerate(edges):
            start_node = nodes[edge.start]
            end_node = nodes[edge.end]

            if (
                not edge.available
                or not start_node.available
                or not end_node.available
            ):
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
