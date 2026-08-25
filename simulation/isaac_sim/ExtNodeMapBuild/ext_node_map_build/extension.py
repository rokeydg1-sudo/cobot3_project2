"""현재 Isaac Sim Stage의 NodeMap을 ROS 2 Service로 제공한다."""

import carb
import omni.ext
import omni.kit.app
import omni.usd
import rclpy

from interfaces.msg import NodeMapChanged
from interfaces.srv import GetNodeMap
from pxr import Usd, UsdGeom


# NODE_ROOT = "/World/Node/WaypointGraph/Nodes"
# EDGE_ROOT = "/World/Node/WaypointGraph/Edges"

NODE_ROOT = "/World/WaypointGraph/Nodes"
EDGE_ROOT = "/World/WaypointGraph/Edges"

SERVICE_NAME = "/get_node_map"
MAP_CHANGED_TOPIC = "/node_map_changed"


class ExtNodeMapBuild(omni.ext.IExt):

    def on_startup(self, ext_id: str) -> None:
        self._ext_id = ext_id
        self._nodes = {}
        self._edges = []
        self._revision = 0

        self._ros_node = None
        self._service = None
        self._map_changed_publisher = None
        self._update_subscription = None
        self._stage_subscription = None
        self._owns_rclpy = False

        carb.log_info("[ExtNodeMapBuild] Extension started")

        # 현재 열린 Stage에서 NodeMap을 읽는다.
        self._load_current_node_map()
        if self._nodes:
            self._revision = 1

        # ROS 2 Service Server를 생성한다.
        self._setup_ros_service()

        # 새 USD Stage가 열리면 NodeMap을 갱신하고 FMS에 알린다.
        self._setup_stage_events()

    def _setup_ros_service(self) -> None:
        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_rclpy = True

        self._ros_node = rclpy.create_node(
            "ext_node_map_build"
        )

        self._service = self._ros_node.create_service(
            GetNodeMap,
            SERVICE_NAME,
            self._handle_get_node_map,
        )

        self._map_changed_publisher = self._ros_node.create_publisher(
            NodeMapChanged,
            MAP_CHANGED_TOPIC,
            10,
        )

        # Isaac Sim의 기존 메인 루프에 callback을 연결한다.
        update_stream = (
            omni.kit.app.get_app()
            .get_update_event_stream()
        )

        self._update_subscription = (
            update_stream.create_subscription_to_pop(
                self._on_update,
                name="ExtNodeMapBuild ROS 2 spin",
            )
        )

        carb.log_info(
            f"[ExtNodeMapBuild] Service ready: {SERVICE_NAME}"
        )

    def _setup_stage_events(self) -> None:
        stage_stream = omni.usd.get_context().get_stage_event_stream()
        self._stage_subscription = stage_stream.create_subscription_to_pop(
            self._on_stage_event,
            name="ExtNodeMapBuild Stage events",
        )

    def _on_stage_event(self, event) -> None:
        """새 Stage가 열린 경우 확정된 NodeMap 정보를 FMS에 알린다."""
        if event.type != int(omni.usd.StageEventType.OPENED):
            return

        self._load_current_node_map()
        if not self._nodes:
            carb.log_warn(
                "[ExtNodeMapBuild] Opened Stage contains no NodeMap"
            )
            return

        self._revision += 1
        self._publish_node_map_changed()

    def _publish_node_map_changed(self) -> None:
        if self._map_changed_publisher is None:
            return

        stage = omni.usd.get_context().get_stage()
        root_layer = stage.GetRootLayer() if stage is not None else None

        message = NodeMapChanged()
        message.revision = self._revision
        message.stage_identifier = (
            root_layer.identifier if root_layer is not None else ""
        )
        message.node_count = len(self._nodes)
        message.edge_count = len(self._edges)
        self._map_changed_publisher.publish(message)

        carb.log_info(
            f"[ExtNodeMapBuild] NodeMap changed: "
            f"revision={self._revision}, "
            f"Nodes={message.node_count}, Edges={message.edge_count}"
        )

    def _on_update(self, _event) -> None:
        """Isaac Sim의 각 update에서 ROS callback을 한 번 처리한다."""
        if self._ros_node is not None and rclpy.ok():
            rclpy.spin_once(
                self._ros_node,
                timeout_sec=0.0,
            )

    def _handle_get_node_map(
        self,
        _request: GetNodeMap.Request,
        response: GetNodeMap.Response,
    ) -> GetNodeMap.Response:
        """현재 Runtime NodeMap을 FMS에 반환한다."""
        # USD가 Extension보다 늦게 열렸거나 변경됐을 수 있으므로
        # Service 요청 시 현재 Stage를 다시 읽는다.
        self._load_current_node_map()

        if not self._nodes:
            response.success = False
            response.message = "NodeMap contains no nodes."
            response.revision = self._revision
            return response

        for node_id in sorted(self._nodes):
            node = self._nodes[node_id]
            x, y, z = node["position"]

            response.node_ids.append(node_id)
            response.node_names.append(node["name"])
            response.node_types.append(node["type"])
            response.node_x.append(x)
            response.node_y.append(y)
            response.node_z.append(z)

        for edge in self._edges:
            response.edge_from.append(edge["start"])
            response.edge_to.append(edge["end"])
            response.edge_weights.append(edge["weight"])
            response.edge_bidirectional.append(
                edge["bidirectional"]
            )

        response.success = True
        response.revision = self._revision
        response.message = (
            f"Nodes={len(self._nodes)}, "
            f"Edges={len(self._edges)}"
        )

        carb.log_info(
            f"[ExtNodeMapBuild] NodeMap response: "
            f"{response.message}"
        )

        return response

    def _load_current_node_map(self) -> None:
        self._nodes = {}
        self._edges = []

        stage = omni.usd.get_context().get_stage()

        if stage is None:
            carb.log_warn(
                "[ExtNodeMapBuild] No Stage is open"
            )
            return

        node_root = stage.GetPrimAtPath(NODE_ROOT)
        edge_root = stage.GetPrimAtPath(EDGE_ROOT)

        if not node_root.IsValid():
            carb.log_warn(
                f"[ExtNodeMapBuild] "
                f"Node root not found: {NODE_ROOT}"
            )
            return

        if not edge_root.IsValid():
            carb.log_warn(
                f"[ExtNodeMapBuild] "
                f"Edge root not found: {EDGE_ROOT}"
            )
            return

        self._nodes = self._load_nodes(node_root)
        self._edges = self._load_edges(edge_root)

        carb.log_info(
            f"[ExtNodeMapBuild] "
            f"Nodes={len(self._nodes)}, "
            f"Edges={len(self._edges)}"
        )

    @staticmethod
    def _parse_node_id_from_name(name: str) -> int | None:
        """Node_<id> 형식의 Prim 이름에서 Node ID를 추출한다."""
        prefix = "Node_"
        if not name.startswith(prefix):
            return None

        node_id = name[len(prefix):]
        return int(node_id) if node_id.isdigit() else None

    @staticmethod
    def _parse_edge_nodes_from_name(name: str) -> tuple[int, int] | None:
        """Edge_<start>_<end> 형식의 Prim 이름에서 양 끝 Node를 추출한다."""
        parts = name.split("_")
        if (
            len(parts) != 3
            or parts[0] != "Edge"
            or not parts[1].isdigit()
            or not parts[2].isdigit()
        ):
            return None

        return int(parts[1]), int(parts[2])

    def _load_nodes(self, node_root) -> dict:
        nodes = {}

        transform_cache = UsdGeom.XformCache(
            Usd.TimeCode.Default()
        )

        for prim in node_root.GetChildren():
            prim_name = prim.GetName()
            node_id_attr = prim.GetAttribute("node_id")
            node_name_attr = prim.GetAttribute("node_name")
            node_type_attr = prim.GetAttribute("node_type")

            node_id_value = (
                node_id_attr.Get()
                if node_id_attr.IsValid()
                else None
            )
            node_id = (
                int(node_id_value)
                if node_id_value is not None
                else self._parse_node_id_from_name(prim_name)
            )

            if node_id is None:
                carb.log_warn(
                    f"[ExtNodeMapBuild] Invalid Node name: {prim_name}"
                )
                continue

            node_name = (
                node_name_attr.Get()
                if node_name_attr.IsValid() and node_name_attr.Get() is not None
                else prim_name
            )

            node_type = (
                node_type_attr.Get()
                if node_type_attr.IsValid() and node_type_attr.Get() is not None
                else "WAYPOINT"
            )

            matrix = transform_cache.GetLocalToWorldTransform(
                prim
            )
            position = matrix.ExtractTranslation()

            nodes[node_id] = {
                "name": str(node_name),
                "type": str(node_type),
                "position": (
                    float(position[0]),
                    float(position[1]),
                    float(position[2]),
                ),
            }

        return nodes

    def _load_edges(self, edge_root) -> list:
        edges = []

        for prim in edge_root.GetChildren():
            prim_name = prim.GetName()
            start_attr = prim.GetAttribute("from_node")
            end_attr = prim.GetAttribute("to_node")
            weight_attr = prim.GetAttribute("weight")
            bidirectional_attr = prim.GetAttribute(
                "bidirectional"
            )

            start_value = start_attr.Get() if start_attr.IsValid() else None
            end_value = end_attr.Get() if end_attr.IsValid() else None

            if start_value is not None and end_value is not None:
                start = int(start_value)
                end = int(end_value)
            else:
                parsed_nodes = self._parse_edge_nodes_from_name(prim_name)
                if parsed_nodes is None:
                    carb.log_warn(
                        f"[ExtNodeMapBuild] Invalid Edge name: {prim_name}"
                    )
                    continue
                start, end = parsed_nodes

            if start not in self._nodes or end not in self._nodes:
                carb.log_warn(
                    f"[ExtNodeMapBuild] Edge references unknown Node: "
                    f"{start} -> {end}"
                )
                continue

            weight_value = (
                weight_attr.Get()
                if weight_attr.IsValid()
                else None
            )
            if weight_value is None:
                start_position = self._nodes[start]["position"]
                end_position = self._nodes[end]["position"]
                weight_value = sum(
                    (end_position[index] - start_position[index]) ** 2
                    for index in range(3)
                ) ** 0.5

            bidirectional_value = (
                bidirectional_attr.Get()
                if bidirectional_attr.IsValid()
                else None
            )

            edges.append(
                {
                    "start": start,
                    "end": end,
                    "weight": float(weight_value),
                    "bidirectional": bool(bidirectional_value)
                    if bidirectional_value is not None
                    else True,
                }
            )

        return edges

    def on_shutdown(self) -> None:
        carb.log_info(
            "[ExtNodeMapBuild] Extension stopping"
        )

        # update callback을 먼저 해제해 추가 spin을 막는다.
        self._update_subscription = None
        self._stage_subscription = None

        if self._ros_node is not None:
            if self._service is not None:
                self._ros_node.destroy_service(
                    self._service
                )

            if self._map_changed_publisher is not None:
                self._ros_node.destroy_publisher(
                    self._map_changed_publisher
                )

            self._ros_node.destroy_node()

        self._service = None
        self._map_changed_publisher = None
        self._ros_node = None

        # 이 Extension이 rclpy를 초기화한 경우에만 종료한다.
        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()

        self._nodes = {}
        self._edges = []
        self._revision = 0
        self._ext_id = None

        carb.log_info(
            "[ExtNodeMapBuild] Extension stopped"
        )
