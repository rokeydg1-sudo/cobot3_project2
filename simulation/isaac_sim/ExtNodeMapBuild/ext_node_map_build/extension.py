"""Isaac Sim Stage의 NodeMap을 ROS 2로 제공하고 AMR Step Planned Path를 시각화한다."""

from __future__ import annotations

import json
from typing import TYPE_CHECKING

import carb
import omni.ext
import omni.kit.app
import omni.ui as ui
import omni.usd
import rclpy

from omni.kit.viewport.utility import get_active_viewport_window
from omni.ui import color as cl
from omni.ui import scene as sc
from pxr import Gf, Usd, UsdGeom
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

if TYPE_CHECKING:
    from interfaces.action import VisualizeRoute
    from interfaces.srv import GetNodeMap
    from rclpy.action import CancelResponse, GoalResponse


# ---------------------------------------------------------------------------
# NodeMap Stage layout
# ---------------------------------------------------------------------------

# NODE_ROOT = "/World/Node/WaypointGraph/Nodes"
# EDGE_ROOT = "/World/Node/WaypointGraph/Edges"

NODE_ROOT = "/World/WaypointGraph/Nodes"
EDGE_ROOT = "/World/WaypointGraph/Edges"


# ---------------------------------------------------------------------------
# ROS 2 interfaces
# ---------------------------------------------------------------------------

SERVICE_NAME = "/get_node_map"
MAP_CHANGED_TOPIC = "/node_map_changed"
VISUALIZE_ROUTE_ACTION = "/visualize_route"
NODE_MAP_BRIDGE_TOPIC = "/cobot3/runtime/node_map"
ROUTE_REQUEST_BRIDGE_TOPIC = "/cobot3/runtime/visualize_route/request"
ROUTE_RESPONSE_BRIDGE_TOPIC = "/cobot3/runtime/visualize_route/response"
EXTENSION_RUNTIME_READY = False


# ---------------------------------------------------------------------------
# Planned Path visualization
# ---------------------------------------------------------------------------

PLANNED_COLOR = Gf.Vec3f(1.0, 1.0, 0.0)  # Yellow

SHOW_ROUTE_ORDER_LABELS = True
ROUTE_LABEL_Z_OFFSET = 0.35
ROUTE_LABEL_FONT_SIZE = 28


class ExtNodeMapBuild(omni.ext.IExt):
    """NodeMap 제공 + AMR Step Planned Path 시각화 Extension."""

    def on_startup(self, ext_id: str) -> None:
        global EXTENSION_RUNTIME_READY
        EXTENSION_RUNTIME_READY = False
        self._ext_id = ext_id

        # Runtime NodeMap
        self._nodes = {}
        self._edges = []
        self._revision = 0

        # Isaac Sim visualization lookup
        self._node_prim_lookup = {}
        self._edge_prim_lookup = {}
        self._original_display_colors = {}
        self._visualized_route = []

        # Viewport route-order labels
        self._viewport_window = None
        self._route_label_scene_view = None

        # ROS 2
        self._ros_node = None
        self._service = None
        self._map_changed_publisher = None
        self._visualize_route_action_server = None
        self._node_map_bridge_publisher = None
        self._route_request_subscription = None
        self._route_response_publisher = None
        self._update_subscription = None
        self._stage_subscription = None
        self._owns_rclpy = False

        self._visualization_busy = False

        carb.log_info("[ExtNodeMapBuild] Extension started")

        # 현재 열린 Stage에서 NodeMap을 읽는다.
        self._load_current_node_map()
        if self._nodes:
            self._revision = 1

        self._setup_ros()
        self._setup_stage_events()
        EXTENSION_RUNTIME_READY = True

    # ------------------------------------------------------------------
    # ROS 2 setup
    # ------------------------------------------------------------------

    def _setup_ros(self) -> None:
        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_rclpy = True

        self._ros_node = rclpy.create_node("ext_node_map_build")

        # Isaac 5.1 bundles an older Fast DDS/Fast-CDR ABI than the host's
        # current Jazzy installation. Keep Stage ownership here while using
        # standard String messages across the private adapter boundary.
        bridge_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._node_map_bridge_publisher = self._ros_node.create_publisher(
            String,
            NODE_MAP_BRIDGE_TOPIC,
            bridge_qos,
        )
        self._route_request_subscription = self._ros_node.create_subscription(
            String,
            ROUTE_REQUEST_BRIDGE_TOPIC,
            self._handle_route_bridge_request,
            10,
        )
        self._route_response_publisher = self._ros_node.create_publisher(
            String,
            ROUTE_RESPONSE_BRIDGE_TOPIC,
            10,
        )

        # Isaac Sim main update loop에서 ROS callback 처리
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
            f"[ExtNodeMapBuild] NodeMap bridge ready: {NODE_MAP_BRIDGE_TOPIC}"
        )
        carb.log_info(
            "[ExtNodeMapBuild] Route visualization bridge ready: "
            f"{ROUTE_REQUEST_BRIDGE_TOPIC}"
        )

    def _on_update(self, _event) -> None:
        if self._ros_node is not None and rclpy.ok():
            rclpy.spin_once(self._ros_node, timeout_sec=0.0)

    # ------------------------------------------------------------------
    # Stage events
    # ------------------------------------------------------------------

    def _setup_stage_events(self) -> None:
        stage_stream = omni.usd.get_context().get_stage_event_stream()
        self._stage_subscription = stage_stream.create_subscription_to_pop(
            self._on_stage_event,
            name="ExtNodeMapBuild Stage events",
        )

    def _on_stage_event(self, event) -> None:
        """새 Stage가 열리면 기존 캐시/시각화를 폐기하고 NodeMap을 재구성한다."""
        if event.type != int(omni.usd.StageEventType.OPENED):
            return

        # 이전 Stage 시각화 상태 제거
        self._original_display_colors = {}
        self._visualized_route = []
        self._clear_route_labels()

        self._load_current_node_map()

        if not self._nodes:
            carb.log_warn(
                "[ExtNodeMapBuild] Opened Stage contains no NodeMap"
            )
            return

        self._revision += 1
        self._publish_node_map_changed()
        self._publish_runtime_node_map()

    # ------------------------------------------------------------------
    # Existing NodeMap ROS interfaces
    # ------------------------------------------------------------------

    def _publish_node_map_changed(self) -> None:
        """Compatibility no-op; the system adapter publishes the public event."""

    def _publish_runtime_node_map(self) -> None:
        if self._node_map_bridge_publisher is None or not self._nodes:
            return
        stage = omni.usd.get_context().get_stage()
        root_layer = stage.GetRootLayer() if stage is not None else None
        payload = {
            "revision": self._revision,
            "stage_identifier": (
                root_layer.identifier if root_layer is not None else ""
            ),
            "nodes": [
                {"id": node_id, **self._nodes[node_id]}
                for node_id in sorted(self._nodes)
            ],
            "edges": self._edges,
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self._node_map_bridge_publisher.publish(message)
        carb.log_info(
            f"[ExtNodeMapBuild] NodeMap bridge published: "
            f"revision={self._revision}, Nodes={len(self._nodes)}, "
            f"Edges={len(self._edges)}"
        )

    def _handle_route_bridge_request(self, message: String) -> None:
        response = {"request_id": "", "success": False, "message": ""}
        try:
            request = json.loads(message.data)
            response["request_id"] = str(request.get("request_id", ""))
            route = [int(node_id) for node_id in request.get("node_ids", [])]
            error = self._validate_visualize_request(
                int(request.get("node_map_revision", -1)),
                route,
            )
            if error is not None:
                raise ValueError(error)

            self._reset_path_visualization()
            self._visualized_route = route
            for node_id in route:
                if not self._set_prim_color(
                    self._node_prim_lookup[node_id], PLANNED_COLOR
                ):
                    raise RuntimeError(f"Failed to highlight Node {node_id}")
            for start, end in zip(route, route[1:]):
                if not self._set_prim_color(
                    self._edge_prim_lookup[(start, end)], PLANNED_COLOR
                ):
                    raise RuntimeError(
                        f"Failed to highlight Edge {start} -> {end}"
                    )
            label_warning = ""
            if SHOW_ROUTE_ORDER_LABELS and not self._draw_route_order_labels(route):
                label_warning = " Route order labels were unavailable."
            response["success"] = True
            response["message"] = (
                f"Visualized AMR={request.get('amr_id', '')}, "
                f"Task={request.get('task_id', '')}, Nodes={route}."
                f"{label_warning}"
            )
        except Exception as exc:
            self._reset_path_visualization()
            response["message"] = f"Visualization failed: {exc}"
            carb.log_error(f"[ExtNodeMapBuild] {response['message']}")

        bridge_response = String()
        bridge_response.data = json.dumps(response, separators=(",", ":"))
        self._route_response_publisher.publish(bridge_response)

    def _handle_get_node_map(
        self,
        _request: GetNodeMap.Request,
        response: GetNodeMap.Response,
    ) -> GetNodeMap.Response:
        """현재 Runtime NodeMap을 FMS에 반환한다."""

        # 기존 코드 정책 유지:
        # USD가 Extension보다 늦게 열렸거나 변경됐을 수 있으므로 요청 시 다시 읽는다.
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
            response.edge_bidirectional.append(edge["bidirectional"])

        response.success = True
        response.revision = self._revision
        response.message = (
            f"Nodes={len(self._nodes)}, Edges={len(self._edges)}"
        )

        carb.log_info(
            f"[ExtNodeMapBuild] NodeMap response: {response.message}"
        )
        return response

    # ------------------------------------------------------------------
    # VisualizeRoute Action
    # ------------------------------------------------------------------

    def _visualize_route_goal_callback(
        self,
        goal_request: VisualizeRoute.Goal,
    ) -> GoalResponse:
        """Action Goal 수락 여부.

        Action은 주행 전체가 아니라 '시각화 처리'의 생명주기만 관리한다.
        """
        if self._visualization_busy:
            carb.log_warn(
                "[ExtNodeMapBuild] VisualizeRoute rejected: "
                "another visualization request is being processed"
            )
            return GoalResponse.REJECT

        # 상세 검증은 execute callback에서 수행하여 Result message를 반환한다.
        return GoalResponse.ACCEPT

    def _visualize_route_cancel_callback(self, _goal_handle) -> CancelResponse:
        """시각화 처리 중 취소 요청을 허용한다."""
        return CancelResponse.ACCEPT

    def _execute_visualize_route(self, goal_handle):
        """현재 Step Planned Path를 NodeMap 위에 표시한다."""
        self._visualization_busy = True
        result = VisualizeRoute.Result()

        try:
            request = goal_handle.request
            route = [int(node_id) for node_id in request.node_ids]

            self._publish_visualize_feedback(
                goal_handle,
                "validating",
            )

            # ----------------------------------------------------------
            # 1. 요청 검증
            # ----------------------------------------------------------
            error = self._validate_visualize_request(
                node_map_revision=int(request.node_map_revision),
                route=route,
            )
            if error is not None:
                goal_handle.abort()
                result.success = False
                result.message = error
                carb.log_error(
                    f"[ExtNodeMapBuild] VisualizeRoute failed: {error}"
                )
                return result

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "Visualization canceled before update."
                return result

            # XYZ는 인터페이스에 예약 필드로 유지하지만 현재는 사용하지 않는다.
            # 빈 배열/값 존재 여부와 관계없이 현재 시각화 처리에는 영향을 주지 않는다.
            if request.node_x or request.node_y or request.node_z:
                carb.log_info(
                    "[ExtNodeMapBuild] XYZ fields received but ignored "
                    "in the current visualization implementation"
                )

            # ----------------------------------------------------------
            # 2. 새 Step 표시 준비
            # ----------------------------------------------------------
            self._publish_visualize_feedback(
                goal_handle,
                "clearing_previous_step",
            )

            self._reset_path_visualization()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "Visualization canceled."
                return result

            # ----------------------------------------------------------
            # 3. Node / Edge Yellow Highlight
            # ----------------------------------------------------------
            self._publish_visualize_feedback(
                goal_handle,
                "highlighting_route",
            )

            self._visualized_route = route

            for node_id in route:
                if not self._set_prim_color(
                    self._node_prim_lookup[node_id],
                    PLANNED_COLOR,
                ):
                    raise RuntimeError(
                        f"Failed to highlight Node {node_id}"
                    )

            for start, end in zip(route, route[1:]):
                edge_path = self._edge_prim_lookup[(start, end)]
                if not self._set_prim_color(
                    edge_path,
                    PLANNED_COLOR,
                ):
                    raise RuntimeError(
                        f"Failed to highlight Edge {start} -> {end}"
                    )

            # ----------------------------------------------------------
            # 4. 방문 순서 Label
            # ----------------------------------------------------------
            label_warning = None

            if SHOW_ROUTE_ORDER_LABELS:
                self._publish_visualize_feedback(
                    goal_handle,
                    "drawing_route_order",
                )

                if not self._draw_route_order_labels(route):
                    # 핵심 경로 Highlight가 정상이라면 Action 자체는 성공 처리한다.
                    # 숫자 Label은 Viewport UI 상태에 따라 사용할 수 없을 수 있다.
                    label_warning = " Route order labels were unavailable."

            if goal_handle.is_cancel_requested:
                self._reset_path_visualization()
                goal_handle.canceled()
                result.success = False
                result.message = "Visualization canceled."
                return result

            # ----------------------------------------------------------
            # 5. 완료
            # ----------------------------------------------------------
            self._publish_visualize_feedback(
                goal_handle,
                "done",
            )

            goal_handle.succeed()
            result.success = True
            result.message = (
                f"Visualized AMR={request.amr_id}, "
                f"Task={request.task_id}, "
                f"Nodes={route}."
                f"{label_warning or ''}"
            )

            carb.log_info(
                f"[ExtNodeMapBuild] VisualizeRoute completed: "
                f"AMR={request.amr_id}, "
                f"Task={request.task_id}, "
                f"route={route}"
            )
            return result

        except Exception as exc:
            # 새 Step 표시 도중 문제가 생기면 부분 시각화를 남기지 않는다.
            carb.log_error(
                f"[ExtNodeMapBuild] VisualizeRoute exception: {exc}"
            )
            self._reset_path_visualization()

            try:
                goal_handle.abort()
            except Exception:
                pass

            result.success = False
            result.message = f"Visualization failed: {exc}"
            return result

        finally:
            self._visualization_busy = False

    def _publish_visualize_feedback(
        self,
        goal_handle,
        status: str,
    ) -> None:
        feedback = VisualizeRoute.Feedback()
        feedback.status = status
        goal_handle.publish_feedback(feedback)

    def _validate_visualize_request(
        self,
        node_map_revision: int,
        route: list[int],
    ) -> str | None:
        """현재 Step Planned Path를 실제 Stage NodeMap 기준으로 검증한다."""

        if not self._nodes:
            return "NodeMap contains no nodes."

        if node_map_revision != self._revision:
            return (
                f"NodeMap revision mismatch: "
                f"request={node_map_revision}, "
                f"current={self._revision}"
            )

        if not route:
            return "node_ids is empty."

        missing_nodes = [
            node_id
            for node_id in route
            if node_id not in self._node_prim_lookup
        ]
        if missing_nodes:
            return f"Unknown Node IDs: {missing_nodes}"

        missing_edges = [
            (start, end)
            for start, end in zip(route, route[1:])
            if (start, end) not in self._edge_prim_lookup
        ]
        if missing_edges:
            return f"Disconnected Edges: {missing_edges}"

        # Highlight 가능한 Geometry인지 이전 Step을 지우기 전에 확인한다.
        invalid_visual_nodes = [
            node_id
            for node_id in route
            if not self._prim_contains_gprim(
                self._node_prim_lookup[node_id]
            )
        ]
        if invalid_visual_nodes:
            return (
                "Node visualization geometry not found: "
                f"{invalid_visual_nodes}"
            )

        invalid_visual_edges = [
            (start, end)
            for start, end in zip(route, route[1:])
            if not self._prim_contains_gprim(
                self._edge_prim_lookup[(start, end)]
            )
        ]
        if invalid_visual_edges:
            return (
                "Edge visualization geometry not found: "
                f"{invalid_visual_edges}"
            )

        return None

    # ------------------------------------------------------------------
    # NodeMap loading / Runtime caches
    # ------------------------------------------------------------------

    def _load_current_node_map(self) -> None:
        """현재 Stage의 Node/Edge 데이터와 Prim lookup을 재구성한다."""
        self._nodes = {}
        self._edges = []
        self._node_prim_lookup = {}
        self._edge_prim_lookup = {}

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            carb.log_warn("[ExtNodeMapBuild] No Stage is open")
            return

        node_root = stage.GetPrimAtPath(NODE_ROOT)
        edge_root = stage.GetPrimAtPath(EDGE_ROOT)

        if not node_root.IsValid():
            carb.log_warn(
                f"[ExtNodeMapBuild] Node root not found: {NODE_ROOT}"
            )
            return

        if not edge_root.IsValid():
            carb.log_warn(
                f"[ExtNodeMapBuild] Edge root not found: {EDGE_ROOT}"
            )
            return

        self._nodes = self._load_nodes(node_root)
        self._edges = self._load_edges(edge_root)
        self._build_visualization_lookup()

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
    def _parse_edge_nodes_from_name(
        name: str,
    ) -> tuple[int, int] | None:
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
                if (
                    node_name_attr.IsValid()
                    and node_name_attr.Get() is not None
                )
                else prim_name
            )

            node_type = (
                node_type_attr.Get()
                if (
                    node_type_attr.IsValid()
                    and node_type_attr.Get() is not None
                )
                else "WAYPOINT"
            )

            matrix = transform_cache.GetLocalToWorldTransform(prim)
            position = matrix.ExtractTranslation()

            nodes[node_id] = {
                "name": str(node_name),
                "type": str(node_type),
                "position": (
                    float(position[0]),
                    float(position[1]),
                    float(position[2]),
                ),
                # Isaac Sim Runtime 시각화용
                "prim_path": str(prim.GetPath()),
            }

        return nodes

    def _load_edges(self, edge_root) -> list:
        edges = []

        for prim in edge_root.GetChildren():
            prim_name = prim.GetName()

            start_attr = prim.GetAttribute("from_node")
            end_attr = prim.GetAttribute("to_node")
            weight_attr = prim.GetAttribute("weight")
            bidirectional_attr = prim.GetAttribute("bidirectional")

            start_value = (
                start_attr.Get()
                if start_attr.IsValid()
                else None
            )
            end_value = (
                end_attr.Get()
                if end_attr.IsValid()
                else None
            )

            if start_value is not None and end_value is not None:
                start = int(start_value)
                end = int(end_value)
            else:
                parsed_nodes = self._parse_edge_nodes_from_name(
                    prim_name
                )
                if parsed_nodes is None:
                    carb.log_warn(
                        f"[ExtNodeMapBuild] "
                        f"Invalid Edge name: {prim_name}"
                    )
                    continue

                start, end = parsed_nodes

            if start not in self._nodes or end not in self._nodes:
                carb.log_warn(
                    f"[ExtNodeMapBuild] "
                    f"Edge references unknown Node: "
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
                    (
                        end_position[index]
                        - start_position[index]
                    ) ** 2
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
                    "bidirectional": (
                        bool(bidirectional_value)
                        if bidirectional_value is not None
                        else True
                    ),
                    # Isaac Sim Runtime 시각화용
                    "prim_path": str(prim.GetPath()),
                }
            )

        return edges

    def _build_visualization_lookup(self) -> None:
        """Node ID / Edge Node pair -> USD Prim path lookup."""

        self._node_prim_lookup = {
            node_id: node["prim_path"]
            for node_id, node in self._nodes.items()
        }

        self._edge_prim_lookup = {}

        for edge in self._edges:
            key = (edge["start"], edge["end"])
            self._edge_prim_lookup[key] = edge["prim_path"]

            if edge["bidirectional"]:
                reverse_key = (edge["end"], edge["start"])
                self._edge_prim_lookup[reverse_key] = edge["prim_path"]

        carb.log_info(
            f"[ExtNodeMapBuild] Visualization cache ready: "
            f"Nodes={len(self._node_prim_lookup)}, "
            f"Directed edge lookups={len(self._edge_prim_lookup)}"
        )

    # ------------------------------------------------------------------
    # Node / Edge color visualization
    # ------------------------------------------------------------------

    def _prim_contains_gprim(self, prim_path: str) -> bool:
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return False

        root_prim = stage.GetPrimAtPath(prim_path)
        if not root_prim.IsValid():
            return False

        return any(
            prim.IsA(UsdGeom.Gprim)
            for prim in Usd.PrimRange(root_prim)
        )

    def _set_prim_color(
        self,
        prim_path: str,
        color: Gf.Vec3f,
    ) -> bool:
        """Prim 또는 하위 Gprim의 displayColor를 변경한다."""

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return False

        root_prim = stage.GetPrimAtPath(prim_path)
        if not root_prim.IsValid():
            carb.log_warn(
                f"[ExtNodeMapBuild] "
                f"Visualization Prim not found: {prim_path}"
            )
            return False

        painted_count = 0

        for prim in Usd.PrimRange(root_prim):
            if not prim.IsA(UsdGeom.Gprim):
                continue

            gprim = UsdGeom.Gprim(prim)
            color_attr = gprim.GetDisplayColorAttr()
            gprim_path = str(prim.GetPath())

            if gprim_path not in self._original_display_colors:
                self._original_display_colors[gprim_path] = {
                    "authored": color_attr.HasAuthoredValueOpinion(),
                    "value": color_attr.Get(),
                }

            color_attr.Set([color])
            painted_count += 1

        if painted_count == 0:
            carb.log_warn(
                f"[ExtNodeMapBuild] No Gprim found below: {prim_path}"
            )
            return False

        return True

    def _reset_path_visualization(self) -> None:
        """현재 Step Highlight 및 순서 Label을 제거한다."""
        self._restore_visualization_colors()
        self._original_display_colors = {}
        self._visualized_route = []
        self._clear_route_labels()

    def _restore_visualization_colors(self) -> None:
        """Highlight 전의 displayColor로 복구한다."""
        if not self._original_display_colors:
            return

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return

        for prim_path, original in self._original_display_colors.items():
            prim = stage.GetPrimAtPath(prim_path)

            if (
                not prim.IsValid()
                or not prim.IsA(UsdGeom.Gprim)
            ):
                continue

            gprim = UsdGeom.Gprim(prim)
            color_attr = gprim.GetDisplayColorAttr()

            if (
                original["authored"]
                and original["value"] is not None
            ):
                color_attr.Set(original["value"])
            else:
                color_attr.Clear()

    # ------------------------------------------------------------------
    # Viewport route order labels
    # ------------------------------------------------------------------

    def _ensure_route_label_scene(self) -> bool:
        """Active Viewport에 route-order SceneView를 준비한다."""
        if (
            self._route_label_scene_view is not None
            and self._viewport_window is not None
        ):
            return True

        try:
            viewport_window = get_active_viewport_window()
            if viewport_window is None:
                carb.log_warn(
                    "[ExtNodeMapBuild] "
                    "Active viewport not found; route labels disabled"
                )
                return False

            frame_name = f"{self._ext_id}.planned_route_labels"

            with viewport_window.get_frame(frame_name):
                scene_view = sc.SceneView()

            viewport_window.viewport_api.add_scene_view(scene_view)

            self._viewport_window = viewport_window
            self._route_label_scene_view = scene_view
            return True

        except Exception as exc:
            carb.log_warn(
                f"[ExtNodeMapBuild] "
                f"Failed to create route label SceneView: {exc}"
            )
            self._viewport_window = None
            self._route_label_scene_view = None
            return False

    def _draw_route_order_labels(
        self,
        route: list[int],
    ) -> bool:
        """node_ids[] 순서대로 1,2,3... Label을 Node 위에 표시한다."""

        if not self._ensure_route_label_scene():
            return False

        try:
            self._route_label_scene_view.scene.clear()

            with self._route_label_scene_view.scene:
                for order, node_id in enumerate(route, start=1):
                    x, y, z = self._nodes[node_id]["position"]

                    transform = sc.Matrix44.get_translation_matrix(
                        x,
                        y,
                        z + ROUTE_LABEL_Z_OFFSET,
                    )

                    with sc.Transform(transform=transform):
                        sc.Label(
                            str(order),
                            color=cl.white,
                            size=ROUTE_LABEL_FONT_SIZE,
                            alignment=ui.Alignment.CENTER,
                        )

            return True

        except Exception as exc:
            carb.log_warn(
                f"[ExtNodeMapBuild] "
                f"Failed to draw route order labels: {exc}"
            )
            self._clear_route_labels()
            return False

    def _clear_route_labels(self) -> None:
        if self._route_label_scene_view is None:
            return

        try:
            self._route_label_scene_view.scene.clear()
        except Exception:
            pass

    def _destroy_route_label_scene(self) -> None:
        if self._route_label_scene_view is None:
            self._viewport_window = None
            return

        try:
            self._route_label_scene_view.scene.clear()

            if self._viewport_window is not None:
                self._viewport_window.viewport_api.remove_scene_view(
                    self._route_label_scene_view
                )

            self._route_label_scene_view.destroy()
        except Exception as exc:
            carb.log_warn(
                f"[ExtNodeMapBuild] "
                f"Route label SceneView cleanup warning: {exc}"
            )

        self._route_label_scene_view = None
        self._viewport_window = None

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def on_shutdown(self) -> None:
        global EXTENSION_RUNTIME_READY
        EXTENSION_RUNTIME_READY = False
        carb.log_info(
            "[ExtNodeMapBuild] Extension stopping"
        )

        # 추가 callback 발생 방지
        self._update_subscription = None
        self._stage_subscription = None

        # 현재 Stage의 시각화를 먼저 복구
        self._reset_path_visualization()
        self._destroy_route_label_scene()

        if self._visualize_route_action_server is not None:
            try:
                self._visualize_route_action_server.destroy()
            except Exception:
                pass
            self._visualize_route_action_server = None

        if self._ros_node is not None:
            if self._service is not None:
                self._ros_node.destroy_service(self._service)

            if self._map_changed_publisher is not None:
                self._ros_node.destroy_publisher(
                    self._map_changed_publisher
                )
            if self._node_map_bridge_publisher is not None:
                self._ros_node.destroy_publisher(
                    self._node_map_bridge_publisher
                )
            if self._route_request_subscription is not None:
                self._ros_node.destroy_subscription(
                    self._route_request_subscription
                )
            if self._route_response_publisher is not None:
                self._ros_node.destroy_publisher(
                    self._route_response_publisher
                )

            self._ros_node.destroy_node()

        self._service = None
        self._map_changed_publisher = None
        self._node_map_bridge_publisher = None
        self._route_request_subscription = None
        self._route_response_publisher = None
        self._ros_node = None

        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()

        self._nodes = {}
        self._edges = {}
        self._node_prim_lookup = {}
        self._edge_prim_lookup = {}
        self._original_display_colors = {}
        self._visualized_route = []
        self._revision = 0
        self._ext_id = None

        carb.log_info(
            "[ExtNodeMapBuild] Extension stopped"
        )
