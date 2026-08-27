#!/usr/bin/env python3
"""Expose Main Scene NodeMap contracts across the Isaac DDS ABI boundary."""

from __future__ import annotations

import json
import threading
import time
import uuid

from interfaces.action import VisualizeRoute
from interfaces.msg import NodeMapChanged
from interfaces.srv import GetNodeMap
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class SceneEndpointAdapter(Node):
    """Keep public custom endpoints on system Jazzy and Stage work in Isaac."""

    def __init__(self) -> None:
        super().__init__("scene_endpoint_adapter")
        self.declare_parameter("visualization_timeout_s", 5.0)
        self.visualization_timeout_s = float(
            self.get_parameter("visualization_timeout_s").value
        )

        self._lock = threading.Lock()
        self._node_map: dict | None = None
        self._pending: dict[str, tuple[threading.Event, dict]] = {}
        callback_group = ReentrantCallbackGroup()
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._map_subscription = self.create_subscription(
            String,
            "/cobot3/runtime/node_map",
            self._on_node_map,
            map_qos,
            callback_group=callback_group,
        )
        self._route_request_publisher = self.create_publisher(
            String,
            "/cobot3/runtime/visualize_route/request",
            10,
        )
        self._route_response_subscription = self.create_subscription(
            String,
            "/cobot3/runtime/visualize_route/response",
            self._on_route_response,
            10,
            callback_group=callback_group,
        )
        self._node_map_changed_publisher = self.create_publisher(
            NodeMapChanged,
            "/node_map_changed",
            10,
        )
        self._node_map_service = self.create_service(
            GetNodeMap,
            "/get_node_map",
            self._get_node_map,
            callback_group=callback_group,
        )
        self._visualize_route_server = ActionServer(
            self,
            VisualizeRoute,
            "/visualize_route",
            execute_callback=self._execute_visualize_route,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=callback_group,
        )
        self.get_logger().info(
            "Scene endpoint adapter ready; waiting for Isaac NodeMap bridge"
        )

    def _on_node_map(self, message: String) -> None:
        try:
            node_map = json.loads(message.data)
            if not node_map.get("nodes"):
                raise ValueError("NodeMap contains no nodes")
        except (json.JSONDecodeError, ValueError) as exc:
            self.get_logger().error(f"Invalid Isaac NodeMap bridge payload: {exc}")
            return

        with self._lock:
            previous_revision = (
                int(self._node_map["revision"])
                if self._node_map is not None
                else None
            )
            self._node_map = node_map

        revision = int(node_map["revision"])
        if revision != previous_revision:
            changed = NodeMapChanged()
            changed.revision = revision
            changed.stage_identifier = str(node_map.get("stage_identifier", ""))
            changed.node_count = len(node_map["nodes"])
            changed.edge_count = len(node_map.get("edges", []))
            self._node_map_changed_publisher.publish(changed)
            self.get_logger().info(
                "Isaac NodeMap ready: "
                f"revision={revision}, nodes={changed.node_count}, "
                f"edges={changed.edge_count}"
            )

    def _get_node_map(self, _request, response):
        with self._lock:
            node_map = self._node_map
        if node_map is None:
            response.success = False
            response.message = "Isaac Main Scene NodeMap is not ready."
            return response

        response.revision = int(node_map["revision"])
        for node in node_map["nodes"]:
            response.node_ids.append(int(node["id"]))
            response.node_names.append(str(node["name"]))
            response.node_types.append(str(node["type"]))
            position = node["position"]
            response.node_x.append(float(position[0]))
            response.node_y.append(float(position[1]))
            response.node_z.append(float(position[2]))
        for edge in node_map.get("edges", []):
            response.edge_from.append(int(edge["start"]))
            response.edge_to.append(int(edge["end"]))
            response.edge_weights.append(float(edge["weight"]))
            response.edge_bidirectional.append(bool(edge["bidirectional"]))
        response.success = True
        response.message = (
            f"Nodes={len(node_map['nodes'])}, Edges={len(node_map.get('edges', []))}"
        )
        return response

    def _goal_callback(self, goal_request) -> GoalResponse:
        with self._lock:
            node_map = self._node_map
        if node_map is None:
            self.get_logger().warning("VisualizeRoute rejected: NodeMap not ready")
            return GoalResponse.REJECT
        if int(goal_request.node_map_revision) != int(node_map["revision"]):
            self.get_logger().warning("VisualizeRoute rejected: revision mismatch")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(_goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute_visualize_route(self, goal_handle):
        result = VisualizeRoute.Result()
        request_id = uuid.uuid4().hex
        event = threading.Event()
        response_holder: dict = {}
        with self._lock:
            self._pending[request_id] = (event, response_holder)

        feedback = VisualizeRoute.Feedback()
        feedback.status = "forwarding_to_isaac_stage"
        goal_handle.publish_feedback(feedback)
        payload = {
            "request_id": request_id,
            "amr_id": goal_handle.request.amr_id,
            "task_id": goal_handle.request.task_id,
            "node_map_revision": int(goal_handle.request.node_map_revision),
            "node_ids": [int(value) for value in goal_handle.request.node_ids],
        }
        bridge_request = String()
        bridge_request.data = json.dumps(payload, separators=(",", ":"))
        self._route_request_publisher.publish(bridge_request)

        deadline = time.monotonic() + self.visualization_timeout_s
        while not event.wait(timeout=0.05):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "Visualization canceled."
                break
            if time.monotonic() >= deadline:
                goal_handle.abort()
                result.success = False
                result.message = "Isaac Stage visualization timed out."
                break
        else:
            result.success = bool(response_holder.get("success", False))
            result.message = str(response_holder.get("message", ""))
            if result.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()

        with self._lock:
            self._pending.pop(request_id, None)
        return result

    def _on_route_response(self, message: String) -> None:
        try:
            response = json.loads(message.data)
            request_id = str(response["request_id"])
        except (json.JSONDecodeError, KeyError) as exc:
            self.get_logger().error(f"Invalid route bridge response: {exc}")
            return
        with self._lock:
            pending = self._pending.get(request_id)
        if pending is None:
            return
        event, response_holder = pending
        response_holder.update(response)
        event.set()

    def destroy_node(self):
        self._visualize_route_server.destroy()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SceneEndpointAdapter()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
