# `amr_node.py`

## 역할

AMR이 IDLE일 때 실제 odom x/y로 FMS Task를 pull하고, FMS가 반환한 approach/delivery Node route를 순차 수행한다. Route 계산과 Task ordering은 하지 않는다.

```text
FMS RequestTask
-> approach VisualizeRoute
-> NavigateThroughPoses
-> DockDolly
-> LiftDolly UP
-> direct cmd_vel odom return
-> delivery VisualizeRoute
-> NavigateThroughPoses
-> LiftDolly DOWN
-> Mission complete
```

## Route와 Pose

`approach_route_*`는 current/recovery Node -> `Task.start`, 기존 `route_*`는 `Task.start` -> `Task.goal`이다. 각 route의 Node/XYZ 배열 길이와 공통 `Task.start`를 검증한다.

AMR이 이미 있는 첫 Node를 `NavigateThroughPoses.Goal.poses`에서 제외한다. 중간 Pose는 다음 Edge 방향, 마지막 Pose는 마지막 incoming Edge 방향을 yaw로 사용한다. route가 한 Node면 Action을 호출하지 않고 odom yaw를 유지한다.

## Mission 상태

```text
IDLE -> TASK_ASSIGNED
-> MOVING_TO_WAYPOINT -> ARRIVED_WAYPOINT
-> PRE_DOCKING -> DOCKING -> DOCKING_COMPLETE
-> LIFTING_UP -> LIFT_UP_COMPLETE
-> RETURNING_TO_WAYPOINT -> RETURNED_TO_WAYPOINT
-> MOVING_TO_DELIVERY -> ARRIVED_DELIVERY
-> LIFTING_DOWN -> LIFT_DOWN_COMPLETE
-> DELIVERY_COMPLETE -> MISSION_COMPLETE -> IDLE
```

Dock/Lift/Navigation Action은 `MultiThreadedExecutor` callback으로 Goal/Result를 받고 worker thread는 `threading.Event`로 대기한다. ROS callback thread에서 장시간 mission을 block하지 않는다.

## Lift와 reverse

Dock 성공 뒤에만 `LiftDolly.LIFT_UP`을 호출한다. UP 성공 뒤 `load_state=LOADED`로 바꾸고 Vision/Nav2가 아닌 direct `Twist.linear.x < 0`으로 후진한다. 시작 odom 대비 평면 변위가 `return_distance_m` 이상이면 정지한다. timeout/exception/shutdown을 포함한 모든 reverse 종료에서 zero Twist를 발행한다.

목적지에서는 `LiftDolly.LIFT_DOWN` 성공 뒤 `load_state=EMPTY`, `DELIVERY_COMPLETE`, `MISSION_COMPLETE`, `IDLE` 순으로 전환한다.

## ROS 계약과 parameter

| 종류 | 기본 이름 | 타입/parameter |
|---|---|---|
| Service | `/fms/request_task` | `interfaces/srv/RequestTask`, `fms_service_name` |
| Action | `/visualize_route` | `interfaces/action/VisualizeRoute`, `visualize_route_action_name` |
| Action | `/navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses`, `nav2_action_name` |
| Action | `/dock_dolly` | `interfaces/action/DockDolly`, `dock_action_name` |
| Action | `/lift_dolly` | `interfaces/action/LiftDolly`, `lift_action_name` |
| Topic | `/amr/odom` | `nav_msgs/msg/Odometry`, `odom_topic` |
| Topic | `/cmd_vel` | `geometry_msgs/msg/Twist`, `cmd_vel_topic` |
| Topic | `/amr/status` | `std_msgs/msg/String` |

reverse parameter 기본값은 `return_distance_m=3.0`, `return_speed_mps=0.20`, `return_timeout_s=30.0`이다.

## 실패 처리

Action server 미준비, reject, timeout, canceled/aborted, `success=false`, invalid route, odom 부재, reverse timeout은 `ERROR/TASK_FAILED`로 끝난다. Dock failure 뒤 Lift/Delivery, Lift Up failure 뒤 reverse/Delivery, reverse timeout 뒤 Delivery는 실행하지 않는다.
