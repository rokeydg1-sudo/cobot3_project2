# Scenario 0 Runtime Flow

## 용어와 Route 계약

- 경유지(Pre-Docking, Parts Supermarket): `Task.start`인 NodeMap Node다.
- 도킹 위치: 경유지에 마지막으로 진입한 Edge 방향의 정면 약 3 m에 있는 Dolly다.
- 목적지: Dolly를 내려놓는 `Task.goal` Node다.
- Route A(`approach_route_*`): 실제 odom에서 가장 가까운 Active recovery Node부터 `Task.start`까지다.
- Route B(기존 `route_*`): `Task.start`부터 `Task.goal`까지의 delivery route다.

`Task.start`와 `Task.goal`의 의미, cuOpt Task ordering, `NodeMapGraphManager`의 shortest path 역할은 바꾸지 않는다. FMS는 다음을 검증한 뒤 병렬 Node/XYZ 배열과 cost를 응답한다.

```text
approach_route[-1] == Task.start
route[0]           == Task.start
route[-1]          == Task.goal
len(node_ids) == len(x) == len(y) == len(z)
```

## Mission state flow

```text
IDLE
-> TASK_ASSIGNED
-> MOVING_TO_WAYPOINT -> ARRIVED_WAYPOINT
-> PRE_DOCKING -> DOCKING -> DOCKING_COMPLETE
-> LIFTING_UP -> LIFT_UP_COMPLETE
-> RETURNING_TO_WAYPOINT -> RETURNED_TO_WAYPOINT
-> MOVING_TO_DELIVERY -> ARRIVED_DELIVERY
-> LIFTING_DOWN -> LIFT_DOWN_COMPLETE
-> DELIVERY_COMPLETE -> MISSION_COMPLETE
-> IDLE
```

AMR은 odometry가 있을 때만 IDLE timer에서 `/fms/request_task`를 호출한다. Mission 종료 후 task ID를 비우고 `IDLE`을 발행하며, 다음 timer tick에서 현재 odom x/y로 다시 Task를 요청한다.

## NavigateThroughPoses와 heading

각 route는 기존 `VisualizeRoute` Action으로 개별 시각화한 다음 `/navigate_through_poses`로 주행한다. interface를 확장하거나 두 route를 수행용 단일 배열로 합치지 않는다.

첫 Node는 AMR이 이미 있는 current/recovery Node이므로 Action Goal에서 제외한다. 한 Node뿐인 route는 Action을 보내지 않고 현재 odom yaw를 유지한 채 성공 처리한다.

`A -> B -> C -> D`에서 Pose yaw는 다음과 같다.

```text
B: B -> C
C: C -> D
D: C -> D (마지막 incoming Edge 유지)
```

`atan2(dy, dx)`와 planar quaternion `z=sin(yaw/2)`, `w=cos(yaw/2)`를 사용한다. Route A 마지막 Pose가 경유지 진입 방향을 유지하므로 카메라 정면 Dolly가 pickup 대상이 된다.

## Dock, Lift, odom return, delivery

Route A 성공 후 `ARRIVED_WAYPOINT`, `PRE_DOCKING`을 발행하고 기존 `DockDolly`를 호출한다. YOLO Pose, PnP, Vision P controller, fallback, final entry tuning은 변경하지 않았다.

`DOCKING_COMPLETE` 뒤 `/lift_dolly`에 `LIFT_UP`을 보내고 성공 Result를 확인한 뒤에만 `load_state=LOADED`와 `LIFT_UP_COMPLETE`로 진행한다. 이어서 Vision/Nav2를 사용하지 않고 parameterized `cmd_vel_topic`에 음수 `linear.x`를 발행한다. 시작 odom `(x, y)` 대비 평면 변위가 `return_distance_m`(기본 `3.0`) 이상이면 zero Twist를 발행하고 Route B를 시작한다. timeout, exception, shutdown을 포함한 모든 종료 경로에서 zero Twist를 발행한다.

목적지 도착 후 `LIFT_DOWN` Result 성공을 확인해 `load_state=EMPTY`로 바꾸고 완료 상태를 발행한다. 이번 MVP에서는 Lift Down 뒤 Dolly 아래에서 별도 탈출하지 않는다.

## Parameterized integration endpoints

| Parameter | Default |
|---|---|
| `odom_topic` | `/amr/odom` |
| `cmd_vel_topic` | `/cmd_vel` |
| `nav2_action_name` | `/navigate_through_poses` |
| `dock_action_name` | `/dock_dolly` |
| `lift_action_name` | `/lift_dolly` |
| `fms_service_name` | `/fms/request_task` |
| `visualize_route_action_name` | `/visualize_route` |
| `return_distance_m` | `3.0` |

Main Scene의 `/AMR1/*` 여부는 확정 Prim/ROS graph를 확인한 뒤 launch parameter/remap으로 해결한다.

## Failure policy

- Dock failure: Lift Up, reverse, delivery를 실행하지 않고 `ERROR/TASK_FAILED`.
- Lift Up failure: reverse와 delivery를 실행하지 않고 `ERROR/TASK_FAILED`.
- Reverse timeout: zero Twist를 보장하고 delivery를 실행하지 않은 채 `ERROR/TASK_FAILED`.
- Action reject/timeout/cancel/exception도 동일한 실패 흐름을 사용한다.

## Runtime-only checks

- `RUNTIME_BLOCKER_MAIN_SCENE`: camera/odom/cmd_vel/scan/TF/clock와 lift Prim/DOF가 확정되지 않았다.
- `RUNTIME_CHECK_DOLLY_UNDER_AMR`: Lift Down 후 Dolly 아래에서 다음 Nav2 Mission이 출발 가능한지 실제 costmap으로 확인해야 한다.
- loaded Dolly footprint/inflation, actual Vision docking, actual Nav2 driving, physical lift와 전체 physical E2E는 GPU workstation에서만 검증한다.
