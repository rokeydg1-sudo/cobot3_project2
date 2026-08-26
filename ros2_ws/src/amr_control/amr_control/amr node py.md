3.2 amr_node.py

기존 역할

기존 AMR 제어에서는 목적지 좌표를 직접 /amr/goal로 전달하고,
/amr/odom의 현재 위치와 목표 위치 사이의 거리를 직접 계산하여 도착 여부를 판단했다.

현재 역할

AMR Node는 이제 직접 주행 경로를 계산하지 않는다.

FMS
 ↓
RequestTask Service
 ↓
AMR Node
 ↓
NavigateToPose
 ↓
Nav2

주요 변경

1. FMS Pull 방식 적용

AMR이 IDLE 상태일 때 일정 주기로 FMS에 다음 Task를 요청한다.

Service:

/fms/request_task

AMR이 전달하는 상태:

amr_id

현재 state

현재 task

현재 x/y 위치

load state

2. Nav2 ActionClient 적용

기존:

/amr/goal Publish

현재:

NavigateToPose Action

실제 Action:

/navigate_to_pose

Goal frame:

map

3. 이동 성공 판정 변경

기존:

/amr/odom
 ↓
목표와 거리 계산
 ↓
threshold 이하이면 성공

현재:

NavigateToPose
 ↓
Feedback
 ↓
Action Result
 ↓
SUCCEEDED / ABORTED / CANCELED

즉 이동 성공 여부를 AMR Node가 임의로 계산하지 않고 Nav2의 결과를 사용한다.

4. AMR 상태 Event 추가

Topic:

/amr/status

대표 상태:

READY
TASK_ASSIGNED
MOVING_TO_PICKUP
ARRIVED_PICKUP
LOADING
LOAD_COMPLETE
MOVING_TO_DELIVERY
ARRIVED_DELIVERY
DELIVERY_COMPLETE
MISSION_COMPLETE
IDLE
TASK_FAILED

FMS는 이 이벤트를 이용해 AMR 상태와 Active Task를 관리한다.

5. MultiThreadedExecutor 적용

동시에 처리해야 하는 통신이 증가하여
다음 Callback을 분리했다.

/amr/odom

FMS Service

Task Request Timer

Nav2 Action

6. Vision Dolly Docking 통합

AMR Node는 Pickup 좌표를 현재 단계의 Pre-Docking pose로 취급한다. 별도의
Pre-Docking 좌표 계산은 하지 않는다. Pickup `NavigateToPose`가
`SUCCEEDED`로 끝난 후 `/dock_dolly`에 `DockDolly` Goal을 보내며, Goal에는
현재 `amr_id`와 `task_id`가 포함된다.

```text
MOVING_TO_PICKUP
  -> NavigateToPose SUCCEEDED
  -> PRE_DOCKING
  -> DockDolly Goal accepted
  -> DOCKING
  -> DOCKING_COMPLETE
  -> ARRIVED_PICKUP
  -> LOADING
```

Docking Result가 `success=true`이고 Action 상태도 `SUCCEEDED`인 경우에만
`ARRIVED_PICKUP`과 `LOADING`으로 진행한다. Action Server 미준비, Goal 거절,
Goal/Result timeout, 예외, canceled/aborted 상태 또는 `success=false`는 모두
기존 `handle_task_failure()`로 연결되어 `TASK_FAILED`와 `ERROR` 상태가 된다.

7. Callback group / executor 구조

AMR Node는 `MultiThreadedExecutor(num_threads=4)`를 사용한다. `/amr/odom`,
FMS Service, Task 요청 Timer, Action callback은 각각의
`MutuallyExclusiveCallbackGroup`으로 분리된다. `VisualizeRoute`,
`NavigateToPose`, `DockDolly`는 순차 Mission이므로 같은 `action_group`을
사용한다. Task worker thread는 ROS callback을 직접 spin하지 않고
`threading.Event`로 Action Goal과 Result를 기다린다.

8. 전체 Task state/event 흐름

```text
IDLE
  -> TASK_ASSIGNED
  -> VisualizeRoute
  -> MOVING_TO_PICKUP
  -> PRE_DOCKING
  -> DOCKING
  -> DOCKING_COMPLETE
  -> ARRIVED_PICKUP
  -> LOADING
  -> LOAD_COMPLETE
  -> MOVING_TO_DELIVERY
  -> ARRIVED_DELIVERY
  -> DELIVERY_COMPLETE
  -> MISSION_COMPLETE
  -> IDLE
```

9. ROS 통신 계약

| 종류 | 이름 | 타입 | 방향 | 역할 |
|---|---|---|---|---|
| Service | `/fms/request_task` | `interfaces/srv/RequestTask` | AMR -> FMS | IDLE AMR의 Pull 방식 Task 요청 |
| Action | `/visualize_route` | `interfaces/action/VisualizeRoute` | AMR -> Isaac Sim | 계획 경로 시각화 |
| Action | `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | AMR -> Nav2 | Pickup/Delivery 이동 |
| Action | `/dock_dolly` | `interfaces/action/DockDolly` | AMR -> Vision | Dolly 인식 기반 미세 정렬 및 진입 |
| Topic | `/amr/odom` | `nav_msgs/msg/Odometry` | Isaac Sim -> AMR | 현재 위치 갱신 및 FMS 요청 좌표 |
| Topic | `/amr/status` | `std_msgs/msg/String` | AMR -> FMS | Task lifecycle event 발행 |

`/dock_dolly` 이름은 `dock_action_name` parameter로 변경할 수 있다. Nav2
Goal이 종료된 다음에만 DockDolly Goal을 보내므로 Nav2와 Vision이 동시에
`/cmd_vel`을 제어하지 않는다.
