# `fleet_management_system.py`

## 역할

FMS는 NodeMap, waiting/active Task, AMR 상태와 cuOpt Task ordering을 관리한다. AMR이 `/fms/request_task`로 현재 odom x/y를 보내면 nearest Active recovery Node를 계산하고 waiting Task 중 수행할 Task를 선택한다.

`Task.start`는 경유지, `Task.goal`은 Dolly delivery 목적지라는 기존 의미를 유지한다. `DELIVERY_COMPLETE` 또는 `MISSION_COMPLETE`를 받으면 active registry에서 Task를 제거한다.

## Scenario 0 two-route 응답

cuOpt selected plan의 기존 route는 `Task.start`에서 `Task.goal`까지의 delivery route로 유지한다. FMS는 `latest_plan.recovery_node_id`를 실제 AMR 위치의 nearest Active Node로 재사용하고 기존 `NodeMapGraphManager.create_route()`로 approach route를 추가 생성한다.

```text
approach_route_*: recovery Node -> Task.start
route_*:          Task.start -> Task.goal
```

`RequestTask` 응답 전 다음 endpoint와 병렬 배열 길이를 검증한다.

```text
approach[-1] == Task.start
route[0]      == Task.start
route[-1]     == Task.goal
len(node_ids) == len(x) == len(y) == len(z)
```

cuOpt production solver와 shortest path 구현은 변경하지 않는다. GPU-free unit test는 작은 `NodeMapGraphManager`와 직접 생성한 `PlannedNodeRoute`로 계약을 검증한다.

## Task lifecycle

```text
waiting queue
-> cuOpt selection
-> TaskManager.assign_task()
-> active_tasks
-> DELIVERY_COMPLETE or MISSION_COMPLETE
-> TaskManager.complete_task()
```

중복 `task_id`, inactive Node, route endpoint mismatch는 거부한다. NodeMap revision은 두 route가 공유하는 `node_map_revision`으로 응답한다.
