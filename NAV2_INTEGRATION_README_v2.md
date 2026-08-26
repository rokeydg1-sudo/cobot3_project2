# Nav2 Integration 변경 내역 및 실행 구조

> 작성 기준: 2026-08-24  
> 목적: Nav2 적용 이전 구조에서 현재 구조로 변경된 내용을 팀원들과 공유하기 위한 작업 정리

---

## 1. 변경 요약

이번 작업의 핵심은 기존의 **직접 목표 좌표 기반 AMR 이동 구조**를
**FMS → AMR Node → Nav2 → `/cmd_vel` → Isaac Sim** 구조로 변경한 것이다.

또한 Nav2 연동 이후 한 번 더 보완 작업을 진행하여,
**Assembly의 Task 재전송을 허용하면서도 FMS에서는 같은 `task_id`가 중복 등록되지 않도록**
`waiting queue`와 `active_tasks`를 분리해 관리하도록 수정했다.

### 이전 구조

```text
Assembly
  ↓
FMS
  ↓
AMR Mission / Direct Goal
  ↓
/amr/goal 또는 TCP
  ↓
Isaac Sim Pose Controller
  ↓
AMR 이동
```

### 현재 구조

```text
Assembly Node
  ↓ /assembly/request
FMS + cuOpt
  ↓ /fms/request_task
AMR Node
  ↓ NavigateToPose Action
Nav2
  ↓ /cmd_vel
Isaac Sim
  ↑
/amr/odom
/front_2d_lidar/scan
/tf
/tf_static
/clock
```

---

## 2. 파일 변경 분류

| 파일 | 구분 | 주요 변경 |
|---|---|---|
| `simulation/isaac_sim/standalone_amr_world.py` | 수정 | ROS2 Bridge 확장, `/cmd_vel`, LiDAR, `/clock`, TF 추가 |
| `ros2_ws/src/amr_control/amr_control/amr_node.py` | 수정/교체 | 직접 Goal 방식 제거, FMS Pull + Nav2 `NavigateToPose` 적용 |
| `fms/scenario0_fms.py` | 수정 | AMR Pull Service, 실제 AMR 상태 기반 cuOpt 호출, waiting/active Task 관리, 중복 `task_id` 방지 |
| `ros2_ws/src/assembly_cell/assembly_cell/assembly_node.py` | 수정 | FMS 미연결/실패 시 같은 Task를 유지하고 일정 시간 뒤 재전송하는 정책 적용 |
| `ros2_ws/src/assembly_cell/assembly_cell/area_detection_node.py` | 추가 | `/amr/odom` 기반 Cell 도착 영역 판정 |
| `config/nav2_params.yaml` | 신규 | Nav2 Controller/Planner/Costmap/Collision Monitor 설정 |
| `ros2_ws/src/navigation/maps/factory_map.yaml` | 신규 | Map Server용 지도 메타데이터 |
| `ros2_ws/src/navigation/maps/factory_map.pgm` | 신규 | 실제 Occupancy Grid 이미지 |

> `RequestTask.srv`, 각 패키지의 `setup.py/package.xml`도 이번 연동 과정에서
> 변경되었을 가능성이 있으므로 Git diff 기준으로 최종 확인이 필요하다.

---

# 3. 기존 파일 수정 내용

## 3.1 `standalone_amr_world.py`

### 기존 역할

기존에는 Isaac Sim 내부의 Nova Carter를 직접 제어하는 역할이 중심이었다.

초기 단계에서는 TCP 5005/5006을 이용해 목표와 Pose를 주고받았고,
이후 ROS2 `/amr/goal`을 받아 Isaac 내부 `WheelBasePoseController`가 목표 좌표까지 직접 이동하는 구조를 사용했다.

### 현재 변경 내용

Nav2를 사용할 수 있도록 Isaac Sim을 **물리 시뮬레이션 + 센서/Actuator 인터페이스 계층**으로 확장했다.

추가된 주요 ROS2 인터페이스:

```text
Isaac Sim → ROS2
/amr/odom
/front_2d_lidar/scan
/clock
/tf
/tf_static

ROS2 → Isaac Sim
/cmd_vel
```

### 추가된 기능

#### 1. `/cmd_vel` Subscriber

Nav2에서 계산한 `geometry_msgs/msg/Twist`를 받아
Nova Carter의 Differential Controller에 전달한다.

```text
Nav2
 ↓
/cmd_vel
 ↓
Isaac Sim ROS2 Bridge
 ↓
DifferentialController
 ↓
Nova Carter Wheel
```

#### 2. Front 2D LiDAR ROS2 Publish

Nova Carter Asset에 원래 포함된 Front RPLidar를 재사용한다.

```text
RPLidar
 ↓
ROS2RtxLidarHelper
 ↓
/front_2d_lidar/scan
```

Frame ID:

```text
front_2d_lidar
```

#### 3. TF 추가

Nav2와 AMCL이 사용할 수 있도록 TF Tree를 추가했다.

```text
odom
 ↓
base_link
 ↓
front_2d_lidar
```

- `odom -> base_link`: Dynamic TF
- `base_link -> front_2d_lidar`: Static TF

#### 4. `/clock` 추가

Isaac Simulation Time을 ROS2 `/clock`으로 발행하여
Nav2 전체가 `use_sim_time:=true`로 같은 시간을 사용하도록 변경했다.

#### 5. TCP 제거

기존 TCP 통신:

```text
5005 Goal
5006 Pose
```

은 제거했다.

현재 파일에는 이전 `/amr/goal` Pose Controller 경로가 호환/테스트용으로 일부 남아 있지만,
Nav2 운용 시 실제 주행 제어는 `/cmd_vel`이 담당한다.

---

## 3.2 `amr_node.py`

### 기존 역할

기존 AMR 제어에서는 목적지 좌표를 직접 `/amr/goal`로 전달하고,
`/amr/odom`의 현재 위치와 목표 위치 사이의 거리를 직접 계산하여 도착 여부를 판단했다.

### 현재 역할

AMR Node는 이제 직접 주행 경로를 계산하지 않는다.

```text
FMS
 ↓
RequestTask Service
 ↓
AMR Node
 ↓
NavigateToPose
 ↓
Nav2
```

### 주요 변경

#### 1. FMS Pull 방식 적용

AMR이 `IDLE` 상태일 때 일정 주기로 FMS에 다음 Task를 요청한다.

Service:

```text
/fms/request_task
```

AMR이 전달하는 상태:

- `amr_id`
- 현재 state
- 현재 task
- 현재 x/y 위치
- load state

#### 2. Nav2 ActionClient 적용

기존:

```text
/amr/goal Publish
```

현재:

```text
NavigateToPose Action
```

실제 Action:

```text
/navigate_to_pose
```

Goal frame:

```text
map
```

#### 3. 이동 성공 판정 변경

기존:

```text
/amr/odom
 ↓
목표와 거리 계산
 ↓
threshold 이하이면 성공
```

현재:

```text
NavigateToPose
 ↓
Feedback
 ↓
Action Result
 ↓
SUCCEEDED / ABORTED / CANCELED
```

즉 이동 성공 여부를 AMR Node가 임의로 계산하지 않고 Nav2의 결과를 사용한다.

#### 4. AMR 상태 Event 추가

Topic:

```text
/amr/status
```

대표 상태:

```text
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
```

FMS는 이 이벤트를 이용해 AMR 상태와 Active Task를 관리한다.

#### 5. MultiThreadedExecutor 적용

동시에 처리해야 하는 통신이 증가하여
다음 Callback을 분리했다.

- `/amr/odom`
- FMS Service
- Task Request Timer
- Nav2 Action

---

## 3.3 `scenario0_fms.py`

### 이전 역할

이전 FMS는 Assembly Task Queue를 보관하고,
AMR의 Route 요청이 들어오면 Queue 전체를 cuOpt에 전달해
최적화 순서를 계산하는 구조였다.

초기 버전에서는 실제 AMR 상태 대신 고정된 초기 상태를 사용했고,
최적화 결과만 계산/보관하는 성격이 강했다.

### 현재 역할

현재 FMS는 단순 최적화 호출기가 아니라
**Task Queue + Task Assignment + AMR State 관리의 중심 Node** 역할을 한다.

### 주요 변경

#### 1. AMR Pull Service 추가

```text
/fms/request_task
```

AMR이 직접 다음 Task를 요청한다.

#### 2. 실제 AMR 상태를 cuOpt 입력으로 사용

AMR Node가 전달한:

```text
x
y
state
load_state
current_task_id
```

를 이용해 `AMRState`를 생성한다.

#### 3. Waiting Queue / Active Task 분리

기존에는 Task가 AMR에 할당되면 Queue에서 빠지고 끝나는 구조였다.

현재는 다음처럼 관리한다.

```text
waiting queue
    ↓ AMR에 할당
active_tasks
    ↓ 작업 완료
제거
```

의미:

```text
task_queue
= 아직 AMR에 할당되지 않은 작업

active_tasks
= 이미 AMR에 할당되어 수행 중인 작업
```

즉 FMS가 단순히 “남아 있는 작업”만 보는 것이 아니라,
**현재 어떤 작업이 이미 실행 중인지까지 추적할 수 있게 변경했다.**

#### 4. 중복 `task_id` 방지

Assembly Node는 부품이 아직 도착하지 않았거나
FMS 응답을 받지 못한 경우 같은 Task를 다시 보낼 수 있다.

예:

```text
task_id=1
task_id=1
task_id=1
```

이 경우 FMS에서는 다음 두 위치를 모두 확인한다.

```text
1. waiting queue에 같은 task_id가 있는가?
2. active_tasks에 같은 task_id가 있는가?
```

둘 중 하나라도 이미 존재하면 새로 등록하지 않는다.

따라서:

```text
Assembly 재전송 허용
        +
FMS 중복 방지
```

두 정책을 함께 적용했다.

#### 5. 완료 Task 정리

AMR Node가 `/amr/status`로 작업 완료 이벤트를 보낸다.

예:

```text
status=DELIVERY_COMPLETE
task_id=1
```

또는:

```text
status=MISSION_COMPLETE
task_id=1
```

FMS는 해당 `task_id`를 `active_tasks`에서 제거한다.

```text
active_tasks
    ↓
task_id=1 제거
    ↓
작업 완료 상태 정리
```

즉 “AMR에 할당된 순간”부터 “완료될 때”까지
Task Lifecycle을 FMS가 추적하는 구조가 추가되었다.

#### 6. logical location → physical coordinate 변환

FMS가 cuOpt 결과를 실제 위치 좌표로 변환하여 AMR Node에 전달한다.

예:

```text
supermarket → (-7.0, 0.0)
cell_a      → ( 7.0, 3.5)
cell_b      → ( 7.0, 0.0)
cell_c      → ( 7.0,-3.5)
```

AMR Node는 이 좌표를 그대로 Nav2 Goal에 넣는다.

---

## 3.4 `assembly_node.py`

### 추가 보완 정책

Nav2 연동 이후 Task 유실 문제를 줄이기 위해
Assembly → FMS 통신 정책도 한 번 더 수정했다.

기존에는 Assembly가 Task를 보냈을 때
FMS가 실행되지 않거나 응답할 수 없는 상태라면
Task 전달이 끊길 가능성이 있었다.

현재는:

```text
Assembly Task 생성
      ↓
FMS 연결 확인
      ↓
FMS 없음
      ↓
[FMS WAIT]
      ↓
Task 유지
      ↓
일정 시간 후 재전송
```

형태로 동작한다.

핵심은 **재전송 시 새로운 Task를 생성하는 것이 아니라,
동일한 `task_id`의 기존 Task를 다시 요청한다는 것**이다.

따라서 Assembly와 FMS의 정책은 서로 맞물린다.

```text
Assembly
= 동일 Task 재전송 가능

FMS
= 동일 task_id는 중복 등록하지 않음
```

이 조합으로 인해 FMS 실행 순서가 늦거나
일시적으로 통신이 끊겨도 Task가 사라지지 않도록 보완했다.

---

# 4. 어제 밤 추가 보완 — Task Lifecycle 안정화

Nav2 연동 이후 한 번 더 수정한 핵심 부분이다.

## 4.1 왜 수정했는가?

Assembly는 해당 부품이 도착하기 전까지
같은 작업 요청을 다시 보낼 수 있다.

그런데 FMS가 단순 Queue만 가지고 있다면:

```text
Task 1 수신
 ↓
AMR에 할당
 ↓
Queue에서 Task 1 제거
 ↓
Assembly가 Task 1 재전송
 ↓
FMS가 새 Task라고 오인
```

하는 문제가 생길 수 있었다.

즉 **이미 AMR이 수행 중인 작업이 다시 Queue에 들어갈 수 있는 문제**가 있었다.

## 4.2 해결 방법

FMS에 `active_tasks` Registry를 추가했다.

```text
Assembly
 ↓
task_id=1
 ↓
FMS Waiting Queue
 ↓
AMR 할당
 ↓
active_tasks[1]
 ↓
AMR 작업 수행
 ↓
DELIVERY_COMPLETE / MISSION_COMPLETE
 ↓
active_tasks에서 제거
```

## 4.3 최종 정책

### Assembly

- FMS가 없으면 `[FMS WAIT]`
- 기존 Task 유지
- 일정 시간 후 같은 `task_id`로 다시 전송 가능

### FMS

- waiting queue의 중복 `task_id` 거절
- active_tasks의 중복 `task_id` 거절
- AMR에 할당하면 waiting → active 이동
- AMR 완료 이벤트 수신 시 active에서 제거

### 결과

```text
Task 유실 방지
+
Task 중복 등록 방지
+
실행 중 Task 추적
```

세 가지를 동시에 만족하도록 변경했다.

---

# 5. 새로 추가된 파일

## 5.1 `nav2_params.yaml`

Nav2 전체 동작을 프로젝트 환경에 맞게 설정하기 위해 새로 생성했다.

주요 설정 대상:

- Controller Server
- Planner Server
- BT Navigator
- Local Costmap
- Global Costmap
- AMCL 연동 Frame
- `/amr/odom`
- `/front_2d_lidar/scan`
- Velocity Smoother
- Collision Monitor

---

## 5.2 `factory_map.yaml`

Map Server가 `factory_map.pgm`을 읽기 위한 메타데이터 파일이다.

현재 프로젝트의 공장 World 크기와 Nav2 좌표계를 맞추기 위해 사용한다.

---

## 5.3 `factory_map.pgm`

Nav2 Global Costmap과 AMCL이 사용하는 실제 Occupancy Grid 이미지다.

기존에는 Isaac 내부 좌표만 사용해 직접 이동했기 때문에
별도의 2D Map이 필요하지 않았지만,
Nav2를 적용하면서 Map Server가 읽을 정적 지도가 필요해져 새로 생성했다.

---

## 5.4 `area_detection_node.py`

AMR의 `/amr/odom`을 구독하여
Assembly Cell A/B/C 영역 진입을 판단한다.

```text
/amr/odom
 ↓
Area Detection Node
 ↓
/assembly/part_arrived
 ↓
Assembly Node
```

동일 Area 안에서 이벤트가 반복 발행되지 않도록
현재 Area 상태를 유지한다.

---

# 6. Nav2에서 사용하는 외부 제공 파일

다음 파일/실행 프로그램은 우리 프로젝트에서 새로 만든 것이 아니라
ROS 2 Jazzy / Nav2 패키지가 제공한다.

| 항목 | 출처 | Git 저장 여부 |
|---|---|---|
| `navigation_launch.py` | `nav2_bringup` | 저장하지 않음 |
| `map_server` | `nav2_map_server` | 저장하지 않음 |
| `amcl` | `nav2_amcl` | 저장하지 않음 |
| `NavigateToPose` | `nav2_msgs` / Nav2 | 저장하지 않음 |

현재 Nav2 실행:

```bash
ros2 launch nav2_bringup navigation_launch.py \
use_sim_time:=true \
params_file:=$(git rev-parse --show-toplevel)/config/nav2_params.yaml
```

여기서 `navigation_launch.py`는 보통 다음 설치 경로에 존재한다.

```text
/opt/ros/jazzy/share/nav2_bringup/launch/navigation_launch.py
```

즉 현재 프로젝트에서는 별도의 Custom Nav2 Launch 파일을 만들지 않았고,
공식 Nav2 Launch 파일에 우리 프로젝트용 `nav2_params.yaml`을 전달하는 방식으로 사용한다.

---

# 7. 현재 ROS2 통신 구조

```text
Assembly Node
    │
    │ /assembly/request
    ▼
FMS + cuOpt
    ▲
    │ /amr/status
    │
    └──────────── AMR Node
                     │
                     │ /fms/request_task
                     ▼
                    FMS

AMR Node
    │
    │ NavigateToPose
    ▼
Nav2
    │
    │ /cmd_vel
    ▼
Isaac Sim
    │
    ├── /amr/odom
    ├── /front_2d_lidar/scan
    ├── /tf
    ├── /tf_static
    └── /clock
```

---

# 8. Task Lifecycle

```text
Assembly Task 생성
       ↓
/assembly/request
       ↓
FMS waiting queue
       ↓
cuOpt 최적화
       ↓
AMR /fms/request_task
       ↓
waiting → active_tasks
       ↓
AMR NavigateToPose
       ↓
Pickup
       ↓
Delivery
       ↓
/amr/status
DELIVERY_COMPLETE / MISSION_COMPLETE
       ↓
active_tasks 제거
       ↓
Task 완료
```

---

# 9. TF 구조

Nav2/AMCL에서 사용하는 TF:

```text
map
 ↓
odom
 ↓
base_link
 ↓
front_2d_lidar
```

담당:

```text
AMCL
└── map -> odom

Isaac Sim
├── odom -> base_link
└── base_link -> front_2d_lidar
```

---

# 10. PC 분리 실행 정책

## PC1 — Simulation PC

실행:

```text
Isaac Sim
└── standalone_amr_world.py
```

담당:

- Nova Carter Simulation
- Physics
- LiDAR
- Odometry
- TF
- Simulation Clock
- `/cmd_vel` 수신

---

## PC2 — Control / Optimization PC

실행:

- FMS + cuOpt
- Map Server
- AMCL
- Nav2
- Area Detection Node
- AMR Node
- Assembly Node

보유 설정:

- `nav2_params.yaml`
- `factory_map.yaml`
- `factory_map.pgm`

---

# 11. 현재 실행 순서

## Terminal 1 — FMS + cuOpt

```bash
source "${CUOPT_ENV:-$HOME/cuopt_env}/bin/activate"
source /opt/ros/jazzy/setup.bash
source "$(git rev-parse --show-toplevel)/ros2_ws/install/setup.bash"

cd "$(git rev-parse --show-toplevel)"
python -m fms.scenario0_fms
```

## Terminal 2 — Isaac Sim World

```bash
source /opt/ros/jazzy/setup.bash
source "$(git rev-parse --show-toplevel)/ros2_ws/install/setup.bash"

cd "$(git rev-parse --show-toplevel)"

"${ISAAC_SIM_DIR:-$HOME/isaacsim}/python.sh" \
simulation/isaac_sim/standalone_amr_world.py
```

## Terminal 3 — Nav2 Map Server

```bash
source /opt/ros/jazzy/setup.bash
source "$(git rev-parse --show-toplevel)/ros2_ws/install/setup.bash"

ros2 run nav2_map_server map_server \
--ros-args \
-p use_sim_time:=true \
-p yaml_filename:=$(git rev-parse --show-toplevel)/ros2_ws/src/navigation/maps/factory_map.yaml
```

Map Server는 Lifecycle Node이므로 현재 실행 환경에서는
configure/activate 과정이 필요하다.

## Terminal 4 — AMCL

```text
scan_topic   = /front_2d_lidar/scan
base_frame   = base_link
odom_frame   = odom
global_frame = map
```

초기 위치:

```text
x = 0.0
y = 0.0
yaw = 0
```

## Terminal 5 — Nav2

```bash
source /opt/ros/jazzy/setup.bash
source "$(git rev-parse --show-toplevel)/ros2_ws/install/setup.bash"

ros2 launch nav2_bringup navigation_launch.py \
use_sim_time:=true \
params_file:=$(git rev-parse --show-toplevel)/config/nav2_params.yaml
```

## Terminal 6 — Area Detection

```bash
ros2 run assembly_cell area_detection_node
```

## Terminal 7 — AMR Node

```bash
ros2 run amr_control amr_node
```

## Terminal 8 — Assembly Node

```bash
ros2 run assembly_cell assembly_node
```

---

# 12. 핵심 설계 정책

### 1. FMS와 Navigation 역할 분리

```text
FMS
= 무엇을 할지 결정

Nav2
= 어떻게 이동할지 결정
```

### 2. AMR Pull 방식

FMS가 AMR에 강제로 작업을 Push하지 않고,
IDLE AMR이 FMS에 다음 작업을 요청한다.

### 3. Assembly 재전송 허용

FMS가 준비되지 않은 경우 동일 Task를 유지하고 다시 요청할 수 있다.

### 4. FMS Task 중복 방지

같은 `task_id`가 waiting 또는 active에 있으면 새로 등록하지 않는다.

### 5. Waiting / Active 분리

```text
waiting
= 아직 미할당

active
= AMR 수행 중
```

### 6. 이동 성공 판정은 Nav2가 담당

AMR Node에서 직접 거리 계산을 하지 않고
`NavigateToPose` Action Result를 사용한다.

### 7. Isaac Sim은 Robot/Physics Layer 역할

Isaac Sim은 경로를 결정하지 않는다.

Nav2가 만든 `/cmd_vel`을 실제 AMR 물리 모델에 적용하고,
센서와 Pose 데이터를 다시 ROS2로 반환한다.

---

# 13. 팀원 확인 필요 항목

아래 파일은 최종 Git diff로 한 번 더 확인하는 것이 좋다.

- `interfaces/srv/RequestTask.srv`
- `assembly_node.py`의 정확한 변경 라인
- `amr_control/setup.py`
- `assembly_cell/setup.py`
- 관련 `package.xml`
- 기타 launch/config 파일

확인 명령:

```bash
git status
git diff
git diff --stat
git log --oneline --decorate -10
```

Nav2 작업 시작 직전 커밋을 알고 있다면:

```bash
git diff <NAV2_작업_전_커밋>..HEAD --stat
git diff <NAV2_작업_전_커밋>..HEAD
```

---

## 한 줄 요약

> 기존 직접 좌표 제어 구조를 FMS → AMR Node → Nav2 → `/cmd_vel` → Isaac Sim 구조로 변경했고, 이후 Assembly의 Task 재전송을 허용하면서도 FMS가 `waiting queue`와 `active_tasks`를 분리하여 동일 `task_id`의 중복 등록을 막고 작업 완료까지 Lifecycle을 추적하도록 보완했다.
