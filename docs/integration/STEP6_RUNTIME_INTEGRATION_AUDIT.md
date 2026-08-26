# Step 6 Runtime Integration Audit

> Historical audit note (2026-08-27): 이 문서는 변경 전 `NavigateToPose` 기반
> runtime 상태를 보존한다. 현재 구현된 two-route `NavigateThroughPoses`,
> `LiftDolly`, odom reverse와 GPU-free 검증 결과는
> `SCENARIO0_RUNTIME_FLOW.md`, `GPU_FREE_DEVELOPMENT_STATUS.md`,
> `STEP6_BLOCKER_RESOLUTION.md`를 기준으로 한다.

## 1. Executive Summary

### Overall verdict: BLOCKER

Step 5의 ROS 코드 계약 자체는 연결되어 있다.

```text
FMS RequestTask
  -> AMR VisualizeRoute Client
  -> Nav2 NavigateToPose Client
  -> AMR DockDolly Client
  -> Vision DockDolly Server
```

다만 실제 End-to-End runtime을 시작하기에는 다음 연결이 확정되지 않았다.

1. repository가 자동으로 여는 **확정 Main factory scene**이 없다.
2. Main factory scene에 camera publisher, `/cmd_vel` subscriber, odometry, TF,
   `/clock`, scan graph가 포함됐는지 정적으로 확인되지 않는다.
3. Isaac runtime 기록은 `/AMR1/*` namespace를 사용하지만 Control/Nav2/Vision
   코드는 대부분 root topic인 `/amr/odom`, `/cmd_vel`, `/tf`를 기대한다.
4. Nav2 최소 launch는 navigation server만 실행하며 TF와 `/clock` producer를
   제공하지 않는다.
5. FMS가 현재 `TEST_MODE=True`에서 임의 reachable Node를 pickup으로 선택하므로,
   해당 Node가 Dolly에서 2.0~3.5 m 떨어진 Pre-Docking pose라는 보장이 없다.

판정 용어는 다음과 같다.

- `STATICALLY_VERIFIED`: source/config에서 producer와 consumer 계약을 확인했다.
- `RUNTIME_VERIFICATION_REQUIRED`: binary USD 또는 외부 runtime 상태라 실행 중
  read-only ROS introspection이 필요하다.
- `BLOCKER`: 실제 이동 Goal을 보내기 전에 계약을 확정하거나 연결해야 한다.

이번 audit에서는 Python, CMake, package.xml, USD 및 runtime 코드를 수정하지
않았다.

## 2. Main Isaac Sim entrypoint

### 2.1 Scene과 실행 진입점

| 항목 | 경로 | 판정 | 근거 |
|---|---|---|---|
| Factory scene 후보 | `simulation/isaac_sim/assets/Assembly/World0.usd` | RUNTIME_VERIFICATION_REQUIRED | repository 내 유일한 `World0` factory asset 후보지만 어떤 script도 이 파일을 참조하거나 자동으로 열지 않는다. USD crate binary다. |
| AMR asset 후보 | `simulation/isaac_sim/assets/amr/Nova_Carter_ROS.usd` | RUNTIME_VERIFICATION_REQUIRED | asset은 존재하지만 현재 standalone/main script에서 load하지 않는다. USD crate binary다. |
| Vision 독립 scene | `vision/isaac_sim/scenes/Collected_Vision_docking_setup_v01/Vision_docking_setup_v01.usd` | STATICALLY_VERIFIED(독립 scene 존재) / RUNTIME_VERIFICATION_REQUIRED(graph 내용) | IW Hub와 Dolly Vision 실험용으로 수집된 별도 binary scene이다. Main factory scene에 compose됐다는 참조는 없다. |
| Isaac integration launcher | `simulation/isaac_sim/run_isaac_sim.sh` | BLOCKER | 별도 `interfaces`를 build/source하고 Isaac GUI를 실행하지만 scene을 지정하지 않는다. 기본 `ISAAC_SIM_DIR`은 존재하지 않는 6.0.1 경로다. `$@`로 scene을 받을 수는 있다. |
| 단순 GUI launcher | `scripts/start_simulation.sh` | WARN | 실제 존재하는 `~/isaacsim/isaac-sim.sh`를 실행하지만 ROS environment, interface overlay, scene, extension을 설정하지 않는다. |
| Standalone samples | `simulation/isaac_sim/scripts/assembly_cells_world.py`, `assembly_cell_test.py`, `assembly_ros_test.py` | STATICALLY_VERIFIED(test only) | 빈 `World`에 색상 Cube Assembly Cell을 생성하는 sample이다. Factory USD, AMR articulation, Vision camera, odometry 또는 cmd_vel graph를 load하지 않는다. |

실제 설치는 `/home/rokey/isaacsim`이며 `VERSION`은
`5.1.0-rc.19+release.26219.9c81211b.gl5.1.0-rc.19`이다. 따라서 target Isaac
Sim 5.1.0과 일치한다. 반면 `simulation/isaac_sim/run_isaac_sim.sh`의 기본
6.0.1 경로는 현재 host에 존재하지 않는다.

### 2.2 ROS2 Bridge

- `simulation/isaac_sim/scripts/assembly_ros_test.py`는
  `enable_extension("isaacsim.ros2.bridge")`를 호출한다. 이 파일은 test scene다.
- `simulation/isaac_sim/ExtNodeMapBuild/config/extension.toml`은
  `isaacsim.ros2.bridge`를 dependency로 선언한다. ExtNodeMapBuild가 활성화되면
  Bridge dependency도 활성화된다.
- `vision/isaac_sim/vision_docking/setup/create_iwhub_cmd_vel_graph.py`는
  `isaacsim.ros2.bridge`, `isaacsim.core.nodes`,
  `isaacsim.robot.wheeled_robots`를 즉시 활성화한다.
- Main factory scene startup에서 ExtNodeMapBuild를 enable하는 committed command나
  config는 없다.

따라서 Bridge 자체 사용 위치는 확인됐지만, **Main startup에서의 활성화는
RUNTIME_VERIFICATION_REQUIRED**다.

### 2.3 Robot, camera, cmd_vel, odometry

| 기능 | 정적으로 확인된 위치 | Main factory 판정 |
|---|---|---|
| IW Hub root | `vision/isaac_sim/vision_docking/setup/create_iwhub_cmd_vel_graph.py`의 `/World/iw_hub_sensors` | Vision 독립 scene 전용. Main factory 포함 여부 미확인 |
| First-person camera prim | `ros2_ws/src/vision_docking/config/vision_config.py`의 `/World/iw_hub_sensors/camera_mount/transporter_camera_first_person` | prim 계약만 확인. Main factory 존재 여부 미확인 |
| Camera ROS publisher | source code에서 생성 위치 없음 | BLOCKER / RUNTIME_VERIFICATION_REQUIRED |
| `/cmd_vel` subscriber graph | `vision/isaac_sim/vision_docking/setup/create_iwhub_cmd_vel_graph.py` | Vision 독립 scene에 수동 생성·저장하는 script. Main factory 적용 근거 없음 |
| Odometry publisher | source code에서 생성 위치 없음 | BLOCKER / RUNTIME_VERIFICATION_REQUIRED |
| TF publisher | source code에서 생성 위치 없음 | BLOCKER / RUNTIME_VERIFICATION_REQUIRED |
| `/clock` publisher | source code에서 생성 위치 없음 | BLOCKER / RUNTIME_VERIFICATION_REQUIRED |
| `/front_2d_lidar/scan` publisher | source code에서 생성 위치 없음 | full Nav2 config 사용 시 BLOCKER |

`create_iwhub_cmd_vel_graph.py`는 `/cmd_vel` `geometry_msgs/Twist`를 differential
controller와 articulation controller에 연결하고 wheel radius `0.08 m`, wheel
distance `0.58 m`를 사용한다. 이는 독립 Vision scene에서 정적으로 확인된
계약이며 Main scene 계약으로 간주할 수 없다.

### 2.4 `/visualize_route` Action Server

`STATICALLY_VERIFIED`:

- 구현: `simulation/isaac_sim/ExtNodeMapBuild/ext_node_map_build/extension.py`
- Action: `/visualize_route`
- 타입: `interfaces/action/VisualizeRoute`
- NodeMap service: `/get_node_map`
- NodeMap event: `/node_map_changed`
- Stage prim roots: `/World/WaypointGraph/Nodes`, `/World/WaypointGraph/Edges`
- ROS callback은 Isaac main update event에서 `rclpy.spin_once()`로 처리한다.

`simulation/isaac_sim/run_isaac_sim.sh`가 build하는 reduced interface workspace는
`VisualizeRoute`, `GetNodeMap`, `NodeMapChanged`를 root workspace와 동일하게
보유한다. `DockDolly`는 이 reduced package에 없으므로 이 overlay를 source한
터미널에서 AMR/Vision process를 실행하면 안 된다. Isaac, Control, Vision은
분리된 terminal/environment를 유지해야 한다.

## 3. Nav2 bringup 분석

### 3.1 실제 standalone launch

실행 파일은 `nav2_minimal_test/nav2_minimal.launch.py`이고 parameter 파일은
`nav2_minimal_test/nav2_minimal_params.yaml`이다.

| 구성요소 | 제공 위치 | 판정 |
|---|---|---|
| `controller_server` | `nav2_controller/controller_server` | STATICALLY_VERIFIED |
| `planner_server` | `nav2_planner/planner_server` | STATICALLY_VERIFIED |
| `behavior_server` | `nav2_behaviors/behavior_server` | STATICALLY_VERIFIED |
| `bt_navigator` | `nav2_bt_navigator/bt_navigator` | STATICALLY_VERIFIED |
| navigation lifecycle manager | `nav2_lifecycle_manager/lifecycle_manager` | STATICALLY_VERIFIED; 위 4개만 activate |
| `/navigate_to_pose` | `bt_navigator` | STATICALLY_VERIFIED |
| `/cmd_vel` | `controller_server`, `enable_stamped_cmd_vel: false` | root `/cmd_vel`로 해석될 것으로 예상; runtime 확인 필요 |
| `map_server` | launch에 없음 | 의도적으로 미사용 |
| AMCL/localization | launch에 없음 | 별도 provider도 repository에서 찾지 못함 |
| map → odom TF | launch에 없음 | BLOCKER unless Isaac/runtime provides it |
| odom → base_link TF | launch에 없음 | BLOCKER unless Isaac/runtime provides it |
| `/scan` | 최소 config에서는 사용하지 않음 | 최소 test에서는 불필요; full config에서는 필요 |
| `/clock` | launch에 없음, 모든 node가 `use_sim_time: true` | BLOCKER unless Isaac publishes it |

최소 config는 local/global costmap에서 obstacle/static layer를 사용하지 않고
inflation layer만 사용한다. Global costmap도 rolling window이므로 `map_server`와
AMCL이 없는 것 자체는 의도된 설계다. 그러나 `global_frame=map`,
`local_frame=odom`, `robot_base_frame=base_link`이므로 유효한 TF chain은 여전히
필수다.

```text
map -> odom -> base_link
```

현재 repository에는 이 chain을 publish하는 launch 또는 source code가 없다.
Isaac binary Action Graph가 제공하는지는 runtime에서 확인해야 한다.

### 3.2 다른 Nav2 파일과의 관계

- `config/nav2_params.yaml`은 static/obstacle layer, `/front_2d_lidar/scan`,
  velocity smoother, collision monitor, docking server 설정까지 포함한다.
- 현재 `nav2_minimal_test/nav2_minimal.launch.py`는 이 파일을 사용하지 않는다.
- `ros2_ws/src/navigation/maps/factory_map.yaml`과 `factory_map.pgm`이 존재하지만
  이를 실행하는 `map_server` launch는 없다.
- `config/world_map.yaml`의 모든 location 좌표는 `(0.0, 0.0)`이며 현재
  NodeMap 기반 FMS가 사용하지 않는다.
- system에는 `nav2_map_server`와 `nav2_amcl` package가 설치돼 있지만, 설치
  여부가 runtime 구성 또는 TF 공급을 의미하지는 않는다.

### 3.3 Nav2 단독 판정

`nav2_minimal.launch.py`만으로 navigation server와 `/navigate_to_pose` Action
endpoint는 만들 수 있다. 하지만 Main Isaac scene에서 실제 Goal을 수행하기
위한 `/clock`, odometry, TF 및 `/cmd_vel` subscriber가 확인되지 않아 **현재
상태만으로 실제 NavigateToPose 성공을 보장할 수 없다**.

`작업로그_종합.md`도 마지막 검증 당시 `/navigate_to_pose` Action Server가 없어
실제 주행을 시작하지 못했고, 남은 작업으로 AMR1 Nav2/cmd_vel/odometry/TF/scan
연결을 명시한다.

## 4. ROS interface contract matrix

| 이름 | Producer/Server | Consumer/Client | 정적 판정 | 문제 |
|---|---|---|---|---|
| `/fms/request_task` | `ros2_ws/src/fms/fms/fleet_management_system.py` | `ros2_ws/src/amr_control/amr_control/amr_node.py` | PASS | 타입 `interfaces/srv/RequestTask` 일치 |
| `/amr/status` | `amr_node.py` | `fleet_management_system.py` | PASS | root topic 일치 |
| `/visualize_route` | `ExtNodeMapBuild/extension.py` | `amr_node.py` | PASS 조건부 | Extension enable과 동일 ROS domain 필요 |
| `/navigate_to_pose` | `nav2_minimal_test/nav2_minimal.launch.py`의 `bt_navigator` | `amr_node.py` | WARN | endpoint는 일치하지만 TF/clock/runtime 미확인 |
| `/dock_dolly` | `ros2_ws/src/vision_docking/vision_docking/dolly_docking_node.py` | `amr_node.py` | PASS | 타입 `interfaces/action/DockDolly` 일치 |
| `/vision/front_camera/image_raw` | source producer 없음 | `dolly_docking_node.py` | BLOCKER | Main scene camera publisher 미확인 |
| `/amr/odom` | source producer 없음 | AMR, Nav2 minimal/full params, Assembly area detection | BLOCKER | 기존 runtime 기록은 `/AMR1/odom` |
| `/AMR1/odom` | binary/runtime 추정 | 현재 committed client 없음 | BLOCKER | AMR에 대한 수동 remap만 작업로그에 존재 |
| `/cmd_vel` | Nav2 controller와 active Vision Docking | Vision 독립 scene graph | BLOCKER 조건부 | Main scene subscriber 미확인; runtime 기록은 `/AMR1/cmd_vel` |
| `/AMR1/cmd_vel` | 현재 committed publisher 없음 | binary/runtime 추정 | BLOCKER | Nav2/Vision remap 또는 namespace launch 없음 |
| `/front_2d_lidar/scan` | source producer 없음 | `config/nav2_params.yaml` | WARN/BLOCKER(full config) | 최소 Nav2 config는 scan을 사용하지 않음 |
| `/tf`, `/tf_static` | source producer 없음 | Nav2 minimal | BLOCKER | runtime 기록은 `/AMR1/tf`, `/AMR1/tf_static` |
| `/clock` | source producer 없음 | `use_sim_time: true`인 Nav2 node | BLOCKER | Main Isaac publisher 확인 필요 |

### 가장 중요한 namespace mismatch

기존 실제 검증 기록은 다음 Isaac topic을 사용한다.

```text
/AMR1/odom
/AMR1/cmd_vel
/AMR1/scan
/AMR1/tf
/AMR1/tf_static
```

현재 코드/최소 Nav2는 다음 root 계약을 사용한다.

```text
/amr/odom
/cmd_vel
/tf
/tf_static
```

`작업로그_종합.md`에는 AMR만 다음처럼 수동 remap한 기록이 있다.

```bash
ros2 run amr_control amr_node --ros-args -r /amr/odom:=/AMR1/odom
```

Nav2 `/cmd_vel`, TF, scan과 Vision `/cmd_vel`에 대한 committed namespace/remap은
없다. Main scene의 실제 graph topic을 먼저 확인한 뒤 root contract 또는
`/AMR1/*` contract 중 하나로 통일해야 한다.

## 5. Runtime environment matrix

| 영역 | Python / environment | ROS 2 | CUDA/GPU | 실행 진입점 |
|---|---|---|---|---|
| Isaac Sim | `/home/rokey/isaacsim/python.sh`, Python 3.11.13, Isaac 자체 runtime | Isaac ROS2 Bridge + system Jazzy environment/interface overlay | Isaac Sim GPU runtime 필요 | `/home/rokey/isaacsim/isaac-sim.sh`; repository wrapper는 `simulation/isaac_sim/run_isaac_sim.sh` |
| Control/FMS/AMR | `<repo>/.venv/bin/python`, Python 3.12.3 | `/opt/ros/jazzy/setup.bash` + `ros2_ws/install/setup.bash`; `scripts/env/control_env.sh` 권장 | `cuopt-cu13==26.8.0`, CUDA 13 계열 | `ros2 run fms fleet_management_system`, `ros2 run amr_control amr_node` |
| Vision Docking | `<repo>/vision/.venv/bin/python`, Python 3.12.3 | system Jazzy + workspace overlay | `torch 2.11.0+cu128`, CUDA available=True | `ros2 launch vision_docking docking.launch.py publish_cmd_vel:=true` |
| Nav2 | `/usr/bin/python3` 및 `/opt/ros/jazzy` binaries, Python 3.12 | ROS 2 Jazzy system packages | 별도 Python CUDA 요구 없음 | `/usr/bin/python3 nav2_minimal_test/nav2_minimal.launch.py` |

설치된 ROS executable shebang도 환경 분리를 확인한다.

```text
fms              -> <repo>/.venv/bin/python
amr_control      -> <repo>/.venv/bin/python
vision_docking   -> <repo>/vision/.venv/bin/python
```

환경을 합치면 안 된다. 특히 Isaac용 reduced `interfaces` overlay를 Control 또는
Vision terminal에 source하지 않는다.

모든 terminal에서 다음 middleware contract를 동일하게 설정해야 한다.

```bash
export ROS_DOMAIN_ID=129
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

`simulation/isaac_sim/run_isaac_sim.sh`는 `ROS_DOMAIN_ID`가 unset이면 로그상 `0`을
사용할 수 있으므로 orchestrator가 실행 전에 반드시 허용 범위 `129`~`135`를
설정해야 한다.

## 6. Pre-Docking pose 분석

### 판정: BLOCKER

현재 데이터 흐름은 다음과 같다.

1. `ExtNodeMapBuild/extension.py`가 열린 Stage의 `/World/WaypointGraph/Nodes`
   world transform을 FMS에 전달한다.
2. `fleet_management_system.py`는 `TEST_MODE=True`다.
3. NodeMap load 후 `create_validation_task()`가
   `NodeMapGraph.choose_random_reachable_nodes()`로 임의의 start/goal Node를
   선택한다.
4. FMS는 선택된 start Node 좌표를 `pickup_x`, `pickup_y`로 응답한다.
5. `amr_node.py`는 그 좌표를 `map` frame Nav2 Goal로 그대로 사용하고,
   orientation을 항상 yaw `0`으로 설정한다.
6. Nav2 성공 후 같은 위치를 Pre-Docking pose로 간주하고 DockDolly를 시작한다.

따라서 현재 pickup은 **Dolly 중심 Node로 확인된 것도 아니고, Vision 시작용
Pre-Docking Node로 확인된 것도 아니다. 임의 reachable Node다.**

또한 committed text/config에는 Stage Node 좌표와 Dolly world pose의 대응표가
없다. `config/world_map.yaml`은 모든 좌표가 `0.0`이고 현재 FMS가 사용하지
않는다. Binary USD 내부 좌표는 이 audit에서 신뢰성 있게 추출할 수 없었다.

Vision 정상 범위 2.0~3.5 m와 camera heading을 만족하는지 확인할 근거가 없고,
`RequestTask.srv`에는 pickup yaw도 없다. 실제 이동 전에 다음 계약이 필요하다.

- Dolly reference pose
- 명시적인 Pre-Docking Node 또는 pose
- Dolly까지의 거리 2.0~3.5 m 검증
- 카메라 optical forward 방향 검증
- Nav2 Goal yaw 또는 Pre-Docking orientation 처리 방침

이번 audit에서는 Pre-Docking 계산이나 schema 변경을 구현하지 않았다.

## 7. Required process startup order

아래는 blocker 해결 후 orchestrator가 따라야 할 순서다.

1. **공통 middleware 환경 확정**
   - 모든 terminal에서 동일 `ROS_DOMAIN_ID`와 `rmw_fastrtps_cpp`를 설정한다.
2. **Isaac Sim Main factory runtime**
   - 확정된 Main USD를 연다.
   - ExtNodeMapBuild와 ROS2 Bridge를 활성화한다.
   - camera, odom, TF, clock, cmd_vel subscriber graph를 활성화한다.
3. **Nav2 minimal bringup**
   - `nav2_minimal_test/nav2_minimal.launch.py`를 실행한다.
   - TF/clock/odom이 유효한 뒤 lifecycle active를 확인한다.
4. **Vision Docking Action Server**
   - `vision/.venv`에서 `docking.launch.py`를 실행한다.
   - Main scene의 camera와 cmd_vel 계약을 확인한 후에만
     `publish_cmd_vel:=true`를 사용한다.
5. **FMS**
   - Control venv에서 `fleet_management_system`을 실행한다.
   - `/get_node_map`을 받아 revision과 Nodes/Edges load 완료를 확인한다.
6. **AMR Node**
   - odometry가 들어오고 `/visualize_route`, `/navigate_to_pose`, `/dock_dolly`
     server가 준비된 뒤 실행한다.
   - 선택한 namespace 계약에 맞는 remap을 적용한다.
7. **Assembly Cell / task source**
   - 현재 FMS는 `/assembly/request` subscriber가 없고 `TEST_MODE` validation
     task를 자동 생성한다. 실제 Assembly task source를 시작하기 전에 이 연결
     방침을 별도로 확정해야 한다.

## 8. Readiness checks

아래 명령은 Goal이나 velocity를 발행하지 않는 read-only 확인이다.

### Isaac ready

```bash
ros2 service type /get_node_map
ros2 action info /visualize_route
ros2 topic type /vision/front_camera/image_raw
ros2 topic hz /vision/front_camera/image_raw
ros2 topic type /amr/odom
ros2 topic hz /amr/odom
ros2 topic hz /clock
ros2 topic info /cmd_vel -v
```

namespace가 `/AMR1/*`이면 같은 검사를 해당 이름으로 수행한다. `/cmd_vel`
subscriber count가 1 이상이어야 하며 subscriber type은
`geometry_msgs/msg/Twist`여야 한다.

### TF ready

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

두 transform이 연속적으로 갱신돼야 한다.

### Nav2 ready

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /behavior_server
ros2 lifecycle get /bt_navigator
ros2 action info /navigate_to_pose
```

네 lifecycle node가 `active`이고 `/navigate_to_pose` server가 보여야 한다.

### Vision ready

```bash
ros2 action info /dock_dolly
ros2 node info /dolly_docking_node
```

Action Server와 camera subscription, `/cmd_vel` publisher가 보여야 한다. Image
publisher/subscriber QoS 호환성도 `ros2 topic info ... -v`로 확인한다.

### FMS/AMR ready

```bash
ros2 service type /fms/request_task
ros2 topic info /amr/status -v
ros2 node info /FleetManagementSystem
ros2 node info /amr_node
```

AMR는 odometry를 받은 뒤에만 FMS Task를 pull한다. FMS log에서 NodeMap revision,
Node count, Edge count load 완료가 먼저 확인돼야 한다.

## 9. Confirmed blockers

1. **Main scene/entrypoint 미확정**
   - `World0.usd`를 Main으로 여는 committed command가 없다.
2. **`run_isaac_sim.sh` 기본 설치 경로 오류**
   - 존재하지 않는 6.0.1 경로를 기본값으로 사용한다. 실제 설치는
     `/home/rokey/isaacsim` 5.1.0이다.
3. **Main scene ROS graph 미확인**
   - camera, cmd_vel, odom, TF, clock, scan producer/subscriber가 source로 확인되지
     않는다. Vision 독립 scene의 graph를 Main에 포함했다고 간주할 수 없다.
4. **AMR namespace mismatch**
   - runtime 기록 `/AMR1/*`와 코드 root topic 계약이 다르며 odom 이외의
     committed remap이 없다.
5. **Nav2 TF/clock 미제공**
   - 최소 launch는 server만 제공한다. Main Isaac 쪽 provider가 확인되지 않았다.
6. **Pre-Docking pose 미정의**
   - FMS `TEST_MODE`가 임의 start Node를 pickup으로 사용하고 yaw는 항상 0이다.
7. **실제 Assembly task source 미연결**
   - `AssemblyNode`는 `/assembly/request`를 publish하지만 현재 FMS에는 해당
     subscription이 없다. FMS는 validation task만 자동 생성한다.
8. **Extension enable 자동화 부재**
   - Main startup이 ExtNodeMapBuild를 확실히 enable한다는 committed 설정이 없다.

## 10. Runtime-only checks

다음은 binary USD 또는 running ROS graph가 필요하므로 정적 근거만으로 PASS
처리하지 않는다.

- Main으로 실제 여는 USD의 절대 경로와 root layer
- `/World/WaypointGraph/Nodes`, `/Edges` 존재 및 좌표
- IW Hub/Nova Carter articulation root와 wheel joint
- first-person camera prim과 optical orientation
- Dolly world pose와 Pre-Docking Node 거리/heading
- camera publisher의 topic, type, QoS, frame ID
- cmd_vel subscriber의 topic과 `Twist` type
- odometry topic과 `odom`/`base_link` frame ID
- `/clock` publish rate
- `/tf`, `/tf_static` namespace와 `map -> odom -> base_link` chain
- lidar scan topic/frame/QoS; full Nav2 config를 사용할 경우 필수
- ExtNodeMapBuild enable 상태와 `/visualize_route` availability
- Isaac reduced interface overlay와 Control interface의 DDS type compatibility
- Nav2가 실제로 publish하는 cmd_vel 이름과 다른 active cmd_vel publisher 유무

이 audit에서는 Action Goal, NavigateToPose Goal, DockDolly Goal 또는 Twist를
발행하지 않았다.

## 11. 다음 작업 추천 순서

1. Main factory USD 한 개를 공식 runtime scene으로 지정하고 정확한 실행 명령을
   문서화한다.
2. `run_isaac_sim.sh`의 실제 5.1.0 경로, `ROS_DOMAIN_ID=129`, Extension enable,
   Main scene open 절차를 확정한다. 이번 audit에서는 수정하지 않는다.
3. Main scene에서 camera, cmd_vel, odom, TF, clock graph를 read-only introspection으로
   확인하고 Vision 독립 graph와 차이를 기록한다.
4. single-AMR topic contract를 root 또는 `/AMR1/*` 중 하나로 결정하고 AMR,
   Nav2, Vision, Isaac 전체 remap 표를 확정한다.
5. Dolly 기준 Pre-Docking Node/pose와 heading을 정의하고 현재 random validation
   pickup을 실제 docking task와 분리한다.
6. Nav2 TF/clock provider를 확정한다. map_server/AMCL은 근거가 생기기 전까지
   임의로 추가하지 않는다.
7. FMS validation task와 Assembly `/assembly/request` 중 실제 E2E task source를
   결정한다.
8. 위 blocker를 해결한 뒤 readiness check만 먼저 수행하고, 모든 항목이 PASS인
   경우에만 제한된 실제 이동 통합시험 계획을 별도로 승인받는다.
