# DollyDockingNode

## 역할

`DollyDockingNode`는 IW Hub의 전방 카메라 영상으로 Dolly 하단 진입부 pose를
추정하고 `/cmd_vel`을 제어하는 `DockDolly` Action Server다. 노드는 계속
실행되지만 `/dock_dolly` Goal이 수락된 동안에만 non-zero Twist를 발행할 수
있다.

## Vision과 제어 pipeline

```text
Camera Image
  -> YOLO Pose
  -> keypoint confidence filter
  -> solvePnPRansac / solvePnP refine
  -> distance_m / lateral_m / yaw_deg
  -> P controller
  -> APPROACH / ALIGN_LATERAL / ALIGN_YAW
  -> vision handoff
  -> open-loop FINAL_ENTRY
  -> STOP
  -> DOCKING_COMPLETE
```

기존 YOLO/PnP 계산, lateral/yaw convention, P controller와 close-range fallback
조건은 그대로 사용한다.

## DockDolly Action Server

- Action 이름: `/dock_dolly`
- Goal: `amr_id`, `task_id`
- Feedback: `state`, `distance_m`, `lateral_m`, `yaw_deg`
- 성공 Result: `success=true`, `message=DOCKING_COMPLETE`
- 실패/취소 Result: `success=false`와 원인 message

동시에 하나의 Goal만 허용한다. Goal 수락 시
`final_entry_active`, `final_entry_complete`, `final_entry_start_time`,
`last_valid_pose`, `invalid_pose_count`, 최신 pose/feedback 및 watchdog 시간을
초기화한다.

## 상태 흐름

```text
IDLE
  -> Goal accepted
  -> WAITING_FOR_VISION
  -> APPROACH / ALIGN_LATERAL / ALIGN_YAW
  -> FINAL_ENTRY
  -> DOCKING_COMPLETE
  -> IDLE
```

`IDLE`에서는 image callback이 즉시 반환하므로 inference와 제어를 실행하지
않고 `/cmd_vel`도 발행하지 않는다. 취소, camera watchdog timeout 또는 runtime
exception이 발생하면 controller 권한을 회수하고 STOP을 발행한 뒤 Action을
`canceled` 또는 `aborted` 상태로 종료한다. `publish_cmd_vel=false`이면 master
safety switch가 모든 실제 Twist 발행을 차단한다.

## Concurrency

노드는 `MultiThreadedExecutor(num_threads=4)`를 사용한다. Action Server는
`ReentrantCallbackGroup`에서 실행되어 execute callback이 Event를 기다리는
동안 cancel callback을 받을 수 있다. Image와 watchdog은 각각 별도의
`MutuallyExclusiveCallbackGroup`에서 동작한다. Image callback이 Vision/control
상태를 갱신하고 완료 Event를 설정하며, execute callback은 최신 snapshot을
주기적으로 Feedback으로 보낸다. 공유 Mission 상태는 `threading.Lock`으로
보호한다.

## 설정과 runtime 경로

튜닝은 `config/docking.yaml`에서 관리한다. 주요 parameter는 다음과 같다.

- handoff: `handoff_distance_m`, `done_lateral_m`, `done_yaw_deg`
- 주행 허용 범위: `drive_lateral_limit_m`, `drive_yaw_limit_deg`
- P control: `k_distance`, `k_yaw`, `k_lateral`
- 속도 제한: `max/min_linear_mps`, `max_angular_rps`, `min_align_angular_rps`
- 좌표 convention: `angular_sign`, `yaw_sign`
- fallback: `invalid_frames_before_handoff`, `fallback_*`
- final entry: `enable_final_entry`, `final_entry_distance_m`,
  `final_entry_speed_mps`
- safety: `image_watchdog_timeout_s`, `publish_cmd_vel`

모델, `camera_intrinsics.npz`, `vision_config.py`는 먼저 설치된 package share에서
찾고, 개발 중에는 source tree로 fallback한다. ROS executable은 검증된
`numpy`, OpenCV, PyTorch, Ultralytics 조합을 유지하기 위해
`vision/.venv/bin/python`을 사용하는 환경에서 build/run한다. Control/FMS venv나
Isaac Sim Python 환경과 합치지 않는다.

## `/cmd_vel` 제어권

AMR Mission은 Nav2 `NavigateToPose` Result가 종료된 후에만 DockDolly Goal을
보낸다. Goal이 active인 동안 Vision이 `/cmd_vel`을 사용하고, 완료·취소·실패
시 STOP한 뒤 권한을 반납한다. 이번 단계에서는 cmd_vel mux를 사용하지 않으며
Nav2와 Vision의 순차 실행으로 제어권 중첩을 방지한다.

## 실제 통합시험

먼저 Action interface와 두 runtime package를 build한다.

```bash
cd ~/cobot3_projects/cobot3_project2/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select interfaces amr_control
source install/setup.bash
source ../vision/.venv/bin/activate
colcon build --symlink-install --packages-select vision_docking
```

Vision terminal에서는 실제 `/cmd_vel` 사용을 명시적으로 활성화한다.

```bash
cd ~/cobot3_projects/cobot3_project2
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source vision/.venv/bin/activate
export ROS_DOMAIN_ID=129
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch vision_docking docking.launch.py publish_cmd_vel:=true
```

Control terminal은 기존 FMS/Nav2/Isaac Sim이 준비된 뒤 실행한다.

```bash
cd ~/cobot3_projects/cobot3_project2
source scripts/env/control_env.sh
ros2 run amr_control amr_node
```

정상 결과는 AMR status가 `MOVING_TO_PICKUP -> PRE_DOCKING -> DOCKING ->
DOCKING_COMPLETE -> ARRIVED_PICKUP -> LOADING` 순서로 진행하고, Vision Action
Result가 `success=true`, `message=DOCKING_COMPLETE`인 것이다. 실제 camera와
Nav2/FMS가 없는 환경에서는 로봇 이동 통합시험을 실행하지 않는다.
