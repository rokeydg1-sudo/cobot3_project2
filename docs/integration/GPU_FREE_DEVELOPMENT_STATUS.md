# GPU-Free Development Status

기준일: 2026-08-27  
환경: Ubuntu 24.04, ROS 2 Jazzy, NVIDIA GPU 없음

## IMPLEMENTED_WITHOUT_GPU

- FMS recovery Node -> `Task.start` approach route와 기존 `Task.start` -> `Task.goal` delivery route.
- `RequestTask.srv` 두 route 병렬 배열/cost 계약과 endpoint/길이 검증 helper.
- AMR `NavigateThroughPoses` full Node route 주행, 첫 current Node skip, single-Node no-op.
- 수평/수직/대각선/음수/마지막 incoming-edge heading과 quaternion 생성.
- 기존 `DockDolly` 뒤 `LiftDolly(LIFT_UP)`, odom reverse, delivery, `LiftDolly(LIFT_DOWN)` 상태 머신.
- parameterized odom/cmd_vel/Nav2/Dock/Lift/FMS/VisualizeRoute endpoint.
- Fake FMS, VisualizeRoute, NavigateThroughPoses, DockDolly, LiftDolly, Odom/kinematics와 Real `AMRNode` runner.
- success 1-cycle 및 Dock failure, Lift Up failure, reverse timeout 자동 판정.
- scene-independent `cobot3_bringup/launch/integration.launch.py`와 self-judging `mission_mock.launch.py`.
- `scripts/run_gpu_free_tests.sh` one-command build/unit/mock summary.

## 실제 GPU-free validation 결과

```text
PASS: interfaces/amr_control/fms/mission_mock/cobot3_bringup build
PASS: RequestTask, DockDolly, LiftDolly interface generation
PASS: official Jazzy NavigateThroughPoses schema 확인
PASS: pure unit tests 16 passed
SKIP: cuOpt GPU runtime 1
PASS: mock success 1-cycle
PASS: mock Dock failure
PASS: mock Lift Up failure
PASS: mock reverse timeout
```

success scenario에서 확인된 핵심 증거:

```text
NavigateThroughPoses calls = 2
poses per call              = 2, 2
VisualizeRoute calls        = 2
DockDolly calls             = 1
LiftDolly commands          = UP, DOWN
reverse odom distance       = 0.301 m (test override 0.30 m)
negative cmd_vel            = observed
terminal zero cmd_vel       = observed
FMS task assignments        = 1
final state                 = IDLE
next FMS request            = observed
```

production `return_distance_m` 기본값은 `3.0 m`이며 mock에서만 `0.30 m`로 override한다.

## 개발 환경 경고

이 host의 `/opt/ros/jazzy`에는 `nav2_msgs`와 `geographic_msgs`가 system package로 설치되어 있지 않았다. 실제 검증은 Debian package를 `/tmp`에 non-root로 추출한 overlay에서 수행했다. `scripts/run_gpu_free_tests.sh`도 system package가 없으면 같은 임시 overlay를 준비한다. 이는 repository나 system ROS 설치를 변경하지 않는다.

root `.venv`도 현재 없으므로 ROS build와 GPU-free test는 `/usr/bin/python3`를 사용했다. requirements lock 파일은 변경하지 않았다.

## RUNTIME_ONLY_BLOCKER

- Main Factory USD와 공식 Isaac Sim 5.1.0 startup entrypoint.
- Main Scene camera, odom, cmd_vel, scan, TF, `/clock` ROS graph.
- `/AMR1/*`와 root topic의 최종 namespace/remap.
- IW Hub lift Prim path, DOF 이름, lower/upper limit, UP/DOWN 방향과 실제 Dolly lifting.
- Main Scene camera에서 YOLO/PnP/P controller/final entry 실제 motion.
- map/localization, `map -> odom -> base_link`, sensor/costmap이 있는 실제 Nav2.
- loaded Dolly footprint/inflation과 목적지 주행 가능성.
- `RUNTIME_CHECK_DOLLY_UNDER_AMR`: Lift Down 후 Dolly 아래에서 다음 Mission 출발.
- FMS -> Nav2 -> Vision -> Lift -> Reverse -> Nav2 -> Lift Down physical E2E.

Main Scene/Joint/Prim 이름, footprint, namespace는 추측해 확정하지 않았다. Isaac Sim, cuOpt GPU solver, YOLO inference는 실행하지 않았다.
