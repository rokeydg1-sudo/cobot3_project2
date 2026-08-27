# Step 6 Blocker Resolution

## IMPLEMENTED_WITHOUT_GPU

1. FMS two-route contract
   - actual odom nearest Active Node를 `latest_plan.recovery_node_id`로 재사용한다.
   - 기존 `NodeMapGraphManager.create_route()`로 recovery -> `Task.start`를 만든다.
   - cuOpt selected plan의 기존 `Task.start` -> `Task.goal` route는 delivery로 유지한다.
   - response array 길이와 두 route endpoint를 전송 전에 검증한다.
2. AMR mission integration
   - `NavigateThroughPoses` 두 번, 첫 Node skip, single-Node no-op, incoming-edge final yaw.
   - approach 시각화/주행 뒤 기존 `DockDolly`를 호출한다.
   - `LiftDolly` UP/DOWN Result 성공을 상태 전이의 필수 조건으로 사용한다.
   - Lift Up 후 direct negative Twist와 odom 변위로 기본 3 m 복귀하며 항상 STOP한다.
3. ROS contracts and parameterization
   - `LiftDolly.action`과 `RequestTask.srv`를 generated interface로 build했다.
   - `/amr/odom`, `/cmd_vel`, Nav2/Dock/Lift/FMS/VisualizeRoute endpoint를 parameter화했다.
   - `integration.launch.py`는 Isaac process/scene path 없이 ROS node 구조만 준비한다.
4. GPU-free automated evidence
   - Real `AMRNode` + Fake FMS/Nav2/Dock/Lift/Visualize/Odom success 1-cycle PASS.
   - Dock failure, Lift Up failure, reverse timeout의 금지 동작과 STOP PASS.
   - unit test 16 passed, cuOpt GPU runtime 1 skipped.

## RUNTIME_ONLY_BLOCKER

1. `RUNTIME_BLOCKER_MAIN_SCENE`
   - 팀이 전달할 Main Factory USD와 공식 Isaac 5.1.0 startup entrypoint가 없다.
   - camera/odom/cmd_vel/scan/TF/clock Action Graph가 Main Scene에서 확인되지 않았다.
2. 최종 ROS namespace/TF
   - `/AMR1/odom` 대 `/amr/odom`, `/AMR1/cmd_vel` 대 `/cmd_vel`과 namespaced TF를 runtime graph로 확정해야 한다.
3. actual Nav2 runtime
   - map, localization, planner/controller/behavior/BT lifecycle, scan, clock, `map -> odom -> base_link`를 확인해야 한다.
4. actual IW Hub lift
   - Prim path, lift DOF, limit, direction을 추측하지 않았다.
   - parameterized physical adapter는 실제 scene introspection 뒤 binding해야 한다.
5. actual Vision docking
   - Main Scene `iw_hub` first-person camera, intrinsics/extrinsics, YOLO/PnP 오차와 actual `/cmd_vel` motion 검증이 필요하다.
6. loaded Dolly navigation
   - 실제 Dolly 치수 기반 footprint/inflation과 목적지까지 주행 가능성을 확인해야 한다.
7. `RUNTIME_CHECK_DOLLY_UNDER_AMR`
   - Lift Down 후 Dolly 아래에서 local costmap이 다음 Nav2 출발을 허용하는지 확인해야 한다.
8. physical end-to-end
   - FMS -> NavigateThroughPoses -> Vision -> Lift -> Reverse -> NavigateThroughPoses -> Lift Down 전체 물리 1-cycle은 GPU workstation에서 실행한다.

## Host-only dependency note

현재 system ROS에는 `nav2_msgs`/`geographic_msgs`가 없었다. source schema를 추측하지 않고 Jazzy Debian package를 `/tmp` overlay로 로드해 interface/build/mock를 검증했다. one-command script도 동일한 non-root fallback을 제공한다. production workstation에는 정상적인 Jazzy Nav2 설치를 권장한다.

## GPU Runtime Update (2026-08-27)

위 `RUNTIME_ONLY_BLOCKER` 목록은 GPU-free checkpoint 시점의 기록이다. 현재 GPU
workstation에서는 다음 항목이 해소되었다.

- `RUNTIME_BLOCKER_MAIN_SCENE`: RESOLVED. Main USD와 Isaac Sim 5.1.0 launcher 및
  Python을 실제 확인했다.
- ROS namespace/TF: RESOLVED for minimal runtime. `/amr1/odom`,
  `/amr1/cmd_vel`, `/amr1/scan`, `map -> amr1/odom -> amr1/base_link`로 검증했다.
- actual Nav2: RESOLVED for rolling-costmap test. `NavigateThroughPoses` physical
  route가 `SUCCEEDED`였다.
- actual IW Hub lift discovery: RESOLVED. `/World/_23/iw_hub_01/lift_joint`,
  Prismatic Z, limits `0.0..0.04 m`, positive=UP을 확인했다.
- actual Vision docking: RESOLVED. first-person camera, YOLO/PnP와 실제
  `/dock_dolly` result `DOCKING_COMPLETE`를 확인했다.
- actual cuOpt: RESOLVED. production adapter와 deterministic two-task input을
  GPU runtime에서 실행했다.
- Jazzy interfaces: RESOLVED. `nav2_msgs`, `nav2_bringup`, `geographic_msgs`가
  정식 설치되어 `/tmp` overlay를 사용하지 않았다.

추가 runtime 결과는 다음과 같다.

- `RUNTIME_BLOCKER_DOCKING_LIFT_GEOMETRY`: **RESOLVED without geometry workaround**. Predocking `3 m`는 Camera-to-Dolly Vision start condition이고, final docking은 Lift Center-to-Dolly Lifting Center alignment다. Camera-to-Lift planar extrinsic은 longitudinal `-0.605948765 m`, lateral `+0.000000878 m`, yaw `0 deg`다. Vision final-entry가 이 extrinsic과 odometry를 사용하도록 변경했고 `final_entry_distance_m=4.60`은 safety cap으로 유지했다.
- runner의 Lift visual/collision offset production default는 `(0, 0)`이다. Main USD와 원본 Lift geometry는 수정하지 않았다. Workaround OFF 상태에서 `DOCKING_COMPLETE`, XY overlap, Lift Up `+0.014925 m`, stable Lift Down을 PASS했다.
- speed chain: actual wheel geometry `radius=0.08 m`, distance `0.57926 m`를 session-layer differential controller에 반영했다. RPP `desired_linear_vel=0.70`, Vision far/final `0.30/0.20 m/s`, integration reverse `0.35 m/s`를 사용한다. 동일 Nav2 route는 `60.30 -> 28.19 s`, loaded 3 m reverse는 `3.024 m / 18.811 s / zero Twist`로 PASS했다.
- Vision 전체 docking elapsed는 `56.65 -> 58.04 s`로 개선되지 않았으나 final-entry는 `31.19 -> 30.25 s`였다. 남은 지배 구간은 PnP alignment이며 이번 범위에서 algorithm/gain을 변경하지 않았다.
- `RUNTIME_BLOCKER_PRODUCTION_MAP_LOCALIZATION`: **PARTIAL/OPEN**. Main Scene
  `/amr1/scan`으로 0.10 m static map을 생성해 전체 NodeMap bounds를 포함시켰고,
  `map_server + AMCL`의 단독 `map -> amr1/odom` ownership과 lifecycle은 PASS했다.
  그러나 static map에 robot-footprint false occupied cell이 남아 production short
  NavigateThroughPoses planner가 실패한다. recoverable costmap clear로는 static
  layer가 복원됐으며 unsafe한 global static-layer disable/persistent obstacle delete는
  적용하지 않았다.
- `RUNTIME_BLOCKER_FMS_SCENE_ENDPOINTS`: **RESOLVED**. 기존 ExtNodeMapBuild가
  Stage NodeMap/route visualization owner를 유지하고 system Jazzy adapter가 public
  custom endpoints를 제공한다. NodeMap 14/16, Route B visualization, actual cuOpt
  RequestTask가 PASS했다.
- `RUNTIME_CHECK_DOLLY_UNDER_AMR`: **OPEN/SKIP**. 이번 두 가지 집중 범위 밖이다.
