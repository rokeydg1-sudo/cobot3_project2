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
