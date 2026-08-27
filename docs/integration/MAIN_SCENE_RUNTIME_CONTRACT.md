# Main Scene Runtime Contract

검증 기준은 Main Scene 원본
`simulation/isaac_sim/worlds/Collected_AF2_FLAT/AF2_FLAT.usd`와 Isaac Sim
5.1.0 실제 runtime이다. 원본 USD는 수정하지 않았다.

## Isaac runtime

- Installation root: `/home/rokey/isaacsim`
- Launcher: `/home/rokey/isaacsim/isaac-sim.sh`
- Isaac Python: `/home/rokey/isaacsim/python.sh` (Python 3.11.13)
- Standalone runner: `simulation/isaac_sim/run_main_scene.py`
- Runtime composition layer:
  `simulation/isaac_sim/config/AF2_FLAT_integration.usda`
- ROS extension: `isaacsim.ros2.bridge`
- ROS domain: `129`
- RMW: `rmw_fastrtps_cpp`

Main USD의 기존 AMR1 Action Graph/LiDAR graph를 재사용하고, session layer에서
topic/frame을 정규화한다. camera, joint-state command, map TF와 Dolly
ground-truth TF만 standalone runtime graph가 추가한다. AMR2 graph는 별도
integration layer에서 비활성화한다.

## Stage and physics

| Contract | Actual value |
|---|---|
| Default prim | `/World` |
| Up axis | `Z` |
| metersPerUnit | `1.0` |
| Main prim count | `94,858` |
| Physics scene | runner initialization으로 생성 |
| Articulation roots | `10` |
| Rigid bodies | `89` |
| Collision prims | `91` |
| Physics joints | `79` |

22개의 visual texture/MDL dependency가
`Collected_Factory_backup/SubUSDs/...` 절대 경로에서 resolve되지 않는다.
Main geometry/physics load와 runtime test는 가능했지만 asset completeness는 WARN이다.

## Physical prims

```text
AMR_PRIM_PATH=/World/_23/iw_hub_01
BASE_PRIM_PATH=/World/_23/iw_hub_01/chassis
LEFT_WHEEL_JOINT=/World/_23/iw_hub_01/left_wheel_joint
RIGHT_WHEEL_JOINT=/World/_23/iw_hub_01/right_wheel_joint
FRONT_CAMERA_PRIM_PATH=/World/_23/iw_hub_01/camera_mount/transporter_camera_first_person
LIDAR_PRIM_PATH=/World/_23/iw_hub_01/chassis/lidar_link/Example_Rotary_2D
LIFT_JOINT_PATH=/World/_23/iw_hub_01/lift_joint
LIFT_JOINT_NAME=lift_joint
LIFT_MOVING_BODY_PATH=/World/_23/iw_hub_01/lift
LIFT_COLLISION_PATH=/World/_23/iw_hub_01/lift/Collision
LIFT_VISUAL_PATH=/World/_23/iw_hub_01/lift/Lift/Lift/Mesh_012
DOLLY_PRIM_PATH=/World/dolly_physics
DOLLY_RIGID_BODY_PATH=/World/dolly_physics/Base
DOLLY_COLLISION_PATH=/World/dolly_physics/Base/Collision
DOLLY_VISUAL_PATH=/World/dolly_physics/Base/FOF_Mesh_Shelf_Cart_B_LOD0
```

Initial AMR world position은 `(-30.360045, 17.247943, 0.338165)`이다.
Node 10은 runtime cuOpt probe 기준 `(-27.846621, 22.502320)`이고,
대상 Dolly는 `(-24.832190, 22.560954)`로 약 3.015 m 앞에 있다.

## ROS graph

| Function | Topic/action | Type | Frame/rate |
|---|---|---|---|
| simulation time | `/clock` | `rosgraph_msgs/msg/Clock` | 약 67 Hz, single source |
| base command | `/amr1/cmd_vel` | `geometry_msgs/msg/Twist` | sequential ownership |
| odometry | `/amr1/odom` | `nav_msgs/msg/Odometry` | `amr1/odom` → `amr1/base_link`, 약 67 Hz |
| scan | `/amr1/scan` | `sensor_msgs/msg/LaserScan` | `amr1/front_lidar`, 약 16.7 Hz, 1066 samples, 360° |
| RGB image | `/vision/front_camera/image_raw` | `sensor_msgs/msg/Image` | `amr1/front_camera`, 약 18 Hz |
| camera info | `/vision/front_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 1280×720 |
| lift measured state | `/amr1/joint_states` | `sensor_msgs/msg/JointState` | `lift_joint` 포함 |
| lift position command | `/amr1/joint_commands` | `sensor_msgs/msg/JointState` | named target |
| transforms | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | 아래 chain |
| navigation | `/navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` | PASS |
| docking | `/dock_dolly` | `interfaces/action/DockDolly` | PASS |
| lift | `/lift_dolly` | `interfaces/action/LiftDolly` | measured-position PASS |

TF chain:

```text
map -> amr1/odom -> amr1/base_link
amr1/base_link -> amr1/front_lidar
amr1/base_link -> amr1/front_camera
map -> ground_truth/dolly_base
```

`run_main_scene.py --no-publish-map-to-odom` production mode에서는 AMCL만
`map -> amr1/odom`을 publish한다. runner의 static publisher는 mapping/minimal test
mode에만 사용한다.

## Production map and localization

```text
MAP_YAML=ros2_ws/src/cobot3_bringup/maps/af2_flat_scenario0_map.yaml
resolution=0.10 m/cell
origin=(-49.325054, -24.624803, 0.0)
size=594 x 583
coverage_x=(-49.325054, 10.074946)
coverage_y=(-24.624803, 33.675197)
localization=AMCL
map_to_odom_owner=/amcl
```

Map coverage는 NodeMap bounds `x=[-42.453572, 7.905887]`,
`y=[-22.572640, 22.502320]`을 포함한다. 실제 `/amr1/scan` mapping 시 Isaac의
1066 ranges와 angle metadata가 1 sample 불일치하고 no-return이 `-1.0`이므로,
mapping 전용 `/amr1/scan_mapping`에서 `angle_max`와 invalid range만 정규화한다.
production `/amr1/scan`은 변경하지 않는다.

`map_server`, `amcl`, planner/controller/behavior/BT lifecycle과 AMCL 단독
`map -> amr1/odom`은 PASS했다. 단, SLAM static map의 robot-footprint 내부 false
occupied cell 때문에 static-layer short navigation은 FAIL했다. Static layer 전체
비활성화나 persistent obstacle 삭제는 적용하지 않았다.

## FMS scene endpoints

Main Stage data/visualization owner는 기존 `ExtNodeMapBuild`다. Isaac 5.1 bundled
Fast DDS와 host Jazzy custom TypeObject ABI 충돌을 피하기 위해 extension은 private
standard-message bridge를 사용하고, system Jazzy
`cobot3_bringup/scene_endpoint_adapter`가 public contract를 제공한다.

```text
/get_node_map       interfaces/srv/GetNodeMap
/node_map_changed   interfaces/msg/NodeMapChanged
/visualize_route    interfaces/action/VisualizeRoute
```

`/get_node_map`은 Stage revision 1, 14 nodes/16 edges를 반환했다.
`/visualize_route` Route B `[10,8,9,11]`은 실제 Stage highlight 응답을 받은 뒤
`SUCCEEDED`를 반환했다. actual cuOpt RequestTask도 recovery Node 10과 유효한
approach/delivery routes를 반환했다.

## Camera and vision

- Runtime resolution: `1280x720`
- CameraInfo K: `fx=fy≈2344.3225`, `cx=640`, `cy=360`
- Existing vision calibration과 일치하도록 camera focal length를 session layer에서
  `0.5`로 설정한다.
- Model: `ros2_ws/src/vision_docking/models/dolly_pose_v1_best.pt`
- YOLO/PnP/P-controller parameter는 retune하지 않았다.

## Lift

- DOF index: `4`
- Type/axis: `Prismatic`, `Z`
- Limits: `0.0 .. 0.039999999 m`
- Positive direction: physical UP
- Probe: `0.0 -> 0.010000187 m`, lift prim `+0.010000348 m`
- Production command: `ArticulationAction`/Articulation Controller position target
- `/lift_dolly` success: measured joint position가 tolerance 안에서 3회 연속 확인될 때만 반환

Joint relationship은 `body0=/World/_23/iw_hub_01/chassis`,
`body1=/World/_23/iw_hub_01/lift`이다. 움직이는 body 아래에는 enabled collision이
`lift/Collision` 하나 있고 visual mesh와 world bounds가 일치한다. Dolly 하부도
`Base/Collision` 하나만 enabled이며, 네 wheel/swivel collision은 disabled다. 따라서
실제 lift contact region은 Dolly Base의 단일 enabled collision footprint다.

### Docking reference contract

`3 m`는 Node 10 pre-docking에서 **front Camera-to-Dolly Vision start distance**를 뜻한다. 최종 mechanical docking 기준은 별개이며, **AMR Lift Center-to-Dolly Lifting Center alignment**다.

초기 Stage transform에서 측정한 planar extrinsic은 다음과 같다.

```text
Camera world XY=(-30.010045, 17.247943)
Base world XY=(-30.360045, 17.247943)
Lift collision center XY=(-30.615994, 17.247944)
Camera -> Lift: longitudinal=-0.605948765 m, lateral=+0.000000878 m, yaw=0 deg
Base -> Lift: longitudinal=-0.255948765 m, lateral=+0.000000878 m
```

기존 Docking Complete에서 측정한 required correction `forward=+0.6641 m, lateral=-0.0938 m`와 비교하면 forward magnitude 차이는 약 `0.0582 m`다. 따라서 주원인은 Dolly visual/collision이나 Lift collision 변형이 아니라 Camera target reference와 rear Lift mechanical reference 사이 extrinsic 누락(B+C)이다. 측정 lateral correction은 camera-to-lift lateral extrinsic이 아니라 docking residual이다.

Vision은 마지막 유효 PnP의 Camera-to-Dolly distance를 보존한 뒤 odometry target을 `camera_distance - camera_to_lift_longitudinal`로 계산한다. `final_entry_distance_m=4.60`은 더하는 correction이 아니라 odometry safety cap이다. YOLO, PnP, P-controller gain은 변경하지 않았다.

runner의 `COBOT3_LIFT_CONTACT_OFFSET_X/Y` debug option은 남아 있으나 production default는 `(0.0, 0.0) m`이다. 원본 Lift moving body, visual, collision transform과 Main USD는 수정하지 않는다.

workaround OFF 재검증 결과는 다음과 같다.

```text
DOCKING_COMPLETE: PASS
Lift/Dolly XY overlap: PASS
Dolly AABB center z: 0.328293 -> 0.343218 m (+0.014925 m)
Lift UP Dolly XY displacement: 0.0120 m
Lift DOWN AABB center z: 0.328292 m
Lift DOWN landing XY displacement from pre-lift: 0.0020 m
```

### Base speed contract

실제 wheel Cylinder world bounds는 diameter `0.160 m`이므로 radius는 `0.080 m`, left/right center distance는 `0.57926 m`다. Main Action Graph 원본 값 `wheelRadius=0.1`, `wheelDistance=0.7`은 runner session layer에서 실제 geometry 값으로 정규화한다. 원본 USD는 저장하지 않는다. Graph 상한 `maxLinearSpeed=15`, `maxAngularSpeed=300`, `maxWheelSpeed=600`은 이번 속도 범위의 clamp가 아니다.

Runtime speed parameters:

```text
Nav2 plugin=nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
Nav2 desired_linear_vel=0.70 m/s
Vision far max_linear_mps=0.30 m/s
Vision final_entry_speed_mps=0.20 m/s
Vision max_angular_rps=0.35 rad/s
AMR return_speed_mps=0.35 m/s (integration launch override)
AMR return_timeout_s=30.0 s
```

RPP에는 이 runtime에서 사용하는 linear acceleration/deceleration parameter가 없어 존재하지 않는 값을 추가하지 않았다. 동일 6.824 m route는 wall elapsed `60.30 -> 28.19 s`로 단축되어 SUCCEEDED했다. Vision docking은 `56.65 -> 58.04 s`로 run-to-run alignment 변동 범위였고, final-entry phase 자체는 `31.19 -> 30.25 s`로 단축됐다. 따라서 Vision의 주 병목은 speed clamp가 아니라 PnP 기반 alignment phase다.

Loaded production reverse는 `0.35 m/s`, 30 s에서 `3.024 m`, `18.811 s`, average wall speed `0.1607 m/s`, zero Twist로 PASS했다. Lift-to-Dolly relative vector 변화는 약 `0.0196 m`, relative yaw 변화는 약 `0.0165 rad`로 overlap을 유지했다.
