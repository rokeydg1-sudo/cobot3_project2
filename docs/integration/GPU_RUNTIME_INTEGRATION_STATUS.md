# GPU Runtime Integration Status

검증일: 2026-08-27  
Branch: `feature/FMS_Vision_Integration`  
Checkpoint: `1da723a` / `checkpoint-gpu-free-mission`

## Overall

**PARTIAL** — FMS scene endpoints와 AMCL ownership은 PASS했다. Main Scene
coordinate coverage를 갖는 static map도 생성했지만 robot-footprint false occupied
cell 때문에 static-layer short navigation이 FAIL하여 loaded delivery 이후 gate는
의도적으로 실행하지 않았다.

## Focused gate results

| Item | Result | Evidence |
|---|---|---|
| Camera-to-Lift extrinsic | PASS | longitudinal `-0.605949 m`, lateral `+0.000001 m`, yaw `0 deg` |
| Lift geometry workaround | PASS | production default `(0, 0)`, Main USD unchanged |
| Lift-center docking | PASS | `/dock_dolly` `DOCKING_COMPLETE`, XY overlap true |
| Physical Lift Up | PASS | Dolly AABB center z `+0.014925 m` |
| Physical Lift Down | PASS | landing z restored, XY shift `0.0020 m` |
| Nav2 short route | PASS | 6.824 m, `60.30 -> 28.19 s`, SUCCEEDED |
| Vision docking speed | WARN | `56.65 -> 58.04 s`; final-entry `31.19 -> 30.25 s` |
| Loaded 3 m reverse | PASS | `3.024 m`, `18.811 s`, zero Twist |
| Production map coverage | PASS | 0.10 m, origin `[-49.325054,-24.624803]`, 594×583 |
| AMCL ownership | PASS | sole `map -> amr1/odom` owner `/amcl` |
| Production short Nav2 | FAIL | static map start cell inscribed by false footprint occupancy |
| `/get_node_map` | PASS | revision 1, 14 nodes/16 edges |
| `/visualize_route` | PASS | Stage Route B highlight round-trip SUCCEEDED |
| FMS RequestTask | PASS | actual cuOpt, recovery Node 10, valid two routes |
| GPU-free regression | PASS | `PASS=13 WARN=1 SKIP=3 FAIL=0` |

## Docking reference resolution

Predocking `3 m`는 Camera-to-Dolly Vision start condition이다. 최종 완료 기준은 rear Lift Center가 Dolly의 enabled `Base/Collision` lifting region 아래에 들어오는 mechanical target이다.

Stage transform은 `Camera -> Lift = (-0.605948765, +0.000000878) m`, yaw `0 deg`다. 기존 required correction `forward=+0.6641 m`와 방향 및 크기가 일치하므로 root cause는 Camera target과 rear Lift target 사이 extrinsic 누락(B+C)이다. 측정 lateral `-0.0938 m`는 camera/lift extrinsic이 아니라 docking residual이다.

Vision은 YOLO/PnP/P-controller를 유지하고 마지막 유효 Camera-to-Dolly PnP distance를 Lift-center odometry target으로 변환한다. `final_entry_distance_m=4.60`은 safety cap이며 `4.60 + 0.6641` 같은 trial-and-error correction을 사용하지 않는다.

runner debug offset option은 보존했지만 default는 0이다. Main USD, Lift visual, collision, moving body 원본 transform은 수정하지 않는다.

## Physical Dock/Lift evidence

```text
Docked lift center=(-24.683965, 22.683991, 0.202403)
Docked Dolly center=(-24.301253, 22.612233, 0.328293)
XY overlap=true, vertical gap=0.020376 m
Lift Up Dolly center z=0.343218 m, delta=+0.014925 m
Lift Up Dolly XY shift=0.0120 m
Lift Down Dolly center z=0.328292 m
Landing XY shift from pre-lift=0.0020 m
```

## Speed chain

- Nav2: active plugin은 RPP이고 기존 `desired_linear_vel=0.25`; runtime 값을 `0.70 m/s`로 변경했다. 이 plugin에 사용되지 않는 `acc_lim_x/decel_lim_x`는 추가하지 않았다.
- Isaac: 기존 graph upper limits는 충분했지만 controller kinematics `wheelRadius=0.1`, `wheelDistance=0.7`이 실제 world geometry `0.08`, `0.57926`과 달랐다. runner session layer에서 실제 geometry로 정규화했다.
- Vision: far max `0.12 -> 0.30 m/s`, final `0.18 -> 0.20 m/s`, angular max `0.25 -> 0.35 rad/s`; P gains는 변경하지 않았다. 전체 elapsed는 alignment 변동 때문에 개선되지 않았고 final-entry만 약 0.94 s 단축됐다.
- Reverse: production integration launch override `0.20 -> 0.35 m/s`, timeout `30 s` 유지. 알고리즘은 변경하지 않았다.

Before direct baseline은 request `0.20 m/s`에서 wall average `0.0946 m/s`였다. After loaded reverse는 request `0.35 m/s`에서 wall average `0.1607 m/s`이며 30 s 안에 완료했다. Reverse 전후 lift-to-Dolly relative vector 변화는 `0.0196 m`, yaw 변화는 `0.0165 rad`로 overlap을 유지했다.

## Remaining blockers

- **BLOCKER** `RUNTIME_BLOCKER_PRODUCTION_MAP_LOCALIZATION`: map coverage와 AMCL
  ownership은 해결했지만 static map의 false robot-footprint occupied cell 때문에
  short NavigateThroughPoses가 아직 실패한다. Static layer 전체 비활성화와
  persistent obstacle 삭제는 안전상 적용하지 않았다.
- **RESOLVED** `RUNTIME_BLOCKER_FMS_SCENE_ENDPOINTS`: 기존 ExtNodeMapBuild의
  Stage owner와 system Jazzy adapter로 `/get_node_map`, `/node_map_changed`,
  `/visualize_route`를 실제 연결했다.
- **SKIP** Loaded Route B/Lift Down: production short Nav2 선행 gate FAIL.
- **NON-BLOCKER** Vision elapsed: speed cap이 아니라 perception/alignment phase가 지배하며 이번 금지 범위인 P-controller/vision redesign은 하지 않았다.
- **RUNTIME CHECK** `RUNTIME_CHECK_DOLLY_UNDER_AMR`: Lift Down 뒤 next-task 출발은 범위 밖이다.
