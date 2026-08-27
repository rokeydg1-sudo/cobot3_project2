# 인수인계: 비전 기반 도킹 구현

새 세션에 이 파일을 통째로 전달하세요. 이미 검증된 사실과 이미 밟은 지뢰가 정리되어 있습니다.

---

## 0. 지금 해야 할 일

**목표**: AMR **2대**가 **비전으로 Dolly에 도킹**하고, 그 인식 결과를 **rqt로 시각 검증**.

- 제한 시간: 약 2시간
- 발표용이므로 **안정성 > 정확도**
- 비전 방식은 자유: 학습된 YOLO Pose 모델이 있고, 안 되면 Canny/HSV 같은 고전 CV도 허용
- 현재는 AMR 3대 / 작업 6개 → **2대로 축소**할 것

**성공 기준**
1. `rqt_image_view`에 AMR 카메라 영상이 뜨고 Dolly에 박스/윤곽이 그려진다
2. 그 인식 결과가 도킹 제어에 실제로 반영된다
3. 인식 실패 시 좌표 기반으로 자동 전환되어 데모가 깨지지 않는다

---

## 1. 환경 (중요)

### 머신
**단일 PC** (`razer`)에서 전부 실행. PC1/PC2 분리는 시도했으나 **미검증**이므로 손대지 말 것.

```
razer   WiFi 172.18.0.25 / 유선 10.10.0.3
```
`10.10.0.2`, `10.10.0.5`가 유선에 붙어 있으나 SSH 차단(port 22 refused)이라 상태 확인 불가.

### `.bashrc`가 자동 설정하는 것 — 직접 export 하지 말 것
```bash
ROS_DOMAIN_ID=130
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_whitelist.xml   # 유선 10.10.0.x 만 허용
```

> **주의**: 스크립트/비대화형 셸은 `.bashrc`를 읽지 않는다. 그래서 스크립트로 띄운 브릿지(domain 30, 화이트리스트 없음)와 사용자 터미널(domain 130)이 서로 안 보이는 사고가 이미 한 번 있었다. **브릿지와 컨트롤러의 도메인·프로파일을 반드시 일치시킬 것.**

### Isaac Sim 실행 전 필수
`.bashrc`에 함수만 정의되어 있고 실행은 안 된다. 안 하면 ROS 2 Bridge 확장이 로드되지 않는다.
```bash
isaac_ros          # LD_LIBRARY_PATH 에 isaacsim.ros2.bridge/jazzy/lib 추가
```

### Python 환경
| 용도 | 경로 | 비고 |
|---|---|---|
| Isaac Sim | `~/isaacsim/python.sh` | Python 3.11 (건드리지 말 것) |
| ROS 2 | 시스템 `python3` | 3.12 |
| 비전 (YOLO/torch/cv2) | `<repo>/.venv_vision/bin/python` | **test 폴더에만 있음**, 842MB |
| cuOpt | `~/.venvs/cuopt/bin/python` | 이번 작업엔 불필요 |

---

## 2. 폴더 구조

```
~/cobot3_project2         메인. 좌표 기반 데모, git main(5ff4957)과 동일. 완전 동작.
~/cobot3_project2_test    비전 실험본. 아래 항목이 이미 들어가 있음.
```

### test 폴더에 이미 구현된 것 (재사용 가능)
- `dolly_docking_node.py` → `publish_debug_image()` 추가됨. 박스·키포인트·거리/각도 오버레이를 그려서
  `/vision/dolly_docking/debug_image` 로 발행. 구독자 없으면 스킵.
- `amr_mission_controller.py` → `vision_dock_mission` 파라미터. 지정한 미션의 `FINAL_DOCK`에서만
  `/dock/cmd_vel`을 우선 적용하고, 1초 끊기면 좌표 기반으로 자동 폴백.
- `.venv_vision/` 복원되어 있음.

**권장**: 메인에서 새로 짜지 말고 test 폴더의 위 두 파일을 가져다 쓸 것.

### 백업 (문제 시 복구)
```
..._124505_docs_portable        ← 메인의 현재 기준점
..._122620_stage3_cuopt_verified
..._120245_stage2_3amr_6task
..._112423_stage1_planner
..._105218_4missions_traffic
..._094853_ros_ok
..._091659                      ← 최초 (.venv_vision 원본 보유)
```

---

## 3. 현재 동작 상태

### 잘 되는 것
- 브릿지 기동 **20초** → `BRIDGE RUNNING`
- AMR 3대 / 작업 6개 좌표 기반 도킹 **6/6 성공**
- 도킹 오차 위치 `0.09~0.12 m`, 각도 `0.4~4.6°`
- Dolly 6대 전부 `z≈0.021` 바닥 안착, 트래픽 대기 0, 오류 0
- 계획 개선: round-robin 대비 makespan `-38%`, 거리 `-32%`

### 안 되는 것 — 이번에 해결할 것
**비전이 Dolly를 인식하지 못한다.** 직전 시도에서 `Vision/PnP invalid` 97회, 유효 인식 0회.

---

## 4. 비전 관련: 이미 확인한 사실 (재조사 불필요)

### 사실 A — 카메라 방향은 아직 결론 나지 않았다
스폰 위치(`-30.36, 17.25`, yaw 0)에서 캡처한 영상이 **벽과 천장**이었다. 그래서 "카메라가 옆을 본다"고
판단하고 `SetLookAt`으로 전방 하향 재구성까지 했으나 — **그 시점 AMR은 Dolly를 등지고 있었다.**
Dolly(`-27.84, 25.52`)는 좌전방 45°, 8m 거리였으므로 벽이 보이는 게 정상일 수 있다.

> **가장 먼저 할 일**: `GO_TO_PRE_DOCK` / `FINAL_DOCK` 구간, 즉 **AMR이 Dolly를 향한 상태**에서
> 카메라 프레임을 캡처해 육안 확인할 것. 이걸 안 하고 카메라를 건드리면 시간을 버린다.

캡처 스크립트는 `/tmp/grab.py`에 있다 (없으면 §7 참조).

### 사실 B — 세션 초기에는 비전이 실제로 동작했다
`bbox confidence 0.526`, `keypoints 4`, `solvePnPRansac 성공`, `DOCKING COMPLETE` 기록이 있다.
당시 조건:
- 카메라 위치 = **에셋 원본 자세 그대로 복사** (지금 메인 폴더 상태와 동일)
- `/tmp/docking_camera_candidate` 파일에 `3` 기록 → 캘리브레이션 후보 3 적용
  (`dx=-0.30, dy=0, dz=+0.15, pitch=+12°`)

> 즉 **메인 폴더의 카메라 코드 + 캘리브레이션 후보 3** 조합이 한 번은 성공했다.
> test 폴더의 `SetLookAt` 버전은 검증되지 않았다. 메인 쪽부터 시도할 것.

### 사실 C — 카메라가 검게 나올 때가 있다
`mean=0.0, std=0.0`인 완전 흑색 프레임. 시뮬을 오래 돌리거나 GUI 창이 가려지면 렌더가 멈춘다.
**브릿지를 재시작하면 복구된다.** 카메라 로직 문제가 아니다.

### 사실 D — 비전 노드는 1회용이다
첫 도킹 후 `final_entry_complete=True`가 되면 `image_callback`이 즉시 리턴한다.
이후 프레임을 아예 처리하지 않아 디버그 이미지도 멈춘다.
**미션마다 리셋하는 로직이 필요하다.**

### 사실 E — 개루프 폴백을 "비전 성공"으로 오인하기 쉽다
비전이 인식에 실패하면 `invalid_frames_before_handoff` 초과 시 `FINAL_ENTRY`(개루프 직진)로 전환되고,
그 명령도 `/dock/cmd_vel`로 나간다. 컨트롤러는 이를 받아 `VISION DOCKING engaged`를 찍는다.

> **토픽 연결만 보고 성공 판정하지 말 것.** 반드시 `bbox_conf` 값이 로그에 찍히는지로 확인할 것.

---

## 5. 이미 밟은 지뢰 (다시 밟지 말 것)

| 함정 | 내용 |
|---|---|
| `ROS2SubscribeJointState` | `name`과 `position` 배열 **길이가 다르면 메시지를 조용히 버린다**. `/dolly_cmd`는 `name=["dolly_cmd","dolly_seq"]`, `position=[코드, 시퀀스]` 2개씩. |
| 같은 프레임 2연속 명령 | 구독자가 마지막 값만 래치한다. ATTACH 직후 LIFT를 보내면 ATTACH가 유실된다. **0.4초 이상 벌릴 것.** |
| `ros2 topic list` 가 빈다 | Isaac Sim 브릿지는 ROS 그래프에 노드로 등록되지 않는다. **고장 아님.** `ros2 topic hz /odom`으로 확인(30Hz면 정상). rclpy 노드를 하나 띄우면 목록에 보인다. |
| Dolly는 PhysX articulation | 링크 9개. `rigidBodyEnabled=False`가 무시된다. 매 프레임 root를 강제 이동하면 솔버와 충돌해 **AMR이 바닥을 뚫고 날아간다**(`z=-374074m` 관측). 그래서 시뮬 시작 **전에** physics 스키마를 제거하고 좌표 추종으로 운반한다. |
| 바닥 collider 누락 | 원본 USD는 `x ≤ -17.25`에만 바닥이 있다. 런타임에 무한 평면(마찰 0.9) 추가로 해결됨. |
| USD 행렬 규약 | row-vector. `child_world * parent^-1`. 단 **순서를 바꿔도 회전은 안 변하고 위치만 바뀐다** — 카메라가 옆을 보는 원인이 아니었다. |
| 잔여 프로세스 | 컨트롤러/브릿지가 쌓이면 `/odom`이 중복 발행되어 로봇이 미쳐 날뛴다. 매번 정리할 것. |

---

## 6. 권장 작업 순서

### 1단계 — 카메라 시야 확인 (최우선, 30분)
1. 메인 폴더에서 AMR을 **2대로 축소** (`AMRS` 리스트에서 amr3 제거, `TASKS`도 2~4개로)
2. 브릿지 기동 → amr1 컨트롤러만 실행
3. `GO_TO_PRE_DOCK` / `FINAL_DOCK` 구간에서 카메라 프레임 캡처 → **육안 확인**
4. Dolly가 프레임에 잡히면 카메라는 정상. 안 잡히면 그때 자세 조정

### 2단계 — 인식 검증 (30분)
- `.venv_vision`으로 `dolly_docking_node.py` 실행
- 로그에 `bbox_conf` 실제 값이 찍히는지 확인 (`Vision/PnP invalid`만 나오면 실패)
- YOLO가 안 되면 **즉시 Canny/윤곽 검출로 전환 판단** (시간 관리)

### 3단계 — 시각화 (20분)
- test 폴더의 `publish_debug_image()` 이식
- `ros2 run rqt_image_view rqt_image_view /vision/dolly_docking/debug_image`

### 4단계 — 제어 연동 (30분)
- test 폴더의 `vision_dock_mission` 파라미터 이식
- **amr1의 첫 미션 1회만** 비전 사용, 나머지는 좌표 기반 유지
- 폴백 동작까지 확인

### 5단계 — 전체 리허설 (10분)

> 2단계에서 YOLO가 안 되면 미련 없이 고전 CV로 전환할 것. Dolly는 색·형상이 뚜렷해
> HSV 마스킹 + 윤곽 중심으로도 "인식했고 그 결과로 정렬한다"는 시연은 충분히 성립한다.

---

## 7. 실행 명령어

### 정리 (매번 먼저)
```bash
pkill -9 -f standalone_factory; pkill -9 -f amr_mission; pkill -9 -f dolly_docking; sleep 5
```

### 브릿지
```bash
cd ~/cobot3_project2
isaac_ros                 # 필수
export HEADLESS=0         # 1이면 창 없이 고속
~/isaacsim/python.sh simulation/isaac_sim/standalone_factory_bridge.py
```
`BRIDGE RUNNING` 대기 (20초)

### 컨트롤러
```bash
cd ~/cobot3_project2/ros2_ws
source install/setup.bash
ros2 run amr_control amr_mission_controller --ros-args -p amr:=amr1
```
비전 적용 시: `-p vision_dock_mission:=0` 추가

### 비전 노드
```bash
cd ~/cobot3_project2
./.venv_vision/bin/python simulation/isaac_sim/vision_docking/runtime/dolly_docking_node.py \
  --ros-args -p publish_cmd_vel:=true
```

### rqt
```bash
ros2 run rqt_image_view rqt_image_view /vision/dolly_docking/debug_image
```

### 카메라 프레임 캡처 (`/tmp/grab.py`)
```python
import sys, rclpy, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
class Grab(Node):
    def __init__(self, topic, out):
        super().__init__("grab")
        self.b=CvBridge(); self.out=out; self.done=False
        self.create_subscription(Image, topic, self.cb, 1)
    def cb(self, msg):
        if self.done: return
        img=self.b.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite(self.out, img)
        print(f"saved {self.out} shape={img.shape} mean={img.mean():.1f} std={img.std():.1f}")
        self.done=True
rclpy.init(); n=Grab(sys.argv[1], sys.argv[2])
for _ in range(200):
    rclpy.spin_once(n, timeout_sec=0.1)
    if n.done: break
rclpy.shutdown()
```
```bash
./.venv_vision/bin/python /tmp/grab.py /vision/front_camera/image_raw /tmp/cam.png
```
`mean=0.0` 이면 렌더 정지 → 브릿지 재시작.

---

## 8. 주요 좌표

| 항목 | 값 |
|---|---|
| 카메라 토픽 | `/vision/front_camera/image_raw` (1280x720, ~15Hz) |
| amr1 스폰 | `(-30.360, 17.248)` yaw 0 |
| amr2 스폰 | `(-29.86, -15.57)` yaw 0 |
| dolly_physics_01 | `(-27.835, 25.521)` yaw 90 — Node10 픽업 |
| dolly_physics_03 | `(-27.364, -23.874)` yaw 90 — Node11 픽업 |
| Node10 / Node11 | `(-27.847, 22.502)` / `(-27.354, -20.815)` |
| YOLO 모델 | `vision_docking/outputs/weights/dolly_pose_v1_best.pt` (6.4MB) |
| 내부 파라미터 | `vision_docking/config/camera_intrinsics.npz` |

---

## 9. 반드시 지킬 것

- **원본 USD를 저장하지 말 것.** 모든 보정은 런타임에서만.
- **좌표 기반 도킹을 제거하지 말 것.** 비전은 그 위에 얹는 옵션이어야 한다. 실패해도 데모가 살아야 한다.
- **작업 전 백업**을 만들 것.
- 사용자와 **같은 머신을 공유**한다. 장시간 테스트는 사용자 작업과 충돌하니 미리 알릴 것.
- 결과를 보고할 때 **토픽 연결이 아니라 실제 인식 수치로 판정**할 것. (§4-E)
