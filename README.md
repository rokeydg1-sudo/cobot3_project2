# Multi-AMR Dolly Transport — Digital Twin

세 대의 자율주행 로봇(AMR)이 가상 공장에서 짐수레(Dolly)를 옮기는
디지털 트윈입니다. Isaac Sim으로 시뮬레이션하고 ROS 2로 제어하며,
**어떤 로봇이 어떤 짐을 옮길지는 NVIDIA cuOpt가** 정하고,
**짐수레 앞에서 멈춰 사진을 찍어** 실제로 거기 있는지 확인한 뒤 진입합니다.

---

## 1. 이 프로젝트가 무엇인가

### 풀려는 문제

공장에서 짐수레를 station 사이로 옮기는 일은 AMR 여러 대에게 맡기는 작업이고,
**실제 하드웨어에서 발견하면 비싼 방식으로 실패합니다.**

- 로봇 두 대가 같은 통로를 동시에 claim
- 기록된 위치에서 밀려난 짐수레에 도킹을 시도
- 한 대가 일을 다 하고 나머지는 놀고 있음

셋 다 **한 대만 테스트해서는 재현되지 않는 fleet 단위 문제**이고, 타이밍에
의존하기 때문에 실제 공장 바닥에서도 안정적으로 재현되지 않습니다.

디지털 트윈이 이걸 시험할 현실적인 자리입니다. 씬은 실제 공장 레이아웃,
로봇은 실제 섀시이고, 위 실패들을 **원할 때 유발해서 측정**할 수 있습니다.

### 검증 목표

| 질문 | 측정 방법 |
|---|---|
| 운반이 전부 완료되는가 | 작업 집합 대비 성공 횟수 |
| 도킹이 얼마나 정확한가 | 미션별 도킹 시점의 위치·자세 오차 |
| 일을 나눠 갖는가 | 로봇별 작업 수, 이동거리, makespan |
| 서로 피하는가 | 통로 잠금 획득·대기 횟수 |
| 인식이 되는가 | 스냅샷별 검출률, 방위 오차 |
| **인식 실패를 견디는가** | 인식이 거부됐을 때의 도킹 정확도 |

마지막 줄이 이 프로젝트의 성격을 말해줍니다. **인식이 실패해도 데모가 깨지지
않는 것**이 인식 성공률보다 우선입니다.

### 최근 실행 결과

```
METRICS amr1 | 298.5 s | 108.2 m | T1=88s/24m T2=99s/49m T4=111s/35m
METRICS amr2 | 249.9 s | 122.9 m | T3=115s/47m T6=135s/76m
METRICS amr3 | 253.4 s | 119.5 m | T7=136s/63m T5=117s/56m

운반 7/7 완료 · 도킹 오차 0.091 ~ 0.119 m (판정 기준 0.15 m)
cuOpt 배분 amr1[T1,T2,T4] amr2[T3,T6] amr3[T7,T5] · makespan 107.2 m
```

---

## 2. 실행 방법

전부 저장소 루트에서 실행합니다. 저장소 위치에 의존하지 않습니다.

### 준비 (새로 clone 했을 때 한 번)

```bash
# ROS 2 패키지 빌드.
# `--packages-select amr_control`만 쓰면 실패합니다 — amr_control이
# interfaces에 의존하는데 colcon이 알아서 함께 빌드하지 않습니다.
cd ros2_ws && colcon build --symlink-install && cd ..

# 비전용 가상환경. --system-site-packages 가 핵심입니다.
# 없이 만들면 pip이 설치한 OpenCV/numpy가 ROS의 cv_bridge와 ABI가 어긋나
# `KeyError: 16` 또는 segfault로 죽습니다. 실제로 겪었습니다.
python3 -m venv --system-site-packages .venv_vision

# cuOpt 환경 (Isaac Sim과 인터프리터를 공유할 수 없음. §5 참조)
python3 -m venv ~/.venvs/cuopt
~/.venvs/cuopt/bin/pip install --extra-index-url https://pypi.nvidia.com \
    cuopt-cu12==26.8.0
```

### PC1 — 시뮬레이션 머신

Isaac Sim, 계획, 인식이 전부 여기서 돕니다. **한 대만으로 전체 데모가
동작합니다.**

```bash
cd ~/cobot3_project2

# 0. 이전 프로세스 정리. 항상 먼저.
#    브릿지가 둘이면 /odom을 둘 다 발행해서 로봇이 서로 다른 위치를 쫓습니다.
bash scripts/cleanup.sh

# 1. cuOpt로 작업 배분을 풀어 cuopt_plan.json 생성.
#    factory_inventory.json이 필요하므로, 새 clone이면 브릿지를 한 번 먼저
#    띄우거나 이 단계를 건너뛰고 PLAN_SOLVER=auto 를 씁니다.
~/.venvs/cuopt/bin/python scripts/plan_cuopt.py --tasks T1,T2,T3,T4,T5,T6,T7

# 2. 시뮬레이션 브릿지. "BRIDGE RUNNING"이 뜰 때까지 약 40초.
HEADLESS=0 CAMERA_WIDTH=640 CAMERA_HEIGHT=360 \
  FLEET=amr1,amr2,amr3 TASK_IDS=T1,T2,T3,T4,T5,T6,T7 PLAN_SOLVER=cuopt \
  bash scripts/run_bridge.sh

# 3. 인식 노드와 시각화 노드 (각각 별도 터미널)
bash scripts/run_vision_node.sh      # 스냅샷 인식
bash scripts/run_planner_panel.sh    # 솔버 비교 막대
bash scripts/run_planner_map.sh      # 공장 지도 + 실시간 위치

# 4. 화면 (각각 별도 터미널) — 무엇이 보이는지는 §3
bash scripts/run_rqt.sh /vision/dolly_docking/debug_image
bash scripts/run_rqt.sh /planner/map_compare

# 5. 컨트롤러, 로봇당 하나. 카메라는 amr1만 씁니다.
bash scripts/run_controller.sh amr1 --vision-all
bash scripts/run_controller.sh amr2
bash scripts/run_controller.sh amr3
```

`--vision` 은 첫 미션만, `--vision-all` 은 모든 미션에서 촬영합니다.
표본 1개의 100%는 증거가 아니므로 `--vision-all` 을 권합니다.

### PC2 — 두 번째 머신 (선택)

> **검증되지 않았습니다.** 개발 중 PC2가 SSH 차단(port 22 refused) 상태라
> 실제로 시험하지 못했습니다. 아래는 설계상 동작해야 하는 절차입니다.
> **발표에서 "다중 PC 구성했다"고 말하면 안 됩니다.**

ROS 2는 같은 도메인·같은 네트워크면 토픽이 머신 경계를 그대로 넘습니다.
PC2는 **모니터링과 제어만** 맡고 Isaac Sim은 띄우지 않습니다.

```bash
# 0. PC1과 반드시 같은 도메인이어야 합니다. rosenv.sh가 130으로 맞춥니다.
#    손으로 export 하지 마세요 — 브릿지가 다른 도메인에 뜨는 바람에
#    서로를 못 본 적이 있습니다.
cd ~/cobot3_project2
source scripts/rosenv.sh

# 1. 토픽이 실제로 보이는지부터 확인. 안 보이면 아래를 손대지 마세요.
ros2 topic list
ros2 topic echo /odom --once
ros2 topic echo /clock --once

# 2. 화면만 띄우기 (가장 안전한 사용법)
bash scripts/run_rqt.sh /vision/dolly_docking/debug_image
bash scripts/run_rqt.sh /planner/map

# 3. 컨트롤러를 PC2에서 돌리려면 — PC1에서는 그 로봇을 띄우지 말 것
bash scripts/run_controller.sh amr2
```

**필요 조건**

| 항목 | 값 |
|---|---|
| `ROS_DOMAIN_ID` | 양쪽 동일 (130) |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` |
| Fast DDS 프로파일 | `~/.ros/fastdds_whitelist.xml` — 양쪽 IP를 등록 |
| 저장소 | PC2에도 clone + `colcon build` 필요 |
| `factory_inventory.json` | **PC1이 생성**. PC2로 복사해야 컨트롤러가 뜸 |

마지막 줄이 실제 함정입니다. 컨트롤러는 이 파일에서 경로와 도킹 좌표를 읽으므로,
PC1이 브릿지를 띄운 뒤 PC2로 복사해야 합니다.

### 성공 판정 — 로그로 확인할 것

**토픽이 연결됐다는 사실은 증거가 아닙니다.** 인식이 실패해도 폴백이 같은
토픽으로 도킹 명령을 계속 발행하므로, 아무것도 인식되지 않은 실행도 연결된
것처럼 보입니다. 반드시 아래 줄을 확인하세요.

```
SNAPSHOT DOLLY seen at -1.18 deg (expected -0.74 deg, 5/5 frames)   인식 성공
SNAPSHOT hold done, resuming approach                               2초 정지
DOCK OK | position_error=0.108 m                                    도킹 성공
```

`DOLLY CONFIRMED on n/5 frames` 는 **짐수레는 확인됐지만 방위가 조향에 쓸
만큼 정밀하지 않다**는 뜻입니다. 좌표 기반으로 진입을 계속합니다.
**설계된 동작이지 실패가 아닙니다.**

---

## 3. 화면 4개는 각각 무엇인가

### `/vision/dolly_docking/debug_image` — AMR 1인칭 카메라

로봇 앞에 달린 도킹 카메라 영상에 인식 결과를 겹쳐 그립니다.
검출되면 초록 박스, 거부되면 빨간 박스와 사유가 표시됩니다.
**요청과 무관하게 항상 발행**되므로 주행 내내 화면이 살아 있습니다.

### `/planner/comparison` — 솔버 비교 막대그래프

**"cuOpt가 얼마나 좋아졌나"**에 답합니다.

```
manual   ████████████████████  180.2 m   total 491.7 m
greedy   ██████████████████    156.8 m   total 405.6 m
cuopt    ███████████           107.2 m   total 299.5 m
vs manual: makespan +40.5%   distance +39.1%
```

아래에 로봇별 배분과 이동거리가 붙습니다.

### `/planner/map` — 공장 지도 + 실시간 위치

**"cuOpt가 무슨 결정을 했나, 로봇은 지금 어디 있나"**에 답합니다.

| 기호 | 의미 |
|---|---|
| ○ 원 | 픽업 지점 |
| ✕ 가위표 | 하차 지점 |
| ■ 사각형 | 로봇 출발 위치 |
| 🟠 주황 점 | Dolly 실제 위치 |
| 초록 / 분홍 / 파랑 | amr1 / amr2 / amr3 |

**로봇 위치가 실시간으로 움직입니다.** 막대그래프만으로는 "무엇을
결정했는지"가 안 보이는데, 지도로 그리면 로봇들이 공장을 나눠 맡았는지
한 통로에 몰렸는지가 즉시 읽힙니다.

Dolly를 실제 색(파랑)으로 그렸더니 amr3 경로색과 구분이 안 돼 amber로
바꿨습니다. 로봇을 세는 사람이 화물도 세어야 하는 상황이었습니다.

### `/planner/map_compare` — manual vs cuOpt 좌우 비교

같은 평면도를 같은 축척으로 나란히 놓습니다. **왼쪽(manual)은 경로가 공장을
가로지르며 엉켜 있고, 오른쪽(cuOpt)은 정리돼 있습니다.**
숫자를 읽지 않아도 차이가 보입니다.

단 **실시간 로봇 위치는 없습니다** — 계획끼리의 비교이기 때문입니다.
로봇이 움직이는 걸 보여주려면 `/planner/map`이 필요합니다.

> 모든 그림은 `factory_inventory.json`에서 읽습니다. 솔버를 다시 돌리지
> 않으므로 **그림이 로봇의 실제 실행과 어긋날 수 없습니다.**

---

## 4. 무엇을 알아냈나

이 절이 이 저장소의 핵심 자산입니다. 결론이 뒤집힌 기록을 포함합니다.

### 카메라 초점거리가 설정값의 2.6배였다

가장 큰 발견입니다. 학습된 YOLO의 검출률이 20~30%로 낮아 처음에는
**"학습 라벨의 기하가 틀렸다"**고 결론 내렸습니다. **그 결론은 틀렸습니다.**
USD에서 직접 실측하니 키포인트는 3 cm 이내로 정확했습니다.

진짜 원인은 카메라였습니다. 거리를 아는 프레임에서 상판 픽셀 폭으로 역산:

```
거리 5.55 m, 폭 342px -> fx = 1529      설정값 fx = 554
거리 4.09 m, 폭 439px -> fx = 1447
거리 3.61 m, 폭 505px -> fx = 1467      중앙값 1467
```

폭이 거리에 정확히 반비례(편차 3%)했습니다. **단일 강체를 올바르게 보고
있다는 뜻**이므로, 문제는 카메라 모델이 2.6배 틀렸다는 것이었습니다.
렌더러가 USD 광학 속성을 무시하고 28° 고정으로 렌더링하고 있었습니다.

설정을 강제하는 대신 **실측값을 기록**하는 방식으로 바꿨습니다.

| | 보정 전 | 보정 후 |
|---|---|---|
| 방위 오차 중앙값 | −5.2° | **0.00°** |
| 방위 오차 표준편차 | 19.3° | **2.63°** |
| 검출 수락률 | 0% | **56%** |

**이 하나가 그날의 잘못된 결론 두 개를 만들어냈습니다** — "학습 라벨이
틀렸다"와 "배경 파란 설비와 병합된다". 둘 다 같은 오차의 증상이었습니다.

### 검출기는 YOLO가 아니라 HSV다

발표에서 틀리기 쉬운 부분입니다. **현재 실행 경로에 학습 모델이 없습니다.**

```python
hsv  = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv, (95, 80, 50), (130, 255, 255))
```

공장이 흰색·회색뿐이라 채도 높은 파랑은 상판밖에 없습니다.
YOLO는 시도했다가 폐기했습니다 — 신뢰도 임계값을 0.05로 낮추면
**완전히 검은 프레임 21장 전부에서 "Dolly 발견"**이라고 답했습니다.
**놓치는 것보다 확신에 찬 오답이 더 위험합니다.**

`vision_docking/sdg/`, `training/`, `runtime/dolly_docking_node.py` 의 YOLO
코드는 남아 있지만 **어떤 실행 경로에도 없습니다.**

### 색으로는 안 되고 형상으로는 되는 것이 있었다

바닥의 파란 도색선이 상판과 같은 HSV 범위에 들어와 오인됐습니다.
상판은 두께 0.257 m라 3~5 m에서 66~107 px인데, 도색선은 몇 px에 통로 폭만큼
깁니다. 높이·세장비 게이트를 추가했습니다.

567프레임 회귀 검증:

| | 수정 전 | 수정 후 |
|---|---|---|
| 시야 내 Dolly 검출 | — | **91/92 (99%)** |
| 빈 프레임 오보고 | 53건 | **8건** |

남은 8건을 열어보니 **전부 실제로 Dolly가 있었습니다.** 판정 기준이 좁아
"없다"로 분류한 것이고 검출기가 맞았습니다. **실질 오검출 0건.**

### 보정을 무제한 적용하면 오히려 나빠진다

방위 오차 표준편차 2.63°는 3 m에서 0.138 m입니다. 그런데 **좌표 도킹만
했을 때의 오차가 이미 0.109~0.120 m**입니다.

```
0.138 m (비전 보정의 불확실성)  >  0.120 m (좌표 도킹의 오차)
```

그래서 ±0.05 m로 자릅니다. 클램프가 실제로 데모를 지킨 기록:

```
SNAPSHOT DOLLY seen at +1.92 deg (expected -2.87 deg, 5/5 frames)
  -> lateral +0.378 m, applying +0.050 m
DOCK OK | position_error=0.119 m
```

그대로 적용했다면 짐수레에서 38 cm 밀려나 도킹 실패였습니다.

**비전의 주 역할은 "짐수레가 실제로 거기 있다"는 진입 허가 판정이고,
보정은 제한된 보조입니다.**

### cuOpt는 빨라서 쓴 게 아니다

첫 실행에서 cuOpt가 **7작업 전부를 한 대에 몰아줬습니다**(314.6 m).
**버그가 아닙니다** — 복귀 거리를 세지 않으므로 총거리 최소화로는 그게
진짜 최적입니다. 정답이지만 fleet 시연으로는 쓸모없는 정답이었습니다.

제약 두 개(`set_min_vehicles`, `set_vehicle_max_costs`)로 fleet 문제로
바꿨습니다. **이 둘은 makespan만 보는 자체 솔버로 표현할 수 없습니다.**

```
exact    makespan=105.3 m | solve   1.6 ms   ← 자체 완전탐색
cuopt    makespan=107.2 m | solve 336.0 ms
```

7작업/3대에서 **자체 `exact` 솔버가 200배 빠르고 makespan도 1.9 m 좋습니다.**
당연합니다 — makespan이 우리 지표이고 `exact`는 그걸 직접 최소화하며 최적을
보장합니다. cuOpt는 총거리를 최소화한 뒤 우리 자로 재채점된 값입니다.

**cuOpt의 가치는 속도가 아니라 제약 표현력입니다.** 발표에서 "빨라서 썼다"고
하면 사실이 아닙니다.

### 표본 1개의 100%는 증거가 아니었다

스냅샷 표본을 1개에서 3개로 늘리자 인식률이 **100% → 67%**로 내려갔습니다.
정직한 수치는 후자입니다.

---

## 5. 무엇을 보충했나

시행착오에서 나온 수정들입니다. 전부 실측 실패가 근거입니다.

| 보충 | 왜 |
|---|---|
| 웨이포인트 마커 제거 | 카메라 시야를 가림. `MakeInvisible()`은 하위 프림이 덮어써서 안 통했고, 루트를 `SetActive(False)`로 비활성화 |
| 화각 30.5° → 60° 시도 | 폭 3.16 m로 오인했던 상판이 화면을 넘침 → 후에 fx 오차가 진짜 원인으로 밝혀짐 |
| 스탠드오프를 **짐수레 앞단** 기준으로 | dock 지점은 짐수레 아래 중앙이라, 3 m 뒤에 서면 앞단은 훨씬 가까움 |
| `SNAPSHOT_ALIGN` 상태 추가 | 접근 경로가 남긴 헤딩 그대로면 짐수레가 화면 옆으로 밀림 |
| 접근 **중** 촬영 (`maybe_snapshot_on_approach`) | 픽업 노드에 도착한 뒤 촬영하면 5 m 후진했다가 다시 진입 |
| 오버레이 색 통일 | 검출됐는데 주황이면 보는 사람에게 실패로 읽힘 |
| 범례 폭을 `cv2.getTextSize`로 실측 | 글자 수로 추정했더니 마지막 항목이 화면 밖으로 |
| 오도메트리에 **스폰 자세 적용** | 빠뜨리면 지도에서 로봇이 14 m 어긋나 그려짐 |
| 판정 기준을 스크립트 **상단 상수**로 고정 | 숫자를 보고 기준을 정하면 의미가 없음 (`0.15 m`, `6.0°`) |
| 인식률과 방위 사용률 **분리 집계** | 합치면 어느 쪽이 실패했는지 사라짐 |
| 런처를 `/tmp` → `scripts/` 이관 | 재부팅 시 사라져 새 clone에서 실행 불가였음 |

### 되돌린 변경 — 다시 시도하지 말 것

`smooth_angular()`의 회전속도 하한(0.13 rad/s)과 가속 제한(1.2 rad/s²)을
**함께** 넣으면 제자리 회전이 수렴하지 못합니다. 허용오차 안에서 하한이 계속
회전을 강제하고, 가속 제한이 반전에 여러 틱을 씁니다. amr2가 목표에서
멀어지는 것이 관측됐고 amr1은 `SNAPSHOT_ALIGN`에서 나오지 못했습니다.

현재는 사실상 비례 제어로 되돌려 둔 상태입니다. 부드럽게 만들려면 하한을
상수로 두지 말고 최종 접근각 안에서 함께 줄어들게 해야 합니다.

---

## 6. 알려진 한계

숫자의 의미를 한정하므로 앞에 둡니다.

- **짐수레 위치가 실행마다 고정됩니다.** 랜덤 배치는 구현되지 않았습니다.
- **카메라가 한 대뿐입니다.** 브릿지가 도킹 카메라를 하나만 만들어서,
  나머지 두 대는 좌표 도킹만 합니다. render product를 3개로 늘리면 GPU
  드라이버 과부하로 튕긴 이력이 있습니다.
- **비전 보정이 클램프되어 있습니다.** §4 참조.
- **단일 머신입니다.** 다중 PC ROS 2는 검증되지 않았습니다 (§2).
- **T4가 기본 작업 집합에서 빠져 있습니다.** Node_11 픽업은 스냅샷이 한 번도
  쓸 만한 측정을 못 낸 유일한 지점입니다. 원인 미규명(상판 폭 예측, 조명,
  접근 헤딩이 후보)이라 **모르는 채 시연에서 실패하는 것보다 빼는 게 낫다**고
  판단했습니다. **증상을 피한 것이지 고친 게 아닙니다.**
  되살리려면 `TASK_IDS=T1,...,T7` 후 계획 재생성.
- **브릿지가 스스로 종료한 적이 있습니다.** 434초와 555초에 각각
  `Simulation App Shutting Down`. 로그에 원인이 없고 GPU 메모리 경고만
  반복됩니다. **시연 영상을 미리 녹화해 두세요.**
- **USD 에셋 2개에 죽은 절대 경로가 있습니다.**
  `Collected_AF2_FLAT/AF2_FLAT.usd` 와 `AF2_MULTI_BACKUP.usd` 가 존재하지
  않는 경로를 참조합니다. 이미 해소되지 않는 참조이고 그 상태로도 씬이
  완전히 동작합니다. 고치려면 126 MB 바이너리 USD를 다시 써야 하는데,
  인수인계 문서의 첫 규칙이 "원본 USD를 저장하지 말 것"이라 손대지 않았습니다.

---

## 7. 요구 환경

| 구성 요소 | 버전 / 경로 | 비고 |
|---|---|---|
| Isaac Sim | `~/isaacsim` 5.x | 자체 Python 3.11 사용 |
| ROS 2 | Jazzy, `/opt/ros/jazzy` | |
| Python (ROS) | 시스템 `python3` 3.12 | |
| 비전 환경 | `.venv_vision` | **`--system-site-packages` 필수** |
| cuOpt 환경 | `~/.venvs/cuopt` | Python 3.12 + `cudf` |
| GPU | NVIDIA, CUDA 12+ | 개발: RTX 5080 Laptop |

`.venv_vision`에 `opencv-python`을 따로 설치하면 안 됩니다. ROS의
`cv_bridge`가 시스템 OpenCV(4.6) 기준으로 빌드되어 있어, pip으로 받은 5.x는
`KeyError: 16`, numpy 2.x는 segfault를 냅니다. 시스템 패키지를 그대로
쓰는 것이 정답입니다.

cuOpt는 Isaac Sim과 인터프리터를 공유할 수 없습니다 — `cudf`가 필요한데
Isaac이 자체 CUDA 스택을 들고 옵니다. 그래서 사전 계산 후 JSON으로 넘깁니다.

### 환경 변수

전부 선택 사항입니다. 기본값은 `standalone_factory_bridge.py`에 있습니다.

| 변수 | 기본값 | 의미 |
|---|---|---|
| `FLEET` | `amr1,amr2` | 스폰할 로봇 |
| `TASK_IDS` | `T1,T2,T3,T5,T6,T7` | 실행할 운반. T4 제외 |
| `PLAN_SOLVER` | `auto` | `auto`/`manual`/`greedy`/`exact`/`cuopt` |
| `HEADLESS` | `0` | `1`이면 창 없이 렌더 |
| `CAMERA_WIDTH` / `HEIGHT` | `1280` / `720` | 도킹 카메라 해상도 |
| `SHOW_WAYPOINT_GRAPH` | `0` | `1`이면 웨이포인트 마커 복원 |
| `ROS_DOMAIN_ID` | `130` | `scripts/rosenv.sh`가 설정 |

**해상도를 바꾸면 반드시** `scripts/scale_intrinsics.py`로 intrinsics를 다시
만드세요. 안 그러면 브릿지가 조리개를 다시 유도하면서 화각이 조용히 바뀝니다.

`scripts/rosenv.sh`를 손으로 export 대신 반드시 source 하세요. 브릿지가
컨트롤러와 다른 도메인에 떠서 서로를 못 본 적이 있습니다.

---

## 8. 구조

```
scripts/
  rosenv.sh              공용 ROS 2 환경 — 어디서나 source
  cleanup.sh             남은 브릿지·컨트롤러·뷰어 정리
  run_bridge.sh          Isaac Sim + ROS 2 브릿지
  run_vision_node.sh     스냅샷 인식 노드
  run_controller.sh      AMR 미션 컨트롤러 1대
  run_planner_panel.sh   솔버 비교 막대 이미지
  run_planner_map.sh     공장 지도 + 좌우 비교 이미지
  run_rqt.sh             이미지 토픽 뷰어
  plan_cuopt.py          cuOpt로 배분을 풀어 JSON 기록
  planner_panel.py       비교 막대 그리기
  planner_map.py         공장 지도 / manual vs cuOpt 그리기
  report_runs.py         컨트롤러 로그를 파싱해 채점
  measure_dolly.py       USD에서 짐수레 치수 실측
  scale_intrinsics.py    해상도·화각에 맞는 intrinsics 생성
  eval_detection.py      기록된 프레임에 대한 검출률
  project_dolly.py       알려진 기하로 짐수레 위치 투영

simulation/isaac_sim/
  standalone_factory_bridge.py   씬 구성, ROS 2 브릿지, fleet 계획
  mission_planner.py             그래프, 솔버, 계획 채점
  vision_docking/
    config/camera_intrinsics*.npz  보정된 카메라 모델 (fx=1286.3)
    runtime/blue_deck_detector.py  HSV 인식  ← 실제 사용
    runtime/dolly_snapshot_node.py 스냅샷 프로토콜 + rqt 오버레이
    runtime/dolly_docking_node.py  YOLO 버전  ← 미사용
    sdg/ , training/               합성 데이터 생성·학습  ← 미사용

ros2_ws/src/amr_control/         미션 컨트롤러(FSM), 통로 잠금
docs/                            작업 기록과 인수인계 문서
```

---

## 9. 설계 요약

다이어그램은 `docs/ARCHITECTURE.md`에 있습니다.

### 제어

각 로봇이 **독립된 상태 기계**를 돌립니다. 모션 명령을 내리는 중앙
관제가 없고, 공유 통로 claim 토픽으로만 협조하므로 **컨트롤러 하나가
죽어도 나머지는 멈추지 않습니다.**

```
WAIT_ODOM -> APPROACH -> [VISION_STANDOFF -> SNAPSHOT_ALIGN
                          -> WAIT_SNAPSHOT -> SNAPSHOT_HOLD]
          -> GO_TO_PRE_DOCK -> FINAL_DOCK -> LIFT_UP -> ATTACH_DOLLY
          -> CARRY -> LIFT_DOWN -> DETACH_DOLLY -> UNDOCK -> 다음 미션
```

대괄호 안은 비전을 켠 로봇·미션에서만 실행됩니다. **접근 주행 중**
dock까지 거리가 스탠드오프 아래로 처음 내려가는 순간 진입하므로 전진만으로
도달합니다. 픽업 노드 도착 후에 조준하면 후진했다가 다시 들어가야 했습니다.

### 인식

**제어 루프가 아니라 정지 상태의 단일 측정**입니다. 이전 버전은 프레임마다
카메라로 조향했는데, 개루프 폴백이 같은 토픽으로 발행하는 바람에
**아무것도 인식되지 않은 실행도 비전이 동작한 것처럼 보였습니다.**

스냅샷은 숫자를 내놓거나 내놓지 못하거나 둘 중 하나이고, 컨트롤러는 이미
가지고 있던 목표에 **제한된 보정**으로만 적용합니다.

검출기는 신뢰도가 다른 두 질문을 분리합니다.

- **짐수레가 있는가** — 1~8 m에서 75~100% 응답. 진입 허가 판정에 사용
- **정확히 어디인가** — 훨씬 드묾. 목표 미세조정에만, 그것도 클램프 안에서

### 계획

`mission_planner.py`가 그래프와 솔버 3종(greedy, 완전탐색, 국소탐색)을
가지고 있고 전부 makespan을 최소화합니다. cuOpt는 `scripts/plan_cuopt.py`가
별도 프로세스에서 풀어 JSON으로 넘깁니다.

**모든 후보 계획을 같은 비용 모델로 재채점**한 뒤 비교하므로, 패널이 한
솔버의 산수를 다른 솔버의 산수와 비교하는 일이 없습니다.

---

## 10. 문서

- `docs/WORKLOG_2026-08-27.md` — 개발 기록. 무엇을 측정했고 무엇을 시도해
  버렸는지. **틀린 것으로 판명된 결론 두 개와 그것을 어떻게 잡았는지 포함.**
- `docs/ARCHITECTURE.md` — 프로세스·인터페이스 맵, 컨트롤러 상태 기계,
  인식에서 바퀴까지의 시퀀스, 계획 데이터 흐름. Mermaid라 GitHub에서 렌더됨
- `docs/HANDOFF_vision_docking.md` — 환경 설정과 함정
- `docs/planner_benchmark.md` — 솔버 벤치마크
