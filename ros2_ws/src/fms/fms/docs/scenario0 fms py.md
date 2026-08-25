3.3 scenario0_fms.py

이전 역할

이전 FMS는 Assembly Task Queue를 보관하고,
AMR의 Route 요청이 들어오면 Queue 전체를 cuOpt에 전달해
최적화 순서를 계산하는 구조였다.

초기 버전에서는 실제 AMR 상태 대신 고정된 초기 상태를 사용했고,
최적화 결과만 계산/보관하는 성격이 강했다.

현재 역할

현재 FMS는 단순 최적화 호출기가 아니라
Task Queue + Task Assignment + AMR State 관리의 중심 Node 역할을 한다.

주요 변경

1. AMR Pull Service 추가

/fms/request_task

AMR이 직접 다음 Task를 요청한다.

2. 실제 AMR 상태를 cuOpt 입력으로 사용

AMR Node가 전달한:

x
y
state
load_state
current_task_id

를 이용해 AMRState를 생성한다.

3. Waiting Queue / Active Task 분리

기존에는 Task가 AMR에 할당되면 Queue에서 빠지고 끝나는 구조였다.

현재는 다음처럼 관리한다.

waiting queue
    ↓ AMR에 할당
active_tasks
    ↓ 작업 완료
제거

의미:

task_queue
= 아직 AMR에 할당되지 않은 작업

active_tasks
= 이미 AMR에 할당되어 수행 중인 작업

즉 FMS가 단순히 “남아 있는 작업”만 보는 것이 아니라,
현재 어떤 작업이 이미 실행 중인지까지 추적할 수 있게 변경했다.

4. 중복 task_id 방지

Assembly Node는 부품이 아직 도착하지 않았거나
FMS 응답을 받지 못한 경우 같은 Task를 다시 보낼 수 있다.

예:

task_id=1
task_id=1
task_id=1

이 경우 FMS에서는 다음 두 위치를 모두 확인한다.

1. waiting queue에 같은 task_id가 있는가?
2. active_tasks에 같은 task_id가 있는가?

둘 중 하나라도 이미 존재하면 새로 등록하지 않는다.

따라서:

Assembly 재전송 허용
        +
FMS 중복 방지

두 정책을 함께 적용했다.

5. 완료 Task 정리

AMR Node가 /amr/status로 작업 완료 이벤트를 보낸다.

예:

status=DELIVERY_COMPLETE
task_id=1

또는:

status=MISSION_COMPLETE
task_id=1

FMS는 해당 task_id를 active_tasks에서 제거한다.

active_tasks
    ↓
task_id=1 제거
    ↓
작업 완료 상태 정리

즉 “AMR에 할당된 순간”부터 “완료될 때”까지
Task Lifecycle을 FMS가 추적하는 구조가 추가되었다.

6. logical location → physical coordinate 변환

FMS가 cuOpt 결과를 실제 위치 좌표로 변환하여 AMR Node에 전달한다.

예:

supermarket → (-7.0, 0.0)
cell_a      → ( 7.0, 3.5)
cell_b      → ( 7.0, 0.0)
cell_c      → ( 7.0,-3.5)

AMR Node는 이 좌표를 그대로 Nav2 Goal에 넣는다.
