# DockDolly Action

`interfaces/action/DockDolly.action`은 AMR Node가 Vision Docking Node에 Dolly
미세 정렬과 최종 진입을 요청하는 ROS 2 Action 계약이다.

## Schema

| 구분 | 필드 | 의미 |
|---|---|---|
| Goal | `string amr_id` | Docking을 요청한 AMR 식별자 |
| Goal | `string task_id` | 현재 FMS Task 식별자 |
| Result | `bool success` | Docking 성공 여부 |
| Result | `string message` | `DOCKING_COMPLETE` 또는 실패·취소 원인 |
| Feedback | `string state` | `WAITING_FOR_VISION`, `APPROACH`, `ALIGN_LATERAL`, `ALIGN_YAW`, `FINAL_ENTRY` |
| Feedback | `float32 distance_m` | 최신 Dolly 전방 거리 |
| Feedback | `float32 lateral_m` | 최신 횡방향 오차 |
| Feedback | `float32 yaw_deg` | 최신 yaw 오차 |

## 관계

```text
AMRNode Action Client
  -> /dock_dolly Goal
DollyDockingNode Action Server
  -> Vision/control Feedback
  -> Result
AMRNode
```

## Sequence

성공 시 Server는 Vision 정렬과 `FINAL_ENTRY`를 완료하고 STOP을 발행한 다음
`success=true`, `message=DOCKING_COMPLETE`로 Action을 `SUCCEEDED` 처리한다.
AMR은 그 이후에만 `DOCKING_COMPLETE`, `ARRIVED_PICKUP`, `LOADING`으로 진행한다.

실패 시 Server는 먼저 STOP하고 `success=false` 및 원인 message로 Action을
`ABORTED` 처리한다. 취소는 STOP 후 `CANCELED` 처리한다. AMR은 Server 미준비,
Goal 거절, timeout, exception, 실패 Result를 모두 기존 Task failure 흐름에
연결한다.
