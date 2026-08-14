# VESC deadman 해제 후 정지 명령 부재 조사

발견: 2026-07-29 (driver 이식 검증 중 우연히 발견) · 상태: **미해결, 조사만 완료**

이 문서는 코드 통합 작업과 무관한 별도 이슈라 분리했다. 다른 세션에서 다룬다.

---

## 증상

deadman(버튼4)을 떼도 가속/후진이 한 박자 늦게, 혹은 계속 들어가는 것처럼 느껴짐.

## 재현 및 측정

`joy`/`mux`를 거치지 않고 `/ackermann_cmd`에 1.0 m/s를 2초간 직접 명령한 뒤
**완전히 발행을 중단**하고 `/sensors/core`(ERPM)를 추적:

```
t=0.00s (명령 중단)   ERPM 2837
t=0.94s               ERPM 3633   ← 명령이 끊겼는데 계속 가속
t=1.02s               ERPM -927   ← 급격히 역전
t=1.18s               ERPM -2167  ← 강한 역방향(제동)
t=1.90s               ERPM 10     ← 그제서야 정지
```

**명령을 끊은 뒤에도 약 2초간 계속 움직인다.** mux/조이스틱을 거치지 않고 재현되므로
QoS나 조이스틱 쪽 문제가 아니라 더 아래 레벨의 문제.

## 원인 (특정 완료)

코드 전체를 확인했는데, **이 체인의 어디에도 "정지" 명령을 명시적으로 보내는 곳이 없다**:

- `joy_teleop`(`/opt/ros/humble/.../joy_teleop.py`): deadman 떼면 `if not self.active: return`
  — 그냥 아무 것도 안 보낸다. 정지 메시지 발행 없음.
- `ackermann_mux`: 입력이 없으면 조용할 뿐, 자체적으로 아무것도 발행하지 않는다.
- `vesc_driver`(`speedCallback`): 새 메시지가 올 때만 시리얼에 씀. 재전송·자동정지 타이머 없음
  (`timerCallback`은 20ms마다 돌지만 상태 폴링/FW버전 확인용이지 커맨드 재전송이 아님).

→ **VESC 자체가 마지막으로 받은 속도 명령을 계속 실행**하다가, VESC 펌웨어 자체의
내부 커맨드 타임아웃(추정 ~1초)이 걸려서야 안전 제동이 걸리는 것으로 보인다.
이건 ROS 소프트웨어가 아니라 **하드웨어/펌웨어 레벨 동작**일 가능성이 높다.

## 오늘 이식 작업과의 관계

**무관하다.** `speedCallback`/`timerCallback`(정지 로직이 있어야 할 자리)은 이번 이식에서
손대지 않았다. QoS depth 10→1 변경은 "밀린 메시지가 재생되는 것"만 막지,
"애초에 정지 명령이 없는 것"은 못 고친다.

**미검증**: 이 문제가 이식 전(`153998d`)에도 있었는지. 확인하려면:
```
git -C src/vesc stash   # 또는 checkout 153998d
colcon build --packages-select vesc_ackermann vesc_driver
# 동일한 hold-test 재실행
```
현재 코드 리딩 근거로는 **이식 전에도 있었을 가능성이 높다** (해당 함수들이 이식 전후 동일).

## 다음 세션에서 할 일

1. 위 "미검증" 확인 — 이식 전 빌드로 동일 테스트, pre-existing 여부 확정
2. 원인 후보 좁히기:
   - VESC Tool로 펌웨어 커맨드 타임아웃 설정값 확인 (App Settings → General → Timeout)
   - `duty_cycle` 모드 vs `speed` 모드 차이 확인 (현재 `setSpeed()` 사용 중)
3. 수정 방향 결정 (택1, 또는 병행):
   - **워치독 노드 추가** (권장): `/teleop`(또는 `/ackermann_cmd`) 수신 간격이 예: 150ms를
     넘으면 강제로 속도 0을 발행. `joy_teleop`/`mux`/`vesc_driver` 어느 것도 안 건드림.
     이번에 이식한 QoS 변경과도 독립적으로 적용 가능.
   - VESC 펌웨어 타임아웃 값 자체를 줄임 (더 근본적이지만 하드웨어 설정)
4. 수정 후 반드시 **동일한 hold-test로 재검증** (위 재현 절차 그대로)

## 참고

- 클리핑 경고(`below/above limit, clipping`)는 이 이슈와 무관. 좌우 게인이 서보 물리
  한계를 일부러 넘도록 잡혀 있어 양쪽 끝에서 클립되는 것은 **정상**(설계상 의도됨).
  [`integration/workspace_unification.md`](../../integration/workspace_unification.md)의 Phase A' 검증 결과 참조.
