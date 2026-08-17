# VESC deadman 해제 후 정지 명령 부재 조사

발견: 2026-07-29 (driver 이식 검증 중 우연히 발견) · 상태: **3호차 소프트웨어 수정 완료, 실차 재검증 필요**

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

## 적용한 수정 (2026-08-15)

3호차 `vesc_car3.yaml`에 `ackermann_to_vesc_node.command_timeout: 0.15`를
추가했다. `ackermann_to_vesc_node`는 이제 `/ackermann_cmd`가 150 ms 넘게
들어오지 않으면 보정된 정지 속도(0 m/s에 해당하는 ERPM)를 50 ms 주기로 VESC에
발행한다. 새 명령이 도착하면 즉시 워치독을 해제하고 그 명령을 처리한다.

이 위치는 `joy_teleop`과 `ackermann_mux` 뒤이므로 deadman 해제뿐 아니라
자율주행 노드 정지·프로세스 장애에도 적용된다. 4호차는 설정값을 지정하지 않아
기존 동작을 유지한다.

## 남은 실차 검증

1. 3호차에서 동일한 hold-test를 실행한다. 명령 중단 뒤 다음 워치독 주기 안에
   `/commands/motor/speed`가 약 `1.0` ERPM(3호차의 0 m/s 보정값)으로 바뀌는지,
   `/sensors/core`가 즉시 감속하는지 기록한다.
2. deadman 버튼을 주행 중 해제해 같은 결과가 나오는지 확인한다.
3. VESC Tool의 펌웨어 timeout도 별도의 최후 안전장치로 확인한다. 이 ROS 워치독은
   VESC와의 통신 자체가 끊기는 경우를 대체하지 않는다.

## 참고

- 클리핑 경고(`below/above limit, clipping`)는 이 이슈와 무관. 좌우 게인이 서보 물리
  한계를 일부러 넘도록 잡혀 있어 양쪽 끝에서 클립되는 것은 **정상**(설계상 의도됨).
  [`integration/workspace_unification.md`](../../integration/workspace_unification.md)의 Phase A' 검증 결과 참조.
