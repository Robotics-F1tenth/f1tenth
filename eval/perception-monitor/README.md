# Perception 감독 도구 (Foxglove)

perception 이 지금 무엇을 보고 무엇을 control 에 넘기는지 **실시간 그래프로** 본다.
control 팀이 쓰는 감독 레이아웃과 같은 방식이다.

```bash
./run_monitor.sh                       # 실차 (이미 돌고 있는 노드에 붙음)
./run_monitor.sh <bag_dir>             # bag 재생
```
Foxglove → `ws://localhost:8765` → **Layout → Import from file** →
`foxglove_perception_monitor.json`

## 왜 필요했나

perception 의 출력은 전부 **배열과 마커 배열**이라 Plot 패널이 그릴 게 없었다. 문제가
생겨도 3D 뷰를 눈으로 보거나 5 초 집계 로그를 사후에 읽는 수밖에 없었다.
`perception_monitor.py` 가 같은 정보를 **스칼라로** 바꿔 발행한다.

## 패널 7개

| 패널 | 보는 것 | 이상 신호 |
|---|---|---|
| **3D** | 맵·스캔·정적 랜드마크·트랙 박스·자차 pose | 장애물이 벽에 붙었는데 마커가 없음 |
| **Obstacle counts** | 트랙 수 / control 전달 수 / **cap 으로 버려진 수** / 랜드마크 수 | **dropped>0** = control 이 못 본 장애물이 있다 |
| **Pipeline health** | `arr` Hz, `/scan` Hz, `arr` 나이 | arr 이 scan 을 못 따라감 / **age>500 ms 면 control 이 장애물 비용을 끈다** |
| **Nearest obstacle** | 최근접 거리·방위각 | \|bearing\|>90° = 라이다 뒤쪽이라 관측 불가 |
| **Track speed** | 최고 속도, 0.5 이상 트랙 수 | 달리는 차가 0 으로 보이는 구간 |
| **[GT] Opponent speed** | GT vs 추정 | 재획득 직후 추정이 0 에서 기어오름 |
| **[GT] Error / age** | 위치·yaw 오차, 트랙 나이 | **나이가 1 로 리셋될 때마다** 오차가 튐 |

`[GT]` 패널은 **두 차량이 mocap 에 잡힌 bag**에서만 값이 나온다(예: `0821/03_13_15`).
아니면 그냥 비어 있고, 나머지는 정상 동작한다.

## 읽는 법 — 세 패널을 같이 봐야 하는 이유

**`count/dropped_by_cap`** 이 0 이 아니면 control 의 `MAX_OBS=5` 에서 잘린 것이다.
자르는 기준이 **거리순이 아니라 배열 순서**라, 가까운 장애물이 6 번째면 그냥 버려진다.

**`[GT] Error / age`** 의 트랙 나이가 1 로 리셋되는 순간마다:
- **Opponent speed** 의 추정선이 0 으로 떨어졌다가 ~20 프레임에 걸쳐 GT 를 따라잡고
- **Error** 의 yaw 오차가 80° 이상으로 뛴다(속도가 0 이면 진행방향이 정의되지 않음)
- 그 구간 내내 control 은 이 차를 **정지로 본다**(0.5 m/s 기준)

**세 패널이 같은 사건의 세 얼굴**이다. 따로 보면 각각 다른 원인으로 오해하게 된다.

## 발행 토픽

전부 `std_msgs/Float32` 라 스키마 설정 없이 어떤 Plot 패널에서도 쓸 수 있다.

```
/pmon/count/{tracks,landmarks,tracks_to_control,dropped_by_cap}
/pmon/rate/{arr_hz,scan_hz}   /pmon/age/arr_ms
/pmon/nearest/{distance,bearing_deg}
/pmon/speed/{max,n_above_static}
/bench/...        (GT 있을 때만; opponent-benchmark-0821/FOXGLOVE.md 참고)
```

> QoS 는 비대칭이다: 구독 `BEST_EFFORT`(라이브 센서와 매칭되고 bag 의 `RELIABLE`
> 발행자와도 매칭), 발행 `RELIABLE`(Foxglove Plot 이 부하 시 BEST_EFFORT 를 흘려 선이
> 끊긴다). 반대로 두면 아무것도 안 그려진다 — 실제로 한 번 겪었다.

## 검증

`0821/03_13_15` 재생으로 전 토픽 실측 확인:
`arr 39.5 Hz`, `scan 40.0 Hz`, `age 24.9 ms`, `landmarks 4`, `nearest 3.79 m @ 22°`.
