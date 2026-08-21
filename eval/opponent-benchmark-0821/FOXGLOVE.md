# Foxglove 로 상대차 추적을 라이브로 보기

bag 을 재생하면서 GT·추정·오차를 실시간으로 그린다. 후처리 없이 바로 본다.

## 1. 띄우기

```bash
# 터미널 1 — 오차 계산 노드
cd eval/opponent-benchmark-0821
python3 live_gt_error.py --ros-args -p use_sim_time:=true

# 터미널 2 — bag 재생
ros2 bag play /home/a/Downloads/f1tenth_bag/0821/rosbag2_2026_08_21-03_13_15 --clock

# 터미널 3 — Foxglove 가 붙을 브리지
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

Foxglove 에서 `ws://localhost:8765` 로 연결 → **Layout → Import from file** →
`foxglove_layout_bench.json`.

상대차 토픽이 다른 bag 이면:
```bash
python3 live_gt_error.py --ros-args -p opponent_topic:=/mocap/car4/pose
```

## 2. 패널 구성 (6개)

| 패널 | 내용 | 봐야 할 것 |
|---|---|---|
| **Speed** | GT vs 추정 | 재획득 직후 추정이 **0 에서 기어오르는** 구간 |
| **Yaw** | GT 진행방향 vs 추정 | 속도가 0 근처일 때 yaw 도 같이 튄다 |
| **Error** | 위치·속도 오차 | 위치는 대체로 ±0.3 m, 속도는 재획득 때 −2 m/s 까지 |
| **Yaw error** | \|yaw 오차\| | 큰 값은 거의 전부 신생 트랙 |
| **Track age / id** | 나이와 id | **id 가 바뀌는 순간 = 재획득**. 나이가 1 로 리셋됨 |
| **Range** | ego→상대 거리 | 8 m 를 넘으면 검출이 0 이 된다 |

## 3. 읽는 법

**핵심은 Track age 패널을 같이 보는 것이다.** 나이가 1 로 리셋되는 순간마다:

- Speed 의 추정선이 **0 으로 떨어졌다가** 20 프레임에 걸쳐 GT 를 따라잡는다
- 그 구간에서 Yaw error 가 **80° 이상**으로 뛴다 (속도가 0 이면 진행방향이 정의되지 않음)
- Speed 패널의 **0.5 기준선 아래**에 있는 동안 control 은 이 차를 정지로 본다

즉 세 패널이 **같은 사건의 세 얼굴**이다. 따로 보면 원인을 놓친다.

**끊긴 구간**: 상대가 매칭되지 않은 사이클에는 아무것도 발행하지 않는다. 그래서 그래프의
공백이 곧 검출 공백이다 — 채워 그리면 커버리지를 과장하게 되므로 일부러 비워둔다.
그 공백의 96.5 % 는 맵 벽에 가렸거나 ego 뒤쪽이라 **불가피**하다(`docs/benchmarks.md` §3-4).

## 4. 발행 토픽

전부 `std_msgs/Float32` 라 어떤 Plot 패널에서도 스키마 설정 없이 쓸 수 있다.

```
/bench/gt/speed  /bench/est/speed  /bench/err/speed
/bench/gt/yaw    /bench/est/yaw    /bench/err/yaw
/bench/err/position   /bench/range   /bench/track/age   /bench/track/id
```

> QoS 는 비대칭이다: 구독은 `BEST_EFFORT`(라이브 센서와 매칭되고, bag 의 `RELIABLE`
> 발행자와도 매칭됨), 발행은 `RELIABLE`(Foxglove Plot 이 부하 시 BEST_EFFORT 샘플을
> 흘려서 선이 끊긴다). 반대로 두면 아무것도 안 그려진다.
