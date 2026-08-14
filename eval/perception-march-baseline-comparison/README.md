# Perception 3월 기준 커밋 vs 현재 코드 — 동일 bag 비교

두 버전을 **같은 bag**으로 나란히 띄워서 눈으로 비교하고 화면 녹화하기 위한 것.
정량 비교 기준은 `src/perception/docs/march-baseline-comparison-criteria.md` 참고.

## 준비 상태 (이미 되어 있음)

| | 위치 | 상태 |
|---|---|---|
| 현재 코드 | `~/f1tenth` | min_age 0.3 포함, 빌드됨 |
| **3월 원본** (`4b3f601`) | `~/f1tenth_march` | git worktree로 격리, 빌드됨 |

원본은 **별도 워크스페이스**라 현재 워크스페이스를 전혀 건드리지 않는다.
(`git worktree`이므로 `~/f1tenth/src/perception`의 파일은 그대로.)

## 실행

```bash
# 현재 버전
bash ~/f1tenth/eval/perception-march-baseline-comparison/run_perception.sh current 22_48_25

# 3월 원본
bash ~/f1tenth/eval/perception-march-baseline-comparison/run_perception.sh march 22_48_25
```

`Ctrl-C`로 종료하면 노드·rviz 정리와 yaml 복원까지 자동으로 된다.

- 쓸 수 있는 bag: `22_25_24` `22_28_57` `22_32_09` `22_35_37` `22_41_13`(맵훼손) `22_48_25` `22_52_38` `22_55_09`
- 장애물 있는 bag: `22_35_37` `22_41_13` `22_48_25` `22_52_38`
- 장애물 없는 대조군: `22_25_24` `22_28_57` `22_32_09` `22_55_09`
- `ROS_DOMAIN_ID=57`로 격리되므로 형이 따로 띄워둔 라이브 세션과 안 섞인다.
- centerline csv(map1/map7)는 bag에 맞춰 스크립트가 자동으로 맞춘다(끝나면 원복).

## 영상 찍을 때 볼 포인트

**3월 원본**
- 트래킹이 **딱 하나만** 뜬다 — `TrackManagement`가 `measure.at(0)` 하나만 쓰고
  주석에 "무조건 하나만 유지". 여러 물체가 있어도 하나만 잡힌다.
- corridor 필터·벽스텁 억제가 없어서 **벽 조각이 장애물로 튀어나온다.**
- 정적장애물 개념 자체가 없다(오렌지 원반 없음).

**현재**
- 다중 트래킹, 정적장애물(오렌지 원반) 별도 발행.
- 랩을 돌아도 정적장애물 개수가 안 불어난다(min_age 수정 이후).

## 주의

- 원본 launch는 인자를 안 받는다(`use_sim_time`이 내부에 하드코딩 True). 스크립트가 알아서 분기한다.
- 원본에는 `f1tenth.rviz`가 없어서 원래 launch의 rviz 노드가 주석처리돼 있었다.
  스크립트는 양쪽에 다 있는 `f1test.rviz`를 따로 띄운다.
- **랜드마크 0개로 나오면 결과가 아니라 캡처 실패 신호**일 수 있다(`static_msgs=0` 확인 후 재실행).
