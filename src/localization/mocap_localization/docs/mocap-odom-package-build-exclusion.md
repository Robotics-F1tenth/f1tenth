# `mocap_odom` 패키지 빌드 제외 기록

## 상태와 목적

`src/f1_localization/mocap_odom/`는 ROS 2 Python 패키지다. 이 패키지는
`/rigid_bodies` (`mocap4r2_msgs/msg/RigidBodies`)를 받아 모션캡 pose의 프레임 간
차분으로 속도와 yaw rate를 계산하고, `nav_msgs/msg/Odometry`를 만든다.

기본 입출력은 다음과 같다.

```text
/rigid_bodies  ->  mocap_odom_node  ->  /mocap_odom
                                    frame_id: mocap_odom
                                    child_frame_id: base_link
```

즉 MCL이나 제어기를 직접 구현한 노드가 아니라, 모션캡 기반의 "이상적인
odometry"를 공급하려던 변환기다. 목적상 VESC/휠 기반 `/odom`의 속도·운동학
오차를 제거한 상태에서 MCL motion model을 검증하거나, 위치추정 오차를 제외한
제어 성능을 평가하려는 실험용 코드였을 가능성이 높다. 이는 패키지가 추가된
커밋 메시지(`오차 15cm version`)와도 부합하지만, 의도에 대한 이 부분은 코드와
이력에 기반한 추정이다.

## 현재 위치추정 구성과의 관계

| 구성요소 | 입력 | 출력 | 현재 역할 |
| --- | --- | --- | --- |
| `mocap_odom` | `/rigid_bodies` | `/mocap_odom` (`Odometry`) | 모션캡 pose에서 유도한 odom (현재 미사용) |
| `mocap_localization` | `/rigid_bodies` | `/mocap_pose1`, `/mocap_pose2` (`PoseStamped`) | 좌표축·맵·tracked point-to-`base_link` 보정을 적용한 모션캡 기준 pose |
| `als_ros/mcl` | `/scan`, `/odom`, `/imu/data`, map | `/newmcl_pose` | LiDAR 기반 MCL |

현재 `mocap_localization/calibration_v2.py`는 `/mocap_pose1`과
`/newmcl_pose`를 비교한다. 따라서 모션캡은 이미 MCL 결과의 ground truth,
보정·오차평가 용도로 사용된다.

MCL은 기본적으로 `/odom`의 `twist.linear.x/y`를 적분해 motion model을
업데이트한다. 따라서 `mocap_odom`을 시험하려면 단순 실행만으로는 부족하며,
출력 `/mocap_odom`을 MCL 입력 `/odom`에 의도적으로 remap해야 한다.

## `COLCON_IGNORE`를 둔 이유

`src/f1_localization/mocap_odom/COLCON_IGNORE`는 0 바이트의 colcon 마커다.
파일 이름 자체가 의미를 가지며, colcon이 해당 디렉터리를 ROS 패키지로 발견·
빌드하지 않게 한다. 파일은 2026-07-24에 생성됐고, 셸 히스토리에 다음 명령이
남아 있다.

```bash
touch ~/f1tenth_ws/src/f1_localization/mocap_odom/COLCON_IGNORE
```

현재 빌드에서 제외하는 이유는 다음과 같다.

1. 어떤 launch 파일·설정·제어 패키지도 `mocap_odom` 또는 `/mocap_odom`을
   사용하지 않는다. 따라서 활성화해도 현재 실행 경로에는 연결되지 않는다.
2. 현재 `mocap_localization`이 모션캡의 `y_up` 축 변환과 `map` 정렬,
   tracked point에서 차량 `base_link`로의 보정을 담당한다. `mocap_odom`은
   이 보정을 수행하지 않고 원시 모션캡 pose를 사용한다.
3. 현재 설치된 `mocap4r2_msgs/RigidBody`의 식별자 필드는
   `rigid_body_name`인데, `mocap_odom`은 `name`을 조회한다. 차량 ID를 지정해도
   찾지 못하고 첫 번째 rigid body로 fallback할 수 있다.
4. 코드상 전진 속도 `v_long`을 `twist.linear.y`에, 횡속도 `-v_lat`을
   `twist.linear.x`에 넣는다. 기본 MCL differential-drive model은
   `linear.x`를 전진거리로 사용하므로, 그대로 `/odom`에 remap하면 운동 방향을
   잘못 해석할 위험이 있다.
5. 별도 launch/파라미터 파일이 없고, 코드가 사용하는 `scipy`도 패키지
   의존성에 선언돼 있지 않다.

따라서 현재 `COLCON_IGNORE`는 사용 중인 `mocap_localization`/MCL 경로를
혼동시키거나, 검증되지 않은 odom을 실수로 실행하는 일을 막기 위한 안전장치다.
패키지를 삭제하지 않은 이유는 모션캡 기반 odom 실험을 재개할 때의 출발점으로
보존하기 위해서다.

## 재활성화 전 확인 사항

다시 사용하려면 `COLCON_IGNORE`를 제거하기 전에 아래를 완료한다.

1. rigid body 선택을 `b.rigid_body_name`으로 수정하고, 실제 차량 ID를
   파라미터로 지정한다.
2. Motive/OptiTrack 축(`y_up` 여부), 맵 좌표계, tracked point → `base_link`
   보정을 현재 `mocap_localization`과 일관되게 적용한다.
3. ROS 표준에 맞게 `linear.x = v_long`, `linear.y = v_lat`인지 확인하고,
   MCL의 `use_omni_directional_model` 설정과 함께 실차·bag으로 검증한다.
4. `scipy` 의존성과 전용 launch/파라미터 파일을 추가한다.
5. MCL 검증용이라면 `/mocap_odom` → `/odom` remap을 명시적으로 적용하고,
   기존 VESC odom과 동시에 `/odom`을 발행하지 않도록 한다.
6. 제어 성능평가용이라면 모션캡 ground truth를 직접 state feedback으로 쓰는
   실험과, MCL pose를 쓰되 모션캡으로 오차를 기록하는 실험을 분리한다.
