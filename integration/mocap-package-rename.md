# `mocap_localization` 패키지 이름 변경 검토

작성: 2026-07-29 · 상태: **미착수 — 팀 논의 필요, 대공사 예상**

---

## 왜 검토하나

`src/localization/mocap_localization` 패키지는 예전엔 MCL(Monte Carlo Localization) 계열
코드를 갖고 있었으나, 그 코드는 전부 삭제됐고 지금은 **모션캡쳐 리지드바디 pose 발행 +
캘리브레이션 스크립트**만 남아 있다. 이름이 실제 역할과 안 맞는다.

## 실제 역할 (확인 완료)

`/mocap_pose1` 토픽 사용처를 전수 조사한 결과, 이 패키지는 **평가용 ground truth(GT) 발행기**로만
쓰이고 있다. 실제 주행에서 소비되는 localization 소스가 아니다:

| 사용처 | 역할 |
|---|---|
| `src/ekf_pose/launch/ekf_evaluate.launch.py` | `gt_topic` 기본값이 `/mocap_pose1` |
| `src/ekf_pose/scripts/ekf_evaluator.py` | `"gt_topic"` 파라미터로 GT 취급 |
| `src/perception/docs/realtime-viz-foxglove.md` | `GT | /mocap_pose1` (문서 표) |
| `src/perception/docs/yaw-lag-reduction-plan.md` | `/newmcl_pose vs /mocap_pose1` RMSE 비교 — MCL 정확도를 검증하는 참값 |

클래스명도 이미 `MocapPosePublisher`([mocap_localization.cpp:12](src/localization/mocap_localization/src/mocap_localization.cpp#L12)), `package.xml`의 `<description>`은 아직 `TODO: Package description`으로 비어 있다.

## 후보 이름 (2026-07-29 논의, 아직 미확정)

- `mocap_gt` — GT 발행이라는 용도가 가장 짧게 드러남. `ekf_evaluator`가 부르는 `gt_topic`
  개념과도 맞아떨어짐.
- `mocap_pose_publisher` — 클래스명(`MocapPosePublisher`)·토픽(`/mocap_pose1`)과 그대로 일치.
  다만 길고, 캘리브레이션 역할은 이름에 안 드러남.

팀 논의 후 확정.

## 바꿀 때 손대야 하는 곳 (전수 확인 완료)

| 파일 | 내용 |
|---|---|
| `package.xml` | `<name>mocap_localization</name>` |
| `CMakeLists.txt` | `project(mocap_localization)`, `add_executable`/`ament_target_dependencies`/`target_include_directories`/`install(TARGETS`에 쓰인 실행파일명 (총 4곳) |
| `launch/mocap_localization.launch.py` | **파일명 자체** + 내부의 `get_package_share_directory('mocap_localization')`, `package=`, `executable=`, `name=` |
| `calibration_v2.py` | 아래 참조 — 겸사겸사 고칠 것 |
| `~/.bashrc:150` | `alias mocap='ros2 launch mocap_localization mocap_localization.launch.py'` — 버전관리 밖이라 각 차량에서 수동 수정 필요 (`bashrc-aliases-pin-launch-args` 메모리 참조) |

## 겸사겸사 고칠 것 — `calibration_v2.py`의 절대경로 하드코딩

같은 파일에 워크스페이스 이름을 하드코딩한 부분이 6곳 있다:

```python
self.launch_path = '/home/a/f1tenth_ws/src/localization/mocap_localization/launch/mocap_localization.launch.py'
self.params_path = '/home/a/f1tenth_ws/src/localization/mocap_localization/params/params.yaml'
```

지금은 4호차·3호차 워크스페이스 이름이 둘 다 `f1tenth_ws`라 우연히 동작하지만,
`integration/workspace-path-unification.md`에서 다룬 것과 정확히 같은 종류의 문제다.
패키지명을 바꾸면 이 경로들도 어차피 다 깨지므로, 이름 변경과 **동시에**
`get_package_share_directory()` 기반으로 바꿔서 워크스페이스 이름 의존성 자체를 없애는 게 낫다.

## 왜 "대공사"인가

- 패키지명은 ROS 2에서 디렉터리명·실행파일명·`ros2 run`/`ros2 launch` 호출부와 전부 얽혀 있어
  한 곳이라도 빠뜨리면 조용히 깨진다 (에러 없이 그냥 옛날 이름으로 빌드되거나, 반대로 빌드 실패).
- `~/.bashrc` alias는 버전관리 밖이라 **3호차에서 별도로 수동 수정**해야 함 — 팀원 조율 필요.
- `params2.yaml`/`mocap_localization2.launch.py` 같은 두 번째 리지드바디용 파일이 `calibration_v2.py`
  안에서 언급되는데 아직 실제로는 존재하지 않음 — 이것도 이름 변경 시 같이 정리할지 결정 필요.

## 다음 세션에서 할 일

1. 팀원과 이름 확정 (`mocap_gt` / `mocap_pose_publisher` / 다른 안)
2. 위 표의 파일들을 한 번에 수정 (부분 적용 시 빌드 깨짐)
3. `calibration_v2.py` 절대경로 → `get_package_share_directory()` 전환
4. 3호차·4호차 양쪽 `~/.bashrc`의 `mocap` alias 동시 수정
5. `colcon build --packages-select <새이름>` 후 `ros2 launch <새이름> <새이름>.launch.py`로 검증
