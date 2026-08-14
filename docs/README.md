# F1TENTH 프로젝트 문서

이 디렉터리는 여러 패키지에 걸친 문서와, 별도 Git 저장소로 공유되지 않는
패키지의 운영·분석 문서를 최상위 `f1tenth` 저장소를 통해 공유한다. 자체 Git
저장소로 공유되는 패키지의 전용 문서는 해당 패키지 `docs/`에 둔다.

## 범주

| 범주 | 문서 |
|---|---|
| Localization | [Localization 버전 비교](localization-version-comparison.md) |
| Mocap | [차량별 mocap calibration과 GT 정확도 분석](mocap-calibration-analysis-2026-08.md) |
| Vehicle | [VESC deadman 해제 후 정지 명령 부재 조사](vehicle/vesc-deadman-command-timeout-investigation.md) |
| Seminar | [`seminar/`](seminar/) 아래 날짜별 발표 자료 |

## 파일명 규칙

- 기능·대상·문서 목적이 드러나는 kebab-case 영문 파일명을 사용한다.
- 시점이 중요한 분석은 `-YYYY-MM` 또는 `-YYYY-MM-DD`를 붙인다.
- 실행 절차는 `workflow`, 조사 기록은 `investigation`, 결정 전 검토는
  `review`로 끝낸다.
- 자체 Git 저장소로 공유되는 패키지의 전용 문서는 패키지 `docs/`에 둔다.
  여러 패키지에 걸친 문서나 별도 Git 공유가 없는 패키지 문서는 이 최상위
  디렉터리에 둔다.
