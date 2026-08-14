# MCL bag 재생 정확도 비교

bag에 이미 기록된 `/newmcl_pose`와 같은 bag으로 새로 계산한 MCL을 mocap
ego pose에 대해 비교한다. `0730/`은 2026-07-30 실차 bag 전용 설정과 실행을
담고, 상위 파일은 이전 record5 재생 설정이다.

권장 진입점은 인자를 명시하는 다음 스크립트다.

```bash
bash eval/localization-bag-replay/0730/run_bag.sh \
  BAG_PATH MCL_PARAMS_YAML LANE_CSV OUTPUT_PREFIX
```

`A`는 baked MCL, `B`는 replay MCL이다. bag의 `/map`과 별도 lane CSV가 같은
트랙 자산인지 확인해야 한다. `/odom` topic 존재와 `odom -> base_link` TF
존재도 각각 확인한다.
