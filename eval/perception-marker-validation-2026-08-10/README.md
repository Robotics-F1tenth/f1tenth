# 2026-08-10 static marker 및 bbox bag 검증 결과

`results/`는 live-recorded bag과 replay/hold-0.5 후보 bag을 분석한 JSON
아카이브다. 분석기는 이 폴더가 아니라 다음 패키지 도구가 원본이다.

```bash
python3 src/perception/tools/analyze_static_marker_bag.py \
  BAG_PATH --output SUMMARY.json
```

판정 근거와 결과 표는
`src/perception/docs/static-obstacle-marker-bag-validation-2026-08-10.md`를
따른다. JSON은 재생성 가능한 산출물이므로 Git에는 포함하지 않는다.
