#!/usr/bin/env python3
"""여러 차량의 캘리브레이션 표본을 한꺼번에 풀어 params yaml 을 쓴다.

    python3 calibration_solve_joint.py \
        car3=/tmp/car3_samples.csv:params/params3.yaml \
        car4=/tmp/car4_samples.csv:params/params4.yaml
    python3 calibration_solve_joint.py --selftest

왜 한꺼번에 푸나
----------------
변환 사슬은 T_map_base = T_map_mocap o T_mocap_point(측정) o T_point_base 다.

  T_map_mocap  : 모캡 원점과 맵 원점 사이의 변환. **방과 맵의 성질이지 차량의 성질이
                 아니다.** 두 차가 반드시 같은 값을 써야 한다.
  T_point_base : 마커 중심 -> 후륜축. 마커를 어떻게 붙였느냐이므로 **차량별**이다.

calibration_v2.py 는 차량마다 6개 파라미터를 독립적으로 적합했다. 그 결과 두 차의
T_map_mocap 이 1.43도 / 9.28 cm 어긋났고, 주행영역에서 두 차의 맵 프레임이 p50
13 cm 벌어졌다. 강체변환에 불변이어야 할 두 차 사이 거리마저 p50 10 cm 왜곡됐다
(계통 bias +5.8 cm). 상세: docs/mocap-calibration-analysis-2026-08.md §2

왜 독립 적합이 어긋나나 (축퇴)
------------------------------
yaw 잔차는 yaw_pred = a + yaw_moc + b 라서 **a 와 b 가 합으로만 나타난다.** a 를
따로 묶어주는 건 위치식의 R(a)·p_moc 항뿐인데, 손으로 찍은 소수 표본에서는 이 항의
정보가 약해 a 가 얕은 골짜기에 놓이고 세션마다 다른 값으로 착지한다. 두 차의 표본을
합쳐 a 를 **하나의 미지수로** 풀면 이 방향이 양쪽 데이터로 함께 구속된다.

이 스크립트는 a, tx, ty 를 공유 미지수로, b_i, bx_i, by_i 를 차량별 미지수로 둔
가우스-뉴턴(LM) 문제를 푼다.
"""
import math
import sys

import numpy as np
import yaml


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def predict(shared, per_car, moc):
    """moc: (N,3) [x, y, yaw] (축변환까지 끝난 mocap pose) -> 예측 base pose."""
    a, tx, ty = shared
    b, bx, by = per_car
    c, s = math.cos(a), math.sin(a)
    x1 = tx + c * moc[:, 0] - s * moc[:, 1]
    y1 = ty + s * moc[:, 0] + c * moc[:, 1]
    yaw1 = moc[:, 2] + a
    c1, s1 = np.cos(yaw1), np.sin(yaw1)
    return np.column_stack([x1 + c1 * bx - s1 * by,
                            y1 + s1 * bx + c1 * by,
                            yaw1 + b])


def residuals(params, cars):
    """params = [a, tx, ty, b0, bx0, by0, b1, bx1, by1, ...]"""
    shared = params[:3]
    out = []
    for i, (moc, mcl) in enumerate(cars):
        pr = predict(shared, params[3 + 3 * i: 6 + 3 * i], moc)
        out.append(pr[:, 0] - mcl[:, 0])
        out.append(pr[:, 1] - mcl[:, 1])
        out.append(np.array([wrap(v) for v in pr[:, 2] - mcl[:, 2]]))
    return np.concatenate(out)


def solve(cars, iters=200):
    n = len(cars)
    # 초기값: yaw 차이의 원형평균으로 a, 그 a 로 맞춘 평행이동으로 t, b 는 0
    diffs = np.concatenate([[wrap(m - o) for m, o in zip(mcl[:, 2], moc[:, 2])]
                            for moc, mcl in cars])
    a0 = math.atan2(np.sin(diffs).mean(), np.cos(diffs).mean())
    R = np.array([[math.cos(a0), -math.sin(a0)], [math.sin(a0), math.cos(a0)]])
    t0 = np.concatenate([mcl[:, :2] - (R @ moc[:, :2].T).T
                         for moc, mcl in cars]).mean(axis=0)
    params = np.concatenate([[a0, t0[0], t0[1]], np.zeros(3 * n)])

    lam = 1e-3
    for _ in range(iters):
        r = residuals(params, cars)
        cost = 0.5 * r @ r
        J = np.zeros((len(r), len(params)))
        for k in range(len(params)):
            p2 = params.copy()
            step = 1e-7
            p2[k] += step
            J[:, k] = (residuals(p2, cars) - r) / step
        H = J.T @ J + lam * np.eye(len(params))
        try:
            dp = -np.linalg.solve(H, J.T @ r)
        except np.linalg.LinAlgError:
            dp = -np.linalg.lstsq(H, J.T @ r, rcond=None)[0]
        cand = params + dp
        cand[0] = wrap(cand[0])
        for i in range(n):
            cand[3 + 3 * i] = wrap(cand[3 + 3 * i])
        if 0.5 * residuals(cand, cars) @ residuals(cand, cars) < cost:
            params = cand
            lam = max(1e-8, lam * 0.7)
            if np.linalg.norm(dp) < 1e-11:
                break
        else:
            lam = min(1e4, lam * 2.5)
    return params


def rot3(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0]


def write_params(path, shared, per_car):
    with open(path) as f:
        data = yaml.safe_load(f)
    p = data["/**"]["ros__parameters"]
    a, tx, ty = shared
    b, bx, by = per_car
    p["rotation"] = rot3(a)
    p["translation"] = [float(tx), float(ty), 0.0]
    p["tracked_point_to_base_rotation"] = rot3(b)
    p["tracked_point_to_base_translation"] = [float(bx), float(by), 0.0]
    with open(path, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=None, width=120)


def load(path):
    d = np.genfromtxt(path, delimiter=",", names=True)
    moc = np.column_stack([d["x_moc"], d["y_moc"], np.asarray(d["yaw_moc"])])
    mcl = np.column_stack([d["x_mcl"], d["y_mcl"], np.asarray(d["yaw_mcl"])])
    return moc, mcl


def report(name, shared, per_car, moc, mcl):
    pr = predict(shared, per_car, moc)
    e = np.hypot(pr[:, 0] - mcl[:, 0], pr[:, 1] - mcl[:, 1])
    ey = np.abs([wrap(v) for v in pr[:, 2] - mcl[:, 2]])
    print(f"  {name}: n={len(moc)}  잔차 p50={np.percentile(e,50)*100:.2f} cm  "
          f"p90={np.percentile(e,90)*100:.2f} cm  "
          f"yaw p50={math.degrees(np.percentile(ey,50)):.2f} deg")
    print(f"      T_point_base: yaw={math.degrees(per_car[0]):+.3f} deg  "
          f"t=[{per_car[1]:+.4f}, {per_car[2]:+.4f}]")


def selftest():
    rng = np.random.default_rng(0)
    a_true, t_true = math.radians(-91.0), np.array([0.09, -4.70])
    truths = [(math.radians(3.0), 0.05, -0.02), (math.radians(-2.0), -0.03, 0.04)]
    cars = []
    for b_true in truths:
        n = 800
        moc = np.column_stack([rng.uniform(-5, 5, n), rng.uniform(-5, 5, n),
                               rng.uniform(-math.pi, math.pi, n)])
        mcl = predict((a_true, t_true[0], t_true[1]), b_true, moc)
        mcl[:, :2] += rng.normal(0, 0.005, (n, 2))
        cars.append((moc, mcl))
    p = solve(cars)
    assert abs(wrap(p[0] - a_true)) < math.radians(0.2), math.degrees(p[0])
    assert np.allclose(p[1:3], t_true, atol=0.01), p[1:3]
    for i, b_true in enumerate(truths):
        assert abs(wrap(p[3 + 3*i] - b_true[0])) < math.radians(0.3)
        assert np.allclose(p[4 + 3*i: 6 + 3*i], b_true[1:], atol=0.01)
    # 표본이 넓게 퍼져 있으면 독립 적합도 a 를 맞춘다. 축퇴는 실제 운용처럼
    # 표본이 적고 좁게 모여 있을 때 드러나므로, 그 조건을 따로 재현한다.
    a_each_rich = [math.degrees(solve([c])[0]) for c in cars]

    sparse = []
    for b_true in truths:
        n = 4  # calibration_v2.py 의 최소 표본 수 부근
        moc = np.column_stack([rng.uniform(-0.6, 0.6, n), rng.uniform(-0.6, 0.6, n),
                               rng.uniform(-0.4, 0.4, n)])
        mcl = predict((a_true, t_true[0], t_true[1]), b_true, moc)
        mcl[:, :2] += rng.normal(0, 0.02, (n, 2))
        sparse.append((moc, mcl))
    a_sparse_each = [math.degrees(solve([c])[0]) for c in sparse]
    a_sparse_joint = math.degrees(solve(sparse)[0])
    spread_each = max(a_sparse_each) - min(a_sparse_each)

    print(f"selftest OK")
    print(f"  참값 a = {math.degrees(a_true):.3f} deg")
    print(f"  표본 많고 넓음: 공유 {math.degrees(p[0]):.3f} / "
          f"독립 {[round(v,3) for v in a_each_rich]}  -> 둘 다 맞음")
    print(f"  표본 적고 좁음: 공유 {a_sparse_joint:.3f} / "
          f"독립 {[round(v,3) for v in a_sparse_each]}  -> 독립은 차마다 "
          f"{spread_each:.2f} deg 갈림")
    print(f"  (실측 두 params 의 a 차이는 1.43 deg 였다)")


def main():
    args = sys.argv[1:]
    if not args or args[0] == "--selftest":
        return selftest()
    names, paths, cars = [], [], []
    for spec in args:
        name, rest = spec.split("=", 1)
        csv_path, yaml_path = rest.rsplit(":", 1)
        names.append(name); paths.append(yaml_path)
        cars.append(load(csv_path))
        print(f"{name}: {csv_path} -> {yaml_path} ({len(cars[-1][0])} 표본)")
    if len(cars) < 2:
        print("\n경고: 차량이 1대뿐이라 T_map_mocap 이 그 차 데이터로만 구속됩니다.\n"
              "      두 대를 함께 풀어야 축퇴가 풀립니다.\n")

    p = solve(cars)
    shared = p[:3]
    print(f"\n공유 T_map_mocap: yaw={math.degrees(shared[0]):+.4f} deg  "
          f"t=[{shared[1]:+.4f}, {shared[2]:+.4f}]")
    for i, name in enumerate(names):
        per = p[3 + 3*i: 6 + 3*i]
        report(name, shared, per, *cars[i])
        write_params(paths[i], shared, per)
        print(f"      -> {paths[i]} 갱신")
    print("\n두 params 의 rotation/translation 은 이제 정의상 동일합니다.")


if __name__ == "__main__":
    main()
