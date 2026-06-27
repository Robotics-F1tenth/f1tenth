#!/usr/bin/env python3
"""
MCL parameter optimizer using bag file replay.

Usage:
  python3 optimize_mcl_params.py /path/to/bag

전략: Hill-climbing + 점진적 탐색 축소 (simulated annealing 스타일)
- bag에서 /map도 함께 재생하므로 map_server 불필요
- /mocap_pose1 을 ego 차량 ground truth로 사용
"""

import subprocess
import time
import math
import random
import yaml
import os
import sys
import signal
import copy

# ──────────────────────────────────────────────────────────────
# Config  ← 여기만 수정하면 됩니다
# ──────────────────────────────────────────────────────────────
BAG_PATH    = '/home/user/bags/rosbag2_2026_03_13-21_52_51'

PARAMS_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'params', 'mcl_params.yaml')

RESULT_FILE    = '/tmp/mcl_eval_result.txt'
LOG_FILE       = '/tmp/mcl_optim_log.txt'

GT_TOPIC       = '/mocap_pose1'   # ego 차량 ground truth (확인됨)
MCL_TOPIC      = '/mcl_pose'

EVAL_DURATION  = 45.0    # 평가 시간 (초) — bag이 57초이므로 45초 이내
SKIP_SECONDS   = 8.0     # MCL 수렴 대기 시간
MAX_ITERATIONS = 20      # 최대 최적화 반복 횟수

# bag 재생 시 제외할 토픽 (새 MCL 출력과 충돌 방지)
BAG_EXCLUDE_TOPICS = [
    '/mcl_pose', '/mcl_particles', '/aligned_scan',
    '/f1_state', '/lpf_imu', '/estimate_odom', '/score',
]

# ──────────────────────────────────────────────────────────────
# 최적화 파라미터 탐색 공간
# ──────────────────────────────────────────────────────────────
SEARCH_SPACE = {
    'particle_num':             {'type': 'int',   'min': 150,   'max': 800,   'current': 450},
    'odom_noise_ddm_0':         {'type': 'float', 'min': 0.01,  'max': 0.5,   'current': 0.1},
    'odom_noise_ddm_1':         {'type': 'float', 'min': 0.01,  'max': 0.5,   'current': 0.1},
    'odom_noise_ddm_2':         {'type': 'float', 'min': 0.01,  'max': 0.5,   'current': 0.1},
    'odom_noise_ddm_3':         {'type': 'float', 'min': 0.01,  'max': 0.5,   'current': 0.1},
    'var_hit':                  {'type': 'float', 'min': 0.005, 'max': 0.15,  'current': 0.025},
    'z_hit':                    {'type': 'float', 'min': 0.4,   'max': 0.9,   'current': 0.65},
    'scan_step':                {'type': 'int',   'min': 1,     'max': 10,    'current': 6},
    'random_particles_noise_xy':{'type': 'float', 'min': 0.01,  'max': 0.2,   'current': 0.05},
    'random_particles_noise_yaw':{'type': 'float','min': 0.005, 'max': 0.1,   'current': 0.02},
}
# ──────────────────────────────────────────────────────────────


def load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def save_yaml(path, data):
    with open(path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, allow_unicode=True)


def apply_params_to_yaml(yaml_data, params):
    p = yaml_data['mcl']['ros__parameters']

    if 'particle_num' in params:
        p['particle_num'] = int(params['particle_num'])
    if 'var_hit' in params:
        p['var_hit'] = round(float(params['var_hit']), 6)
    if 'z_hit' in params:
        p['z_hit'] = round(float(params['z_hit']), 4)
    if 'scan_step' in params:
        p['scan_step'] = int(params['scan_step'])

    noise_keys = ['odom_noise_ddm_0', 'odom_noise_ddm_1',
                  'odom_noise_ddm_2', 'odom_noise_ddm_3']
    if any(k in params for k in noise_keys):
        cur = list(p.get('odom_noise_ddm', [0.1, 0.1, 0.1, 0.1]))
        for i, k in enumerate(noise_keys):
            if k in params:
                cur[i] = round(float(params[k]), 4)
        p['odom_noise_ddm'] = cur

    cur_noise = list(p.get('random_particles_noise', [0.05, 0.05, 0.02]))
    if 'random_particles_noise_xy' in params:
        v = round(float(params['random_particles_noise_xy']), 4)
        cur_noise[0] = v
        cur_noise[1] = v
    if 'random_particles_noise_yaw' in params:
        cur_noise[2] = round(float(params['random_particles_noise_yaw']), 4)
    p['random_particles_noise'] = cur_noise

    return yaml_data


def read_result():
    if not os.path.exists(RESULT_FILE):
        return None
    with open(RESULT_FILE, 'r') as f:
        for line in f:
            if line.startswith('rmse='):
                return float(line.strip().split('=')[1])
    return None


def kill_ros_nodes():
    """실행 중인 MCL 관련 ROS2 프로세스 정리."""
    for name in ['mcl', 'evaluate_localization', 'rosbag2']:
        subprocess.run(['pkill', '-f', name], capture_output=True)
    time.sleep(2.0)


def run_evaluation(params_file):
    """MCL 노드 + bag 재생 + 평가기 실행 → RMSE 반환."""
    if os.path.exists(RESULT_FILE):
        os.remove(RESULT_FILE)

    kill_ros_nodes()
    procs = []

    try:
        # 1) bag 먼저 재생 → /clock 토픽 공급
        bag_cmd = ['ros2', 'bag', 'play', BAG_PATH, '--clock', '--rate', '1.0']
        for t in BAG_EXCLUDE_TOPICS:
            bag_cmd += ['--exclude-topics', t]
        bag_proc = subprocess.Popen(
            bag_cmd,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid)
        procs.append(bag_proc)
        time.sleep(2.0)  # /clock + /tf_static 안정화 대기

        # 2) MCL 노드 (use_sim_time=true)
        mcl_proc = subprocess.Popen(
            ['ros2', 'run', 'als_ros', 'mcl',
             '--ros-args', '--params-file', params_file,
             '-p', 'use_sim_time:=true'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid)
        procs.append(mcl_proc)
        time.sleep(3.0)  # MCL 초기화 대기

        # 3) 평가 노드 (use_sim_time=true)
        eval_cmd = [
            'ros2', 'run', 'als_ros', 'evaluate_localization.py',
            '--ros-args',
            '-p', f'gt_topic:={GT_TOPIC}',
            '-p', f'mcl_topic:={MCL_TOPIC}',
            '-p', f'duration:={EVAL_DURATION}',
            '-p', f'skip_seconds:={SKIP_SECONDS}',
            '-p', f'result_file:={RESULT_FILE}',
            '-p', 'use_sim_time:=true',
        ]
        eval_proc = subprocess.Popen(
            eval_cmd,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            preexec_fn=os.setsid)
        procs.append(eval_proc)

        # 평가 완료 대기
        try:
            stdout, _ = eval_proc.communicate(timeout=EVAL_DURATION + 25.0)
            if stdout:
                for line in stdout.decode(errors='ignore').splitlines():
                    if 'RMSE' in line or 'done' in line.lower():
                        print(f'  [eval] {line.strip()}')
        except subprocess.TimeoutExpired:
            print('  [WARN] Evaluation timed out')

    except Exception as e:
        print(f'  [ERROR] {e}')
    finally:
        for p in procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
            except Exception:
                try:
                    p.kill()
                except Exception:
                    pass
        time.sleep(3.0)
        kill_ros_nodes()

    return read_result()


def perturb(current_params, scale):
    """현재 파라미터에서 scale 크기로 랜덤 탐색."""
    new_params = {}
    for k, v in current_params.items():
        sp = SEARCH_SPACE[k]
        rng = sp['max'] - sp['min']
        if sp['type'] == 'int':
            delta = random.randint(-max(1, int(rng * scale)), max(1, int(rng * scale)))
            val = max(sp['min'], min(sp['max'], int(v) + delta))
        else:
            delta = random.gauss(0, rng * scale)
            val = max(sp['min'], min(sp['max'], float(v) + delta))
        new_params[k] = val
    return new_params


def fmt_params(p):
    return (f"N={int(p['particle_num'])} "
            f"noise=[{p['odom_noise_ddm_0']:.3f},{p['odom_noise_ddm_1']:.3f},"
            f"{p['odom_noise_ddm_2']:.3f},{p['odom_noise_ddm_3']:.3f}] "
            f"var_hit={p['var_hit']:.4f} z_hit={p['z_hit']:.3f} "
            f"step={int(p['scan_step'])}")


def log(msg):
    ts = time.strftime('%H:%M:%S')
    line = f'[{ts}] {msg}'
    print(line)
    with open(LOG_FILE, 'a') as f:
        f.write(line + '\n')


def main():
    print('=' * 65)
    print('MCL Parameter Optimizer')
    print(f'  Bag:       {BAG_PATH}')
    print(f'  GT topic:  {GT_TOPIC}')
    print(f'  Params:    {PARAMS_FILE}')
    print(f'  Max iter:  {MAX_ITERATIONS}  |  Eval: {EVAL_DURATION}s/iter')
    print(f'  Log:       {LOG_FILE}')
    print('=' * 65)

    with open(LOG_FILE, 'w') as f:
        f.write(f'MCL Optimizer started at {time.strftime("%Y-%m-%d %H:%M:%S")}\n')

    current_params = {k: v['current'] for k, v in SEARCH_SPACE.items()}
    best_params = copy.deepcopy(current_params)
    best_rmse = 9999.0
    no_improve_count = 0

    for iteration in range(MAX_ITERATIONS):
        print(f'\n{"─"*65}')
        print(f'Iteration {iteration+1}/{MAX_ITERATIONS}')
        print(f'  Params: {fmt_params(current_params)}')

        # yaml 수정
        yaml_data = load_yaml(PARAMS_FILE)
        yaml_data = apply_params_to_yaml(yaml_data, current_params)
        save_yaml(PARAMS_FILE, yaml_data)

        log(f'Iter {iteration+1}/{MAX_ITERATIONS} | {fmt_params(current_params)}')

        rmse = run_evaluation(PARAMS_FILE)

        if rmse is None:
            log(f'  → No result (evaluation failed), retrying with best params')
            current_params = perturb(best_params, scale=0.25)
            no_improve_count += 1
            continue

        log(f'  → RMSE = {rmse:.4f} m  (best so far: {best_rmse:.4f} m)')

        if rmse < best_rmse:
            best_rmse = rmse
            best_params = copy.deepcopy(current_params)
            no_improve_count = 0
            log(f'  *** NEW BEST *** RMSE={best_rmse:.4f} m')

            # 최적 파라미터 즉시 저장
            yaml_data = load_yaml(PARAMS_FILE)
            yaml_data = apply_params_to_yaml(yaml_data, best_params)
            save_yaml(PARAMS_FILE, yaml_data)
            print(f'  Best params saved → {PARAMS_FILE}')
        else:
            no_improve_count += 1

        # 탐색 범위: 개선 없으면 축소 (exploitation), 개선 있으면 유지 (exploration)
        scale = max(0.08, 0.35 * (0.85 ** no_improve_count))
        current_params = perturb(best_params, scale=scale)

    print(f'\n{"="*65}')
    print(f'Optimization complete!')
    print(f'  Best RMSE : {best_rmse:.4f} m')
    print(f'  Best params: {fmt_params(best_params)}')
    print(f'  Saved to  : {PARAMS_FILE}')
    print(f'  Full log  : {LOG_FILE}')
    log(f'DONE | Best RMSE={best_rmse:.4f}m | {fmt_params(best_params)}')

    # 최종 저장
    yaml_data = load_yaml(PARAMS_FILE)
    yaml_data = apply_params_to_yaml(yaml_data, best_params)
    save_yaml(PARAMS_FILE, yaml_data)


if __name__ == '__main__':
    random.seed(42)
    main()
