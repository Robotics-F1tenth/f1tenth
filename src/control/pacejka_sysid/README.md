# pacejka_sysid — 온트랙 Pacejka 타이어 파라미터 식별

ForzaETH **On-Track-SysID** ([github.com/ForzaETH/On-Track-SysID](https://github.com/ForzaETH/On-Track-SysID),
[arXiv:2411.17508](https://arxiv.org/abs/2411.17508)) 를 **ROS 2 Humble** 로 이식한 패키지.

트랙을 30초 정도 주행한 데이터로 타이어의 Pacejka 계수를 식별해서 SMPPI CUDA 컨트롤러의
동역학 모델에 반영한다. 신경망이 공칭 차량모델의 **잔차**를 학습하고, 그 잔차를 더한 모델로
가상의 정상상태 데이터를 만든 뒤, 거기에 Pacejka 곡선을 최소자승 피팅하는 과정을 반복한다.

> **학습 알고리즘은 원본을 그대로 유지했다.** 바꾼 것은 ROS 인터페이스(rospy→rclpy),
> 파일 경로 해석(rospkg→ament), 그리고 우리 차량에는 없는 `v_y` 센서를 IMU로 대체한 부분뿐이다.
> 자세한 변경 내역은 맨 아래 [원본 대비 변경점](#원본-대비-변경점) 참고.

---

## 0. 빌드

```bash
cd ~/f1tenth_ws
colcon build --packages-select pacejka_sysid
source install/setup.bash
```

> ### ⚠ `--symlink-install` 을 쓰지 말 것
> colcon 의 `--symlink-install` 은 ament_python 패키지의 `data_files` 를 설치하지 않는다.
> 그러면 `params/`, `models/`, `launch/` 가 share 에 통째로 빠져서 노드가 시작하자마자 죽는다.
> **`~/.bashrc` 의 `cb` 별칭이 `--symlink-install` 을 쓰므로 이 패키지에는 사용하면 안 된다.**
> (그 경우를 대비해 에러 메시지에 힌트를 넣어 두었다.)

필요한 파이썬 패키지: `torch`, `numpy`, `scipy`, `matplotlib`, `PyYAML`, `tqdm`.
`tqdm` 이 없으면 다음 중 하나로 설치한다.

```bash
sudo apt install python3-tqdm      # 권장 (package.xml 에 rosdep 키로 선언되어 있음)
# 또는
pip3 install --user tqdm
```

---

## 1. 사전 준비 — 브링업 순서

식별에 필요한 토픽은 **`/odom`, `/imu/data`, `/ackermann_cmd`** 세 개뿐이다.
위치추정(MCL)이나 라이다는 **필요 없다.** 자율주행이 아니라 사람이 조종기로 몰기 때문이다.

| 순서 | 명령 (`~/.bashrc` 별칭) | 실제 launch | 나오는 것 |
|---|---|---|---|
| 1 | `imu` | `ros2 launch microstrain_inertial_driver microstrain_launch.py params_file:=~/f1tenth_ws/config/imu_params.yml` | `/imu/data` (100 Hz) |
| 2 | `sensors` | `ros2 launch f1tenth_stack bringup_launch.py` | `/odom` (50 Hz), `/ackermann_cmd` (30 Hz), joy→teleop→mux→VESC |
| 3 | — | (아래 4장) | 식별 노드 |

`sensors` 하나가 `joy`, `joy_teleop`, `vesc_driver`, `ackermann_to_vesc`, `vesc_to_odom`,
`ackermann_mux`, `urg_node`, `base_link→laser` static TF 를 전부 띄운다.

**조종기**: `joy_teleop.yaml` 기준 **데드맨 버튼 4번을 누르고 있어야** 조향/구동이 나간다.
최대 속도 스케일 3.5 m/s, 최대 조향 0.4788 rad.

브링업이 제대로 됐는지 확인:

```bash
ros2 topic hz /odom /imu/data /ackermann_cmd
```

---

## 2. 차량 파라미터 파일

`models/<racecar_version>/<racecar_version>_pacejka.txt` 가 **입력(차량 제원)이자 출력(식별된 계수)** 이다.
기본으로 `models/F1TENTH/F1TENTH_pacejka.txt` 가 들어 있다.

```yaml
I_z: 0.04712     # 요축 관성모멘트 [kg·m²]
h_cg: 0.02       # 무게중심 높이 [m]
l_f: 0.163       # 무게중심 → 전륜축 [m]
l_r: 0.161       # 무게중심 → 후륜축 [m]
l_wb: 0.324      # 휠베이스 [m]  (= l_f + l_r)
m: 3.74          # 차량 질량 [kg]
model_name: F1TENTH
tire_model: pacejka
C_Pf: [B, C, D, E]   # 학습 전 값은 결과에 영향 없음 (원본 README 명시)
C_Pr: [B, C, D, E]
```

- `racecar_version` 이름에 **밑줄(`_`)을 쓰면 안 된다.** 원본 `load_model.py` 가
  `model_name.split("_")` 로 파일명을 파싱한다.
- 학습이 끝나면 `yaml.dump` 가 이 파일을 덮어쓰므로 **주석은 사라진다.**
  덮어써지는 대상은 `output_dir` 로 복사된 사본이고, 소스 트리의 템플릿은 그대로 남는다.

### 값의 근거와 실측이 필요한 것

| 항목 | 값 | 근거 |
|---|---|---|
| `m`, `l_f`, `l_r` | 3.74 / 0.163 / 0.161 | `smppi_cuda_controller/config/params.yaml` |
| `l_wb` | 0.324 | `l_f+l_r`, `f1tenth_stack/config/vesc.yaml` 의 `wheelbase: .324` 와 일치 ✔ |
| **`I_z`** | 0.04712 | ⚠ **미검증.** 아래 참고 |
| **`h_cg`** | 0.02 | ⚠ **미검증.** `generate_lut: false` 면 사용되지 않음 |

#### `I_z` 는 반드시 확인할 것

워크스페이스 안에서 값이 **3.4배 엇갈린다**:
`smppi config/params.yaml` 은 `0.16`, `smppi_node.cpp` 기본값과 `racecar_simulator/params/simulation.yaml` 은 `0.04712`.

`I_z` 는 요 운동방정식 `ω̇ = M / I_z` 에 직접 들어가므로 **식별되는 `D` 를 그대로 스케일한다.**
`I_z` 가 2배 틀리면 `D` 도 대략 2배 틀린다. 식별 전에 확정해야 한다.

- 참조값: ForzaETH NUC6 (`m=3.75 kg`, `l_wb=0.325 m`) → `I_z = 0.056`. 우리 차와 제원이 거의 같다.
- **이선 진자(bifilar pendulum) 실측**: 차를 두 가닥 줄로 수평하게 매달고 요 방향으로 살짝 비틀어
  진동 주기 `T` 를 잰다 (10주기 재서 나누면 정확하다).

  ```
  I_z = m · g · b² · T² / (16 · π² · L)
        b = 두 줄 사이 거리 [m],  L = 줄 길이 [m]
  ```

- 어떤 값을 쓰든 `F1TENTH_pacejka.txt` 의 `I_z` 와 `smppi config/params.yaml` 의 `I_z` 를
  **반드시 같은 값으로 맞춰야** 식별 결과가 컨트롤러에서 의미를 갖는다.

---

## 3. 파라미터 파일 3개

### 3-1. `params/nn_params.yaml` — 학습 하이퍼파라미터 (원본 그대로)

| 키 | 기본값 | 설명 |
|---|---|---|
| `data_collection_duration` | `30` | 데이터 수집 시간 [초]. **가장 중요.** `v_x > 1 m/s` 인 구간만 세므로 실제 주행은 더 길어진다. 시간 여유가 있으면 50~60 으로 늘리면 좋다 |
| `num_of_iterations` | `6` | 학습·피팅 반복 횟수. 매 반복마다 이전 결과를 사전(prior)으로 삼는다 |
| `num_of_epochs` | `100` | 반복 1회당 학습 에폭 |
| `lr` | `0.0005` | Adam 학습률 |
| `weight_decay` | `0.0` | L2 정규화 계수 (원본에서 손실에 더하는 부분은 주석 처리되어 있음) |

> **반복 횟수 관련 주의.** 합성 데이터로 검증해 보면 1회차 결과가 정답에 가장 가깝고,
> 반복이 늘수록 `D` 가 서서히 커지는 경향이 있다. 각 반복이 "이전 피팅 + 새 NN 잔차" 로 만든
> **가상** 데이터에 다시 피팅하기 때문에 오차가 누적된다.
> 특히 데이터의 슬립각 범위가 좁으면(완만한 주행) 가상 데이터 생성이 `δ`를 0→0.4 rad 까지
> 외삽하므로 이 드리프트가 커진다. 로그에 찍히는 반복별 값을 보고 수렴하는지 확인하고,
> 튀면 `num_of_iterations` 를 줄이거나 더 공격적으로 주행한 데이터를 쓴다.

### 3-2. `params/pacejka_params.yaml` — 초기값과 비교 기준 (원본 그대로)

```yaml
pacejka_model:      # 최소자승 피팅의 시작점. 수렴은 초기값에 크게 의존하지 않는다
  C_Pf_model: [7.1737, 1.5628, 0.3882, 0.5278]
  C_Pr_model: [8.2885, 2.1091, 0.3752, 0.4036]

pacejka_ref:        # 학습에 쓰이지 않고 그림에만 나오는 '비교 기준' 곡선
  C_Pf_ref: [7.17, 1.56, 0.69, 0.53]
  C_Pr_ref: [8.29, 2.11, 0.68, 0.40]
```

배열은 `[B, C, D, E]` 순이다.

### 3-3. `params/sysid_node.yaml` — ROS 인터페이스 (신규)

| 키 | 기본값 | 설명 |
|---|---|---|
| `racecar_version` | `F1TENTH` | `models/<이름>/` 을 찾는 이름 |
| `save_LUT_name` | `F1TENTH_on_track_pacejka` | `generate_lut: true` 일 때만 의미 있음 |
| `output_dir` | `~/.ros/pacejka_sysid` | 결과 모델 / CSV / 그림이 쌓이는 곳 |
| `state_source` | `imu_odom` | `imu_odom`(실차) 또는 `odom`(단일 Odometry, 검증용) |
| `odom_topic` | `/odom` | `v_x` 출처 |
| `imu_topic` | `/imu/data` | `ω`, `a_y` 출처 |
| `ackermann_topic` | `/ackermann_cmd` | 조향각 `δ` 출처 (mux 출력 = 실제 인가된 명령) |
| `imu_rotation_roll` | `180.0` | [deg] IMU 축/부호 보정. 실측 근거는 아래 "좌표계" 항목 참고 |
| `imu_rotation_pitch` / `_yaw` | `0.0` | 〃 |
| `vy_washout_tau` | `2.0` | [s] `v_y` 적분 드리프트 억제 시정수. 작을수록 드리프트에 강하지만 저주파 성분이 깎인다 |
| `imu_to_cog_x` | `0.0` | [m] IMU 장착점의 CoG 대비 세로 오프셋. ⚠ 미실측 |
| `bias_calib_samples` | `200` | 정지 상태에서 `a_y`/`ω` 바이어스를 뽑을 샘플 수 (100 Hz → 2초) |
| `standstill_speed` | `0.05` | [m/s] 이 아래를 '정지'로 판단 |
| `min_collect_speed` | `1.0` | [m/s] 원본의 하드코딩 `v_x > 1` 과 동일 |
| `plot_model` | `true` | 반복마다 결과 그림 표시 |
| `save_plots` | `true` | 그림을 `output_dir` 에 png 로 저장 (headless 대응) |
| `export_csv` | `true` | 수집 데이터를 CSV 로 덤프 |
| `prompt_before_export` | `false` | `true` 면 원본처럼 stdin 으로 `Y` 를 묻는다 |
| `generate_lut` | `false` | 조향 LUT 생성. ForzaETH MAP 컨트롤러 전용이라 기본 off |

#### `v_y` 를 IMU로 얻는다 — 이 이식판만의 특징

원본은 `/car_state/odom` 하나에서 `(v_x, v_y, ω)` 를 전부 받는다. 우리 스택에는 그런 토픽이 없다:

- `vesc_to_odom.cpp:157` — `/odom` 의 `twist.linear.y` 가 **`0.0` 으로 하드코딩**되어 있다 (횡속도 센서 없음)
- `vesc_to_odom.cpp:110` — `/odom` 의 `angular.z` 는 측정값이 아니라 `v·tan(δ)/L` **운동학 재구성**이다
- `ekf_pose_estimator.cpp:295,306` — `/ekf_odom` 은 위의 "항상 0인 `vy`" 를 측정값으로 쓰기 때문에
  추정 `v_y` 를 계속 0 으로 끌어내린다

그래서 IMU에서 유도한다:

```
v̇_y = a_y − v_x·ω − v_y/τ          (바디 프레임 횡방향 운동방정식 + 워시아웃)
```

#### 좌표계 — 이 패키지는 FLU(ISO 8855)를 목표로 한다

원본(무수정) 슬립각 공식이 이미 좌표계를 고정한다:

```python
alpha_f = -arctan((v_y + ω·l_f) / v_x) + δ     # generate_predictions.py, solve_pacejka.py
```

이건 표준 **FLU / ISO 8855** 규약이다 — x=전방, y=좌측, z=상방, **양(+)의 ω = 좌회전**
(위에서 봤을 때 반시계), 양(+)의 δ = 좌측 조향, 양(+)의 v_y = 좌측 이동, 정지 시
`a_z = +9.81`(REP-145: 가속도계는 중력이 아니라 중력에 저항하는 반력을 잰다).

`imu_rotation_roll/pitch/yaw` 는 IMU가 실제로 내보내는 값을 이 목표 좌표계로 맞추는 보정이다.

> **"축이 FLU 방향(전방/좌측/상방)을 향한다"는 것과 "가속도 부호가 REP-145 표준(정지 시
> +9.81)을 따른다"는 것은 별개다.** 드라이버가 축 방향은 FLU 로 내보내면서 가속도 부호는
> 표준과 반대로(반력이 아니라 중력이 당기는 방향 그대로) 낼 수 있다.
>
> 이 워크스페이스의 MicroStrain 센서가 정확히 이 경우다. **실측 결과 정지 시 `a_z ≈ -9.81`**
> 이었다 — 이건 `als_ros/MCL.h:509-514` 의 관찰, 그리고 이미 같은 `/imu/data` 토픽에
> `imu_rotation_roll=180` 을 기본값으로 쓰고 있는 `mcl.launch.py:48` 과 정확히 일치한다.
> 그래서 기본값을 `180°` 로 둔다 — `(x,y,z) → (x,-y,-z)` 로 뒤집어야 `a_z`가 `+9.81`,
> `ω`/`a_y` 부호가 목표 좌표계와 맞게 나온다.
>
> 다른 하드웨어나 장착 상태에서는 다를 수 있으니 기본값을 맹신하지 말고 확인할 것:
>
> 1. 차를 **완전히 정지**시킨 상태에서 직접 확인:
>    ```bash
>    ros2 topic echo /imu/data/linear_acceleration --field z
>    ```
>    **`+9.81` 근처면 `roll=0`**, **`-9.81` 근처면 `roll=180`** (이 워크스페이스는 후자).
> 2. 좌회전(`δ>0`)으로 몰면서 조립된 `ω`가 양수로 나오는지 확인한다 (아래 문제 해결 표 참고).

- 노드 시작 후 차를 **정지시킨 채로** 두면 `a_y`/`ω` 바이어스를 먼저 추정한다.
  `IMU 바이어스 보정 완료` 로그가 뜬 뒤에 주행을 시작할 것.
- 출력 `v_y` 는 **후륜축 기준**이다. 원본 `filter_data` 가 `v_y += l_r·ω` 로 CoG 기준으로 되돌리는
  전제를 그대로 두기 위해 조립기 쪽에서 규약을 맞춘다.

---

## 4. 실행

```bash
# 1) 브링업 (터미널 2개)
imu
sensors

# 2) 식별 노드
source ~/f1tenth_ws/install/setup.bash
ros2 launch pacejka_sysid pacejka_sysid.launch.py
```

런치 인자로 덮어쓸 수 있다:

```bash
ros2 launch pacejka_sysid pacejka_sysid.launch.py \
    racecar_version:=F1TENTH plot_model:=false generate_lut:=false

# 토픽 바꾸기
ros2 launch pacejka_sysid pacejka_sysid.launch.py \
    odom_topic:=/odom imu_topic:=/imu/data ackermann_topic:=/ackermann_cmd
```

진행바(tqdm)를 깔끔하게 보려면 `ros2 run` 이 낫다:

```bash
ros2 run pacejka_sysid pacejka_sysid_node --ros-args \
    --params-file $(ros2 pkg prefix pacejka_sysid)/share/pacejka_sysid/params/sysid_node.yaml
```

### 진행 순서

1. 노드가 뜨면 **차를 세워 둔 채로** 바이어스 보정을 기다린다 (약 2초).
2. `Collecting data` 진행바가 나오면 주행 시작. `v_x > 1 m/s` 인 샘플만 카운트된다.
3. 진행바가 100%가 되면 자동으로 학습이 시작된다 (수 분).
4. `plot_model: true` 면 반복마다 그림 창이 뜬다. **Q 를 눌러 닫아야 다음으로 넘어간다.**
5. 결과가 `output_dir` 에 저장된다.

> **2~3회 돌려서 비교할 것.** 신경망이 매 반복 랜덤 초기화되고 `B·C·D·E` 가 서로 상쇄되므로
> 같은 데이터라도 실행마다 파라미터 값이 꽤 달라진다 (검증에서 `D_f` 가 0.61~0.91 로 흔들렸다).
> 횡력 **곡선**은 6~12% 안에서 일치하니, 그림을 보고 곡선이 재현되는지로 판단하고
> 가장 대표적인 결과를 채택한다. 자세한 내용은 [검증 상태](#검증-상태).

### 수동 주행 시 주의사항

- **안전한 최대 속도로 달릴 것.** 이 방법의 품질은 데이터가 커버하는 **슬립각 범위**에 좌우된다.
  천천히 돌면 타이어가 선형 구간에만 머물러 곡선의 비선형부(`C`, `D`, `E`)를 식별할 수 없다.
- **일관된 속도 구간을 유지할 것.** `simulated_data_gen` 이 수집 데이터의 **평균 속도**(2.75~4 m/s로 클리핑)
  하나로 가상 데이터를 만든다. 속도가 들쭉날쭉하면 대표성이 떨어진다.
- 코너를 계속 물고 돌면서 좌/우를 고루 섞는다. (알고리즘이 데이터를 좌우 대칭으로 증강하긴 하지만
  실제 양방향 데이터가 있는 편이 낫다.)
- 급가감속으로 하중이동을 크게 만들지 않는다 — 모델이 정하중을 가정한다.
- 벽에 부딪히거나 스핀하면 그 구간 데이터가 오염된다. 다시 시작하는 게 낫다.
- 수집 종료 후 로그의 **수집 요약** 줄을 확인한다:
  ```
  수집 요약  v_x[1.02, 4.31] m/s | |v_y|max=0.184 m/s (std 0.061) | |omega|max=2.85 rad/s | |delta|max=0.221 rad
  ```
  `|v_y|max` 가 0 이면 배선이 잘못된 것이고, `|delta|max` 가 0.05 rad 미만이면 너무 얌전히 몬 것이다.

---

## 5. `pacejka_ref` 로 검증하기

`plot_model: true` 로 돌리면 전륜/후륜 각각에 대해 네 가지가 한 그림에 겹쳐 나온다.

| 색 | 의미 |
|---|---|
| 초록 점 | NN 잔차를 더한 모델이 만든 가상 정상상태 데이터 |
| **파랑** | **이번에 식별된 모델** |
| 빨강 | 직전 반복의 사전(prior) 모델 |
| 검정 | `pacejka_params.yaml` 의 `pacejka_ref` — 비교 기준 |

`pacejka_ref` 는 학습에 전혀 쓰이지 않는다. 판단 기준으로 쓴다:

- 기준보다 **그립이 낮은** 노면(먼지, 매끈한 타일)이라면 파랑 곡선의 **최대 횡력이 검정보다 낮게** 나와야 정상이다.
- 기울기(원점 부근 = 코너링 강성)가 검정과 비슷하면 `B·C·D` 조합이 합리적이라는 뜻이다.
- 파랑이 검정보다 훨씬 위에 있거나 모양이 이상하면(피크가 사라지거나 단조증가) 데이터의 슬립각 범위가
  부족한 것이다. 더 빠르게 다시 수집한다.
- 초록 점이 파랑 곡선에서 크게 벗어나면 피팅이 실패한 것이다.

기존 값과 직접 비교하려면 `pacejka_ref` 를 **현재 SMPPI 설정값**으로 바꿔 놓고 돌리면 된다.
현재 `smppi config/params.yaml` 값은 `[1.5, 1.5, 2.0843, 0.0]` (전), `[1.5, 1.5, 1.9233, 0.0]` (후) 이다.

---

## 6. 식별 결과를 SMPPI CUDA 모델에 반영하기

### 6-1. 이제는 그대로 붙여넣으면 된다

원래 우리 CUDA 모델은 3-파라미터에 `D` 가 **뉴턴 단위**였고, ForzaETH 는 4-파라미터에 `D` 가
**무차원(정하중 `F_z` 를 곱함)** 이라 서로 맞지 않았다.
이번에 **CUDA 모델을 ForzaETH 규약에 맞춰 바꿔** 두었다:

```cpp
// mppi_core.cu
F_fy = F_zf * D_f * sin( C_f * atan( B_f*α_f − E_f*(B_f*α_f − atan(B_f*α_f)) ) )
```

`F_zf = m·g·l_r/l_wb`, `F_zr = m·g·l_f/l_wb` 는 `smppi_node.cpp` 가 자동 계산한다
(현재 제원 기준 `F_zf = 18.2315 N`, `F_zr = 18.4579 N`).

따라서 식별 결과를 **1:1로** 옮기면 된다:

```
C_Pf = [B, C, D, E]  →  B_f, C_f, D_f, E_f
C_Pr = [B, C, D, E]  →  B_r, C_r, D_r, E_r
```

식별 결과 파일:

```bash
cat ~/.ros/pacejka_sysid/models/F1TENTH/F1TENTH_pacejka.txt
```

`src/control/smppi_cuda_controller/config/params.yaml` 을 수정한 뒤 재빌드:

```bash
colcon build --packages-select smppi_cuda_controller
```

### 6-2. ⚠ 거동이 크게 바뀐다 — 단계적으로 적용할 것

현재 설정값은 `D_f = 2.0843` 인데, 이건 **마찰계수 μ ≈ 2.08** 이라는 뜻으로 물리적으로 불가능하다
(실제 고무 타이어는 0.6~1.0). 즉 지금 컨트롤러는 **실제보다 3~4배 큰 횡력을 낼 수 있다고 믿고** 있다.

식별된 `D` 는 0.5~0.9 정도로 나올 것이고, 이를 적용하면 컨트롤러가 "그립이 훨씬 적다"고 인식해
**훨씬 보수적으로(느리고 완만하게) 조향**하게 된다. 이건 모델이 정확해진 결과이지 고장이 아니다.

권장 절차:

1. 먼저 `params.yaml` 의 `target_speed` / `max_speed` 를 절반으로 낮춘다.
2. 넓고 장애물 없는 곳에서 식별값을 적용해 주행해 본다.
3. `/mppi_optimal_trajectory` 와 실제 궤적이 얼마나 맞는지 본다. 모델이 맞으면 예측이 실제와 잘 겹친다.
4. 문제없으면 속도를 원래대로 단계적으로 올린다.
5. 원복이 필요하면 아래 값으로 되돌린다 (변경 전 거동과 **수치적으로 동일**, 오차 0.0004%):
   ```yaml
   B_f: 1.5   C_f: 1.5   D_f: 2.0843   E_f: 0.0
   B_r: 1.5   C_r: 1.5   D_r: 1.9233   E_r: 0.0
   ```

> **시뮬레이터 주의**: `racecar_simulator` 의 플랜트(`simulator.cpp:742-773`)는 손대지 않았다.
> 여전히 3-파라미터에 `D` 가 뉴턴 단위(`simulation.yaml`: `D_f0: 40.0`)다.
> 시뮬레이터에서 검증할 때 플랜트와 컨트롤러의 규약이 다르다는 점을 감안할 것.

---

## 7. 반복 재학습

새 모델로 더 빨리 달릴 수 있게 되면 **다시 수집하는 편이 낫다.** 더 높은 슬립각까지 커버한
데이터로 학습하면 모델이 더 정확해지고, 그러면 또 더 빨리 달릴 수 있다.

1. 새 파라미터로 SMPPI 를 돌려 이전보다 빠르게 주행이 되는지 확인한다.
2. `models/F1TENTH/F1TENTH_pacejka.txt` 의 `I_z`, `m`, `l_f`, `l_r` 는 그대로 두고
   (`C_Pf`/`C_Pr` 은 덮어써져도 무방하다 — 피팅 초기값이 아니다),
3. 더 빠른 속도로 다시 수집·학습한다.
4. 반복별 결과가 이전 회차와 비슷하게 나오면 수렴한 것이다.

노면이 바뀌면(다른 체육관, 먼지, 온도) **반드시 다시 식별해야 한다.** 그게 이 방법의 요점이다.

### 재학습 시 초기화

`output_dir` 의 모델 파일은 매번 덮어써진다. 완전히 처음부터 하려면:

```bash
rm -rf ~/.ros/pacejka_sysid/models/F1TENTH
```

다음 실행 때 share 의 템플릿이 다시 복사된다.

---

## 8. 문제 해결

| 증상 | 원인 / 조치 |
|---|---|
| `파라미터 파일이 없습니다: .../params/nn_params.yaml` | `--symlink-install` 로 빌드했다. 빼고 다시 빌드 (0장 참고) |
| `차량 파라미터 파일이 없습니다` | `racecar_version` 오타 또는 `models/<이름>/` 미생성. 이름에 `_` 금지 |
| `ModuleNotFoundError: No module named 'tqdm'` | `sudo apt install python3-tqdm` |
| 진행바가 전혀 안 움직임 | `v_x > 1 m/s` 가 안 나오는 것. `ros2 topic echo /odom` 로 확인. 데드맨 버튼 확인 |
| `IMU 바이어스 보정 완료` 로그가 안 뜸 | 차가 움직이고 있다(`v_x ≥ 0.05`). 완전히 세워 둘 것. `/imu/data` 가 나오는지도 확인 |
| 수집 요약에 `|v_y|max=0.000` | `state_source`/`imu_topic` 배선 문제. `/imu/data` 확인 |
| `ω` 부호가 반대인 것 같음 | `imu_rotation_roll` 을 `180.0` ↔ `0.0` 로 바꿔 본다. 좌회전(`δ>0`)에서 `ω>0` 이어야 정상 |
| 정지 상태 `a_z` 가 `+9.81` 근처인데 `roll=180` 을 씀 | 이 하드웨어에서는 `roll=0` 이 맞다. `linear_acceleration.z` 를 직접 재서 부호에 맞게 설정 |
| 그림 창이 안 뜸 (SSH) | 정상. `save_plots: true` 로 `output_dir` 에 png 가 저장된다 |
| 식별값이 반복마다 크게 튐 | 슬립각 커버리지 부족. 더 빠르게 재수집하거나 `num_of_iterations` 를 줄인다 |
| 돌릴 때마다 `D` 가 다르게 나옴 | **정상이다.** 신경망이 매번 랜덤 초기화되고 `B·C·D·E` 가 서로 상쇄된다. 곡선(그림)으로 판단하고 2~3회 돌려 비교할 것 ([검증 상태](#검증-상태) 참고) |
| 진행바만 계속 있고 아무 일도 안 일어남 | `IMU 바이어스 보정 대기 중` 경고가 5초마다 뜨는지 확인. 차를 완전히 세워야 보정이 끝난다 |

### 실차 없이 검증하기

`state_source: odom` 으로 두면 원본과 동일하게 단일 Odometry 에서 `(v_x, v_y, ω)` 를 받는다.
`racecar_simulator` 의 `/odom0` 은 Pacejka 플랜트에서 나온 **진짜** `v_y`/`ω` 를 발행하므로
포팅 정확성을 확인하기 좋다:

```bash
ros2 launch pacejka_sysid pacejka_sysid.launch.py \
    state_source:=odom odom_topic:=/odom0 ackermann_topic:=/ackermann_cmd0
```

기록된 bag 으로 상태 조립기만 점검할 수도 있다
(`/odom`, `/imu/data`, `/ackermann_cmd` 가 모두 들어 있다):

```bash
ros2 bag play ~/f1tenth_bag/0728/rosbag2_2026_07_28-01_10_07
```

---

## 원본 대비 변경점

### 파일별

| 파일 | 상태 |
|---|---|
| `helpers/pacejka_formula.py`, `generate_predictions.py`, `generate_inputs_errors.py`, `solve_pacejka.py`, `SimpleNN.py`, `dotdict.py`, `vehicle_dynamics_stown.py` | **바이트 단위로 원본과 동일** (출처 헤더 주석만 추가) |
| `helpers/plot_results.py` | 임포트 1줄만 상대 임포트로 변경 |
| `helpers/load_model.py`, `save_model.py` | 경로 해석 `rospkg` → `paths.py`, 로깅 shim |
| `helpers/train_model.py` | 경로/로깅 + `generate_lut`·`save_plots` 인자 추가. **학습 알고리즘 무수정** |
| `helpers/simulate_model.py` | 경로/로깅만 |
| `params/nn_params.yaml`, `pacejka_params.yaml` | **원본 그대로** |
| `pacejka_sysid_node.py` | 원본 `on_track_sys_id.py` 의 rclpy 재작성 |
| `state_assembler.py`, `paths.py`, `ros_log.py` | 신규 (원본에 대응물 없음) |

### 알고리즘에 손댄 부분

**없다.** 필터링(Butterworth + 5샘플 조향 시프트), 좌우 대칭 증강, NN 잔차 학습,
가상 정상상태 데이터 생성, 최소자승 Pacejka 피팅, 반복 갱신 — 전부 원본 그대로다.

`train_model.py` 에서 원본과 달라진 실행 흐름은 다음 세 가지뿐이며 모두 알고리즘 바깥이다:

1. `generate_lut=False` 일 때 마지막 `LookupGenerator(...).run_generator()` 호출을 건너뛴다
   (LUT 는 ForzaETH MAP 컨트롤러 전용, SMPPI 는 쓰지 않음). 코드 자체는 그대로 있어 `true` 로 켜면 동작한다.
2. `save_plots=True` 일 때 `plot_results()` 직후 `plt.savefig()` 로 그림을 저장하고 `plt.close('all')` 한다.
3. 식별된 `C_Pf`, `C_Pr` 을 반환한다 (원본은 반환값 없음). 노드가 로그로 찍기 위함.

### 검증 상태

**ROS 레벨 통합 테스트 (가짜 차량 노드로 `/odom`·`/imu/data`·`/ackermann_cmd` 발행)**

알려진 Pacejka 계수 `C_Pf=[7.5, 1.5, 0.65, 0.4]`, `C_Pr=[8.0, 1.9, 0.60, 0.3]` 로
단일트랙 모델을 돌려 토픽을 발행하고, `state_source: imu_odom` 경로(= 실차와 동일하게
`v_y` 를 IMU 에서 유도)로 3회 식별했다. 각 실행 약 50초 (수집 30초 + 학습).

가짜 차량은 실제 스택의 특성을 그대로 재현한다 — `/odom` 의 `twist.linear.y = 0.0` 하드코딩,
`angular.z` 의 운동학 재구성, IMU z축 아래 방향 장착.

| 실행 | 전륜 곡선 최대오차 | 후륜 곡선 최대오차 | `D_f` | `D_r` |
|---|---|---|---|---|
| 1 | 0.71 N (6.8%) | 0.86 N (7.8%) | 0.609 | 0.592 |
| 2 | 0.69 N (6.5%) | 0.77 N (7.0%) | 0.854 | 0.742 |
| 3 | 1.28 N (12.2%) | 1.16 N (10.5%) | 0.909 | 0.798 |
| **정답** | — | — | **0.65** | **0.60** |

(데이터가 실제 커버한 `|α| ≤ 0.13 rad` 구간에서 평가. 정답 피크 전륜 10.53 N, 후륜 11.01 N)

**해석 — 반드시 알아 둘 것:**

- **횡력 곡선은 6.5~12.2% 오차로 잘 복원된다.** 이게 컨트롤러가 실제로 쓰는 값이다.
- 그러나 **개별 파라미터는 실행마다 상당히 흔들린다** (`D_f` 가 0.61~0.91). `B`·`C`·`D`·`E` 가
  서로 상쇄되며 여러 조합이 비슷한 곡선을 만들기 때문이다(파라미터 축퇴).
  원본 알고리즘이 매 반복마다 신경망을 **랜덤 초기화**하고 시드를 고정하지 않는 것도 원인이다.
- 따라서 **`D` 값 하나를 놓고 일희일비하지 말 것.** 판단은 곡선(그림)으로 하고,
  **2~3회 돌려서 곡선이 비슷하게 나오는지** 확인한 뒤 채택하는 것을 권장한다.

**그 외**

- CUDA 모델 변경은 `E=0` 마이그레이션 값에서 변경 전과 **최대오차 0.0004% (float32 기준)** 로 동일함을 확인.
- 이식된 헬퍼 7개가 원본과 바이트 단위로 동일함을 `diff` 로 확인.
- **실차 검증은 아직 하지 않았다.** 아래 가정들이 남아 있다.

### 아직 검증되지 않은 가정

1. **`I_z`** — 워크스페이스 안에서 `0.16` vs `0.04712` 로 3.4배 불일치. 식별 결과에 직결된다 (2장 참고).
2. **`h_cg`** — 실측값 없음. `generate_lut: false` 면 사용되지 않는다.
3. **`imu_rotation_roll = 180°` 가 `a_y` 에도 옳은지** — 정지 시 `a_z ≈ -9.81` 실측으로
   자이로 쪽은 확인됐다(als_ros MCL의 동일 토픽 관찰과도 일치). 가속도계 `a_y`도 같은
   180° 보정으로 옳게 나오는지는 좌회전에서 `ω > 0`, 직진 가속 시 부호가 맞는지로
   별도 확인할 것 (8장 문제 해결 표).
4. **`imu_to_cog_x`** — IMU 장착점의 CoG 대비 세로 오프셋. 기본 `0.0` 으로 두었으나 실제로는 0이 아닐 수 있다.
5. **`vy_washout_tau = 2.0 s`** — 드리프트 억제와 저주파 보존의 절충. 실차 데이터로 조정이 필요할 수 있다.
6. **`/ackermann_cmd` 를 `δ` 로 사용** — 실제 서보 클리핑(`servo_min/max` 0.01/0.99)은
   `/sensors/servo_position_command` 에만 반영된다. 조향이 한계에 걸리는 주행에서는 차이가 날 수 있다.
7. **`racecar_version` 이름을 `F1TENTH` 로 둠** — 차량 명명 규칙이 따로 있으면 바꿀 것.

---

## 인용

```bibtex
@misc{dikici2024learningbasedontrackidentificationscaled,
      title={Learning-Based On-Track System Identification for Scaled Autonomous Racing in Under a Minute},
      author={Onur Dikici and Edoardo Ghignone and Cheng Hu and Nicolas Baumann and Lei Xie and
              Andrea Carron and Michele Magno and Matteo Corno},
      year={2024},
      eprint={2411.17508},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2411.17508},
}
```
