# Humanoid Controller 학습 랜덤화 정리

현재 `humanoid_controller`의 기본 학습 설정에서 적용되는 도메인 랜덤화와 입력·학습 과정의 랜덤성을 정리한다.

- 확인 기준: 2026-09-07의 로컬 코드. CLI 인자나 저장된 설정으로 덮어쓰기 전 기본값이다.
- 확인 방법: 설정과 실제 호출 경로의 정적 분석. 학습 실행 검증은 포함하지 않는다.
- 표기: `U(a, b)`는 균등분포이며, 관측 노이즈의 ± 값은 표준편차가 아닌 최대 절댓값이다.
- 관련 문서: [LIPM 및 RL 데이터 흐름](LIPM_RL_DATA_FLOW.md)

## 1. 물리 파라미터와 외란

| 항목 | 현재 적용 방식 | 적용 시점 |
|---|---|---|
| 로봇 충돌 형상의 마찰계수 | `[0.5, 1.25]`에서 64개 값을 샘플링하고 환경마다 하나를 무작위 배정. 한 로봇의 모든 충돌 형상에 같은 값 적용 | 환경 생성 시. 에피소드 리셋 시 재샘플링하지 않음 |
| 몸통 질량 | 기본 몸통 질량에 `U(-1, 1)` kg 추가 | 환경 생성 시, 환경별 고정 |
| 수평 속도 외란 | 월드 좌표 몸통 `vx`, `vy`를 각각 `U(-0.5, 0.5)` m/s로 설정 | 전역 시뮬레이션 시간 기준 2.5초마다 |

외란은 현재 속도에 랜덤 증분을 더하는 방식이 아니라, 샘플링한 속도로 덮어쓰는 방식이다. 따라서 실제 속도 변화량은 0.5 m/s보다 클 수 있다. 모든 환경에 같은 시점에 적용되지만 값은 환경마다 다르다.

마찰 랜덤화는 로봇 충돌 형상의 속성을 바꾼다. 지면 자체의 마찰 설정을 환경별로 바꾸는 구현은 아니다.

현재 URDF의 기본 몸통 질량은 8.52 kg이므로 ±1 kg은 몸통 기준 약 ±11.7%다. 몸통 질량 변경 후 `recomputeInertia=True`로 시뮬레이터 속성을 갱신하지만, 관성 텐서를 별도 분포에서 샘플링하는 구현은 없다.

코드:

- [랜덤화 설정](../gym/envs/humanoid/humanoid_controller_config.py): `HumanoidControllerCfg.domain_rand`
- [물리 속성 및 외란 적용](../gym/envs/base/legged_robot.py): `_process_rigid_shape_props()`, `_process_rigid_body_props()`, `_push_robots()`

## 2. 목표 명령과 보행 파라미터

| 항목 | 샘플링 범위 또는 계산 | 실제 동작 |
|---|---|---|
| 목표 월드 `vx` | `U(-2, 2)` m/s | 랜덤 |
| 목표 월드 `vy` | `U(-2, 2)` m/s | 랜덤 |
| 목표 yaw rate | 0 rad/s | 고정 |
| 한 스텝 시간 | `torch.randint(35, 36)` × 0.01초 | 상한 제외이므로 항상 0.35초 |
| 목표 좌우 보폭 | `U(0.3, 0.3)` m | 항상 0.3 m |
| 다음 착지 위치 | 현재 상태와 명령으로 LIPM 계산 | 별도 랜덤 오프셋 없음 |

수평 속도 명령의 크기가 0.2 m/s 이하이면 `vx`, `vy`를 모두 0으로 만든다. 설정 파일에 `lin_vel_x`, `lin_vel_y`가 중복 정의되어 있지만, 나중에 정의된 위 값이 유효하다.

명령은 에피소드 리셋 시 재샘플링한다. 주기적 재샘플링 설정은 10초지만 에피소드 최대 길이가 5초이므로, 정상적인 에피소드에서는 주로 리셋 시 변경된다.

활성 LIPM 함수의 변수명이 `random_step_command`여도 함수 내부에서 착지점을 무작위로 뽑지는 않는다. 랜덤하게 주어진 명령과 변화한 로봇 상태에 따라 계산 결과가 달라진다.

코드:

- [명령 설정](../gym/envs/humanoid/humanoid_controller_config.py): `HumanoidControllerCfg.commands`
- [속도 명령 샘플링](../gym/envs/base/legged_robot.py): `_resample_commands()`
- [보행 파라미터 및 발 목표 계산](../gym/envs/humanoid/humanoid_controller.py): `_resample_commands()`, `_generate_step_command_by_3DLIPM_XCoM()`

## 3. Actor 관측 노이즈

각 관측 성분에 매번 독립적인 균등 노이즈를 더한다.

```text
actor_observation[i] = observation[i] + U(-amplitude[i], amplitude[i])
```

아래 값은 현재 스케일 설정 기준이며, running mean/std 정규화 전에 적용된다.

| 관측 항목 | 노이즈 범위 |
|---|---|
| 몸통 높이 | ±0.05 m |
| 몸통 월드 선속도, 각 축 | ±0.05 m/s |
| 몸통 heading | ±0.01 rad |
| 몸통 좌표 각속도, 각 축 | ±0.05 rad/s |
| Projected gravity, 각 성분 | ±0.05 |
| 양발 실제 위치, 각 축 | ±0.01 m |
| 양발 실제 heading | ±0.01 rad |
| 양발 목표 위치, 각 축 | ±0.05 m |
| 양발 목표 heading | ±0.05 rad |
| 속도 명령 `vx`, `vy` | ±0.1 m/s |
| yaw rate 명령 | ±0.1 rad/s |
| 관절각 10개 | ±0.05 rad |
| 관절속도 10개 | ±0.5 rad/s |
| 위상 `sin`, `cos` | 없음 |

### 적용 범위

- Actor가 받는 관측에 추가한다. 물리 상태 자체를 변경하지 않는다.
- Critic은 같은 종류의 관측을 받지만 이 추가 노이즈는 받지 않는다.
- 플래너와 보상 계산에도 이 Actor용 노이즈가 직접 적용되지는 않는다.
- 실제 yaw rate 명령은 0이어도 Actor가 보는 값에는 ±0.1 rad/s 노이즈가 들어간다.
- 양발 목표 위치의 노이즈는 base 좌표로 변환된 관측에 적용된다. 플래너가 저장한 월드 좌표 착지 목표를 바꾸는 것은 아니다.
- `foot_contact = 0.1`과 `base_lin_vel = 0.05`도 노이즈 설정에 있지만, 현재 Actor 관측 목록에 해당 항목이 없어 적용되지 않는다. 선속도 관측은 `base_lin_vel_world`를 사용한다.

코드:

- [Actor 관측 목록 및 노이즈 크기](../gym/envs/humanoid/humanoid_controller_config.py): `HumanoidControllerRunnerCfg.policy`
- [관측 노이즈 적용 및 Actor/Critic 입력 분리](../learning/runners/on_policy_runner.py): `get_noisy_obs()`, `get_obs_noise_vec()`, `learn()`

## 4. 초기화와 학습 자체의 랜덤성

| 항목 | 현재 동작 |
|---|---|
| 초기 관절각·관절속도 | 기본 관절각, 속도 0으로 고정 |
| 초기 몸통 자세·속도 | 높이 0.62 m, 기본 자세, 선속도·각속도 0 |
| 초기 스윙발·보행 위상 | 왼발 스윙, 위상 0으로 고정 |
| 에피소드 시간 카운터 | 학습 시작 시 환경마다 랜덤하게 설정해 첫 timeout 시점을 분산 |
| PPO 액션 | 매 policy step에 관절별 Gaussian 분포에서 샘플링 |
| 액션 표준편차 | 관절별 초기값 1.0, 이후 학습되는 파라미터 |
| 학습 seed | 기본 `seed=-1`: seed를 0~9999 중 무작위 선택. CLI로 지정 가능 |
| 기타 | 신경망 초기 가중치, PPO 미니배치 순서도 랜덤 |

### 초기 상태

초기 상태 범위가 설정 파일에 존재하지만 현재 `reset_mode='reset_to_basic'`이므로 사용되지 않는다. `reset_to_range`를 활성화해야 관절각·관절속도·몸통 자세·속도의 범위 샘플링이 적용된다.

로봇 생성 시 위치에 ±1 m를 주는 코드도 있지만, 학습 리셋에서는 환경 원점에 기본 상태를 더한 위치로 설정한다. 이를 매 에피소드 초기 위치 랜덤화로 해석하면 안 된다.

학습 시작 시 랜덤화하는 `episode_length_buf`는 종료 시간 카운터다. 보행 위상이나 물리 상태를 그 시간만큼 진행시키는 것은 아니다.

### 액션 탐색

```text
action ~ Normal(actor_mean(observation), learned_std)
action = clip(action, -10, 10)
q_des = default_q + action
```

현재 스케일에서 액션은 관절 목표각 오프셋이므로 초기 표준편차 1.0은 약 1 rad에 해당한다. 이는 별도의 모터 토크 노이즈가 아니라 PPO 탐색을 위한 액션 샘플링이다. 추론용 함수는 샘플 대신 평균 액션을 사용한다.

코드:

- [상태 리셋](../gym/envs/base/legged_robot.py): `reset_to_basic()`, `reset_to_range()`
- [보행 상태 리셋](../gym/envs/humanoid/humanoid_controller.py): `_reset_system()`
- [학습 시작 및 액션 클리핑](../learning/runners/on_policy_runner.py): `learn()`, `set_actions()`
- [Gaussian 액션](../learning/modules/actor.py): `update_distribution()`, `act()`, `act_inference()`
- [seed 설정](../gym/utils/helpers.py): `set_seed()`

## 5. 현재 비활성 또는 미구현인 랜덤화

| 항목 | 현재 상태 |
|---|---|
| 발·다리 등 링크별 질량 랜덤화 | 없음. 몸통 질량만 변경 |
| 각 링크의 로컬 CoM 위치 랜덤화 | 없음 |
| 관성 텐서의 독립적인 랜덤 샘플링 | 없음 |
| PD gain 랜덤화 | 없음 |
| 모터 강도·토크 배율 랜덤화 | 없음 |
| 제어 지연·통신 지연 랜덤화 | 없음 |
| Rotor inertia·angular damping 랜덤화 | 고정 설정. 추가 예정 주석만 존재 |
| 지형 랜덤화 | 현재 `mesh_type='plane'`. 다른 지형 관련 설정은 현재 경로에서 사용하지 않음 |
| 초기 자세·속도 랜덤화 | 범위와 구현은 있으나 현재 reset 모드에서 비활성 |
| 스텝 시간·좌우 보폭 랜덤화 | 샘플링 구현은 있으나 현재 범위가 단일 값 |

## 6. 발 질량을 고려한 모델로 확장할 때의 해석

현재는 질량 랜덤화 대상이 몸통에 한정되어 있다. 몸통 질량 변화만으로도 전체 CoM은 달라질 수 있지만, 발 질량이나 발 내부 CoM 위치가 바뀌면서 생기는 스윙 동역학의 변화를 직접 다루지는 않는다.

또한 `_process_rigid_body_props()`는 랜덤화 전 질량을 `rigid_body_mass`와 `mass_total`에 저장한 뒤 시뮬레이션 몸통 질량을 바꾼다. `_calculate_CoM()`은 이 저장된 질량을 가중치로 사용한다. 따라서 플래너의 CoM 계산에 사용하는 질량과 실제 시뮬레이션 질량이 일치하지 않는다.

후속 실험에서는 다음을 구분하면 결과를 해석하기 쉽다.

1. 실제 로봇의 발 질량과 질량 분포를 반영한 nominal 모델 구성.
2. 그 모델의 질량·CoM 추정 오차에 대한 랜덤화.
3. 샘플링한 실제 질량을 플래너에도 제공할지, nominal 질량을 유지해 모델 오차를 줄지 결정.

링크별 질량·CoM 랜덤화와 비교하면 현재 설정은 변화시키는 파라미터의 범위가 제한적이다. 다만 랜덤화 강도 자체는 각 방법의 수치 범위와 샘플링 방식까지 확인해야 비교할 수 있다.
