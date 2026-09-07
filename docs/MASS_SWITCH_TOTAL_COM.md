# 전체 CoM 초기 높이를 맞춘 질량 스위칭 LIPM

몸통 기준 mass-switch 데모를 보존하면서 전체 로봇 CoM의 초기 높이를 기존 LIPM과 맞추는 별도 모델이다.

- 모델: LIPM/models/LIPM_3D_mass_switch_total_com.py
- 데모: LIPM/demos/demo_LIPM_3D_mass_switch_total_com.py
- 기반 동역학: LIPM/models/LIPM_3D_mass_switch.py

## 실행

```powershell
.\.venv\Scripts\python.exe LIPM/demos/demo_LIPM_3D_mass_switch_total_com.py
.\.venv\Scripts\python.exe LIPM/demos/demo_LIPM_3D_mass_switch_total_com.py --periodic-start
.\.venv\Scripts\python.exe LIPM/demos/demo_LIPM_3D_mass_switch_total_com.py --periodic-start --headless --output-dir outputs/mass_switch_total_com
```

기본 질량은 몸통 40 kg, 발 각각 10 kg이다. --height 0.6은 몸통 높이가 아니라 전체 CoM의 초기 높이다. --body-mass, --foot-mass, --ss, --ds, --vx, --vy를 지원한다. 기본은 최종 그래프이며 --animate로 재생한다.

## 높이와 초기 상태

초기 전체 CoM c_total과 발 위치에서 몸통 위치를 역산한다.

body = ((mb + 2mf) * c_total - mf * (left + right)) / mb.

양발 점질량 높이가 0일 때 전체 CoM 높이 0.6 m를 만들려면 몸통 높이가 0.9 m여야 한다. 몸통 높이는 이후 고정한다. 40/10 kg의 유효 계획 높이는 40/50 * 0.9 = 0.72 m다.

전체 CoM 높이는 초기값만 맞추고 매 순간 고정하지 않는다. 스윙 중에는 전체 CoM 높이 = 0.6 + (10/60) * (왼발 높이 + 오른발 높이)다. 실제 유효 CoM 높이도 발을 따라 변하며, LIPM 계획에는 구간별 고정 높이를 사용한다.

일반 시작은 전체 CoM (0,0,0.6), 초기 전체 CoM 속도는 명령값(기본 0.3,0)이다. 양발은 (0,+0.2,0)/(0,-0.2,0)에서 정지하므로 몸통 초기 전진 속도는 0.45 m/s가 된다.

--periodic-start는 기반 모델의 두 스텝 고정점을 사용한다. 수평 CoM 위치·속도와 양발 배치는 주기조건에 맞게 바뀌므로 일반 초기조건과 같지 않다. 양발이 지면에 있어 초기 전체 CoM 높이 0.6 m는 유지한다. 정지 출발 기능은 아니다.

## 네 점과 궤적

| 표시 | 포함 질량 | 용도 |
|---|---:|---|
| 빨간 원 | 전체 60 kg | 궤적·속도 그래프 기준 |
| 검정 사각형 | 몸통 40 kg | 고정 높이 기준, 다리 선의 시작점 |
| 청록 마름모 | 몸통과 왼발 50 kg | 오른발 지지 시 유효 CoM |
| 황토 마름모 | 몸통과 오른발 50 kg | 왼발 지지 시 유효 CoM |

현재 선택된 유효 CoM을 크게 표시한다. 전체 CoM만 궤적을 그리고, 3D에서는 기존 방식의 지면 투영 궤적을 표시한다. 위에서 본 그림에서 점들이 겹치는 것은 수평 위치가 같을 때 정상이다.

NPZ의 position/velocity는 전체 CoM의 수평 상태다. whole_com/whole_com_velocity는 전체 3D 상태, body_position/body_velocity는 몸통 3D 상태다. effective_com, active_foot, planning_com, planning_height 및 기존 로그를 보존한다.

## 비교와 검증

기존 DSP LIPM 높이 0.6 m와 전체 CoM 초기 높이를 맞춘 비교다. 기존 몸통 기준 mass-switch의 --height 0.6과는 의미가 다르다. 기존 모델에 몸통 높이 0.9 m와 동일한 물리 초기 상태를 주면 이 모델과 동역학은 같다.

기반 모델의 목표 DCM 오프셋은 아직 기존 DSP 식이므로 속도·폭 오차가 남는다. 현재 기본 주기 평균 속도는 명령 0.3 m/s에 대해 약 0.24047 m/s다. 초기 높이를 맞춘 것만으로 완전히 같은 동역학 조건이나 물리적 안정성이 보장되지는 않는다.

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s LIPM/tests -p test_mass_switch_total_com.py
```

초기 높이·속도, 스윙 시 전체 CoM 높이 변화, 기존 몸통 높이 0.9 m 모델과의 일치, DSP 유무의 두 스텝 반복 및 질량 0 기준 일치를 검증한다.

실제 제어에 사용하기 전 추정 상태가 전체 CoM인지 몸통 CoM인지 루트 위치인지 확인해야 한다. 현재 구현은 실험용 축약 모델이며 로봇 정책의 상태 정의를 자동으로 바꾸지 않는다.
