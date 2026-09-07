# 몸통 기준 질량 스위칭 LIPM

양발을 제외한 로봇을 몸통 질량점으로 합치고, 현재 스윙발을 포함한 유효 CoM으로 보행을 계획하는 실험 모델이다. 일반 LIPM과 더블 스탠스 모델은 별도로 유지한다.

## 파일과 실행

- 모델: LIPM/models/LIPM_3D_mass_switch.py
- 데모: LIPM/demos/demo_LIPM_3D_mass_switch.py
- 초기 전체 CoM 높이를 맞추는 별도 모델: MASS_SWITCH_TOTAL_COM.md 참고

```powershell
.\.venv\Scripts\python.exe LIPM/demos/demo_LIPM_3D_mass_switch.py
.\.venv\Scripts\python.exe LIPM/demos/demo_LIPM_3D_mass_switch.py --periodic-start
.\.venv\Scripts\python.exe LIPM/demos/demo_LIPM_3D_mass_switch.py --body-mass 40 --foot-mass 10
.\.venv\Scripts\python.exe LIPM/demos/demo_LIPM_3D_mass_switch.py --periodic-start --headless --output-dir outputs/mass_switch
```

기본은 최종 그래프이며 --animate를 붙이면 재생한다. 기본 질량은 몸통 40 kg, 발 각각 10 kg이다. 모델 모듈의 DEFAULT_BODY_MASS / DEFAULT_FOOT_MASS를 클래스·헬퍼·CLI가 공통으로 사용한다. 제목과 콘솔은 실제 적용된 모델 질량을 표시한다. 질량은 실험용 값이며 실제 로봇 식별 결과를 의미하지 않는다.

기본 SSP/DSP는 0.48/0.12초, dt 0.02초, 목표 속도 0.3 m/s, 실행 시간 10초다. --height 0.6은 몸통 질량점 높이다. 발너비 목표는 0.4 → 0.6 → 0.4 m로 10/20번째 스텝에 변경한다. 첫 목표 보폭부터 명령 속력 * (SSP+DSP)로 계산한다.

## 질량점과 상태

질량 비율 a = mf / (mb + mf)를 사용한다.

- 유효 CoM = (1-a) * 몸통 위치 + a * 포함한 발 위치
- 유효 CoM 속도 = (1-a) * 몸통 속도 + a * 포함한 발 속도
- 전체 CoM = (mb * 몸통 위치 + mf * 양발 위치 합) / (mb + 2mf)

SSP에서는 스윙발을 포함하고 지지발을 제외한다. 유효 질량점은 몸통과 스윙발을 잇는 선분 위에 있다. 몸통 높이는 고정한다. 스윙 궤적은 데모에서 계산하며 양 끝 속도가 0인 quintic 수평 궤적과 매끄러운 발 높이 궤적을 사용한다.

유효 CoM 수평 상태를 LIPM으로 전파한 뒤, 갱신된 스윙발 위치·속도를 이용해 몸통 상태를 역산한다. 따라서 수평 유효 CoM과 표시되는 질량점 정의가 일치한다.

## 착지와 고정 높이

착지 순간 몸통 위치·속도를 보존하고, 다음에 들 발을 포함하는 유효 CoM으로 상태를 재설정한다. 이때 설정한 계획 높이를 DSP와 다음 SSP 동안 유지한다. DSP의 ZMP는 기존 방식대로 이전 발에서 새 발로 선형 이동한다. DSP 시간 0도 지원한다.

40/10 kg, 몸통 높이 0.6 m에서는 계획 높이가 0.48 m다. 실제 유효 CoM은 발을 들면 높이가 변하지만 계획 높이는 고정한다. 양발 질량과 착지 높이가 같으면 좌우 계획 높이는 같다.

## 착지점 계산과 periodic-start

착지점을 계산할 때 SSP 전파, 질량점 재설정, DSP 전파를 포함한 말단 DCM 조건을 2×2 선형계로 푼다. 현재 목표 DCM 오프셋은 기존 DSP 모델의 식을 사용한다. 질량 스위칭에 맞춘 목표 오프셋 보정은 아직 적용하지 않은 기준 버전이다.

이 때문에 유한 발 질량에서는 명령 보폭·발너비와 실제 주기값이 다를 수 있다. 40/10 kg, 몸통 높이 0.6 m, 속도 명령 0.3 m/s의 주기 평균 속도는 약 0.23826 m/s다. 초기 과도응답과 별개의 정상 보행 오차다.

--periodic-start는 두 번의 SSP·착지 재설정·DSP·지지발 교체를 거친 상대 상태 사상의 고정점을 계산한다. 유효 CoM 위치/속도와 반대 발 위치의 수평 6개 상태에 대한 아핀 사상을 구성하고 선형계를 푼다. 두 스텝 후 평행 이동을 제외하면 같은 상태가 된다. 정지 출발 기능은 아니며 목표 속도 오차를 자동 보정하지 않는다.

## 표시와 데이터

이 버전의 빨간 점, 초록 궤적, 속도 그래프는 몸통 CoM(Body CoM) 기준이다. 청록·황토색 마름모는 각각 왼발·오른발 포함 유효 CoM이다. 현재 사용하는 점을 크게 표시한다. 유효 CoM 궤적은 그리지 않는다.

trajectory.npz의 position/velocity는 몸통의 수평 상태다. body_position/body_velocity는 몸통 3D 상태, whole_com/whole_com_velocity는 전체 로봇 3D 상태다. effective_com은 왼발/오른발 순 후보점이고, active_foot, planning_com, planning_height, foot_velocity 및 발·ZMP·명령 기록을 저장한다.

일반 시작은 몸통 (0,0,0.6), 양발 (0,+0.2,0)/(0,-0.2,0), 몸통 속도는 명령값이다. 기본 전체 CoM 초기 높이는 0.4 m, 전진 속도는 0.2 m/s다.

## 검증과 한계

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s LIPM/tests -p test_mass_switch.py
.\.venv\Scripts\python.exe -m unittest discover -s LIPM/tests -p test_mass_switch_periodic.py
```

질량 0에서 기존 모델 일치, 몸통 상태 보존, 질량중심 정의, 고정 높이, 말단 DCM, 입력 검사 및 여러 질량·방향·DSP 조건의 전체 궤적 주기 반복을 검증한다.

지지발을 제외한 부분계에 LIPM을 적용한 근사이며, 전체 각운동량·스윙발 관성력·접촉 충격을 유도한 완전한 로봇 동역학은 아니다. 다리 길이·토크·마찰·착지 범위 제한도 없다. 일반 시작의 횡방향 과도응답이 크므로 물리적 안정성이 검증된 것으로 해석하면 안 된다.

실제 정책 적용 전 트렁크/루트 위치, 양발 제외 CoM, 전체 CoM을 구분해야 한다. 전체 CoM을 몸통 위치로 대입하면 발 질량을 이중 반영한다. 이 데모는 RL 관측이나 상태 추정을 변경하지 않는다.
