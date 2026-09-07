# 더블 스탠스 LIPM 분석

기존 더블 스탠스 데모의 create_model과 simulate를 재사용한다. 질량 스위칭 모델이 아니라 고정 높이 DSP LIPM을 분석한다.

## 실행

```powershell
.\.venv\Scripts\python.exe LIPM/analysis/demo_LIPM_3D_double_support_analysis.py
.\.venv\Scripts\python.exe LIPM/analysis/demo_LIPM_3D_double_support_analysis.py --start 0 --end 1.8
.\.venv\Scripts\python.exe LIPM/analysis/demo_LIPM_3D_double_support_analysis.py --periodic-start --duration 14 --start 5 --end 13
.\.venv\Scripts\python.exe LIPM/analysis/demo_LIPM_3D_double_support_analysis.py --headless
```

기본 설정은 SSP 0.48초, DSP 0.12초, dt 0.02초, 목표 속도 (0.3, 0) m/s, 높이 0.6 m, 총 10초다. 일반 시작의 초기 속도는 기존 DSP 데모와 같은 (0.3, 0) m/s이며 명령 옵션과 별개다. 발너비 목표는 0.4 → 0.6 → 0.4 m로 10/20번째 스텝에 변경한다.

## 그래프와 시간 범위

기본 출력은 원본 outputs/lipm_analysis_compatible의 LIP_2D / LIP_3D와 같은 독립 2D·3D 공간 그림이다. 검정·흰색 CoM 마커, 초록 CoM 궤적, 노란 ICP 궤적, 좌우 착지 목표점, 명령 속도 화살표 및 거리 주석을 표시한다. ZMP는 가는 주황선이다.

기본 확대 구간은 2.4~4.8초이며 짧은 실행에서는 자동 조절한다. --start/--end로 표시 구간만 선택하며 시뮬레이션은 항상 0초부터 진행한다. --snapshot으로 로봇 마커 시점을 선택한다. 생략하면 구간 중간이다.

--diagnostics를 추가하면 기존 네 패널 overview 및 ICP 시간 그래프도 화면에 표시한다. 이 추가 그래프는 옵션과 무관하게 파일로 저장한다. 실시간 애니메이션이나 동영상 자동 저장이 없으므로 ffmpeg가 필요 없다.

## ICP 정의

- 순간 ICP: xi = c + v / omega, omega = sqrt(g / h).
- 고정 높이 LIPM에서 수평 ICP와 DCM의 값은 같다.
- 운동식: xi_dot = omega * (xi - ZMP). DSP에서도 성립한다.
- 기존 모델의 eICP는 순간값이 아니라 SSP 종료 예측값이다.

기본 노란 선은 순간 ICP다. 원본 eICP와 같은 의미를 보려면 --icp-kind ssp-end를 사용한다. DSP 종료 예측값은 --icp-kind step-end다. SSP 종료 예측 배열은 DSP 구간에서 NaN으로 비워 놓는다. 예측은 현재 상태와 착지 목표를 이용하며, DSP에서는 선형 이동 ZMP의 정확한 전파를 사용한다.

2D의 b_x,b_y는 선택된 완료 스텝의 DSP 종료 순간 ICP에서 새 지지발을 뺀 세계 좌표 성분이다. s_x,w_y는 실제 양발 사이의 세계 좌표 거리다. 전진 보행에서 목표와 일치할 때만 s_d,w_d로 표시한다. SSP 종료 예측선을 선택해도 b 주석은 DSP 종료 기준이다.

ICP가 ZMP에서 멀다는 이유만으로 보행이 불안정하다고 판정할 수 없다. 이 모델에는 발바닥 지지 다각형이나 착지 가능 거리 제한이 없다.

## 저장과 검증

기본 출력 폴더는 outputs/lipm_double_support_analysis이며 --output-dir로 변경한다.

- LIP_2D.pdf/png, LIP_3D.pdf/png: 공간 그림
- overview.pdf/png, icp_diagnostics.pdf/png: 다중 패널 진단
- trajectory_analysis.npz: 전체 시뮬레이션과 ICP 분석 데이터
- icp_analysis.csv: 시간, 스텝, 지지 구간, CoM·속도·ZMP·ICP·착지점·예측값

CSV의 dsp는 1=DSP, 0=SSP다. 시간 단위는 초, 위치는 m, 속도는 m/s다. 표시 시간창과 무관하게 전체 데이터를 저장한다.

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s LIPM/tests -p test_double_support_analysis.py
```

일반/주기 시작 및 DSP 유무의 네 조건에서 예측 종료 ICP를 실제 시뮬레이션 종료 상태와 비교한다. DCM 전파 잔차는 구현 일관성 검사이며 실제 로봇 안정성 검증은 아니다.
