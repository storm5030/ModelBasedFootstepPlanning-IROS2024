# LIPM 폴더 구성과 실행

기존 모델들의 계산 방식은 유지하고 파일을 역할별로 분류했다.

- `LIPM/models/`: 기본 LIPM, DSP, mass-switch, total-CoM 기준 mass-switch 클래스
- `LIPM/demos/`: 네 모델의 실행 데모
- `LIPM/analysis/`: 원본/호환 analysis, DSP analysis, 공간 그림 도우미
- `LIPM/tests/`: 수치 검증과 회귀 테스트

레포지토리 루트에서 실행한다. 기존 LIPM 루트의 실행 파일은 하위 폴더로 이동했으므로 실행 경로를 변경해야 한다. README는 수정하지 않았다.

```powershell
.\.venv\Scripts\python.exe LIPM/demos/demo_LIPM_3D_vt.py
.\.venv\Scripts\python.exe LIPM/demos/demo_LIPM_3D_double_support.py
.\.venv\Scripts\python.exe LIPM/demos/demo_LIPM_3D_mass_switch.py --periodic-start
.\.venv\Scripts\python.exe LIPM/demos/demo_LIPM_3D_mass_switch_total_com.py --periodic-start
.\.venv\Scripts\python.exe LIPM/analysis/demo_LIPM_3D_double_support_analysis.py
.\.venv\Scripts\python.exe -m unittest discover -s LIPM/tests -p "test_*.py"
```

모듈 실행도 가능하다.

```powershell
.\.venv\Scripts\python.exe -m LIPM.demos.demo_LIPM_3D_mass_switch_total_com --periodic-start
```

total-CoM 데모의 --periodic-start는 이미 구현되어 있으며 이동 후에도 유지하고 검증했다. 옵션이 없으면 일반 초기값을 사용한다. 전체 CoM 초기 높이 0.6 m, 몸통 고정 높이 0.9 m, 계획 높이 0.72 m는 기본 40/10 kg 기준이다.

외부 코드에서 직접 import한다면 새 패키지 경로를 사용한다. 예: `from LIPM.models.LIPM_3D_mass_switch_total_com import LIPM3DMassSwitchTotalCoM`.

원본 analysis와 compatible analysis의 영상 저장 동작은 그대로다. 새 DSP analysis는 정적 PDF/PNG를 출력한다. 결과 파일은 기존처럼 outputs 아래에 저장하며 상대 출력 경로는 실행 작업 디렉터리 기준이다.
