# Dexmate Vega-1 제자리 선회 토트 적재 PoC

MuJoCo에서 **로봇 뒤쪽 컨베이어**의 빈 토트를 양손 후크로 손잡이 구멍에 꽂아 집고,
수평 이동 없이 **제자리에서 뒤로 돌아** 정면 팔레트의 2×2 구역에 중첩 적재합니다.
베이스는 회전만 하고 위치는 고정이며, 공식 모델의 몸통 3관절과 양팔 14관절을 사용합니다.
기본 32개, 구역당 8층입니다.

## 실행

```bash
cd /home/ohmyhong1/robot_ws/PoC_sim
conda activate robot
python run.py --viewer
```

기본 2배속이며 `--speed 4`로 더 빠르게 재생할 수 있습니다. 물리 timestep과 관절 속도 제한은 유지합니다. 실제 배속은 컴퓨터 성능에 따라 제한될 수 있습니다.

GUI 없는 검증도 카메라 렌더링이 필요합니다.

```bash
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 python run.py --boxes 32 --seed 2 --image reports/turn_base.png --camera-image reports/overhead_camera.png
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 python -m unittest discover -s tests -v
```

모델 자체의 점검은 `python scripts/step1_check.py`(`--write`로 `reports/step1_model_check.md`
갱신), 선회 중 바퀴 횡슬립 측정은 `python scripts/verify_steering.py`,
동역학 통과 기준 9개는 `python scripts/verify_dynamics.py`입니다.

## 구동 모드

`--mode kinematic`(기본)은 궤적을 상태에 직접 기록하는 이상적 위치 재생이며,
`reports/baseline_kinematic/`의 동결 결과가 이 모드로 기록됐습니다. 토트는 두 모드에서
모두 실제 물리입니다.

`--mode dynamic`은 몸통 3 + 양팔 14 = **17개 관절을 토크 액추에이터로 구동**합니다
(`forcerange` = URDF effort). 제어기는 역동역학 피드포워드 + 적재물 보상 + PD이고,
게인은 운반 자세에서 측정한 관절 관성에 비례합니다. 베이스 선회·머리·바퀴는 2단계에서
아직 이상화된 위치 구속이며 4단계에서 실제 모터로 바뀝니다. timestep은 2 ms입니다.

근거 없는 값(ωn, armature β 등)은 `config/dynamics.json`에 `source: estimated`로
들어가고 실행 시작 시 목록으로 출력됩니다. 측정 결과와 남은 한계는
`docs/dynamics_step2_design.md` 9절에 있습니다.

- `--seed 2`: 임의 적재 순서 재현. 생략하면 실행마다 새 seed를 생성하고 보고서에 기록합니다.
- `--selection ordered`: 같은 높이의 후보 중 슬롯 번호순 선택. 기본은 `random`입니다.
- `--nesting-pitch 0.058`: 토트 하나를 포갤 때 증가 높이(m). 사용자 현장 수치입니다.
- `--boxes 32`: 이번 팔레트에서 처리할 개수. 기본 32는 구역당 8층이며 최대값입니다.
- `--report reports/latest.json`: 이벤트와 검증 결과 저장 경로.

처음 설치할 때 `bash scripts/setup.sh`로 공식 로봇 패키지를 프로젝트 `.deps`에 설치하고, 이어서
`python scripts/prepare_robot.py`로 모델을 생성합니다. 저장소는 생성물(`assets/vega.xml`,
`assets/vega_collision.urdf`, `assets/scene.xml`, `assets/collision/`)을 추적하지 않으므로 이 단계를
건너뛰면 모델이 없습니다. 생성 결과는 `python scripts/step1_check.py`로 검증할 수 있습니다.
`.deps`에 런타임 의존성도 설치했다면 `PYTHONPATH="$PWD/.deps" python run.py --viewer`로 실행합니다.
프로젝트를 옮기면 준비 스크립트를 다시 실행하세요. 현재 Python은
`/home/ohmyhong1/miniforge3/envs/robot/bin/python`, MuJoCo는 3.13.0입니다.

## 현장 수치

| 항목 | 값 | 출처 |
|---|---|---|
| 팔레트 높이 | 123 mm | 사용자 |
| 토트 높이 | 252 mm | 사용자 |
| 중첩 1개당 증가 | 58 mm (단차 없이 맞물림) | 사용자 |
| 토트 바닥 면적 | 400 × 500 mm | 첨부 사진 |
| 엔드이펙터 | 손잡이 구멍 삽입식 고정 후크 | 사용자 사진 5장 |

## 동작과 판단

상단 RGB-D 카메라로 네 구역의 높이/개수 확인 → 가장 낮은 가시 구역 중 임의 선택 →
운반 자세로 복귀 → **heading π로 제자리 선회(뒤돌기)** → 컨베이어 공급 →
후크를 벌려 토트 옆으로 하강 → 후크를 손잡이 구멍에 삽입 → 턱에 안착 →
양쪽 돌기 접촉 확인 후 soft weld → 들어 올리기 → **heading 0으로 제자리 선회** →
구역 위로 이동 → 하강해 중첩 → weld 해제 → 후크를 빼고 림 위로 상승 → 카메라로 결과 확인.

컨베이어에서 토트를 집은 뒤 별도의 수평 이동 없이 선회만으로 팔레트 쪽으로 옮깁니다.
선회 중 운반 토트의 궤적 반경(0.684 m)이 가장 가까운 적재 열의 안쪽 끝(0.733 m)보다
작도록 운반 자세를 잡았습니다. 공식 모델에 허리 회전 관절이 없으므로 뒤돌기는 원래
있는 베이스의 Z축 회전으로 구현했습니다. 베이스는 차동 구동이 아니라 **조향 구동**이어서
바퀴마다 조향 관절(`*_wheel_j1`)이 굴림 관절(`*_wheel_j2`) 위에 있습니다. 선회 중심이 구동축
중점보다 185 mm 뒤에 있으므로, 세 바퀴를 각자의 선회 접선으로 조향한 뒤 굴림각을 다시 계산해
접지점 횡슬립을 없앴습니다(최대 5.1e-7 m/s, 조향 고정 시 0.14 m/s).

**층수는 8층입니다.** 중첩 pitch가 58 mm로 확정되면서, 9층째를 쌓으려면 먼 열(x=1.37) 위를
토트를 든 채 1.015 m 높이로 지나가야 하는데 이 자세의 도달 한계가 0.991 m입니다.
근거와 측정값은 `docs/requirements.md`에 있습니다.

**현재는 운동학 경로 프로토타입입니다.** 후크는 사용자 사진을 기능 중심으로 근사한 모델이며,
양쪽 돌기가 손잡이 턱에 닿은 것을 확인한 뒤 soft weld로 운반을 안정화합니다. 후크 강성이나
모터 토크·주행 동역학·전도 안정성은 검증하지 않습니다. 카메라 깊이 영상에서 높이를
측정하지만 토트 영역은 시뮬레이터의 semantic mask를 사용합니다. 중첩 접촉은 단순 지지
형상을 사용합니다.

## 파일

- `poc_sim/gripper.py`: 손잡이 후크 형상과 삽입/해제 간격
- `poc_sim/ik.py`: 몸통·양팔 동시 IK, 양손 간격(span) 지정, 공식 관절 범위 유지
- `poc_sim/perception.py`: 상단 RGB/깊이/분할 영상 및 구역별 높이 추정
- `poc_sim/simulation.py`: 후크 안착 확인, 제자리 선회, 공급·운반·적재 흐름
- `poc_sim/layout.py`, `poc_sim/scene.py`: 작업대 배치와 장면, 베이스 회전 관절
- `docs/requirements.md`: 자료 근거, 배치 가정, 층수 한계 계산과 모델 한계
- `docs/validation.md`: 전체 검증 결과
- `docs/visual_assets.md`: 토트·팔레트·후크 외형 근거
- `reports/latest.json`: 최신 전체 실행 기록

공식 원본: https://github.com/dexmate-ai/dexmate-urdf (Apache-2.0), `dexmate-urdf==0.8.4`. 실제 현장 모델의 세부 변형은 미확인입니다. 청보라색 테이퍼 토트, 녹색 격자 팔레트, 검정 후크는 사진을 참고한 근사 외형입니다.

`MUJOCO_GL=egl python scripts/preview_props.py`로 외형을, `MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 python scripts/preview_grasp.py`로 실제 후크 삽입 장면을 저장할 수 있습니다.
