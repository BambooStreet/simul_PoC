# 운동학 기준선 (동역학 전환 전 동결)

동역학 전환 직전의 운동학 모드 결과입니다. 2~4단계의 비교 대상이며,
`--mode kinematic`으로 재현되어야 하는 값입니다.

## 모델 상태

P0(충돌 메쉬 면 누락) **복구 이후**입니다. 전 소스 OBJ 로드율 100%.
지문은 `metrics.json`의 `model_fingerprint`에 있습니다.

| 항목 | 값 |
|---|---|
| MuJoCo | 3.13.0 |
| 로봇 질량 | 142.1136 kg |
| 로봇 충돌 geom | 47개 (P0 복구 전 32개) |
| 환경 충돌 geom | 3개 (floor, pallet, conveyor) |
| `nmesh` / `ngeom` | 38 / 1563 |
| `nu` (액추에이터) | **0** — 위치 재생 모드 |
| timestep | 0.005 s |
| URDF sha256 (앞 16자) | `ed78894414455cef` |

## 실행 기록

| 파일 | 적재 | 구역별 | 시뮬 시간 | 개당 | 결과 |
|---|---|---|---|---|---|
| `run32_ordered_PASS.json` | 32/32 | [8,8,8,8] | 1341.6 s | 41.93 s | PASS |
| `run32_seed7_PASS.json` | 32/32 | [8,8,8,8] | 1341.8 s | 41.93 s | PASS |
| `run28_seed2_PASS.json` | 28/28 | [7,7,7,7] | 1183.6 s | 42.27 s | PASS |
| `run32_seed2_FAIL.json` | 30/32 | [8,7,8,7] | 1295.6 s | 43.19 s | **FAIL** |
| `run32_seed2_PRE_P0_PASS.json` | 32/32 | [8,8,8,8] | 1341.7 s | 41.93 s | PASS (P0 복구 **전**, 참고용) |

`run32_seed2_FAIL`은 알려진 미해결 건입니다 — 원거리 열 8층째의 여유가
`AVOID_REQUIRE`(14 mm)를 밑돌아(슬롯0 +5.3 mm, 슬롯1 +13.8 mm) 웨이포인트
사이 보간 구간에서 3.8 mm 관통합니다. 웨이포인트 자체는 전부 해가 있습니다.
적재 순서에 의존하므로 `ordered`와 seed 7에서는 통과합니다.
동역학 작업으로 자세가 바뀌면 수치가 달라지므로 **마지막에 재측정**합니다.

## 재현

```bash
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 python run.py --boxes 32 --selection ordered
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 python run.py --boxes 32 --seed 7
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 python run.py --boxes 28 --seed 2
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 python run.py --boxes 32 --seed 2   # FAIL 재현
```

모델을 다시 만들어야 하면 `python scripts/prepare_robot.py`를 먼저 돌립니다.
모델 무결성은 `python scripts/step1_check.py`로 확인합니다.

## 동역학이 재현해야 할 수치

| 지표 | 운동학 값 | 비고 |
|---|---|---|
| 사이클 타임 | 41.93 s/개 | 속도 한계만으로 결정된 하한. 동역학에서는 늘어나는 것이 정상 |
| 최대 관절 속도 사용률 | 0.866 | 유일하게 살아 있던 물리 제약 |
| 최대 IK 성분 오차 | 2.94e-4 m | 동역학에서는 추종 오차로 대체됨 |
| 최대 스택 드리프트 | 0.0164 m | 3단계(실제 접촉) 이후 재측정 |
| 베이스 변위 | 0.0 m | 제자리 선회. 4단계에서 슬립이 생기면 커짐 |

`step1_collision_front.png` / `step1_collision_side.png`는 복구 후 충돌 형상입니다.
