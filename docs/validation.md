# 제자리 선회 후크 적재 검증

프로젝트: `/home/ohmyhong1/robot_ws/PoC_sim`, hostname `LAPTOP-CAHVEOTT`.
Python `/home/ohmyhong1/miniforge3/envs/robot/bin/python`, MuJoCo 3.13.0.

## 전체 사이클

구역당 8층, 총 32개를 두 가지 임의 적재 순서로 끝까지 수행했다.

| 항목 | seed 2 | seed 7 |
|---|---|---|
| 처리 개수 | 32 | 32 |
| 카메라 최종 개수 | [8, 8, 8, 8] | [8, 8, 8, 8] |
| 후크 안착 성공 | 32 / 32 | 32 / 32 |
| 제자리 선회 횟수 | 64 | 64 |
| 베이스 변위 | 0 m | 0 m |
| 최종 heading | 0.0000 rad | 0.0000 rad |
| 적재 중심 오차 최대 | 16.4 mm | 9.5 mm |
| 적재 중심 오차 평균 | 6.2 mm | 4.9 mm |
| 명령 관절 속도 / 제한 | 0.8662 | 0.8485 |
| IK 최대 가중 성분 오차 | 0.000294 | 0.000294 |
| 시뮬레이션 시간 | 1341.7 s | 1341.7 s |

베이스는 매 사이클 heading π(컨베이어)와 heading 0(팔레트) 사이를 왕복하며,
두 실행 모두 위치 변위 0 m로 제자리 회전만 했다. 구동 바퀴는 접지점이 미끄러지지
않는 각도로 함께 굴렸다.

슬롯은 0부터 시작한다(0=먼쪽 −Y, 1=먼쪽 +Y, 2=가까운쪽 −Y, 3=가까운쪽 +Y).

- seed 2 순서: 3, 0, 1, 2, 1, 2, 3, 0, 1, 0, 2, 3, 2, 3, 1, 0, 3, 0, 2, 1, 0, 2, 1, 3, 0, 2, 1, 3, 2, 0, 1, 3
- seed 7 순서: 3, 1, 2, 0, 3, 1, 2, 0, 3, 0, 1, 2, 1, 0, 3, 2, 3, 0, 1, 2, 3, 0, 2, 1, 0, 2, 3, 1, 1, 2, 0, 3

기록은 `reports/turn_base.json`, `reports/turn_base_seed7.json`이고
`reports/latest.json`은 seed 2 결과의 사본이다.

## 추가 검사

`tests/test_baseline.py` 10개 통과:

- IK 도달성과 공식 관절 범위 유지, 도달 불가 목표 거부.
- 후크 기준 자세(공구 좌표계가 월드 정렬)가 실제로 도달 가능.
- 손잡이 구멍이 중첩 pitch 이내에 있어 후크가 갇히지 않음, 벌린 후크가 토트 최대 폭 바깥으로 빠짐.
- 최상층(8층)을 운반 폭과 벌린 폭 양쪽에서 도달, 뒤돌아선 자세에서 컨베이어 집기 도달.
- 로봇/운반물의 장애물 침투 차단.
- 중력 하에서 중첩 pitch대로 안착.
- 베이스가 팔 동작 중에는 완전히 정지하고, 선회 시에는 위치를 유지한 채 heading만 바뀌며 바퀴가 구름.
- 양쪽 후크가 모두 턱에 닿지 않으면 결합 거부.
- 내부 개수 값을 의도적으로 틀리게 줘도 카메라 영상으로 실제 장면 개수를 추정하고, 가림 시 선택 거부.

## 그림

- `reports/turn_base.png`: seed 2 완료 상태.
- `reports/overhead_camera.png`: 판단에 사용한 상단 카메라 RGB.
- `reports/hook_pick.png`: 실제 후크 삽입 동작 렌더.
- `reports/hook_tool.png`, `reports/props_detail.png`: 후크 공구와 토트/팔레트 외형.

## 재현

```bash
cd /home/ohmyhong1/robot_ws/PoC_sim
conda activate robot
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 python run.py --boxes 32 --seed 2 --report reports/turn_base.json
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 python run.py --boxes 32 --seed 7 --report reports/turn_base_seed7.json
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 python -m unittest discover -s tests -v
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 python work/turn_base/reach_scan.py
python run.py --viewer --speed 2
```

## 검증 범위

로봇은 운동학 추종, 토트는 동적 안착이다. 양쪽 후크가 손잡이 턱에 닿은 것을 확인한 뒤
soft weld로 운반하며, 실제 후크 하중 분배·모터 토크·주행 동역학·전도 안정성 검증은 아니다.
카메라는 실제 렌더링한 깊이를 측정하되 분할 마스크는 시뮬레이터가 제공한다.
8층을 넘는 적재, 다른 pitch/배치, 연속 무충돌 보장은 하지 않는다.
