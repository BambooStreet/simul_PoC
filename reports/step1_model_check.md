# 1단계 로봇 모델 검증 보고서

- 대상: Dexmate Vega-1 토트 적재 PoC (`/home/ohmyhong1/robot_ws/PoC_sim`)
- 검증 기준: **MuJoCo에 로드·컴파일된 mjModel** (`assets/scene.xml`)
- MuJoCo 3.13.0, Python 3.10.21
- 재실행: `MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 python scripts/step1_check.py --write`
- 모델 규모: `nbody=80` `njnt=59` `nq=251` `nv=219` `ngeom=1563` `nu=0` `ncam=2` `neq=64` `nexclude=131`
- 로봇 body 범위: 1 ~ 47 (47개), 토트: 48 이상

## 판정 요약

| 장 | 항목 | 판정 | 요약 |
|---|---|---|---|
| 0 | 모델 파일 · 변환 경로 | 의심 | `vega_1.urdf` → `assets/vega.xml` 2단계 변환. vega.xml은 **동결 산출물**이라 `.deps` 갱신이 자동 반영되지 않음 |
| 1 | 질량 합계 | 의심 | 142.11 kg vs 제시 스펙 135 kg (+5.3%). 스펙 출처 미확인 |
| 1 | 관성 타당성 | 정상 | 주관성 전부 양수, 삼각부등식 위반 0개, 변환 중 재계산 0개. 질량 0 링크 6개는 모두 센서·기준 프레임 |
| 2 | 관절 위치 한계 | 정상 | URDF lower/upper가 `jnt_range`에 전부 일치 반영. placeholder 의심값 없음 |
| 2 | `effort` 반영 | 문제 | `jnt_actfrcrange`에 값은 들어왔으나 `nu=0`(액추에이터 0개)라 **토크 한계가 동작에 전혀 작용하지 않음** |
| 2 | `velocity` 반영 | 정상 | MuJoCo 모델 차원에는 속도 한계가 없음(엔진 구조). `simulation.py`가 URDF를 재파싱해 궤적 시간 계산에 사용 — 경로 확인됨 |
| 2 | damping / armature / frictionloss | 의심 | 전 DOF 합계가 모두 0. 모터 회전자 관성·조인트 마찰 없음 |
| 3 | 단위 (각도·길이·질량·토크) | 정상 | `angle=radian`, `mesh_scale=(1,1,1)`, 중력 −9.81 m/s². SI 일관 |
| 4 | 충돌 메쉬 무결성 | 정상 | 전 소스 OBJ 로드율 100%. 볼록 분해 조각이 각각 별도 geom으로 등록됨 |
| 4 | 충돌 ↔ visual 치수 | 의심 | 18/19는 +1~4%(정상적 팽창). `arm_l8`만 축별 +24/−27/−19%로 형상 불일치 |
| 4 | contype / conaffinity · exclude | 정상 | 비트 규약 일관, SRDF 131쌍이 `nexclude=131`에 전부 반영 |
| 4 | 후크 형상 | 정상 | 돌기 20×84 mm가 손잡이 구멍 40×116 mm에 여유 있게 삽입. 구동 조인트 없음은 문서에 명시된 설계 |
| 5 | 헤드 카메라 | 문제 | 로봇 부착 카메라 0개. 공식 URDF가 `zed_left/right_camera`를 렌더용으로 쓰라고 주석에 명시했으나 미정의 |
| 5 | 실제 장비 사양 | 미확인 | ZED 계열·베이스라인 50 mm까지만 URDF로 확인. 모델명·해상도·FOV 미확인 |

판정 기준 — **정상**: 확인 결과 문제 없음 / **의심**: 근거가 부족하거나 값이 어긋나지만 즉시 오작동을 일으키지는 않음 / **문제**: 2단계 결과를 왜곡함 / **미확인**: 외부 자료로 확인하지 못함.

## 0. 사용 중인 모델 파일

### 로드 경로 (2단계 변환)

```
.deps/dexmate_urdf/robots/humanoid/vega_1/vega_1.urdf      공식 원본
        │  scripts/prepare_robot.py
        │    · 모든 <visual> 엘리먼트 제거
        │    · <mujoco><compiler discardvisual=true
        │                        fusestatic=false
        │                        balanceinertia=true/>  주입
        ▼
assets/vega_collision.urdf                                 중간 산출물
        │  MjModel.from_xml_path() -> mj_saveLastXML()
        ▼
assets/vega.xml                                            동결된 MJCF
        │  poc_sim/scene.py build()
        │    · base_yaw hinge 추가, 후크/토트/팔레트/컨베이어/카메라 추가
        │    · vega_1.srdf 의 disable_collisions -> <contact><exclude>
        ▼
assets/scene.xml  ->  MjModel  (이 보고서의 검증 대상)
```

| 단계 | 경로 | 존재 | 크기 |
|---|---|---|---|
| 공식 URDF | `.deps/dexmate_urdf/robots/humanoid/vega_1/vega_1.urdf` | 있음 | 31,833 B |
| 중간 URDF | `assets/vega_collision.urdf` | 있음 | 30,027 B |
| 동결 MJCF (로봇) | `assets/vega.xml` | 있음 | 20,319 B |
| 최종 MJCF (씬) | `assets/scene.xml` | 있음 | 265,145 B |
| SRDF | `.deps/dexmate_urdf/robots/humanoid/vega_1/vega_1.srdf` | 있음 | 9,673 B |

**선택된 변형:** `vega_1.urdf` — 손 없는 기본 변형입니다. `vega_1_gripper.urdf`(dexs 그리퍼), `vega_1_f5d6.urdf`(5지 핸드)는 쓰지 않습니다. 말단은 `poc_sim/gripper.py`가 붙이는 **자체 제작 후크**입니다.

### compiler 옵션

- URDF에 주입된 값: `discardvisual=true`, `fusestatic=false`, `balanceinertia=true`
- 저장된 `assets/vega.xml`의 compiler: `angle=radian`
- `meshdir`: 설정하지 않음. `prepare_robot.py`가 메쉬 경로를 **절대경로로 치환**합니다.
- `inertiafromgeom`: 설정하지 않음 → MuJoCo 기본값 `auto` (URDF에 `<inertial>`이 있으면 그것을 사용, 없으면 geom에서 계산)

## 1. 질량 · 관성

- **mjModel 로봇 질량 합계: 142.114 kg** (body 1 ~ 47, 토트 제외)
- URDF `<mass>` 합계: 140.914 kg
- 그 중 자체 제작 후크(`L/R_tool`): 1.200 kg
- 공식 URDF 유래 질량: 140.914 kg

| 비교 대상 | 값 | 차이 |
|---|---|---|
| 공개 스펙 (사용자 제시) | 약 135 kg | - |
| mjModel 전체 | 142.11 kg | +7.11 kg (+5.3%) |
| mjModel − 후크 | 140.91 kg | +5.91 kg (+4.4%) |

> 공개 스펙 135 kg의 **출처를 확인하지 못했습니다**(사용자 제시값). 구성(핸드 종류, 배터리)에 따라 달라질 수 있어 판정은 참고용입니다.

### 링크별 질량 · 관성

| 링크 | 질량 kg | Ixx' | Iyy' | Izz' | 플래그 | 비고 |
|---|---|---|---|---|---|---|
| base | 88.435 | 1.809 | 1.072 | 0.9905 | - | URDF질량 88.435 |
| B_wheel_l1 | 0.200 | 0.0001474 | 0.0001096 | 0.0001 | - | - |
| B_wheel_l2 | 0.300 | 0.0002203 | 0.0001343 | 0.0001343 | - | - |
| R_wheel_l1 | 0.358 | 0.0007096 | 0.000664 | 0.0002204 | - | - |
| R_wheel_l2 | 3.700 | 0.01139 | 0.00676 | 0.00676 | - | - |
| L_wheel_l1 | 0.358 | 0.0007096 | 0.000664 | 0.0002204 | - | - |
| L_wheel_l2 | 3.700 | 0.01139 | 0.00676 | 0.00676 | - | - |
| torso_l1 | 8.832 | 0.2218 | 0.214 | 0.02657 | - | - |
| torso_l2 | 14.589 | 0.4247 | 0.4082 | 0.03998 | - | URDF질량 14.589 |
| torso_l3 | 5.329 | 0.114 | 0.1049 | 0.02467 | - | - |
| arm_center | 0.000 | 0 | 0 | 0 | 질량0 관성≤0 | URDF 링크이지만 `<inertial>` 없음 (센서/기준 프레임) |
| head_l1 | 0.438 | 0.00051 | 0.00043 | 0.00025 | - | - |
| head_l2 | 0.608 | 0.0005591 | 0.00052 | 0.0003309 | - | - |
| head_l3 | 0.633 | 0.0008318 | 0.0007784 | 0.0003398 | - | - |
| zed_depth_frame | 0.000 | 0 | 0 | 0 | 질량0 관성≤0 | URDF 링크이지만 `<inertial>` 없음 (센서/기준 프레임) |
| zed_left_camera | 0.000 | 0 | 0 | 0 | 질량0 관성≤0 | URDF 링크이지만 `<inertial>` 없음 (센서/기준 프레임) |
| zed_right_camera | 0.000 | 0 | 0 | 0 | 질량0 관성≤0 | URDF 링크이지만 `<inertial>` 없음 (센서/기준 프레임) |
| L_arm_l1 | 1.414 | 0.002131 | 0.002086 | 0.001432 | - | - |
| L_arm_l2 | 1.406 | 0.00441 | 0.004187 | 0.001613 | - | - |
| L_arm_l3 | 1.161 | 0.003378 | 0.003262 | 0.001 | - | - |
| L_arm_l4 | 0.932 | 0.003091 | 0.002825 | 0.0008848 | - | - |
| L_arm_l5 | 0.646 | 0.0005601 | 0.00052 | 0.0003399 | - | - |
| L_arm_l6 | 0.554 | 0.0004364 | 0.00038 | 0.0002736 | - | - |
| L_arm_l7 | 0.358 | 0.0003767 | 0.00031 | 0.0001633 | - | - |
| L_arm_l8 | 0.201 | 9.246e-05 | 5.736e-05 | 5.668e-05 | - | - |
| L_ee | 0.000 | 0 | 0 | 0 | 질량0 관성≤0 | URDF 링크이지만 `<inertial>` 없음 (센서/기준 프레임) |
| L_tool | 0.600 | 0.00439 | 0.004383 | 0.0009282 | - | URDF에 없음 (프로젝트 추가분) |
| R_arm_l1 | 1.424 | 0.002025 | 0.001907 | 0.001338 | - | - |
| R_arm_l2 | 1.406 | 0.00441 | 0.004187 | 0.001613 | - | - |
| R_arm_l3 | 1.161 | 0.003379 | 0.003262 | 0.0009997 | - | - |
| R_arm_l4 | 0.932 | 0.003091 | 0.002825 | 0.0008848 | - | - |
| R_arm_l5 | 0.646 | 0.0005601 | 0.00052 | 0.0003399 | - | - |
| R_arm_l6 | 0.554 | 0.0004365 | 0.00038 | 0.0002735 | - | - |
| R_arm_l7 | 0.358 | 0.0003767 | 0.00031 | 0.0001633 | - | - |
| R_arm_l8 | 0.201 | 9.246e-05 | 5.736e-05 | 5.668e-05 | - | - |
| R_ee | 0.000 | 0 | 0 | 0 | 질량0 관성≤0 | URDF 링크이지만 `<inertial>` 없음 (센서/기준 프레임) |
| R_tool | 0.600 | 0.00439 | 0.004383 | 0.0009282 | - | URDF에 없음 (프로젝트 추가분) |
| lidar | 0.004 | 1.641e-07 | 1.387e-07 | 1.362e-07 | 질량<10g | - |
| imu | 0.018 | 3.494e-06 | 2.229e-06 | 1.976e-06 | - | - |
| L_base_cam | 0.003 | 1.666e-07 | 1.229e-07 | 9.214e-08 | 질량<10g | - |
| R_base_cam | 0.003 | 1.666e-07 | 1.229e-07 | 9.214e-08 | 질량<10g | - |
| F_base_cam | 0.003 | 1.666e-07 | 1.229e-07 | 9.214e-08 | 질량<10g | - |
| B_base_cam | 0.003 | 1.666e-07 | 1.229e-07 | 9.214e-08 | 질량<10g | - |
| LF_ultrasonic | 0.012 | 1.374e-06 | 1.154e-06 | 7.623e-07 | - | - |
| RF_ultrasonic | 0.012 | 1.374e-06 | 1.154e-06 | 7.623e-07 | - | - |
| LB_ultrasonic | 0.012 | 1.371e-06 | 1.157e-06 | 7.622e-07 | - | - |
| RB_ultrasonic | 0.012 | 1.371e-06 | 1.157e-06 | 7.622e-07 | - | - |

*Ixx'/Iyy'/Izz' 는 mjModel이 보유한 **주관성모멘트**(principal moments, kg·m²)입니다.*

### 관성 타당성 판정

- 질량 0 링크: **6개** — arm_center, zed_depth_frame, zed_left_camera, zed_right_camera, L_ee, R_ee
- mjModel 주관성 중 0 이하: **6개**
- mjModel 삼각부등식 위반: **0개**
- **URDF 원본**이 삼각부등식을 위반한 링크: **0개**
- 변환 중 관성이 1% 이상 바뀐 링크: **0개**
`balanceinertia=true`가 켜져 있으므로, URDF가 삼각부등식을 위반한 링크는 MuJoCo가 **경고 없이 수정**합니다.

## 2. 관절 한계

| 관절 | 타입 | mjModel jnt_range | URDF lower~upper | 반영 | URDF velocity | URDF effort | jnt_actfrcrange |
|---|---|---|---|---|---|---|---|
| base_yaw | hinge | 무제한 | - | - | - | - | 미설정 |
| B_wheel_j1 | hinge | 무제한 | - | - | - | - | 미설정 |
| B_wheel_j2 | hinge | 무제한 | - | - | - | - | 미설정 |
| R_wheel_j1 | hinge | -2.722 ~ 2.722 | -2.722 ~ 2.722 | 일치 | 3 | 6 | True [-6.0, 6.0] |
| R_wheel_j2 | hinge | 무제한 | - | - | 12 | 16 | True [-16.0, 16.0] |
| L_wheel_j1 | hinge | -2.722 ~ 2.722 | -2.722 ~ 2.722 | 일치 | 3 | 6 | True [-6.0, 6.0] |
| L_wheel_j2 | hinge | 무제한 | - | - | 12 | 16 | True [-16.0, 16.0] |
| torso_j1 | hinge | 0 ~ 1.57 | 0 ~ 1.57 | 일치 | 0.9 | 700 | True [-700.0, 700.0] |
| torso_j2 | hinge | 0 ~ 3.141 | 0 ~ 3.141 | 일치 | 0.9 | 380 | True [-380.0, 380.0] |
| torso_j3 | hinge | -1.57 ~ 1.57 | -1.57 ~ 1.57 | 일치 | 0.9 | 380 | True [-380.0, 380.0] |
| head_j1 | hinge | -1.483 ~ 1.483 | -1.483 ~ 1.483 | 일치 | 3.2 | 6 | True [-6.0, 6.0] |
| head_j2 | hinge | -2.792 ~ 2.792 | -2.792 ~ 2.792 | 일치 | 3.2 | 2.5 | True [-2.5, 2.5] |
| head_j3 | hinge | -1.378 ~ 1.483 | -1.378 ~ 1.483 | 일치 | 3.2 | 6 | True [-6.0, 6.0] |
| L_arm_j1 | hinge | -3.071 ~ 3.071 | -3.071 ~ 3.071 | 일치 | 2.4 | 150 | True [-150.0, 150.0] |
| L_arm_j2 | hinge | -0.453 ~ 1.553 | -0.453 ~ 1.553 | 일치 | 2.4 | 150 | True [-150.0, 150.0] |
| L_arm_j3 | hinge | -3.071 ~ 3.071 | -3.071 ~ 3.071 | 일치 | 2.7 | 80 | True [-80.0, 80.0] |
| L_arm_j4 | hinge | -3.071 ~ 0.244 | -3.071 ~ 0.244 | 일치 | 2.7 | 80 | True [-80.0, 80.0] |
| L_arm_j5 | hinge | -3.071 ~ 3.071 | -3.071 ~ 3.071 | 일치 | 2.7 | 25 | True [-25.0, 25.0] |
| L_arm_j6 | hinge | -1.396 ~ 1.396 | -1.396 ~ 1.396 | 일치 | 2.7 | 25 | True [-25.0, 25.0] |
| L_arm_j7 | hinge | -1.378 ~ 1.117 | -1.378 ~ 1.117 | 일치 | 2.7 | 25 | True [-25.0, 25.0] |
| R_arm_j1 | hinge | -3.071 ~ 3.071 | -3.071 ~ 3.071 | 일치 | 2.4 | 150 | True [-150.0, 150.0] |
| R_arm_j2 | hinge | -1.553 ~ 0.453 | -1.553 ~ 0.453 | 일치 | 2.4 | 150 | True [-150.0, 150.0] |
| R_arm_j3 | hinge | -3.071 ~ 3.071 | -3.071 ~ 3.071 | 일치 | 2.7 | 80 | True [-80.0, 80.0] |
| R_arm_j4 | hinge | -3.071 ~ 0.244 | -3.071 ~ 0.244 | 일치 | 2.7 | 80 | True [-80.0, 80.0] |
| R_arm_j5 | hinge | -3.071 ~ 3.071 | -3.071 ~ 3.071 | 일치 | 2.7 | 25 | True [-25.0, 25.0] |
| R_arm_j6 | hinge | -1.396 ~ 1.396 | -1.396 ~ 1.396 | 일치 | 2.7 | 25 | True [-25.0, 25.0] |
| R_arm_j7 | hinge | -1.117 ~ 1.378 | -1.117 ~ 1.378 | 일치 | 2.7 | 25 | True [-25.0, 25.0] |

### URDF velocity / effort 가 모델에 반영되는가

| 값 | mjModel 필드 | 상태 |
|---|---|---|
| `lower`/`upper` | `jnt_range` + `jnt_limited` | 반영됨 (위 표 '반영' 열) |
| `velocity` | **없음** | MuJoCo는 관절 속도 한계를 모델 차원에서 지원하지 않습니다 |
| `effort` | `jnt_actfrcrange` (또는 actuator `forcerange`) | `nu=0` 이므로 **반영 경로가 존재하지 않습니다** |

`velocity` 값의 실제 사용처: `poc_sim/simulation.py:116-121`이 **URDF를 직접 재파싱해서** `self.vel_limits`에 담고, `track()`이 궤적 구간 소요시간을 `max(|Δq| / vel_limit) × 1.15`로 계산할 때 씁니다. 즉 mjModel이 아니라 **프로젝트 플래너가 자체적으로** 지키는 제약입니다.

`effort` 값의 실제 사용처: **없음.** 모델에 액추에이터가 0개(`nu=0`, `na=0`, `d.ctrl` 크기 0)이고, `poc_sim/` 전체에서 `effort`를 읽는 코드가 없습니다.

### placeholder 의심값

- 1.0 / 10 / 100 / 1000 같은 둥근 값으로 채워진 velocity·effort는 없습니다.

동일값 반복 분포 (URDF 전체 관절):

| 항목 | 값 → 관절 수 |
|---|---|
| velocity | 0.9→3, 2.4→4, 2.7→10, 3→2, 3.2→3, 12→2 |
| effort | 2.5→1, 6→4, 16→2, 25→6, 80→4, 150→4, 380→2, 700→1 |

### 관절 저항 파라미터

| 필드 | 총합 | 비고 |
|---|---|---|
| `dof_damping` | 0 | URDF `<dynamics damping>` 미지정 |
| `dof_armature` | 0 | 모터 회전자 관성 없음 |
| `dof_frictionloss` | 0 | 조인트 마찰 없음 |

## 3. 단위

- **각도 단위:** `assets/vega.xml`의 `compiler angle="radian"`. URDF는 규격상 항상 라디안이고, MuJoCo URDF 임포터는 radian으로 고정하므로 일치합니다.
- 제한된 hinge 22개의 가동범위: 최소 1.570 rad (90.0°), 최대 6.142 rad (351.9°)
  - 라디안 해석 시 최대 351.9° → 합리적. 만약 이 값이 degree였다면 6.1° 로 비정상적으로 좁아집니다. **라디안 맞습니다.**
  - 2π 초과 가동범위: 0개

### 메쉬 스케일 / 전체 치수

- 로봇 메쉬 34개의 `mesh_scale` 고유값: (1, 1, 1)
  - `(1,1,1)` 이면 OBJ가 **미터 단위**로 저장돼 있고 스케일 변환이 없다는 뜻입니다. 해당됩니다.

| 축 | min (m) | max (m) | 크기 (m) |
|---|---|---|---|
| X (전후) | -0.4634 | +0.4920 | 0.9554 |
| Y (좌우) | -0.4325 | +0.4325 | 0.8650 |
| Z (높이) | +0.0001 | +1.6092 | 1.6092 |

- 위 값은 **현 자세**(IK 초기해 — 몸통을 굽히고 양팔을 운반 자세로 든 상태) 기준입니다. 바운딩박스는 `qpos`에 의존하므로 스펙 대조에는 부적절합니다.

- 모든 관절 0 (`mj_resetData`)에서의 높이: **1.064 m**. `torso_j1/j2`의 가동범위가 각각 0~1.57, 0~3.141이라 **0은 직립이 아니라 가장 접힌 자세**입니다. 스펙 대조에는 쓸 수 없습니다.

몸통 3관절을 13×13×13 격자로 훑어 찾은 **최대 신장 자세** (`torso_j1=1.308`, `torso_j2=2.094`, `torso_j3=1.570` rad, 팔은 0):

| 축 | min (m) | max (m) | 크기 (m) |
|---|---|---|---|
| X (전후) | -0.9501 | +0.3487 | 1.2988 |
| Y (좌우) | -0.2847 | +0.2847 | 0.5694 |
| Z (높이) | +0.0001 | +1.9323 | 1.9322 |

- **최대 신장 높이: 1.932 m** — 사용자 제시 약 1.7 m 대비 +0.232 m (+13.7%)
- 운전 중 실제 자세(IK 초기해) 높이: 1.609 m
- 높이는 **면이 누락된 메쉬**(`head_l1`, `head_l3`, `torso_l3`)의 영향을 받습니다. 머리 충돌 형상이 잘려 있으므로 실제 외형 높이는 이보다 클 수 있습니다. 4장의 메쉬 무결성 항목을 참조하세요.
- 1.7 m라는 수치의 **출처는 확인하지 못했습니다**(사용자 제시값).

### 토크 · 질량 단위

- URDF `effort`는 revolute 관절에서 규격상 **N·m**입니다. 구동 16, 조향 6, 팔 관절은 아래 2장 표 참조.
- URDF `<mass value>`는 규격상 **kg**. 합계가 사람이 들 수 없는 수준(142 kg)으로 나오는 것이 kg 해석과 일관됩니다 (g이었다면 0.142 kg).
- `m.opt.gravity = [0.0, 0.0, -9.81]` → m/s². 길이 m · 질량 kg · 시간 s 의 SI 조합이 일관됩니다.

## 4. 충돌 형상

### 링크별 충돌 geom

| 링크 | 충돌 geom 수 | 종류 | visual geom 수 | 비고 |
|---|---|---|---|---|
| base | 3 | mesh×3 | 0 | - |
| B_wheel_l1 | 0 | - | 0 | **충돌 geom 없음** |
| B_wheel_l2 | 1 | mesh×1 | 0 | - |
| R_wheel_l1 | 0 | - | 0 | **충돌 geom 없음** |
| R_wheel_l2 | 1 | mesh×1 | 0 | - |
| L_wheel_l1 | 0 | - | 0 | **충돌 geom 없음** |
| L_wheel_l2 | 1 | mesh×1 | 0 | - |
| torso_l1 | 1 | mesh×1 | 0 | - |
| torso_l2 | 1 | mesh×1 | 0 | - |
| torso_l3 | 9 | mesh×9 | 0 | - |
| arm_center | 0 | - | 0 | **충돌 geom 없음** |
| head_l1 | 3 | mesh×3 | 0 | - |
| head_l2 | 1 | mesh×1 | 0 | - |
| head_l3 | 2 | mesh×2 | 0 | - |
| zed_depth_frame | 0 | - | 0 | **충돌 geom 없음** |
| zed_left_camera | 0 | - | 0 | **충돌 geom 없음** |
| zed_right_camera | 0 | - | 0 | **충돌 geom 없음** |
| L_arm_l1 | 1 | mesh×1 | 0 | - |
| L_arm_l2 | 1 | mesh×1 | 0 | - |
| L_arm_l3 | 1 | mesh×1 | 0 | - |
| L_arm_l4 | 1 | mesh×1 | 0 | - |
| L_arm_l5 | 1 | mesh×1 | 0 | - |
| L_arm_l6 | 1 | mesh×1 | 0 | - |
| L_arm_l7 | 2 | mesh×2 | 0 | - |
| L_arm_l8 | 1 | mesh×1 | 0 | - |
| L_ee | 0 | - | 0 | **충돌 geom 없음** |
| L_tool | 3 | box×3 | 16 | - |
| R_arm_l1 | 1 | mesh×1 | 0 | - |
| R_arm_l2 | 1 | mesh×1 | 0 | - |
| R_arm_l3 | 1 | mesh×1 | 0 | - |
| R_arm_l4 | 1 | mesh×1 | 0 | - |
| R_arm_l5 | 1 | mesh×1 | 0 | - |
| R_arm_l6 | 1 | mesh×1 | 0 | - |
| R_arm_l7 | 2 | mesh×2 | 0 | - |
| R_arm_l8 | 1 | mesh×1 | 0 | - |
| R_ee | 0 | - | 0 | **충돌 geom 없음** |
| R_tool | 3 | box×3 | 16 | - |
| lidar | 0 | - | 0 | **충돌 geom 없음** |
| imu | 0 | - | 0 | **충돌 geom 없음** |
| L_base_cam | 0 | - | 0 | **충돌 geom 없음** |
| R_base_cam | 0 | - | 0 | **충돌 geom 없음** |
| F_base_cam | 0 | - | 0 | **충돌 geom 없음** |
| B_base_cam | 0 | - | 0 | **충돌 geom 없음** |
| LF_ultrasonic | 0 | - | 0 | **충돌 geom 없음** |
| RF_ultrasonic | 0 | - | 0 | **충돌 geom 없음** |
| LB_ultrasonic | 0 | - | 0 | **충돌 geom 없음** |
| RB_ultrasonic | 0 | - | 0 | **충돌 geom 없음** |

- 충돌 geom이 **전혀 없는 링크: 19개** — ['B_wheel_l1', 'R_wheel_l1', 'L_wheel_l1', 'arm_center', 'zed_depth_frame', 'zed_left_camera', 'zed_right_camera', 'L_ee', 'R_ee', 'lidar', 'imu', 'L_base_cam', 'R_base_cam', 'F_base_cam', 'B_base_cam', 'LF_ultrasonic', 'RF_ultrasonic', 'LB_ultrasonic', 'RB_ultrasonic']

### 충돌 메쉬 무결성

MuJoCo의 OBJ 로더는 **파일 안의 첫 번째 객체(`o` 블록)만** 읽고, 나머지 면은 경고 없이 버립니다. 공식 collision OBJ 중 5개가 여러 객체로 분할돼 있어 이 함정에 해당합니다. 아래는 소스 OBJ의 전체 삼각형 수와 모델에 실제로 들어간 면 수를 대조한 것입니다.

| 소스 OBJ | 등록 geom 수 | OBJ 삼각형 | 로드된 면 | 로드율 | 사용 링크 | 판정 |
|---|---|---|---|---|---|---|
| B_wheel_l2 | 1 | 19,906 | 19,906 | 100% | B_wheel_l2 | 정상 |
| L_arm_l1 | 1 | 10,092 | 10,092 | 100% | L_arm_l1 | 정상 |
| L_arm_l6 | 1 | 10,402 | 10,402 | 100% | L_arm_l6 | 정상 |
| R_arm_l1 | 1 | 10,108 | 10,108 | 100% | R_arm_l1 | 정상 |
| R_arm_l6 | 1 | 11,208 | 11,208 | 100% | R_arm_l6 | 정상 |
| arm_l2 | 1 | 8,782 | 8,782 | 100% | L_arm_l2, R_arm_l2 | 정상 |
| arm_l3 | 1 | 11,760 | 11,760 | 100% | L_arm_l3, R_arm_l3 | 정상 |
| arm_l4 | 1 | 9,322 | 9,322 | 100% | L_arm_l4, R_arm_l4 | 정상 |
| arm_l5 | 1 | 5,772 | 5,772 | 100% | L_arm_l5, R_arm_l5 | 정상 |
| arm_l7 | 2 | 9,912 | 9,912 | 100% | L_arm_l7, R_arm_l7 | 정상 |
| arm_l8 | 1 | 3,896 | 3,896 | 100% | L_arm_l8, R_arm_l8 | 정상 |
| base | 3 | 11,126 | 11,126 | 100% | base | 정상 |
| head_l1 | 3 | 8,858 | 8,858 | 100% | head_l1 | 정상 |
| head_l2 | 1 | 5,406 | 5,406 | 100% | head_l2 | 정상 |
| head_l3 | 2 | 13,480 | 13,480 | 100% | head_l3 | 정상 |
| torso_l1 | 1 | 3,882 | 3,882 | 100% | torso_l1 | 정상 |
| torso_l2 | 1 | 1,454 | 1,454 | 100% | torso_l2 | 정상 |
| torso_l3 | 9 | 8,006 | 8,006 | 100% | torso_l3 | 정상 |
| wheel_l2 | 1 | 17,260 | 17,260 | 100% | L_wheel_l2, R_wheel_l2 | 정상 |

- **면이 누락된 소스 OBJ: 0개 / 19개**

`scripts/prepare_robot.py`가 각 OBJ의 `o` 블록을 개별 파일 (`assets/collision/<stem>_p<i>.obj`)로 분리해 링크마다 여러 `<collision>`으로 등록합니다. 조각은 전부 **정확히 볼록**(면 평면 이탈 ≤ 0.0001 mm)인 trimesh 볼록 분해 결과라, 조각별로 따로 convex hull을 씌우는 이 방식이 원래 오목 형상을 보존합니다. 하나로 병합하면 전체에 단일 hull이 씌워져 오목부가 메워집니다.

### 충돌 geom ↔ visual geom 바운딩박스 비교

공식 URDF에서 온 링크의 visual geom 수: **0개**

`scripts/prepare_robot.py`가 URDF의 모든 `<visual>` 엘리먼트를 제거한 뒤 `discardvisual=true`로 컴파일하므로, **로봇 링크에는 visual geom이 존재하지 않습니다.** 뷰어에 보이는 로봇 외형은 충돌 메쉬 그 자체입니다. 따라서 이 항목의 20% 비교는 **수행 불가**이며, 동시에 **충돌/외형 불일치가 원리적으로 발생할 수 없다**는 뜻이기도 합니다.

단, 공식 URDF에는 별도의 visual 메쉬가 존재합니다. 비교하려면 `meshes/visual/` 과 `meshes/collision/` 의 OBJ를 직접 대조해야 합니다:

- `meshes/visual/` 존재: True (25개 파일)
- `meshes/collision/` 존재: True (19개 파일)

`visual/`은 `.glb`, `collision/`은 `.obj`로 포맷이 달라 **파일 스템(확장자 제외 이름)으로 대조**했습니다. GLB는 glTF accessor의 `min`/`max`를 노드 변환까지 적용해 읽었고, OBJ는 정점을 직접 파싱했습니다. 두 값 모두 링크 로컬 프레임 기준입니다.

| 메쉬 스템 | visual AABB (m) | collision OBJ 전체 (m) | 로드분 크기비 | 축별 차 % (x/y/z) | 최대절대차 | 판정 |
|---|---|---|---|---|---|---|
| B_wheel_l2 | 0.080 × 0.033 × 0.080 | 0.081 × 0.034 × 0.081 | 100% | +1.0 / +2.4 / +0.8 | +2.4% | 정상 |
| L_arm_l1 | 0.124 × 0.116 × 0.092 | 0.125 × 0.117 × 0.093 | 100% | +1.0 / +1.1 / +1.4 | +1.4% | 정상 |
| L_arm_l6 | 0.133 × 0.081 × 0.079 | 0.135 × 0.082 × 0.080 | 100% | +1.0 / +1.6 / +1.8 | +1.8% | 정상 |
| R_arm_l1 | 0.124 × 0.116 × 0.092 | 0.125 × 0.117 × 0.093 | 100% | +1.1 / +1.1 / +1.4 | +1.4% | 정상 |
| R_arm_l6 | 0.133 × 0.081 × 0.079 | 0.135 × 0.082 × 0.080 | 100% | +1.0 / +1.7 / +1.8 | +1.8% | 정상 |
| arm_l2 | 0.209 × 0.089 × 0.120 | 0.211 × 0.091 × 0.122 | 100% | +1.0 / +2.0 / +1.9 | +2.0% | 정상 |
| arm_l3 | 0.163 × 0.086 × 0.136 | 0.165 × 0.088 × 0.138 | 100% | +1.1 / +2.0 / +1.3 | +2.0% | 정상 |
| arm_l4 | 0.237 × 0.108 × 0.118 | 0.240 × 0.110 × 0.120 | 100% | +1.0 / +2.3 / +1.9 | +2.3% | 정상 |
| arm_l5 | 0.119 × 0.072 × 0.062 | 0.120 × 0.074 × 0.063 | 100% | +1.1 / +1.8 / +2.1 | +2.1% | 정상 |
| arm_l7 | 0.109 × 0.073 × 0.079 | 0.111 × 0.074 × 0.081 | 100% | +1.0 / +1.6 / +1.4 | +1.6% | 정상 |
| arm_l8 | 0.046 × 0.081 × 0.084 | 0.057 × 0.060 × 0.068 | 100% | +24.4 / -26.7 / -18.7 | -26.7% | **차이>=20%** |
| base | 0.690 × 0.550 × 0.350 | 0.697 × 0.558 × 0.357 | 100% | +1.1 / +1.4 / +2.0 | +2.0% | 정상 |
| head_l1 | 0.078 × 0.150 × 0.082 | 0.080 × 0.152 × 0.084 | 100% | +2.1 / +1.1 / +1.8 | +2.1% | 정상 |
| head_l2 | 0.074 × 0.120 × 0.101 | 0.075 × 0.121 × 0.102 | 100% | +1.7 / +1.1 / +1.3 | +1.7% | 정상 |
| head_l3 | 0.067 × 0.131 × 0.107 | 0.068 × 0.132 × 0.108 | 100% | +2.1 / +1.1 / +1.3 | +2.1% | 정상 |
| torso_l1 | 0.567 × 0.247 × 0.253 | 0.573 × 0.253 × 0.258 | 100% | +1.0 / +2.5 / +2.1 | +2.5% | 정상 |
| torso_l2 | 0.561 × 0.142 × 0.251 | 0.567 × 0.148 × 0.257 | 100% | +1.1 / +4.3 / +2.4 | +4.3% | 정상 |
| torso_l3 | 0.254 × 0.341 × 0.572 | 0.259 × 0.347 × 0.578 | 100% | +2.0 / +1.8 / +1.1 | +2.0% | 정상 |
| wheel_l2 | 0.173 × 0.075 × 0.173 | 0.175 × 0.077 × 0.174 | 100% | +1.0 / +2.5 / +0.9 | +2.5% | 정상 |

- **20% 이상 차이: 1개 / 대조 19개**
- collision에만 있는 스템: 없음
- visual에만 있는 스템: ['B_wheel_l1', 'base_cam', 'imu', 'lidar', 'ultrasonic', 'wheel_l1']
- `로드분 크기비` = (MuJoCo가 실제로 로드한 면의 정점 퍼짐) ÷ (OBJ 전체 정점 퍼짐). 퍼짐은 중심으로부터의 최대거리라 **회전에 불변**이므로, MuJoCo가 메쉬를 주축 정렬해 저장하는 것과 무관하게 비교됩니다. **100% 미만 = 충돌 형상이 잘려 있음.**
- 축별 차는 `collision OBJ 전체 − visual`입니다. **음수**면 충돌 형상이 외형보다 작고(파고들 수 있음), **양수**면 큽니다(실제보다 일찍 막힘).

### contype / conaffinity

| (contype, conaffinity) | 로봇 충돌 geom 수 | 의미 |
|---|---|---|
| (2, 7) | 47 | 로봇 본체 — 바닥(1)·토트(4)와 충돌 |

프로젝트의 비트 규약:

| 비트 | 할당 |
|---|---|
| 1 | 환경 (floor, pallet, conveyor) |
| 2 | 로봇 본체 + 후크 |
| 4 | 토트 벽/손잡이 |

두 geom은 `contype & conaffinity` 가 어느 방향으로든 0이 아니어야 충돌 쌍이 됩니다. 로봇(contype=2, conaffinity=7)은 환경·토트·로봇 모두와 쌍이 생기고, 환경(contype=1, conaffinity=7)은 로봇·토트와 생깁니다.

### 자기충돌 제외 쌍 (exclude)

- mjModel `nexclude` = **131**
- SRDF `disable_collisions` 엔트리 = **131**
- 일치: 예

<details><summary>전체 제외 쌍 목록</summary>

| # | body1 | body2 |
|---|---|---|
| 1 | base | B_wheel_l2 |
| 2 | base | R_wheel_l2 |
| 3 | base | L_wheel_l2 |
| 4 | base | torso_l1 |
| 5 | base | torso_l2 |
| 6 | base | torso_l3 |
| 7 | B_wheel_l2 | R_wheel_l2 |
| 8 | B_wheel_l2 | L_wheel_l2 |
| 9 | B_wheel_l2 | torso_l1 |
| 10 | B_wheel_l2 | torso_l2 |
| 11 | B_wheel_l2 | torso_l3 |
| 12 | B_wheel_l2 | head_l1 |
| 13 | B_wheel_l2 | head_l2 |
| 14 | B_wheel_l2 | head_l3 |
| 15 | B_wheel_l2 | L_arm_l1 |
| 16 | B_wheel_l2 | L_arm_l2 |
| 17 | B_wheel_l2 | R_arm_l1 |
| 18 | B_wheel_l2 | R_arm_l2 |
| 19 | B_wheel_l2 | R_arm_l3 |
| 20 | R_wheel_l2 | L_wheel_l2 |
| 21 | R_wheel_l2 | torso_l1 |
| 22 | R_wheel_l2 | torso_l2 |
| 23 | R_wheel_l2 | head_l1 |
| 24 | R_wheel_l2 | head_l2 |
| 25 | R_wheel_l2 | head_l3 |
| 26 | R_wheel_l2 | L_arm_l1 |
| 27 | R_wheel_l2 | L_arm_l2 |
| 28 | R_wheel_l2 | L_arm_l3 |
| 29 | R_wheel_l2 | L_arm_l4 |
| 30 | L_wheel_l2 | torso_l1 |
| 31 | L_wheel_l2 | torso_l2 |
| 32 | L_wheel_l2 | torso_l3 |
| 33 | L_wheel_l2 | head_l1 |
| 34 | L_wheel_l2 | head_l2 |
| 35 | L_wheel_l2 | head_l3 |
| 36 | L_wheel_l2 | R_arm_l1 |
| 37 | L_wheel_l2 | R_arm_l2 |
| 38 | L_wheel_l2 | R_arm_l3 |
| 39 | L_wheel_l2 | R_arm_l4 |
| 40 | torso_l1 | torso_l2 |
| 41 | torso_l1 | torso_l3 |
| 42 | torso_l1 | head_l1 |
| 43 | torso_l1 | head_l2 |
| 44 | torso_l1 | head_l3 |
| 45 | torso_l1 | L_arm_l1 |
| 46 | torso_l1 | L_arm_l2 |
| 47 | torso_l1 | R_arm_l1 |
| 48 | torso_l1 | R_arm_l2 |
| 49 | torso_l2 | torso_l3 |
| 50 | torso_l2 | head_l1 |
| 51 | torso_l2 | head_l2 |
| 52 | torso_l2 | head_l3 |
| 53 | torso_l2 | L_arm_l1 |
| 54 | torso_l2 | L_arm_l2 |
| 55 | torso_l2 | R_arm_l1 |
| 56 | torso_l2 | R_arm_l2 |
| 57 | torso_l3 | head_l1 |
| 58 | torso_l3 | head_l2 |
| 59 | torso_l3 | head_l3 |
| 60 | torso_l3 | L_arm_l1 |
| 61 | torso_l3 | L_arm_l2 |
| 62 | torso_l3 | R_arm_l1 |
| 63 | torso_l3 | R_arm_l2 |
| 64 | head_l1 | head_l2 |
| 65 | head_l1 | head_l3 |
| 66 | head_l1 | L_arm_l1 |
| 67 | head_l1 | L_arm_l2 |
| 68 | head_l1 | L_arm_l3 |
| 69 | head_l1 | R_arm_l1 |
| 70 | head_l1 | R_arm_l2 |
| 71 | head_l1 | R_arm_l3 |
| 72 | head_l2 | head_l3 |
| 73 | head_l2 | L_arm_l1 |
| 74 | head_l2 | L_arm_l2 |
| 75 | head_l2 | L_arm_l3 |
| 76 | head_l2 | R_arm_l1 |
| 77 | head_l2 | R_arm_l2 |
| 78 | head_l2 | R_arm_l3 |
| 79 | head_l3 | L_arm_l1 |
| 80 | head_l3 | L_arm_l2 |
| 81 | head_l3 | L_arm_l3 |
| 82 | head_l3 | R_arm_l1 |
| 83 | head_l3 | R_arm_l2 |
| 84 | head_l3 | R_arm_l3 |
| 85 | L_arm_l1 | L_arm_l2 |
| 86 | L_arm_l1 | L_arm_l3 |
| 87 | L_arm_l1 | L_arm_l4 |
| 88 | L_arm_l1 | R_arm_l1 |
| 89 | L_arm_l1 | R_arm_l2 |
| 90 | L_arm_l1 | R_arm_l3 |
| 91 | L_arm_l1 | R_arm_l4 |
| 92 | L_arm_l1 | R_arm_l5 |
| 93 | L_arm_l2 | L_arm_l3 |
| 94 | L_arm_l2 | R_arm_l1 |
| 95 | L_arm_l2 | R_arm_l2 |
| 96 | L_arm_l2 | R_arm_l3 |
| 97 | L_arm_l2 | R_arm_l4 |
| 98 | L_arm_l3 | L_arm_l4 |
| 99 | L_arm_l3 | L_arm_l5 |
| 100 | L_arm_l3 | L_arm_l6 |
| 101 | L_arm_l3 | L_arm_l7 |
| 102 | L_arm_l3 | R_arm_l1 |
| 103 | L_arm_l3 | R_arm_l2 |
| 104 | L_arm_l3 | R_arm_l3 |
| 105 | L_arm_l4 | L_arm_l5 |
| 106 | L_arm_l4 | L_arm_l6 |
| 107 | L_arm_l4 | L_arm_l7 |
| 108 | L_arm_l4 | R_arm_l1 |
| 109 | L_arm_l4 | R_arm_l2 |
| 110 | L_arm_l5 | L_arm_l6 |
| 111 | L_arm_l5 | L_arm_l7 |
| 112 | L_arm_l5 | R_arm_l1 |
| 113 | L_arm_l6 | L_arm_l7 |
| 114 | L_arm_l6 | L_arm_l8 |
| 115 | L_arm_l7 | L_arm_l8 |
| 116 | R_arm_l1 | R_arm_l2 |
| 117 | R_arm_l1 | R_arm_l3 |
| 118 | R_arm_l1 | R_arm_l4 |
| 119 | R_arm_l2 | R_arm_l3 |
| 120 | R_arm_l3 | R_arm_l4 |
| 121 | R_arm_l3 | R_arm_l5 |
| 122 | R_arm_l3 | R_arm_l6 |
| 123 | R_arm_l3 | R_arm_l7 |
| 124 | R_arm_l4 | R_arm_l5 |
| 125 | R_arm_l4 | R_arm_l6 |
| 126 | R_arm_l4 | R_arm_l7 |
| 127 | R_arm_l5 | R_arm_l6 |
| 128 | R_arm_l5 | R_arm_l7 |
| 129 | R_arm_l6 | R_arm_l7 |
| 130 | R_arm_l6 | R_arm_l8 |
| 131 | R_arm_l7 | R_arm_l8 |

</details>

### 후크(말단) 형상이 토트를 집을 수 있는가

공식 그리퍼가 아니라 `poc_sim/gripper.py`의 자체 제작 후크입니다. **구동 조인트가 없는 고정 형상**이고, '집는다'는 동작은 양손 간격(span) 변화로만 표현됩니다.

| geom | 타입 | 전체 크기 mm (x×y×z) | 소속 body 질량 kg | 마찰 μ | 접촉 옵션 |
|---|---|---|---|---|---|
| L_tool_plate | box | 110.0 × 104.0 × 26.0 | 0.600 | 1.00 | margin=0 gap=0 |
| L_tool_column | box | 40.0 × 94.0 × 158.0 | 0.600 | 1.00 | margin=0 gap=0 |
| L_prong | box | 118.5 × 84.0 × 20.0 | 0.600 | 1.50 | margin=0.004 gap=0.004 |
| R_tool_plate | box | 110.0 × 104.0 × 26.0 | 0.600 | 1.00 | margin=0 gap=0 |
| R_tool_column | box | 40.0 × 94.0 × 158.0 | 0.600 | 1.00 | margin=0 gap=0 |
| R_prong | box | 118.5 × 84.0 × 20.0 | 0.600 | 1.50 | margin=0.004 gap=0.004 |


손잡이 구멍 대 돌기(prong) 치수 적합성:

| 항목 | 구멍 | 돌기 | 여유 | 판정 |
|---|---|---|---|---|
| 높이 mm | 40.0 | 20.0 | +20.0 | 들어감 |
| 폭 mm | 116.0 | 84.0 | +32.0 | 들어감 |

- 삽입 깊이 `ENGAGE_DEPTH` = 60 mm, 벽 두께 `WALL_T` = 2.5 mm → 벽 중심선을 +27.5 mm 지나 걸칩니다.
- span: `SPAN_OPEN` 297.5 mm → `SPAN_HOLD` 217.5 mm (편측 80 mm 삽입 행정)
- 돌기가 얹히는 턱: 손잡이 하단 `HANDLE_BOTTOM` = +78.0 mm (토트 중심 기준, 토트 높이 252 mm)

**운반 안정화는 형상이 아니라 `<equality><weld>`가 담당합니다** (`poc_sim/scene.py:188-196`, 토트당 2개 × 좌우). 돌기 접촉이 확인되면 weld를 활성화하므로, 후크의 마찰·강성은 운반 성패에 관여하지 않습니다.

- 모델 내 weld equality 수: **64** (토트 32개 × 2)

### 충돌 형상 렌더링

- **정면 (+X에서 -X 방향)** — `reports/step1_collision_front.png`  
  (free camera azimuth=180°, elevation=-8°, distance=2.7 m, lookat=(0.05, 0, 0.80), geomgroup 전체 활성)
- **측면 (+Y에서 -Y 방향, 로봇 왼쪽)** — `reports/step1_collision_side.png`  
  (free camera azimuth=270°, elevation=-8°, distance=2.7 m, lookat=(0.05, 0, 0.80), geomgroup 전체 활성)

렌더 방식: **로봇 body의 충돌 geom 47개만** 표시하고 (`contype` 또는 `conaffinity` ≠ 0), 바닥·컨베이어·팔레트·토트와 visual 전용 geom은 알파 0으로 숨겼습니다. 색: **파랑 = 공식 URDF 링크**, **주황 = 프로젝트가 추가한 후크**. 메모리상의 `MjModel` 인스턴스만 바꾸며 `assets/*.xml`은 건드리지 않습니다.

## 5. 카메라

| 카메라 | 부착 body | pos | fovy (°) | quat | mode |
|---|---|---|---|---|---|
| overview | world | [3.8, -4.8, 3.5] | 45.00 | [0.8216, 0.4743, 0.1581, 0.2739] | fixed |
| overhead | world | [1.15, 0.0, 3.5] | 45.00 | [1.0, 0.0, 0.0, 0.0] | fixed |

- 모델 내 MuJoCo `<camera>` 총 **2개**.
- 로봇 링크에 부착된 카메라: **0개**

**헤드 카메라·손목 카메라가 MuJoCo 카메라로 정의돼 있지 않습니다.** 모델의 두 카메라는 모두 `worldbody` 직속입니다:

- `overview`: 보고서용 전경 샷 전용. 인지에 쓰이지 않습니다.
- `overhead`: 팔레트 중심 위 3.5 m 고정. `poc_sim/perception.py`가 이 카메라로 RGB·깊이·세그멘테이션을 받아 구역 높이를 추정합니다. 로봇에 붙어 있지 않으므로 **천장 설치 고정 카메라**를 가정한 구성입니다.

### 공식 URDF가 정의한 센서 프레임

URDF에는 센서 장착 프레임이 **실재합니다**. 질량 0 또는 극소 링크로 들어와 있고, 우리 mjModel에도 body로 로드돼 있지만 **MuJoCo 카메라가 하나도 붙어 있지 않습니다.**

| 센서 프레임 | 부모 링크 | mount xyz (m) | mount rpy (rad) | mjModel | 현 자세 world pos | MuJoCo camera |
|---|---|---|---|---|---|---|
| zed_depth_frame | head_l3 | [0.025, 0.023, 0.0489] | [-1.5708, 0.0, -1.5708] | 로드됨 | [-0.2364, 0.025, 1.5676] | **없음** |
| zed_left_camera | head_l3 | [0.0365, 0.023, 0.0489] | [-1.5708, 0.0, -1.5708] | 로드됨 | [-0.226, 0.025, 1.5625] | **없음** |
| zed_right_camera | head_l3 | [0.0365, -0.027, 0.0489] | [-1.5708, 0.0, -1.5708] | 로드됨 | [-0.226, -0.025, 1.5625] | **없음** |
| lidar | base | [0.2975, 0.0, 0.144] | [0.0, 0.0, 0.0] | 로드됨 | [0.2975, 0.0, 0.1988] | **없음** |
| imu | base | [0.165, 0.0, 0.2071] | [0.0, 0.0, 0.0] | 로드됨 | [0.165, 0.0, 0.2619] | **없음** |
| L_base_cam | base | [0.02, 0.2673, 0.208] | [0.0, 0.0, 1.5708] | 로드됨 | [0.02, 0.2673, 0.2628] | **없음** |
| R_base_cam | base | [0.02, -0.2673, 0.208] | [0.0, 0.0, -1.5708] | 로드됨 | [0.02, -0.2673, 0.2628] | **없음** |
| F_base_cam | base | [0.3373, 0.0, 0.208] | [0.0, 0.0, 0.0] | 로드됨 | [0.3373, 0.0, 0.2628] | **없음** |
| B_base_cam | base | [-0.3303, 0.0, 0.176] | [0.0, 0.0, 3.1416] | 로드됨 | [-0.3303, 0.0, 0.2308] | **없음** |
| LF_ultrasonic | base | [0.3164, 0.1475, 0.1585] | [0.0174, -0.4189, 0.4363] | 로드됨 | [0.3164, 0.1475, 0.2133] | **없음** |
| RF_ultrasonic | base | [0.3164, -0.1475, 0.1585] | [0.0174, -0.4189, -0.4363] | 로드됨 | [0.3164, -0.1475, 0.2133] | **없음** |
| LB_ultrasonic | base | [-0.3163, 0.1475, 0.0789] | [0.0, -0.4538, 2.7053] | 로드됨 | [-0.3163, 0.1475, 0.1337] | **없음** |
| RB_ultrasonic | base | [-0.3163, -0.1475, 0.0789] | [0.0, -0.4538, -2.7053] | 로드됨 | [-0.3163, -0.1475, 0.1337] | **없음** |


URDF 원문의 주석(`vega_1.urdf:334-335`):

> *The zed_left_camera and zed_right_camera is used for simulation rendering. The depth frame i[s] used for the real robot transformation.*

**공식 모델은 `zed_left_camera` / `zed_right_camera`를 시뮬레이션 렌더링용 카메라로 쓰라고 명시하고 있습니다.** 우리 구현은 이 프레임을 쓰지 않고 별도의 천장 카메라를 만들어 썼습니다.

### 스테레오 베이스라인 (URDF 실측)

- `zed_left_camera` mount xyz = [0.0365, 0.023, 0.0489]
- `zed_right_camera` mount xyz = [0.0365, -0.027, 0.0489]
- **베이스라인 = 50.0 mm**
- `zed_depth_frame`은 좌 카메라보다 x로 -11.5 mm 떨어져 있습니다.

### 실제 장비 사양 대조

| 항목 | 확인 내용 | 출처 |
|---|---|---|
| 헤드 카메라 종류 | **스테레오 카메라** (좌/우 + depth 프레임) | URDF 링크 구조 |
| 제조사 계열 | Stereolabs **ZED** 계열 (링크명 `zed_*`) | URDF 링크명 |
| 스테레오 베이스라인 | **50.0 mm** (URDF mount 좌표 실측) | URDF |
| 정확한 모델명 (ZED X Mini 등) | **미확인** | - |
| 해상도 / 프레임레이트 | **미확인** | - |
| FOV 실측값 | **미확인** | - |
| depth 방식 (스테레오/ToF) | **미확인** (스테레오 추정) | - |
| 손목 카메라 | **URDF에 없음** → 미탑재로 추정 | URDF |
| 베이스 카메라 4대 사양 | **미확인** (링크만 존재) | URDF |
| LiDAR 모델 | **미확인** (링크만 존재) | URDF |
| 현장 천장 카메라 사양 | **미확인** | - |

> 베이스라인 50 mm는 URDF 좌표에서 계산한 값입니다. 제품 데이터시트로 교차검증하지 않았습니다.

시뮬레이터 `overhead` 카메라의 `fovy` = **45.0°** (MuJoCo 기본값 45°와 동일). 실제 장비와 대조하지 않은 임의값입니다.

### 머리 관절

| 관절 | mjModel jnt_range | URDF velocity | URDF effort |
|---|---|---|---|
| head_j1 | -1.483 ~ 1.483 | 3.2 | 6 |
| head_j2 | -2.792 ~ 2.792 | 3.2 | 2.5 |
| head_j3 | -1.378 ~ 1.483 | 3.2 | 6 |

머리 관절 3개는 모델에 로드돼 있지만 `poc_sim/ik.py:44-46`의 제어 대상(`torso_j1~3`, `L/R_arm_j1~7` 17개)에 **포함되지 않습니다.** 전 구동 동안 초기값에 고정되어 움직이지 않습니다. 헤드 카메라를 쓰려면 이 관절들로 시선을 돌려야 하므로 함께 묶이는 항목입니다.

## 2단계로 넘어가기 전에 고쳐야 할 것

우선순위 순입니다. P0는 고치지 않으면 2단계 결과를 신뢰할 수 없는 항목입니다.

### P0 — 충돌 메쉬 면 누락 — **해결됨**

전 소스 OBJ 19개의 로드율이 **100%**입니다 (삼각형 180,632개, geom 33개).

`scripts/prepare_robot.py`가 볼록 분해 조각을 `assets/collision/<stem>_p<i>.obj`로 분리해 링크마다 별도 `<collision>`으로 등록합니다. 4장의 무결성 표에서 확인할 수 있습니다.

이 수정으로 **충돌 형상이 실제로 커졌습니다.** 아래 항목은 복구 이후를 기준으로 다시 읽어야 합니다:

- `poc_sim/ik.py`의 장애물 회피 여유(`AVOID_REQUIRE` 14 mm, `AVOID_SAFE` 45 mm)가 더 큰 형상에 대해 평가됩니다.
- 도달 한계와 층수 판정(`docs/requirements.md`의 8층 근거)은 축소된 형상으로 계산된 값이므로 재확인이 필요합니다.

### P1 — 액추에이터 부재 (동역학으로 갈 경우 필수)

`nu=0`, `na=0`, `d.ctrl` 크기 0. URDF의 `effort`(구동 16, 조향 6, 몸통 380~700, 팔 25~150 N·m)가 모델에 들어와 있지만 (`jnt_actfrcrange`) 이를 소비할 액추에이터가 없어 **토크 한계가 무력**합니다. `poc_sim/simulation.py:242-246`이 `mj_step` 뒤에 로봇 `qpos`를 목표값으로 덮어쓰므로 물리 적분 결과도 버려집니다.

2단계가 운동학 범위라면 현 상태 유지도 일관성은 있습니다 (README가 한계를 명시). 다만 그 경우 **토크·슬립·전도에 관한 어떤 수치도 이 모델에서 나올 수 없다**는 점을 결과물에 못박아야 합니다.

### P1 — 조향 조인트 미사용 — **해결됨**

`rolling_wheels()`가 세 바퀴 모두 선회 접선으로 조향한 뒤 굴림비를 재계산합니다. 제자리 선회는 접지점을 베이스 원점 기준 고정 원 위에 두므로 **베이스 좌표계에서 접선 방향이 변하지 않습니다** — 조향각은 상수이고 굴림각만 heading에 비례합니다. 후방 캐스터는 조향축보다 34 mm 뒤에 접지하므로 조향이 접지점을 움직여서, 반복 해법으로 수렴시킵니다.

| 바퀴 | 조향각 | 굴림비 (rad/rad) | 레버암 | 조향 전 횡슬립 | 조향 후 |
|---|---|---|---|---|---|
| `L_wheel` | +0.6881 rad (+39.43°) | -3.3328 | 0.2910 m | 1.110e-01 m/s | **7.7e-17 m/s** |
| `R_wheel` | -0.6881 rad (-39.43°) | -3.3328 | 0.2910 m | 1.110e-01 m/s | **2.8e-17 m/s** |
| `B_wheel` | +1.6543 rad (+94.78°) | +6.4414 | 0.2628 m | 1.792e-01 m/s | **5.9e-17 m/s** |

선회율 0.6 rad/s 기준입니다. `L/R_wheel_j1`의 조향각은 한계 ±2.722 rad 안에 들어오고 `B_wheel_j1`은 continuous라 한계가 없습니다. 위 수치는 운반 자세에서의 해석적 잔차이고, 실제 180° 선회를 돌려 접지점 속도를 유한차분으로 읽으면 수치 잡음이 섞여 5e-7 m/s 수준으로 측정됩니다 — `scripts/verify_steering.py`.

후방 캐스터 `B_wheel_j1`/`B_wheel_j2`는 수정 전에는 구동 목록에 아예 없었습니다. 레버암이 0.299 m로 가장 길어 횡슬립도 가장 컸습니다.

### P2 — 헤드 ZED 카메라 미정의

공식 URDF 주석이 `zed_left_camera`/`zed_right_camera`를 시뮬레이션 렌더링용으로 쓰라고 명시하는데, 모델에는 이 프레임에 `<camera>`가 없습니다. 현재 인지는 천장 고정 `overhead` 카메라로 대체돼 있습니다. 실제 로봇이 자기 머리 카메라로 팔레트를 본다면 **가림(occlusion)·시야각·선회 중 흔들림**이 전부 다르게 작용하므로, 인지 성능을 논하려면 이 프레임에 카메라를 붙이고 `head_j1~j3`로 시선을 돌려야 합니다. 머리 3관절도 현재 IK 제어 대상이 아닙니다.

### P2 — 확인이 필요한 수치

| 항목 | 현재 | 확인할 것 |
|---|---|---|
| 로봇 질량 | 142.11 kg (후크 1.2 kg 포함) | 공개 스펙 135 kg의 출처와 구성(핸드·배터리). 5.3% 차이의 정체 |
| `arm_l8` 충돌 형상 | visual 대비 +24/−27/−19% | 손목 말단 링크. 면 누락은 아니므로 원본 자산 자체의 불일치 |
| 전체 높이 | 최대 신장 1.932 m / 운전 자세 1.566 m | 1.7 m 스펙이 어느 자세 기준인지 |
| `overhead` fovy | 45° (MuJoCo 기본값) | 실제 천장 카메라 화각 |

### P3 — 구조적으로 남겨둘지 결정할 것

- `dof_damping` / `dof_armature` / `dof_frictionloss`가 전 DOF 0. 동역학으로 가면 모터 회전자 관성과 조인트 마찰이 필요합니다.
- `assets/vega.xml`은 `prepare_robot.py` 산출물을 **동결**한 파일입니다. `.deps`의 URDF가 바뀌어도 자동 반영되지 않습니다. 버전 기록(생성 시 URDF 해시)을 남기면 안전합니다.
- `L/R/B_wheel_l1`(조향 포크·캐스터 포크)에 충돌 geom이 없습니다. `visual/`에는 `wheel_l1.glb`, `B_wheel_l1.glb`가 존재하므로 collision OBJ가 애초에 제공되지 않은 것입니다. 바닥 근처라 영향은 작지만 의도된 생략인지 확인이 필요합니다.
