# 사용 설명서

## 실행 환경과 기본 실행

MATLAB Current Folder를 `Rendezvous_and_Reentry` 루트로 설정합니다. Root와 필요한 validation 폴더만 path에 추가합니다. `addpath(genpath(...))`는 `legacy` 함수가 활성 함수를 가릴 수 있어 사용하지 않습니다. 기본 경로에는 Python optimizer, Simulink, GPOPS-II가 필요하지 않습니다.

```matlab
Main_Mission_Simulator
```

기본값은 Apollo capsule을 탑재한 프로젝트 chaser의 300→500 km orbital mission입니다.
Phasing/homing, proximity, 30 m standoff까지 실행하고 dashboard를 그립니다.
Deorbit, entry, touchdown까지 수행한 결과는 아닙니다. Python이나 optimizer JSON도
필요하지 않습니다.

Workspace에서 확인할 값:

```matlab
Budget
mission_result.proximity.reached_standoff
mission_result.proximity.final_position_error
mission_result.proximity.control.max_force_N
mission_result.interfaces.phase1_to_phase2
```

`mission_result`가 전체 결과입니다. `hist_p1`, `hist_p2`는 편의 변수이며,
실행하지 않은 `hist_p3`, `hist_reentry`, `X_entry_interface` 등은 `[]`입니다.

## 설정 위치와 API

| 설정 | 위치 |
|---|---|
| 실행 범위 | Main의 `simulation_scope` |
| 궤도 높이·기본 물성 | `Mission_Config.m` 또는 API `options.system` |
| Proximity, integration, correction, entry 실행 옵션 | `Mission_Run_Config.m` 또는 API override |
| Reference 데이터 | `+reference_vehicle` — 단순 실행을 위해 수정할 필요 없음 |

Main은 `python_config.mode="NONE"`, `phase1.mode="HOHMANN"`,
`hohmann_method="NOMINAL_TARGET"`를 선택합니다. 이 세 항목은 `Mission_Run_Config.m`
값보다 우선하고, 나머지 설정은 해당 파일을 따릅니다. 비교 실험은 override를 권장합니다.

```matlab
settings = struct();
settings.phase2.autonomous.approach_mode = '+V';
result = Run_Nominal_Orbit(settings, struct('plot',true));
```

모드는 `'+R'`, `'-R'`, `'+V'`, `'-V'`입니다. R은 radial outward, V는 transverse
forward이며 부호는 chaser가 위치한 쪽입니다. 기본 −R은 target 아래에서 접근합니다.

```matlab
horus = Run_Nominal_Orbit(struct(),struct('preset','HORUS_2B','verbose',false));
ard = Run_Nominal_Orbit(struct(),struct('preset','ARD','verbose',false));
```

이 역시 orbital-only 실행이며 해당 vehicle의 전체 descent를 검증한 것은 아닙니다.

## Orbit correction

```matlab
settings = struct();
settings.phase1.correction.enabled = true;
settings.phase1.correction.corrections_enabled = true;
settings.phase1.correction.seed = 42;
sys = Mission_Config();
stack_mass = sys.Chaser_Mass_Init + sys.reentry_vehicle.capsule.mass_kg;
% 실험용 allowance: 기존 4800 kg stack의 400 kg 비율 적용
settings.phase1.correction.max_propellant_kg = 400 * stack_mass / 4800;
result = Run_Nominal_Orbit(settings,struct('verbose',false));
```

기본 400 kg correction allowance는 Apollo nominal Phase 1에도 부족합니다.
위 allowance는 실험 가정이지 실물 propellant capacity가 아닙니다.
Perfect state knowledge, 기본 σr=20 m/axis, σv=0.02 m/s/axis,
burn gain σ=0.2%, pointing-vector σ=0.0002 rad를 사용합니다.

`enabled=true`, `corrections_enabled=false`이면 동일한 disturbed nominal을 correction
없이 실행합니다. Handoff에 실패하면 일반 runner는 다음 phase를 실행하지 않고 오류를
냅니다. 실패 case의 partial history까지 비교하려면 evaluation driver를 사용합니다.

## Standalone Apollo entry

실행 가능한 open-loop baseline입니다. 실제 upstream state를 사용한 integrated
mission이 아니라 초기 조건을 직접 지정하는 연구입니다.

```matlab
sys = Mission_Config();
sys.environment.atmospheric_drag.use_matlab_atmosisa = false;
vehicle = entry_design.vehicle(sys,'CAPSULE');
c = entry_design.defaults();
c.max_time_s = 1800;
c.max_step_s = 2;
c.relative_tolerance = 1e-9;
c.terminal_altitude_m = 7620;
c.initial_bank_deg = 30;
% lat/lon [deg], altitude [m], air-relative speed [m/s], FPA [deg], heading [deg]
x = entry_design.initial_state(sys,[25 40],121920,7400,-2,70,c);
entry = entry_design.propagate(sys,vehicle,x,[30 30 30],c);
plot(entry.time_s,(vecnorm(entry.state(:,1:3),2,2)-sys.Re)/1000);
xlabel('Entry time (s)'); ylabel('Altitude (km)'); grid on;
```

FPA sweep은 `initial_state`의 `-2`만 바꿉니다. Negative FPA는 descent입니다.
AoA는 Apollo trim schedule을 사용하므로 arbitrary constant AoA를 넣으면 오류가 납니다.
Bank와 AoA는 다른 값입니다. FPA에 맞추려고 mass·aero coefficient를 변경하지 않습니다.
Endpoint 도달과 recovery 성공도 구분해야 합니다.

## Full mission과 오류 구분

Main의 `simulation_scope`를 `"FULL_MISSION"`으로 바꾸면 같은 nominal design으로
deorbit와 entry까지 시도합니다. 현재 Apollo의 실제 handoff는 약 Mach 28.79여서
table 상한 27.72에서 막힙니다. 입력 실수가 아니라 현재 모델의 한계입니다.

| 오류 또는 증상 | 의미 / 조치 |
|---|---|
| Python artifact lacks a verified physics contract | 예전 JSON의 호환성이 확인되지 않음. Main/Run_Nominal_Orbit 사용. 무조건 legacy replay로 우회하지 않음 |
| AoA query outside ... domain / reference_vehicle:Domain | 실제 query가 aero/profile 범위를 초과함. 현재 full mission의 예상 제한 |
| reference_vehicle:TrimOnly | Apollo trim table에 arbitrary AoA를 적용하려 함 |
| PROPELLANT_LIMIT | 설정한 allowance로 maneuver를 수행할 수 없음. 실물 capacity와 구분해 예산 확인 |
| Handoff / correction failure | 해당 candidate가 tolerance/resource 조건을 만족하지 못함. 모든 mission의 물리적 불가능성을 의미하지 않음 |

인자 없는 `Run_Mission()`은 기존 AUTO/archive 설정을 읽는 낮은 수준의 API입니다.
처음 실행할 때는 Main 또는 `Run_Nominal_Orbit`를 사용하십시오. 전체 경로를
API로 명시하려면 다음과 같습니다.

```matlab
s = struct();
s.python_config.mode = 'NONE';
s.phase1.mode = 'HOHMANN';
s.phase1.hohmann_method = 'NOMINAL_TARGET';
% 현재 reference에서는 entry domain error가 발생할 수 있음.
full_result = Run_Mission(s,struct('plot',false));
```

## 검증·평가 재현

```matlab
addpath('validation');
checks = Run_All_Validations();

addpath('docs/evaluation_stage_2026-09-22');
results = evaluate_simulator();
results = finalize_evaluation();
```

결과와 plots는 [평가 및 검증](ANALYSIS_AND_VALIDATION.md)에 있습니다. Evaluation driver는 저장된 evidence 경로에 결과를 쓰므로 원본과 별도로 실행 결과를 보관할 때는 실행용 복사본과 output 경로를 먼저 분리합니다. Source manifest는 해당 evidence를 만든 코드의 식별자입니다.


## Footprint와 target 연구

앞의 standalone 예제에서 구성한 `sys`, `vehicle`, `x`, `c`를 재사용합니다. 공력 범위 안의 초기 조건과 종료점을 유지해야 합니다.

```matlab
fp = entry_design.footprint(sys,vehicle,x,c);
```

Footprint는 선택한 bank profile들의 endpoint 표본입니다. 표시한 hull 전체가 도달 가능 영역이라는 뜻은 아닙니다. Target 탐색은 `entry_design.target`, 고정 policy의 오차 전파는 `entry_design.dispersion`을 사용합니다. 함수 입력은 각 함수의 signature를 따르며, numerical search limit과 물리적 acceptance를 별도로 설정합니다. Target마다 bank profile을 다시 설계한 결과를 fixed-policy robustness로 집계하지 않습니다.

`Run_Footprint_Study`는 두 vehicle에 공통 초기 조건을 주는 연구용 wrapper입니다. 기본 초기 speed와 최신 reference의 조합이 공력 범위를 벗어날 수 있어 모든 preset의 실행 예제로 사용하지 않습니다. Terminal altitude를 0으로 바꾸어도 공력 범위가 확장되거나 landing model이 생기지 않습니다.

## Deorbit window와 분할 burn

`Run_Deorbit_Window_Study(mission_result, options)`는 Phase 2 종료 결과를 받아 대기시간 후보마다 실제 coast/deorbit/entry를 계산합니다. Candidate별로 entry policy를 찾으므로 design sweep입니다. 현재 reference의 nominal EI는 공력 범위 밖이어서 기본 full window를 성공 사례로 사용할 수 없습니다.

Default grid는 0–86400 s의 600 s 간격이며 coarse screening 뒤 제한된 basin refinement를 사용합니다. 기본값으로 큰 연구를 시작하기보다 유효한 vehicle/state를 확인하고 candidate delay, refinement 횟수와 output directory를 명시합니다. 기본 target은 위도/경도 36.5°/130.5°, endpoint는 고도 20 km에서 target distance 10 km 이내입니다. 그 위치는 touchdown target이 아닙니다.

결과 `candidates.csv`, `all_samples.csv`, `window_results.mat`, `window_search.png`는 기본 `output/deorbit_window`에 기록됩니다. 같은 경로는 덮어쓸 수 있으므로 `options.output_dir`를 case별로 지정합니다. `Delay_s`는 Phase 2 종료 기준, `ScheduleStartMission_s`는 schedule 시작, `FirstBurnMission_s`는 실제 첫 burn 시각입니다. `NEXT_APOGEE`에서는 두 시각이 다릅니다. `EntryMission_s`는 누적 mission time이며 UTC는 epoch를 지정한 경우에만 의미가 있습니다. 대기 중 target과의 docking/stationkeeping은 모델링하지 않습니다.

분할 burn 설정은 `phase3.apogee_burns`에 있습니다. `enabled`, `total_delta_v_m_s`, `fractions`, `thrust_N`, `isp_s`, `max_burn_duration_s`, `cooldown_s`, `first_burn`을 사용하며 정확한 schema는 `mission.apogee_burn_defaults()`로 확인합니다. 수동 budget 실행 기능이고 요청 FPA나 thermal limit에 맞춘 자동 최적화가 아닙니다.

## 논문 식·자료 검사

```matlab
report = Run_Paper_Reproduction_Suite();
zhang = paperstudies.zhang.run();
saito = paperstudies.saito.run();
```

Default는 빠른 source/equation/state 검사이며 trajectory optimizer를 실행하지 않습니다. `Run_Paper_Reproduction_Suite(struct('forward',true))`는 명시한 surrogate model의 open-loop propagation을 추가합니다. 반환한 assumptions/status를 함께 읽어야 하며 Apollo/HORUS의 nominal mission과 동일 case로 취급하지 않습니다.

Saito의 `eq17_reference_range` 등 equation helper는 공개된 대수식 단위의 API입니다. `CONVENTION_REQUIRED`는 distance redimensionalization 또는 gain이 확정되지 않았다는 뜻입니다. Helper를 연결하는 것만으로 논문의 guidance controller가 되지 않습니다.

## 선택적 Python 및 legacy 비교

Nominal 실행이 목적이면 Python은 필요하지 않습니다. Python 설계 도구의 CLI 옵션은 각 파일의 `--help`로 확인하고 iteration/runtime을 제한합니다. Export JSON을 사용할 때는 `python_config.mode='FILE'`와 `python_config.file`로 artifact를 고정합니다. 검증된 physics contract가 없다는 오류를 무조건 `allow_legacy_replay=true`로 우회하지 않습니다.

`phase1.hohmann_method='GRID_SEARCH'`는 이전 bounded-search 비교, `phase2.mode='LEGACY_IMPULSIVE'`는 cycloid/hops 비교입니다. Nominal의 빠른 targeting과 같은 runtime·성공 범위를 보장하지 않습니다. `MULTI_HOHMANN`은 preliminary 경로이며 검증된 추가 orbit-correction strategy가 아닙니다.

Python I/O 회귀 검사:

```powershell
python -m unittest discover -s validation -p 'test_*.py'
```

설정 필드와 단위는 [설정 명세](CONFIGURATION_REFERENCE.md), 모델 범위는 [기체 모델](VEHICLE_MODELS.md), 반환 지표와 실패 해석은 [평가 및 검증](ANALYSIS_AND_VALIDATION.md)를 따릅니다.
