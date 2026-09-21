# 목표 좌표로 향하는 Deorbit 시작 후보 탐색

`Run_Deorbit_Window_Study.m`는 기존 `entry_design.window`와 `target`,
`mission.deorbit`를 연결하는 실행 함수다. 기존 미션 구조의 교체는 필요 없다.

## 바로 실행

```matlab
w = Run_Deorbit_Window_Study();
w.candidates
```

기본 목표는 **36°30'00.0"N 130°30'00.0"E = [36.5 130.5] deg**.
Mission_Config / Mission_Run_Config를 사용해 Phase 1/2를 한 번 실행한 뒤
**Phase 2 종료 상태**를 추출한다. 그 상태에서 기다렸다가 deorbit하는
대안들을 탐색한다. 이미 계산한 미션이 있으면 재사용한다.

```matlab
% Main_Mission_Simulator가 생성한 mission_result도 사용 가능하다.
w = Run_Deorbit_Window_Study(mission_result);
```

직접 준비할 때는 `Run_Mission(struct(),struct('stop_after_proximity',true))`로
Phase 2까지만 계산할 수 있다. 따라서 기본 deorbit가 실패하는 설정에서도
deorbit 전의 상태를 얻어 다른 시작 시각을 탐색할 수 있다. 기존
`Run_Mission()`과 `Main_Mission_Simulator`의 전체 실행 동작은 유지된다.

기본 설정:

- 기체: 해당 미션의 `reentry_vehicle.vehicle_mode`.
- 탐색 범위: Phase 2 종료 후 0~24시간, 600초 간격. 먼저 zero-bank 대표
  궤적으로 계산한다(MaxStep 30초, RelTol 1e-6의 빠른 선별 계산).
- 추가 탐색: 거리가 가까운 국소 최소점 최대 2개 주변에서 1차원 시간
  최소화를 최대 12회 평가한다. 각 중심의 ±30초를 10초 간격으로
  원래 정확도로 다시 계산하고 여러 bank profile 및 국소 최적화를 적용한다.
- 정밀 적분은 MaxStep 20초, RelTol 1e-8이며, 각 시각의 bank 최적화는
  기본 최대 24회 추가 적분을 허용한다. `o.entry.max_step_s`와
  `o.entry.target_max_evaluations`로 계산 비용과 탐색 강도를 변경한다.
- 판정: 고도 **20 km**에서 목표 위경도와의 지표 대원거리 **10 km 이내**.
- deorbit: 미션의 확정된 deorbit 설정을 그대로 사용한다. Impulsive 및
  apogee-split finite 설정이 같은 인터페이스로 작동한다.

## 목표, 시간 범위, 정확도 변경

```matlab
o.target_latlon_deg = [36.5 130.5];
o.delays_s = 0:300:48*3600;
o.refinement_step_s = 20;
o.refinement_basins = 8;
o.entry.target_tolerance_m = 1000;
o.vehicle_mode = "CAPSULE"; % 또는 "SPACEPLANE"
w = Run_Deorbit_Window_Study(mission_result,o);
```

`refinement_basins=0`이면 지정한 grid의 모든 시각을 원래 정확도와 bank
최적화로 검사하며, 시간 정밀화는 생략한다. 작은 `delays_s`로 먼저
실행해 설정을 확인할 수 있다. 임의의 위경도는 십진수 도 단위이며
남위/서경은 음수다. 기체 형상과 질량은 기존 config에서 읽는다.

## 결과와 시각의 의미

- `w.candidates`: 실제 적분 궤적으로 도달이 확인된 행만 포함.
- `w.table`: 실패/미확인까지 포함한 모든 시각과 miss distance.
- `SCREEN_ONLY`: 빠른 zero-bank 선별만 한 시각. 도달 후보로 인증하지
  않는다. 선별에서 허용오차 안에 들어온 시각도 원래 정확도로 재검사한다.
- `w.witnesses{k}`: `w.table(k,:)`에 대응하는 최선의 실제 궤적,
  bank profile, 열/하중 진단. 도달 성공 여부는 `Status`를 확인한다.
- `Delay_s`: Phase 2 종료 후 deorbit 스케줄을 시작하기까지의 대기시간.
- `ScheduleStartMission_s`: 미션 t=0 기준 스케줄 시작 시각.
- `FirstBurnMission_s`: 미션 t=0 기준 **실제 첫 점화 시각**.
  NEXT_APOGEE 방식이면 스케줄 시작보다 늦을 수 있다.
- `w.samples(k).ignition_times_s`: 다회 연소를 포함한 모든 실제 점화 시각.
- `EntryMission_s`: 120 km 등 설정된 entry interface에 도달한 미션 시각.
- `EndpointLatitude_deg`, `EndpointLongitude_deg`: 계산 종료점 위경도.
  실패나 timeout 행의 종료점은 도달 증거가 아니다.

저장 위치는 `output/deorbit_window/`이며, `candidates.csv`,
`all_samples.csv`, 전체 궤적을 포함한 `window_results.mat`,
`window_search.png`가 생성된다. 같은 디렉터리로 재실행하면 갱신된다.
다른 실험은 `o.output_dir`로 구분할 수 있다.

## 지구 자전과 날짜

기본값은 기존 Mission_Config의 미션 t=0 Greenwich angle을 그대로 따른다.
Phase 1/2 경과시간, 대기시간, deorbit 및 재진입 시간을 모두 합해
ECI 위치를 지구고정 위경도로 바꾼다. 기본 결과는 **미션 상대시간**이다.
현실의 날짜를 자동으로 추측하지 않는다.

실제 UTC 시작시각에 연결하려면 **미션 t=0의 UTC**를 지정한다.

```matlab
o = struct();
o.mission_epoch_utc = datetime(2026,9,21,0,0,0,'TimeZone','UTC');
w = Run_Deorbit_Window_Study(mission_result,o);
% 결과에 FirstBurnUTC 열이 추가된다.
```

이는 저장된 ECI 초기상태에 날짜를 부여하는 것이다. 기존 미션 결과의
통신 해석 등을 새 날짜로 다시 계산하지는 않는다. 실제 궤도 자료는 같은
ECI 기준과 시각으로 변환해야 한다. 변환은 기존 근사 GMST와 구형지구
모델을 쓰므로 지도상의 WGS84 측지좌표에 대한 정밀 항법 모델은 아니다.

## 해석 범위

1. 후보는 계산한 **이산 시각들의 집합**이다. 인접 후보 사이 전 구간을
   가능하다고 보간하지 않는다. Coarse scan과 국소 정밀화는 좁은 기회를
   놓칠 수 있다. 특히 zero-bank 대표궤적으로 고른 통과가 모든 유효한
   bank 기동의 기회를 대표하지는 않는다. `UNRESOLVED_BY_SEARCH`는
   불가능 증명이 아니다. 전체 지정 grid를 검사하려면 `refinement_basins=0`을 쓴다.
2. 고정된 deorbit 법칙 아래 시간과 bank profile을 탐색한다. Delta-V,
   추진 방향, 궤도면까지 동시에 최적화한 전체 가능 영역이 아니다.
3. 기다리는 동안은 자유비행이다. 도킹 상태, station keeping, 분리 기동,
   근접 비행 충돌 회피를 새로 가정하거나 검증하지 않는다.
4. 기본 목표는 해당 좌표 **20 km 상공**이다. `terminal_altitude_m=0`을
   설정하면 현재 공력모델의 지면 교차점까지 연장할 뿐이며, 캡슐 낙하산,
   바람에 의한 하강 표류, spaceplane flare/활주로 착륙은 모델링하지 않는다.
5. 열/하중 한계의 기본값은 무한대다. `o.entry.max_dynamic_pressure_Pa`,
   `max_g_load`, `max_heat_flux_W_m2`, `max_heat_load_J_m2`에 유효한 기체
   한계를 지정해야 그 제약을 만족하는 궤적만 후보로 인정된다.
   통신/항행/운용구역 제약과 불확실성은 이 후보 판정에 포함되지 않는다.

검증 실행:

```matlab
addpath('validation');
Validate_Deorbit_Window_Study;
Validate_Entry_Design;
```

확인 항목은 알려진 도달 궤적의 후보 복원, 시간 원점과 지구각의 일관성,
UTC 변환, 미도달 결과 처리, 파일 출력 및 기존 finite-burn/entry 검증이다.

## 2026-09-21 기본 목표 실행 결과

현재 CAPSULE 설정으로 `[36.5 130.5]`를 실행했다. 미션 시작 날짜를 따로
부여하지 않고 기존 Greenwich angle=0 convention을 사용했다. 설정은 AUTO가
선택한 `impulsive_drag-off_h300.0-500.0_phase90.0_31c1e45c_410e0069_20260630_134559Z`
사례이며, deorbit는 기존 impulsive 설정이다.

| 항목 | 결과 |
| --- | --- |
| 1차 탐색 | 0~24시간, 145개 시각의 zero-bank 선별 |
| 정밀 탐색 | 선택한 2개 통과 구간의 14개 시각 |
| 허용오차 / 도달 고도 | 10 km / 20 km |
| 확인된 도달 후보 | 0개 |
| 최선의 시작 지연 | Phase 2 종료 후 9215.0967초 = 153.584945분 |
| 실제 점화 시각 | 미션 t=0 기준 52891.8284초 |
| 최소 목표 오차 | 248197.203 m |
| 그 궤적의 종료점 | 35.858916° N, 127.854301° E |
| 선택된 bank profile | [8.9997, 26.3899, 36.3327] deg |
| 최대 적분 간격 20→5초 재검증 | 도달점 변화 0.1763 m |

이 결과는 **이번 탐색에서 찾지 못했다**는 뜻이다. 전역 불가능 판정이
아니며, 특히 각 시각의 국소 최적화 평가 예산이 24회로 제한되어 있다.
도달 후보가 필요하다고 해서 허용오차나 초기 지구각을 임의로 바꾸지는 않았다.
추가 탐색에는 `refinement_basins`, `delays_s`, bank profile과 평가 예산을
명시적으로 바꾸고 결과를 비교할 수 있다.

실행 결과는 `output/deorbit_window/`에 저장했고 `verification.json`은
최선 사례의 재적분 비교값을 기록한다. 알려진 도달 궤적에 대해서는 별도
검증이 실제 후보를 복원했다. 좌표/UTC/실패 처리 검증, 기존 entry design
검증 및 전체 132개 MATLAB 파일 Code Analyzer 검사를 통과했다.
