# 분할 Deorbit / Footprint 개발 보고

## 결정과 구현 범위

기존 프로젝트 안에 **독립 `+entry_design` 계산 패키지**를 추가했다.
중력·대기·공력은 기존 `reentry_core`를 재사용한다. Mission 실행기에 footprint 탐색 루프를
직접 넣지 않아, 기체 교체·반복 최적화·추후 항법 실험을 별도로 수행할 수 있다.
외부 프로그램이나 Simulink는 필수가 아니다.

| 요청 | 구현 |
|---|---|
| 여러 원지점에서 finite burn | 옵션으로 추가. duration cap, cooldown, 연료 하한, 횟수·전체 시간 제한 |
| Spaceplane / capsule footprint | 같은 solver에 서로 다른 mass/shape를 주입. bank schedule과 bank-rate 제한 적용 |
| 특정 목표점 도달 여부 | 실제 궤적 witness 탐색, 선택적 bank profile 국소 최적화 |
| Deorbit window | 각 대기시간마다 궤도 → deorbit → 실제 entry 상태/시각 → 목표점 탐색을 다시 계산 |
| Landing dispersion | 초기 ECI 상태 공분산 및 대기·Cd·L/D 변동의 seeded 고정-policy 실험 |

**범위 구분:** 기본 endpoint는 고도 20 km이다. 현재 기체 모델에 runway flare, 착륙장치,
낙하산과 지상 바람 모델이 없으므로 이것을 touchdown footprint로 표시하지 않는다.
`terminal_altitude_m=0`을 지정하면 공력 모델을 지면까지 연장한 *ground-intersection surrogate*를
계산할 수 있지만, 실제 착륙 성능 예측은 아니다. 기존 mission의 capsule parachute-speed 종료
설정과 독립적이며, 이 계산기는 명시적으로 지정한 고도에서 종료한다.

## 1. 분할 deorbit

NASA 공식 회고에서 HTV-1의 세 번의 deorbit burn을 확인했다.
그러나 **열 한계가 그 직접 원인이었고 모든 burn이 apogee였다는 근거는 확인하지 못했다.**
이번 기능은 사용자가 요청한 apogee-start 전략이며 HTV 비행 이력 재현으로 주장하지 않는다.
[NASA HTV-1 회고](https://www.nasa.gov/history/15-years-ago-japan-launches-htv-1-its-first-resupply-mission-to-the-space-station/)

기존 기본 모드는 그대로다. 다음과 같이 선택한다.

```matlab
o.phase3.apogee_burns.enabled = true;
o.phase3.apogee_burns.total_delta_v_m_s = 150;
o.phase3.apogee_burns.fractions = [0.2 0.2 0.6];
o.phase3.apogee_burns.thrust_N = 300;
o.phase3.apogee_burns.isp_s = 200;
o.phase3.apogee_burns.max_burn_duration_s = 900;
o.phase3.apogee_burns.cooldown_s = 600;
o.runtime.allow_environment_overrides = false;
r = Run_Mission(o, struct('verbose', false));
disp(struct2table(r.deorbit.info.burns))
```

900초와 600초는 실제 추진기 규격이 아닌 예제 설정이다. 열 상태를 계산하지 않고
최대 연속 연소시간과 재점화 대기시간을 제약으로 표현한다. 한 배분량이 duration cap을 넘으면
남은 양을 이후 apogee에서 이어서 연소한다. 대기 도중 먼저 entry에 진입하거나 연료·횟수·시간
제약을 어기면 실패로 반환한다. Burn 중 entry에 도달하면 연소를 종료하고 실제 투입 ΔV를 기록한다.

첫 burn은 기본 `first_burn="CURRENT"`이다. 원궤도에서는 apogee가 유일하지 않기 때문이다.
타원궤도의 다음 apogee부터 시작하려면 `"NEXT_APOGEE"`를 사용한다.
후속 burn은 냉각 대기 후 radial velocity가 양→음으로 바뀌는 사건에서 시작한다.
연소 중심을 apogee에 맞추는 방식은 아니다.

500 km 시험에서 30 / 30 / 90 m/s를 3회 실행했고, 연소시간은 약 198.48 / 195.47 / 568.77초였다.
후속 시작 radial velocity는 수치적으로 0이었다. 120 km entry의 실제 FPA는 약 −1.933°였다.
추가 120 m/s·300초 cap 시험에서는 요청한 2개 배분량을 더 많은 apogee burn으로 나눠 완료했다.
기존 전체 임무에 연결한 분할 burn 실행도 완료했다.

**이 옵션은 수동 ΔV budget의 실행기다. 기존 single-burn JSON이나 지정 FPA에 자동으로
재최적화하는 기능은 아니다.** FPA와 entry 위치·속도·시각은 실제 전파 결과로 기록되며 window
계산에도 이 상태를 넘긴다. Phase 1 burn mode와도 독립적으로 선택된다.

## 2. Footprint, 목표점, 시각 탐색

Footprint는 가능한 control 선택에 따른 도달영역, dispersion은 하나의 policy에서 오차 때문에
흩어지는 영역으로 분리했다. 여러 bank 크기와 reversal profile을 전파하며, 힘 방향은 기존
lift-direction 모델을 사용한다. Bank는 rate limit과 response time을 가지는 동적 상태다.
동압·가속도·열유속·열량 제한을 지정하면 위반한 궤적을 제외한다.

현재 기본 그림은 **열·동압·하중 제한을 무한대로 둔 운동학적 연구 envelope**다.
이는 기존 논문 한계값과 사용 중인 열 surrogate의 비호환성을 숨기지 않기 위한 명시적 선택이다.
실제 설계에는 검증한 한계값을 설정해야 한다. 기본 제어 표본 9개는 조밀한 경계 최적화가 아니다.
회색 convex hull은 그림을 위한 것이며, 내부 모든 점이 도달 가능하다는 증명이 아니다.

`entry_design.target`은 목표까지의 실제 종점 오차가 허용 반경 안에 들고 설정된 경로제약을
통과한 경우에만 `REACHABLE_WITNESS`를 반환한다. 해를 못 찾으면 `UNRESOLVED_BY_SEARCH`다.
“물리적으로 도달 불가능”이라고 단정하지 않는다. `refine=true`는 제한된 bank parameter를
종점 민감도의 유한차분과 bounded differential correction으로 조정하며 전역 최적화가 아니다.
허용 반경은 기본 10 km로, pinpoint 요구가 생기면
`target_tolerance_m`를 줄여서 다시 검증해야 한다.

`entry_design.window`는 입력한 시간 격자의 검증된 기회를 반환한다. 중간에 계산하지 않은
시각까지 연결해 연속 window라고 주장하지 않는다. 관심 경계에서는 격자를 좁히거나
`refine=true`로 bank 탐색을 강화해야 한다. Capsule은 분리 후 질량을, spaceplane은 deorbit 후
남은 stack 질량을 사용한다. 다른 임무는 `vehicle.deorbit_mass_policy`를 명시적으로 바꾼다.

지구 자전과 entry까지 경과한 시간을 ECI→ECEF 변환에 반영한다. 위경도는 구면 지구의
geocentric 값이며, downrange/crossrange는 entry 지점과 진행방향을 기준으로 한 구면 좌표다.
UTC와 연결하려면 `entry_design.earth_angle(epochUTC)`로 시작 회전각을 지정할 수 있다.
이는 UTC≈UT1인 근사 GMST이고 EOP·nutation을 포함한 정밀 좌표변환은 아니다.
[USNO sidereal-time 설명](https://aa.usno.navy.mil/faq/GAST)

## 3. 실행과 파라미터 교체

```matlab
% 두 기체의 동일 entry 조건 비교와 그림/JSON 생성
result = Run_Footprint_Study();

% 임의 기체/entry 상태로 독립 실행
sys = Mission_Config();
vehicle = entry_design.vehicle(sys, "CAPSULE");
vehicle.mass_kg = 100;             % 예: 차후 기체값으로 교체
vehicle.shape.reference_area_m2 = 0.8;
vehicle.shape.nominal_ld = 0.3;
c = entry_design.defaults();
state = entry_design.initial_state(sys,[0 0],120e3,7500,-3,90,c);
fp = entry_design.footprint(sys,vehicle,state,c);

% 계산된 표본 한 곳을 목표로 삼는 실행 가능한 예
idx = find(fp.feasible,1);
site = fp.latlon_deg(idx,:);
answer = entry_design.target(sys,vehicle,state,site,c,true);

% 분산: 항법 filter가 아닌 초기 상태/환경 불확실성 실험
u = struct('trials',20,'seed',42,'density_coefficient_of_variation',0.05);
spread = entry_design.dispersion(sys,vehicle,state,[0 0 0],c,u);
```

실제 mission entry를 사용할 때는 `r.entry.initial_state(1:6)`, 해당 기체 질량과
`c.entry_epoch_s=r.entry.mission_start_time`을 함께 넘긴다.
기체 adapter는 기존 `Mission_Config`를 읽는 편의 함수일 뿐, solver 내부에는 capsule/spaceplane
종류별 궤적 공식 분기가 없다. 질량·면적·Cd·L/D·AoA schedule·공력 다항식을 교체할 수 있다.

Window API의 순서는 다음과 같다.

```matlab
windows = entry_design.window(sys, Xc14, Xt6, vehicle, ...
    candidate_delays_s, [target_latitude target_longitude], c, deorbit_params, true);
% deorbit_params = r.config.deorbit 등을 사용.
% c.entry_epoch_s는 Xc14/Xt6가 정의된 시각; candidate_delays_s는 그 이후 대기시간.
% 실제 ignition epoch는 samples.ignition_times_s에 기록됨.
```

## 검증과 현재 결과

- 원지점 사건, cooldown, duration cap에 따른 추가 분할, burn-count 실패, 질량/ΔV 장부 확인.
- 20 km endpoint 사건과 bank-rate 제한, 경로제약 위반 제외 확인.
- 적분 정밀도 강화 시 capsule endpoint 차이 약 0.047 m.
- L/D=0에서 bank 부호를 바꿔도 궤적이 같아지는 물리적 성질 확인.
- 알려진 목표는 witness로, 반대편 목표는 미해결로 판정.
- 초기 bank library에 없는 목표점도 20회 추가 전파로 약 24.7 m 오차에 도달(시험 허용 반경 100 m).
  완전한 상태 정보와 같은 모델을 사용하는 수치 시험이며 실제 착륙 정확도 예측은 아니다.
- 분산 seed 재현성과 호출자 RNG 보존 확인.
- 0 / 120 / 240초 대기 후 분할 deorbit+entry를 실제로 계산해 첫 시각의 목표 도달을 검증.
- 기존 MATLAB 회귀 검사와 분할 deorbit를 포함한 전체 임무 실행 통과.

동일한 예제 entry(120 km, 대기상대 7.5 km/s, FPA −3°, 적도 동향), 20 km endpoint,
기본 9개 bank 표본과 무제약 envelope에서 다음 차이가 나왔다.

| 기체 | 표본 downrange | 최대 표본 절대 crossrange |
|---|---:|---:|
| 현재 spaceplane, 2,000 kg | 약 4,479–11,596 km | 약 1,570 km |
| 현재 capsule, 60 kg | 약 1,704–2,255 km | 약 100 km |

이는 현 파라미터의 연구 계산 결과이며 실제 기체의 착륙 가능 범위나 완전한 경계가 아니다.
기체 종류 이름만으로 범위를 정하지 않으며 입력 공력·질량·entry 조건이 바뀌면 다시 계산한다.

## 근거와 다음 단계

[Saraf 외, Landing Footprint Computation for Entry Vehicles, AIAA 2004-4774](https://fdcl.eng.uci.edu/pdf/GNC_LFP_2004.pdf)의
공개 원문에서 footprint 정의, entry endpoint와 TAEM 연결, bank 제어·지구 자전·경로제약의
중요성을 확인했다. 논문의 EAGLE 알고리즘을 복제한 것은 아니다.
[Ridderhof 외, Stochastic Entry Guidance](https://arxiv.org/abs/2103.05168)의 공개 초록은
대기 불확실성과 bank 제어를 함께 다루는 후속 연구 방향의 참고다.

IMU-only / IMU+GNSS / IMU+drag 비교는 아직 구현하지 않았다. 이를 위해서는 sensor
측정식·bias/noise·갱신주기·관측 가능 구간과 navigation filter, 그 추정치를 사용하는 폐루프
guidance가 필요하다. 이번 dispersion을 그 비교 결과처럼 해석하면 안 된다.
다음 우선순위는 새 기체 데이터와 terminal 조건을 정한 뒤, 경로제약을 적용한 조밀한
reachable-set 탐색과 navigation-in-the-loop 검증을 연결하는 것이다.

결과: [footprints.png](../output/footprint/footprints.png),
[footprint_results.json](../output/footprint/footprint_results.json),
[validation_results.json](../output/footprint/validation_results.json).
검증 실행: `addpath validation; Validate_Entry_Design; Validate_Footprint_Target; Run_All_Validations`.
