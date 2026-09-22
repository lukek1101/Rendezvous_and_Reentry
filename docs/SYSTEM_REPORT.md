# 시스템 기술 보고서

## 목적과 해석 범위

이 시스템은 orbital rendezvous, proximity operation, deorbit, atmospheric entry의 병진 운동과 자원 소모를 계산하는 MATLAB 연구용 simulator다. 기본 실행은 300 km 원궤도에서 500 km target으로 이동하여 30 m standoff를 유지하는 Phase 1–2다. Capsule reference는 Apollo 7 preflight trim, spaceplane reference는 HORUS-2B다. Reference vehicle의 명칭은 실제 발사체·서비스 모듈·비행 제어계를 포함한 mission reproduction을 뜻하지 않는다.

현재 nominal orbital sequence와 공력 범위 안의 standalone entry는 계산할 수 있다. 기본 orbital mission에서 생성되는 entry state는 reference 공력 범위를 초과하므로 궤도부터 회수까지의 연속 임무는 검증되어 있지 않다. Closed-loop entry guidance, navigation estimator, full attitude dynamics, parachute, landing 및 docking contact는 구현 범위 밖이다.

문서 체계는 다음과 같다. 알고리즘과 기능 범위는 이 보고서, 수치 설정과 데이터 전달 규약은 [설정 명세](CONFIGURATION_REFERENCE.md), 공력·기체·자료의 적용 범위는 [기체 모델](VEHICLE_MODELS.md), 실험 결과는 [평가 및 검증](ANALYSIS_AND_VALIDATION.md), 실행 절차는 [사용 설명서](USER_GUIDE_KR.md), 논문과 데이터 근거는 [출처 목록](REFERENCES.md)에 정의한다.

## 구성과 실행 경로

```text
Main_Mission_Simulator                 대화형 실행 및 결과 변수
 ├─ Run_Nominal_Orbit                  Phase 1–2 기본 실행
 └─ Run_Mission                        명시적 전체/부분 실행 API
     └─ +mission                      설정 해석, phase 실행, 예산, 결과
         ├─ +orbit_core               J2 중력, LVLH 변환, 궤도 공통식
         └─ Reentry_Propagator
             └─ +reentry_core         entry 물리, event, 공력, 통신 기하

+entry_design → +reentry_core          standalone entry, footprint, target/window 탐색
+paperstudies → +reentry_core          Zhang/Saito 공개식 검사와 surrogate 전파
Python design tools → JSON → +mission 선택적 maneuver 후보 전달
```

`Mission_Config`와 `+reference_vehicle`가 시나리오·기체의 기본 물성을 제공하고, `Mission_Run_Config`가 실행 방법을 제공한다. `mission.configure`는 override와 artifact 호환성을 검사한다. `mission.run`은 실제 phase 종료 상태를 다음 phase로 전달하고, `mission.plot_results`는 결과 구조체를 시각화한다. `legacy`는 활성 경로에 포함하지 않는다.

## 동역학과 자원 계산

기본 translational state는 ECI 위치·속도·질량이다. 중력은 central 또는 J2이며, 궤도 drag는 별도 옵션이다. 대기 속도는 co-rotation 설정 시 `v_air = v_ECI − omega × r`다. 바람, 고차 중력장, 제3체, 복사압은 현재 nominal model에 포함하지 않는다.

Impulsive maneuver는 위치와 시각을 유지하고 속도를 Δv만큼 바꾼다. 질량은 `m_after = m_before exp(−|Δv|/(Isp g0))`로 감소한다. Finite force는 운동방정식에 `F/m`을 더하고 `dm/dt = −|F|/(Isp g0)`를 적분한다. Force saturation 이후 실제 적용한 힘으로 비용을 계산한다. Phase별 ΔV뿐 아니라 impulse, continuous control, 분리체 질량을 구분한 budget을 사용한다.

14-state mission vector의 quaternion과 angular-rate 항은 현재 mission의 완성된 attitude-control loop를 구성하지 않는다. 수 N 수준의 연속 3축 force를 제공하는 ideal actuator 가정은 RCS thruster allocation이나 minimum impulse bit 모델과 구별한다.

## Phase 1: nominal phasing 및 homing

지원하는 빠른 planner는 상승 coplanar transfer, X–Z polar plane, J2, drag-free, impulsive maneuver 조건을 사용한다. 임의 inclination, plane change, finite-burn 또는 drag-on nominal targeting으로 일반화되어 있지 않다.

Hohmann half-period와 원궤도 mean-motion 차이로 첫 출발 대기시간을 구한다. 실제 J2 dynamics로 대기 상태를 전파한 뒤, 출발 impulse의 두 in-plane 성분을 finite-difference Jacobian과 bounded Newton correction으로 수정한다. 대기시간과 transfer duration은 고정한다. 도착 impulse는 실제 위치에서 목표 rotating-LVLH 속도에 맞춘다. 기본 handoff는 `[0, −5000, 0] m`, 상대속도 0이다.

이는 position/velocity terminal targeting이며 ΔV optimization이 아니다. `GRID_SEARCH`, `CUSTOM_IMPULSE`, Python shooting/optimization은 선택적 비교 경로다. 이전 custom 방식은 속도 정합 비용을 Phase 2 handoff에 부과하므로 Phase 1 비용만 비교해서는 안 된다.

### 제한된 orbit correction

선택적 correction 경로는 nominal plan을 고정한 채 초기 ECI 상태와 maneuver 실행에 seeded error를 적용한다. State knowledge는 `PERFECT_INSTANTANEOUS`다. Transfer의 35%, 75%에서 terminal error를 예측하고 제한된 3D correction을 계산한다. 기존 arrival maneuver를 retarget하며, 설정에 따라 마지막 velocity cleanup을 사용할 수 있다.

Correction count, 개별/총 ΔV, propellant, minimum mass, elapsed time 및 remaining transfer time을 검사한다. 실제 전달된 impulse와 실행 오차를 state와 budget에 반영하며 위치·속도 reset으로 성공을 만들지 않는다. 시험 범위 밖의 오차를 해결하기 위한 추가 phasing-orbit leg나 일반 sequence optimizer는 없다.

## Phase 2: selectable proximity approach

`HYBRID_AUTONOMOUS`는 실제 Phase 1 state에서 handoff braking, 60 s free drift, acquisition transfer, controlled approach, hold 순서로 진행한다. +R/−R/+V/−V의 부호는 target에 대한 chaser의 위치 쪽이다. R은 radial outward, V는 transverse forward다.

Acquisition은 기본 500 m이며 CW 해를 초기 추정으로 사용하고 비선형 propagation으로 수정한다. 30/45/60/75/90 min의 transfer 후보 중 검사한 제약을 만족하는 closing ΔV 최소 후보를 선택한다. 이 유한 후보 선택은 전체 임무 최적화가 아니다.

500→250→30 m 구간에는 양 끝 속도·가속도가 0인 quintic reference와 CW feedforward + PD를 사용한다. LVLH `[R,V,H]`에서 선형 관계는 다음과 같다.

```text
Rddot = 3 n² R + 2 n Vdot + uR
Vddot = −2 n Rdot + uV
Hddot = −n² H + uH
```

따라서 radial/along-track 접근은 단순한 그림 회전으로 동등해지지 않는다. 기준궤적을 선택한 축에 구성하고 해당 축의 gravity-gradient/Coriolis 항을 계산한다. 실제 propagation에는 공통 비선형 dynamics를 사용한다. 매 control period마다 구한 ECI force를 다음 갱신까지 유지한다.

Corridor, keep-out radius, speed, force, mass, gate 및 final standoff를 검사한다. 실패 시 다음 phase를 막는 monitor는 있지만 실제 retreat/abort maneuver는 없다. `LEGACY_IMPULSIVE`는 cycloid 및 stop/start R-bar hops의 별도 비교 경로다. HTV는 500/250/30 m geometry의 공개 근거이며 실제 ISS orbit, 센서, 10 m capture, 자세 전환의 재현 대상은 아니다.

## Phase 3: deorbit와 entry interface

기본 direct deorbit는 conic 계산으로 요청 FPA에 대응하는 injection을 구한 뒤 실제 dynamics로 descending entry-altitude event까지 전파한다. 요청 FPA와 실제 도착 FPA는 다른 결과 항목이다. 요청값을 만족하도록 종료 state를 교체하지 않는다. Drag-aware Python design은 호환성이 확인된 artifact를 사용할 때만 별도로 적용한다.

`apogee_burns`는 명시한 ΔV 배분과 duration/cooldown/count/time/mass 제한을 실행하는 선택적 finite-burn schedule이다. 첫 burn은 `CURRENT` 또는 `NEXT_APOGEE`이며 후속 burn은 cooldown 후 radial velocity의 양→음 crossing에서 시작한다. Duration cap 때문에 한 배분량이 여러 burn으로 나뉠 수 있다. 이것은 thermal model, 자동 FPA targeting 또는 HTV deorbit reproduction이 아니다.

## Phase 4: entry와 독립 연구 기능

Integrated entry는 실제 deorbit terminal position, velocity, epoch를 사용한다. Capsule의 entry-interface separation은 zero impulse이며 carrier의 남은 질량은 별도 ledger에 남긴다. Carrier의 후속 대기 궤적은 전파하지 않는다. Standalone entry에서는 FPA·heading·speed를 연구 입력으로 지정할 수 있다.

공력은 선택한 reference의 validity check를 통과해야 한다. AoA reference schedule은 guidance law가 아니며, bank는 lift 방향을 정한다. 현재 integrated baseline은 open-loop다. `entry_design`의 bank magnitude/rate/response 모델과 고정 profile 탐색도 비행 중 재계획하는 closed-loop predictor-corrector와 다르다.

| 연구 기능 | 계산 의미 | 해석 제한 |
|---|---|---|
| `entry_design.footprint` | 고정 bank profile들의 endpoint | Hull 내부 전체의 도달 가능성 보장 없음 |
| `entry_design.target` | 제한된 profile 탐색·국소 보정으로 target witness 탐색 | 탐색 실패는 물리적 불가능성 아님 |
| `entry_design.dispersion` | 고정 policy의 초기 상태·환경 민감도 | Navigation filter 또는 flight reliability 아님 |
| `entry_design.window` | 대기시간별 실제 deorbit/entry와 target 설계 | Candidate마다 redesign하는 design sweep |
| `+paperstudies` | 공개식·표·좌표 검사, 선택적 surrogate forward | 논문의 optimal trajectory/landing 재현 아님 |

통신 진단은 antenna RAAP, cone, range, Earth occultation을 계산한다. Mission target relay 또는 Earth-fixed GEO reference를 선택할 수 있다. `PAPER_BZC`는 최초 downward 80 km 진입부터 최초 downward 60 km 이탈까지 latch되므로 중간 skip에도 유지된다. Instantaneous bank scan은 rate/trajectory 제약을 함께 푸는 guidance가 아니다. RF link budget와 plasma attenuation은 없다.

## 기능 확장에 필요한 조건

| 대상 | 선행 조건 | 수용 기준 |
|---|---|---|
| Integrated reference entry | 실제 handoff Mach를 포괄하는 일관된 aero·upper-atmosphere 정의 | 범위 밖 query 없이 실제 state/epoch/mass가 연속되는 전파, source·수치 수렴 확인 |
| Closed-loop entry guidance | 유효한 전 구간 predictor, 명시한 endpoint/target, bank 한계 | Open-loop와 동일 초기 조건 비교; terminal/path/control 지표와 실패 원인 기록 |
| Recovery/landing | 저속 공력, parachute 또는 approach/flare·지상 환경 | Atmospheric endpoint와 touchdown의 별도 acceptance |
| Navigation/actuator realism | 센서·추력기·attitude 사양 및 uncertainty 근거 | Truth/estimate/command/delivered state 분리, resource/phase 연속성 검사 |
| 추가 orbital architecture | 현재 제한으로 처리할 수 없는 구체적 mission 요구 | 기존 nominal baseline 유지, entry/exit 조건과 독립 비용 검증 |

수치 오차가 작다는 사실은 공력·대기·actuator 가정의 비행 정확도를 입증하지 않는다. 평가의 success는 명시한 모델과 자원 조건 안에서만 정의한다.
