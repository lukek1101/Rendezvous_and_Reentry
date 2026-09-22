# 설정 및 인터페이스 명세

## 설정 소유권과 우선순위

`Mission_Config(preset)`는 orbit/environment/vehicle의 기본값을 구성한다. `Run_Mission(overrides, options)`의 `options.system`은 물성 override, 첫 번째 인자는 `Mission_Run_Config`의 실행 옵션 override다. Unknown field는 오류로 처리한다.

```text
Reference preset → options.system → run overrides → enabled environment overrides
Python JSON → 호환성 검사된 maneuver 후보만 제공
```

Environment override는 `runtime.allow_environment_overrides=false`가 기본이다. JSON은 vehicle·environment의 물성 권한을 갖지 않는다. Nonempty run maneuver 값은 JSON 값보다 우선한다. 결과의 `metadata.provenance`, configuration provenance, physics contract에 preset과 override를 기록한다. Source-derived preset을 바꾸는 사용자 값은 source의 published value로 간주하지 않는다.

| 진입점 | 실행 선택 |
|---|---|
| `Main_Mission_Simulator` | `ORBIT_ONLY`; Python `NONE`, Phase 1 `HOHMANN/NOMINAL_TARGET`, plot 표시 |
| `Run_Nominal_Orbit(overrides, options)` | 같은 nominal 기본값; Phase 2 후 종료 |
| `Run_Mission(overrides, options)` | 전체 runner; 인자 생략 시 기존 `AUTO/CUSTOM_IMPULSE` 설정 |
| `options` | `plot`, `verbose`, `seed`, `preset`, `system`, `stop_after_proximity` |

기본 preset은 `APOLLO7_PREFLIGHT_TRIM`이다. SPACEPLANE 모드 선택 시 HORUS가 사용되며 명시적 `options.preset`으로 reference를 지정할 수 있다. 설정 파일의 일부 주석에 남아 있는 과거 60 kg capsule/네 spaceplane 설명은 현재 preset 정의가 아니다. 실제 물성과 선택 가능한 모델은 [기체 명세](VEHICLE_MODELS.md)를 따른다.

## 단위·좌표·시각

| 항목 | 규약 |
|---|---|
| Translational core state | `[r_ECI(3); v_ECI(3); mass]`, m, m/s, kg |
| Mission state | `[r(3); v(3); quaternion(4); angular_rate(3); mass]`, 14성분 |
| LVLH | `[R,V,H]`: outward radial, transverse forward, orbit normal |
| Relative velocity | ECI 차이를 회전 frame으로 변환하고 `h/r²` frame-rate 항 반영 |
| Force / propulsion | N, Isp s, propellant kg, maneuver ΔV m/s |
| Angles | `_deg`는 degree, `_rad`는 radian; 무접미 internal 식은 함수 계약 확인 |
| Mission epoch | 상대 mission elapsed seconds; UTC 기본 미지정 |
| Earth-fixed 위치 | 회전하는 구형 Earth의 geocentric latitude/longitude |
| Standalone heading/FPA | North에서 clockwise heading; descent FPA 음수; 입력 speed/FPA는 air-relative |
| Integrated FPA | 실제 state로 산출; inertial/air-relative 값을 구분 |

`entry_design.earth_angle`의 UTC 기반 회전각은 UTC≈UT1의 근사 GMST다. EOP, nutation, WGS-84 geodetic 변환을 포함한 정밀 지구고정좌표가 아니다. UTC 미지정 상태에서 longitude가 실제 날짜의 ground track이라고 해석하지 않는다.

## Phase-state 계약

| 경계 | 전달 및 검사 |
|---|---|
| Phase 1 → 2 | 실제 chaser/target ECI state, 질량, elapsed time; LVLH position/velocity acceptance |
| Phase 2 → 3 | 실제 final hold state; standoff 실패 시 downstream 중단 |
| Phase 3 → 4 | 실제 descending altitude event state 및 누적 epoch; 요청/달성 entry 조건 별도 기록 |
| Capsule separation | 동일 position/velocity/epoch; capsule과 carrier mass ledger 분리, zero separation impulse |
| Standalone entry | 명시적으로 구성한 초기 조건; integrated handoff와 혼용하지 않음 |

Entry event의 altitude/history 일관성 검사는 1 mm 수준이다. 이 수치 검사는 요청 FPA를 달성했다는 판정이 아니다. `result.interfaces`, `result.phasing`, `result.proximity`, `result.deorbit`, entry 결과 및 budget을 함께 확인한다. 실행하지 않은 phase의 Main compatibility alias는 빈 배열이며 성공 결과가 아니다.

## Nominal targeting과 correction 한계

| 종류 | 설정/기본값 | 의미 |
|---|---|---|
| Integration | RelTol 1e−11; AbsTol position 1e−4 m, velocity 1e−7 m/s, mass 1e−9 kg; MaxStep 60 s | ODE local error control |
| Target acceptance | `position_tolerance_m=0.5`, `velocity_tolerance_m_s=0.001` | Propagated handoff 검사 |
| Numerical work | 8 iterations, 40 evaluations, 60 s guard | 실시간 보장이 아닌 계산 제한 |
| Newton correction | FD 0.01 m/s, update norm 25 m/s, damping 1/.5/.25 | Departure impulse 보정 |
| Optimization | Python `ftol`, iteration budget | Target tolerance와 별개; 수렴 상태도 별도 확인 |

`phase1.correction`은 다음 기본값을 갖는다. Propellant allowance는 correction impulse만의 예산이 아니라 해당 Phase 1 sequence에 적용되므로 nominal maneuver도 차감된다.

| 설정군 | 값 |
|---|---|
| 활성화 / 상태 지식 | `enabled=false`, `corrections_enabled=true`, `PERFECT_INSTANTANEOUS` |
| 난수 | seed 42; 독립 Gaussian ECI σr=[20,20,20] m, σv=[.02,.02,.02] m/s |
| Burn error | gain σ=.002; pointing-vector σ=.0002 rad |
| Opportunity | transfer fractions [.35,.75]; terminal velocity cleanup 활성 |
| Correction budget | 최대 3회; 1회 2 m/s; 합계 3 m/s |
| 기타 maneuver/resource | nominal burn 최대 150 m/s, propellant 400 kg, minimum mass 1000 kg |
| Time / work | elapsed 40000 s; remaining transfer 120 s 이상; 6 iterations/30 evaluations/60 s |

난수는 전역 RNG에 의존하지 않는 seeded stream과 정해진 draw slot을 사용한다. Correction off/on 비교에서 같은 nominal disturbance를 사용해야 한다. 위 오차 크기는 탐색 실험 가정이며 실제 spacecraft 사양이 아니다. Apollo stack에는 기본 400 kg allowance가 부족하다. 평가의 mass-scaled allowance를 실제 탱크 용량으로 해석하지 않는다.

## Proximity 기본 설정

`phase2.mode="HYBRID_AUTONOMOUS"`, `phase2.autonomous.approach_mode="-R"`가 기본이다.

| 항목 | 값 |
|---|---|
| Acquisition / hold / terminal range | 500 / 250 / 30 m |
| Closing candidate times | [1800,2700,3600,4500,5400] s |
| Approach times / holds | [3200,2800] s; 중간·마지막 각 60 s |
| Handoff free dwell | 60 s |
| Controller | natural frequency .02 rad/s, damping 1, control period 1 s |
| Ideal force | 최대 300 N; nominal project propulsion Isp 200 s |
| Corridor / keep-out | half-angle 10°, floor 1 m, keep-out 20 m |
| Closing bounds | minimum range 250 m, maximum speed 10 m/s |
| Controlled approach speed | 최대 .17 m/s |
| Gate / final | position 1 m / .25 m; speed .01 m/s |
| Mass | minimum 1000 kg |

`phase2.terminal_standoff_m`는 mode-independent override다. 빈 값이면 기존 `S4_R_abs_m`를 사용한다. Phase 1의 `FINITE_BURN` 선택이 proximity handoff/closing impulse를 finite burn으로 바꾸지는 않는다.

## Entry 설정과 response

| 설정 위치 | 역할 |
|---|---|
| `reentry.vehicle_mode`, `options.preset` | Capsule/spaceplane 및 reference 선택 |
| `reentry.aoa_profile`, `aoa_deg` | 비어 있으면 reference fallback; 명시 profile과 scalar 동시 지정은 오류 |
| `reentry.bank_angle_deg` | Mission open-loop bank |
| `reentry.gravity_model` | `CENTRAL_SPHERICAL` 또는 `J2` |
| `reentry.uncertainty` | `density_scale`, `cd_scale`, `ld_scale`; vehicle 자체를 바꾸지 않는 study scale |
| `capsule.separation_mode` | `ENTRY_INTERFACE` 또는 `ATTACHED` |
| `communication` | relay, antenna, cone/range, tracking scope, epoch rotation angle |

`entry_design.defaults`는 endpoint 20 km, 최대 5000 s, MaxStep 5 s, RelTol 1e−8, bank magnitude 60°, rate 10°/s, response 2 s, initial bank 0°를 사용한다. 세 speed fraction [1,.75,.4]에 대응하는 bank parameter로 profile을 표현한다. Target tolerance 기본 10 km, local search 최대 35 iterations/80 evaluations다. Default path limits가 Inf이면 terminal 도착만으로 q/load/heating acceptance를 주장할 수 없다.

Apollo mission preset의 altitude endpoint는 7620 m이며 standalone defaults의 20 km와 다르다. 모두 touchdown 전의 분석 종료점이다. Reference 공력 경계는 종료 고도를 낮춘다고 연장되지 않는다.

## Python coupling과 artifact 관리

`J2PolarHohmann.py`, `J2PolarHohmannShooting.py`, `DragDeorbitDesigner.py`는 선택적 설계 도구다. `mission_io.py`가 JSON/hash/index 저장을 담당한다. MATLAB-only nominal 실행에는 필요하지 않다.

선택 모드는 `NONE`, `FILE`, `CASE_ID`, `HASH`, `AUTO`다. `AUTO`의 점수 기반 선택 자체는 physics 호환성 증명이 아니다. 선택 후 physics contract, reference/preset, 적용 조건, hash 및 optimizer 상태를 확인해야 한다. 명시적 `FILE`이나 case/hash 선택이 재현성에 적합하다. `allow_legacy_replay=false`가 기본이며, 검증된 contract가 없는 archive는 자동으로 신규 reference mission의 설계가 되지 않는다.

`CUSTOM_IMPULSE`에는 phase angle, ΔV, burn direction gamma가 필요하다. JSON이 없을 때 세 값을 명시하지 않으면 과거 fallback 숫자를 쓰지 않고 오류를 낸다. 저장소의 historical JSON이나 CLI 실행 결과가 존재한다는 사실만으로 현재 조건의 solution임을 보장하지 않는다.

MATLAB과 Python의 J2 상수 및 대기 구현에는 차이가 있다. 특히 drag-on 결과를 동일 dynamics의 해로 간주하지 않는다. JSON 파일별 교체는 atomic하지만 다중 파일 transaction이나 동시 writer index 갱신은 보장하지 않는다.

## 재현 가능한 실행 기록

한 case는 preset, system/run override, seed, code/artifact hash, solver settings, 실제 initial state, requested/achieved handoff, termination reason, resource ledger를 함께 보관한다. Mission-relative time과 UTC를 구분하고, numerical tolerance와 물리적 acceptance를 각각 기록한다. Historical evidence의 날짜 포함 폴더명은 artifact 식별자이며 현재 설정 우선순위가 아니다.
