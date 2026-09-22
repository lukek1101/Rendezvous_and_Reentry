# 기체 및 Entry 모델 명세

## Reference의 의미

Reference preset은 출처가 있는 질량·면적·공력과 프로젝트 가정을 구분한 입력 집합이다. 실제 vehicle의 모든 subsystem을 재현하지 않는다. Published는 원문 수치, derived는 단위 변환·명시한 식으로 계산한 수치, surrogate는 프로젝트 근사다. 누락된 값은 다른 variant에서 자동으로 보충하지 않는다. 출처 번호는 [자료 목록](REFERENCES.md)을 따른다.

| Preset | 역할 | Entry 공력 범위 | 핵심 제한 |
|---|---|---|---|
| `APOLLO7_PREFLIGHT_TRIM` | 기본 capsule | Mach .40–27.72, trim curve | 임의 AoA·고 Mach extrapolation 없음 |
| `HORUS_2B` | 기본 spaceplane | Mach 1.2–20, AoA 0–45° | Clean untrimmed table, missing cells |
| `ARD` | 별도 capsule reference | Mach 10–26, AoA 15–25° surrogate | Full descent database 아님 |
| `LEGACY_CAPSULE_60KG` | 이전 reduced capsule 비교 | Constant-coefficient 가정 | 실제 ARD/Apollo/HSRC 재현 아님 |
| `LEGACY_PAPER_RLV` | 이전 reduced spaceplane 비교 | Polynomial surrogate | HORUS trim model 아님 |

Legacy preset은 독립된 비교 모델이며 현재 reference의 범위 밖 데이터를 대신 제공하지 않는다. 이전 spaceplane 형상 중 보존된 비교 정의는 former COMPROMISE 기반 legacy 하나다.

## Apollo 7 capsule

### 기체·공력 데이터

| 입력 | 값 | 성격과 근거 |
|---|---:|---|
| Capsule mass | 5608.261421917 kg | A7 Table/post-separation weight 12364.1 lb의 SI 변환 |
| Reference area | 12.021653376 m² | A8 Table 6-1의 129.4 ft² 변환 |
| Diameter | 3.9116 m | 154 in 변환 |
| Heat-shield radius | 4.694 m | A9; Sutton–Graves nose-radius 입력 |
| Length | NaN | 현재 preset에 확정하지 않은 입력 |
| CL, CD, trim AoA | 12 Mach knots, linear interpolation | A7 Table II(b), preflight trimmed coefficients |
| Requested EI | 121920 m, inertial FPA −2.055° | Mission 설계 입력; 실제 달성값과 별도 |
| Mission analysis endpoint | 7620 m | Parachute/landing 전 프로젝트 종료점 |

Apollo body AoA와 simulator AoA는 `alpha_sim = 180° − alpha_body`로 변환한다. Trim table의 CL 부호와 공통 lift 정의를 유지한다. Table은 Mach별 equilibrium trim 값이며 2D Mach–AoA surface가 아니다. Trim query와의 차이가 1e−8 degree를 넘거나 Mach 범위 밖이면 거부한다.

Preflight aerodynamic curve와 같은 mission의 post-separation mass를 결합한 reference다. 정확한 preflight prediction 또는 Apollo 7 flight restitution 자체가 아니다. A7 Table I(a)의 platform coordinate XYZ는 일반 ECI 좌표로 사용하지 않는다. 공개된 geodetic entry state와 현재 spherical-Earth mission도 같은 초기 상태가 아니다.

### Integrated stack

프로젝트 carrier wet mass 2000 kg에 capsule을 더한 초기 stack은 7608.261421917 kg이다. 300 N, Isp 200 s의 propulsion은 프로젝트 chaser 값이며 Apollo Service Module/RCS 사양이 아니다. Phases 1–3에서 연료를 차감하고 entry interface에서 zero-impulse separation을 수행한다. Capsule reference mass와 남은 carrier mass를 별도 기록한다. 이미 분리한 standalone capsule에서는 다시 질량을 빼지 않아야 한다.

### 유효 범위와 실제 handoff

기본 orbital mission의 실제 entry는 inertial speed 약 7891.77 m/s, inertial FPA 약 −2.048858°, air-relative speed 약 7892.5 m/s, Mach 약 28.7946이다. 이 값은 table 상한 27.72를 넘는다. Standalone 7400 m/s 사례가 성공해도 이 integrated state의 전파를 검증한 것은 아니다.

Upper-atmosphere model은 84852 m 위에서 temperature 186.946 K를 고정하고 calorically perfect sound speed를 사용한다. 따라서 고고도 Mach 해석에는 공력 table의 범위뿐 아니라 이 대기 가정도 함께 적용된다. 단순히 Mach를 clamp하거나 EI speed/FPA를 바꾸어 연속 임무 성공을 만들 수 없다.

## HORUS-2B spaceplane

| 입력 | 값 / 정의 | 근거 |
|---|---|---|
| Mass | 26029 kg | H1 reference; integrated 초기 wet mass로 사용한 프로젝트 가정 |
| Sref | 110 m² | H1 coefficient convention |
| Nose radius | .8 m | H2 reference |
| Length / width / height | 25 / 13 / 4.5 m | H1 geometry |
| Aerodynamics | Mach 1.2–20, AoA 0–45° | H1 clean, untrimmed tabulated CL/CD |
| Missing cells | CD 4개, CL 7개 | Nonzero interpolation weight가 걸리는 missing cell query는 거부 |

Mass는 maneuver 비용에 따라 감소하며 entry에서 26029 kg으로 reset하지 않는다. Clean coefficients는 trimmed coefficients와 다르다. Control-surface trim 및 high-Mach coverage를 확보하지 않은 상태에서 비행 전 구간 reference라고 해석하지 않는다. 추가 vehicle database에서 확인한 값은 출처·variant 검토 없이 활성 table에 덮어쓰지 않는다.

AoA fallback은 사용자 지정 Shuttle-inspired air-speed profile이다.

```text
V ≤ 2000 m/s          alpha = 15°
2000 < V < 5000 m/s   alpha = 15° + 25° (V − 2000)/3000
V ≥ 5000 m/s          alpha = 40°
```

이는 H1이 제공한 HORUS flight schedule이나 trim solution이 아니다. Profile이 AoA를 정의해도 Mach 20 밖의 공력은 생성되지 않는다. 높은 entry speed에서 시작하는 integrated/standalone case는 초기부터 domain error가 날 수 있다.

## ESA ARD capsule

Mass 2800 kg, diameter 2.8 m로부터 Sref=`π(2.8²)/4`=6.157521601 m², nose radius 3.36 m를 사용한다. A1–A4는 geometry, 시험 및 flight profile의 근거다. 현재 force model은 A3의 hypersonic 자료를 근사한 CA=1.36, CN=−.07을 사용하며 Mach 10–26, AoA 15–25°로 적용 범위를 제한한다.

```text
CD = CA cos(alpha) − CN sin(alpha)
CL = CA sin(alpha) + CN cos(alpha)
```

CA/CN과 CL/CD를 직접 동일시하지 않으며 AoA convention과 reference area를 함께 적용한다. A3의 altitude–AoA graph digitization은 별도의 fallback profile이다. AoA/time/altitude flight history가 존재한다는 사실은 같은 범위의 CL/CD database가 확보되었다는 뜻이 아니다. Mach 10 아래의 reference force prediction, 저속 회수 dynamics, full guidance history는 현재 모델을 검증하지 못한다.

ARD 자료는 확보한 원문까지 검토 대상에 포함한다. 제공받은 Johnston 및 EN-AVT-130 chapter를 미열람 문헌처럼 취급하지 않는다. Chapter가 인용한 Rolland flight-control restitution과 Paulat aerodynamic postflight analysis의 상세 원문은 확보하지 못했으므로 그 안의 계수·guidance를 구현했다고 주장하지 않는다.

## Legacy 및 논문 adapter

Legacy capsule은 mass 60 kg, Sref .554 m², CD 1.3, nominal L/D .25, nose radius .420 m surrogate다. Saito의 150 kg capsule과 다르며 논문의 flight performance로 60 kg 결과를 검증할 수 없다. Legacy RLV는 별도 shape와 Zhang의 AoA/CL/CD 표현을 결합한 reduced model이다. 질량·면적의 coefficient convention이 논문에서 완전히 확인된 실제 vehicle은 아니다.

`paperstudies.zhang`는 세 초기 case, terminal box, speed/AoA와 CL/CD 식, spherical dimensionless dynamics, TDRS/RAAP/antenna geometry, latched blackout interval 및 비열적 path constraints를 제공한다. 차량 질량·면적, density/nondimensionalization constants, heating coefficient, optimal bank history, GPOPS-II transcription은 없으므로 optimal trajectory reproduction은 제공하지 않는다. Table 3과 Fig. 8의 heating 단위/한계 불일치는 compliance Boolean으로 변환하지 않는다.

`paperstudies.saito`는 Tables 1,3–11, 네 entry state, 27 ignition epochs, 15 explicit-guidance/225 RPC uncertainty cases 및 공개 algebraic helper를 제공한다. Table 6 개별 entry state/range는 미공개여서 NaN이다. Eq. 17의 distance scaling, explicit-guidance V0/K1/K2/K3, HSRC CA/CN, trim, filter/deadband/predictor 및 RW/RCS 정보는 누락되어 있다. Table 4 좌표로 계산한 range와 표의 range 사이 약 63–65 km 차이도 임의로 fitting하지 않는다.

두 adapter의 default 실행은 equation/source check다. 선택적 forward 실행은 shared entry kernel을 사용하지만 `SURROGATE`로 분류한다. Lu의 unified predictor-corrector는 검토된 guidance 후보이며 활성 closed-loop controller가 아니다. 상세 helper 사용법은 [사용 설명서](USER_GUIDE_KR.md)를 따른다.

## 환경·공력·열·통신 모델

공력은 `q=.5 rho V_air²`, `D=q Sref CD`, `L=q Sref CL`로 계산한다. Lift 방향은 velocity 기준 AoA/bank convention에 의존하며 bank response는 attitude dynamics와 별도다. Density/CD/L/D scale은 민감도 입력이다. Nominal vehicle source 값을 그 scale에 맞추어 재정의하지 않는다.

저고도 대기는 repository ISA76 또는 선택적 MATLAB atmosphere를 사용하며 고고도 density는 table 기반이다. Paper adapter는 toolbox 유무에 따른 차이를 피하려고 repository atmosphere를 고정한다. Rarefied/transitional flow, real-gas chemistry, wind, ablation, TPS response는 없다.

Heating은 Sutton–Graves 형태 `k sqrt(rho/Rn) V³`의 surrogate이고 aerodynamic load는 aero acceleration/g0다. Zhang의 V^3.15 식이나 Saito의 Detra–Kemp–Riddell enthalpy 식과 직접 같은 값이 아니다. Heat flux·integrated heat를 출력해도 재료/TPS acceptance가 확보된 것은 아니다. Infinite default path limit은 제약 검증이 생략된 상태를 뜻한다.

Antenna의 body-frame pointing은 kinematic AoA/bank 변환에 의한 진단이다. 기본 aft mount와 논문의 upper-body mount는 구별한다. Geometric LOS 성공은 plasma blackout 해소, RF margin 또는 relay service availability의 증거가 아니다.
