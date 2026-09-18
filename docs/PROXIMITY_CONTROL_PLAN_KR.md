# Closing / Final Approach 설계 제안

상태: 초기 설계 초안 보관. 2026-09-18에 hybrid R-bar 제어기를 구현했다.
현재 선택과 수치 검증은 [의사결정 보고서](PROXIMITY_DECISION_REPORT_KR.md)를 따른다.
아래의 '현재 코드' 설명은 LEGACY_IMPULSIVE 비교 모드에 해당한다.
±V-bar 접근, 항법/추력 오차 모델, 실제 abort 기동은 아직 구현하지 않았다.

## 현재 코드에서 확인한 내용

- `+mission/proximity.m`의 Phase 2는 S2 (-V-bar 5 km), cycloidal drift,
  S3 braking, CW waypoint targeting, R-bar hop, S4 braking 순서다.
- `orbit_core.relative_state`의 좌표는 [radial outward, along-track, orbit normal]이다.
  +V-bar는 target 진행 방향 앞쪽, -V-bar는 뒤쪽이다. 상대속도에는 frame 회전을 반영한다.
- S4는 실제 S3 radial 부호를 따라 정해진다. 출력의 '+R-bar' 표기만으로
  실제 접근 방향을 판단하면 안 된다.
- 각 impulse는 속도를 순간적으로 바꾸므로 궤적 접선이 불연속일 수 있다.
  보간으로 그림만 매끄럽게 만드는 것은 물리 모델 개선이 아니다.
- 기본 S4는 30 m standoff이다. 실제 접촉/도킹이나 최종 수 m의 접근은 구현되어 있지 않다.
- `Env_EOM.m`에 회전 상태와 body force 입력이 있어도, 현재 Phase 2는
  free propagation과 impulse를 사용한다. 이것을 폐루프 6-DOF GNC라고 부르기는 어렵다.

## 권장 범위와 구조

우선 MATLAB 기반 3-DOF mission-level closed-loop proximity simulation을 목표로 한다.
기존 impulse 결과는 비교 기준으로 유지한다. Phase 2의 함수 추출과 결과 구조체 반환은
완료했으며, 폐루프 control의 종료 사유/성능 지표는 후속 구현에서 확장한다.
다음 모듈을 분리하면 이후 Simulink로 옮겨도 수식과 검증을 재사용할 수 있다.

1. Guidance: 접근 corridor와 hold point, 기준 위치·속도·가속도를 생성한다.
   정지 구간 사이에는 경계 속도/가속도를 맞춘 quintic profile을 후보로 사용하되,
   접근 시간과 추력 한계로 실행 가능성을 검증한다. 반드시 모든 waypoint에서 정지할 필요는 없다.
2. Controller: CW feedforward + 상대 위치/속도 오차 PD를 첫 비교 기준으로 삼는다.
   이는 원궤도 근방 선형 모델 기반 설계이며 실제 검증 propagation은 기존 비선형 J2를 사용한다.
   PD gain, 주기, 속도 제한은 임의의 mission 검증값으로 포장하지 않고 설정과 근거를 기록한다.
3. Actuator: 처음에는 ECI 방향 힘을 구현하는 이상적 3축 추력 모델로 명시하고,
   force saturation, mass depletion, 선택적 response lag를 적용한다.
   현재 body-frame force 입력에 LVLH 벡터를 그대로 넣지 않는다.
   실제 RCS 배치, minimum impulse bit, attitude loop는 후속 모델 단계다.
4. Sequencer: closing, hold, final standoff, abort 상태와 전환 조건을 관리한다.
   허용 위치·속도 오차, dwell time, timeout, 연료 부족, corridor 위반을 기록한다.
5. Runner: 설정과 난수 seed를 명시적으로 받고 plot과 분리한다.
   Monte Carlo 결과는 terminal error, 상대속도, delta-V, 시간, 포화시간,
   최소거리, corridor 위반, 종료 사유로 비교한다.

유한 추력은 속도 점프를 제거하지만, smooth trajectory가 연료 최소를 뜻하지는 않는다.
추력 명령까지 부드럽게 하려면 기준 가속도 연속성 및 actuator response도 확인해야 한다.
제어기는 정해진 주기로 갱신하고 적분 substep에서는 명령을 유지한다.
센서 잡음을 RK4 각 평가마다 새로 뽑지 않고 측정 주기에만 샘플링한다.

## 접근 방향 확장

`approach_axis = R_BAR | V_BAR`, `approach_side = +1 | -1`처럼 geometry를 명시한다.
예를 들어 30 m standoff는 R-bar에서 [±30, 0, 0], V-bar에서 [0, ±30, 0] m이다.
이것은 interface 제안이며 현재 동작하는 설정이 아니다.

CW radial/along-track dynamics는 서로 다르므로 기존 궤적의 x/y 교환으로 구현하지 않는다.
특히 S2가 -V-bar에 있으므로 +V-bar corridor로 가는 transfer를 별도로 설계해야 한다.
target 중심을 가로지르는 보간을 금지하고, keep-out 영역과 corridor 진입점을 먼저 정한다.
V-bar endpoint만 바꾸는 것과 해당 corridor를 따라 접근하는 것은 서로 다른 요구조건이다.

## MATLAB / Simulink 판단

Simulink는 동역학·제어기·센서·actuator를 block diagram으로 연결하여
시간에 따른 폐루프 동작을 시뮬레이션하고 검증하는 환경이다.
MATLAB과 함께 사용할 수 있으며 코드 생성과 hardware test 연계도 지원한다.
근거: [MathWorks Simulink 소개](https://www.mathworks.com/products/simulink.html).

현재 코드 재사용과 궤적/연료 비교가 중심이면 MATLAB 함수 분리가 우선이라는 것이
이번 설계 판단이다. 실제 추진기·센서의 서로 다른 갱신 주기, 자세제어,
software/hardware-in-the-loop가 주요 요구가 되면 Simulink 전환을 다시 평가한다.
현재 프로젝트에 대한 속도 benchmark나 이전 공수 비교는 수행하지 않았으며,
라이선스 비용이 더 저렴하다는 주장도 하지 않는다.

Monte Carlo는 Simulink 도입의 필수 사유가 아니다. MATLAB runner를 seed별로
반복 실행할 수 있다. Simulink도 여러 입력의 simulation과 `parsim`을 지원한다.
근거: [MathWorks multiple simulations](https://www.mathworks.com/help/simulink/ug/run-multiple-simulations.html).

## 구현 순서와 검증 기준

1. 완료: Phase 2 함수 추출 및 기존 impulse 상태/시간/delta-V 일치 확인.
2. 현재 R-bar 시나리오에 폐루프 제어 추가; 무잡음 nominal case와 시간 간격 수렴 확인.
3. 추력·질량 한계, tracking error, 속도 제한, hold/abort 동작 확인.
4. ±V-bar transfer/corridor 추가 후 동일 초기 조건에서 연료와 시간 비교.
5. 초기 상대 상태, 측정 오차, 추력 크기/방향 오차를 분리해 seed 고정 Monte Carlo 수행.
   불확실성 분포·크기와 성공 판정 임계값은 사용자가 선택한 mission 요구에서 정한다.

## 대화에서 확정할 사항

- 첫 목표를 30 m standoff까지의 translation control로 둘지, 실제 최종 접촉까지 확장할지.
- closing/final 구간 거리, hold 시간, 허용 접근 속도, keep-out 반경과 corridor 폭.
- 이상적 3축 force 가정의 허용 여부와 최대 추력/Isp/갱신 주기.
- 특정 mission 재현을 원한다면 해당 mission의 궤적·waypoint·접근 속도 자료.

NASA 검색에서 `Rendezvous Integration Complexities of the International Space Station`
(NTRS 20090007819) 및 `CTV rendezvous techniques` (NTRS 19930013081)가 확인되었다.
이번 환경에서 해당 NASA 원문/상세 페이지 열기는 실패(403 포함)했다.
따라서 이 문서는 그 원문의 제어법이나 수치를 재현했다고 주장하지 않는다.
특정 사례와 맞추려면 접근 궤적 figure, 좌표 부호 정의, hold point 표,
속도/추력 제한 및 guidance/control section이 포함된 PDF나 발췌가 필요하다.
