# 코드 구조 검토 및 모델 결정 사항

## 검토 범위

활성 MATLAB 진입점/propagator/환경 모델, `+reentry_core`, 두 `+paperstudies`
패키지, validation, Python 설계 도구 3개와 JSON 연결부를 검토했다.
`legacy`의 이전 MATLAB 10개 파일도 구조와 호출 경로를 확인했으며,
실행 대상에서 제외된 역사적 snapshot으로 유지한다. PDF 논문의 수치 자체를
이번 코드 검토에서 새롭게 검증했다는 의미는 아니다.

## 반영한 구조 개선

| 문제 | 변경 |
|---|---|
| 대형 script에 설정·미션·그림이 결합 | `Run_Mission`과 `+mission` 모듈로 분리 |
| `clear`, `close all`로 호출자 작업을 변경 | interactive wrapper에서 제거 |
| 반복 실행마다 파일 수정 및 figure 생성 | struct override, headless API, 독립 plot 함수 |
| 난수 실험의 재현성/전역 상태 의존 | 명시적 seed, 결과에 초기 RNG 기록, 정상/오류 종료 시 복원 |
| MATLAB J2/LVLH 식이 여러 곳에 복제 | `+orbit_core`로 통합, 기존 frame-rate 근사 유지 |
| 상대경로 JSON이 current folder에 의존 | project root 기준으로 해석 |
| cycloid의 매 step마다 전체 history 재복사 | 미리 할당한 구간 buffer에 기록 후 한 번 결합 |
| 설정 오타/0 timestep의 늦은 실패 | unknown override field 거부, 실행 전 loop-bound 검증 |
| Python JSON/hash/index 저장 코드 중복 | `mission_io.py`로 통합 |
| 손상된 index를 빈 것으로 간주해 덮어쓰기 | 오류를 내고 보존; 파일별 atomic replace |
| SPACEPLANE 테스트가 사용자 CAPSULE 기본값에 의존 | 테스트 vehicle을 명시적으로 고정 |
| static 검사 범위가 일부 re-entry 파일에 한정 | 모든 활성 MATLAB package를 찾는 검사 추가 |

기존 root 진입점, optimizer CLI, archive hash 형식은 유지했다.
`Main_Mission_Simulator`는 주요 분석 변수의 호환 alias를 남긴다. 내부 loop 변수까지
공용 interface로 보장하지 않으며, 새 코드는 `mission_result`를 사용한다.
Phase 3의 사라진 parking 경로에 쓰이던 무효 radius tolerance와 연료 미반영 내부 분기도 제거했다.

`+paperstudies`의 식별 방정식별 분리와 `+reentry_core`의 물리 kernel은 이미
역할이 나뉘어 있으므로 무리하게 합치지 않았다. Phase 1 targeting 및 entry event/summary
정책도 수치 알고리즘 자체는 유지했다. 모든 반복문을 벡터화하거나 새로운 적분기로
바꾸지 않았으며, wall-clock 속도 향상 비율을 주장하지 않는다.

## 함께 결정할 Fundamental 사항

### 1. Rendezvous 인계 조건을 위치와 상대속도로 정의할 것인가

기준 실행은 Phase 1에서 S2 위치 오차 약 0.134 m를 달성하지만,
인계 상대속도는 약 57.419 m/s다. Phase 2의 S2 hold trim이 그 속도를 impulse로 제거한다.
`Phasing_Propagator`의 capture 허용오차는 위치 중심이고, Python의
`two_impulse_total` 목적함수는 존재하지만 최종 접근에 허용할 인계 속도 제한과는 별개다.

제안: S2를 "위치 통과점"으로 볼지 "속도까지 맞춘 hold point"로 볼지 정하고,
후자라면 인계 속도·최대 추력·시간을 제한하는 terminal targeting을 설계한다.
제어기 gain 조정만으로 기존 57 m/s 인계가 자연스럽고 저비용이 된다고 가정하지 않는다.
목적함수·제약 변경은 이번에 구현하지 않았다.

### 2. 시뮬레이터의 fidelity와 종료점을 명시할 것인가

현재 기본 미션은 impulse 중심 병진 운동 연구이며, quaternion/회전 상태를 갖는
`Env_EOM`이 있어도 자세 추종 제어기·추력기 배치·navigation filter가 구성된
폐루프 6-DOF mission은 아니다. "berthing"도 실제 접촉이 아니라 약 30 m standoff다.

제안: 1차 목표를 "3-DOF mission-level + finite-thrust proximity control"로 고정하고,
도킹 접촉/6-DOF/RCS allocation은 별도 검증 단계로 둔다. ±V-bar를 추가하려면
접근 corridor와 keep-out 조건, S2에서 corridor까지의 transfer가 먼저 필요하다.
제어기와 ±V-bar 옵션은 아직 구현하지 않았다.

### 3. Python 설계와 MATLAB 검증의 모델을 일치시킬 것인가

MATLAB `Mission_Config`의 J2는 `1.08263e-3`, Python `EarthJ2`는
`1.08262668e-3`이다. MATLAB 저고도 대기는 layer 식/선택적 `atmosisa`,
Python 대기는 density table의 log interpolation을 사용한다.
현재 drag-off Phase 1에서 모든 차이가 큰 영향을 준다는 뜻은 아니지만,
drag-on 결과를 교차 검증할 때는 동일한 모델이라고 가정할 수 없다.

또한 optimizer AUTO 선택은 burn model은 hard filter지만 altitude/drag는 score다.
맞는 case가 없을 때 다른 조건의 결과를 선택할 수 있다. 이 정책은 유지했다.
제안: 향후 공통 환경 명세·상수·대기 golden cases를 만들고, 엄격한 scenario matching을
기본으로 둘지 결정한다. 검증에서는 `FILE`로 정확한 archive를 고정했다.

### 4. 목표 entry FPA를 정확한 constraint로 볼 것인가

현재 CAPSULE 기본값은 paper entry 조건 적용으로 목표 FPA가 -1.16 deg이며,
직접 재진입 실제 결과는 약 -1.061644 deg다. 현재 conic 기반 injection 계산이
비선형 J2 propagation의 terminal angle을 반복 보정하는 shooting solver는 아니다.
제안: 목표 FPA를 정확히 맞추려면 동일한 propagator 위에서 deorbit targeting을 수행하고
허용오차를 선언한다. 이번에는 기존 궤적을 유지했다.

### 5. Monte Carlo의 불확실성·성공 조건을 무엇으로 정의할 것인가

현재 난수 seed interface는 준비했지만, mission-specific noise 분포·센서 주기·추력
오차·hold/abort 조건은 아직 정의되지 않았다. 난수를 추가하는 것만으로 검증 가능한
Monte Carlo 연구가 되지는 않는다. 측정 잡음은 측정 시점에, actuator 오차는 해당 모델의
갱신 주기에 샘플링해야 한다. RK4 내부 평가마다 임의로 바꾸는 방식은 피한다.
어떤 변수의 어떤 분포를 쓸지는 사용자와 결정한 뒤 구현한다.

`result.proximity.reached_standoff`로 위치 허용오차 충족 여부를 노출했지만,
미충족 시 전체 미션을 중단/재시도/abort할지는 기존 정책을 유지했다.
Monte Carlo의 성공률을 계산하기 전 이 전환 정책과 속도 기준도 명시해야 한다.

## 검증 및 한계

- 분리 직후에는 변경 전 전체 trajectory/history/budget/entry diagnostic이 정확히 일치했다.
- 공통 LVLH 계산 적용 후 차이는 Phase 3 상대속도 history의 최대
  `2.27373675443e-13 m/s`였고, 그 외 비교 항목은 동일했다.
- 기존 두 재진입 회귀 실패는 테스트의 사용자 vehicle default 의존을 제거해 해결했다.
- 전체 MATLAB 검사: `addpath validation; Run_All_Validations`.
- 활성 MATLAB 105개 파일의 Code Analyzer, 전체 미션/기존 script 및 3개 dashboard,
  drag-aware impulsive/finite-burn 실행 검증을 통과했다.
- Python 검사: `python -m unittest discover -s validation -p 'test_*.py'`.
- Python 저장/호환성 테스트 6개 및 compile 검사를 통과했다.
- 추가 비교는 기본 drag-off/impulsive CAPSULE 미션의 전체 history를 대상으로 했다.
  모든 optimizer/finite-burn/drag-on 조합의 재최적화를 수행한 것은 아니다.
- 기존 재진입 physics/event 및 paper adapter 회귀를 함께 실행한다. 논문의 완전한
  수치 재현이나 실제 임무 안전성 입증으로 해석하지 않는다.
- JSON 파일별 교체는 atomic하지만, 여러 파일의 일괄 transaction이나 여러 writer의
  동시 index 갱신까지 지원하는 것은 아니다.
