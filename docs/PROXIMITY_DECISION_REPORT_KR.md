# 무인 자율 접근 설계 보고 — 2026-09-18

**결정:** HTV의 nadir/R-bar 접근 구성을 참고한 3-DOF 연구 모델을 구현했다.
기본 실행은 `HYBRID_AUTONOMOUS`이며, 인계·closing은 impulsive,
500 m 이후는 유한 추력 피드백 제어다. 실제 HTV 재현이나 완성된 자율 GNC는 아니다.

## 왜 이 방식을 선택했나

| 근거 자료 | 확인한 내용 | 이번 프로젝트에 적용한 범위 |
|---|---|---|
| [JAXA HTV2 Press Kit](https://iss.jaxa.jp/en/htv/mission/htv-2/library/presskit/htv2_presskit_en.pdf), §2.5.1–2.5.2, 인쇄 pp.2-20–2-22 / PDF pp.70–72 | 후방 5 km, 하방 500 m 진입, 250 m와 30 m 정지점 | 거리와 접근 방향의 사례 근거. 실제 10 m 포획, 자세 전환, 센서는 제외 |
| [JAXA HTV Operations](https://iss.jaxa.jp/en/htv/operation/) | 하방 접근, RVS 구간 접근속도 1–10 m/min | 이동 구간 최고속도를 약 8.84 m/min으로 설계. 정지 부근은 0까지 감속 |
| [ESA ATV Safety and Autonomy](https://www.esa.int/Science_Exploration/Human_and_Robotic_Exploration/ATV/Safety_and_autonomy_make_the_ATV_unique) | 자동 기동, hold point의 GO 승인, 별도 실패 대응 계층 | 단계별 상태 검사 개념. 우리 모델의 GO는 수치 조건으로 자동 판정; 실제 운영 승인·독립 안전계층은 미구현 |
| [Berning 외, 2024 공개본](https://arxiv.org/html/2401.11077v1), §§2–4 | CW 기반 기동 설계, 불확실성, 제어 상실 후 drift 검증 | CW 초기해와 비선형 검증을 분리하고 무추력 후속궤적을 진단. 논문의 확률제약 최적화는 구현하지 않음 |

Shuttle·Soyuz까지 모두 섞기보다, 현재 목표인 무인 R-bar 접근에 직접 맞는 HTV를
기하 기준으로 골랐다. ATV는 접근 경로를 복사하지 않고 자동 운용 구조의 참고로 사용했다.
공개 자료의 본문을 확인했으며, 논문의 제어기나 비공개 비행 소프트웨어를 복제한 것은 아니다.

## 적용한 시나리오

1. S2 도착 위치를 확인하고 잔여 상대속도를 **별도 impulse**로 제거한다.
2. **60초 무추력 대기**를 실제 동역학으로 전파한다. 열/냉각 모델은 없다.
3. 실제 출발 상태에서 500 m 하방까지 **출발·도착 두 impulse**를 설계한다.
   비행시간 30/45/60/75/90분을 비교하고, 경로·속도 제약을 만족하는 후보 중 closing ΔV 최소를 선택한다.
4. 500 → 250 m를 3,200초에 접근하고 **60초 능동 정지**한다.
5. 250 → 30 m를 2,800초에 접근하고 **60초 능동 정지** 후 위치·속도 조건을 확인한다.

CW 해는 초기 추정이고, J2/설정된 drag를 포함한 비선형 shooting으로 출발속도를 보정한다.
최종 접근은 양 끝 속도·가속도가 0인 quintic 기준궤적과 CW feedforward + PD 제어를 사용한다.
추력은 매 1초 갱신하고 그 사이 ECI 힘을 유지한다. 질량은 실제 적용한 힘으로 감소한다.
수치 제한 위반 시 실행을 중단하고 deorbit 진입을 막는다. 실제 retreat/abort 기동은 수행하지 않는다.

**별도 연구 가정:** 대기 60초, PD 고유진동수 0.02 rad/s·감쇠비 1,
최대 힘 300 N, corridor 반각 10°와 1 m 여유, 추상적 제외 반경 20 m,
최종 허용 오차 0.25 m·0.01 m/s, 중간 gate 1 m·0.01 m/s,
최소 질량 1,000 kg. Closing은 거리 250 m 이상·속도 10 m/s 이하를 검사한다.
이는 ISS 요구조건이 아니다. 우리 궤도는 기존 500 km 극궤도를 유지하므로 실제 ISS 궤도 재현도 아니다.

## 다른 선택과 판단

| 선택지 | 판단 |
|---|---|
| 기존 cycloid + 8-hop | 비교 기준으로 보존. 반복 횟수는 직접 지정한 3회가 아니라 첫 R-bar 교차에서 결정됨 |
| 비선형 보정한 두-impulse closing | **채택.** 기동 수가 적고, 시간별 비용을 설명할 수 있음 |
| 중간 기동을 추가하는 다중-impulse closing | 이번에는 미구현. 직접 transfer가 실패하거나 안전·시간 제약이 추가되면 비교 후보로 확장 |
| 모든 waypoint에서 정지 | 접근 구간에는 제거. 이번에는 250 m와 30 m에 명시적인 정지 목적을 부여 |
| 전 구간 finite burn / MPC / 확률제약 최적제어 | 후속 단계. 우선 명시적인 hybrid 실행과 검증 가능한 피드백 기준 확보 |
| ±V-bar 접근 | 후속 단계. docking port·corridor 요구가 정해지면 같은 경계조건으로 비교 |
| Simulink | 현재는 MATLAB으로 구현. 기존 동역학을 재사용하며 수치 실험을 자동화할 수 있음 |

75분 해는 **조사한 5개 후보 중 closing 비용 최소**이며 전역 최적해가 아니다.
R-bar 진입 거리도 최적화한 값이 아니라 사례에서 택한 500 m이다.
최종 접근 비용까지 포함한 전체 최적화는 아직 하지 않았다.

## 검증 결과

같은 Phase 1 실제 인계 상태와 고정된 Python archive를 사용했다.
재진입 논문 조건 및 FPA 모델은 변경하지 않았다.

| 지표 | 기존 cycloid/hops | 새 hybrid |
|---|---:|---:|
| Phase 2 총 시간 | 279.63 min | 178.00 min |
| Phase 2 총 ΔV | 61.129 m/s | 64.892 m/s |
| 인계 감속 제외 ΔV | 3.710 m/s | 7.472 m/s |
| 30 m 종점 위치오차 | 0.0156 m | 0.000711 m |

공통 인계 감속은 57.419 m/s다. 새 closing 비용은 1.333 m/s,
유한 추력 접근·정지는 6.140 m/s다. **시간은 약 36% 줄었지만 연료는 증가했다.**
정지 상태 유지와 corridor 추종의 비용을 지불한 결과이며, 기존 대비 연료 최적이라고 주장하지 않는다.
두 모델은 최종 접근 경로와 정지시간이 다르므로 이 표는 알고리즘 효율만 분리한 비교도 아니다.

- 최종 접근 최대속도 0.1473 m/s, 최대 힘 3.775 N, 최대 기준궤적 오차 0.00961 m, 추력 포화 없음.
- 적분 간격과 제어주기를 각각 절반으로 줄였을 때 최종 위치 차이 0.000204 m.
- 초기 상태 오차만 부여한 10회 시험: 10/10 통과. 각 축 위치 σ=1 m, 속도 σ=0.01 m/s, seed=1809.
  항법 오차와 추력 오차는 없는 시험이며 신뢰도 입증용 Monte Carlo가 아니다.
- Corridor 이탈과 0.01 N으로 제한한 추력 부족 사례는 실패로 검출.
- 최종 접근의 9개 시점에서 각각 10분간 무추력 전파: 5초 표본에서 모두 연구용 20 m 제외 반경 바깥.
  연속시간·전체 시점·장시간의 passive safety 보장은 아니다.
- 전체 임무 실행, 기존 MATLAB 회귀 검증과 Python 6개 검증 통과.

**한계:** 완전한 항법 추정, 자세제어, 추진기 배치·minimum impulse bit,
센서/추력 오차, 실제 충돌회피 동작을 모델링하지 않았다. 300 N 한도 내에서
수 N의 힘을 연속적으로 조절할 수 있는 이상적 actuator를 가정한다.
작은 종점 오차는 이 수치 모델의 결과이며 실제 운용 정확도 예측이 아니다.

## 확인과 재현

`Main_Mission_Simulator`는 새 모드를 기본 실행한다.
`Mission_Run_Config.m`의 `phase2.mode="LEGACY_IMPULSIVE"`로 이전 방식을 선택할 수 있다.
`phase2.autonomous`에서 연구 가정을 조정한다. Phase 1의 `FINITE_BURN` 선택은
Phase 2 인계·closing을 finite burn으로 바꾸지 않는다.

```matlab
addpath validation
Study_Proximity_Options
Run_All_Validations
```

결과 원본: [study_results.json](../output/proximity/study_results.json).
비교 그림: [proximity_comparison.png](../output/proximity/proximity_comparison.png).
연구 재현은 `Study_Proximity_Options`처럼 FILE로 archive를 고정한다. 기존 AUTO는 아직 점수 기반 선택이다.

지금 추가로 받아야 하는 문서는 없다. NASA NTRS의
[Trajectory design and navigation analysis for Cargo Transfer Vehicle proximity operations](https://ntrs.nasa.gov/citations/19920069442)는
접근이 거부되어 설계 근거로 사용하지 않았다. 후속 다중 기동/안전성 비교에 사용할 때 원문을 요청할 수 있다.
