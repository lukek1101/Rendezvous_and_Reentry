# Rendezvous and Reentry Simulator

MATLAB 기반 orbital rendezvous·proximity·atmospheric entry 연구용 simulator입니다. 기본 실행은 Apollo capsule을 탑재한 프로젝트 chaser의 300→500 km phasing/homing과 30 m standoff까지입니다. Spaceplane reference는 HORUS-2B입니다.

## 실행

MATLAB Current Folder를 저장소 루트로 설정합니다.

```matlab
Main_Mission_Simulator
```

Python optimizer 없이 `mission_result`, `Budget`와 orbital dashboard를 생성합니다. 현재 reference의 실제 deorbit handoff는 공력 Mach 범위를 초과하므로 full entry·회수까지 검증된 실행이 아닙니다. Standalone entry는 유효 범위 안에서 별도로 수행할 수 있습니다.

## 문서

| 문서 | 내용 |
|---|---|
| [시스템 기술 보고서](docs/SYSTEM_REPORT.md) | 구성, phase별 알고리즘, 구현 범위와 제약 |
| [사용 설명서](docs/USER_GUIDE_KR.md) | 실행, 설정 예제, 결과 확인, 오류 대응 |
| [설정 및 인터페이스 명세](docs/CONFIGURATION_REFERENCE.md) | 설정 우선순위, 단위·좌표·시각, state와 resource 계약 |
| [기체 및 Entry 모델](docs/VEHICLE_MODELS.md) | Apollo/HORUS/ARD/legacy, 데이터와 validity |
| [성능 평가 및 검증](docs/ANALYSIS_AND_VALIDATION.md) | Benchmark, fixed-design robustness, 결과·실패 분류, 재현 자료 |
| [자료 출처](docs/REFERENCES.md) | Primary sources, provenance, 누락 데이터와 적용 범위 |

## 검사

```matlab
addpath('validation');
checks = Run_All_Validations();
```

수치·회귀 검증과 실제 비행 검증은 구별합니다. `legacy`가 path에 들어가지 않도록 `addpath(genpath(...))`는 사용하지 않습니다.

## 인용 및 License

[CITATION.cff](CITATION.cff)와 사용한 원출처를 함께 인용합니다. 코드·문서에는 [BSD 3-Clause](LICENSE)가 적용되며 제3자 자료의 권리는 [Third-party notices](THIRD_PARTY_NOTICES.md)를 따릅니다.
