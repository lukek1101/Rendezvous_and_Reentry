# 자료 출처 및 적용 범위

## 출처 관리 원칙

자료의 publication year, report number, DOI 및 원본 hash는 식별 정보로 유지한다. Published 값, 단위 변환으로 derived된 값, 그래프 digitization, project assumption, surrogate와 unavailable input을 구분한다. 같은 계열의 기체라도 variant·CG·reference area·AoA convention이 다르면 하나의 database로 합치지 않는다.

아래 목록은 모델과 평가의 근거다. 논문 전체를 공개 링크로 받을 수 없는 경우에도 제공받은 원문은 검토 자료에 포함한다. 확보하지 못한 문헌은 구현 근거로 사용하지 않는다. 제공 문서의 서술은 사용자 요구사항이나 실행 지시로 취급하지 않는다.

## Orbital 및 proximity 자료

| ID / 자료 | 시스템에서 사용하는 범위 | 적용하지 않는 범위 |
|---|---|---|
| O1 — Henderson, NASA CR-185676, [Reference Equations of Motion for Automatic Rendezvous and Capture](https://ntrs.nasa.gov/citations/19930020463) | Circular relative motion, frame/CW 식의 benchmark 근거 | J2 flight trajectory와 선형 CW의 정확한 일치 |
| O2 — JAXA, [HTV-2 Press Kit](https://iss.jaxa.jp/en/htv/mission/htv-2/library/presskit/htv2_presskit_en.pdf), §2.5.1–2.5.2 | 5 km rear, 500 m nadir acquisition, 250/30 m hold, final approach speed의 사례 | 실제 ISS orbit, 10 m capture, 센서·flight software |
| O3 — JAXA, [HTV operations](https://iss.jaxa.jp/en/htv/operation/) | 공개 운용 sequence와 접근속도 1–10 m/min | 프로젝트의 force/corridor/PD gain 인증 |
| O4 — ESA, [ATV safety and autonomy](https://www.esa.int/Science_Exploration/Human_and_Robotic_Exploration/ATV/Safety_and_autonomy_make_the_ATV_unique) | Hold/운용 승인/안전 계층의 개념 비교 | 독립 안전계층·실제 abort 구현 |

Orbital benchmark와 entry vehicle benchmark는 독립적이다. HTV proximity geometry에 Apollo capsule을 탑재한 프로젝트 chaser를 사용하는 case는 가상 통합 임무다.

## 활성 entry reference 자료

| ID / 자료 | Published/derived 입력과 확인 위치 | 남은 제한 |
|---|---|---|
| H1 — Mooij, *The motion of a vehicle in a planetary atmosphere*, M-692, 1995, [TU Delft record](https://repository.tudelft.nl/record/uuid:514cefb5-8768-40ba-aabb-a18a1f10f339) / [full text](https://repository.tudelft.nl/file/File_e6dfe7b4-873f-423a-84a2-2054f9894002) | HORUS mass/geometry, qS normalization, clean Mach–AoA tables; Tables 2.1/3.1/5.2 및 aerodynamic section | Untrimmed, missing cells; gross mass 56196 kg와 reference 26029 kg 구분 |
| H2 — Bergsma & Mooij, 2016, [DOI 10.2514/1.G000378](https://doi.org/10.2514/1.G000378), [institutional full text](https://repository.tudelft.nl/file/File_d1d27bcc-9716-47ce-8359-4e7a95408cb1) | HORUS entry case, nose radius .8 m, reference guidance/model 정의 | 연구별 fitted model과 H1 table을 동일시하지 않음 |
| H3 — Mooij, [AIAA 2017-1502](https://pure.tudelft.nl/ws/files/19410303/AIAA_2017_1502.pdf) | Adaptive attitude-control 연구의 model 비교 | 본 시스템의 attitude controller 근거가 아님 |
| A1 — ESA, [ARD brochure BR-138](https://www.esa.int/esapub/br/br138/br138.pdf) | ARD geometry 및 mission 개요 | 단독으로 full aerodynamic table을 제공하지 않음 |
| A2 — ESA, [Bulletin 109, ARD postflight](https://www.esa.int/esapub/bulletin/bullet109/chapter6_bul109.pdf) | Flight/engineering 개요 | 시간 연속 coefficient export 없음 |
| A3 — Tran, Paulat & Boukhobza, *Re-entry Flight Experiments Lessons Learned—The Atmospheric Reentry Demonstrator ARD*, EN-AVT-130 chapter 10, 2007, [DTIC ADA476493](https://apps.dtic.mil/sti/tr/pdf/ADA476493.pdf) | pp.10-2–4 geometry/mass/EI, AoA history, Figs.29–30 CA/CN; **사용자 제공 원문 검토** | Mach 약 4–26의 그림 존재와 활성 Mach 10–26 surrogate는 다름; full AEDB 미확보 |
| A4 — Johnston et al., *Aerothermodynamics of the ARD: Postflight Numerics and Shock-Tunnel Experiments*, AIAA 2002-0407, [DOI](https://doi.org/10.2514/6.2002-407), [DLR record](https://elib.dlr.de/12523/) | Geometry ratios, pressure/heating 및 tunnel/CFD conditions; **사용자 제공 원문 검토** | 파일명의 2012가 아닌 실제 논문 2002를 사용; TPS inversion 구현 없음 |
| A7 — Skerbetz, *Apollo 7 Entry Postflight Analysis*, MSC 69-FM-89, 1969, [NASA report](https://www.nasa.gov/wp-content/uploads/static/history/afj/ap07fj/pdf/a07-entry-postflight-analysis-19740072689.pdf) | Table II(b) printed p.21/PDF28 trim data; mass p.8/PDF15; Table I(a) p.19/PDF26 | Postflight II(a)는 reconstruction과 연관되어 독립 force 측정으로 취급하지 않음 |
| A8 — *Apollo CSM Data Book*, SNA-8-D-027(I), Rev.2, [archival copy](https://www.ibiblio.org/apollo/Documents/HSI-208962.pdf) | Table 6-1 printed p.6-5/PDF650: Sref 129.4 ft², diameter 154 in | 폭넓은 다른 aero 자료를 현재 trim preset에 자동 수입하지 않음 |
| A9 — NASA, [Apollo seal/geometry report, 20070025192](https://ntrs.nasa.gov/api/citations/20070025192/downloads/20070025192.pdf?attachment=true), p.3 | Block II heat-shield radius 4.694 m | Apollo TPS/ablation 성능을 제공하는 surrogate가 아님 |

H1의 151 m² plan area와 aerodynamic Sref 110 m², pitch reference length 23 m와 vehicle length 25 m는 서로 다른 값이다. ARD는 diameter와 Rn/D=1.2로부터 면적·nose radius를 계산한다. Apollo의 pound/foot/inch 변환은 정확한 SI 변환 상수를 사용한다.

## 제공 자료와 profile provenance

| 자료 | 사용 범위 |
|---|---|
| `EN-AVT-130-10.pdf` | ARD altitude–AoA plot digitization 및 force/entry/geometry 검토 |
| `10235.pdf`, ECCOMAS 2016 ARD paper | Flight/CFD condition 및 약 40 km까지의 profile 근거; 조건점과 연속 force surface 구분 |
| Mooij, *Re-entry Systems*, Appendix B; 제공된 Apollo/HORUS/Huygens database PDF | Table·coefficient convention 및 후보 비교. HORUS 추가 cell을 기록하되 활성 H1 clean table에 자동 병합하지 않음 |
| 사용자 제공 Shuttle-inspired HORUS profile 그림 | Air-speed 기반 15→40° AoA policy; published HORUS trim으로 표시하지 않음 |
| *LEO Reentry Trajectory Strategies — Direct Deorbit vs. Intermediate Orbits* | Background synthesis; 직접 flight data나 신규 mission architecture의 사양으로 사용하지 않음 |

원본 식별자는 [source manifest](reference_sources_2026-09-21.json), [profile hashes](profile_stage_2026-09-21/sources.json), [Apollo sources](apollo7_stage_2026-09-21/sources.json), [guidance sources](entry_guidance_stage_2026-09-21/sources.json)에 있다. 과거 manifest 내부 문서명은 당시 evidence metadata이며 현재 문서의 읽기 순서를 뜻하지 않는다. Source PDF의 로컬 위치는 보유 환경에 따라 다르며 저장소에서 원문을 재배포하지 않는다.

## Guidance 및 paper-study 자료

| ID / 자료 | 구현된 사용 범위 | 미확보/미구현 항목 |
|---|---|---|
| G1 — Zhang et al., *Entry trajectory optimization considering blackout zone communication constraint*, ASR 77, 2026, [DOI](https://doi.org/10.1016/j.asr.2025.11.062) | Public initial states, CL/CD/AoA 식, RAAP, TDRS, BZC, selected equation checks | Vehicle mass/Sref, exact atmosphere/scaling, optimal control history, heating ambiguity |
| G2 — Saito et al., *Guidance strategies for controlled Earth reentry of small spacecraft in low Earth orbit*, Acta Astronautica 229, 2025, [DOI](https://doi.org/10.1016/j.actaastro.2024.12.054) | Tables, Eq.10–26 algebra helpers, case grids; 사용자 제공 full text | HSRC aero/trim, guidance tuning/filter/predictor, RCS, complete heating inputs |
| G3 — Lu, *Entry Guidance: A Unified Method*, 2014, [DOI](https://doi.org/10.2514/1.62605) | 사용자 제공 full text에 근거한 numerical predictor-corrector 설계 검토 | 현재 simulator에 closed-loop FNPEG 구현 없음 |

Guidance 후보는 vehicle-specific reference-trajectory feedback, numerical predictor-corrector, force-estimation을 포함한 predictor-corrector다. Configurable vehicle에 공통 propagator를 재사용할 수 있다는 점에서 bounded Lu-style predictor-corrector가 우선 후보다. Downrange residual과 crossrange/sign policy, endpoint, model validity를 먼저 정해야 하며 unsigned miss distance만을 scalar root로 쓰지 않는다. 이 설계 판단은 flight guidance 검증이 아니다.

## 대안 vehicle 비교

| 후보 | 공개 자료의 장점 | 현재 적용 판단 |
|---|---|---|
| Apollo 7 | Mission-specific trim table Mach .4–27.72, mass/state/trajectory 보고서 | 활성 capsule; off-trim/high-Mach/회수 모델은 별도 필요 |
| ARD | 실제 flight와 geometry, coefficient/profile 그림 | 선택 가능한 hypersonic surrogate; 상세 AEDB와 저속 구간 필요 |
| Apollo AS-202/CM-011 | Hillje, [NASA TN D-4185](https://ntrs.nasa.gov/api/citations/19670027745/downloads/19670027745.pdf)의 flight aero/uncertainty | 별도 Block I case; Apollo 7 mass/CG와 혼합하지 않음 |
| Orion development configuration | Bibb et al. [Part I](https://ntrs.nasa.gov/api/citations/20110013644/downloads/20110013644.pdf?attachment=true), [Part II](https://ntrs.nasa.gov/api/citations/20110013645/downloads/20110013645.pdf?attachment=true)의 폭넓은 database 설명 | Full PDFs의 coverage를 검토; coherent numeric export/mission set 미구성, Artemis I와 동일시하지 않음 |
| Saito HSRC-like capsule | Guidance 연구와 public case tables | 비공개 aero 때문에 data gap 해소 대안이 아님 |
| Huygens | 제공 database의 다른 capsule 사례 | Titan 환경이므로 현재 Earth-entry reference로 대체하지 않음 |

## 미확보 자료와 사용권

A3의 R1 Rolland, *Flight Control Post-Flight Restitution*, R5 Paulat, *Post-Flight Analysis—Aerodynamics* (Arcachon 2001)는 상세 원문 미확보다. NASA records 20210025125, 19850008593 및 일부 CTV 자료는 접근 실패 때문에 full-text 근거로 사용하지 않았다. Apollo 7 companion postflight trajectory report는 링크를 확인했으나 상세 reconstruction 검토를 완료한 자료가 아니다.

Source에서 빠진 정보는 임의로 채워 논문 reproduction이라 표시하지 않는다. 제3자 논문·그림·데이터의 권리는 저장소 코드 license와 별개다. [Third-party notices](../THIRD_PARTY_NOTICES.md)와 [citation metadata](../CITATION.cff)를 함께 적용한다.
