# Saito paper-study API

공개 tables, entry-state constructors, uncertainty grids 및 Eq.10–26 helpers를 제공한다. 미공개 aerodynamic/guidance 정보를 채운 flight reproduction은 포함하지 않는다.

```matlab
audit = paperstudies.saito.run();
status = paperstudies.saito.status();
```

기본 실행은 source/equation audit이다. `run(struct('forward',true))`는 별도 가정을 사용하는 surrogate forward propagation이다.

모델과 누락 입력은 [기체 모델](../../docs/VEHICLE_MODELS.md), 실행법은 [사용 설명서](../../docs/USER_GUIDE_KR.md), 원출처는 [자료 목록](../../docs/REFERENCES.md)에 정의한다.
