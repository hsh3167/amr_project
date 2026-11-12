# 대학생 AMR 프로젝트 — build123d / Rev08

2WD 차동구동 + 캐스터 4개, **30kg 기준 6륜 독립 서스펜션**입니다. 구동륜은 미스미 샤프트와 부시, GL-65 캐스터는 각 1세트의 와이드 레일로 안내합니다. 상판 윗면은 지면에서 287mm입니다. 설계 원본은 build123d Python과 JSON이며, FreeCAD에서 측정·단면·조립을 검토합니다. [서스펜션 상세](docs/suspension-misumi.md) · [미스미 구매 BOM](bom/suspension_misumi.csv)

- [3D 뷰어](https://hsh3167.github.io/amr_project/) · [최신 STEP](cad-output/amr_current.step) · [검증 결과](cad-output/build_report.json)
- [최신 설계 상태](design_state.md) · [생성/수정/FreeCAD 실행 방법](docs/build123d-workflow.md)
- [제조사 CAD와 버전 차이](docs/vendor-cad.md) · [기구·전원 검토](docs/engineering-decisions.md) · [BOM](bom/concept_bom.csv)
- [원본 Rev05/변환 비교](cad/baseline/rev05/comparison_build123d.json) · [Rev05 설계 이력](docs/README-rev05.md)

```powershell
powershell -ExecutionPolicy Bypass -File tools/setup_cad.ps1
.venv\Scripts\python.exe -m cad build --part motor_left --format step,stl --individual
.venv\Scripts\python.exe -m cad build --format step,stl,glb --individual
.venv\Scripts\python.exe -m cad validate --collisions
powershell -ExecutionPolicy Bypass -File tools/review_freecad.ps1
```

필요한 부품만 생성하고 출력 형식을 선택할 수 있습니다. JSON 편집 또는 `python -m cad param parameters.base_length 530`으로 AI 없이 치수를 변경합니다. `--pose LEFT=10 --pose LF=-5`로 독립 스트로크 위치를 생성합니다. 출력은 `output/cad/`, 개별 부품은 그 아래 `parts/`입니다. FreeCAD 검토 사본은 `output/freecad/amr_review.FCStd`입니다.

확정 구매품은 ZLLG65ASM250-N V2 2개/Ø180, GL-65-ASF 4개, SICK TiM571-2050101 2개, Astra Mini Pro MX6000 A V1.2 2개입니다. 대각 LiDAR, 카메라 위치, 판재 크기와 좌표계는 Rev05를 유지합니다. 모든 수직 구조 기둥은 Al2020 범용 프로파일입니다. 미래 로봇암용 상판/보강/체결 영역은 별도 조립 그룹이며 실제 암 볼트 패턴·반력·전도는 미검증입니다.

Rev05 85부품의 순수 엔진 전환을 먼저 비교 검증했습니다. Rev06은 공식 캐스터 STEP(EUS 참고) 및 카메라 제품군 STEP을 연결하고 서스펜션 구조를 추가한 별도 설계 변경입니다. 모든 구매품이 정확한 제조사 STEP인 것은 아닙니다. 모터 케이스Ø140, LiDAR 상세 외형, 범용 프로파일, 전장품·배터리, 브래킷은 확인 한계/가정이 남습니다.

**카메라 STEP은 일반 불러오기에서 위상 공차 복원이 필요합니다. 제공 FreeCAD 매크로가 이를 자동 처리합니다.** 원본 불러오기/복원 후 결과는 검증 보고서에 구분합니다.

캐스터는 제조사 CAD 높이83mm와 카탈로그82mm의 차이가 있습니다. 카메라 3D V0.1은 깊이24.264mm, 2D V1.2는19.92REF입니다. 구동륜±10/캐스터±5mm에서 미스미 스프링의 공칭 장착 길이와 변형 여유를 검토했습니다. 실차 접지하중, 감쇠, 스토퍼 충격 강도, 체결과 피로 검증이 남아 있습니다.

모터 정격24V/사용18–30V,4Nm/500rpm/10A,스톨11.7Nm/30A,1024라인은 수신 사양서를 유지합니다. 4.2V 완충 리튬이온7S라면29.4V로30V까지0.6V뿐입니다. **모터 사용 범위와 드라이버 허용전압은 별개**이며 회생전압·BMS·클램프를 함께 선정합니다. TiM571 최대28V에는 안정화24V를 공급합니다. [사양서 충돌·축 상세](docs/motor-datasheet-review.md)를 참고합니다.

GitHub Actions는 동일한 build123d CLI로 기준 비교/검사 후 STEP과 웹 표시용 GLB를 생성합니다. Three.js 뷰어는 GLB를 읽고 두 LiDAR 외형에는 SICK TiM5xx 공용 STL을 표시합니다. [형상 출처와 정렬 기준](docs/vendor-cad.md). CadQuery2.5.2 기존 환경과 고정 Rev05 스크립트 `cad/amr_layout_cadquery.py`, 기준 STEP은 복원을 위해 보존합니다.
