# CAD 생성·검토

설계 원본은 build123d Python이다. 공통 데이터는 `cad/layout.json`(배치·설계 파라미터), `cad/parts.json`(구매품 사양)에서 관리한다. `cad/parameters.py`가 두 파일을 읽는다. `parts/`는 부품별 생성, `assembly.py`는 조립, `export.py`는 선택 출력, `validation.py`는 검증을 담당한다. 부품 함수를 불러오는 것만으로 CAD 파일을 만들거나 GUI를 열지 않는다.

## Windows 시작

```powershell
powershell -ExecutionPolicy Bypass -File tools/setup_cad.ps1
.venv\Scripts\python.exe -m cad list
.venv\Scripts\python.exe -m cad build --part motor_left --format step,stl --individual
.venv\Scripts\python.exe -m cad build --part suspension --format none
.venv\Scripts\python.exe -m cad build --format step,stl --individual
.venv\Scripts\python.exe -m cad validate --collisions
.venv\Scripts\python.exe -m pytest
```

출력은 `output/cad/`(Git 제외), 개별 부품은 `output/cad/parts/`이다. `--part`를 여러 번 지정하면 해당 그룹만 생성한다. `--format none`은 파일 출력 없이 검사한다. `--output`으로 다른 출력 위치를 지정한다. 정상 결과는 한 줄, 상세는 JSON 보고서다. `tools/cad.ps1`도 같은 인수를 받는다.

AI 없이 주요 치수를 변경하는 예:

```powershell
.venv\Scripts\python.exe -m cad param parameters.base_length 530
.venv\Scripts\python.exe -m cad build --part chassis --format step
```

구조화된 좌표·구매품 변경은 JSON을 편집한다. 자동으로 다른 부품 치수를 추정하지 않는다. 판재/기둥/상판의 결합 위치를 바꾸면 관련 파라미터와 전체 조립 검증도 함께 갱신한다. 서스펜션 하드웨어는 현재 개념 치수이며 임의의 스트로크 확장을 제작 승인으로 취급하지 않는다.

## 기준 비교와 복원

```powershell
.venv\Scripts\python.exe -m cad compare
.venv\Scripts\python.exe -m cad build --variant baseline --name amr_rev05_build123d
py -3.10 cad/amr_layout_cadquery.py
```

마지막 명령은 별도의 기존 CadQuery2.5.2 환경을 사용한다. build123d 가상환경에 CadQuery를 섞어 설치하지 않는다. 기준은 `61483e8`, 복원 브랜치 `backup/cadquery-rev05-20261111`, 원본 STEP/JSON은 `cad/baseline/rev05/`다. Rev05 **85부품의 동일 형상 전환**과 Rev06 **공식 CAD·6륜 서스펜션 설계 변경**을 별도로 평가한다. 새 구매품 CAD를 단순화한 Rev05와 동일하다고 주장하지 않는다.

## FreeCAD

```powershell
powershell -ExecutionPolicy Bypass -File tools/review_freecad.ps1
powershell -ExecutionPolicy Bypass -File tools/review_freecad.ps1 -Headless -Wait
```

기본 STEP은 `output/cad/amr_current.step`이다. `-StepPath <file>`로 개별 부품도 검토한다. GUI에서는 트리의 부품명·조립 그룹을 확인하고, 클리핑/단면 및 측정 도구로 검토한다. 자동 매크로는 STEP을 불러오고 유효성을 검사하여 `output/freecad/amr_review.FCStd`와 `review_report.json`을 저장한다. GUI 직접 실행 시 `cad/freecad/ImportAMR.FCMacro`, FreeCADCmd는 `import_amr.py`를 사용한다. FCStd는 검토 사본이며 build123d 코드로 변경을 자동 반영하지 않는다.

## 검증 범위

- BRep 유효성, 양의 체적, 부품 개수, 6륜 공칭 접지, Ø180×42.5, STEP 재불러오기.
- 카메라 STEP은 일반 불러오기에서 위상 공차 복원이 필요하다. 보고서의 `raw_import_valid`와 `valid_after_explicit_normalization`을 구분한다. 제공 FreeCAD 매크로는 카메라에만 최소 위상 공차0.00001mm를 복원하고 체적 불변을 검사한다. 일반 STEP 불러오기 자체가 모두 정상이라고 주장하지 않는다.
- Rev05 비교: 이름별 경계/질량중심/체적/솔리드 수/양방향 차집합. 기본 허용 경계오차0.0001mm, 체적 상대1e-8 또는0.001mm³. `--fast`는 차집합을 생략한다.
- `--collisions`: 센서 간섭 및 서스펜션 스트로크의 선별 간섭 검사. 체결부 의도된 접촉과 구매품 내부의 형상을 전수 간섭 검사로 오해하지 않는다.
- 강도/피로/제동/스프링 상수/실제 접지하중/로봇암 동하중은 별도 검증 대상이다.

GitHub Actions는 같은 CLI와 검사를 사용하고 `cad-output/`에 배포 STEP·검증 결과를 갱신한다. 기존 웹 뷰어는 계속 Three.js를 사용한다.
