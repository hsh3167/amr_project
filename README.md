# 대학생 AMR 프로젝트 — Rev 0.3

**6-wheel differential mobile base / 구조 레이아웃 검토용.** 확정 부품과 1차 가정 치수를 구분합니다. 현재는 제작도면이 아닌 CAD 선행배치 모델입니다.

## 3D 모델 보기

- [브라우저 3D 레이아웃 (GitHub Pages)](https://hsh3167.github.io/amr_project/) — 좌우 회전·확대·TOP/FRONT·부품 숨기기·분해 보기·PNG 저장·개념 모션
- [CadQuery 파라메트릭 생성 소스](cad/amr_layout.py)
- [최신 3D STEP](cad-output/amr_6w_layout_rev03.step) — GitHub Actions가 자동 생성함
- [1차 BOM](bom/concept_bom.csv)
- [설계 판단 및 리스크](docs/engineering-decisions.md)

> **중요:** 웹의 주행 모션은 시각적 데모일 뿐 실제 차량 동역학/접지력/SLAM/간섭/강도 검증이 아닙니다. STEP 모델은 제조사 CAD 원본이 아니며, 부품 간섭 및 체결부 실측 전 제작 금지. 브라우저 3D는 Three.js CDN 연결이 필요합니다.

## 확정 부품 (기본 수량)

| 분류 | 품명 | 수량 | 확정/카탈로그 정보 | 아직 미확인 |
|---|---|---:|---|---|
| 구동륜 | **ZLLG65ASM250-N V2** | 2 | **Ø180mm**(프로젝트 사용자 정정), 고무타이어, 24V, 브레이크 없음(프로젝트 조건) | 정확한 N V2의 실물 치수, 트레드폭, 축/마운트, 전력/토크/드라이버 |
| 캐스터 | **GL-65-ASF** | 4 | Ø64mm, 설치 높이 82mm, 옵셋 22mm, 플레이트 65×48mm, 피치 54×24mm, 7mm 홀, 정격하중 50kg/개 | 휠 종류 suffix: EUS(고탄성 우레탄) vs TUN(사출 우레탄), 장착부 실측 |
| LiDAR | **SICK TiM571-2050101** | 2 | 약 76.25×74.39×101.12mm 외형 범위, 270° 스캔, 공급전압 9–28V | 실제 광학 스캔면, 커넥터/케이블 공간, 시야 가림 |
| Camera | **Orbbec Astra Mini Pro / MX6000 A V1.2** | 2 | 84.90×20.00×19.92mm, 깊이 시야각 H58.4°, USB2.0 | 카메라 각도, 케이블 굴곡 반경, 마운트 체결 위치 |
| 배터리 | 보유 회사 7S9P | 1(후보) | 7 직렬·9 병렬 토폴로지만 확인 | 셀 화학종/규격/BMS/전압/최대전류/잔존용량/크기/충전기 |

**참고:** 공개 검색에서 `ZLLG65ASM250-N V2`의 정확한 **제조사 원본 도면을 확보하지 못했으므로**, 유사모델 `-4096 V2`의 사양을 절대로 혼용하지 않습니다. Ø180mm는 사용자 확인 치수입니다.

## 현재 레이아웃 — 미확정 치수 모두 [가정]

| 항목 | 설정 |
|---|---|
| 본체 전체 엔벨로프(센서 포함) | 약 W470×L610×H420mm |
| 상판 | 420×550×t4mm, 탈착식 |
| 하부판 | 390×530×t4mm, 중앙 휠 웰 |
| 20×20 프로파일 | 상하판 연결 4개 |
| 구동륜 센터 트랙 | 422mm |
| 구동륜 중심 높이 | Ø180/2 = **90mm** |
| 캐스터 중심 | X±138mm, 종방향 Y±215mm |
| 캐스터 어댑터 | 상판/하판 높이 차이를 위한 약 34mm 드롭 브라켓 (추정) |
| LiDAR | 전방(+Y), 후방(-Y) 각각 1개, 상판 높이 위치 |
| 카메라 | 전방 좌우 각 1개, ±23° 대각 배치 (가설계) |
| 향후 로봇암 | 상판 중앙 예비 체결영역 150×150mm |
| 배터리 더미 | 235×155×95mm, 낮은 위치 중심 근처 |

## 캐스터 접지 · 모터 높이

현재 모터는 지면에서 중심고 90mm로 배치하며, 캐스터 GL-65-ASF의 **설치 높이 82mm**에 맞춰 하부판 아래에 **드롭 브라켓(가설계)**을 설정했습니다. 6개 휠의 동시 접지/구동륜 수직반력은 아직 검증되지 않았습니다. 가공 전 캐스터 시밍/높이 조절 기능을 확보하고, 실측 바퀴 반지름과 타이어 처짐을 반영해 계산해야 합니다.

## 배터리 및 전원

기존 7S9P 팩은 셀 종류·BMS·충전기·사용승인을 확인한 뒤 사용합니다. 7S 리튬이온(셀당 공칭 3.6V **가정**)은 공칭 25.2V, 최대 29.4V입니다. 3Ah 셀이라면 27Ah, 약 680Wh가 계산됩니다. 실제 BMS 전류, 회생 전압, 모터 드라이버 입력 허용범위, 보호퓨즈, 비상정지 회로를 사전 검토하세요. 브레이크 없는 AMR은 시험 초기 평지에서만 운용합니다.

## STEP 재생성

```bash
pip install cadquery==2.5.2
python cad/amr_layout.py
# cad-output/amr_6w_layout_rev03.step
```

GitHub Actions에서도 STEP이 자동 생성됩니다. 부품 고유 CAD가 없는 요소는 단순 입체 더미이며 실제 제조용 정확도를 보장하지 않습니다.

## 검증 및 후속 작업

1. **N V2 모터의 제조사 도면 또는 실물 측정**: Ø180 기준 외 타이어 폭, 샤프트 형상, 외측 허브 깊이, 배선 출구, 드라이버 모델 확인
2. **GL-65-ASF 휠 종류 확인**: EUS 혹은 TUN, 실제 강성/접지/스위블 저항 검토
3. LiDAR 2개 스캔면 높이와 카메라 2개 FoV 및 상호 간섭 검토
4. 배터리 및 전장 실물 크기 확정
5. 하부/상판 브라켓과 체결 홀을 도면화하여 FreeCAD·SolidWorks 등에서 실제 조립 검증

## 공식 제조사 자료
- [SICK TiM571-2050101 PDF](https://www.sick.com/media/pdf/4/44/444/dataSheet_TiM571-2050101_1075091_ko.pdf)
- [Orbbec Astra Mini Pro](https://store.orbbec.com/products/astra-mini-pro)
- [GL-65-ASF EUS 옵션](https://www.gdok.co.kr/sub/caster_detail.php?gd_model_type=%ED%8F%89%ED%8C%90%EC%B7%A8%EB%B6%80%ED%98%95&gd_type2=EUS&gm_code=pd15&gs_code=&gs_name=GL-65&gs_type=GL)
- [GL-65-ASF TUN 옵션](https://www.gdok.co.kr/sub/caster_detail.php?gd_model_type=%ED%8F%89%ED%8C%90%EC%B7%A8%EB%B6%80%ED%98%95&gd_type2=TUN&gm_code=pd07&gs_code=&gs_name=GL-65&gs_type=GL)
- [ZLTECH 250 시리즈 참고(동일 N V2 아님)](https://zltech.co.kr/product/view.php?pcode=PROD00000000098)
