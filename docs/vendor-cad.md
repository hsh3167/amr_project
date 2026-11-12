# 제조사 형상과 확인 한계

원본은 `cad/vendor/*.stp.gz`에 무손실 압축 보관한다. 원본 바이트 SHA256·출처는 `provenance.json`에 기록한다. 캐시에서 압축을 풀어 읽으며 단위는 mm, 형상 스케일 변경은 없다.

카메라의 기구 CAD 파생본은 원본58개 솔리드를 사용하고 3개 열린 참고 셸은 제외한다. 배치 후 필요한 최소 위상 공차는0.00001mm이며 3D 표면/정점 위치와 체적을 바꾸지 않는다. STEP 재저장 시 이 공차 하한이 사라져 일반 불러오기에서는 일부 솔리드가 유효하지 않다고 표시될 수 있다. 이 한계는 해결된 것으로 숨기지 않는다. `cad/export.py`는 원본 불러오기 유효성과 카메라에 한정한 명시적 공차 정규화 후 유효성을 구분하여 기록한다. FreeCAD 매크로도 동일 처리를 수행하고 원래 불량 표시 이름/정규화 이름을 기록한다. 다른 CAD 프로그램에서 직접 열 때도 확인이 필요하다. FCStd 검토 사본은 복원된 위상 공차를 보존한다.

| 부품 | 반영 데이터 | 남은 확인 |
|---|---|---|
| GL-65-ASF | [제조사 ZIP](https://www.gdok.co.kr/data/product/gd_file3_2024062409450217.zip)의 GL-65-ASF-EUS STEP, 3솔리드 | EUS/TUN 실물 suffix. CAD 설치높이83, 카탈로그82의 차이. 휠+허브 총폭30.4와 트레드23 구별 |
| Astra Mini Pro | [Orbbec 공식 다운로드](https://www.orbbec.com.cn/index/Download2025/info.html?cate=121&id=1)에서 연결한 [제조사 폴더](https://xm917ch2uk.feishu.cn/drive/folder/X6WrfJsbaloeSKdTTAicOlFVn19)의 3D V0.1/20221012 STEP, 58솔리드 | 2D V1.2/20221019와 깊이 차이. 구매품 MX6000 A V1.2 라벨/기판·마운트 확인 |
| SICK TiM571-2050101 | 웹 뷰어의 두 LiDAR는 SICK TiM5xx 공용 STL. 기구 STEP은 공식 치수도 기반 엔벨로프 | [품번 1075091 공식 다운로드](https://www.sick.com/us/en/catalog/products/lidar-and-radar-sensors/lidar-sensors/tim/tim571-2050101/p/p412444?tab=downloads)의 CAD 항목은 [EPLAN EDZ](https://www.sick.com/media/eplan/SICK.1075091.edz) 1건. 정확한 품번의 기구 STEP과 [SICK AAS](https://aas.sick.com/) 로그인 후 제공 여부 미확인 |
| ZLLG65ASM250-N V2.02 | 수신23쪽 사양서 치수 형상 유지 | 정확한 제조사 STEP 미확보. Ø140 케이싱은 계속 가정. 다른4096/L/V3 모델로 대체하지 않음 |
| Al2020 | 범용 T슬롯 파라메트릭 형상 | 구매 제조사/단면/재질등급·접합 상세 미선정 |

캐스터 원본 장착판 중심을 XY0으로 옮기고 바닥을 Z0으로 만든 뒤 Z축 -90° 회전한다. 따라서 조립에서 판48X×65Y, 홀24X×54Y, 축 X방향/오프셋Y-22다. 원본 CAD83mm를 임의로82mm로 줄이지 않는다.

LiDAR 외형은 SICK `sick_scan_xd`가 TiM57x에 사용하는 [공용 STL 원본](https://github.com/SICKAG/sick_scan_xd/blob/0c35a36cc4cf40d2a04325e2145112c27623acd1/meshes/sick_tim_5xx.stl)이다. [로컬 사본](../web/vendor/sick/sick_tim_5xx.stl)과 [Apache 2.0 라이선스](../web/vendor/sick/LICENSE)를 함께 보관한다. ROS 좌표와 m를 뷰어 좌표와 mm로 변환하고, 열린 메시의 최저점을 기존 장착판 Z163에 맞춘 뒤 기존 방위각을 적용한다. 이 배치에서 STL 원점은 Z225.96이며 FOV는 기존 설계 스캔면 Z240.83에 표시한다. 실물 체결면과 광학 원점의 관계는 미검증이다.

카메라 원본 경계84.9(X)×20(Y)×24.264(Z)mm를 유지한다. 원본 광학방향+Z를 조립의 로컬+Y로, 높이Y를+Z로 회전하고 경계 중심을 기존 카메라 중심에 맞춘다. 이후 기존 yaw±15°/pitch하향10°를 적용한다. 2D 도면은84.90±0.30×20.00±0.25×19.92REF, 금속 탭2×Ø2.20/M2, 피치80±0.30이다. 3D와 도면 깊이 차이4.344mm는 버전 차이로 남긴다. 도면과 맞추려고 제조사 모델을 변형하지 않는다. 카메라 어댑터는 여전히 개념 형상이며 이 홀 기준으로 상세 설계가 필요하다.
