---
description: TLM 패킷 CSV 필드와 tlm_data 딕셔너리의 정합성을 검증한다
allowed-tools: Read Grep
---

`comm/commapp.py`의 텔레메트리 스키마 정합성을 검증하라.

## 검증 항목

### 1. tlm_data 필드 목록화
`command_handler`에서 `tlm_data[...]` 에 할당되는 모든 키를 추출.

### 2. send_tlm CSV 필드 목록화
`send_tlm` 함수에서 실제 CSV 문자열로 직렬화되는 필드 순서와 이름 추출.

### 3. 누락/불일치 탐지
| 항목 | 확인 결과 |
|------|-----------|
| tlm_data에 있지만 CSV에 없는 필드 | ? |
| CSV에 있지만 tlm_data에 없는 필드 | ? |
| cmd_echo 뒤 빈 콤마 삽입 버그 | ? |

### 4. AAS 2026 규격 필드 확인
TEAM_ID, MISSION_TIME, PACKET_COUNT, MODE, STATE,
ALTITUDE, TEMPERATURE, PRESSURE, VOLTAGE, CURRENT,
GPS_TIME, GPS_ALTITUDE, GPS_LATITUDE, GPS_LONGITUDE, GPS_SATS,
TILT_X(roll), TILT_Y(pitch), ROT_Z(yaw), CMD_ECHO

### 5. 좌표 정밀도 확인
`lat`, `lon` 포맷 문자열이 소수점 5자리 이상인지 확인. 미달 시 수정 코드 제시.
