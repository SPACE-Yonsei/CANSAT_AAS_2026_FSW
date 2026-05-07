# GCS 시뮬레이션 테스트 리스트

FSW는 라인 단위로 `CMD,1070,<본문>`을 수신한다. **본문**은 아래 표의 **Body** 열과 동일하게 GCS 입력란에 넣으면 된다 (전송 시 `CMD,1070,`가 자동으로 붙음).

- 참조: `comm/commapp.py:_dispatch_command`, `ground_station/ground_station.py`
- 대소문자 무관 (`ss,3` 가능)
- 팀 ID `1070` 고정 (`TEAM_ID`)

---

## 범례

| 열 | 의미 |
|----|------|
| **#** | 시나리오 번호 |
| **Body** | GCS `Body` 입력란에 넣을 문자열 (줄마다 순서대로 전송) |
| **기대 / 비고** | 텔레메트리·로그·상태에서 확인할 점 |

---

## 1. 통신·텔레메트리 기본

| # | Body | 기대 / 비고 |
|---|------|-------------|
| 1.1 | `CX,ON` | TLM 송신 시작 (`TELEMETRY_ENABLE`), 주기 CSV 수신 |
| 1.2 | `CX,OFF` | TLM 중지 (모드 컬럼 등은 유지될 수 있음) |
| 1.3 | `CX,ON` | 다시 ON |

---

## 2. 시간 동기 (ST)

| # | Body | 기대 / 비고 |
|---|------|-------------|
| 2.1 | `ST,GPS` | `ST_timedelta` 리셋, TLM `time` 필드가 **현지 시계** 기준으로 맞춤 |
| 2.2 | `ST,12:34:56` | 원하는 시각으로 오프셋 (형식 `HH:MM:SS`). 성공 시 `cmd_echo`에 반영 |

---

## 3. SIM 모드 — 활성화 / 비활성

| # | Body | 기대 / 비고 |
|---|------|-------------|
| 3.1 | `SIM,ENABLE` | TLM `mode` → **A** (준비), SIM GPS 클리어 트리거 |
| 3.2 | `SIM,ACTIVATE` | `mode` → **S** (시뮬 활성). **이후에만** SIMP/SIMG가 FL·GPS에 반영 |
| 3.3 | `SIM,DISABLE` | `mode` → **F**, 실제 GPS 경로 복귀 |
| 3.4 | `SIM,ENABLE` → 잠시 후 `SIM,DISABLE` | ENABLE만 하고 ACTIVATE 없이 끄기 — SIMP/SIMG는 영향 없었어야 함 |

---

## 4. SIM — 기압고도 주입 (SIMP)

**전제:** `SIM,ACTIVATE` 이후

| # | Body | 기대 / 비고 |
|---|------|-------------|
| 4.1 | `SIMP,120` | 시뮬 고도 120 m 상태머신 입력, TLM `altitude_m` 즉시 반영 (`comm`이 `tlm_data` 갱신) |
| 4.2 | `SIMP,350` | 고도 변화에 따른 상태 전이(카운터) 점검용 |
| 4.3 | `SIMP,0` | 저고도 |

**부정 시나리오 (SIM 비활성):**

| # | Body | 기대 / 비고 |
|---|------|-------------|
| 4.N1 | (`SIM,DISABLE` 후) `SIMP,100` | FL `handle_simp`에서 무시 — 상태/고도 로직에 시뮬 고도 미반영 |

---

## 5. SIM — GPS 주입 (SIMG)

**전제:** `SIM,ACTIVATE` 이후  
형식: `SIMG,lat,lon,course_deg,speed_m_s` 또는 `SIMG,lat,lon,course_deg,speed_m_s,alt_m`

| # | Body | 기대 / 비고 |
|---|------|-------------|
| 5.1 | `SIMG,37.56,126.93,90,8.5` | 시뮬 속도·항로·위치가 GPS 앱으로 전달, TLM `gps_lat/lon` 등 갱신 |
| 5.2 | `SIMG,37.56,126.93,90,8.5,100` | 5번째 인자 alt 포함 |
| 5.3 | (좌표를 천천히 바꿔가며 여러 번 SIMG) | GCS 지도 궤적·`start/target/carrot` 필드가 있으면 맵·헤딩 표시 확인 |

**부정:**

| # | Body | 기대 / 비고 |
|---|------|-------------|
| 5.N1 | (`SIM,ENABLE`만) `SIMG,37.56,126.93,90,8.5` | ACTIVATE 전이면 FL에서 무시·경고 로그 가능 |
| 5.N2 | `SIMG,0,0,0,0` | `(0,0)` 좌표는 `cmd_simg`에서 거절 |

---

## 6. 목표 좌표 (TC) — 릴리즈 전 필수

| # | Body | 기대 / 비고 |
|---|------|-------------|
| 6.1 | `TC,37.57,126.94` | `prevstate` 타깃 + 모터 타깃 IPC, TLM `target_lat/target_lon` |
| 6.2 | `TC,-35.0,149.0` | 유효 범위 내 임의 좌표 |

**부정:**

| # | Body | 기대 / 비고 |
|---|------|-------------|
| 6.N1 | `TC,0,0` | FL에서 거절 (안전 정책) |

---

## 7. 상태 강제 점프 (SS)

| # | Body | 기대 / 비고 |
|---|------|-------------|
| 7.1 | `SS,0` | LAUNCH_PAD |
| 7.2 | `SS,1` | ASCENT (FL에서 카메라 활성 메시지 등) |
| 7.3 | `SS,2` | APOGEE |
| 7.4 | `SS,3` | RELEASE — **유효한 TC가 있어야 함** |
| 7.5 | `SS,4` | EGG |
| 7.6 | `SS,5` | LANDED |

**릴리즈 차단 (중요):**

| # | Body | 기대 / 비고 |
|---|------|-------------|
| 7.N1 | (타깃 미설정 상태) `SS,3` | `to_release()`에서 **전이 거부**, 로그에 target 필요 메시지 |
| 7.N2 | `TC,37.57,126.94` 후 `SS,3` | 정상 전이·번와이어/모터 메시지 순서는 FSW 구현 따름 |

---

## 8. 바로미터 보정 (CAL)

| # | Body | 기대 / 비고 |
|---|------|-------------|
| 8.1 | `CAL,` | 빈 옵션 — 현재 `ALTITUDE`를 오프셋에 반영 (제로셋). Baro → FL로 `CAL` 릴레이 |
| 8.2 | `CAL,0.5,1.2` 등 (데이터에 `,` 포함 형식) | Baro 앱: `parts` 길이에 따라 오프셋 누적 분기 — `Sensor_Barometer/barometerapp` 동작 확인 |

---

## 9. 기구 (MEC)

| # | Body | 기대 / 비고 |
|---|------|-------------|
| 9.1 | `MEC,ON` | Motor 앱 `MID_RouteCmd_MEC` |
| 9.2 | `MEC,OFF` | 동일 |

---

## 10. 카메라 (CAM)

| # | Body | 기대 / 비고 |
|---|------|-------------|
| 10.1 | `CAM,ON` | Camera 앱 녹화 플래그 ON, `PICAM_Video/` 세그먼트 (Pi + picamera2일 때) |
| 10.2 | `CAM,OFF` | 녹화 중지 |

---

## 11. XBee 리셋 (XRST)

| # | Body | 기대 / 비고 |
|---|------|-------------|
| 11.1 | `XRST,NOW` | `xbeereset.send_reset_pulse()` — **보드 GPIO 배선 필요** |
| 11.2 | `XRST,ON` | 구현상 `NOW`/`1`/`ON` 동치 |

---

## 12. 엔드투엔드 벤치 시퀀스 (문서 권장 순서)

연속으로 보내며 지도·상태·모드 확인. 좌표는 테스트 지역에 맞게 수정.

```
CX,ON
SIM,ENABLE
SIM,ACTIVATE
SIMP,120
SIMG,37.56,126.93,90,8.5
TC,37.57,126.94
SS,3
CAM,ON
MEC,OFF
SIM,DISABLE
CX,ON
```

- `SS,3` **전에 반드시 `TC,...`** 가 있어야 함.
- 미션 끝에 `SIM,DISABLE`으로 실기 GPS 경로로 복귀.
- `SS,4`/`SS,5` 등으로 낙하·착륙 시나리오를 이어서 테스트할 수 있음.

---

## 13. 상태 기계 + 고도 시뮬 (자동 전이 점검)

SIM ACTIVATE 후 SIMP로 고도를 단계적으로 올리며 **자동 전이**를 유도 (FSW 카운터·임계값은 `flight_logic/flightlogicapp.py` 참고).

| 순서 | Body | 의도 |
|------|------|------|
| 1 | `SIM,ENABLE` | 준비 |
| 2 | `SIM,ACTIVATE` | SIM on |
| 3 | `SIMP,50` | 상승/초기 구간 |
| 4 | `SIMP,200` | 고고도·맥스 고도 갱신 등 |
| 5 | `SIMP,80` | 낙하대 |
| … | (문서화된 `max_alt` 비율·`cnt_*` 조건에 맞게 반복) | RELEASE/APOGEE/EGG 등 |

---

## 14. FSW에만 있고 GCS 프리셋에 없는 명령

| Body | 비고 |
|------|------|
| `RBT,...` | 원격 재부트 — **토큰·시퀀스·안전 상태** 환경변수 필요. GCS 기본 프리셋 없음. |

---

## 15. 체크리스트 (한 줄)

시험 완료 시 표시:

- [ ] CX ON/OFF
- [ ] ST GPS / ST 시각
- [ ] SIM ENABLE → ACTIVATE → SIMP/SIMG → DISABLE
- [ ] SIMG without ACTIVATE → 무시 확인
- [ ] TC 후 SS,3 / TC 없이 SS,3 차단
- [ ] TC `0,0` 거절
- [ ] CAL 빈 본문
- [ ] MEC ON/OFF
- [ ] CAM ON/OFF
- [ ] SS 0–5 각각
- [ ] XRST (하드웨어 있을 때만)
- [ ] CSV 로그 저장·콘솔 `cmd_echo` 일치
