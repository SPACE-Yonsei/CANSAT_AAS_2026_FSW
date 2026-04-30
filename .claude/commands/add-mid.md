---
description: lib/appargs.py에 새 MID와 AppID를 등록한다
allowed-tools: Read Edit
argument-hint: "<SenderAppName> <ReceiverAppName> <MID_suffix> (예: Imu Motor imu_heading)"
---

`lib/appargs.py`에 새 MID를 등록하라. 인자: `$ARGUMENTS`

## MID 명명 규칙
```
MID = <SenderAppID><ReceiverAppID_2자리><순번_2자리>
예: Imu(14) -> Motor(19): MID = 1401901, 1401902, ...
예: Barometer(13) -> Comm(12): MID = 1301201
```

## 수행 절차
1. `lib/appargs.py`를 읽어 현재 Sender 클래스의 최대 MID 순번 확인
2. 충돌 없는 다음 번호로 MID 값 결정
3. 해당 Sender 클래스에 `MID_<receiver>_<suffix>: int = <value>` 추가
4. 추가 후 전체 MID 목록에서 중복 없음을 확인
5. 변경 내용 요약 출력

## 현재 AppID 매핑 (참조용)
| AppID | 클래스 |
|-------|--------|
| 10 | MainAppArg |
| 11 | FlightlogicAppArg |
| 12 | CommAppArg |
| 13 | BarometerAppArg |
| 14 | ImuAppArg |
| 15 | GpsAppArg |
| 16 | DistanceAppArg |
| 17 | ElectroAppArg |
| 18 | CameraAppArg |
| 19 | MotorAppArg |
