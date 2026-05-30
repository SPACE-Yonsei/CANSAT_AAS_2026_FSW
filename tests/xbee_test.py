"""XBee 양방향 통신 테스트 스크립트 (지상국 PC에서 실행).

사용법:
  python tools/xbee_test.py             # 자동 포트 탐색
  python tools/xbee_test.py COM5        # 포트 직접 지정
  python tools/xbee_test.py COM5 38400  # 포트 + 보드레이트 지정

모드:
  r  수신 전용 (기체 TLM 수신 확인)
  s  송신 전용 (기체에 CMD 전송)
  b  양방향 동시 (기본)
"""

from __future__ import annotations

import sys
import threading
import time

BAUD = 38400
TEST_CMD = "CMD,1070,CX,ON\n"
RECV_TIMEOUT = 5.0


def _open_port(port: str, baud: int):
    import serial
    ser = serial.Serial(port, baud, timeout=1)
    print(f"[OK] 포트 열림: {port} @ {baud}")
    return ser


def _discover_ports() -> list[str]:
    try:
        from serial.tools import list_ports
        return [p.device for p in list_ports.comports()]
    except Exception:
        return []


def rx_loop(ser, stop: threading.Event) -> None:
    last_rx = time.time()
    print("[RX] 수신 대기 중 (Ctrl+C로 종료)...")
    while not stop.is_set():
        try:
            raw = ser.readline()
        except Exception as e:
            print(f"[RX ERROR] {e}")
            break
        if raw:
            line = raw.decode("utf-8", errors="ignore").strip()
            if line:
                elapsed = time.time() - last_rx
                print(f"[RX +{elapsed:.1f}s] {line}")
                last_rx = time.time()


def tx_loop(ser, stop: threading.Event) -> None:
    print(f"[TX] 3초마다 전송: {TEST_CMD.strip()}")
    while not stop.is_set():
        try:
            ser.write(TEST_CMD.encode())
            print(f"[TX] 전송됨")
        except Exception as e:
            print(f"[TX ERROR] {e}")
            break
        time.sleep(3.0)


def main() -> None:
    port = sys.argv[1] if len(sys.argv) > 1 else None
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else BAUD
    mode = sys.argv[3].lower() if len(sys.argv) > 3 else "b"

    try:
        import serial  # noqa: F401
    except ImportError:
        print("[ERROR] pyserial 미설치: pip install pyserial")
        sys.exit(1)

    if port is None:
        ports = _discover_ports()
        if not ports:
            print("[ERROR] 사용 가능한 시리얼 포트 없음. 포트를 직접 지정하세요.")
            sys.exit(1)
        print(f"[INFO] 발견된 포트: {ports}")
        port = ports[0]
        print(f"[INFO] {port} 사용 (다른 포트: python xbee_test.py <포트>)")

    try:
        ser = _open_port(port, baud)
    except Exception as e:
        print(f"[ERROR] 포트 열기 실패: {e}")
        sys.exit(1)

    stop = threading.Event()
    threads = []

    if mode in ("r", "b"):
        t = threading.Thread(target=rx_loop, args=(ser, stop), daemon=True)
        threads.append(t)
    if mode in ("s", "b"):
        t = threading.Thread(target=tx_loop, args=(ser, stop), daemon=True)
        threads.append(t)

    for t in threads:
        t.start()

    print(f"[INFO] 모드={mode.upper()}  Ctrl+C로 종료")
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        ser.close()
        print("\n[종료]")


if __name__ == "__main__":
    main()
