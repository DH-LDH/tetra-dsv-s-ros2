#!/usr/bin/env python3
"""TETRA-DSV-S 구동보드 통신 확인 — ROS 없이 단독 실행됩니다.

젯슨에서 ROS 를 띄우기 전에 이걸 먼저 돌리세요. ROS 노드가 조용히 안 움직일 때
원인이 시리얼인지 ROS 인지 가르는 데 30초면 됩니다.

    python3 probe_drive_board.py --list
    python3 probe_drive_board.py --port /dev/ttyUSB0
    python3 probe_drive_board.py --port /dev/ttyUSB0 --spin 3.0

--spin 은 바퀴를 실제로 돌립니다. 로봇을 들어올리거나 벽에서 떨어뜨린 뒤 쓰세요.

의존성은 pyserial 하나입니다:  pip3 install pyserial
"""

import argparse
import glob
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial 이 없습니다:  pip3 install pyserial")

STX = 0x02
ETX = 0x03


def lrc(payload: bytes) -> int:
    """STX 와 ETX 사이 전체의 단순 XOR. 체크섬도 CRC 도 아닙니다."""
    acc = 0
    for b in payload:
        acc ^= b
    return acc


def frame(body: bytes) -> bytes:
    """body = 명령문자 + 페이로드. STX/ETX/LRC 를 붙여 완성합니다."""
    inner = body + bytes([ETX])
    return bytes([STX]) + inner + bytes([lrc(inner)])


def encode_speed(mm_s: int) -> bytes:
    """부호는 상위 바이트 bit7. 2의 보수가 아닙니다 (-500 -> 0x81F4)."""
    if mm_s < 0:
        mag = -mm_s
        return bytes([(mag >> 8) | 0x80, mag & 0xFF])
    return bytes([(mm_s >> 8) & 0xFF, mm_s & 0xFF])


def read_reply(ser, timeout=0.3, idle_gap=0.02):
    """STX 로 재동기화한 뒤, 더 이상 바이트가 안 들어오는 짧은 공백을
    프레임 끝으로 봅니다.

    예전에는 값이 ETX(0x03) 와 같은 바이트를 만나면 그 자리서 끊었는데,
    X/Y/각도 같은 페이로드 안에도 0x03 이 흔히 등장합니다
    (예: theta_raw=0x0379 의 상위 바이트). 그러면 실제 프레임보다 훨씬
    일찍 잘려서 decode_bv() 가 매번 실패했습니다. 이 보드는 응답을 한
    번에 버스트로 보내므로, 바이트 사이 공백(idle_gap)으로 끝을 판정하
    는 쪽이 값에 의존하지 않아 안전합니다.
    """
    deadline = time.time() + timeout
    buf = bytearray()
    last_byte_time = None
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            if buf and last_byte_time is not None \
                    and time.time() - last_byte_time > idle_gap:
                break
            continue
        if not buf and b[0] != STX:
            continue          # 이전 주기의 잔여 바이트 버리기
        buf += b
        last_byte_time = time.time()
    return bytes(buf)


def decode_bv(rx: bytes):
    """BV 응답 해석. 실패하면 None."""
    if len(rx) < 14 or rx[1] not in (0x30, 0x32):
        return None
    xb = ((rx[2] << 24) & 0x7F000000) | ((rx[3] << 16) & 0xFF0000) \
        | ((rx[4] << 8) & 0xFF00) | (rx[5] & 0xFF)
    yb = ((rx[6] << 24) & 0x7F000000) | ((rx[7] << 16) & 0xFF0000) \
        | ((rx[8] << 8) & 0xFF00) | (rx[9] & 0xFF)
    return {
        "x_mm": -xb if rx[2] >> 7 else xb,
        "y_mm": -yb if rx[6] >> 7 else yb,
        "theta_raw": ((rx[10] << 8) & 0xFF00) | (rx[11] & 0xFF),
        "bumper": rx[12],
        "emergency": rx[1] != 0x30,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--list", action="store_true", help="시리얼 포트 후보만 출력")
    ap.add_argument("--spin", type=float, default=0.0,
                    help="이 시간(초)만큼 제자리 회전. 로봇을 들어올린 뒤 쓸 것")
    ap.add_argument("--speed", type=int, default=100, help="바퀴 속도 mm/s")
    args = ap.parse_args()

    if args.list:
        found = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
                       + glob.glob("/dev/tetra*"))
        print("\n".join(found) if found else "시리얼 장치가 하나도 없습니다")
        return 0

    print(f"[1/4] {args.port} 115200 8N1 열기")
    ser = serial.Serial(args.port, 115200, timeout=0.1)
    time.sleep(0.2)
    ser.reset_input_buffer()

    print("[2/4] 속도모드 진입 (CZ11) — 이걸 안 하면 BV 가 조용히 무시됩니다")
    ser.write(frame(b"CZ11"))
    print(f"      응답: {read_reply(ser).hex(' ') or '(없음)'}")

    print("[3/4] 서보 ON (DB11)")
    ser.write(frame(b"DB11"))
    print(f"      응답: {read_reply(ser).hex(' ') or '(없음)'}")

    print("[4/4] BV 왕복 — 정지 명령으로 상태 읽기")
    ser.write(frame(b"BV" + encode_speed(0) + encode_speed(0)))
    rx = read_reply(ser)
    if not rx:
        print("      응답 없음. 포트/결선/전원 확인. 다른 포트일 수도 있습니다")
        return 1
    print(f"      raw: {rx.hex(' ')}")
    st = decode_bv(rx)
    if st is None:
        print("      프레임 해석 실패 — 이 포트가 구동보드가 아닐 수 있습니다")
        print("      (CN24 전원/센서 보드는 다른 응답을 냅니다)")
        return 1
    print(f"      해석: {st}")
    if st["emergency"]:
        print("      ⚠ 비상정지가 걸려 있습니다. 해제하지 않으면 안 움직입니다")

    if args.spin > 0.0:
        print(f"\n제자리 회전 {args.spin:.1f}s @ ±{args.speed} mm/s")
        print("바퀴가 떠 있는지 확인하세요. Ctrl-C 로 즉시 정지합니다.")
        t0 = time.time()
        try:
            while time.time() - t0 < args.spin:
                ser.write(frame(b"BV" + encode_speed(-args.speed)
                                + encode_speed(args.speed)))
                s = decode_bv(read_reply(ser))
                if s:
                    print(f"  x={s['x_mm']:6d}mm y={s['y_mm']:6d}mm "
                          f"th={s['theta_raw']:5d} bump={s['bumper']} "
                          f"emg={s['emergency']}")
                time.sleep(1.0 / 30.0)
        finally:
            for _ in range(3):
                ser.write(frame(b"BV" + encode_speed(0) + encode_speed(0)))
                read_reply(ser)
            print("정지 명령 전송 완료")

    ser.close()
    print("\n통신 정상. 이제 ROS 노드를 띄워도 됩니다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
