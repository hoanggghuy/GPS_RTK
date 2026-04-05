#!/usr/bin/env python3
import serial

PORT = "/dev/ttyUSB2"
BAUD = 57600   # chỉnh đúng theo RF của m

def get_rtcm_payload_length(buf):
    if len(buf) < 3:
        return None
    return ((buf[1] & 0x03) << 8) | buf[2]

def get_rtcm_message_type(frame):
    if len(frame) < 8:
        return None
    payload = frame[3:-3]
    if len(payload) < 2:
        return None
    return ((payload[0] << 4) | (payload[1] >> 4)) & 0x0FFF

def extract_rtcm_frames(buffer):
    frames = []

    while True:
        start = buffer.find(b"\xD3")
        if start < 0:
            buffer.clear()
            break

        if start > 0:
            del buffer[:start]

        if len(buffer) < 3:
            break

        payload_len = get_rtcm_payload_length(buffer)
        if payload_len is None:
            break

        total_len = 3 + payload_len + 3

        if len(buffer) < total_len:
            break

        frame = bytes(buffer[:total_len])
        frames.append(frame)
        del buffer[:total_len]

    return frames


def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    print(f"[OK] Opened {PORT} @ {BAUD}")

    rx_buffer = bytearray()
    total_bytes = 0
    total_frames = 0

    while True:
        data = ser.read(4096)
        if not data:
            continue

        rx_buffer.extend(data)
        total_bytes += len(data)

        frames = extract_rtcm_frames(rx_buffer)

        for frame in frames:
            total_frames += 1
            msg_type = get_rtcm_message_type(frame)
            payload_len = len(frame) - 6

            print(f"[RTCM] type={msg_type} | payload={payload_len} | total={len(frame)}")

        # debug nhẹ
        if total_frames % 50 == 0 and total_frames > 0:
            print(f"[STATS] frames={total_frames} bytes={total_bytes}")


if __name__ == "__main__":
    main()