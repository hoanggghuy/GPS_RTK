import serial

PORT = "/dev/ttyACM0"
BAUD = 115200   # thường RTCM hay dùng 115200 (nếu không chắc thì test lại)

def parse_rtcm_frame(data):
    """
    Parse RTCM frame để lấy message type
    """
    if len(data) < 6:
        return None

    # byte 1-2 chứa length
    length = ((data[1] & 0x03) << 8) | data[2]

    if len(data) < length + 6:
        return None

    # message type nằm trong 12 bit đầu của payload
    msg_type = (data[3] << 4) | (data[4] >> 4)

    return msg_type, length


ser = serial.Serial(PORT, BAUD, timeout=1)

print("🚀 Reading RTCM...")

buffer = bytearray()

while True:
    try:
        chunk = ser.read(1024)
        buffer.extend(chunk)

        while True:
            # tìm header RTCM (0xD3)
            if len(buffer) < 3:
                break

            if buffer[0] != 0xD3:
                buffer.pop(0)
                continue

            # lấy length
            length = ((buffer[1] & 0x03) << 8) | buffer[2]
            total_len = length + 6  # header + payload + CRC

            if len(buffer) < total_len:
                break

            frame = buffer[:total_len]
            buffer = buffer[total_len:]

            result = parse_rtcm_frame(frame)
            if result:
                msg_type, length = result
                print(f"📡 RTCM Type: {msg_type}, Length: {length}")

    except KeyboardInterrupt:
        break
    except Exception as e:
        print("Error:", e)

ser.close()