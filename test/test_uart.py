import serial

PORT = "/dev/ttyUSB2"
BAUD = 57600

def convert_to_decimal(raw, direction):
    """
    Convert ddmm.mmmm -> decimal degrees
    """
    if not raw:
        return None

    raw = float(raw)
    degrees = int(raw / 100)
    minutes = raw - degrees * 100
    decimal = degrees + minutes / 60

    if direction in ['S', 'W']:
        decimal *= -1

    return decimal


def parse_gga(line):
    """
    Parse GNGGA sentence
    """
    try:
        parts = line.split(',')

        lat = convert_to_decimal(parts[2], parts[3])
        lon = convert_to_decimal(parts[4], parts[5])
        fix = parts[6]  # 0 = no fix, 1 = GPS fix, 4 = RTK fixed 👀

        return lat, lon, fix
    except:
        return None, None, None


ser = serial.Serial(PORT, BAUD, timeout=1)

print("🚀 Reading GPS...")

while True:
    try:
        line = ser.readline().decode('ascii', errors='ignore').strip()

        if line.startswith("$GNGGA") or line.startswith("$GPGGA"):
            lat, lon, fix = parse_gga(line)

            if lat and lon:
                print(f"📍 Lat: {lat:.6f}, Lon: {lon:.6f} | Fix: {fix}")

    except KeyboardInterrupt:
        break
    except Exception as e:
        print("Error:", e)

ser.close()