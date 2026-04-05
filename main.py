#!/usr/bin/env python3
import time
import threading
from dataclasses import dataclass
from typing import Dict, Optional

import serial
import pynmea2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


# =========================================================
# CONFIG
# =========================================================

# 1 cổng nhận RTCM từ RF
COM_RTCM_IN = "/dev/ttyUSB2"

# 2 rover
COM_FRONT_ROVER = "/dev/ttyACM1"
COM_REAR_ROVER = "/dev/ttyUSB0"

BAUD_RTCM_IN = 57600
BAUD_ROVER = 57600

TOPIC_FRONT = "/gps/front/fix"
TOPIC_REAR = "/gps/rear/fix"

PRINT_ALL_NMEA = False

# Chỉ publish khi front + rear có cùng UTC GGA
SYNC_TIMEOUT_SEC = 0.50      # nếu 1 bên bị chậm quá thì bỏ mẫu cũ
SERIAL_RETRY_SEC = 2.0
READ_CHUNK_SIZE = 4096

# Nếu muốn chỉ publish khi cả 2 đều RTK FIX thì bật True
PUBLISH_ONLY_WHEN_BOTH_RTK_FIX = False


# =========================================================
# DATA TYPES
# =========================================================
@dataclass
class GgaFrame:
    source: str
    utc_key: str
    rx_pc_time: float
    lat: float
    lon: float
    alt: float
    sats: int
    hdop: float
    fix_quality: int
    fix_text: str
    raw_nmea: str


# =========================================================
# HELPERS
# =========================================================
def fix_quality_to_text(fix: int) -> str:
    fix_map = {
        0: "NO FIX",
        1: "GPS",
        2: "DGPS",
        3: "PPS",
        4: "RTK FIX",
        5: "RTK FLOAT",
        6: "ESTIMATED",
        7: "MANUAL",
        8: "SIMULATION",
    }
    return fix_map.get(fix, f"UNKNOWN({fix})")


def utc_key_from_timestamp(ts) -> str:
    """
    Lấy key đồng bộ từ timestamp NMEA.
    Ưu tiên có phần microsecond để phân biệt khi GPS output >1Hz.
    Ví dụ:
      12:34:56.00 -> "123456000000"
      12:34:56.20 -> "123456200000"
    """
    if ts is None:
        return ""
    try:
        return ts.strftime("%H%M%S%f")
    except Exception:
        return str(ts)


def navsat_status_from_fix_quality(fix_quality: int) -> int:
    if fix_quality in (4, 5):   # RTK FIX / RTK FLOAT
        return NavSatStatus.STATUS_GBAS_FIX
    if fix_quality in (1, 2, 3):
        return NavSatStatus.STATUS_FIX
    return NavSatStatus.STATUS_NO_FIX


def estimate_covariance_from_hdop(hdop: float):
    """
    Estimate rất thô từ HDOP.
    Nếu sau này bạn có eph/epv thật thì thay chỗ này.
    """
    hdop = max(hdop, 0.5)
    horiz_var = hdop * hdop
    vert_var = (2.0 * hdop) * (2.0 * hdop)
    return [
        horiz_var, 0.0, 0.0,
        0.0, horiz_var, 0.0,
        0.0, 0.0, vert_var
    ]


# =========================================================
# SERIAL WRAPPER FOR EACH ROVER
# =========================================================
class RoverSerial:
    def __init__(self, name: str, port: str, baud: int, node_logger):
        self.name = name
        self.port = port
        self.baud = baud
        self.logger = node_logger

        self.ser: Optional[serial.Serial] = None
        self.lock = threading.Lock()

        self.rx_nmea_lines = 0
        self.tx_rtcm_bytes = 0

    def open(self):
        with self.lock:
            if self.ser is None:
                self.ser = serial.Serial(
                    self.port,
                    self.baud,
                    timeout=0.1,
                    write_timeout=0.5
                )
                self.logger.info(f"[{self.name}] Opened rover port {self.port} @ {self.baud}")

    def close(self):
        with self.lock:
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None

    def ensure_open(self):
        if self.ser is None:
            self.open()

    def write_rtcm(self, data: bytes) -> int:
        with self.lock:
            if self.ser is None:
                raise serial.SerialException(f"{self.name}: serial not open")
            written = self.ser.write(data)
            self.tx_rtcm_bytes += int(written)
            return int(written)

    def readline(self) -> Optional[str]:
        with self.lock:
            if self.ser is None:
                raise serial.SerialException(f"{self.name}: serial not open")

            if self.ser.in_waiting <= 0:
                return None

            raw = self.ser.readline()
            if not raw:
                return None

            line = raw.decode("ascii", errors="ignore").strip()
            if line:
                self.rx_nmea_lines += 1
            return line


# =========================================================
# ROS2 NODE
# =========================================================
class DualGpsHeadingNode(Node):
    def __init__(self):
        super().__init__("dual_gps_heading_node")

        self.front_pub = self.create_publisher(NavSatFix, TOPIC_FRONT, 10)
        self.rear_pub = self.create_publisher(NavSatFix, TOPIC_REAR, 10)

        self.stop_event = threading.Event()

        # serial for RTCM input
        self.rtcm_ser: Optional[serial.Serial] = None
        self.rtcm_lock = threading.Lock()
        self.rtcm_in_bytes = 0

        # 2 rover serial
        self.front_rover = RoverSerial("front", COM_FRONT_ROVER, BAUD_ROVER, self.get_logger())
        self.rear_rover = RoverSerial("rear", COM_REAR_ROVER, BAUD_ROVER, self.get_logger())

        # buffer đồng bộ theo UTC
        self.front_buffer: Dict[str, GgaFrame] = {}
        self.rear_buffer: Dict[str, GgaFrame] = {}
        self.buffer_lock = threading.Lock()

        self.last_stats_time = time.time()

        # threads
        self.threads = [
            threading.Thread(target=self.rtcm_broadcast_thread, daemon=True),
            threading.Thread(target=self.rover_read_thread, args=(self.front_rover, "front"), daemon=True),
            threading.Thread(target=self.rover_read_thread, args=(self.rear_rover, "rear"), daemon=True),
            threading.Thread(target=self.sync_publish_thread, daemon=True),
        ]

        for t in self.threads:
            t.start()

        self.get_logger().info("Dual GPS heading node started")
        self.get_logger().info(f"RTCM input: {COM_RTCM_IN}")
        self.get_logger().info(f"Front rover: {COM_FRONT_ROVER} -> {TOPIC_FRONT}")
        self.get_logger().info(f"Rear rover : {COM_REAR_ROVER} -> {TOPIC_REAR}")

    # -----------------------------------------------------
    # RTCM INPUT
    # -----------------------------------------------------
    def open_rtcm_input(self):
        with self.rtcm_lock:
            if self.rtcm_ser is None:
                self.rtcm_ser = serial.Serial(COM_RTCM_IN, BAUD_RTCM_IN, timeout=0.1)
                self.get_logger().info(f"[RTCM] Opened input {COM_RTCM_IN} @ {BAUD_RTCM_IN}")

    def close_rtcm_input(self):
        with self.rtcm_lock:
            if self.rtcm_ser is not None:
                try:
                    self.rtcm_ser.close()
                except Exception:
                    pass
                self.rtcm_ser = None

    def rtcm_broadcast_thread(self):
        while rclpy.ok() and not self.stop_event.is_set():
            try:
                if self.rtcm_ser is None:
                    self.open_rtcm_input()

                self.front_rover.ensure_open()
                self.rear_rover.ensure_open()

                with self.rtcm_lock:
                    if self.rtcm_ser is None:
                        raise serial.SerialException("RTCM input not open")
                    data = self.rtcm_ser.read(READ_CHUNK_SIZE)

                if data:
                    self.rtcm_in_bytes += len(data)

                    # broadcast cùng một block sang cả 2 rover
                    try:
                        self.front_rover.write_rtcm(data)
                    except serial.SerialTimeoutException:
                        self.get_logger().warn("[front] Write timeout while sending RTCM")
                    except serial.SerialException as e:
                        self.get_logger().error(f"[front] RTCM write serial error: {e}")
                        self.front_rover.close()

                    try:
                        self.rear_rover.write_rtcm(data)
                    except serial.SerialTimeoutException:
                        self.get_logger().warn("[rear] Write timeout while sending RTCM")
                    except serial.SerialException as e:
                        self.get_logger().error(f"[rear] RTCM write serial error: {e}")
                        self.rear_rover.close()

                self.print_stats_if_needed()
                time.sleep(0.002)

            except serial.SerialException as e:
                self.get_logger().error(f"[RTCM INPUT ERROR] {e}")
                self.close_rtcm_input()
                time.sleep(SERIAL_RETRY_SEC)

            except Exception as e:
                self.get_logger().error(f"[RTCM INPUT UNEXPECTED ERROR] {e}")
                time.sleep(1.0)

        self.close_rtcm_input()

    # -----------------------------------------------------
    # READ NMEA FROM EACH ROVER
    # -----------------------------------------------------
    def rover_read_thread(self, rover: RoverSerial, source: str):
        while rclpy.ok() and not self.stop_event.is_set():
            try:
                rover.ensure_open()
                line = rover.readline()

                if line is None:
                    time.sleep(0.005)
                    continue

                if PRINT_ALL_NMEA and line.startswith("$"):
                    self.get_logger().info(f"[{source}][NMEA] {line}")

                if line.startswith("$GNGGA") or line.startswith("$GPGGA"):
                    gga = self.parse_gga(source, line)
                    if gga is not None:
                        self.store_gga(gga)

                # nếu muốn debug thêm RMC thì mở đoạn dưới
                # elif line.startswith("$GNRMC") or line.startswith("$GPRMC"):
                #     self.handle_rmc(source, line)

            except serial.SerialException as e:
                self.get_logger().error(f"[{source}] Rover serial error: {e}")
                rover.close()
                time.sleep(SERIAL_RETRY_SEC)

            except Exception as e:
                self.get_logger().error(f"[{source}] Rover unexpected error: {e}")
                time.sleep(0.5)

    # -----------------------------------------------------
    # PARSING
    # -----------------------------------------------------
    def parse_gga(self, source: str, line: str) -> Optional[GgaFrame]:
        try:
            msg = pynmea2.parse(line)

            fix_quality = int(msg.gps_qual) if msg.gps_qual not in [None, ""] else 0
            fix_text = fix_quality_to_text(fix_quality)

            nmea_ts = getattr(msg, "timestamp", None)
            utc_key = utc_key_from_timestamp(nmea_ts)

            lat = float(msg.latitude) if msg.latitude not in [None, ""] else 0.0
            lon = float(msg.longitude) if msg.longitude not in [None, ""] else 0.0
            alt = float(msg.altitude) if msg.altitude not in [None, ""] else 0.0
            sats = int(msg.num_sats) if msg.num_sats not in [None, ""] else 0
            hdop = float(msg.horizontal_dil) if msg.horizontal_dil not in [None, ""] else 99.9

            self.get_logger().info(
                f"[{source}][GGA] utc={utc_key} lat={lat:.8f} lon={lon:.8f} "
                f"alt={alt:.3f} sats={sats} hdop={hdop:.2f} "
                f"fix={fix_quality} ({fix_text})"
            )

            return GgaFrame(
                source=source,
                utc_key=utc_key,
                rx_pc_time=time.time(),
                lat=lat,
                lon=lon,
                alt=alt,
                sats=sats,
                hdop=hdop,
                fix_quality=fix_quality,
                fix_text=fix_text,
                raw_nmea=line,
            )

        except pynmea2.ParseError as e:
            self.get_logger().warn(f"[{source}] GGA parse error: {e}")
            return None
        except Exception as e:
            self.get_logger().warn(f"[{source}] GGA unexpected parse error: {e}")
            return None

    def handle_rmc(self, source: str, line: str):
        try:
            msg = pynmea2.parse(line)
            nmea_time = str(msg.timestamp) if getattr(msg, "timestamp", None) else ""
            lat = float(msg.latitude) if msg.latitude not in [None, ""] else 0.0
            lon = float(msg.longitude) if msg.longitude not in [None, ""] else 0.0
            status_char = getattr(msg, "status", "")
            rmc_status = "VALID" if status_char == "A" else "INVALID"

            self.get_logger().info(
                f"[{source}][RMC] utc={nmea_time} lat={lat:.8f} lon={lon:.8f} status={rmc_status}"
            )
        except Exception:
            pass

    # -----------------------------------------------------
    # BUFFER
    # -----------------------------------------------------
    def store_gga(self, gga: GgaFrame):
        if not gga.utc_key:
            return

        with self.buffer_lock:
            if gga.source == "front":
                self.front_buffer[gga.utc_key] = gga
            else:
                self.rear_buffer[gga.utc_key] = gga

            self.cleanup_old_buffers_locked()

    def cleanup_old_buffers_locked(self):
        now = time.time()

        old_front_keys = [
            k for k, v in self.front_buffer.items()
            if (now - v.rx_pc_time) > SYNC_TIMEOUT_SEC
        ]
        for k in old_front_keys:
            del self.front_buffer[k]

        old_rear_keys = [
            k for k, v in self.rear_buffer.items()
            if (now - v.rx_pc_time) > SYNC_TIMEOUT_SEC
        ]
        for k in old_rear_keys:
            del self.rear_buffer[k]

    # -----------------------------------------------------
    # SYNC PUBLISH
    # -----------------------------------------------------
    def sync_publish_thread(self):
        while rclpy.ok() and not self.stop_event.is_set():
            try:
                front_gga = None
                rear_gga = None
                sync_key = None

                with self.buffer_lock:
                    self.cleanup_old_buffers_locked()

                    common_keys = sorted(set(self.front_buffer.keys()) & set(self.rear_buffer.keys()))
                    if common_keys:
                        sync_key = common_keys[0]
                        front_gga = self.front_buffer.pop(sync_key)
                        rear_gga = self.rear_buffer.pop(sync_key)

                if front_gga is None or rear_gga is None:
                    time.sleep(0.005)
                    continue

                if PUBLISH_ONLY_WHEN_BOTH_RTK_FIX:
                    if not (front_gga.fix_quality == 4 and rear_gga.fix_quality == 4):
                        self.get_logger().warn(
                            f"[SYNC DROP] utc={sync_key} because not both RTK FIX "
                            f"(front={front_gga.fix_quality}, rear={rear_gga.fix_quality})"
                        )
                        continue

                # Quan trọng: dùng cùng 1 ROS stamp cho cả 2 topic
                stamp = self.get_clock().now().to_msg()

                front_msg = self.make_navsatfix_msg(front_gga, stamp)
                rear_msg = self.make_navsatfix_msg(rear_gga, stamp)

                self.front_pub.publish(front_msg)
                self.rear_pub.publish(rear_msg)

                delta_ms = abs(front_gga.rx_pc_time - rear_gga.rx_pc_time) * 1000.0
                self.get_logger().info(
                    f"[SYNC PUBLISH] utc={sync_key} | dt_rx={delta_ms:.1f} ms | "
                    f"front=({front_gga.lat:.8f},{front_gga.lon:.8f}) "
                    f"rear=({rear_gga.lat:.8f},{rear_gga.lon:.8f})"
                )

            except Exception as e:
                self.get_logger().error(f"[SYNC ERROR] {e}")
                time.sleep(0.05)

    # -----------------------------------------------------
    # ROS MSG BUILD
    # -----------------------------------------------------
    def make_navsatfix_msg(self, gga: GgaFrame, stamp):
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = f"{gga.source}_gps"

        msg.status.status = navsat_status_from_fix_quality(gga.fix_quality)
        msg.status.service = NavSatStatus.SERVICE_GPS

        msg.latitude = gga.lat
        msg.longitude = gga.lon
        msg.altitude = gga.alt

        msg.position_covariance = estimate_covariance_from_hdop(gga.hdop)
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        return msg

    # -----------------------------------------------------
    # STATS
    # -----------------------------------------------------
    def print_stats_if_needed(self):
        now = time.time()
        if now - self.last_stats_time < 5.0:
            return

        self.get_logger().info(
            "[STATS] "
            f"RTCM in={self.rtcm_in_bytes} bytes | "
            f"front rtcm out={self.front_rover.tx_rtcm_bytes} bytes, nmea lines={self.front_rover.rx_nmea_lines} | "
            f"rear rtcm out={self.rear_rover.tx_rtcm_bytes} bytes, nmea lines={self.rear_rover.rx_nmea_lines}"
        )
        self.last_stats_time = now

    # -----------------------------------------------------
    # CLEANUP
    # -----------------------------------------------------
    def destroy_node(self):
        self.stop_event.set()
        time.sleep(0.5)

        self.close_rtcm_input()
        self.front_rover.close()
        self.rear_rover.close()

        super().destroy_node()


# =========================================================
# MAIN
# =========================================================
def main(args=None):
    rclpy.init(args=args)
    node = DualGpsHeadingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()