import collections
import socket, json, time, math

# Python 3.10+ uyumluluk: dronekit eski collections API'sini bekliyor
if not hasattr(collections, "MutableMapping"):
    import collections.abc
    collections.MutableMapping = collections.abc.MutableMapping  # type: ignore
    collections.MutableSequence = collections.abc.MutableSequence  # type: ignore
    collections.MutableSet = collections.abc.MutableSet  # type: ignore

from dronekit import connect, LocationGlobalRelative
from pymavlink import mavutil

# Bağlantı ayarları
# MP/SITL TCP bağlantısı (MP varsayılanı 5760)
MAVLINK_CONN = "tcp:127.0.0.1:5760"   # SITL/Mission Planner bağlantın
GPS_NODE_IP = "127.0.0.1"
GPS_NODE_PORT = 5799   # gps.py telemetri girişi
CMD_LISTEN_IP = "127.0.0.1"
CMD_LISTEN_PORT = 5771 # gps.py komut çıkışı (aynı portu dinliyoruz)

# Güvenlik sınırları (platformuna göre ayarla)
SAFETY_MIN_SPEED = 15.0
SAFETY_MAX_SPEED = 60.0
SAFETY_MIN_ALT   = 20.0
SAFETY_MAX_ALT   = 150.0

def clamp(val, lo, hi): return max(lo, min(hi, val))

def send_ned_velocity(vehicle, vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)

def condition_yaw(vehicle, heading):
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        heading, 0, 1, 0,
        0, 0, 0)
    vehicle.send_mavlink(msg)

def main():
    vehicle = connect(MAVLINK_CONN, wait_ready=True)
    sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_cmd.bind((CMD_LISTEN_IP, CMD_LISTEN_PORT))
    sock_tele = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"Bridge: MAVLink[{MAVLINK_CONN}] <-> gps.py [{GPS_NODE_IP}:{GPS_NODE_PORT}]")
    while True:
        # 1) Telemetriyi gps.py’ye yayınla
        loc = vehicle.location.global_frame
        hdop = getattr(vehicle.gps_0, "eph", 1.0) or 1.0
        groundspeed = vehicle.groundspeed or 0.0
        pkt = {
            "time_s": time.time(),
            "gps": {
                "lat": loc.lat,
                "lon": loc.lon,
                "alt": loc.alt,
                "is_valid": vehicle.gps_0.fix_type >= 3,
                "hdop": hdop,
                "speed_ms": groundspeed,
                "ground_speed": groundspeed
            },
            "imu": {"acc": [0,0,-9.81], "gyr": [0,0,0]},
            "network_data": []  # hedef listesi yoksa boş
        }
        sock_tele.sendto(json.dumps(pkt).encode(), (GPS_NODE_IP, GPS_NODE_PORT))

        # 2) gps.py komutlarını al ve ArduPilot’a uygula
        sock_cmd.settimeout(0.0)
        try:
            data, _ = sock_cmd.recvfrom(4096)
            cmd = json.loads(data.decode())

            target_yaw = float(cmd.get("yaw", vehicle.heading)) % 360
            target_speed = clamp(float(cmd.get("speed", 0)), SAFETY_MIN_SPEED, SAFETY_MAX_SPEED)
            target_alt = clamp(float(cmd.get("alt", vehicle.location.global_relative_frame.alt)),
                               SAFETY_MIN_ALT, SAFETY_MAX_ALT)

            if vehicle.mode.name != "GUIDED":
                continue

            # İrtifa
            cur = vehicle.location.global_relative_frame
            vehicle.simple_goto(LocationGlobalRelative(cur.lat, cur.lon, target_alt))
            # Yaw
            condition_yaw(vehicle, target_yaw)
            # Hız vektörü (NED)
            yaw_rad = math.radians(target_yaw)
            vx = target_speed * math.cos(yaw_rad)
            vy = target_speed * math.sin(yaw_rad)
            send_ned_velocity(vehicle, vx, vy, 0)
        except socket.timeout:
            pass
        except Exception as e:
            print("Command error:", e)

        time.sleep(0.02)  # ~50 Hz loop

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
