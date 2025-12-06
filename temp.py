import collections
import json
import math
import socket
import time
from dronekit import connect, LocationGlobalRelative, VehicleMode
from pymavlink import mavutil

# Python 3.10+ uyumluluk (DroneKit eski collections API'sini kullanıyor)
if not hasattr(collections, "MutableMapping"):
    import collections.abc
    collections.MutableMapping = collections.abc.MutableMapping  # type: ignore
    collections.MutableSequence = collections.abc.MutableSequence  # type: ignore
    collections.MutableSet = collections.abc.MutableSet  # type: ignore

# Bağlantı ayarları (Mission Planner/SITL TCP varsayılanı 5760)
CONNECTION_STRING = "tcp:127.0.0.1:5770"
BAUD_RATE = 115200  # TCP'de kullanılmaz, seri için gerekli

# gps.py komutlarını dinleyeceğimiz soket
CMD_LISTEN_IP = "127.0.0.1"
CMD_LISTEN_PORT = 5771

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

def clamp(val, lo, hi):
    return max(lo, min(hi, val))

def main():
    vehicle = connect(CONNECTION_STRING, wait_ready=True, baud=BAUD_RATE)
    print(f"Connected: {CONNECTION_STRING}")

    # Auto switch to GUIDED, arm, and take off once
    target_takeoff_alt = 40  # meters
    try:
        if vehicle.mode.name != "GUIDED":
            print("Switching to GUIDED...")
            vehicle.mode = VehicleMode("GUIDED")
            vehicle.flush()

        if not vehicle.armed:
            print("Arming...")
            vehicle.armed = True
            vehicle.flush()

        print(f"Takeoff to {target_takeoff_alt} m...")
        vehicle.simple_takeoff(target_takeoff_alt)

        t_start = time.time()
        while True:
            alt_now = vehicle.location.global_relative_frame.alt or 0.0
            print(f"Alt: {alt_now:.1f} m", end="\r")
            if alt_now >= target_takeoff_alt * 0.95:
                print(f"\nReached takeoff altitude (~{alt_now:.1f} m)")
                break
            if time.time() - t_start > 45:
                print("\nTakeoff wait timed out, continuing anyway.")
                break
            time.sleep(0.5)
    except Exception as e:
        print(f"Auto GUIDED/ARM/TAKEOFF failed: {e}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((CMD_LISTEN_IP, CMD_LISTEN_PORT))
    print(f"Komut dinleniyor: {CMD_LISTEN_IP}:{CMD_LISTEN_PORT} -> {CONNECTION_STRING}")

    while True:
        try:
            data, _ = sock.recvfrom(4096)
            cmd = json.loads(data.decode())

            target_yaw = float(cmd.get("yaw", vehicle.heading)) % 360
            target_speed = float(cmd.get("speed", 0))
            target_alt = float(cmd.get("alt", vehicle.location.global_relative_frame.alt))

            # GUIDED değilse komutu uygulama
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

        except KeyboardInterrupt:
            print("\nAdapter kapatılıyor...")
            break
        except Exception as e:
            print(f"Hata: {e}")
            continue

    vehicle.close()
    sock.close()

if __name__ == "__main__":
    main()

