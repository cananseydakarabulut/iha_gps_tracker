import socket, json, time, math
from dronekit import connect

GPS_NODE_IP = "127.0.0.1"
GPS_NODE_PORT = 5799

# SITL veya Pixhawk bağlantısı
CONN_STR = "udp:127.0.0.1:14550"  # Mission Planner/SITL’e göre güncelle
vehicle = connect(CONN_STR, wait_ready=True)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_packet():
    loc = vehicle.location.global_frame
    alt_rel = vehicle.location.global_relative_frame.alt
    hdop = getattr(vehicle.gps_0, "eph", 1.0) or 1.0
    groundspeed = vehicle.groundspeed or 0.0
    vx = vehicle.velocity[0] or 0.0
    vy = vehicle.velocity[1] or 0.0
    vz = vehicle.velocity[2] or 0.0
    # yaw from attitude (rad → deg)
    yaw_deg = math.degrees(vehicle.attitude.yaw or 0.0) % 360

    pkt = {
        "time_s": time.time(),
        "gps": {
            "lat": loc.lat,
            "lon": loc.lon,
            "alt": loc.alt,          # AMSL
            "is_valid": vehicle.gps_0.fix_type >= 3,
            "hdop": hdop,
            "speed_ms": groundspeed,
            "ground_speed": groundspeed
        },
        "imu": {
            # İvmeölçer yoksa 0 giriyoruz; UKF zaten hız ölçümünü kullanıyor
            "acc": [0, 0, -9.81],
            "gyr": [0, 0, 0],
        },
        # Hedef listesi yoksa boş bırak
        "network_data": []
    }
    sock.sendto(json.dumps(pkt).encode(), (GPS_NODE_IP, GPS_NODE_PORT))

def main():
    print(f"Köprü başladı, GPS node -> {GPS_NODE_IP}:{GPS_NODE_PORT}")
    try:
        while True:
            send_packet()
            time.sleep(0.02)  # ~50 Hz
    except KeyboardInterrupt:
        pass
    finally:
        vehicle.close()
        sock.close()

if __name__ == "__main__":
    main()
