# -*- coding: utf-8 -*-
import argparse
import math
import random
import time
from pymavlink import mavutil

# Varsayılan lider bağlantısı (Terminal 1'deki araç)
DEFAULT_MASTER = "tcp:127.0.0.1:5760"
ALTITUDE = 20  # Uçuş yüksekliği
RADIUS = 60    # Ne kadar uzağa gidebilir? (Metre)

def main():
    ap = argparse.ArgumentParser(description="Lider İHA'yı rastgele gezdirir")
    ap.add_argument(
        "--master",
        default=DEFAULT_MASTER,
        help="MAVLink bağlantı dizesi (örn. tcp:127.0.0.1:5760 veya udp:127.0.0.1:14550)",
    )
    args = ap.parse_args()

    print(f"Lider uçağa bağlanılıyor: {args.master}...")
    vehicle = mavutil.mavlink_connection(args.master)
    vehicle.wait_heartbeat()
    print("✅ Lider uçak bağlandı! Rastgele görev başlıyor...")

    # Modu GUIDED yap, Arm et ve Kaldır
    vehicle.set_mode_apm("GUIDED")
    vehicle.arducopter_arm()
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        ALTITUDE,
    )
    time.sleep(10)  # Kalkış için bekle

    # İlk konumu al (Merkez kabul et)
    msg = vehicle.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
    start_lat = msg.lat
    start_lon = msg.lon

    print("🎲 Rastgele devriye başladı! (Durdurmak için Ctrl+C)")

    while True:
        # Rastgele yön ve mesafe seç
        angle = random.uniform(0, 2 * math.pi)
        dist = random.uniform(10, RADIUS)

        # Koordinat öteleme (Yaklaşık hesap)
        d_lat = (dist * math.cos(angle)) / 111132.0
        d_lon = (dist * math.sin(angle)) / (111132.0 * math.cos(start_lat / 1e7 * math.pi / 180.0))

        target_lat = start_lat + int(d_lat * 1e7)
        target_lon = start_lon + int(d_lon * 1e7)

        print(f"📍 Lider yeni hedefe gidiyor: Lat={target_lat/1e7}, Lon={target_lon/1e7}")

        # Lidere "Buraya Git" emri ver
        vehicle.mav.set_position_target_global_int_send(
            0,
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b110111111000,
            target_lat,
            target_lon,
            ALTITUDE,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )

        # 12 saniye bekle (Hedefe varması için)
        time.sleep(12)


if __name__ == "__main__":
    main()