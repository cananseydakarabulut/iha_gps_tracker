#!/usr/bin/env python3
"""
Rakip İHA Otopilot - SITL Testi için
İHA-2'yi otomatik dairesel rotada hareket ettirir
"""

import time
import math
from pymavlink import mavutil
import argparse


def main():
    parser = argparse.ArgumentParser(description="Rakip İHA otopilot kontrolü")
    parser.add_argument("--connect", default="tcp:127.0.0.1:14560", help="MAVLink bağlantı adresi")
    parser.add_argument("--radius", type=float, default=200.0, help="Dairesel hareket yarıçapı (m)")
    parser.add_argument("--altitude", type=float, default=60.0, help="Uçuş irtifası (m)")
    parser.add_argument("--speed", type=float, default=0.05, help="Açısal hız (radyan/saniye)")
    args = parser.parse_args()

    print(f"Rakip İHA'ya bağlanılıyor: {args.connect}")
    master = mavutil.mavlink_connection(args.connect)

    # Heartbeat bekle
    print("Heartbeat bekleniyor...")
    master.wait_heartbeat()
    print(f"✓ Bağlandı (sysid={master.target_system}, compid={master.target_component})")

    # Mevcut konumu al (başlangıç noktası olarak kullanacağız)
    print("İlk GPS konumu bekleniyor...")
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=10)
    if msg:
        center_lat = msg.lat / 1e7
        center_lon = msg.lon / 1e7
        print(f"Merkez konum: {center_lat:.6f}, {center_lon:.6f}")
    else:
        print("⚠ GPS konumu alınamadı, varsayılan konum kullanılıyor")
        center_lat = -35.3632607
        center_lon = 149.1652351

    # Arm et
    print("\nArm ediliyor...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # confirmation
        1,  # param1 (1=arm)
        0, 0, 0, 0, 0, 0
    )

    # Arm onayı bekle
    time.sleep(2)
    print("✓ Arm edildi")

    # Takeoff
    print(f"Takeoff yapılıyor ({args.altitude}m)...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0,
        args.altitude
    )

    # Kalkışı bekle
    print("Kalkış bekleniyor (15 saniye)...")
    time.sleep(15)
    print("✓ Kalkış tamamlandı")

    # Guided moduna geç
    print("\nGuided moduna geçiliyor...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4,  # Guided mode (ArduPlane)
        0, 0, 0, 0, 0
    )
    time.sleep(2)
    print("✓ Guided mode aktif")

    # Dairesel hareket
    print(f"\n🔄 Dairesel hareket başlıyor (yarıçap={args.radius}m, irtifa={args.altitude}m)")
    print("Durdurmak için Ctrl+C")

    angle = 0.0
    waypoint_count = 0

    try:
        while True:
            # Dairesel pozisyon hesapla
            lat_offset = (args.radius * math.cos(angle)) / 111111.0
            lon_offset = (args.radius * math.sin(angle)) / (111111.0 * math.cos(math.radians(center_lat)))

            target_lat = center_lat + lat_offset
            target_lon = center_lon + lon_offset

            # SET_POSITION_TARGET_GLOBAL_INT mesajı gönder (Guided mode için)
            master.mav.set_position_target_global_int_send(
                0,  # time_boot_ms (ignored)
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                0b0000111111111000,  # type_mask (only position)
                int(target_lat * 1e7),
                int(target_lon * 1e7),
                args.altitude,
                0, 0, 0,  # vx, vy, vz (ignored)
                0, 0, 0,  # afx, afy, afz (ignored)
                0, 0      # yaw, yaw_rate (ignored)
            )

            waypoint_count += 1
            if waypoint_count % 10 == 0:
                print(f"Waypoint #{waypoint_count} | Açı: {math.degrees(angle):.1f}° | "
                      f"Konum: ({target_lat:.6f}, {target_lon:.6f})")

            # Açıyı artır
            angle += args.speed
            if angle > 2 * math.pi:
                angle -= 2 * math.pi
                print(f"🔄 Tam tur tamamlandı ({waypoint_count} waypoint)")

            time.sleep(1)  # 1 Hz güncelleme

    except KeyboardInterrupt:
        print("\n\n⏹ Otopiolt durduruldu")
        print(f"Toplam {waypoint_count} waypoint gönderildi")

        # RTL (Return to Launch)
        print("RTL moduna geçiliyor...")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        time.sleep(2)
        print("✓ RTL aktif")


if __name__ == "__main__":
    main()
