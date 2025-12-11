#!/usr/bin/env python3
"""
Rakip İHA Otopilot - ArduPlane için AUTO Mode Circular Mission
"""

import time
import math
from pymavlink import mavutil
import argparse


def create_circular_mission(master, center_lat, center_lon, radius_m, altitude_m, num_waypoints=16):
    """
    Dairesel mission oluştur ve ArduPlane'e yükle
    """
    print(f"\n📋 Dairesel mission oluşturuluyor:")
    print(f"   Merkez: ({center_lat:.6f}, {center_lon:.6f})")
    print(f"   Yarıçap: {radius_m}m, İrtifa: {altitude_m}m")
    print(f"   Waypoint sayısı: {num_waypoints}")

    waypoints = []

    # WP 0: HOME (gerekli)
    wp = mavutil.mavlink.MAVLink_mission_item_int_message(
        master.target_system,
        master.target_component,
        0,  # seq
        mavutil.mavlink.MAV_FRAME_GLOBAL,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 0,  # current, autocontinue
        0, 0, 0, 0,  # param1-4
        int(center_lat * 1e7),
        int(center_lon * 1e7),
        altitude_m
    )
    waypoints.append(wp)

    # WP 1: Takeoff
    wp = mavutil.mavlink.MAVLink_mission_item_int_message(
        master.target_system,
        master.target_component,
        1,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 1,  # current=0, autocontinue=1
        0, 0, 0, 0,  # param1-4 (pitch)
        int(center_lat * 1e7),
        int(center_lon * 1e7),
        altitude_m
    )
    waypoints.append(wp)

    # Dairesel waypoint'ler
    for i in range(num_waypoints):
        angle = (2 * math.pi * i) / num_waypoints

        # Offset hesapla (yaklaşık)
        lat_offset = (radius_m * math.cos(angle)) / 111111.0
        lon_offset = (radius_m * math.sin(angle)) / (111111.0 * math.cos(math.radians(center_lat)))

        wp_lat = center_lat + lat_offset
        wp_lon = center_lon + lon_offset

        wp = mavutil.mavlink.MAVLink_mission_item_int_message(
            master.target_system,
            master.target_component,
            i + 2,  # seq (0=home, 1=takeoff, 2+=circle)
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1,  # current, autocontinue
            0, 0, 0, 0,  # param1-4
            int(wp_lat * 1e7),
            int(wp_lon * 1e7),
            altitude_m
        )
        waypoints.append(wp)

    # Son waypoint: İlk dairesel waypoint'e dön (loop)
    wp = mavutil.mavlink.MAVLink_mission_item_int_message(
        master.target_system,
        master.target_component,
        len(waypoints),
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_JUMP,
        0, 1,
        2,  # param1: jump to WP 2 (first circle waypoint)
        -1,  # param2: repeat forever (-1)
        0, 0,
        0, 0, 0
    )
    waypoints.append(wp)

    return waypoints


def upload_mission(master, waypoints):
    """Mission'ı ArduPlane'e yükle"""
    print(f"\n📤 Mission yükleniyor ({len(waypoints)} waypoint)...")

    # Mission count gönder
    master.mav.mission_count_send(
        master.target_system,
        master.target_component,
        len(waypoints),
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION
    )

    # Her waypoint için ACK bekle ve gönder
    for wp in waypoints:
        msg = master.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'], blocking=True, timeout=5)
        if msg is None:
            print(f"❌ Waypoint {wp.seq} için REQUEST timeout!")
            return False

        if msg.seq != wp.seq:
            print(f"⚠️ Sıra uyuşmazlığı: beklenen {msg.seq}, gönderilen {wp.seq}")

        master.mav.send(wp)
        print(f"  ✓ WP {wp.seq}/{len(waypoints)-1} gönderildi")

    # ACK bekle
    msg = master.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
    if msg and msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
        print("✅ Mission başarıyla yüklendi!")
        return True
    else:
        print(f"❌ Mission ACK hatası: {msg.type if msg else 'timeout'}")
        return False


def main():
    parser = argparse.ArgumentParser(description="ArduPlane Rakip Otopilot (AUTO Mode)")
    parser.add_argument("--connect", default="udp:127.0.0.1:14561", help="MAVLink bağlantı")
    parser.add_argument("--radius", type=float, default=200.0, help="Daire yarıçapı (m)")
    parser.add_argument("--altitude", type=float, default=60.0, help="Uçuş irtifası (m)")
    parser.add_argument("--waypoints", type=int, default=16, help="Dairedeki waypoint sayısı")
    args = parser.parse_args()

    print(f"🛩️  ArduPlane Rakip Otopilot Başlatılıyor...")
    print(f"Bağlantı: {args.connect}")

    # Bağlan
    master = mavutil.mavlink_connection(args.connect)
    print("Heartbeat bekleniyor...")
    master.wait_heartbeat()
    print(f"✓ Bağlandı (sysid={master.target_system}, compid={master.target_component})")

    # Merkez konum al
    print("\n📍 GPS konumu bekleniyor...")
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=10)
    if msg:
        center_lat = msg.lat / 1e7
        center_lon = msg.lon / 1e7
        print(f"✓ Merkez: {center_lat:.6f}, {center_lon:.6f}")
    else:
        print("⚠️ GPS alınamadı, varsayılan konum kullanılıyor")
        center_lat = -35.3632607
        center_lon = 149.1652351

    # Dairesel mission oluştur
    waypoints = create_circular_mission(
        master,
        center_lat,
        center_lon,
        args.radius,
        args.altitude,
        args.waypoints
    )

    # Mission'ı yükle
    if not upload_mission(master, waypoints):
        print("❌ Mission yüklenemedi! Çıkılıyor...")
        return

    time.sleep(2)

    # Arm
    print("\n🔧 Arm ediliyor...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,  # arm
        0, 0, 0, 0, 0, 0
    )

    # Arm ACK bekle
    msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if msg and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("✓ Armed")
    else:
        print("⚠️ Arm ACK alınamadı ama devam ediliyor...")

    time.sleep(1)

    # AUTO moda geç
    print("\n✈️  AUTO moduna geçiliyor...")
    master.set_mode('AUTO')
    time.sleep(2)
    print("✓ AUTO mode aktif - Mission başladı!")

    print(f"\n🔄 Rakip İHA dairesel uçuşta (yarıçap={args.radius}m, irtifa={args.altitude}m)")
    print("Durdurmak için Ctrl+C")

    # Mission progress izle
    last_wp = -1
    try:
        while True:
            msg = master.recv_match(type='MISSION_CURRENT', blocking=False, timeout=0.1)
            if msg and msg.seq != last_wp:
                last_wp = msg.seq
                print(f"📍 Waypoint: {msg.seq}/{len(waypoints)-1}")
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\n⏹  Otopilot durduruldu")
        print("RTL moduna geçiliyor...")
        master.set_mode('RTL')
        time.sleep(1)
        print("✓ RTL aktif")


if __name__ == "__main__":
    main()
