#!/usr/bin/env python3
"""
Rakip İHA Simülatörü
Kendi İHA'nıza sahte rakip verisi gönderir
"""
import socket
import json
import time
import math


def main():
    print("=" * 60)
    print("Rakip İHA Simülatörü")
    print("=" * 60)

    # Kendi İHA'nızın portunu belirleyin (varsayılan IHA-1: 5799)
    my_iha_port = int(input("Kendi IHA portunu girin (varsayılan 5799): ") or "5799")

    # Rakip takım numarası (kendi takımınızdan farklı olmalı)
    rival_team = int(input("Rakip takım ID (varsayılan 7): ") or "7")

    # Referans merkezi (takipçi SITL'in home'u ile aynı olsun)
    center_lat = float(input("Saha merkez lat (varsayılan 39.9208): ") or "39.9208")
    center_lon = float(input("Saha merkez lon (varsayılan 32.8542): ") or "32.8542")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Başlangıç konumu (merkeze yakın)

    print(f"\nRakip Takım {rival_team} simüle ediliyor...")
    print(f"Veri gönderiliyor -> 127.0.0.1:{my_iha_port}")
    print("Ctrl+C ile durdurun\n")

    t_start = time.time()
    angle = 0.0

    try:
        while True:
            elapsed = time.time() - t_start

            # Dairesel hareket (200m yarıçap, 10°/s)
            angle = (elapsed * 10) % 360
            radius_m = 200.0

            # GPS koordinat hesabı (basitleştirilmiş)
            lat_offset = (radius_m * math.sin(math.radians(angle))) / 111000.0
            lon_offset = (radius_m * math.cos(math.radians(angle))) / (111000.0 * math.cos(math.radians(center_lat)))

            rival_lat = center_lat + lat_offset
            rival_lon = center_lon + lon_offset
            rival_alt = 90.0
            rival_speed = 20.0
            rival_heading = (angle + 90) % 360  # Teğet yönünde

            # Teknofest formatında paket oluştur
            packet = {
                "time_s": time.time(),
                "gps": {
                    "is_valid": True,
                    "lat": rival_lat,
                    "lon": rival_lon,
                    "alt": rival_alt,
                    "speed_ms": rival_speed,
                    "heading": rival_heading,
                    "hdop": 1.0
                },
                "imu": {
                    "acc": [0.0, 0.0, 9.81],
                    "gyr": [0.0, 0.0, 0.0]
                },
                "network_data": [{
                    "takim_numarasi": rival_team,
                    "iha_enlem": rival_lat,
                    "iha_boylam": rival_lon,
                    "iha_irtifa": rival_alt,
                    "iha_hiz": rival_speed,
                    "iha_yonelme": rival_heading,
                    "iha_dikilme": 0.0,
                    "iha_yatis": 0.0,
                    "gecikme_ms": 50.0
                }]
            }

            # Gönder
            sock.sendto(json.dumps(packet).encode(), ("127.0.0.1", my_iha_port))

            print(f"[{elapsed:.1f}s] Rakip: Lat={rival_lat:.6f}, Lon={rival_lon:.6f}, "
                  f"Alt={rival_alt:.0f}m, Spd={rival_speed:.0f}m/s, Hdg={rival_heading:.0f}°",
                  end="\r")

            time.sleep(0.1)  # 10 Hz

    except KeyboardInterrupt:
        print("\n\nRakip simülasyonu durduruldu.")
        sock.close()


if __name__ == "__main__":
    main()
