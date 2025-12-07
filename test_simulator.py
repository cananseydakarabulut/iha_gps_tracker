#!/usr/bin/env python3
"""
Basitleştirilmiş Test Simülatörü
Hem kendi İHA'yı hem rakip İHA'yı simüle eder
"""
import socket
import json
import time
import math


def main():
    print("=" * 60)
    print("GPS Test Simülatörü")
    print("=" * 60)

    # Saha merkezi (Ankara)
    center_lat = 39.9208
    center_lon = 32.8542

    # Kendi İHA başlangıç
    my_lat = center_lat
    my_lon = center_lon
    my_alt = 100.0
    my_heading = 0.0
    my_speed = 0.0

    # Rakip İHA (200m ileride, dairesel hareket)
    rival_team_id = 7

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("\nSimülasyon başladı...")
    print("- Kendi İHA merkez pozisyonda (sabit)")
    print("- Rakip İHA dairesel hareket yapıyor (200m yarıçap)")
    print("- Veri gönderiliyor -> 127.0.0.1:5799")
    print("Ctrl+C ile durdurun\n")

    t_start = time.time()
    angle = 0.0

    try:
        while True:
            elapsed = time.time() - t_start
            current_time = time.time()

            # Rakip dairesel hareket (200m yarıçap, 10°/s)
            angle = (elapsed * 10) % 360
            radius_m = 200.0

            # Rakip pozisyon hesabı
            lat_offset = (radius_m * math.sin(math.radians(angle))) / 111000.0
            lon_offset = (radius_m * math.cos(math.radians(angle))) / (
                111000.0 * math.cos(math.radians(center_lat))
            )

            rival_lat = center_lat + lat_offset
            rival_lon = center_lon + lon_offset
            rival_alt = 95.0
            rival_speed = 20.0
            rival_heading = (angle + 90) % 360

            # Kendi telemetri paketi + rakip verisi
            packet = {
                "time_s": current_time,
                "gps": {
                    "is_valid": True,
                    "lat": my_lat,
                    "lon": my_lon,
                    "alt": my_alt,
                    "speed_ms": my_speed,
                    "heading": my_heading,
                    "hdop": 1.0
                },
                "imu": {
                    "acc": [0.0, 0.0, -9.81],  # Body frame (NED: down = positive)
                    "gyr": [0.0, 0.0, 0.0]
                },
                "network_data": [{
                    "takim_numarasi": rival_team_id,
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
            sock.sendto(json.dumps(packet).encode(), ("127.0.0.1", 5799))

            # Konsol çıktısı
            rival_angle_deg = math.degrees(math.atan2(lat_offset, lon_offset))
            print(
                f"\r[{elapsed:5.1f}s] "
                f"Kendi: ({my_lat:.6f}, {my_lon:.6f}, {my_alt:.0f}m) | "
                f"Rakip: {rival_angle_deg:3.0f}° {radius_m:.0f}m uzakta "
                f"(Hız:{rival_speed:.0f}m/s)",
                end=""
            )

            time.sleep(0.1)  # 10 Hz

    except KeyboardInterrupt:
        print("\n\nSimülasyon durduruldu.")
        sock.close()


if __name__ == "__main__":
    main()
