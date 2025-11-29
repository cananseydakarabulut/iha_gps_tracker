import socket
import json
import time
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# ==========================
# AYARLAR
# ==========================
# Pixhawk Bağlantı Adresi
# USB ile bağlıysa genelde: '/dev/ttyACM0' veya 'COM3'
# Telemetri portundan bağlıysa: '/dev/ttyTHS1' (Jetson/Pi için)
CONNECTION_STRING = '/dev/ttyACM0' 
BAUD_RATE = 115200

# gps.py'dan gelen komutları dinleyeceğimiz port
LISTEN_IP = "127.0.0.1"
LISTEN_PORT = 5771 

# ==========================
# DRONEKIT YARDIMCI FONKSİYONLAR
# ==========================
def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z, duration):
    """
    İHA'ya hız vektörü gönderir (LOCAL_NED frame).
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # m/s
        0, 0, 0, # x, y, z acceleration
        0, 0)    # yaw, yaw_rate
    vehicle.send_mavlink(msg)

def condition_yaw(vehicle, heading, relative=False):
    """
    İHA'nın burnunu belli bir dereceye döndürür.
    heading: Derece (0-360)
    relative: True ise şu anki açısına ekler, False ise Kuzeye göre döner.
    """
    if relative:
        is_relative = 1 # yaw relative to direction of travel
    else:
        is_relative = 0 # yaw is an absolute angle
    
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, # command
        0,       # confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    vehicle.send_mavlink(msg)

# ==========================
# ANA DÖNGÜ
# ==========================
def main():
    print(f"🔌 Pixhawk'a bağlanılıyor: {CONNECTION_STRING}...")
    try:
        # İHA'ya bağlan
        vehicle = connect(CONNECTION_STRING, wait_ready=True, baud=BAUD_RATE)
        print("✅ İHA BAĞLANDI!")
    except Exception as e:
        print(f"❌ Bağlantı Hatası: {e}")
        return

    # UDP Soketi Aç (gps.py'yi dinle)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    print(f"📡 Komutlar Bekleniyor: {LISTEN_IP}:{LISTEN_PORT}")

    while True:
        try:
            # 1. gps.py'dan JSON verisini al
            data, _ = sock.recvfrom(4096)
            cmd = json.loads(data.decode())
            
            # cmd içeriği: {'type': 'control', 'yaw': 45.0, 'speed': 22.0, 'alt': 50.0}
            
            target_yaw = float(cmd.get("yaw", vehicle.heading))
            target_speed = float(cmd.get("speed", 0))
            target_alt = float(cmd.get("alt", vehicle.location.global_relative_frame.alt))

            print(f"Komut Alındı -> Yaw:{target_yaw:.0f}° Hız:{target_speed:.1f}m/s Alt:{target_alt:.1f}m", end='\r')

            # --- GÜVENLİK KONTROLÜ ---
            if vehicle.mode.name != "GUIDED":
                # Eğer pilot kumandadan modu değiştirdiyse müdahale etme!
                continue

            # 2. İrtifa Kontrolü (Simple Goto ile Z ekseni)
            # Mevcut konumun enlem/boylamını koru, sadece irtifayı değiştir
            current_loc = vehicle.location.global_relative_frame
            target_loc = LocationGlobalRelative(current_loc.lat, current_loc.lon, target_alt)
            vehicle.simple_goto(target_loc)

            # 3. Yön (Yaw) Kontrolü
            condition_yaw(vehicle, target_yaw, relative=False)

            # 4. Hız Kontrolü (Velocity Vector)
            # Hızı, gidilen yöne (Yaw) göre bileşenlerine ayır
            yaw_rad = math.radians(target_yaw)
            vx = target_speed * math.cos(yaw_rad) # Kuzey hızı
            vy = target_speed * math.sin(yaw_rad) # Doğu hızı
            
            # Z hızı 0 olsun, irtifayı simple_goto yönetiyor
            send_ned_velocity(vehicle, vx, vy, 0, 1)

        except KeyboardInterrupt:
            print("\n🛑 Adaptör kapatılıyor...")
            break
        except Exception as e:
            print(f"Hata: {e}")
            continue

    vehicle.close()
    sock.close()

if __name__ == "__main__":
    main()