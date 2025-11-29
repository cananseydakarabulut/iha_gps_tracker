import math
import time
import random
import socket
import json
from datetime import datetime

# --- CONFIG.PY DESTEĞİ ---
try:
    from config import SimCfg
    from geo_utils import parse_if_dms
    REF_LAT = parse_if_dms(SimCfg.lat0)
    REF_LON = parse_if_dms(SimCfg.lon0)
    REF_ALT = SimCfg.h0
except ImportError:
    REF_LAT, REF_LON, REF_ALT = 41.508775, 36.118335, 38.0

# ==========================
# AYARLAR
# ==========================
# Bu sunucu veriyi sitl_drone.py'ye (Donanım Simülatörüne) atar
UDP_IP = "127.0.0.1"
TARGET_PORT = 6000 

# ==========================
# YARDIMCI: METRE -> GPS ÇEVİRİCİ
# ==========================
def enu_to_llh(x, y, z, ref_lat, ref_lon, ref_h):
    earth_radius = 6378137.0
    lat_rad = math.radians(ref_lat)
    d_lat = y / earth_radius
    d_lon = x / (earth_radius * math.cos(lat_rad))
    return ref_lat + math.degrees(d_lat), ref_lon + math.degrees(d_lon), ref_h + z

# ==========================
# RAKİP HAREKET MODELLERİ (SENİN ORİJİNAL KODUN)
# ==========================
def simulate_target_uav(t: float, tid: int):

    # 🟩 SENİN EKLEDİĞİN ÖLÇEK KODU (AYNEN DURUYOR)
    ARENA_SCALE = 0.35   # 0.35 = %35’e küçült
    # 🟩 

    # --- TAKIM 2: SEKİZ ÇİZME ---
    if tid == 2:
        scale = 200.0
        speed = 32.0
        tau = t * 0.25

        x = scale * math.cos(tau)
        y = (scale / 2) * math.sin(tau) * math.cos(tau)
        z = 120.0 + 10.0 * math.sin(t * 0.3)

        next_tau = tau + 0.05
        nx = scale * math.cos(next_tau)
        ny = (scale / 2) * math.sin(next_tau) * math.cos(next_tau)
        yaw = math.degrees(math.atan2(ny - y, nx - x))

        roll = 18.0 * math.sin(tau)
        pitch = 4.0 * math.sin(t * 0.2)

    # --- TAKIM 3: ZİG-ZAG ---
    elif tid == 3:
        forward_speed = 35.0
        x = -400.0 + (forward_speed * t) % 800.0

        zigzag_period = 8.0
        amplitude = 100.0
        y = amplitude * math.sin((2 * math.pi / zigzag_period) * t)

        z = 110.0 + 10.0 * math.cos(t * 0.4)

        vy = amplitude * (2 * math.pi / zigzag_period) * math.cos((2 * math.pi / zigzag_period) * t)
        vx = forward_speed
        speed = math.sqrt(vx**2 + vy**2)

        yaw = math.degrees(math.atan2(vy, vx))
        roll = -28.0 * math.cos((2 * math.pi / zigzag_period) * t)
        pitch = -3.0

    # --- TAKIM 4: SPİRAL ---
    else:
        radius = 180.0
        w = 0.35
        x = radius * math.cos(t * w)
        y = radius * math.sin(t * w)
        z = 60.0 + (t * 7) % 180

        speed = 33.0
        yaw = (math.degrees(math.atan2(y, x)) + 90) % 360

        roll = 22.0
        pitch = 8.0

    yaw = yaw % 360

    # 🟩 SENİN EKLEDİĞİN KONUM KÜÇÜLTME (AYNEN DURUYOR)
    x *= ARENA_SCALE
    y *= ARENA_SCALE
    z *= 1.0     
    # 🟩 

    return {
        "x": x, "y": y, "z": z,
        "speed": speed, "yaw": yaw,
        "pitch": pitch, "roll": roll
    }

# ==========================
# ANA SİMÜLASYON DÖNGÜSÜ
# ==========================
def main():
    # Sadece GÖNDEREN soket (Dinleme yapmaz -> HATA VERMEZ)
    sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"📡 Mock Server Başladı! (Senin Orijinal Hareket Kodlarınla)")
    print(f"   📤 Hedef Port: {TARGET_PORT} (sitl_drone dinleyecek)")

    start_time = time.time()

    while True:
        now = time.time()
        t = now - start_time
        dt_obj = datetime.now()

        # --- SADECE RAKİPLERİ SİMÜLE ET VE GÖNDER ---
        # (Senin İHA'nı yönetme işini sitl_drone.py'ye bıraktık, doğrusu bu)
        
        network_data = []
        for tid in [2, 3, 4]:
            st = simulate_target_uav(t, tid)
            
            # ENU -> GPS
            lat, lon, alt = enu_to_llh(st["x"], st["y"], st["z"], REF_LAT, REF_LON, REF_ALT)
            
            # Teknofest Formatı (Senin istediğin gibi eksiksiz)
            network_data.append({
                "takim_numarasi": tid,
                "iha_enlem": lat, "iha_boylam": lon, "iha_irtifa": alt,
                "iha_dikilme": st["pitch"], "iha_yonelme": st["yaw"], "iha_yatis": st["roll"],
                "iha_hiz": st["speed"], 
                "hedef_merkez_X": 0, "hedef_merkez_Y": 0, "hedef_genislik": 0, "hedef_yukseklik": 0,
                "gps_saati": {"saat": dt_obj.hour, "dakika": dt_obj.minute, "saniye": dt_obj.second, "milisaniye": 0},
                "gecikme_ms": random.uniform(20, 80)
            })

        # Veriyi JSON yapıp gönder
        try:
            sock_out.sendto(json.dumps(network_data).encode(), (UDP_IP, TARGET_PORT))
            print(f"📤 Veri Basılıyor... [Zaman: {t:.1f}s | {len(network_data)} Rakip]", end='\r')
        except Exception as e:
            print(f"Hata: {e}")

        time.sleep(0.05) # 20Hz

if __name__ == "__main__":
    main()