import socket
import json
import math
import time
import select

# --- CONFIG VE GEO UTILS ENTEGRASYONU ---
try:
    from config import SimCfg
    from geo_utils import enu_to_llh, parse_if_dms
    
    REF_LAT = parse_if_dms(SimCfg.lat0)
    REF_LON = parse_if_dms(SimCfg.lon0)
    REF_ALT = SimCfg.h0
    print(f"✅ Config Yüklendi. Referans: {REF_LAT}, {REF_LON}")
except ImportError:
    REF_LAT, REF_LON, REF_ALT = 41.508775, 36.118335, 38.0
    print("⚠️ Config bulunamadı, varsayılan koordinat kullanılıyor.")

# ==========================
# AYARLAR
# ==========================
PORT_SERVER_DATA = 6000   # Mock Server -> SITL
PORT_COMMANDS    = 5771   # gps.py -> SITL komutları
PORT_GPS_NODE    = 5799   # SITL -> gps.py telemetri

# ==========================
# SAHA LIMITİ
# ==========================
FIELD_RADIUS = 600.0       # metre
SAFE_RETURN_ALT = 50.0     # geri dönüş irtifası
SAFE_RETURN_SPEED = 20.0   # geri dönüş hızı

# ==========================
# ANA FONKSİYON
# ==========================
def main():

    # 1. Rakipleri Dinleyen Soket
    sock_server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock_server.bind(("0.0.0.0", PORT_SERVER_DATA))
        sock_server.setblocking(False)
    except OSError:
        print(f"❌ HATA: Port {PORT_SERVER_DATA} dolu!")
        return
    
    # 2. gps.py’den Komut Soketi
    sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock_cmd.bind(("0.0.0.0", PORT_COMMANDS))
        sock_cmd.setblocking(False)
    except OSError:
        print(f"❌ HATA: Port {PORT_COMMANDS} dolu!")
        return
    
    # 3. Telemetri Gönderme Soketi
    sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("🚁 SITL İHA BAŞLADI")
    print("📍 Pozisyon başlangıcı: (0,0,50)")

    # -----------------------------
    # İHA DURUMU (ASLA SİLMEDİM)
    # -----------------------------
    my_state = {
        "x": 0.0,
        "y": 0.0,
        "z": 50.0,
        "yaw": 0.0,
        "speed": 12.0
    }

    last_enemies = []
    last_time = time.time()

    # ==========================
    # ANA DÖNGÜ
    # ==========================
    while True:
        now = time.time()
        dt = now - last_time
        if dt > 0.1: dt = 0.1
        if dt < 0.001: dt = 0.001
        last_time = now

        # -------------------------------------
        # 1) Veri OKUMA – rakipler + komutlar
        # -------------------------------------
        readable, _, _ = select.select([sock_server, sock_cmd], [], [], 0.001)

        for s in readable:
            try:
                data, _ = s.recvfrom(8192)
                
                # --- RAKİP LİSTESİ ---
                if s is sock_server:
                    last_enemies = json.loads(data.decode())

                    # Açı limitlerini uygula (YATAY, PITCH, ROLL)
                    for e in last_enemies:
                        e["iha_dikilme"] = max(-90, min(90, e.get("iha_dikilme", 0)))
                        e["iha_yatis"]   = max(-90, min(90, e.get("iha_yatis", 0)))
                        e["iha_yonelme"] = e.get("iha_yonelme", 0) % 360

                # --- gps.py KOMUTLARI ---
                elif s is sock_cmd:
                    cmd = json.loads(data.decode())
                    my_state["yaw"] = float(cmd.get("yaw", my_state["yaw"]))
                    my_state["speed"] = float(cmd.get("speed", my_state["speed"]))

                    # İrtifa smooth
                    target_alt = float(cmd.get("alt", my_state["z"]))
                    my_state["z"] += (target_alt - my_state["z"]) * 0.08

            except Exception as e:
                print(f"[HATA] Veri okuma: {e}")

        # -------------------------------------
        # 2) SAHA DIŞINA ÇIKTI MI?
        # -------------------------------------
        dist_center = math.sqrt(my_state["x"]**2 + my_state["y"]**2)

        if dist_center > FIELD_RADIUS:
            # merkeze dön
            return_yaw = math.degrees(math.atan2(-my_state["y"], -my_state["x"]))
            my_state["yaw"] = return_yaw % 360
            my_state["speed"] = SAFE_RETURN_SPEED
            my_state["z"] += (SAFE_RETURN_ALT - my_state["z"]) * 0.05

        # -------------------------------------
        # 3) FİZİK MOTORU
        # -------------------------------------
        yaw_rad = math.radians(my_state["yaw"])
        step = my_state["speed"] * dt

        my_state["x"] += step * math.cos(yaw_rad)
        my_state["y"] += step * math.sin(yaw_rad)

        # -------------------------------------
        # 4) GPS TELEMETRİ ÜRET
        # -------------------------------------
        my_lat, my_lon, my_alt = enu_to_llh(
            my_state["x"], my_state["y"], my_state["z"],
            REF_LAT, REF_LON, REF_ALT
        )

        full_packet = {
            "time_s": now,
            "gps": {
                "lat": my_lat,
                "lon": my_lon,
                "alt": my_alt,
                "is_valid": True,
                "hdop": 0.8
            },
            "imu": {
                "acc": [0,0,-9.81],
                "gyr": [0,0,0]
            },
            "network_data": last_enemies
        }

        try:
            sock_out.sendto(json.dumps(full_packet).encode(),
                            ("127.0.0.1", PORT_GPS_NODE))
        except:
            pass

        print(
            f"🚁 X={my_state['x']:.1f} "
            f"Y={my_state['y']:.1f} "
            f"Z={my_state['z']:.1f} "
            f"Yaw={my_state['yaw']:.1f}°  | "
            f"Rakip={len(last_enemies)}      ",
            end="\r"
        )

        # 50 Hz
        time.sleep(0.02)



if __name__ == "__main__":
    main()
