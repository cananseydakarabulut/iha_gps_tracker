import socket
import select
import json
import sys
import math
import time
import numpy as np

from config import VehicleConfig, KFConfig, SimCfg
from ukf import KF3D_UKF
from geo_utils import llh_to_enu
from rival_tracker import RivalTracker
from guidance import GpsPursuitController, SAHA_YARICAPI
from quaternion_utils import q_to_euler_bounded

# -----------------------------
# PORT AYARLARI (BASE - Her IHA kendi ID'sine gore offset ekler)
# -----------------------------
LISTEN_IP = "127.0.0.1"
BASE_LISTEN_PORT = 5799  # IHA-1: 5799, IHA-2: 5800, IHA-3: 5801 ...
BASE_CONTROL_PORT = 5771  # IHA-1: 5771, IHA-2: 5772, IHA-3: 5773 ...

BUFFER_SIZE = 8192

# Gercek saha ayarlari (Teknofest)
SAHA_YARICAPI = 500.0  # metre
SAHA_DISI_TIMEOUT_S = 25.0  # 30s kural icin marj
SAHA_DISI_WARNING_S = 15.0

TELEMETRY_TIMEOUT_S = 2.0  # veri yoksa fallback

LOCK_THRESHOLD_M = 30.0  # Kamera menzili - 30m
LOCK_ANGLE_DEG = 15.0     # Kamera için yaw hizalanma toleransı - 15 derece
LOCK_ALTITUDE_DIFF_M = 8.0  # İrtifa farkı toleransı - 8m
LOCK_DURATION = 5.0       # Kilit süresi (Teknofest kuralı)
SAFETY_MIN_SPEED = 15.0   # Sabit kanat stall hızı
SAFETY_MAX_SPEED = 30.0   # Sabit kanat için güvenli maksimum (Teknofest)
SAFETY_MIN_ALT = 20.0     # Minimum güvenli irtifa
SAFETY_MAX_ALT = 150.0    # Maksimum irtifa (Teknofest sınırı)

# Çarpışma önleme (çoklu araç senaryosu için)
COLLISION_AVOIDANCE_DISTANCE = 15.0  # Kritik mesafe - acil kaçış (metre)
COLLISION_CHECK_RADIUS = 50.0        # Tüm rakipleri kontrol et (metre)


def run_receiver_node():
    # Konfigürasyon yükle
    vehicle_cfg = VehicleConfig()
    sim_cfg = SimCfg()
    kf_cfg = KFConfig()

    # Her IHA kendi portlarını kullanır
    listen_port = BASE_LISTEN_PORT + vehicle_cfg.vehicle_id - 1
    control_port = BASE_CONTROL_PORT + vehicle_cfg.vehicle_id - 1

    print("=" * 60)
    print(f"GPS Takip Sistemi v{vehicle_cfg.version}")
    print(f"IHA ID: {vehicle_cfg.vehicle_id} | Takim: {vehicle_cfg.team_id} | Tip: {vehicle_cfg.vehicle_type}")
    print(f"Dinleme Portu: {listen_port} | Kontrol Portu: {control_port}")
    print("=" * 60)

    # ✅ FIX: Config.py'den sabit referans koordinatları al (GPS'ten değil!)
    from geo_utils import parse_if_dms
    ref_lat = parse_if_dms(sim_cfg.lat0)
    ref_lon = parse_if_dms(sim_cfg.lon0)
    ref_h = sim_cfg.h0
    print(f"[CONFIG] Sabit referans: {ref_lat:.6f}, {ref_lon:.6f}, h={ref_h:.1f}m")

    kf = KF3D_UKF(kf_cfg)
    rival_tracker = RivalTracker(ref_lat, ref_lon, ref_h, my_team_id=vehicle_cfg.team_id)
    guidance = GpsPursuitController()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind((LISTEN_IP, listen_port))
        sock.setblocking(False)
        print(f"Listening (non-blocking): {LISTEN_IP}:{listen_port}")
    except Exception as e:
        sys.exit(f"Socket Hatasi: {e}")

    control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    last_time = None
    is_init = False

    my_speed = 0.0
    lock_timer_start = None
    locking_target_id = None
    locked_target_id = None  # Kilitlenen hedef - değiştirme!
    outside_timer_start = None
    last_valid_xy = None

    last_telemetry_time = time.time()
    telemetry_timeout_count = 0
    last_valid_tm = None

    # VZ hesaplama için GPS altitude geçmişi
    last_gps_alt = None
    last_gps_time = None

    fallback_state = {"x": 0.0, "y": 0.0, "z": 50.0, "speed": 0.0, "yaw": 0.0}

    print("Telemetri bekleniyor (2s timeout)...")

    while True:
        try:
            t_now = time.time()
            tm = None
            telemetry_status = "NO_DATA"

            ready = select.select([sock], [], [], 0.01)
            if ready[0]:
                try:
                    data, _ = sock.recvfrom(BUFFER_SIZE)
                    tm = json.loads(data.decode("utf-8"))
                    last_telemetry_time = t_now
                    last_valid_tm = tm
                    telemetry_timeout_count = 0
                    telemetry_status = "ACTIVE"

                    # Rival-only paketleri (leader_to_rival.py'den gelen) yakala
                    if tm.get("type") == "rival_gps":
                        if rival_tracker is not None:
                            converted = []
                            for pkt in tm.get("packets", []):
                                konum = pkt.get("konum", {})
                                aci = konum.get("aci", {}) if isinstance(konum.get("aci", {}), dict) else {}
                                converted.append(
                                    {
                                        "takim_numarasi": pkt.get("takim_id"),
                                        "iha_enlem": konum.get("enlem"),
                                        "iha_boylam": konum.get("boylam"),
                                        "iha_irtifa": konum.get("irtifa"),
                                        "iha_hiz": konum.get("hiz"),
                                        "iha_yonelme": konum.get("yonelim"),
                                        "iha_dikilme": aci.get("pitch", 0.0),
                                        "iha_yatis": aci.get("roll", 0.0),
                                        "gecikme_ms": pkt.get("gecikme_ms", 50.0),
                                    }
                                )
                            if converted:
                                rival_tracker.update_from_server_response(converted)
                        continue
                except json.JSONDecodeError:
                    telemetry_status = "PARSE_ERROR"
                    tm = None
                except Exception:
                    telemetry_status = "READ_ERROR"
                    tm = None

            time_since_last = t_now - last_telemetry_time
            if tm is None:
                if time_since_last > TELEMETRY_TIMEOUT_S:
                    telemetry_timeout_count += 1
                    telemetry_status = "TIMEOUT"
                    if telemetry_timeout_count % 20 == 1:
                        print(f"Telemetri yok ({time_since_last:.1f}s), emniyet loiter")
                    # Veri gelmiyorsa hesaplama yapma, sadece güvenli komut gönder
                    safe_cmd = {
                        "type": "control",
                        "yaw": 0.0,
                        "speed": max(10.0, fallback_state["speed"]),
                        "alt": max(SAFETY_MIN_ALT, fallback_state["z"]),
                        "action": "loiter",
                    }
                    control_sock.sendto(json.dumps(safe_cmd).encode(), (LISTEN_IP, control_port))
                    time.sleep(0.1)
                    continue
                else:
                    time.sleep(0.01)
                    continue

            if tm is None or "time_s" not in tm or "gps" not in tm or "imu" not in tm:
                time.sleep(0.01)
                continue

            gps_valid = tm.get("gps", {}).get("is_valid", False)
            ground_speed = float(tm.get("gps", {}).get("speed_ms", tm.get("gps", {}).get("ground_speed", 0.0)))

            # GPS geçerli değilse ya da yerdeyse (hız yok, hedef de yok): sadece bekle/loiter, hesaplama yapma
            if (not gps_valid) or (ground_speed < 0.5 and (tm.get("targets") in (None, [], {}) or tm.get("target") is None)):
                safe_cmd = {
                    "type": "control",
                    "yaw": 0.0,
                    "speed": SAFETY_MIN_SPEED,
                    "alt": max(SAFETY_MIN_ALT, fallback_state["z"]),
                    "action": "loiter",
                }
                control_sock.sendto(json.dumps(safe_cmd).encode(), (LISTEN_IP, control_port))
                time.sleep(0.05)
                continue

            # dt'yi gerçek zamanla sınırla; paket zamanı güvenilmez olabilir
            t_now = time.time()
            if last_time is None:
                last_time = t_now
            dt = max(0.001, min(0.1, t_now - last_time))
            last_time = t_now

            if not is_init:
                if tm["gps"]["is_valid"]:
                    # ✅ Referans zaten config.py'den alındı, GPS'ten almaya gerek yok

                    pos0 = llh_to_enu(
                        tm["gps"]["lat"],
                        tm["gps"]["lon"],
                        tm["gps"]["alt"],
                        ref_lat,
                        ref_lon,
                        ref_h,
                    )

                    # GPS'ten gelen hiz ve heading bilgilerini al
                    gps_speed = float(tm["gps"].get("speed_ms", tm["gps"].get("ground_speed", 0.0)))
                    gps_heading_deg = float(tm["gps"].get("heading", 0.0))
                    gps_heading_rad = math.radians(gps_heading_deg)

                    # Hiz vektorunu hesapla (yatay duzlemde)
                    vx_init = gps_speed * math.cos(gps_heading_rad)
                    vy_init = gps_speed * math.sin(gps_heading_rad)
                    vz_init = 0.0  # Baslangicta dikey hiz sifir varsayiyoruz

                    v_init = np.array([vx_init, vy_init, vz_init])

                    # UKF'yi hiz bilgisiyle baslat
                    kf.initialize_from_pos(pos0, v_init=v_init)
                    is_init = True
                    print(f"[IHA-{vehicle_cfg.vehicle_id}] UKF Initialized | Pos: [{pos0[0]:.1f}, {pos0[1]:.1f}, {pos0[2]:.1f}] | Speed: {gps_speed:.1f} m/s")
                    fallback_state.update({"x": pos0[0], "y": pos0[1], "z": pos0[2], "speed": gps_speed})
                else:
                    print("GPS fix bekleniyor...", end="\r")
                continue

            acc_body = np.array([tm["imu"]["acc"][1], tm["imu"]["acc"][0], tm["imu"]["acc"][2]])
            gyr_body = np.array(
                tm["imu"].get("gyr", [0.0, 0.0, 0.0])
            )
            z_imu_raw = np.concatenate(
                (acc_body, np.array([gyr_body[1], gyr_body[0], gyr_body[2]]))
            )

            kf.predict(dt, z_imu_raw)

            if tm["gps"]["is_valid"] and ref_lat is not None:
                raw_pos = llh_to_enu(
                    tm["gps"]["lat"],
                    tm["gps"]["lon"],
                    tm["gps"]["alt"],
                    ref_lat,
                    ref_lon,
                    ref_h,
                )

                gps_speed = float(tm["gps"].get("speed_ms", tm["gps"].get("ground_speed", my_speed)))
                gps_heading_deg = float(tm["gps"].get("heading", 0.0))
                gps_heading_rad = math.radians(gps_heading_deg)

                # Vektörel hız bileşenleri (yatay)
                gps_vx = gps_speed * math.cos(gps_heading_rad)
                gps_vy = gps_speed * math.sin(gps_heading_rad)

                # VZ hesaplama - Altitude türevinden
                current_alt = tm["gps"]["alt"]
                if last_gps_alt is not None and last_gps_time is not None:
                    dt_gps = t_now - last_gps_time
                    if dt_gps > 0.01:  # Minimum zaman farkı kontrolü
                        vz_gps = (current_alt - last_gps_alt) / dt_gps
                        # Gürültü filtrele - makul dikey hız limitleri
                        vz_gps = np.clip(vz_gps, -15.0, 15.0)
                    else:
                        vz_gps = 0.0
                else:
                    vz_gps = 0.0

                last_gps_alt = current_alt
                last_gps_time = t_now

                # Airspeed sensor ölçümü (varsa)
                # ArduPilot VFR_HUD mesajındaki airspeed değeri
                airspeed_measured = float(tm["gps"].get("airspeed", 0.0))
                if airspeed_measured == 0.0:
                    # Eğer airspeed sensörü yoksa, ground speed'i kullan (yedek)
                    airspeed_measured = gps_speed

                # UKF ölçüm vektörü: [pos(3), acc_body(3), mag_dummy(3), vel(3), airspeed(1)]
                z_full = np.array(
                    [
                        raw_pos[0],
                        raw_pos[1],
                        raw_pos[2],
                        acc_body[0],
                        acc_body[1],
                        acc_body[2],
                        0.0,
                        0.0,
                        0.0,
                        gps_vx,
                        gps_vy,
                        vz_gps,
                        airspeed_measured,
                    ]
                )

                kf.update(z_full, dt, tm["gps"]["hdop"])

            est_x = float(kf.x[0].item())
            est_y = float(kf.x[1].item())
            est_z = float(kf.x[2].item())

            vx, vy, vz = kf.get_velocity_3d()
            my_speed = kf.get_speed()
            roll_d, pitch_d, yaw_d = q_to_euler_bounded(kf.x[3:7])

            fallback_state.update({"x": est_x, "y": est_y, "z": est_z, "speed": my_speed, "yaw": yaw_d})

            if gps_valid:
                last_valid_xy = (est_x, est_y)

                # Rüzgar tahminini güncelle (GPS ve body velocity ile)
                gps_vel = np.array([vx, vy, 0.0])  # ENU frame'de GPS hızı
                body_vel = np.array([my_speed, 0.0, 0.0])  # Body frame (basitleştirilmiş)
                guidance.wind_comp.update(gps_vel, body_vel, math.radians(yaw_d))

            if rival_tracker and "network_data" in tm:
                rival_tracker.update_from_server_response(tm["network_data"])

            # HEDEF SEÇİMİ: Kilitli hedef varsa onu kullan, yoksa en yakını seç
            if locked_target_id and rival_tracker:
                # Kilitlenmiş hedef var - sadece onu takip et!
                all_rivals = rival_tracker.get_all_rivals()
                if locked_target_id in all_rivals:
                    rival_data = all_rivals[locked_target_id]
                    target = {
                        "takim_numarasi": locked_target_id,
                        "x": rival_data["x"],
                        "y": rival_data["y"],
                        "z": rival_data["z"],
                        "v": rival_data["speed"],
                    }
                else:
                    # Kilitli hedef kayboldu
                    locked_target_id = None
                    target = rival_tracker.get_closest_rival(est_x, est_y, est_z, yaw_d)
            else:
                # Normal hedef seçimi
                target = rival_tracker.get_closest_rival(est_x, est_y, est_z, yaw_d) if rival_tracker else None

            # Saha disina cikmis hedefi takip etme
            if target:
                tgt_dist_center = math.hypot(target.get("x", 0.0), target.get("y", 0.0))
                if tgt_dist_center > SAHA_YARICAPI:
                    target = None
                    lock_timer_start = None
                    locking_target_id = None

            kilit = 0
            visual_mode = False
            cmd = {}
            t_id = 0
            t_dist = 0.0
            mode_str = "BEKLEME"

            dist_center_precheck = math.hypot(est_x, est_y)
            soft_geofence = 0.7 * SAHA_YARICAPI
            hard_geofence = 0.85 * SAHA_YARICAPI

            # ÖNCELİK 1: GEOFENCE KONTROLÜ (En yüksek öncelik - saha dışı tehlikesi)
            # Eğer saha sınırına çok yakınsak, çarpışma önlemeyi devre dışı bırak
            geofence_critical = dist_center_precheck > hard_geofence  # 425m+

            # ÖNCELİK 2: ÇARPIŞMA ÖNLEME (Sadece geofence güvenliyse)
            collision_threat = None
            min_threat_dist = float('inf')

            if rival_tracker and not geofence_critical:
                # Tüm rakipleri al ve mesafe kontrol et
                all_rivals = rival_tracker.get_all_rivals()
                for rival_id, rival_data in all_rivals.items():
                    rx = rival_data.get("x", 0.0)
                    ry = rival_data.get("y", 0.0)
                    rz = rival_data.get("z", est_z)

                    dist_to_rival = math.sqrt(
                        (rx - est_x)**2 +
                        (ry - est_y)**2 +
                        (rz - est_z)**2
                    )

                    # Kritik mesafede mi?
                    if dist_to_rival < COLLISION_AVOIDANCE_DISTANCE and dist_to_rival < min_threat_dist:
                        min_threat_dist = dist_to_rival
                        collision_threat = {"x": rx, "y": ry, "z": rz, "id": rival_id, "dist": dist_to_rival}

            # ACIL KAÇIŞ: Herhangi bir rakip çok yakınsa (VE geofence güvenli)
            if collision_threat and not geofence_critical:
                # En yakın tehlikeden kaç: ters yön + yüksel
                threat_x = collision_threat["x"]
                threat_y = collision_threat["y"]
                escape_yaw = (math.degrees(math.atan2(threat_y - est_y, threat_x - est_x)) + 180.0) % 360.0
                escape_alt = min(est_z + 20.0, SAFETY_MAX_ALT)  # +20m yüksel

                cmd = {
                    "type": "control",
                    "yaw": escape_yaw,
                    "speed": SAFETY_MAX_SPEED,
                    "alt": escape_alt,
                }
                mode_str = f"⚠ ACIL KAÇIŞ - Araç #{collision_threat['id']} ({collision_threat['dist']:.1f}m)"
                t_dist = collision_threat["dist"]

                # Kamera kilidi devam ediyorsa, hedefi koru!
                if lock_timer_start is not None and locking_target_id is not None:
                    locked_target_id = locking_target_id  # Kilitlenen hedefi sakla
                    lock_timer_start = None  # Timer'ı sıfırla (sonra devam edecek)
                    print(f"🚨 ÇARPIŞMA! Araç #{collision_threat['id']} ({collision_threat['dist']:.1f}m) - Hedef #{locked_target_id} KİLİTLİ KALIYOR, KAÇILIYOR!")
                else:
                    print(f"🚨 ÇARPIŞMA TEHLİKESİ! Araç #{collision_threat['id']} - {collision_threat['dist']:.1f}m - KAÇILIYOR!")

            # NORMAL TAKİP: Çarpışma riski yoksa
            elif target and dist_center_precheck < soft_geofence:
                g_cmd = guidance.compute_command(
                    (est_x, est_y, est_z),
                    my_speed,
                    target,
                )
                t_id = target["takim_numarasi"]
                t_dist = g_cmd.distance_to_target_m
                mode_str = g_cmd.mode

                # KAMERA KLDU çN 3D HZALANMA KONTROLÜ
                # 1. Mesafe kontrolü (kamera menzili içinde)
                # 2. Yaw hizalama (rakibin arkasında)
                # 3. İrtifa hizalama (aynı seviyede)

                yaw_err = abs(g_cmd.desired_yaw_deg - yaw_d)
                if yaw_err > 180:
                    yaw_err = 360 - yaw_err

                # İrtifa farkı kontrolü
                altitude_diff = abs(est_z - target.get("z", est_z))

                # Kamera kilit koşulları (3D hizalanma)
                lock_conditions_met = (
                    t_dist < LOCK_THRESHOLD_M and          # Mesafe: 30m içinde
                    yaw_err < LOCK_ANGLE_DEG and           # Yaw: 15 derece hizalı
                    altitude_diff < LOCK_ALTITUDE_DIFF_M   # İrtifa: 8m fark içinde
                )

                if lock_conditions_met:
                    if lock_timer_start is None or locking_target_id != t_id:
                        lock_timer_start = time.time()
                        locking_target_id = t_id
                        locked_target_id = t_id  # Hedefi kilitle - başka hedef arama!
                        print(f"🎯 KAMERA KILIT başladı: Hedef={t_id} (KİLİTLİ) | Mesafe={t_dist:.1f}m | Yaw={yaw_err:.1f}° | Alt={altitude_diff:.1f}m")

                    elapsed = time.time() - lock_timer_start
                    if elapsed >= LOCK_DURATION:
                        if rival_tracker.report_lock_event(t_id):
                            kilit = 1
                            visual_mode = True
                            mode_str = "KAMERA KILIT TAMAMLANDI"
                            lock_timer_start = None
                            locking_target_id = None
                            locked_target_id = None  # Hedef vuruldu, artık başka hedef ara
                            print(f"✓ HEDEF {t_id} KİLİTLENDİ VE VURULDU!")
                    else:
                        mode_str += f" | KILIT {LOCK_DURATION - elapsed:.1f}s | Y:{yaw_err:.0f}° A:{altitude_diff:.0f}m"
                else:
                    # Kilit koşulları kayboldu (ama locked_target varsa geri dönmeye çalışacak)
                    if lock_timer_start is not None and locked_target_id is None:
                        # Henüz kilitlenmemişse iptal et
                        print(f"Kilit kaybedildi: Mesafe={t_dist:.1f}m Yaw={yaw_err:.1f}° Alt={altitude_diff:.1f}m")
                        lock_timer_start = None
                        locking_target_id = None

                cmd = {
                    "type": "control",
                    "yaw": g_cmd.desired_yaw_deg,
                    "speed": g_cmd.desired_speed_ms,
                    "alt": g_cmd.desired_altitude_m,
                }
            else:
                # Hedef yoksa veya sinir tamponu: merkeze dogru loiter
                center_yaw = math.degrees(math.atan2(-est_y, -est_x)) % 360
                cmd = {
                    "type": "control",
                    "yaw": center_yaw if dist_center_precheck > soft_geofence else yaw_d,
                    "speed": 18.0,
                    "alt": 100.0,
                    "action": "loiter" if dist_center_precheck > soft_geofence else cmd.get("action", None),
                }
                mode_str = "SINIR TAMPONU" if dist_center_precheck > soft_geofence else "HEDEF YOK"
                lock_timer_start = None
                locking_target_id = None

            # Hedef yoksa VEYA telemetri sorunluysa sinira yaklasinca merkeze yonel
            dist_center_precheck = math.hypot(est_x, est_y)
            if dist_center_precheck > hard_geofence and (target is None or telemetry_status != "ACTIVE"):
                center_yaw = math.degrees(math.atan2(-est_y, -est_x)) % 360
                cmd = {
                    "type": "control",
                    "yaw": center_yaw,
                    "speed": min(20.0, cmd.get("speed", 18.0)),
                    "alt": max(80.0, cmd.get("alt", 80.0)),
                    "action": "loiter",
                }
                mode_str = "HEDEF YOK - SINIRDA, MERKEZE DON"
                lock_timer_start = None
                locking_target_id = None

            # Visual kilit sonrasi: KAMERA KLDU - TAM ARKA HZALANMA
            if (kilit or visual_mode) and cmd:
                if target:
                    # Rakibin TAM ARKASINA hizalan (rakibin heading'ine bak)
                    cmd["yaw"] = target.get("yaw_deg", cmd.get("yaw", yaw_d)) % 360.0

                    # Rakiple TAM AYNI irtifada ol (kamera için kritik)
                    cmd["alt"] = target.get("z", cmd.get("alt", 50.0))

                    # Hızı rakiple TAM AYNI hızda eşitle - İDEAL MESAFE TAKİBİ
                    target_speed = target.get("hiz", 15.0)

                    # İDEAL MESAFE (15-30m) - TAM EŞİTLE
                    if 15.0 <= t_dist <= 30.0:
                        # KAMERA AÇISI İÇİN İDEAL - Rakiple TAM AYNI HIZ
                        cmd["speed"] = min(target_speed, SAFETY_MAX_SPEED)

                    # ÇOK YAKIN (<15m) - ÇARPIŞMA RİSKİ!
                    elif t_dist < 15.0:
                        if target_speed < 15.0:
                            # Yavaş hedef + çok yakın → ACİL FREN!
                            cmd["speed"] = max(6.0, target_speed * 0.6)
                        else:
                            # Hızlı hedef + çok yakın → Kontrollü yavaşla
                            cmd["speed"] = max(6.0, min(target_speed * 0.85, 20.0))

                    # UZAK (>30m) - YAKLAŞ
                    else:  # t_dist > 30.0
                        if target_speed < 15.0:
                            # Yavaş hedef + uzak → Dikkatli yaklaş
                            cmd["speed"] = min(target_speed + 3.0, 20.0)
                        else:
                            # Hızlı hedef + uzak → Hızlı yaklaş
                            cmd["speed"] = min(target_speed + 5.0, SAFETY_MAX_SPEED)

                    mode_str = "KILIT | KAMERA AKTIF | ARKA HIZA"
                else:
                    # Hedef yoksa güvenli konum
                    cmd["speed"] = 10.0
                    cmd["alt"] = max(cmd.get("alt", 50.0), 50.0)
                    mode_str = "KILIT | HEDEF KAYIP"

            # GÜÇLÜ Saha dışı kontrol - ÖNCELİK EN YÜKSEK! (Teknofest 30s kuralı)
            dist_from_center = math.hypot(est_x, est_y)

            # Kritik saha dışı - ACİL DÖNÜŞ + 30s SAYACI
            if dist_from_center > SAHA_YARICAPI:
                # 30 saniye sayacını başlat
                if outside_timer_start is None:
                    outside_timer_start = time.time()
                    print(f"\n⚠️ SAHA DIŞINA ÇIKTI! 30 saniye içinde geri dönülmeli!")

                outside_duration = time.time() - outside_timer_start
                remaining_time = SAHA_DISI_TIMEOUT_S - outside_duration

                # 30 saniye kontrolü
                if outside_duration > SAHA_DISI_TIMEOUT_S:
                    mode_str = f"🚫 DİSKALİFİYE! {dist_from_center:.0f}m - 30s AŞILDI"
                    print(f"\n🚫 DİSKALİFİYE: {outside_duration:.1f}s saha dışında kaldı!")
                    # Acil iniş komutu (yarış bitti)
                    cmd = {
                        "type": "control",
                        "yaw": 0.0,
                        "speed": 15.0,
                        "alt": 20.0,
                        "action": "land",  # İniş yap
                    }
                elif outside_duration > SAHA_DISI_WARNING_S:
                    # 15 saniye uyarısı
                    center_yaw = math.degrees(math.atan2(-est_y, -est_x)) % 360
                    cmd = {
                        "type": "control",
                        "yaw": center_yaw,
                        "speed": 30.0,  # Maksimum hızla dön
                        "alt": 80.0,
                        "action": "rtl",
                    }
                    mode_str = f"🚨 KRİTİK! {dist_from_center:.0f}m - {remaining_time:.1f}s KALDI!"
                    if int(outside_duration) % 2 == 0:  # Her 2 saniyede uyar
                        print(f"⏰ UYARI: {remaining_time:.1f}s içinde saha içine dönülmeli!")
                else:
                    # İlk 15 saniye - normal RTL
                    center_yaw = math.degrees(math.atan2(-est_y, -est_x)) % 360
                    cmd = {
                        "type": "control",
                        "yaw": center_yaw,
                        "speed": 25.0,
                        "alt": 80.0,
                        "action": "rtl",
                    }
                    mode_str = f"🚨 SAHA DIŞI {dist_from_center:.0f}m - RTL ({remaining_time:.1f}s)"

                lock_timer_start = None
                locking_target_id = None

            # Uyarı bölgesi - yavaşla ve merkeze yönel
            elif dist_from_center > SAHA_YARICAPI * 0.90:  # %90 sınırda (450m)
                # Saha içine geri döndü - sayacı sıfırla
                if outside_timer_start is not None:
                    print(f"\n✅ SAHA İÇİNE GERİ DÖNÜLDÜ! ({outside_duration:.1f}s saha dışında kaldı)")
                    outside_timer_start = None

                center_yaw = math.degrees(math.atan2(-est_y, -est_x)) % 360
                # Mevcut komutu değiştir - merkeze yönelme ekle
                if cmd and "yaw" in cmd:
                    # Hedef yaw ile merkez yaw'ını karıştır (merkeze öncelik ver)
                    target_yaw = cmd["yaw"]
                    blend_factor = (dist_from_center - SAHA_YARICAPI * 0.90) / (SAHA_YARICAPI * 0.10)
                    cmd["yaw"] = (target_yaw * (1 - blend_factor) + center_yaw * blend_factor) % 360
                    cmd["speed"] = min(cmd.get("speed", 20.0), 20.0)  # Hızı sınırla
                    mode_str = f"⚠️ SINIR YAKIN {dist_from_center:.0f}m - MERKEZE YÖN"
            else:
                # Güvenli bölgede - sayacı sıfırla
                if outside_timer_start is not None:
                    print(f"\n✅ SAHA İÇİNE GERİ DÖNÜLDÜ! ({time.time() - outside_timer_start:.1f}s saha dışında kaldı)")
                    outside_timer_start = None

            if cmd:
                cmd["speed"] = max(SAFETY_MIN_SPEED, min(SAFETY_MAX_SPEED, cmd.get("speed", SAFETY_MIN_SPEED)))
                cmd["alt"] = max(SAFETY_MIN_ALT, min(SAFETY_MAX_ALT, cmd.get("alt", SAFETY_MIN_ALT)))

            control_sock.sendto(json.dumps(cmd).encode(), (LISTEN_IP, control_port))

            dist_center = math.hypot(est_x, est_y)
            marker = "DIS" if dist_center > SAHA_YARICAPI else "IC"

            # Rakip konum bilgisi
            target_pos_str = ""
            if target:
                tx = target.get("x", 0.0)
                ty = target.get("y", 0.0)
                tz = target.get("z", 0.0)
                target_pos_str = f" Tgt:[{tx:.0f},{ty:.0f},{tz:.0f}]"

            # Hız vektörü bilgisi
            vel_str = f"V:[{vx:.1f},{vy:.1f},{vz:.1f}]"

            print(
                f"[IHA-{vehicle_cfg.vehicle_id}] {marker} Me:[{est_x:.0f},{est_y:.0f},{est_z:.0f}] {vel_str} "
                f"Spd:{my_speed:.1f} | Tgt:{t_id}{target_pos_str} Dst:{t_dist:.0f}m | {mode_str}",
                end="\r",
            )

        except KeyboardInterrupt:
            print("\nDurduruldu")
            break
        except KeyError as e:
            print(f"\nTelemetri ANAHTAR HATASI: {e} - Paket yapisi hatali")
            if tm:
                print(f"Gelen paket anahtarlari: {list(tm.keys())}")
            time.sleep(0.1)
            continue
        except Exception as e:
            print(f"\nGENEL HATA: {type(e).__name__}: {e}")
            import traceback
            traceback.print_exc()
            time.sleep(0.1)
            continue

    sock.close()
    control_sock.close()
    print("Sistem kapatildi")


if __name__ == "__main__":
    run_receiver_node()



