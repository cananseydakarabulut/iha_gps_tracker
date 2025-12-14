#!/usr/bin/env python3
"""
GPS Takip Sistemi - MAVLink Direkt Bağlantı Versiyonu
JSON bridge kullanmadan direkt MAVLink ile çalışır
"""

import sys
import math
import time
import numpy as np
from pymavlink import mavutil

from config import VehicleConfig, KFConfig, SimCfg
from ukf import KF3D_UKF
from geo_utils import llh_to_enu, parse_if_dms
from rival_tracker import RivalTracker
from guidance import GpsPursuitController  # SAHA_YARICAPI import etme - kendi değerimizi kullan
from quaternion_utils import angle_to_q, q_to_euler_bounded

# -----------------------------
# BAĞLANTI AYARLARI
# -----------------------------
# Saha yarıçapını SITL için büyüt (10 km)
SAHA_YARICAPI = 10000.0

# MAVLink bağlantı portları (UDP)
VEHICLE_UDP_PORT = 15550  # Vehicle 1 MAVLink UDP port
PEER_UDP_PORT = 15560     # Vehicle 2 (rakip) MAVLink UDP port

# Gercek saha ayarlari (Teknofest)
SAHA_DISI_TIMEOUT_S = 25.0
SAHA_DISI_WARNING_S = 15.0
TELEMETRY_TIMEOUT_S = 2.0

LOCK_THRESHOLD_M = 30.0
LOCK_ANGLE_DEG = 15.0
LOCK_ALTITUDE_DIFF_M = 8.0
LOCK_DURATION = 5.0
SAFETY_MIN_SPEED = 15.0
SAFETY_MAX_SPEED = 30.0
SAFETY_MIN_ALT = 20.0
SAFETY_MAX_ALT = 150.0

COLLISION_AVOIDANCE_DISTANCE = 15.0
COLLISION_CHECK_RADIUS = 50.0


def send_plane_reposition(master, lat, lon, alt_rel, yaw_deg=None, speed_ms=None, radius=0.0):
    """
    ArduPlane GUIDED komutu (MAV_CMD_DO_REPOSITION) gönderir.
    - lat/lon: float (deg)
    - alt_rel: float (m, relative)
    - yaw_deg: heading (deg) veya None
    - speed_ms: groundspeed (m/s) veya None (= autopilot varsayılanı)
    - radius: loiter yarıçapı (m) 0 => autopilot varsayılanı
    """
    if master is None:
        return
    heading = float('nan') if yaw_deg is None else float(yaw_deg)
    speed = -1.0 if speed_ms is None else float(speed_ms)

    master.mav.command_int_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        mavutil.mavlink.MAV_CMD_DO_REPOSITION,
        0,  # current (not used)
        0,  # autocontinue (not used)
        speed,        # param1: groundspeed (m/s) (-1 = default)
        0,            # param2: bitmask (0 = none)
        radius,       # param3: loiter radius (m) (0 = default)
        heading,      # param4: yaw (deg, NaN = current)
        int(lat * 1e7),
        int(lon * 1e7),
        float(alt_rel)
    )


def run_mavlink_node():
    """Ana MAVLink kontrol döngüsü"""

    # Konfigürasyon
    vehicle_cfg = VehicleConfig()
    sim_cfg = SimCfg()
    kf_cfg = KFConfig()

    print("=" * 60)
    print(f"GPS Takip Sistemi v{vehicle_cfg.version} - MAVLink Edition")
    print(f"IHA ID: {vehicle_cfg.vehicle_id} | Takim: {vehicle_cfg.team_id} | Tip: {vehicle_cfg.vehicle_type}")
    print("=" * 60)

    # Sabit referans koordinatları
    ref_lat = parse_if_dms(sim_cfg.lat0)
    ref_lon = parse_if_dms(sim_cfg.lon0)
    ref_h = sim_cfg.h0
    print(f"[CONFIG] Sabit referans: {ref_lat:.6f}, {ref_lon:.6f}, h={ref_h:.1f}m")

    # Modüller
    kf = KF3D_UKF(kf_cfg)
    rival_tracker = RivalTracker(ref_lat, ref_lon, ref_h, my_team_id=vehicle_cfg.team_id, arena_radius=10000.0, allow_same_team=False)
    guidance = GpsPursuitController()

    # MAVLink bağlantıları (UDP)
    print(f"[MAVLink] Kendi aracımıza bağlanıyor: udpin:127.0.0.1:{VEHICLE_UDP_PORT}")
    master = mavutil.mavlink_connection(f'udpin:127.0.0.1:{VEHICLE_UDP_PORT}')
    master.wait_heartbeat()
    print(f"[MAVLink] ✓ Heartbeat alındı (sysid={master.target_system})")

    print(f"[MAVLink] Rakip araca bağlanıyor: udpin:127.0.0.1:{PEER_UDP_PORT}")
    peer = mavutil.mavlink_connection(f'udpin:127.0.0.1:{PEER_UDP_PORT}')
    peer.wait_heartbeat()
    print(f"[MAVLink] ✓ Rakip heartbeat alındı (sysid={peer.target_system})")
    print("[RivalTracker] Kendi takim ID:", vehicle_cfg.team_id)

    # Değişkenler
    last_time = None
    is_init = False
    my_speed = 0.0
    lock_timer_start = None
    locking_target_id = None
    locked_target_id = None
    outside_timer_start = None
    last_valid_xy = None
    last_telemetry_time = time.time()
    telemetry_timeout_count = 0
    last_gps_alt = None
    last_gps_time = None

    # MAVLink mesaj cache
    my_gpos = None
    peer_gpos_last = None
    my_vfr = None
    my_attitude = None
    my_imu = None
    last_hold_lat = None
    last_hold_lon = None
    last_hold_alt = None
    last_loiter_cmd_time = 0.0

    print("\nTelemetri bekleniyor...")

    try:
        while True:
            t_now = time.time()

            # ─────────────────────────────────────────────────
            # 1. KENDİ ARACIMIZIN TELEMETRİSİNİ AL
            # ─────────────────────────────────────────────────
            msg = master.recv_match(type=['GLOBAL_POSITION_INT', 'VFR_HUD', 'ATTITUDE', 'HIGHRES_IMU', 'SCALED_IMU2'], blocking=False, timeout=0.01)

            if msg:
                last_telemetry_time = t_now
                telemetry_timeout_count = 0

                mtype = msg.get_type()
                if mtype == 'GLOBAL_POSITION_INT':
                    my_gpos = msg
                    last_hold_lat = my_gpos.lat / 1e7
                    last_hold_lon = my_gpos.lon / 1e7
                    last_hold_alt = my_gpos.alt / 1000.0
                elif mtype == 'VFR_HUD':
                    my_vfr = msg
                elif mtype == 'ATTITUDE':
                    my_attitude = msg
                elif mtype in ['HIGHRES_IMU', 'SCALED_IMU2']:
                    my_imu = msg

            # ─────────────────────────────────────────────────
            # 2. RAKİP ARACIN TELEMETRİSİNİ AL
            # ─────────────────────────────────────────────────
            peer_msg = peer.recv_match(type=['GLOBAL_POSITION_INT', 'VFR_HUD', 'ATTITUDE'], blocking=False, timeout=0.01)

            if peer_msg:
                # Rakip telemetrisini network_data formatına çevir
                if peer_msg.get_type() == 'GLOBAL_POSITION_INT':
                    peer_gpos = peer_msg
                    peer_gpos_last = peer_msg
                    # RivalTracker'a gönder
                    network_data = [{
                        "takim_numarasi": 99,  # Rakip takım ID (peer_team_id)
                        "iha_enlem": peer_gpos.lat / 1e7,
                        "iha_boylam": peer_gpos.lon / 1e7,
                        "iha_irtifa": peer_gpos.alt / 1000.0,
                        "iha_hiz": math.sqrt((peer_gpos.vx/100)**2 + (peer_gpos.vy/100)**2 + (peer_gpos.vz/100)**2),
                        "iha_yonelme": peer_gpos.hdg / 100.0 if peer_gpos.hdg != 65535 else 0.0,
                        "iha_dikilme": 0.0,
                        "iha_yatis": 0.0,
                        "gecikme_ms": 100.0
                    }]
                    rival_tracker.update_from_server_response(network_data)

            # ─────────────────────────────────────────────────
            # 3. TELEMETRİ TIMEOUT KONTROLÜ
            # ─────────────────────────────────────────────────
            time_since_last = t_now - last_telemetry_time
            if my_gpos is None:
                if time_since_last > TELEMETRY_TIMEOUT_S:
                    telemetry_timeout_count += 1
                    if telemetry_timeout_count % 10 == 1:
                        print(f"\n⚠️ Telemetri yok ({time_since_last:.1f}s) - tutma komutu gönderiliyor...")
                    # Son bilinen pozisyon varsa HOLD komutu gonder (ArduPlane)
                    if last_hold_lat is not None:
                        send_plane_reposition(
                            master,
                            last_hold_lat,
                            last_hold_lon,
                            last_hold_alt if last_hold_alt is not None else 50.0,
                            yaw_deg=None,
                            speed_ms=SAFETY_MIN_SPEED,
                            radius=80.0,
                        )
                    continue

            # ─────────────────────────────────────────────────
            # 4. GPS VE SENSÖR VERİLERİNİ PARSE ET
            # ─────────────────────────────────────────────────
                if not my_gpos:
                    time.sleep(0.1)
                    continue

            # GPS pozisyonu
            lat = my_gpos.lat / 1e7
            lon = my_gpos.lon / 1e7
            alt = my_gpos.alt / 1000.0

            # GPS geçerlilik kontrolü (fix_type >= 3)
            gps_valid = True  # ArduPilot SITL'de her zaman geçerli
            if not gps_valid:
                if telemetry_timeout_count % 20 == 1:
                    print(f"⚠️ GPS geçersiz - KOMUT GÖNDERİLMİYOR!")
                time.sleep(0.1)
                continue

            # ENU koordinatlarına çevir
            x, y, z = llh_to_enu(lat, lon, alt, ref_lat, ref_lon, ref_h)

            # Hız bilgisi
            if my_vfr:
                my_speed = my_vfr.groundspeed
            else:
                vx = my_gpos.vx / 100.0
                vy = my_gpos.vy / 100.0
                my_speed = math.sqrt(vx*vx + vy*vy)

            # VZ hesaplama (dikey hız)
            vz = my_gpos.vz / 100.0 if my_gpos else 0.0

            # IMU verisi
            if my_imu:
                if hasattr(my_imu, 'xacc'):  # HIGHRES_IMU
                    acc = np.array([my_imu.xacc, my_imu.yacc, my_imu.zacc])
                    gyr = np.array([my_imu.xgyro, my_imu.ygyro, my_imu.zgyro])
                elif hasattr(my_imu, 'xacc'):  # SCALED_IMU2
                    acc = np.array([my_imu.xacc/1000*9.81, my_imu.yacc/1000*9.81, my_imu.zacc/1000*9.81])
                    gyr = np.array([my_imu.xgyro/1000, my_imu.ygyro/1000, my_imu.zgyro/1000])
                else:
                    acc = np.array([0.0, 0.0, -9.81])
                    gyr = np.array([0.0, 0.0, 0.0])
            else:
                acc = np.array([0.0, 0.0, -9.81])
                gyr = np.array([0.0, 0.0, 0.0])

            # ─────────────────────────────────────────────────
            # 5. UKF GÜNCELLEMESİ
            # ─────────────────────────────────────────────────
            if not is_init:
                heading_rad = math.atan2(my_gpos.vy, my_gpos.vx) if my_gpos else 0.0
                v_init = np.array([
                    my_speed * math.cos(heading_rad),
                    my_speed * math.sin(heading_rad),
                    vz,
                ])
                kf.initialize_from_pos(np.array([x, y, z]), v_init=v_init)
                kf.x[3:7, 0] = angle_to_q(0.0, 0.0, heading_rad)  # yaw'ı hizaya al
                is_init = True
                last_time = t_now
                kf_fail_count = 0
                print(f"[IHA-{vehicle_cfg.vehicle_id}] UKF Initialized | Pos: [{x:.1f}, {y:.1f}, {z:.1f}] | Speed: {my_speed:.1f} m/s")
                continue

            # KF güncelle
            dt = 0.1 if last_time is None else min(t_now - last_time, 1.0)
            last_time = t_now

            z_imu_raw = np.concatenate((acc, gyr))
            kf.predict(dt, z_imu_raw)

            gps_heading_rad = math.atan2(my_gpos.vy, my_gpos.vx) if my_gpos else 0.0
            gps_vx = my_gpos.vx / 100.0 if my_gpos else my_speed * math.cos(gps_heading_rad)
            gps_vy = my_gpos.vy / 100.0 if my_gpos else my_speed * math.sin(gps_heading_rad)
            vz_gps = my_gpos.vz / 100.0 if my_gpos else vz
            airspeed_measured = getattr(my_vfr, "airspeed", 0.0) or my_speed

            z_full = np.array([
                x, y, z,
                acc[0], acc[1], acc[2],
                0.0, 0.0, 0.0,
                gps_vx, gps_vy, vz_gps,
                airspeed_measured,
            ])
            ok = kf.update(z_full, dt, hdop_simulated=1.0)
            if not ok:
                # Ölçüm reddedildi: UKF'yi GPS'e resetle ki saha dışına taşmasın
                kf.initialize_from_pos(np.array([x, y, z]), v_init=np.array([gps_vx, gps_vy, vz_gps]))
                kf.x[3:7, 0] = angle_to_q(0.0, 0.0, gps_heading_rad)
                last_time = t_now
                print("⚠️ UKF güncellemesi reddedildi, GPS'e resetlendi", end="\r")
                continue

            est_x = float(kf.x[0, 0])
            est_y = float(kf.x[1, 0])
            est_z = float(kf.x[2, 0])

            vx, vy, vz_kf = kf.get_velocity_3d()
            my_speed = kf.get_speed()

            # ─────────────────────────────────────────────────
            # 6. HEDEF SEÇİMİ VE TAKİP
            # ─────────────────────────────────────────────────
            heading_deg = math.degrees(math.atan2(vy, vx)) % 360.0

            # KİLİTLİ HEDEF VARSA, SADECE ONU TAKİP ET! (gps.py:323-336)
            if locked_target_id is not None:
                rival_data = rival_tracker.get_rival_by_id(locked_target_id)
                if rival_data:
                    # Kilitli hedefi takip et
                    target = {
                        "takim_numarasi": locked_target_id,
                        "dist": math.sqrt((rival_data["x"]-est_x)**2 + (rival_data["y"]-est_y)**2 + (rival_data["z"]-est_z)**2),
                        "x": rival_data["x"],
                        "y": rival_data["y"],
                        "z": rival_data["z"],
                        "yaw_deg": rival_data.get("yaw", 0.0),
                        "hiz": rival_data.get("speed", 15.0),
                    }
                else:
                    # Kilitli hedef kayboldu
                    locked_target_id = None
                    target = rival_tracker.get_closest_rival(est_x, est_y, est_z, heading_deg)
            else:
                # Normal hedef seçimi (en yakın rakip)
                target = rival_tracker.get_closest_rival(est_x, est_y, est_z, heading_deg)

            # GÜVENLIK: Saha dışı hedefi takip etme
            if target:
                tgt_dist_center = math.hypot(target.get("x", 0.0), target.get("y", 0.0))
                if tgt_dist_center > SAHA_YARICAPI:
                    target = None
                    lock_timer_start = None
                    locking_target_id = None

            if target is None and peer_gpos_last:
                # RivalTracker boş olsa bile doğrudan rakip GPS'inden hedef oluştur
                p_lat = peer_gpos_last.lat / 1e7
                p_lon = peer_gpos_last.lon / 1e7
                p_alt = peer_gpos_last.alt / 1000.0
                px, py, pz = llh_to_enu(p_lat, p_lon, p_alt, ref_lat, ref_lon, ref_h)
                pvx = peer_gpos_last.vx / 100.0
                pvy = peer_gpos_last.vy / 100.0
                pspeed = math.sqrt(pvx*pvx + pvy*pvy + (peer_gpos_last.vz/100.0)**2)
                target = {
                    "takim_numarasi": 99,
                    "x": px,
                    "y": py,
                    "z": pz,
                    "hiz": pspeed,
                    "yaw_deg": (peer_gpos_last.hdg/100.0) if peer_gpos_last.hdg != 65535 else 0.0,
                    "dist": math.sqrt((px-est_x)**2 + (py-est_y)**2 + (pz-est_z)**2),
                }

            if target is None:
                # HEDEF YOK - POZİSYON TUT (Drift önleme)
                # Sabit nokta loiter (sabit kanat için)
                hold_lat = lat if last_hold_lat is None else last_hold_lat
                hold_lon = lon if last_hold_lon is None else last_hold_lon
                hold_alt = alt if last_hold_alt is None else last_hold_alt
                if t_now - last_loiter_cmd_time > 1.5:
                    master.mav.command_long_send(
                        master.target_system,
                        master.target_component,
                        mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
                        0,
                        0,          # param1 (boş)
                        0,          # param2 (boş)
                        80,         # param3: loiter yarıçapı (m)
                        float('nan'),  # param4: yönelim (kullanmıyoruz)
                        hold_lat,
                        hold_lon,
                        hold_alt
                    )
                    last_loiter_cmd_time = t_now
                print(f"\n⚠️ HEDEF YOK - LOITER ({hold_lat:.6f},{hold_lon:.6f},{hold_alt:.1f}m)", end="\r")
                time.sleep(0.1)
                continue

            t_id = target["takim_numarasi"]
            t_dist = target["dist"]
            target_speed = target.get("hiz", 20.0)

            # Çarpışma önleme (tüm rakipler)
            all_rivals = rival_tracker.get_all_rivals()
            collision_threat = None
            for rid, rdata in all_rivals.items():
                if rid == t_id:
                    continue
                rdist = math.sqrt((rdata["x"]-est_x)**2 + (rdata["y"]-est_y)**2 + (rdata["z"]-est_z)**2)
                if rdist < COLLISION_AVOIDANCE_DISTANCE:
                    collision_threat = {"id": rid, "dist": rdist}
                    break

            # Kamera kilit sistemi
            my_pos_tuple = (est_x, est_y, est_z)
            yaw_err = abs((heading_deg - target["yaw_deg"] + 180) % 360 - 180)
            altitude_diff = abs(est_z - target["z"])

            marker = "🎯"
            mode_str = ""

            # Kamera kilit durumu kontrolü
            kilit = locked_target_id == t_id
            visual_mode = (t_dist < LOCK_THRESHOLD_M and
                          yaw_err < LOCK_ANGLE_DEG and
                          altitude_diff < LOCK_ALTITUDE_DIFF_M)

            # Kilit süreç kontrolü
            if locked_target_id == t_id:
                # Kilitli hedef
                marker = "🔒"
                mode_str = f"KİLİTLİ #{t_id}"

            elif locking_target_id == t_id and lock_timer_start:
                # Kilitlenme süreci devam ediyor
                lock_elapsed = t_now - lock_timer_start
                if (t_dist < LOCK_THRESHOLD_M and
                    yaw_err < LOCK_ANGLE_DEG and
                    altitude_diff < LOCK_ALTITUDE_DIFF_M):

                    if lock_elapsed >= LOCK_DURATION:
                        # VURUŞ BAŞARILI! (gps.py:449-458)
                        if rival_tracker.report_lock_event(t_id):
                            kilit = 1
                            visual_mode = True
                            mode_str = "KAMERA KILIT TAMAMLANDI"
                            lock_timer_start = None
                            locking_target_id = None
                            locked_target_id = None  # ⚠️ TEMİZLE - yeni hedef ara!
                            print(f"\n✓ HEDEF {t_id} KİLİTLENDİ VE VURULDU!")
                    else:
                        marker = "🔒"
                        mode_str = f"KAMERA KILIT {lock_elapsed:.1f}/{LOCK_DURATION}s"
                        print(f"🎯 KAMERA KILIT başladı: Hedef={t_id} (KİLİTLİ) | Mesafe={t_dist:.1f}m | Yaw={yaw_err:.1f}° | Alt={altitude_diff:.1f}m")
                else:
                    # Kilit koşulları kayboldu (gps.py:461-467)
                    # Henüz kilitlenmemişse iptal et (locked_target varsa devam edecek)
                    if lock_timer_start is not None and locked_target_id is None:
                        print(f"Kilit kaybedildi: Mesafe={t_dist:.1f}m Yaw={yaw_err:.1f}° Alt={altitude_diff:.1f}m")
                        lock_timer_start = None
                        locking_target_id = None

            elif (t_dist < LOCK_THRESHOLD_M and
                  yaw_err < LOCK_ANGLE_DEG and
                  altitude_diff < LOCK_ALTITUDE_DIFF_M and
                  locked_target_id != t_id):
                # Yeni kilit başlat (gps.py:442-447)
                if lock_timer_start is None or locking_target_id != t_id:
                    lock_timer_start = t_now
                    locking_target_id = t_id
                    locked_target_id = t_id  # ⚠️ HEMEN SET ET - hedef değiştirmeyi önle!
                    marker = "🔒"
                    mode_str = "KAMERA KILIT BAŞLIYOR"
                    print(f"🎯 KAMERA KILIT başladı: Hedef={t_id} (KİLİTLİ) | Mesafe={t_dist:.1f}m | Yaw={yaw_err:.1f}° | Alt={altitude_diff:.1f}m")

            # ─────────────────────────────────────────────────
            # GEOFENCE VE PRİORİTE SİSTEMİ
            # ─────────────────────────────────────────────────
            dist_from_center = math.hypot(est_x, est_y)
            soft_geofence = 0.7 * SAHA_YARICAPI   # 350m (yumuşak sınır)
            hard_geofence = 0.85 * SAHA_YARICAPI  # 425m (kritik sınır)
            geofence_critical = dist_from_center > hard_geofence

            # PRİORİTE 1: SAHA DIŞI KONTROLÜ (EN YÜKSEK ÖNCELİK - Teknofest 30s kuralı)
            if dist_from_center > SAHA_YARICAPI:
                # 30 saniye sayacını başlat
                if outside_timer_start is None:
                    outside_timer_start = t_now
                    print(f"\n⚠️ SAHA DIŞINA ÇIKTI! 30 saniye içinde geri dönülmeli!")

                outside_duration = t_now - outside_timer_start
                remaining_time = SAHA_DISI_TIMEOUT_S - outside_duration

                # DİSKALİFİYE kontrolü (30s aşıldı)
                if outside_duration > SAHA_DISI_TIMEOUT_S:
                    mode_str = f"🚫 DİSKALİFİYE! {dist_from_center:.0f}m - 30s AŞILDI"
                    print(f"\n🚫 DİSKALİFİYE: {outside_duration:.1f}s saha dışında kaldı!")
                    # Acil iniş
                    center_yaw = math.degrees(math.atan2(-est_y, -est_x)) % 360
                    cmd_yaw = center_yaw
                    cmd_speed = SAFETY_MIN_SPEED
                    cmd_alt = 20.0
                    marker = "🚫"

                # KRİTİK UYARI (15-30s arası)
                elif outside_duration > SAHA_DISI_WARNING_S:
                    center_yaw = math.degrees(math.atan2(-est_y, -est_x)) % 360
                    cmd_yaw = center_yaw
                    cmd_speed = SAFETY_MAX_SPEED
                    cmd_alt = 80.0
                    mode_str = f"🚨 KRİTİK! {dist_from_center:.0f}m - {remaining_time:.1f}s KALDI!"
                    marker = "🚨"
                    if int(outside_duration) % 2 == 0:
                        print(f"⏰ UYARI: {remaining_time:.1f}s içinde saha içine dönülmeli!")

                # İLK UYARI (0-15s)
                else:
                    center_yaw = math.degrees(math.atan2(-est_y, -est_x)) % 360
                    cmd_yaw = center_yaw
                    cmd_speed = 25.0
                    cmd_alt = 80.0
                    mode_str = f"🚨 SAHA DIŞI {dist_from_center:.0f}m - RTL ({remaining_time:.1f}s)"
                    marker = "⚠️"

                lock_timer_start = None
                locking_target_id = None

            # UYARI BÖLGESİ (%90 sınırda - 450m)
            elif dist_from_center > SAHA_YARICAPI * 0.90:
                # Saha içine geri döndü - sayacı sıfırla
                if outside_timer_start is not None:
                    outside_duration = t_now - outside_timer_start
                    print(f"\n✅ SAHA İÇİNE GERİ DÖNÜLDÜ! ({outside_duration:.1f}s saha dışında kaldı)")
                    outside_timer_start = None

                # Normal guidance hesapla ama merkeze yönelme ekle
                center_yaw = math.degrees(math.atan2(-est_y, -est_x)) % 360

                if collision_threat and not geofence_critical:
                    # Çarpışma önleme
                    mode_str = f"🚨 ÇARPIŞMA ÖNLEME (#{collision_threat['id']})"
                    marker = "⚠️"
                    escape_yaw = (heading_deg + 90) % 360
                    cmd_yaw = escape_yaw
                    cmd_speed = SAFETY_MAX_SPEED
                    cmd_alt = max(SAFETY_MIN_ALT, est_z + 10)

                    # Kamera kilidi devam ediyorsa, hedefi koru! (gps.py:404-410)
                    if lock_timer_start is not None and locking_target_id is not None:
                        locked_target_id = locking_target_id  # Kilitlenen hedefi sakla
                        lock_timer_start = None  # Timer'ı sıfırla (sonra devam edecek)
                        print(f"🚨 ÇARPIŞMA! Araç #{collision_threat['id']} ({collision_threat['dist']:.1f}m) - Hedef #{locked_target_id} KİLİTLİ KALIYOR, KAÇILIYOR!")
                    else:
                        print(f"🚨 ÇARPIŞMA TEHLİKESİ! Araç #{collision_threat['id']} - {collision_threat['dist']:.1f}m - KAÇILIYOR!")
                else:
                    # Normal guidance ama merkeze karıştır
                    cmd = guidance.compute_command(my_pos_tuple, my_speed, target)
                    target_yaw = cmd.desired_yaw_deg
                    # Merkeze yönelme faktörü (sınıra yaklaştıkça artar)
                    blend_factor = (dist_from_center - SAHA_YARICAPI * 0.90) / (SAHA_YARICAPI * 0.10)
                    cmd_yaw = (target_yaw * (1 - blend_factor) + center_yaw * blend_factor) % 360
                    cmd_speed = min(cmd.desired_speed_ms, 20.0)  # Hız sınırla
                    cmd_alt = cmd.desired_altitude_m
                    mode_str = f"⚠️ SINIR YAKIN {dist_from_center:.0f}m - MERKEZE YÖN"
                    marker = "⚠️"

            # GÜVENLİ BÖLGE
            else:
                # Sayacı sıfırla
                if outside_timer_start is not None:
                    outside_duration = t_now - outside_timer_start
                    print(f"\n✅ SAHA İÇİNE GERİ DÖNÜLDÜ! ({outside_duration:.1f}s saha dışında kaldı)")
                    outside_timer_start = None

                # PRİORİTE 2: ÇARPIŞMA ÖNLEME (sadece geofence güvenliyse)
                if collision_threat and not geofence_critical:
                    mode_str = f"🚨 ÇARPIŞMA ÖNLEME (#{collision_threat['id']})"
                    marker = "⚠️"
                    escape_yaw = (heading_deg + 90) % 360
                    cmd_yaw = escape_yaw
                    cmd_speed = SAFETY_MAX_SPEED
                    cmd_alt = max(SAFETY_MIN_ALT, est_z + 10)

                    # Kamera kilidi devam ediyorsa, hedefi koru! (gps.py:404-410)
                    if lock_timer_start is not None and locking_target_id is not None:
                        locked_target_id = locking_target_id  # Kilitlenen hedefi sakla
                        lock_timer_start = None  # Timer'ı sıfırla (sonra devam edecek)
                        print(f"🚨 ÇARPIŞMA! Araç #{collision_threat['id']} ({collision_threat['dist']:.1f}m) - Hedef #{locked_target_id} KİLİTLİ KALIYOR, KAÇILIYOR!")
                    else:
                        print(f"🚨 ÇARPIŞMA TEHLİKESİ! Araç #{collision_threat['id']} - {collision_threat['dist']:.1f}m - KAÇILIYOR!")

                # PRİORİTE 3: NORMAL TAKİP
                else:
                    cmd = guidance.compute_command(my_pos_tuple, my_speed, target)
                    cmd_yaw = cmd.desired_yaw_deg
                    cmd_speed = cmd.desired_speed_ms
                    cmd_alt = cmd.desired_altitude_m
                    if not mode_str:
                        mode_str = cmd.mode

            # ─────────────────────────────────────────────────
            # PRİORİTE 4: KAMERA KİLİT HIZ UYARLAMASI
            # ─────────────────────────────────────────────────
            # Visual kilit sonrası: KAMERA KİLİTLİ - TAM ARKA HİZALANMA
            if (kilit or visual_mode) and target:
                # Rakibin TAM ARKASINA hizalan (rakibin heading'ine bak)
                cmd_yaw = target.get("yaw_deg", cmd_yaw) % 360.0

                # Rakiple TAM AYNI irtifada ol (kamera için kritik)
                cmd_alt = target.get("z", cmd_alt)

                # Hızı rakiple TAM AYNI hızda eşitle - İDEAL MESAFE TAKİBİ
                target_speed = target.get("hiz", 15.0)

                # İDEAL MESAFE (15-30m) - TAM EŞİTLE
                if 15.0 <= t_dist <= 30.0:
                    # KAMERA AÇISI İÇİN İDEAL - Rakiple TAM AYNI HIZ
                    cmd_speed = min(target_speed, SAFETY_MAX_SPEED)

                # ÇOK YAKIN (<15m) - ÇARPIŞMA RİSKİ!
                elif t_dist < 15.0:
                    if target_speed < 15.0:
                        # Yavaş hedef + çok yakın → ACİL FREN!
                        cmd_speed = max(6.0, target_speed * 0.6)
                    else:
                        # Hızlı hedef + çok yakın → Kontrollü yavaşla
                        cmd_speed = max(6.0, min(target_speed * 0.85, 20.0))

                # UZAK (>30m) - YAKLAŞ
                else:  # t_dist > 30.0
                    if target_speed < 15.0:
                        # Yavaş hedef + uzak → Dikkatli yaklaş
                        cmd_speed = min(target_speed + 3.0, 20.0)
                    else:
                        # Hızlı hedef + uzak → Hızlı yaklaş
                        cmd_speed = min(target_speed + 5.0, SAFETY_MAX_SPEED)

                mode_str = "KILIT | KAMERA AKTIF | ARKA HIZA"

            # Güvenlik sınırları (her durumda uygula)
            cmd_speed = max(SAFETY_MIN_SPEED, min(SAFETY_MAX_SPEED, cmd_speed))
            cmd_alt = max(SAFETY_MIN_ALT, min(SAFETY_MAX_ALT, cmd_alt))

            # UKF initialize olmadan komut gönderme
            if not is_init:
                print("⚠️ UKF initialize olmamış - KOMUT GÖNDERİLMİYOR!")
                time.sleep(0.1)
                continue

            # ─────────────────────────────────────────────────
            # 7. KOMUT G?NDER - ArduPlane GUIDED (reposition + speed)
            cmd_yaw_deg = cmd_yaw % 360.0
            look_ahead = 200.0  # Plane i?in daha uzun mesafe (200m)

            # Yeni hedef koordinatlar?n? hesapla
            target_lat = lat + (look_ahead * math.cos(math.radians(cmd_yaw_deg))) / 111320.0
            target_lon = lon + (look_ahead * math.sin(math.radians(cmd_yaw_deg))) / (111320.0 * math.cos(math.radians(lat)))

            # ArduPlane i?in h?z komutu (MAV_CMD_DO_CHANGE_SPEED)
            if t_now - last_speed_cmd_time > 0.5:
                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                    0,
                    1,              # param1: speed type (1 = groundspeed)
                    cmd_speed,      # param2: target speed (m/s)
                    -1,             # param3: throttle (keep current)
                    0, 0, 0, 0      # unused params
                )
                last_speed_cmd_time = t_now

            # ArduPlane i?in GUIDED pozisyon komutu
            send_plane_reposition(
                master,
                target_lat,
                target_lon,
                cmd_alt,
                yaw_deg=cmd_yaw_deg,
                speed_ms=cmd_speed,
                radius=0.0,
            )

            # 8. DURUM ÇIKTISI
            # ─────────────────────────────────────────────────
            target_pos_str = ""
            if target:
                tx = target.get("x", 0.0)
                ty = target.get("y", 0.0)
                tz = target.get("z", 0.0)
                target_pos_str = f" Tgt:[{tx:.0f},{ty:.0f},{tz:.0f}]"

            vel_str = f"V:[{vx:.1f},{vy:.1f},{vz:.1f}]"

            print(
                f"[IHA-{vehicle_cfg.vehicle_id}] {marker} Me:[{est_x:.0f},{est_y:.0f},{est_z:.0f}] {vel_str} "
                f"Spd:{my_speed:.1f} | Tgt:{t_id}{target_pos_str} Dst:{t_dist:.0f}m | {mode_str}",
                end="\r",
            )

    except KeyboardInterrupt:
        print("\nDurduruldu")
    except Exception as e:
        print(f"\nHATA: {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Sistem kapatılıyor...")
        master.close()
        peer.close()


if __name__ == "__main__":
    run_mavlink_node()
