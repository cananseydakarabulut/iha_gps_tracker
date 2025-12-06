import socket
import json
import sys
import math
import time
import csv
import numpy as np
from datetime import datetime

# Kendi modüllerin
from config import KFConfig, SimCfg
from ukf import KF3D_UKF
from geo_utils import llh_to_enu, parse_if_dms
from rival_tracker import RivalTracker, MY_TEAM_ID
from guidance import GpsPursuitController, SAHA_YARICAPI
from quaternion_utils import q_to_euler_bounded

# -----------------------------
# PORT AYARLARI
# -----------------------------
LISTEN_IP = "127.0.0.1"
LISTEN_PORT = 5799
CONTROL_PORT = 5771

BUFFER_SIZE = 8192
CSV_FILENAME = "telemetry_log.csv"
LOCK_THRESHOLD_M = 20.0
LOCK_ANGLE_DEG = 20.0
LOCK_DURATION = 5.0
SAFETY_MIN_SPEED = 15.0
SAFETY_MAX_SPEED = 60.0
SAFETY_MIN_ALT = 20.0
SAFETY_MAX_ALT = 150.0
SAHA_DISI_TIMEOUT_S = 40.0


def run_receiver_node():
    print("GPS Node Baslatiliyor...")

    sim_cfg = SimCfg()
    kf_cfg = KFConfig()

    # Referansı ilk geçerli GPS'ten alacağız
    ref_lat = None
    ref_lon = None
    ref_h = None

    kf = KF3D_UKF(kf_cfg)
    rival_tracker = None
    guidance = GpsPursuitController()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((LISTEN_IP, LISTEN_PORT))
        print(f"Bekleniyor: {LISTEN_IP}:{LISTEN_PORT}")
    except Exception as e:
        sys.exit(f"Soket Hatasi: {e}")

    control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    last_time = None
    is_init = False

    my_speed = 0.0
    lock_timer_start = None
    locking_target_id = None
    outside_timer_start = None

    # ---------------------------
    # CSV LOG
    # ---------------------------
    with open(CSV_FILENAME, "w", newline="", encoding="utf-8") as f:

        writer = csv.DictWriter(
            f,
            fieldnames=[
                "time",
                "my_x", "my_y", "my_z",
                "my_speed", "my_roll", "my_pitch", "my_yaw",
                "target_id",
                "target_x", "target_y", "target_z",
                "target_speed", "target_gecikme_ms",
                "target_dist",
                "kilitlenme",
                "cmd_yaw", "cmd_speed", "cmd_alt",
                "mode_log"
            ],
        )
        writer.writeheader()

        # =========================================================
        #                     ANA LOOP
        # =========================================================
        while True:
            try:
                data, _ = sock.recvfrom(BUFFER_SIZE)
                tm = json.loads(data.decode("utf-8"))

                gps_valid = tm.get("gps", {}).get("is_valid", False)

                t_now = tm["time_s"]
                if last_time is None:
                    last_time = t_now
                dt = t_now - last_time
                if dt <= 0:
                    continue
                last_time = t_now

                # --------------------------------------
                # UKF INIT
                # --------------------------------------
                if not is_init:
                    if tm["gps"]["is_valid"]:
                        if ref_lat is None:
                            ref_lat = tm["gps"]["lat"]
                            ref_lon = tm["gps"]["lon"]
                            ref_h = tm["gps"]["alt"]
                            rival_tracker = RivalTracker(ref_lat, ref_lon, ref_h)

                        pos0 = llh_to_enu(
                            tm["gps"]["lat"], tm["gps"]["lon"], tm["gps"]["alt"],
                            ref_lat, ref_lon, ref_h
                        )
                        
                        kf.initialize_from_pos(pos0)
                        kf.P[6, 6] = 1000.0
                        kf.P[7, 7] = 1000.0
                        kf.P[8, 8] = 1000.0
                        
                        is_init = True
                        print("UKF Initialized")
                    continue

                # ============================================================
                # ⭐ IMU → BODY FRAME DÖNÜŞÜMÜ
                # ============================================================
                acc_body = np.array([
                    tm["imu"]["acc"][1],
                    tm["imu"]["acc"][0],
                    tm["imu"]["acc"][2]
                ])

                gyr_body = np.array([
                    tm["imu"]["gyr"][1],
                    tm["imu"]["gyr"][0],
                    tm["imu"]["gyr"][2]
                ])

                imu_vec = np.concatenate((acc_body, gyr_body), axis=0)

                # ------------------
                # UKF PREDICT
                # ------------------
                kf.predict(dt, imu_vec)

                # ============================================================
                # ⭐ GPS → ENU
                # ============================================================
                if tm["gps"]["is_valid"] and ref_lat is not None:

                    raw_pos = llh_to_enu(
                        tm["gps"]["lat"], tm["gps"]["lon"], tm["gps"]["alt"],
                        ref_lat, ref_lon, ref_h
                    )
                    
                    # Eksen düzeltme YOK (Orijinal halin)
                    ukf_input_pos = raw_pos 

                    # GPS hızını ölçüm vektörüne ekle (UKF 10 eleman bekliyor)
                    gps_speed = float(tm["gps"].get("speed_ms", tm["gps"].get("ground_speed", my_speed)))

                    z_full = np.concatenate(
                        (
                            ukf_input_pos,      # 3
                            acc_body,           # 3
                            [20.0, 0.0, 45.0],  # manyetometre yer tutucu
                            [gps_speed],        # yer hızı büyüklüğü
                        )
                    )

                    kf.update(z_full, dt, tm["gps"]["hdop"])

                # ============================================================
                # UKF STATE OKU
                # ============================================================
                est_x = kf.x[0, 0]
                est_y = kf.x[1, 0]
                est_z = kf.x[2, 0]

                # 3D HIZ HESABI
                vx = kf.x[7, 0]
                vy = kf.x[8, 0]
                vz = kf.x[9, 0]
                
                roll_d, pitch_d, yaw_d = q_to_euler_bounded(kf.x[3:7, 0])

                # Hız sensörü varsa UKF update ölçümüne büyüklük olarak ekleniyor; burada ayrıca karıştırmıyoruz
                my_speed = math.sqrt(vx**2 + vy**2 + vz**2)

                # RIVAL
                if rival_tracker and "network_data" in tm:
                    rival_tracker.update_from_server_response(tm["network_data"])

                target = rival_tracker.get_closest_rival(est_x, est_y, est_z, yaw_d) if rival_tracker else None

                kilit = 0
                visual_mode = False
                cmd = {}
                t_id = 0
                t_dist = 0.0
                mode_str = "BEKLEME"

                # GUIDANCE
                if target:
                    g_cmd = guidance.compute_command(
                        (est_x, est_y, est_z),
                        my_speed,
                        target
                    )

                    t_id = target["takim_numarasi"]
                    t_dist = g_cmd.distance_to_target_m
                    mode_str = g_cmd.mode

                    # --------------------------
                    # KİLİTLENME LOGİĞİ
                    # --------------------------
                    yaw_err = abs(g_cmd.desired_yaw_deg - yaw_d)
                    if yaw_err > 180:
                        yaw_err = 360 - yaw_err

                    if t_dist < LOCK_THRESHOLD_M and yaw_err < LOCK_ANGLE_DEG:

                        if lock_timer_start is None or locking_target_id != t_id:
                            lock_timer_start = time.time()
                            locking_target_id = t_id
                            print(f"⏳ Kilitlenme Başladı: Hedef {t_id}...")

                        elapsed = time.time() - lock_timer_start
                        if elapsed >= LOCK_DURATION:
                            if rival_tracker.report_lock_event(t_id):
                                kilit = 1
                                print(f"🎯 KİLİT TAMAMLANDI! Takım {t_id} VURULDU!")
                                visual_mode = True
                                mode_str = "VISUAL FOLLOW | KILIT"
                                lock_timer_start = None
                                locking_target_id = None
                        else:
                            mode_str += f" | 🔒 {LOCK_DURATION - elapsed:.1f}s"

                    else:
                        lock_timer_start = None
                        locking_target_id = None

                    cmd = {
                        "type": "control",
                        "yaw": g_cmd.desired_yaw_deg,
                        "speed": g_cmd.desired_speed_ms,
                        "alt": g_cmd.desired_altitude_m,
                    }

                else:
                    cmd = {
                        "type": "control",
                        "yaw": yaw_d,
                        "speed": 20.0,
                        "alt": 100.0,
                    }
                    mode_str = "HEDEF YOK"
                    lock_timer_start = None

                # Kilit sonrası kontrollü yavaşlama
                if (kilit or visual_mode) and cmd:
                    slow_speed = getattr(guidance, "full_brake", 6.0)
                    cmd["speed"] = min(cmd.get("speed", slow_speed), slow_speed)
                    cmd["yaw"] = (cmd.get("yaw", yaw_d) + 90.0) % 360.0
                    cmd["alt"] = max(cmd.get("alt", 0), 50.0)
                    if "KILIT FREN" not in mode_str:
                        mode_str += " | KILIT FREN"

                # ============================================================
                # ============================================================
                # SAHA DISI KONTROL / RTB
                # ============================================================
                # GPS gecerli degilse yanlis RTB tetiklemesin diye saha kontrolunu atla
                if gps_valid:
                    ARENA_RADIUS = SAHA_YARICAPI + 100.0 
                    dist_me = math.sqrt(est_x**2 + est_y**2)

                    if dist_me > ARENA_RADIUS:
                        # Merkeze donuk aciyi bul
                        center_yaw = math.degrees(math.atan2(-est_y, -est_x)) % 360
                        outside_timer_start = outside_timer_start or time.time()
                        outside_elapsed = time.time() - outside_timer_start
                        cmd["yaw"] = center_yaw
                        if outside_elapsed > SAHA_DISI_TIMEOUT_S:
                            cmd["speed"] = max(cmd.get("speed", 0), 28.0)
                            cmd["alt"] = max(cmd.get("alt", 0), 60.0)
                            mode_str = "RTB/LOITER"
                            cmd["action"] = "rtl"
                        elif outside_elapsed > 30.0:
                            cmd["speed"] = max(cmd.get("speed", 0), 25.0)
                            cmd["alt"] = max(cmd.get("alt", 0), 50.0)
                            mode_str = "SAHA DISI (ACIL)"
                            cmd["action"] = cmd.get("action") or "loiter"
                        else:
                            cmd["speed"] = max(cmd.get("speed", 0), 22.0)
                            cmd["alt"] = max(cmd.get("alt", 0), 40.0)
                            mode_str = "SAHA DISI"
                    else:
                        outside_timer_start = None
                # Platform hız/irtifa sınırlarını uygula
                if cmd:
                    cmd["speed"] = max(SAFETY_MIN_SPEED, min(SAFETY_MAX_SPEED, cmd.get("speed", SAFETY_MIN_SPEED)))
                    cmd["alt"] = max(SAFETY_MIN_ALT, min(SAFETY_MAX_ALT, cmd.get("alt", SAFETY_MIN_ALT)))

                # KOMUT GÖNDER
                control_sock.sendto(
                    json.dumps(cmd).encode(),
                    (LISTEN_IP, CONTROL_PORT)
                )

                # CSV LOG
                if target:
                    tx, ty, tz = target["x"], target["y"], target["z"]
                    tspeed = target["hiz"]
                    tdelay = target["gecikme_ms"]
                else:
                    tx = ty = tz = tspeed = tdelay = 0.0

                writer.writerow({
                    "time": f"{t_now:.2f}",
                    "my_x": f"{est_x:.2f}",
                    "my_y": f"{est_y:.2f}",
                    "my_z": f"{est_z:.2f}",
                    "my_speed": f"{my_speed:.2f}",
                    "my_roll": f"{roll_d:.2f}",
                    "my_pitch": f"{pitch_d:.2f}",
                    "my_yaw": f"{yaw_d:.2f}",
                    "target_id": t_id,
                    "target_x": f"{tx:.2f}",
                    "target_y": f"{ty:.2f}",
                    "target_z": f"{tz:.2f}",
                    "target_speed": f"{tspeed:.2f}",
                    "target_gecikme_ms": f"{tdelay:.2f}",
                    "target_dist": f"{t_dist:.2f}",
                    "kilitlenme": kilit,
                    "cmd_yaw": f"{cmd.get('yaw', 0):.2f}",
                    "cmd_speed": f"{cmd.get('speed', 0):.2f}",
                    "cmd_alt": f"{cmd.get('alt', 0):.2f}",
                    "mode_log": mode_str
                })

                print(
                    f"Me:[{est_x:.0f},{est_y:.0f}] Spd:{my_speed:.1f} | "
                    f"Tgt:{t_id} Dist:{t_dist:.0f}m | {mode_str}",
                    end="\r"
                )

            except KeyboardInterrupt:
                break

            except Exception as e:
                print("\nHATA:", e)
                continue

    sock.close()
    control_sock.close()


if __name__ == "__main__":
    run_receiver_node()
