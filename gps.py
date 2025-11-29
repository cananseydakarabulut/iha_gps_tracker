import socket
import json
import sys
import math
import time
import csv
import numpy as np
from datetime import datetime

from config import KFConfig, SimCfg
from ukf import KF3D_UKF
from geo_utils import llh_to_enu, parse_if_dms
from rival_tracker import RivalTracker, MY_TEAM_ID
from guidance import GpsPursuitController
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


def run_receiver_node():
    print("GPS Node Baslatiliyor...")

    sim_cfg = SimCfg()
    kf_cfg = KFConfig()

    ref_lat = parse_if_dms(sim_cfg.lat0)
    ref_lon = parse_if_dms(sim_cfg.lon0)
    ref_h = sim_cfg.h0

    kf = KF3D_UKF(kf_cfg)
    rival_tracker = RivalTracker(ref_lat, ref_lon, ref_h)
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

    lock_timer_start = None
    locking_target_id = None

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
                        pos0 = llh_to_enu(
                            tm["gps"]["lat"], tm["gps"]["lon"], tm["gps"]["alt"],
                            ref_lat, ref_lon, ref_h
                        )
                        
                        # 1. UKF'i başlat
                        kf.initialize_from_pos(pos0)

                        # 2. HIZ BELİRSİZLİĞİNİ YÜKSELT (Hız anında otursun)
                        kf.P[7, 7] = 1000.0
                        kf.P[8, 8] = 1000.0
                        kf.P[9, 9] = 1000.0
                        
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
                if tm["gps"]["is_valid"]:

                    raw_pos = llh_to_enu(
                        tm["gps"]["lat"], tm["gps"]["lon"], tm["gps"]["alt"],
                        ref_lat, ref_lon, ref_h
                    )
                    
                    # Eksen düzeltme YOK (Orijinal halin)
                    ukf_input_pos = raw_pos 

                    z_full = np.concatenate(
                        (
                            ukf_input_pos,
                            acc_body,
                            [20.0, 0.0, 45.0]
                        )
                    )

                    kf.update(z_full, dt, tm["gps"]["hdop"])

                # ============================================================
                # UKF STATE OKU
                # ============================================================
                est_x = kf.x[0, 0]
                est_y = kf.x[1, 0]
                est_z = kf.x[2, 0]

                # 🛑 TEK DEĞİŞİKLİK BURASI: 3D HIZ HESABI
                # (Eskisi sadece yatay hıza bakıyordu, bu hepsine bakıyor)
                vx = kf.x[7, 0]
                vy = kf.x[8, 0]
                vz = kf.x[9, 0]
                my_speed = math.sqrt(vx**2 + vy**2 + vz**2)
                
                roll_d, pitch_d, yaw_d = q_to_euler_bounded(kf.x[3:7, 0])

                # RIVAL
                if "network_data" in tm:
                    rival_tracker.update_from_server_response(tm["network_data"])

                target = rival_tracker.get_closest_rival(est_x, est_y, est_z)

                kilit = 0
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

                # ============================================================
                # SAHA DIŞI KONTROL
                # ============================================================
                ARENA_RADIUS = 200.0
                dist_me = math.sqrt(est_x**2 + est_y**2)

                if dist_me > ARENA_RADIUS:
                    center_yaw = math.degrees(math.atan2(-est_y, -est_x)) % 360
                    cmd["yaw"] = center_yaw
                    cmd["speed"] = 22.0
                    mode_str = "🚧 SAHA DIŞI"
                    lock_timer_start = None

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