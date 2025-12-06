# my_python_module.py
# MAVProxy modulu: coklu arac takibi, hiz/ivme tahmini ve takip (pos/vel/guide setpoint)

import math
import numpy as np
import time
from pymavlink import mavutil
from geo_utils import llh_to_enu, enu_to_llh
from guidance import GpsPursuitController
from ukf import KF3D_UKF
from config import KFConfig

# Varsayilan: sysid 1 = kendi arac, sysid 2 = hedef/rakip arac
master = None
vehicles = {}  # sysid -> {'last_seen': ts, 'heartbeat': ts, 'pos': {...}, 'est': {...}}
target_system = 2
message_requests = {}  # (sysid, msgid) -> ts
last_status_print = 0.0
last_follow_send = 0.0
last_follow_log = 0.0
follow_enabled = True
follow_mode = "pos"  # pos | vel | guide
ref_origin = None  # {'lat','lon','alt'}
guide = GpsPursuitController()
ukf_cfg = KFConfig()

REQUEST_RATES_HZ = {
    mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 5,  # 5 Hz pozisyon
}

# Hiz/ivme tahmini icin basit filtre parametreleri
VEL_ALPHA = 0.35  # olcum agirligi (daha yuksek -> daha hizli tepki)
ACC_DAMP = 0.5    # ivme yumusatma katsayisi
PRED_HORIZON_S = 1.0  # hedefin ileriye projeksiyon zamani (s)
MAX_VEL_CMD = 8.0     # hiz setpoint siniri (m/s)
VISUAL_DIST_THRESH_M = 35.0  # gorsel moda gecis mesafe esigi
VISUAL_SPEED_THRESH = 6.0    # gorsel moda gecis hedef hiz esigi

visual_active = False
last_visual_log = 0.0


def init(mpstate):
    """MAVProxy modulu yuklenirken cagirilir."""
    global master
    master = mpstate.master()

    mpstate.command_map["targetsys"] = (
        cmd_targetsys,
        "targetsys <id>   # hedef sysid ayarla/gor",
    )
    mpstate.command_map["vehlist"] = (
        cmd_vehlist,
        "vehlist          # bilinen araclari yazdir",
    )
    mpstate.command_map["follow"] = (
        cmd_follow,
        "follow [on|off]  # hedef sysid'i takip ac/kapat",
    )
    mpstate.command_map["followmode"] = (
        cmd_followmode,
        "followmode pos|vel|guide  # pozisyon/hiz veya guidance setpoint",
    )

    print(">>> Python modulu baslatildi (coklu arac hazir)")


def mavlink_packet(mpstate, packet):
    """Her gelen MAVLink paketi icin cagirilir."""
    global ref_origin
    if packet is None:
        return

    sysid = getattr(packet, "get_srcSystem", lambda: None)()
    if sysid is None:
        return

    mtype = packet.get_type()
    info = vehicles.setdefault(
        sysid, {"last_seen": 0.0, "heartbeat": 0.0, "pos": None, "ukf": None}
    )
    now = time.time()
    info["last_seen"] = now

    if mtype == "HEARTBEAT":
        info["heartbeat"] = now

    elif mtype == "GLOBAL_POSITION_INT":
        vx = getattr(packet, "vx", 0) / 100.0  # m/s
        vy = getattr(packet, "vy", 0) / 100.0
        vz = getattr(packet, "vz", 0) / 100.0
        info["pos"] = {
            "lat": packet.lat / 1e7,
            "lon": packet.lon / 1e7,
            "alt": packet.alt / 1000.0,
            "ground_speed": (packet.vx**2 + packet.vy**2) ** 0.5 / 100.0,
            "vx": vx,
            "vy": vy,
            "vz": vz,
        }
        update_motion_estimates(info, now)
        if sysid == master.target_system and ref_origin is None:
            ref_origin = {"lat": info["pos"]["lat"], "lon": info["pos"]["lon"], "alt": info["pos"]["alt"]}
        update_ukf(info, now)


def cmd_targetsys(args):
    """MAVProxy komutu: hedef sysid goster/ayarla."""
    global target_system

    if not args:
        print(f"[mod] hedef sysid: {target_system}")
        return

    try:
        target_system = int(args[0])
        print(f"[mod] hedef sysid {target_system} olarak ayarlandi.")
    except ValueError:
        print("[mod] targetsys <id> seklinde kullanin.")


def cmd_follow(args):
    """MAVProxy komutu: takip ac/kapat."""
    global follow_enabled
    if not args:
        print(f"[mod] takip durumu: {'acik' if follow_enabled else 'kapali'}")
        return
    val = args[0].lower()
    if val in ("on", "1", "ac", "acik"):
        follow_enabled = True
        print("[mod] takip acildi.")
    elif val in ("off", "0", "kapa", "kapali"):
        follow_enabled = False
        print("[mod] takip kapandi.")
    else:
        print("[mod] follow on|off seklinde kullanin.")


def cmd_followmode(args):
    """MAVProxy komutu: takip modu (pos/vel/guide)."""
    global follow_mode
    if not args:
        print(f"[mod] takip modu: {follow_mode}")
        return
    val = args[0].lower()
    if val in ("pos", "vel", "guide"):
        follow_mode = val
        print(f"[mod] takip modu {follow_mode} olarak ayarlandi.")
    else:
        print("[mod] followmode pos|vel|guide seklinde kullanin.")


def cmd_vehlist(args):
    """MAVProxy komutu: bilinen araclari ozetle."""
    if not vehicles:
        print("[mod] henuz arac gorulmedi.")
        return

    now = time.time()
    for sysid, info in sorted(vehicles.items()):
        marker = "*" if sysid == target_system else " "
        pos = info.get("pos")
        if pos:
            vel = info.get("est", {}).get("vel_ms")
            vel_txt = ""
            if vel:
                speed_mag = (vel[0] ** 2 + vel[1] ** 2 + vel[2] ** 2) ** 0.5
                vel_txt = f", filt_v {speed_mag:.1f} m/s"
            pos_txt = (
                f"lat {pos['lat']:.5f}, lon {pos['lon']:.5f}, "
                f"alt {pos['alt']:.1f} m, gs {pos['ground_speed']:.1f} m/s{vel_txt}"
            )
        else:
            pos_txt = "pos yok"

        since = now - info.get("last_seen", now)
        print(f"[mod]{marker} sysid {sysid}: {pos_txt} | {since:4.1f}s once")


def ensure_msg_interval(sysid, msgid, rate_hz):
    """Belirli araca mesaj frekansi iste."""
    if master is None:
        return

    key = (sysid, msgid)
    now = time.time()
    if (now - message_requests.get(key, 0.0)) < 5.0:
        return

    usec = int(1_000_000 / rate_hz)
    try:
        master.mav.command_long_send(
            sysid,
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msgid,
            usec,
            0,
            0,
            0,
            0,
            0,
        )
        message_requests[key] = now
    except Exception as exc:
        message_requests[key] = now
        print(f"[mod] msg interval hatasi (sysid {sysid}): {exc}")


def update_motion_estimates(info, now):
    """Basit alpha-beta benzeri hiz/ivme tahmini."""
    pos = info.get("pos")
    if not pos:
        return

    est = info.setdefault("est", {})
    last_t = est.get("t")
    prev_v = est.get("vel_ms", (0.0, 0.0, 0.0))
    prev_a = est.get("acc_ms2", (0.0, 0.0, 0.0))

    meas_v = (pos["vx"], pos["vy"], pos["vz"])

    if last_t is None:
        est["vel_ms"] = meas_v
        est["acc_ms2"] = (0.0, 0.0, 0.0)
        est["t"] = now
        return

    dt = max(1e-3, now - last_t)
    vel_filt = tuple(VEL_ALPHA * m + (1 - VEL_ALPHA) * p for m, p in zip(meas_v, prev_v))
    acc_raw = tuple((v - p) / dt for v, p in zip(vel_filt, prev_v))
    acc_filt = tuple(ACC_DAMP * a + (1 - ACC_DAMP) * pa for a, pa in zip(acc_raw, prev_a))

    est["vel_ms"] = vel_filt
    est["acc_ms2"] = acc_filt
    est["t"] = now


def update_ukf(info, now):
    """UKF tabanli takip icin hedefi guncelle."""
    if ref_origin is None:
        return
    pos = info.get("pos")
    if not pos:
        return

    if info.get("ukf") is None:
        kf = KF3D_UKF(ukf_cfg)
        pos_enu = llh_to_enu(
            pos["lat"], pos["lon"], pos["alt"],
            ref_origin["lat"], ref_origin["lon"], ref_origin["alt"]
        )
        kf.initialize_from_pos(pos_enu)
        info["ukf"] = {"kf": kf, "t": now}
        return

    kf = info["ukf"]["kf"]
    last_t = info["ukf"].get("t", now)
    dt = max(1e-3, min(0.1, now - last_t))  # IMU yoksa dt'yi kisa tut
    info["ukf"]["t"] = now

    # IMU MAVLink'ten alinmiyor; sifir girdi ile predict
    imu_zero = np.zeros(6)
    kf.predict(dt, imu_zero)

    pos_enu = llh_to_enu(
        pos["lat"], pos["lon"], pos["alt"],
        ref_origin["lat"], ref_origin["lon"], ref_origin["alt"]
    )
    speed_mag = pos.get("ground_speed", 0.0)
    z_meas = np.concatenate([pos_enu, np.zeros(6), [speed_mag]])
    kf.update(z_meas, dt, hdop_simulated=1.0)


def predict_position(info, horizon=PRED_HORIZON_S):
    """Basit ileri projeksiyon (lat/lon derece, alt m)."""
    pos = info.get("pos")
    est = info.get("est", {})
    if not pos:
        return None

    # UKF'den tahmin varsa onu kullan
    ukf_entry = info.get("ukf")
    if ukf_entry and ukf_entry.get("kf") and ref_origin is not None:
        kf = ukf_entry["kf"]
        state = kf.x.flatten()
        x_e, y_e, z_e = state[0], state[1], state[2]
        vx_e, vy_e, vz_e = state[7], state[8], state[9]
        lat_u, lon_u, alt_u = enu_to_llh(
            x_e, y_e, z_e,
            ref_origin["lat"], ref_origin["lon"], ref_origin["alt"]
        )
        return {"lat": lat_u, "lon": lon_u, "alt": alt_u, "vx": vx_e, "vy": vy_e, "vz": vz_e}

    lat = pos["lat"]
    lon = pos["lon"]
    alt = pos["alt"]

    vel = est.get("vel_ms")
    acc = est.get("acc_ms2")
    if not vel or not acc:
        return {"lat": lat, "lon": lon, "alt": alt}

    vx, vy, vz = vel
    ax, ay, az = acc
    dt = horizon

    dx = vx * dt + 0.5 * ax * dt * dt
    dy = vy * dt + 0.5 * ay * dt * dt
    dz = vz * dt + 0.5 * az * dt * dt

    meters_per_deg_lat = 111_111.0
    meters_per_deg_lon = max(1e-6, meters_per_deg_lat * abs(math.cos(math.radians(lat))))

    lat_p = lat + dx / meters_per_deg_lat
    lon_p = lon + dy / meters_per_deg_lon
    alt_p = alt + dz

    return {"lat": lat_p, "lon": lon_p, "alt": alt_p, "vx": vx, "vy": vy, "vz": vz}


def dynamic_horizon():
    """Gorsel moda gecince daha kisa ileri projeksiyon."""
    return 0.3 if visual_active else PRED_HORIZON_S


def send_follow_setpoint():
    """Hedef sysid'i konum veya hiz setpoint ile takip et."""
    global last_follow_send, last_follow_log
    if master is None or not follow_enabled:
        return

    target = vehicles.get(target_system, {})
    pos = target.get("pos")
    if not pos:
        return

    now = time.time()
    if (now - last_follow_send) < 1.0:
        return

    predicted = predict_position(target, dynamic_horizon())
    pos_to_use = predicted or pos

    try:
        if follow_mode == "guide":
            send_guidance_setpoint(pos_to_use)
        else:
            send_basic_setpoint(pos_to_use)
        last_follow_send = now
        if (now - last_follow_log) > 3.0:
            print(
                f"[mod] takip setpoint gonderildi ({follow_mode}) -> "
                f"lat {pos_to_use['lat']:.6f}, lon {pos_to_use['lon']:.6f}"
            )
            last_follow_log = now
    except Exception as exc:
        last_follow_send = now
        print(f"[mod] takip setpoint hatasi: {exc}")


def send_basic_setpoint(pos_to_use):
    """Pos/vel modlari icin setpoint gonder."""
    lat_int = int(pos_to_use["lat"] * 1e7)
    lon_int = int(pos_to_use["lon"] * 1e7)
    alt_m = float(pos_to_use["alt"])

    mask_pos = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )

    if follow_mode == "pos":
        master.mav.set_position_target_global_int_send(
            0,
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            mask_pos,
            lat_int,
            lon_int,
            alt_m,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        return

    # hiz setpoint
    vx = pos_to_use.get("vx", 0.0)
    vy = pos_to_use.get("vy", 0.0)
    vz = pos_to_use.get("vz", 0.0)
    speed_mag = (vx * vx + vy * vy + vz * vz) ** 0.5
    if speed_mag > MAX_VEL_CMD:
        scale = MAX_VEL_CMD / speed_mag
        vx, vy, vz = vx * scale, vy * scale, vz * scale

    mask_vel = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )
    master.mav.set_position_target_global_int_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        mask_vel,
        0,
        0,
        0,
        vx,
        vy,
        vz,
        0,
        0,
        0,
        0,
        0,
    )


def send_guidance_setpoint(pos_to_use):
    """Guidance kurallarina gore hiz setpoint gonder."""
    if ref_origin is None:
        return

    my = vehicles.get(master.target_system, {})
    my_pos = my.get("pos")
    if not my_pos:
        return

    # ENU hesapla
    ox, oy, oz = llh_to_enu(
        my_pos["lat"],
        my_pos["lon"],
        my_pos["alt"],
        ref_origin["lat"],
        ref_origin["lon"],
        ref_origin["alt"],
    )
    tx, ty, tz = llh_to_enu(
        pos_to_use["lat"],
        pos_to_use["lon"],
        pos_to_use["alt"],
        ref_origin["lat"],
        ref_origin["lon"],
        ref_origin["alt"],
    )

    target_vel = pos_to_use.get("vx"), pos_to_use.get("vy"), pos_to_use.get("vz")
    target_speed = 0.0
    if all(v is not None for v in target_vel):
        target_speed = (target_vel[0] ** 2 + target_vel[1] ** 2 + target_vel[2] ** 2) ** 0.5

    yaw_deg = math.degrees(math.atan2(pos_to_use.get("vy", 0.0), pos_to_use.get("vx", 0.0))) % 360.0
    target = {
        "x": tx,
        "y": ty,
        "z": tz,
        "hiz": target_speed,
        "yaw_deg": yaw_deg,
    }

    my_vel = my.get("est", {}).get("vel_ms")
    my_speed = (my_vel[0] ** 2 + my_vel[1] ** 2 + my_vel[2] ** 2) ** 0.5 if my_vel else my_pos.get("ground_speed", 0.0)

    cmd = guide.compute_command((ox, oy, oz), my_speed, target)

    yaw_rad = math.radians(cmd.desired_yaw_deg)
    vx_n = cmd.desired_speed_ms * math.cos(yaw_rad)  # north
    vy_e = cmd.desired_speed_ms * math.sin(yaw_rad)  # east

    alt_err = cmd.desired_altitude_m - my_pos["alt"]
    vz_ned = -max(-2.0, min(2.0, alt_err * 0.5))  # up -> negative in NED

    mask_vel = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )

    master.mav.set_position_target_global_int_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        mask_vel,
        0,
        0,
        0,
        vx_n,
        vy_e,
        vz_ned,
        0,
        0,
        0,
        0,
        0,
    )


def idle_task(mpstate):
    """MAVProxy dongusu sirasinda hafif isler."""
    global last_status_print
    now = time.time()

    for sysid in list(vehicles.keys()):
        for msgid, rate in REQUEST_RATES_HZ.items():
            ensure_msg_interval(sysid, msgid, rate)

    send_follow_setpoint()
    check_visual_lock(now)

    if (now - last_status_print) > 3.0:
        cmd_vehlist([])
        last_status_print = now


def check_visual_lock(now):
    """GPS'ten gorsel moda gecis sinyali (bilgilendirici)."""
    global visual_active, last_visual_log
    if ref_origin is None:
        return

    my = vehicles.get(master.target_system, {})
    tgt = vehicles.get(target_system, {})
    my_pos = my.get("pos")
    tgt_pos = tgt.get("pos")
    if not my_pos or not tgt_pos:
        return

    ox, oy, oz = llh_to_enu(
        my_pos["lat"], my_pos["lon"], my_pos["alt"],
        ref_origin["lat"], ref_origin["lon"], ref_origin["alt"]
    )
    tx, ty, tz = llh_to_enu(
        tgt_pos["lat"], tgt_pos["lon"], tgt_pos["alt"],
        ref_origin["lat"], ref_origin["lon"], ref_origin["alt"]
    )
    dist = math.sqrt((tx - ox) ** 2 + (ty - oy) ** 2 + (tz - oz) ** 2)

    speed = tgt_pos.get("ground_speed", 0.0)
    should_visual = (dist < VISUAL_DIST_THRESH_M) and (speed < VISUAL_SPEED_THRESH)

    if should_visual and not visual_active:
        visual_active = True
        if (now - last_visual_log) > 1.0:
            print(f"[mod] gorsel moda gecildi (dist={dist:.1f} m, gs={speed:.1f} m/s)")
            last_visual_log = now
    elif (not should_visual) and visual_active:
        visual_active = False
        if (now - last_visual_log) > 1.0:
            print(f"[mod] gorsel moddan cikildi (dist={dist:.1f} m, gs={speed:.1f} m/s)")
            last_visual_log = now
