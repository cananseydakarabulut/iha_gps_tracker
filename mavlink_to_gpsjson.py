# Bridge: MAVLink (SITL/QGC) -> gps.py JSON UDP (127.0.0.1:5799)
# gps.py'ya dokunmadan, MAVLink'ten okuduunuzu onun bekledii formata evirir.
import json
import math
import socket
import time
from typing import Optional

from pymavlink import mavutil


def _connect(master_str: str):
    print(f"[bridge] Connecting MAVLink: {master_str}")
    m = mavutil.mavlink_connection(master_str)
    m.wait_heartbeat()
    print(f"[bridge] Heartbeat ok sysid={m.target_system} compid={m.target_component}")
    return m


def _recv_matching(m: mavutil.mavlink_connection, types):
    try:
        return m.recv_match(type=types, blocking=False, timeout=0.02)
    except Exception:
        return None


def _build_imu(hr_imu, scaled_imu) -> dict:
    if hr_imu:
        return {
            "acc": [hr_imu.xacc, hr_imu.yacc, hr_imu.zacc],
            "gyr": [hr_imu.xgyro, hr_imu.ygyro, hr_imu.zgyro],
        }
    if scaled_imu:
        # scaled_imu: mg ve mrad/s, yaklak m/s^2 ve rad/s'e evir
        return {
            "acc": [scaled_imu.xacc / 1000.0 * 9.81, scaled_imu.yacc / 1000.0 * 9.81, scaled_imu.zacc / 1000.0 * 9.81],
            "gyr": [scaled_imu.xgyro / 1000.0, scaled_imu.ygyro / 1000.0, scaled_imu.zgyro / 1000.0],
        }
    return {"acc": [0.0, 0.0, 0.0], "gyr": [0.0, 0.0, 0.0]}


def _build_gps(gpos, vfr) -> Optional[dict]:
    if gpos is None:
        return None
    lat = gpos.lat / 1e7
    lon = gpos.lon / 1e7
    alt = gpos.alt / 1000.0  # mm -> m
    # vx, vy cm/s -> m/s
    vx = gpos.vx / 100.0
    vy = gpos.vy / 100.0
    vz = gpos.vz / 100.0
    speed_ms = math.sqrt(vx * vx + vy * vy + vz * vz)
    if vfr:
        speed_ms = max(speed_ms, getattr(vfr, "groundspeed", 0.0))

    # Heading (yönelim) hesapla
    heading = gpos.hdg / 100.0 if hasattr(gpos, "hdg") and gpos.hdg != 65535 else 0.0

    # Airspeed (hava hızı) - pitot tube sensöründen
    airspeed = getattr(vfr, "airspeed", 0.0) if vfr else 0.0

    return {
        "is_valid": True,
        "lat": lat,
        "lon": lon,
        "alt": alt,
        "speed_ms": speed_ms,
        "ground_speed": speed_ms,
        "heading": heading,
        "hdop": getattr(gpos, "eph", 1.0) if hasattr(gpos, "eph") else 1.0,
        "airspeed": airspeed,  # Hava hızı (m/s)
    }


def main():
    import argparse
    import os

    ap = argparse.ArgumentParser(description="MAVLink -> gps.py JSON bridge")
    ap.add_argument(
        "--master",
        default="tcp:127.0.0.1:5770",
        help="MAVLink master (takipi/kendi SITL, ornek: tcp:127.0.0.1:5770)",
    )
    ap.add_argument("--out-ip", default="127.0.0.1", help="gps.py dinleyen IP (varsaylan 127.0.0.1)")
    ap.add_argument("--out-port", type=int, default=None, help="gps.py dinleyen port (varsayilan: 5799 + VEHICLE_ID - 1)")
    ap.add_argument("--vehicle-id", type=int, default=None, help="IHA ID (VEHICLE_ID ortam değişkeninden okunur)")
    ap.add_argument("--peer-master", default=None, help="Rakip IHA MAVLink bağlantısı (ornek: tcp:127.0.0.1:5770)")
    ap.add_argument("--peer-team-id", type=int, default=None, help="Rakip takım numarası")
    args = ap.parse_args()

    # Vehicle ID belirle (argüman > ortam değişkeni > varsayılan)
    vehicle_id = args.vehicle_id if args.vehicle_id is not None else int(os.getenv("VEHICLE_ID", "1"))

    # Port belirle
    if args.out_port is None:
        args.out_port = 5799 + vehicle_id - 1  # IHA-1: 5799, IHA-2: 5800, vb.

    print(f"[bridge] IHA ID: {vehicle_id} -> UDP Port: {args.out_port}")

    m = _connect(args.master)

    # Rakip bağlantısı (opsiyonel)
    peer = None
    peer_team_id = args.peer_team_id
    if args.peer_master:
        print(f"[bridge] Rakip bağlantısı kuruluyor: {args.peer_master}")
        try:
            peer = mavutil.mavlink_connection(args.peer_master)
            peer.wait_heartbeat(timeout=5)
            print(f"[bridge] Rakip heartbeat ok (Team ID: {peer_team_id})")
        except Exception as e:
            print(f"[bridge] UYARI: Rakip bağlantısı başarısız: {e}")
            peer = None

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[bridge] Forwarding to {args.out_ip}:{args.out_port}")

    last_imu = None
    last_scaled_imu = None
    last_gpos = None
    last_vfr = None

    # Rakip telemetrisi
    peer_last_gpos = None
    peer_last_vfr = None

    while True:
        msg = _recv_matching(m, ["GLOBAL_POSITION_INT", "VFR_HUD", "HIGHRES_IMU", "SCALED_IMU2", "SCALED_IMU", "ATTITUDE"])
        now = time.time()

        if msg is None:
            continue

        mtype = msg.get_type()
        if mtype == "GLOBAL_POSITION_INT":
            last_gpos = msg
        elif mtype == "VFR_HUD":
            last_vfr = msg
        elif mtype == "HIGHRES_IMU":
            last_imu = msg
        elif mtype in ("SCALED_IMU", "SCALED_IMU2"):
            last_scaled_imu = msg

        # Rakip telemetrisini oku (non-blocking)
        if peer:
            peer_msg = _recv_matching(peer, ["GLOBAL_POSITION_INT", "VFR_HUD"])
            if peer_msg:
                peer_mtype = peer_msg.get_type()
                if peer_mtype == "GLOBAL_POSITION_INT":
                    peer_last_gpos = peer_msg
                elif peer_mtype == "VFR_HUD":
                    peer_last_vfr = peer_msg

        gps_block = _build_gps(last_gpos, last_vfr)
        imu_block = _build_imu(last_imu, last_scaled_imu)

        if gps_block is None:
            continue

        # Rakip telemetrisini network_data'ya ekle
        network_data = []
        if peer and peer_last_gpos and peer_team_id is not None:
            peer_gps = _build_gps(peer_last_gpos, peer_last_vfr)
            if peer_gps:
                network_data.append({
                    "takim_numarasi": peer_team_id,
                    "iha_enlem": peer_gps["lat"],
                    "iha_boylam": peer_gps["lon"],
                    "iha_irtifa": peer_gps["alt"],
                    "iha_hiz": peer_gps["speed_ms"],
                    "iha_yonelme": peer_gps["heading"],
                    "iha_dikilme": 0.0,  # pitch bilgisi yok
                    "iha_yatis": 0.0,    # roll bilgisi yok
                })

        tm = {
            "time_s": now,
            "gps": gps_block,
            "imu": imu_block,
            "network_data": network_data,
        }

        payload = json.dumps(tm).encode("utf-8")
        sock.sendto(payload, (args.out_ip, args.out_port))


if __name__ == "__main__":
    main()
