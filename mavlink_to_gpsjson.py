# Bridge: MAVLink (SITL/QGC) -> gps.py JSON UDP (127.0.0.1:5799)
# gps.py'ya dokunmadan, MAVLink'ten okuduğunuzu onun beklediği formata çevirir.
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
        # scaled_imu: mg ve mrad/s, yaklaşık m/s^2 ve rad/s'e çevir
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
    return {
        "is_valid": True,
        "lat": lat,
        "lon": lon,
        "alt": alt,
        "speed_ms": speed_ms,
        "ground_speed": speed_ms,
        "hdop": getattr(gpos, "eph", 1.0) if hasattr(gpos, "eph") else 1.0,
    }


def main():
    import argparse

    ap = argparse.ArgumentParser(description="MAVLink -> gps.py JSON bridge")
    ap.add_argument(
        "--master",
        default="tcp:127.0.0.1:5770",
        help="MAVLink master (takipçi/kendi SITL, ornek: tcp:127.0.0.1:5770)",
    )
    ap.add_argument("--out-ip", default="127.0.0.1", help="gps.py dinleyen IP (varsayılan 127.0.0.1)")
    ap.add_argument("--out-port", type=int, default=5799, help="gps.py dinleyen port (varsayılan 5799)")
    args = ap.parse_args()

    m = _connect(args.master)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[bridge] Forwarding to {args.out_ip}:{args.out_port}")

    last_imu = None
    last_scaled_imu = None
    last_gpos = None
    last_vfr = None

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

        gps_block = _build_gps(last_gpos, last_vfr)
        imu_block = _build_imu(last_imu, last_scaled_imu)

        if gps_block is None:
            continue

        tm = {
            "time_s": now,
            "gps": gps_block,
            "imu": imu_block,
            "network_data": [],  # rival_tracker icin bos bir liste
        }

        payload = json.dumps(tm).encode("utf-8")
        sock.sendto(payload, (args.out_ip, args.out_port))


if __name__ == "__main__":
    main()
