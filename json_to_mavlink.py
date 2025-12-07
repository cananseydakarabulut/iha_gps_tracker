# Bridge: gps.py JSON komutlar (UDP 5771) -> MAVLink velocity/alt setpoint
# gps.py'ya dokunmadan ArduPilot'a komut iletmek iin kullanlr.
import json
import math
import socket
import time
from typing import Optional

from pymavlink import mavutil


def connect_master(master_str: str):
    print(f"[json->mav] Connecting MAVLink master: {master_str}")
    m = mavutil.mavlink_connection(master_str)
    m.wait_heartbeat()
    print(f"[json->mav] Heartbeat ok sysid={m.target_system} compid={m.target_component}")
    return m


def recv_json(sock) -> Optional[dict]:
    try:
        data, _ = sock.recvfrom(4096)
        return json.loads(data.decode("utf-8"))
    except Exception:
        return None


def main():
    import argparse
    import os

    ap = argparse.ArgumentParser(description="gps.py JSON -> MAVLink bridge")
    ap.add_argument("--listen-ip", default="127.0.0.1", help="gps.py komut IP (vars: 127.0.0.1)")
    ap.add_argument("--listen-port", type=int, default=None, help="gps.py komut port (varsayilan: 5771 + VEHICLE_ID - 1)")
    ap.add_argument("--vehicle-id", type=int, default=None, help="IHA ID (VEHICLE_ID ortam değişkeninden okunur)")
    ap.add_argument(
        "--master",
        default="tcp:127.0.0.1:5770",
        help="MAVLink master (takipi SITL, rn: tcp:127.0.0.1:5770)",
    )
    args = ap.parse_args()

    # Vehicle ID belirle (argüman > ortam değişkeni > varsayılan)
    vehicle_id = args.vehicle_id if args.vehicle_id is not None else int(os.getenv("VEHICLE_ID", "1"))

    # Port belirle
    if args.listen_port is None:
        args.listen_port = 5771 + vehicle_id - 1  # IHA-1: 5771, IHA-2: 5772, vb.

    print(f"[json->mav] IHA ID: {vehicle_id} -> UDP Port: {args.listen_port}")

    m = connect_master(args.master)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.listen_ip, args.listen_port))
    print(f"[json->mav] Listening {args.listen_ip}:{args.listen_port} for gps.py commands")

    last_pos = None  # (lat, lon, alt)
    last_recv = 0.0

    while True:
        # MAVLink'ten pozisyonu gncelle
        msg = m.recv_match(type=["GLOBAL_POSITION_INT", "HEARTBEAT"], blocking=False, timeout=0.01)
        if msg and msg.get_type() == "GLOBAL_POSITION_INT":
            last_pos = (
                msg.lat / 1e7,
                msg.lon / 1e7,
                msg.alt / 1000.0,
            )

        cmd = recv_json(sock)
        now = time.time()
        if cmd is None:
            continue
        last_recv = now

        # cmd format: {"yaw": deg, "speed": m/s, "alt": m, ...}
        yaw_deg = float(cmd.get("yaw", 0.0))
        speed = float(cmd.get("speed", 0.0))
        alt_cmd = float(cmd.get("alt", 0.0))

        yaw_rad = math.radians(yaw_deg)
        vx = speed * math.cos(yaw_rad)  # N
        vy = speed * math.sin(yaw_rad)  # E

        vz = 0.0
        if last_pos is not None:
            alt_err = alt_cmd - last_pos[2]
            vz = -max(-2.0, min(2.0, alt_err * 0.5))  # up is negative in NED

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

        try:
            m.mav.set_position_target_local_ned_send(
                0,
                m.target_system,
                m.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
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
            print(f"[json->mav] cmd yaw={yaw_deg:.1f} spd={speed:.1f} alt={alt_cmd:.1f} -> vx={vx:.1f} vy={vy:.1f} vz={vz:.2f}")
        except Exception as exc:
            print(f"[json->mav] send error: {exc}")

        # Balant hayatta kalsn diye arada heartbeat dinle
        if (now - last_recv) > 1.0:
            m.recv_match(type="HEARTBEAT", blocking=False, timeout=0.01)


if __name__ == "__main__":
    main()
