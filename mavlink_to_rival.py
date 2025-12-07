#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SITL/MAVLink'ten rakip verisini okuyup gps.py'ya Teknofest formatında yollar.
Default: MAVProxy out udp:127.0.0.1:14560 (SITL-2), gps.py dinleme 5799.
"""
import argparse
import json
import math
import socket
import time
from typing import Optional

from pymavlink import mavutil


def connect(master_str: str):
    print(f"[rival] Connecting MAVLink: {master_str}")
    m = mavutil.mavlink_connection(master_str)
    m.wait_heartbeat()
    print(f"[rival] Heartbeat ok sysid={m.target_system} compid={m.target_component}")
    return m


def recv_matching(m: mavutil.mavlink_connection, types):
    try:
        return m.recv_match(type=types, blocking=False, timeout=0.02)
    except Exception:
        return None


def build_packet(gpos, vfr, att, team_id: int, iha_id: int) -> Optional[dict]:
    if gpos is None:
        return None

    lat = gpos.lat / 1e7
    lon = gpos.lon / 1e7
    alt = gpos.alt / 1000.0  # mm -> m

    vx = getattr(gpos, "vx", 0) / 100.0
    vy = getattr(gpos, "vy", 0) / 100.0
    speed_ms = math.sqrt(vx * vx + vy * vy)
    if vfr:
        speed_ms = max(speed_ms, getattr(vfr, "groundspeed", 0.0))

    heading = gpos.hdg / 100.0 if getattr(gpos, "hdg", 65535) != 65535 else 0.0

    pitch = getattr(att, "pitch", 0.0) if att else 0.0
    roll = getattr(att, "roll", 0.0) if att else 0.0

    return {
        "type": "rival_gps",
        "packets": [
            {
                "takim_id": team_id,
                "iha_id": iha_id,
                "konum": {
                    "enlem": lat,
                    "boylam": lon,
                    "irtifa": alt,
                    "aci": {"pitch": pitch, "roll": roll},
                    "hiz": speed_ms,
                    "yonelim": heading,
                },
                "gecikme_ms": 50.0,
            }
        ],
    }


def main():
    ap = argparse.ArgumentParser(description="MAVLink -> gps.py rival bridge")
    ap.add_argument("--master", default="udp:127.0.0.1:14560", help="MAVLink master (rakip SITL out)")
    ap.add_argument("--out-ip", default="127.0.0.1", help="gps.py dinleyen IP")
    ap.add_argument("--out-port", type=int, default=5799, help="gps.py dinleyen port")
    ap.add_argument("--team-id", type=int, default=7, help="Rakip takım ID")
    ap.add_argument("--iha-id", type=int, default=2, help="Rakip araç ID")
    ap.add_argument("--rate-hz", type=float, default=10.0, help="Gönderim hızı (Hz)")
    args = ap.parse_args()

    m = connect(args.master)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / max(args.rate_hz, 0.1)
    last_send = 0.0

    last_gpos = None
    last_vfr = None
    last_att = None

    while True:
        msg = recv_matching(m, ["GLOBAL_POSITION_INT", "VFR_HUD", "ATTITUDE"])
        now = time.time()
        if msg is None:
            time.sleep(0.01)
            continue

        mtype = msg.get_type()
        if mtype == "GLOBAL_POSITION_INT":
            last_gpos = msg
        elif mtype == "VFR_HUD":
            last_vfr = msg
        elif mtype == "ATTITUDE":
            last_att = msg

        if last_gpos and (now - last_send) >= period:
            pkt = build_packet(last_gpos, last_vfr, last_att, args.team_id, args.iha_id)
            if pkt:
                payload = json.dumps(pkt).encode("utf-8")
                sock.sendto(payload, (args.out_ip, args.out_port))
                last_send = now


if __name__ == "__main__":
    main()
