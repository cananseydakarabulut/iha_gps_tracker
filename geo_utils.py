import math
import re
import numpy as np

R = 6371000.0  # Dünya yarıçapı (m)


def dmstodecimal(deg, minute, sec, direction):
    decimal = deg + (minute / 60.0) + (sec / 3600.0)
    if direction in ["S", "W"]:
        decimal = -decimal
    return decimal


def parse_if_dms(value):
    if isinstance(value, (int, float)):
        return float(value)
    value = str(value).strip()

    # Desteklenen formatlar: 39°55'14.8"N, 39 55 14.8N, 39d55m14.8sN
    dms_pattern = re.compile(
        r"(\d+)[°ºd]?\s*(\d+)'?\s*([\d.]+)\"?\s*([NSEW])",
        re.IGNORECASE,
    )
    match = dms_pattern.match(value.replace(" ", ""))
    if match:
        deg, minute, sec, direction = match.groups()
        return dmstodecimal(float(deg), float(minute), float(sec), direction.upper())

    value = value.replace(",", ".")
    if value and value[-1] in ["N", "S", "E", "W"]:
        num = float(value[:-1])
        if value[-1] in ["S", "W"]:
            num = -num
        return num

    return float(value)


def haversine_m(lat1, lon1, lat2, lon2):
    lat1 = parse_if_dms(lat1)
    lon1 = parse_if_dms(lon1)
    lat2 = parse_if_dms(lat2)
    lon2 = parse_if_dms(lon2)

    p = math.pi / 180.0
    dlat = (lat2 - lat1) * p
    dlon = (lon2 - lon1) * p
    lat1r = lat1 * p
    lat2r = lat2 * p
    a = math.sin(dlat / 2.0) ** 2 + math.cos(lat1r) * math.cos(lat2r) * math.sin(dlon / 2.0) ** 2
    c = 2.0 * math.asin(math.sqrt(a))
    return R * c


def bearing_deg(lat1, lon1, lat2, lon2):
    lat1 = parse_if_dms(lat1)
    lon1 = parse_if_dms(lon1)
    lat2 = parse_if_dms(lat2)
    lon2 = parse_if_dms(lon2)

    p = math.pi / 180.0
    y = math.sin((lon2 - lon1) * p) * math.cos(lat2 * p)
    x = math.cos(lat1 * p) * math.sin(lat2 * p) - math.sin(lat1 * p) * math.cos(lat2 * p) * math.cos((lon2 - lon1) * p)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


# ==============================================================================
# Parametre sırası (Hedef, Referans) olarak ayarlandı.
# gps.py ve rival_tracker.py bu sırayı bekliyor.
# ==============================================================================
def llh_to_enu(lat, lon, h, lat_ref, lon_ref, h_ref):
    """
    GPS -> ENU (Metre)
    Sıralama: Hedef Lat/Lon/Alt, sonra Referans Lat/Lon/Alt
    """
    pr = math.pi / 180.0
    lat_ref = parse_if_dms(lat_ref)
    lon_ref = parse_if_dms(lon_ref)
    lat = parse_if_dms(lat)
    lon = parse_if_dms(lon)

    dlat = (lat - lat_ref) * pr
    dlon = (lon - lon_ref) * pr

    x = R * math.cos(lat_ref * pr) * dlon
    y = R * dlat
    z = h - h_ref

    return np.array([x, y, z], dtype=float)


def enu_to_llh(x, y, z, lat_ref, lon_ref, h_ref):
    """
    ENU (Metre) -> GPS
    """
    pr = math.pi / 180.0
    lat_ref = parse_if_dms(lat_ref)
    lon_ref = parse_if_dms(lon_ref)

    lat = lat_ref + (y / R) / pr
    lon = lon_ref + (x / (R * math.cos(lat_ref * pr))) / pr
    h = h_ref + z

    return lat, lon, h


__all__ = [
    "R",
    "bearing_deg",
    "enu_to_llh",
    "haversine_m",
    "llh_to_enu",
    "parse_if_dms",
]
