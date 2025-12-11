import os
from dataclasses import dataclass
from scipy.stats import chi2


@dataclass
class VehicleConfig:
    """İHA kimlik ve takım bilgileri"""
    vehicle_id: int = int(os.getenv("VEHICLE_ID", "1"))
    team_id: int = int(os.getenv("MY_TEAM_ID", "2"))
    vehicle_type: str = os.getenv("VEHICLE_TYPE", "plane")
    version: str = "1.0.0"


@dataclass
class KFConfig:
    # Süreç Gürültüleri
    q_jerk: float = 50.0
    q_omega_bias: float = 0.0005
    q_acc_bias: float = 0.2

    # Ölçüm Gürültüleri
    r_gps_h: float = 3.0
    r_gps_v: float = 5.0
    r_acc: float = 0.5
    r_mag: float = 0.05
    r_imu: float = 0.01
    r_speed: float = 0.5

    # Chi2 aykırı ölçüm testi
    chi2_threshold: float = chi2.ppf(0.99, df=9)


@dataclass
class SimCfg:
    # Başlangıç pozisyonu (DMS, yön harfi ile)
    # SITL default location: Canberra, Australia
    lat0: str = "35°21'47.6\"S"
    lon0: str = "149°09'54.9\"E"
    h0: float = 584.0

    # Simülasyon parametreleri
    T: float = 300.0
    fs_imu: float = 50.0
    fs_gps: float = 1.0
    v0: float = 20.0

    # Hedef koordinatları
    target_lat: str = "35°21'50.0\"S"
    target_lon: str = "149°10'00.0\"E"
    target_h: float = 600.0


__all__ = ["VehicleConfig", "KFConfig", "SimCfg"]
