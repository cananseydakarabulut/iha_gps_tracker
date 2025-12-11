import os
from dataclasses import dataclass
from scipy.stats import chi2


@dataclass
class VehicleConfig:
    """İHA kimlik ve takım bilgileri"""
    # Kendi İHA ID'miz (ortam değişkeninden veya varsayılan)
    vehicle_id: int = int(os.getenv("VEHICLE_ID", "1"))

    # Kendi takım numaramız (ortam değişkeninden veya varsayılan)
    team_id: int = int(os.getenv("MY_TEAM_ID", "2"))

    # İHA tipi (plane, copter, vtol)
    vehicle_type: str = os.getenv("VEHICLE_TYPE", "plane")

    # Sürüm bilgisi
    version: str = "1.0.0"


@dataclass
class KFConfig:
    # Süreç Gürültüleri
    q_jerk: float = 50.0           # pozisyon-hız model jerk gürültüsü
    q_omega_bias: float = 0.0005   # gyro bias süreç gürültüsü
    q_acc_bias: float = 0.2        # ivme bias süreç gürültüsü

    # Ölçüm Gürültüleri
    r_gps_h: float = 3.0           # yatay GPS gürültüsü (m)
    r_gps_v: float = 5.0           # dikey GPS gürültüsü (m)
    r_acc: float = 0.5             # ivme gürültüsü
    r_mag: float = 0.05            # manyetometre gürültüsü
    r_imu: float = 0.01            # gyro gürültüsü (rad/s)
    r_speed: float = 0.5           # hız sensör gürültüsü (m/s) - GPS hız daha güvenilir

    # Chi2 aykırı ölçüm test eşiği
    chi2_threshold: float = chi2.ppf(0.99, df=9)  # 9 boyutlu ölçüm için


@dataclass
class SimCfg:
    # Başlangıç pozisyonu (DMS formatı, destekleniyor)
    # SITL default location: Canberra, Australia
    lat0: str = "-35°21'47.6\"S"
    lon0: str = "149°09'54.9\"E"
    h0: float = 584.0

    # Simülasyon parametreleri
    T: float = 300.0           # toplam sim süresi (s)
    fs_imu: float = 50.0       # IMU frekansı (Hz)
    fs_gps: float = 1.0        # GPS frekansı (Hz)
    v0: float = 20.0           # başlangıç hızı

    # Hedef koordinatları (SITL test için Avustralya'da)
    target_lat: str = "-35°21'50.0\"S"
    target_lon: str = "149°10'00.0\"E"
    target_h: float = 600.0


__all__ = ["VehicleConfig", "KFConfig", "SimCfg"]

