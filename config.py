from dataclasses import dataclass
from scipy.stats import chi2


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

    # Chi2 aykırı ölçüm test eşiği
    chi2_threshold: float = chi2.ppf(0.99, df=9)  # 9 boyutlu ölçüm için


@dataclass
class SimCfg:
    # Başlangıç pozisyonu (DMS formatı, destekleniyor)
    lat0: str = "39°55'14.8\"N"
    lon0: str = "32°51'14.8\"E"
    h0: float = 900.0

    # Simülasyon parametreleri
    T: float = 300.0           # toplam sim süresi (s)
    fs_imu: float = 50.0       # IMU frekansı (Hz)
    fs_gps: float = 1.0        # GPS frekansı (Hz)
    v0: float = 20.0           # başlangıç hızı

    # Hedef koordinatları
    target_lat: str = "39°56'00.0\"N"
    target_lon: str = "32°52'12.0\"E"
    target_h: float = 950.0


__all__ = ["KFConfig", "SimCfg"]

