import time
import math
import numpy as np
from dataclasses import dataclass

# Projendeki geo_utils dosyasından import ediliyor
from geo_utils import llh_to_enu

# ==========================
# AYARLAR
# ==========================
MY_TEAM_ID = 1  # Kendi takım numaran

# ==========================
# GÜÇLENDİRİLMİŞ 5D ÖLÇÜMLÜ UKF
# ==========================
class UnscentedKalmanFilter:
    """
    NONLINEAR 7D UKF (x, y, z, v, psi, psi_dot, vz)
    
    YENİLİK:
    - Artık iha_hiz ve iha_yonelme verilerini DOĞRUDAN ölçüm olarak alır.
    - Ölçüm Vektörü (z): [x, y, z, v, psi] (5 Boyutlu)
    """

    def __init__(self, x0, y0, z0, v0, psi0):
        self.n = 7
        # Durum: [x, y, z, v, psi, psi_dot, vz]
        # Başlangıç hızını ve yönünü de gelen veriden atıyoruz
        self.x = np.array([x0, y0, z0, v0, psi0, 0.0, 0.0], dtype=float)

        self.P = np.eye(self.n) * 10.0 # Başlangıç belirsizliği

        # Süreç Gürültüsü (Q) - Sistemin ne kadar "oynak" olduğu
        self.Q = np.diag([
            0.5,    # x (GPS gürültüsü kadar)
            0.5,    # y
            0.5,    # z
            2.0,    # v (Ani hızlanabilir)
            0.1,    # psi (Ani dönebilir)
            0.05,   # psi_dot
            1.0     # vz
        ])

        # Ölçüm Gürültüsü (R) - Sensörlerin ne kadar güvenilir olduğu
        # Ölçüm: [x, y, z, v, psi] -> 5 Boyutlu Matris
        self.R = np.diag([
            5.0,    # GPS X Hatası (m)
            5.0,    # GPS Y Hatası (m)
            8.0,    # GPS Z Hatası (m) - Z genelde daha hatalıdır
            1.0,    # Hız Hatası (m/s)
            math.radians(5.0) # Pusula/Yaw Hatası (radyan) ~5 derece
        ])

        # UKF Parametreleri
        self.alpha = 1e-3
        self.beta = 2.0
        self.kappa = 0.0
        self.lambda_ = self.alpha**2 * (self.n + self.kappa) - self.n

        self.Wm = np.full(2 * self.n + 1, 1.0 / (2 * (self.n + self.lambda_)))
        self.Wc = np.full(2 * self.n + 1, 1.0 / (2 * (self.n + self.lambda_)))

        self.Wm[0] = self.lambda_ / (self.n + self.lambda_)
        self.Wc[0] = self.lambda_ / (self.n + self.lambda_) + (1 - self.alpha**2 + self.beta)

    def f(self, x, dt: float):
        """Fiziksel Hareket Modeli (CTRV - Constant Turn Rate and Velocity)"""
        px, py, pz, v, psi, psi_dot, vz = x

        if dt <= 0: return x

        # Dönüş oranı (psi_dot) çok küçükse düz git, yoksa dairesel formül
        if abs(psi_dot) > 0.001:
            px_new = px + (v / psi_dot) * (math.sin(psi + psi_dot * dt) - math.sin(psi))
            py_new = py + (v / psi_dot) * (math.cos(psi) - math.cos(psi + psi_dot * dt))
        else:
            px_new = px + v * math.cos(psi) * dt
            py_new = py + v * math.sin(psi) * dt

        pz_new = pz + vz * dt
        psi_new = psi + psi_dot * dt
        
        # Açı normalizasyonu
        psi_new = (psi_new + math.pi) % (2 * math.pi) - math.pi

        return np.array([px_new, py_new, pz_new, v, psi_new, psi_dot, vz])

    def h(self, x):
        """
        Ölçüm Modeli (Measurement Function)
        Artık sadece konumu değil, HIZI ve AÇIYI da ölçüyoruz.
        Durumdan [x, y, z, v, psi] çekiyoruz.
        """
        return np.array([x[0], x[1], x[2], x[3], x[4]])

    def _sigma_points(self):
        """Korumalı Sigma Noktası Oluşturma"""
        self.P = (self.P + self.P.T) / 2.0
        jitter = 1e-6
        for _ in range(3):
            try:
                sqrt_P = np.linalg.cholesky((self.n + self.lambda_) * (self.P + np.eye(self.n) * jitter))
                break
            except np.linalg.LinAlgError:
                jitter *= 10.0
        else:
            sqrt_P = np.eye(self.n) * 1e-3

        sigmas = np.zeros((2 * self.n + 1, self.n))
        sigmas[0] = self.x
        for i in range(self.n):
            sigmas[i+1] = self.x + sqrt_P[i]
            sigmas[self.n+i+1] = self.x - sqrt_P[i]
        return sigmas

    def predict(self, dt):
        if dt <= 0: return
        try: sig = self._sigma_points()
        except: return

        for i in range(2 * self.n + 1):
            sig[i] = self.f(sig[i], dt)

        x_pred = np.zeros(self.n)
        for i in range(2 * self.n + 1):
            x_pred += self.Wm[i] * sig[i]

        P_pred = np.zeros((self.n, self.n))
        for i in range(2 * self.n + 1):
            y = sig[i] - x_pred
            y[4] = (y[4] + math.pi) % (2 * math.pi) - math.pi # Açı farkı
            P_pred += self.Wc[i] * np.outer(y, y)

        P_pred += self.Q
        self.x = x_pred
        self.P = P_pred
        self.P = (self.P + self.P.T) / 2.0

    def update(self, measurement):
        """
        measurement: [x, y, z, speed, yaw_rad]
        """
        z = np.array(measurement)
        try: sig = self._sigma_points()
        except: return

        Z_sig = np.zeros((2 * self.n + 1, 5)) # Ölçüm boyutu 5 oldu
        for i in range(2 * self.n + 1):
            Z_sig[i] = self.h(sig[i])

        z_pred = np.zeros(5)
        for i in range(2 * self.n + 1):
            z_pred += self.Wm[i] * Z_sig[i]

        S = np.zeros((5, 5))
        Pxz = np.zeros((self.n, 5))

        for i in range(2 * self.n + 1):
            dz = Z_sig[i] - z_pred
            # Yaw açısı (indeks 4) için fark hesabı
            dz[4] = (dz[4] + math.pi) % (2 * math.pi) - math.pi
            
            dx = sig[i] - self.x
            dx[4] = (dx[4] + math.pi) % (2 * math.pi) - math.pi
            
            S += self.Wc[i] * np.outer(dz, dz)
            Pxz += self.Wc[i] * np.outer(dx, dz)

        S += self.R

        try: S_inv = np.linalg.inv(S)
        except: S_inv = np.linalg.pinv(S)

        K = Pxz @ S_inv
        y = z - z_pred
        y[4] = (y[4] + math.pi) % (2 * math.pi) - math.pi # Residual açı düzeltmesi

        self.x = self.x + K @ y
        self.P = self.P - K @ S @ K.T
        self.P = (self.P + self.P.T) / 2.0

    def get_state(self):
        return self.x.copy()


# ==================================================
# RIVAL TRACKER - TEKNOFEST FULL PARSER
# ==================================================
class RivalTracker:
    def __init__(self, ref_lat, ref_lon, ref_h):
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon
        self.ref_h = ref_h

        self.filters = {}
        self.ignored = set()
        self.latest_lag_s = {}
        self.last_update_time = {}
        
        # Rakip durumunu saklamak için (Pitch/Roll gibi UKF'e girmeyenler için)
        self.rival_attitude = {} 

    def update_from_server_response(self, data):
        """
        Teknofest Verilerini Full İşler:
        - iha_enlem, iha_boylam, iha_irtifa -> Konum
        - iha_hiz -> Hız
        - iha_yonelme -> Yaw
        - iha_dikilme, iha_yatis -> Pitch, Roll (Kaydedilir)
        """
        now = time.time()

        for pkt in data:
            tid = pkt.get("takim_numarasi")
            if tid is None or tid == MY_TEAM_ID: continue

            try:
                # 1. TEMEL GPS VERİSİ
                lat = float(pkt["iha_enlem"])
                lon = float(pkt["iha_boylam"])
                alt = float(pkt["iha_irtifa"])
                
                # 2. EKSTRA VERİLER (Hız ve Yönelme KRİTİK)
                speed_ms = float(pkt.get("iha_hiz", 0.0))
                
                # Yaw (Derece -> Radyan)
                yaw_deg = float(pkt.get("iha_yonelme", 0.0))
                yaw_rad = math.radians(yaw_deg)
                
                # Pitch ve Roll (Sadece saklamak için, UKF durumu değil)
                pitch_deg = float(pkt.get("iha_dikilme", 0.0))
                roll_deg = float(pkt.get("iha_yatis", 0.0))
                
                # Gecikme
                delay_ms = float(pkt.get("gecikme_ms", 100.0))

            except (KeyError, ValueError, TypeError):
                continue # Veri bozuksa atla

            # GPS -> ENU Dönüşümü
            x, y, z = llh_to_enu(lat, lon, alt, self.ref_lat, self.ref_lon, self.ref_h)

            # --- UKF GÜNCELLEME ---
            if tid not in self.filters:
                # İlk başlatırken hızı ve yönü de veriyoruz!
                self.filters[tid] = UnscentedKalmanFilter(x, y, z, speed_ms, yaw_rad)
                self.last_update_time[tid] = now
            else:
                dt = now - self.last_update_time.get(tid, now)
                if dt < 0.001: dt = 0.001
                
                self.filters[tid].predict(dt)
                
                # 5D ÖLÇÜM GÖNDERİYORUZ: [X, Y, Z, V, YAW]
                self.filters[tid].update([x, y, z, speed_ms, yaw_rad])
                
                self.last_update_time[tid] = now
            
            # Ekstra verileri ve gecikmeyi sakla
            self.latest_lag_s[tid] = delay_ms / 1000.0
            self.rival_attitude[tid] = {"pitch": pitch_deg, "roll": roll_deg}

    def ignore_rival(self, tid):
        self.ignored.add(tid)
        print(f"🚫 Rakip {tid} VURULDU (Listeden çıkarıldı).")

    def get_closest_rival(self, my_x, my_y, my_z):
        now = time.time()
        closest = None
        min_d = float("inf")

        for tid, ukf in self.filters.items():
            if tid in self.ignored: continue
            if now - self.last_update_time.get(tid, 0) > 5.0: continue

            # UKF Durumu: [x, y, z, v, psi, psi_dot, vz]
            s = ukf.get_state()
            px, py, pz = s[0], s[1], s[2]
            v = s[3]
            psi = s[4]
            vz = s[6]

            # Latency Telafisi
            lag = self.latest_lag_s.get(tid, 0.1)
            dt_future = lag + (now - self.last_update_time[tid])
            
            # Tahmini Gelecek Pozisyon
            px_f = px + v * math.cos(psi) * dt_future
            py_f = py + v * math.sin(psi) * dt_future
            pz_f = pz + vz * dt_future

            dist = math.sqrt((px_f - my_x)**2 + (py_f - my_y)**2 + (pz_f - my_z)**2)

            if dist < min_d:
                min_d = dist
                att = self.rival_attitude.get(tid, {"pitch":0, "roll":0})
                closest = {
                    "takim_numarasi": tid,
                    "x": px_f, "y": py_f, "z": pz_f,
                    "hiz": math.sqrt(v*v + vz*vz),
                    "yaw_deg": math.degrees(psi),
                    "pitch_deg": att["pitch"], # Ekstra bilgi
                    "roll_deg": att["roll"],   # Ekstra bilgi
                    "dist": dist,
                    "gecikme_ms": dt_future * 1000.0
                }

        return closest