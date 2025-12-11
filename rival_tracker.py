import time
import math
from collections import deque
from dataclasses import dataclass
import numpy as np

from geo_utils import llh_to_enu
from tactical_modules import RivalBehaviorMemory, MultiTargetOptimizer

# ==========================
# AYARLAR
# ==========================
MAX_RIVAL_SPEED = 70.0  # m/s hz patlamasn engelle

# Hedef puanlama katsaylar (4.1.2 forml referans)
DEFAULT_SCORING_WEIGHTS = {
    "w_h": 0.5,         # irtifa fark (|h_H - h_G|) - metre cinsinden
    "w_d": 0.003,       # yatay mesafe - metre cinsinden (ornek: 100m = 0.3 puan)
    "w_heading": 0.01,  # yonelim farki - derece cinsinden (ornek: 45deg = 0.45 puan)
    "w_inv_speed": 2.0  # 1 / V_H - yavas hedeflere bonus (ornek: 10m/s = 0.2 puan)
}
MIN_INV_SPEED = 5.0  # Cok yavas hedefleri fazla tercih etmemek icin (m/s)


# ==========================
# GLENDRLM 5D LML UKF
# ==========================
class UnscentedKalmanFilter:
    """
    NONLINEAR 10D UKF (x, y, z, v, psi, psi_dot, vz, ax, ay, az)

    State vektoru: [x, y, z, v, psi, psi_dot, vz, ax, ay, az]
    Olcum vektoru: [x, y, z, v, psi] (5 boyut)

    ax, ay: Yatay duzlemde ivme (m/s^2)
    az: Dikey ivme (m/s^2)
    """

    def __init__(self, x0, y0, z0, v0, psi0):
        self.n = 10
        # Durum: [x, y, z, v, psi, psi_dot, vz, ax, ay, az]
        self.x = np.array([x0, y0, z0, v0, psi0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)

        self.P = np.eye(self.n) * 10.0  # belirsizlik

        # Sre grlts (Q)
        self.Q = np.diag([
            0.5,    # x
            0.5,    # y
            0.5,    # z
            2.0,    # v
            0.1,    # psi
            0.05,   # psi_dot
            1.0,    # vz
            0.8,    # ax - yatay ivme belirsizligi
            0.8,    # ay - yatay ivme belirsizligi
            0.6     # az - dikey ivme belirsizligi (daha az değişken)
        ])

        # lm grlts (R)
        self.R = np.diag([
            5.0,    # x
            5.0,    # y
            8.0,    # z
            1.0,    # speed
            math.radians(5.0)  # yaw rad
        ])

        # UKF parametreleri
        self.alpha = 1e-3
        self.beta = 2.0
        self.kappa = 0.0
        self.lambda_ = self.alpha**2 * (self.n + self.kappa) - self.n

        self.Wm = np.full(2 * self.n + 1, 1.0 / (2 * (self.n + self.lambda_)))
        self.Wc = np.full(2 * self.n + 1, 1.0 / (2 * (self.n + self.lambda_)))

        self.Wm[0] = self.lambda_ / (self.n + self.lambda_)
        self.Wc[0] = self.lambda_ / (self.n + self.lambda_) + (1 - self.alpha**2 + self.beta)

    def f(self, x, dt: float):
        """
        CTRV + 3D Ivme hareket modeli
        State: [x, y, z, v, psi, psi_dot, vz, ax, ay, az]
        """
        px, py, pz, v, psi, psi_dot, vz, ax, ay, az = x

        if dt <= 0:
            return x

        # Ivme ile hiz guncellemesi
        # v_new = sqrt(vx^2 + vy^2), ivme eklenince degisir
        vx = v * math.cos(psi)
        vy = v * math.sin(psi)

        vx_new = vx + ax * dt
        vy_new = vy + ay * dt
        v_new = math.sqrt(vx_new**2 + vy_new**2)

        # Yeni yonelim (ivme yonelimi degistirebilir)
        if v_new > 0.5:  # Yeterince hizli ise
            psi_from_velocity = math.atan2(vy_new, vx_new)
            # Eski psi ile yeni psi'yi karistir (smooth transition)
            psi_blend = 0.3  # 30% yeni, 70% eski
            psi_new = psi + psi_dot * dt
            psi_new = (1 - psi_blend) * psi_new + psi_blend * psi_from_velocity
        else:
            psi_new = psi + psi_dot * dt

        # Konum guncellemesi (ivme dahil)
        if abs(psi_dot) > 0.001:
            # Donme hareketi
            px_new = px + (v / psi_dot) * (math.sin(psi + psi_dot * dt) - math.sin(psi)) + 0.5 * ax * dt**2
            py_new = py + (v / psi_dot) * (math.cos(psi) - math.cos(psi + psi_dot * dt)) + 0.5 * ay * dt**2
        else:
            # Duz hareket
            px_new = px + v * math.cos(psi) * dt + 0.5 * ax * dt**2
            py_new = py + v * math.sin(psi) * dt + 0.5 * ay * dt**2

        # Dikey hareket (ivme dahil)
        pz_new = pz + vz * dt + 0.5 * az * dt**2
        vz_new = vz + az * dt

        # Aci normalizasyonu
        psi_new = (psi_new + math.pi) % (2 * math.pi) - math.pi

        # Ivme azalma (drag benzeri) - manevra bittikten sonra ivme sifira donme egilimi
        ax_new = ax * 0.9
        ay_new = ay * 0.9
        az_new = az * 0.85  # Dikey ivme daha hızlı sönümlenir

        return np.array([px_new, py_new, pz_new, v_new, psi_new, psi_dot, vz_new, ax_new, ay_new, az_new])

    def h(self, x):
        """lm modeli"""
        return np.array([x[0], x[1], x[2], x[3], x[4]])

    def _sigma_points(self):
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
        if dt <= 0:
            return
        try:
            sig = self._sigma_points()
        except Exception:
            return

        for i in range(2 * self.n + 1):
            sig[i] = self.f(sig[i], dt)

        x_pred = np.zeros(self.n)
        for i in range(2 * self.n + 1):
            x_pred += self.Wm[i] * sig[i]

        P_pred = np.zeros((self.n, self.n))
        for i in range(2 * self.n + 1):
            y = sig[i] - x_pred
            y[4] = (y[4] + math.pi) % (2 * math.pi) - math.pi
            P_pred += self.Wc[i] * np.outer(y, y)

        P_pred += self.Q
        self.x = x_pred
        self.P = (P_pred + P_pred.T) / 2.0

    def update(self, measurement):
        """
        measurement: [x, y, z, speed, yaw_rad]
        """
        z = np.array(measurement)
        try:
            sig = self._sigma_points()
        except Exception:
            return

        Z_sig = np.zeros((2 * self.n + 1, 5))
        for i in range(2 * self.n + 1):
            Z_sig[i] = self.h(sig[i])

        z_pred = np.zeros(5)
        for i in range(2 * self.n + 1):
            z_pred += self.Wm[i] * Z_sig[i]

        S = np.zeros((5, 5))
        Pxz = np.zeros((self.n, 5))

        for i in range(2 * self.n + 1):
            dz = Z_sig[i] - z_pred
            dz[4] = (dz[4] + math.pi) % (2 * math.pi) - math.pi

            dx = sig[i] - self.x
            dx[4] = (dx[4] + math.pi) % (2 * math.pi) - math.pi

            S += self.Wc[i] * np.outer(dz, dz)
            Pxz += self.Wc[i] * np.outer(dx, dz)

        S += self.R

        try:
            S_inv = np.linalg.inv(S)
        except Exception:
            S_inv = np.linalg.pinv(S)

        K = Pxz @ S_inv
        y = z - z_pred
        y[4] = (y[4] + math.pi) % (2 * math.pi) - math.pi

        self.x = self.x + K @ y
        self.P = self.P - K @ S @ K.T
        self.P = (self.P + self.P.T) / 2.0

        # Hz patlamasn frenle
        v_mag = abs(self.x[3])
        if v_mag > MAX_RIVAL_SPEED:
            self.x[3] = math.copysign(MAX_RIVAL_SPEED, self.x[3])

    def get_state(self):
        return self.x.copy()


# ==================================================
# RIVAL TRACKER - TEKNOFEST FULL PARSER
# ==================================================
class RivalTracker:
    def __init__(self, ref_lat, ref_lon, ref_h, my_team_id=2, scoring_weights=None):
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon
        self.ref_h = ref_h
        self.my_team_id = my_team_id  # Kendi takım ID'miz

        self.filters = {}
        self.ignored = set()
        self.latest_lag_s = {}
        self.last_update_time = {}
        self.history = {}  # tid -> deque(t, x, y, z, v, psi, vz)

        # Rakip tutumunu (pitch/roll) saklamak iin
        self.rival_attitude = {}

        # Puanlama katsaylar
        self.weights = dict(DEFAULT_SCORING_WEIGHTS)
        if scoring_weights:
            self.weights.update(scoring_weights)

        # Taktiksel modüller (tek sefer tanımla)
        self.behavior_memory = RivalBehaviorMemory()
        self.multi_target_optimizer = MultiTargetOptimizer()

        print(f"[RivalTracker] Kendi takim ID: {self.my_team_id}")

    def update_from_server_response(self, data):
        """
        Teknofest verilerini iler:
        - iha_enlem, iha_boylam, iha_irtifa -> konum
        - iha_hiz -> hz
        - iha_yonelme -> yaw
        - iha_dikilme, iha_yatis -> pitch, roll
        """
        now = time.time()

        for pkt in data:
            tid = pkt.get("takim_numarasi")
            if tid is None or tid == self.my_team_id:
                continue

            try:
                lat = float(pkt["iha_enlem"])
                lon = float(pkt["iha_boylam"])
                alt = float(pkt["iha_irtifa"])
                speed_ms = float(pkt.get("iha_hiz", 0.0))
                yaw_deg = float(pkt.get("iha_yonelme", 0.0))
                yaw_rad = math.radians(yaw_deg)
                pitch_deg = float(pkt.get("iha_dikilme", 0.0))
                roll_deg = float(pkt.get("iha_yatis", 0.0))
                delay_ms = float(pkt.get("gecikme_ms", 100.0))
            except (KeyError, ValueError, TypeError):
                continue

            # GPS -> ENU
            x, y, z = llh_to_enu(lat, lon, alt, self.ref_lat, self.ref_lon, self.ref_h)
            vz_est = 0.0

            # History'den ivme tahmini (3D)
            hist = self.history.setdefault(tid, deque(maxlen=50))
            ax_est = 0.0
            ay_est = 0.0
            az_est = 0.0

            if len(hist) >= 2:
                # Son iki olcumden ivme hesapla
                t2, x2, y2, z2, v2, psi2, vz2 = hist[-1]
                t1, x1, y1, z1, v1, psi1, vz1 = hist[-2]

                dt_hist = max(1e-3, t2 - t1)

                # Yatay hiz vektorleri
                vx2 = v2 * math.cos(psi2)
                vy2 = v2 * math.sin(psi2)
                vx1 = v1 * math.cos(psi1)
                vy1 = v1 * math.sin(psi1)

                # Yatay ivme tahmini
                ax_est = (vx2 - vx1) / dt_hist
                ay_est = (vy2 - vy1) / dt_hist

                # Dikey ivme tahmini
                az_est = (vz2 - vz1) / dt_hist

                # Ivme limitleme (mantikli degerler icin)
                ax_est = np.clip(ax_est, -10.0, 10.0)
                ay_est = np.clip(ay_est, -10.0, 10.0)
                az_est = np.clip(az_est, -8.0, 8.0)  # Dikey ivme daha sınırlı

            # Dikey hiz tahmini
            if len(hist) >= 1:
                last_t, _, _, last_z, _, _, _ = hist[-1]
                dt_z = max(1e-3, now - last_t)
                vz_est = (z - last_z) / dt_z
                vz_est = np.clip(vz_est, -10.0, 10.0)

            # UKF
            if tid not in self.filters:
                self.filters[tid] = UnscentedKalmanFilter(x, y, z, speed_ms, yaw_rad)
                self.last_update_time[tid] = now
            else:
                dt = now - self.last_update_time.get(tid, now)
                if dt < 0.001:
                    dt = 0.001

                self.filters[tid].predict(dt)
                self.filters[tid].update([x, y, z, speed_ms, yaw_rad])

                # 3D Ivme tahminini UKF state'ine enjekte et
                if ax_est != 0.0 or ay_est != 0.0 or az_est != 0.0:
                    # Mevcut ivme ile yeni ivmeyi harmanlayalim (smooth)
                    blend = 0.3  # 30% yeni, 70% eski
                    self.filters[tid].x[7] = (1 - blend) * self.filters[tid].x[7] + blend * ax_est
                    self.filters[tid].x[8] = (1 - blend) * self.filters[tid].x[8] + blend * ay_est
                    self.filters[tid].x[9] = (1 - blend) * self.filters[tid].x[9] + blend * az_est

                self.last_update_time[tid] = now

            hist.append((now, x, y, z, speed_ms, yaw_rad, vz_est))

            self.latest_lag_s[tid] = delay_ms / 1000.0
            self.rival_attitude[tid] = {"pitch": pitch_deg, "roll": roll_deg}

    def ignore_rival(self, tid):
        self.ignored.add(tid)
        print(f"Rakip {tid} VURULDU (listeden karld).")

    def report_lock_event(self, tid):
        """
        Grsel kilit tamamlandnda rakibi tek seferlik olarak ignore set'ine atar.
        Ayn rakibe ikinci kez vurulduysa False dner.
        """
        if tid in self.ignored:
            return False
        self.ignore_rival(tid)
        return True

    def get_all_rivals(self):
        """
        Tüm aktif rakiplerin pozisyonlarını döndürür (çarpışma önleme için).
        Returns: dict {team_id: {"x": px, "y": py, "z": pz, "speed": v}}
        """
        now = time.time()
        all_rivals = {}

        for tid, ukf in self.filters.items():
            if tid in self.ignored:
                continue
            if now - self.last_update_time.get(tid, 0) > 5.0:  # Eski veri
                continue

            s = ukf.get_state()
            px, py, pz = s[0], s[1], s[2]
            v = s[3]

            all_rivals[tid] = {
                "x": px,
                "y": py,
                "z": pz,
                "speed": v,
            }

        return all_rivals

    def get_closest_rival(self, my_x, my_y, my_z, my_heading_deg):
        now = time.time()
        best = None
        best_score = -float("inf")
        my_heading_deg = (my_heading_deg if my_heading_deg is not None else 0.0) % 360.0
        my_pos = (my_x, my_y, my_z)
        my_dist_from_center = math.sqrt(my_x**2 + my_y**2)
        ARENA_RADIUS = 500.0
        SAFE_ZONE = 450.0

        for tid, ukf in self.filters.items():
            if tid in self.ignored:
                continue
            if now - self.last_update_time.get(tid, 0) > 5.0:
                continue

            s = ukf.get_state()
            px, py, pz = s[0], s[1], s[2]
            v = s[3]
            psi = s[4]
            vz = s[6]
            ax = s[7] if len(s) > 7 else 0.0  # Yatay ivme bilgisi
            ay = s[8] if len(s) > 8 else 0.0
            az = s[9] if len(s) > 9 else 0.0  # Dikey ivme bilgisi

            target_dist_from_center = math.sqrt(px**2 + py**2)
            if target_dist_from_center > ARENA_RADIUS:
                continue

            lag = self.latest_lag_s.get(tid, 0.1)
            dt_future = lag + (now - self.last_update_time[tid])
            # 3D Ivme bilgisini de gonder
            px_f, py_f, pz_f, v_f = self._predict_future(tid, (px, py, pz, v, psi, vz, ax, ay, az), dt_future)

            predicted_dist_from_center = math.sqrt(px_f**2 + py_f**2)
            if predicted_dist_from_center > ARENA_RADIUS:
                continue

            dist = math.sqrt((px_f - my_x) ** 2 + (py_f - my_y) ** 2 + (pz_f - my_z) ** 2)

            yaw_deg = math.degrees(psi) % 360.0
            suitability, parts = self._score_candidate(
                my_pos, my_heading_deg, px_f, py_f, pz_f, v_f, yaw_deg
            )

            if suitability > best_score:
                best_score = suitability
                att = self.rival_attitude.get(tid, {"pitch": 0, "roll": 0})
                best = {
                    "takim_numarasi": tid,
                    "x": px_f,
                    "y": py_f,
                    "z": pz_f,
                    "hiz": math.sqrt(v_f * v_f + vz * vz),
                    "yaw_deg": yaw_deg,
                    "yaw_rate": self._estimate_yaw_rate(tid),
                    "pitch_deg": att["pitch"],
                    "roll_deg": att["roll"],
                    "dist": dist,
                    "gecikme_ms": dt_future * 1000.0,
                    "score": suitability,
                    "raw_score": parts["raw_total"],
                    "heading_diff": parts["heading_diff"],
                }

        return best

    # -------------------------------------------------------------
    # Yardmclar
    # -------------------------------------------------------------
    def _predict_future(self, tid, state, dt_future: float):
        """
        UKF state'inden gelen 3D ivme bilgisi ile gelecek konumu tahmin eder.
        State: (px, py, pz, v, psi, vz, ax, ay, az)
        """
        if len(state) == 6:
            # Eski format (ivme yok)
            px, py, pz, v, psi, vz = state
            ax = ay = az = 0.0
        elif len(state) == 8:
            # Orta format (sadece yatay ivme)
            px, py, pz, v, psi, vz, ax, ay = state
            az = 0.0
        else:
            # Yeni format (3D ivme var)
            px, py, pz, v, psi, vz, ax, ay, az = state

        # Yatay hiz vektorleri
        vx = v * math.cos(psi)
        vy = v * math.sin(psi)

        # 3D Ivmeli projeksiyon (kinematics: x = x0 + v*t + 0.5*a*t^2)
        px_f = px + vx * dt_future + 0.5 * ax * dt_future**2
        py_f = py + vy * dt_future + 0.5 * ay * dt_future**2
        pz_f = pz + vz * dt_future + 0.5 * az * dt_future**2  # Dikey ivme dahil

        # Gelecekteki hiz (3D)
        vx_f = vx + ax * dt_future
        vy_f = vy + ay * dt_future
        vz_f = vz + az * dt_future

        # Toplam hiz buyuklugu
        speed_mag = math.sqrt(vx_f**2 + vy_f**2 + vz_f**2)

        # Hiz limiti
        if speed_mag > MAX_RIVAL_SPEED:
            scale = MAX_RIVAL_SPEED / speed_mag
            vx_f *= scale
            vy_f *= scale
            vz_f *= scale

        v_f = math.sqrt(vx_f**2 + vy_f**2 + vz_f**2)
        return px_f, py_f, pz_f, v_f

    def _velocity_variability(self, tid):
        hist = self.history.get(tid)
        if not hist or len(hist) < 3:
            return 0.0

        speeds = []
        last = None
        for item in list(hist)[-5:]:
            t, x, y, z, _, _, _ = item
            if last:
                dt = max(1e-3, t - last[0])
                vx = (x - last[1]) / dt
                vy = (y - last[2]) / dt
                vz = (z - last[3]) / dt
                speeds.append(math.sqrt(vx * vx + vy * vy + vz * vz))
            last = (t, x, y, z)

        if not speeds:
            return 0.0
        return float(np.std(speeds))

    def _estimate_yaw_rate(self, tid):
        hist = self.history.get(tid)
        if not hist or len(hist) < 2:
            return 0.0
        t2, _, _, _, _, yaw2, _ = hist[-1]
        t1, _, _, _, _, yaw1, _ = hist[-2]
        dt = max(1e-3, t2 - t1)
        yaw_rate = (yaw2 - yaw1) / dt
        return math.degrees(yaw_rate)

    def _score_candidate(self, my_pos, my_heading_deg, px_f, py_f, pz_f, v_f, yaw_deg):
        """
        4.1.2 formllerine gre puan hesaplar.
        Dnecek deer: 1 / (1 + (P_h + P_d + P_BV))
        """
        h_diff = abs(my_pos[2] - pz_f)
        d_xy = math.hypot(px_f - my_pos[0], py_f - my_pos[1])
        heading_diff = abs(((my_heading_deg - yaw_deg) + 180.0) % 180.0)
        inv_speed = 1.0 / max(v_f, MIN_INV_SPEED)

        ph = self.weights["w_h"] * h_diff
        pd = self.weights["w_d"] * d_xy
        pbv = self.weights["w_heading"] * heading_diff + self.weights["w_inv_speed"] * inv_speed

        raw_total = ph + pd + pbv
        suitability = 1.0 / (1.0 + raw_total)

        return suitability, {
            "raw_total": raw_total,
            "h_diff": h_diff,
            "d_xy": d_xy,
            "heading_diff": heading_diff,
            "inv_speed": inv_speed,
        }


__all__ = ["RivalTracker"]
