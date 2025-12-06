import time
import math
from collections import deque
from dataclasses import dataclass
import numpy as np

from geo_utils import llh_to_enu

# ==========================
# AYARLAR
# ==========================
MY_TEAM_ID = 1  # kendi takım numaran
MAX_RIVAL_SPEED = 70.0  # m/s hız patlamasını engelle

# Hedef puanlama katsayıları (4.1.2 formül referansı)
DEFAULT_SCORING_WEIGHTS = {
    "w_h": 0.6,         # irtifa farkı (|h_H - h_G|)
    "w_d": 0.25,        # yatay mesafe
    "w_heading": 0.12,  # b_GH
    "w_inv_speed": 15.0 # 1 / V_H
}
MIN_INV_SPEED = 1.0  # 1/V hesaplarında sıfır bölünmemeleri için


# ==========================
# GÜÇLENDİRİLMİŞ 5D ÖLÇÜMLÜ UKF
# ==========================
class UnscentedKalmanFilter:
    """
    NONLINEAR 7D UKF (x, y, z, v, psi, psi_dot, vz)

    Ölçüm vektörü: [x, y, z, v, psi] (5 boyut)
    """

    def __init__(self, x0, y0, z0, v0, psi0):
        self.n = 7
        # Durum: [x, y, z, v, psi, psi_dot, vz]
        self.x = np.array([x0, y0, z0, v0, psi0, 0.0, 0.0], dtype=float)

        self.P = np.eye(self.n) * 10.0  # belirsizlik

        # Süreç gürültüsü (Q)
        self.Q = np.diag([
            0.5,    # x
            0.5,    # y
            0.5,    # z
            2.0,    # v
            0.1,    # psi
            0.05,   # psi_dot
            1.0     # vz
        ])

        # Ölçüm gürültüsü (R)
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
        """CTRV hareket modeli"""
        px, py, pz, v, psi, psi_dot, vz = x

        if dt <= 0:
            return x

        if abs(psi_dot) > 0.001:
            px_new = px + (v / psi_dot) * (math.sin(psi + psi_dot * dt) - math.sin(psi))
            py_new = py + (v / psi_dot) * (math.cos(psi) - math.cos(psi + psi_dot * dt))
        else:
            px_new = px + v * math.cos(psi) * dt
            py_new = py + v * math.sin(psi) * dt

        pz_new = pz + vz * dt
        psi_new = psi + psi_dot * dt

        psi_new = (psi_new + math.pi) % (2 * math.pi) - math.pi

        return np.array([px_new, py_new, pz_new, v, psi_new, psi_dot, vz])

    def h(self, x):
        """Ölçüm modeli"""
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

        # Hız patlamasını frenle
        v_mag = abs(self.x[3])
        if v_mag > MAX_RIVAL_SPEED:
            self.x[3] = math.copysign(MAX_RIVAL_SPEED, self.x[3])

    def get_state(self):
        return self.x.copy()


# ==================================================
# RIVAL TRACKER - TEKNOFEST FULL PARSER
# ==================================================
class RivalTracker:
    def __init__(self, ref_lat, ref_lon, ref_h, scoring_weights=None):
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon
        self.ref_h = ref_h

        self.filters = {}
        self.ignored = set()
        self.latest_lag_s = {}
        self.last_update_time = {}
        self.history = {}  # tid -> deque(t, x, y, z, v, psi, vz)

        # Rakip tutumunu (pitch/roll) saklamak için
        self.rival_attitude = {}

        # Puanlama katsayıları
        self.weights = dict(DEFAULT_SCORING_WEIGHTS)
        if scoring_weights:
            self.weights.update(scoring_weights)

    def update_from_server_response(self, data):
        """
        Teknofest verilerini işler:
        - iha_enlem, iha_boylam, iha_irtifa -> konum
        - iha_hiz -> hız
        - iha_yonelme -> yaw
        - iha_dikilme, iha_yatis -> pitch, roll
        """
        now = time.time()

        for pkt in data:
            tid = pkt.get("takim_numarasi")
            if tid is None or tid == MY_TEAM_ID:
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

            # UKF
            if tid not in self.filters:
                self.filters[tid] = UnscentedKalmanFilter(x, y, z, speed_ms, yaw_rad)
                self.last_update_time[tid] = now
                self.history[tid] = deque(maxlen=50)
            else:
                dt = now - self.last_update_time.get(tid, now)
                if dt < 0.001:
                    dt = 0.001

                self.filters[tid].predict(dt)
                self.filters[tid].update([x, y, z, speed_ms, yaw_rad])

                self.last_update_time[tid] = now

            hist = self.history.setdefault(tid, deque(maxlen=50))
            if len(hist) >= 1:
                last_t, _, _, last_z, _, _, _ = hist[-1]
                dt_z = max(1e-3, now - last_t)
                vz_est = (z - last_z) / dt_z

            hist.append((now, x, y, z, speed_ms, yaw_rad, vz_est))

            self.latest_lag_s[tid] = delay_ms / 1000.0
            self.rival_attitude[tid] = {"pitch": pitch_deg, "roll": roll_deg}

    def ignore_rival(self, tid):
        self.ignored.add(tid)
        print(f"Rakip {tid} VURULDU (listeden çıkarıldı).")

    def report_lock_event(self, tid):
        """
        Görsel kilit tamamlandığında rakibi tek seferlik olarak ignore set'ine atar.
        Aynı rakibe ikinci kez vurulduysa False döner.
        """
        if tid in self.ignored:
            return False
        self.ignore_rival(tid)
        return True

    def get_closest_rival(self, my_x, my_y, my_z, my_heading_deg):
        now = time.time()
        best = None
        best_score = -float("inf")
        my_heading_deg = (my_heading_deg if my_heading_deg is not None else 0.0) % 360.0
        my_pos = (my_x, my_y, my_z)

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

            lag = self.latest_lag_s.get(tid, 0.1)
            dt_future = lag + (now - self.last_update_time[tid])
            px_f, py_f, pz_f, v_f = self._predict_future(tid, (px, py, pz, v, psi, vz), dt_future)

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
    # Yardımcılar
    # -------------------------------------------------------------
    def _predict_future(self, tid, state, dt_future: float):
        """
        Basit sabit hız + yön. İmkân varsa son iki hızdan ivme tahmin edip
        0.5*a*t^2 ile projeksiyon yapar; hız sınırı korunur.
        """
        px, py, pz, v, psi, vz = state
        hist = self.history.get(tid, None)

        # Varsayılan sabit hız (UKF state)
        vx, vy = v * math.cos(psi), v * math.sin(psi)
        ax = ay = az = 0.0

        if hist and len(hist) >= 2:
            t2, x2, y2, z2, _, _, _ = hist[-1]
            t1, x1, y1, z1, _, _, _ = hist[-2]
            dt = max(1e-3, t2 - t1)
            vx = (x2 - x1) / dt
            vy = (y2 - y1) / dt
            vz = (z2 - z1) / dt

            # İvme tahmini için bir adım daha geriye bak
            if len(hist) >= 3:
                t0, x0, y0, z0, _, _, _ = hist[-3]
                dt_prev = max(1e-3, t1 - t0)
                vx_prev = (x1 - x0) / dt_prev
                vy_prev = (y1 - y0) / dt_prev
                vz_prev = (z1 - z0) / dt_prev

                dt_vel = max(1e-3, t2 - t0)
                ax = (vx - vx_prev) / dt_vel
                ay = (vy - vy_prev) / dt_vel
                az = (vz - vz_prev) / dt_vel

        # İvmeli projeksiyon
        px_f = px + vx * dt_future + 0.5 * ax * dt_future * dt_future
        py_f = py + vy * dt_future + 0.5 * ay * dt_future * dt_future
        pz_f = pz + vz * dt_future + 0.5 * az * dt_future * dt_future

        vx_f = vx + ax * dt_future
        vy_f = vy + ay * dt_future
        vz_f = vz + az * dt_future

        speed_mag = math.sqrt(vx_f * vx_f + vy_f * vy_f + vz_f * vz_f)
        if speed_mag > MAX_RIVAL_SPEED:
            scale = MAX_RIVAL_SPEED / speed_mag
            vx_f, vy_f, vz_f = vx_f * scale, vy_f * scale, vz_f * scale

        v_f = math.sqrt(vx_f * vx_f + vy_f * vy_f + vz_f * vz_f)
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

    def _score_candidate(self, my_pos, my_heading_deg, px_f, py_f, pz_f, v_f, yaw_deg):
        """
        4.1.2 formüllerine göre puan hesaplar.
        Dönecek değer: 1 / (1 + (P_h + P_d + P_BV))
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


__all__ = ["RivalTracker", "MY_TEAM_ID"]
