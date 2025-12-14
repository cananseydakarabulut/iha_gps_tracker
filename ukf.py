import math
import numpy as np
from numpy.linalg import cholesky, inv, LinAlgError, pinv

from config import KFConfig
from quaternion_utils import (
    angle_to_q,
    q_average,
    q_exp,
    q_log,
    q_to_rot_matrix,
    quat_mult,
)

# ==============================================================================
# 1. FZKSEL HAREKET MODEL (FX) - (HIZ PATLAMASI DZELTLD)
# ==============================================================================
def fx(x, dt, u, g_enu=np.array([0, 0, -9.81])):
    """
    Durum gei fonksiyonu.
    """
    x_new = x.copy()

    # IMU Verilerini Ayr
    a_body_meas = u[0:3]
    w_body_meas = u[3:6]

    # Biaslar kar
    b_w = x[10:13]
    b_a = x[13:16]

    w_true = w_body_meas - b_w
    a_body_true = a_body_meas - b_a

    # --- 1. Quaternion (A) Gncellemesi ---
    q_old = x[3:7]
    omega_quat = np.concatenate(([0.0], w_true))

    q_dot = 0.5 * quat_mult(q_old, omega_quat)
    q_new = q_old + q_dot * dt

    norm_q = np.linalg.norm(q_new)
    if norm_q < 1e-9:
        q_new = angle_to_q(0, 0, 0)
    else:
        q_new /= norm_q

    x_new[3:7] = q_new

    # --- 2. Hz ve Konum Gncellemesi ---
    R_BODY_TO_ENU = q_to_rot_matrix(q_new)
    a_ENU = R_BODY_TO_ENU @ a_body_true - g_enu

    #  PATCH 1: vme Limitleme (Senin kodundaki mantk aynen duruyor)
    a_ENU = np.clip(a_ENU, -30.0, 30.0)

    # Kk ivmeyi yok say (deadband), gereksiz hz art olmasn
    # 0.5 -> 0.2 m/s² (daha hassas, düşük manevralara tepki verir)
    a_norm = np.linalg.norm(a_ENU)
    if a_norm < 0.2:
        a_ENU = np.zeros(3)

    v_old = x[7:10]
    p_old = x[0:3]

    # Hz modeli: ivme varsa ekle, yoksa sabit hz + hafif snm
    dv = a_ENU * dt
    dv_norm = np.linalg.norm(dv)
    max_dv = 8.0  # tek admda izin verilen en byk hz deiimi (m/s)

    # 0.5 -> 0.1 m/s (daha hassas hız değişimi algılama)
    if dv_norm < 0.1:
        dv = np.zeros(3)
    elif dv_norm > max_dv:
        dv = dv * (max_dv / dv_norm)

    vel_decay = 0.995  # hafif drag benzeri etki
    v_new = v_old * vel_decay + dv

    # Konum hesabnda ivmeyi kullanmaya devam edebiliriz (ksa anlk tepki iin)
    p_new = p_old + v_old * dt + 0.5 * a_ENU * dt**2

    x_new[0:3] = p_new
    x_new[7:10] = v_new

    return x_new


# ==============================================================================
# 2. LM MODEL (HX) - (HIZ BYKL EKLEND)
# ==============================================================================
def hx(x, g_enu=np.array([0, 0, -9.81]), m_ref=np.array([20.0, 0.0, 45.0]), a_enu_est=np.zeros(3)):
    """
    lm fonksiyonu.
    a_enu_est: Tahmin edilen ENU frame ivmesi (fx'ten geliyor)
    """

    z_gps = x[0:3]
    q = x[3:7]
    b_a = x[13:16]
    v_enu = x[7:10]  # 3D hız vektörü

    R_BODY_TO_ENU = q_to_rot_matrix(q)
    R_ENU_TO_BODY = R_BODY_TO_ENU.T

    # Gerçek ivme tahmini (artık sıfır değil!)
    a_real = a_enu_est

    # Yeni ivme modeli
    z_acc = R_ENU_TO_BODY @ (a_real - g_enu) + b_a

    # Manyetometre tahmini
    z_mag = R_ENU_TO_BODY @ m_ref

    # 3D hız vektörü (vx, vy, vz)
    # Airspeed tahmini (body frame'deki forward hız büyüklüğü)
    v_body = R_ENU_TO_BODY @ v_enu
    airspeed = math.sqrt(v_body[0]**2 + v_body[1]**2 + v_body[2]**2)

    return np.concatenate((z_gps, z_acc, z_mag, v_enu, [airspeed]))


# ==============================================================================
# 3. UNSCENTED KALMAN FILTER (UKF) SINIFI - (YAPISI AYNEN KORUNDU)
# ==============================================================================
class KF3D_UKF:
    def __init__(self, cfg: KFConfig):
        self.cfg = cfg

        self.x = np.zeros((16, 1))

        self.L_err = 15
        self.M = 13  # pos(3) + acc(3) + mag(3) + vel(3) + airspeed(1)

        self.P = np.eye(self.L_err) * 10.0
        self.initialized = False

        # Son tahmin edilen ENU frame ivmesi (hx için)
        self.last_accel_enu = np.zeros(3)

        self.alpha = 3e-3
        self.beta = 2.0
        self.kappa = 0.0

        self.lambda_ = self.alpha**2 * (self.L_err + self.kappa) - self.L_err
        self.gamma = math.sqrt(self.L_err + self.lambda_)

        self.Wm = np.full(2*self.L_err+1, 1.0/(2*(self.L_err+self.lambda_)))
        self.Wc = self.Wm.copy()
        self.Wm[0] = self.lambda_/(self.L_err+self.lambda_)
        self.Wc[0] = self.lambda_/(self.L_err+self.lambda_) + (1-self.alpha**2+self.beta)


 
    def initialize_from_pos(self, pos_enu, v_init=None):
        """
        UKF Balatc.
        v_init: Eer elinde balang hz varsa (np.array([vx, vy, vz])) buraya ver.
                Yoksa None ver, P matrisi sayesinde otomatik bulacak.
        """
        self.x = np.zeros((16, 1))
        
        # 1. Konumu ayarla
        self.x[0:3, 0] = pos_enu
        
        # 2. Ay sfrla (veya elinde varsa onu da ver)
        self.x[3:7, 0] = angle_to_q(0, 0, 0) 

        # 3. HIZ AYARI (HIZLI TEPK N)
        if v_init is not None:
            # Eer elinde yaklak hz varsa (rn: 33 m/s), direkt onu ver
            self.x[7:10, 0] = v_init
        else:
            # Hz yoksa 0 balat
            self.x[7:10, 0] = 0

        self.x[10:13, 0] = 0 # Biaslar
        self.x[13:16, 0] = 0 # Biaslar

        # =====================================================
        #  SHRL DOKUNU: BELRSZLK MATRS (P) AYARI
        # =====================================================
        self.P = np.eye(self.L_err) * 1.0
        
        # Hz Belirsizliini (Variance) OK YKSEK yapyoruz (1000.0).
        # Bu sayede UKF, ilk lmde "hzm 0'm" demez, GPS ne diyorsa ona atlar.
        self.P[6, 6] = 1000.0  # Vx belirsizlii
        self.P[7, 7] = 1000.0  # Vy belirsizlii
        self.P[8, 8] = 1000.0  # Vz belirsizlii

        self.initialized = True
        print(f" UKF Balatld (Hzl Adaptasyon Modu). Konum: {pos_enu}")

    # ----------------------------------------------------
    def _Q(self, dt):
        Q = np.zeros((self.L_err, self.L_err))

        Q[9:12, 9:12] = np.eye(3) * self.cfg.q_omega_bias * dt
        Q[12:15, 12:15] = np.eye(3) * self.cfg.q_acc_bias * dt

        q_p = self.cfg.q_jerk * dt**3 / 3.0
        q_v = self.cfg.q_jerk * dt

        Q[0:3, 0:3] = np.eye(3) * q_p
        Q[6:9, 6:9] = np.eye(3) * q_v

        return Q


    # ----------------------------------------------------
    def _sigma_points(self):
        self.P = (self.P + self.P.T) / 2.0

        jitter = 1e-6
        for _ in range(3):
            try:
                P_sqrt = cholesky(self.P + np.eye(self.L_err)*jitter)
                break
            except LinAlgError:
                jitter *= 10
        else:
            P_sqrt = np.eye(self.L_err) * 1e-3

        P_term = self.gamma * P_sqrt

        X_sigma = np.zeros((16, 2*self.L_err+1))
        X_sigma[:,0] = self.x[:,0]

        for i in range(self.L_err):
            e = P_term[:,i]

            xp = self.x[:,0].copy()
            xp[0:3] += e[0:3]
            xp[7:10] += e[6:9]
            xp[10:13] += e[9:12]
            xp[13:16] += e[12:15]

            q_err_p = q_exp(e[3:6])
            xp[3:7] = quat_mult(q_err_p, xp[3:7])
            xp[3:7] /= np.linalg.norm(xp[3:7])
            X_sigma[:,i+1] = xp

            xm = self.x[:,0].copy()
            xm[0:3] -= e[0:3]
            xm[7:10] -= e[6:9]
            xm[10:13] -= e[9:12]
            xm[13:16] -= e[12:15]

            q_err_m = q_exp(-e[3:6])
            xm[3:7] = quat_mult(q_err_m, xm[3:7])
            xm[3:7] /= np.linalg.norm(xm[3:7])
            X_sigma[:,i+1+self.L_err] = xm

        return X_sigma


    # ----------------------------------------------------
    def predict(self, dt, z_imu_raw):
        if not self.initialized:
            return

        u = z_imu_raw.reshape(6)
        Q = self._Q(dt)

        try:
            X_sigma = self._sigma_points()
        except:
            return

        X_pred = np.zeros_like(X_sigma)
        for i in range(X_sigma.shape[1]):
            X_pred[:,i] = fx(X_sigma[:,i], dt, u)

        x_new = (X_pred @ self.Wm).reshape(16,1)
        x_new[3:7,0] = q_average(X_pred, self.Wm)

        #  PATCH 3: Hz Limitleme - Gerçekçi limitler
        speed_mag = np.linalg.norm(x_new[7:10])
        MAX_SPEED = 35.0  # Gerçekçi İHA maksimum hızı (m/s)
        if speed_mag > MAX_SPEED:
            x_new[7:10] *= (MAX_SPEED / speed_mag)

        self.x = x_new

        P_pred = Q.copy()
        for i in range(X_pred.shape[1]):
            dx = np.zeros(self.L_err)
            dx[0:3] = X_pred[0:3,i] - self.x[0:3,0]
            dx[6:9] = X_pred[7:10,i] - self.x[7:10,0]
            dx[9:12] = X_pred[10:13,i] - self.x[10:13,0]
            dx[12:15] = X_pred[13:16,i] - self.x[13:16,0]

            q_err = quat_mult(
                X_pred[3:7,i],
                np.array([self.x[3,0], -self.x[4,0],
                          -self.x[5,0], -self.x[6,0]])
            )
            dx[3:6] = q_log(q_err)

            P_pred += self.Wc[i] * np.outer(dx, dx)

        self.P = (P_pred + P_pred.T)/2.0
        self.x_pred = self.x.copy()
        self.P_pred = self.P.copy()

        # İvmeyi IMU ölçümünden hesapla (hx için)
        a_body = u[0:3]
        b_a = self.x[13:16, 0]
        a_body_corrected = a_body - b_a
        q = self.x[3:7, 0]
        R_body_to_enu = q_to_rot_matrix(q)
        self.last_accel_enu = R_body_to_enu @ a_body_corrected - np.array([0, 0, -9.81])


    # ----------------------------------------------------
    def update(self, z_full, dt, hdop_simulated):
        if not self.initialized:
            return False

        z = z_full.reshape(self.M,1)
        self.x = self.x_pred.copy()
        self.P = self.P_pred.copy()

        try:
            X_sigma = self._sigma_points()
        except:
            return False

        Z_sigma = np.zeros((self.M, X_sigma.shape[1]))
        for i in range(X_sigma.shape[1]):
            Z_sigma[:,i] = hx(X_sigma[:,i], a_enu_est=self.last_accel_enu)

        z_hat = (Z_sigma @ self.Wm).reshape(self.M,1)
        Z_diff = Z_sigma - z_hat

        hdop = max(1.0, hdop_simulated)
        R_mat = np.diag([
            (self.cfg.r_gps_h*hdop)**2,      # px
            (self.cfg.r_gps_h*hdop)**2,      # py
            (self.cfg.r_gps_v*hdop)**2,      # pz
            self.cfg.r_acc**2,                # ax
            self.cfg.r_acc**2,                # ay
            self.cfg.r_acc**2,                # az
            self.cfg.r_mag**2,                # mx
            self.cfg.r_mag**2,                # my
            self.cfg.r_mag**2,                # mz
            (self.cfg.r_speed)**2,            # vx (GPS hızından)
            (self.cfg.r_speed)**2,            # vy (GPS hızından)
            (self.cfg.r_speed * 2.0)**2,      # vz (altitude türevinden - biraz daha gürültülü)
            0.3**2                            # airspeed (pitot tube gürültüsü ~0.3 m/s)
        ])

        S = R_mat.copy()
        for i in range(Z_sigma.shape[1]):
            S += self.Wc[i] * np.outer(Z_diff[:,i], Z_diff[:,i])

        S = (S + S.T)/2.0

        try:
            S_inv = inv(S)
        except:
            S_inv = pinv(S)

        y = z - z_hat
        d2 = float(y.T @ S_inv @ y)
        if d2 > self.cfg.chi2_threshold:
            # Aykırı ölçüm: yeniliği küçültüp yine de güncelle
            scale = math.sqrt(self.cfg.chi2_threshold / d2)
            y *= scale

        P_xz = np.zeros((self.L_err,self.M))
        for i in range(X_sigma.shape[1]):
            dx = np.zeros(self.L_err)

            dx[0:3] = X_sigma[0:3,i] - self.x[0:3,0]
            dx[6:9] = X_sigma[7:10,i] - self.x[7:10,0]
            dx[9:12] = X_sigma[10:13,i] - self.x[10:13,0]
            dx[12:15] = X_sigma[13:16,i] - self.x[13:16,0]

            q_err = quat_mult(
                X_sigma[3:7,i],
                np.array([self.x[3,0], -self.x[4,0],
                          -self.x[5,0], -self.x[6,0]])
            )
            dx[3:6] = q_log(q_err)

            P_xz += self.Wc[i] * np.outer(dx, Z_diff[:,i])

        K = P_xz @ S_inv
        delta = K @ y
        
        #  PATCH 2: Delta (Deiim) Hzn Krpma (Aynen korundu)
        delta[6:9] = np.clip(delta[6:9], -5.0, 5.0)

        self.x[0:3] += delta[0:3]
        self.x[7:10] += delta[6:9]
        self.x[10:13] += delta[9:12]
        self.x[13:16] += delta[12:15]

        q_delta = q_exp(delta[3:6,0])
        self.x[3:7,0] = quat_mult(q_delta, self.x[3:7,0])

        nq = np.linalg.norm(self.x[3:7,0])
        if nq < 1e-9:
            self.x[3:7,0] = angle_to_q(0,0,0)
        else:
            self.x[3:7,0] /= nq

        self.P = self.P - K @ S @ K.T
        self.P = (self.P + self.P.T)/2.0
        self.P += np.eye(self.L_err) * 1e-9

        # Hz limiti (lm gncellemesinden sonra da uygula)
        speed_mag = np.linalg.norm(self.x[7:10])
        MAX_SPEED = 35.0  # Gerçekçi İHA maksimum hızı
        if speed_mag > MAX_SPEED:
            self.x[7:10] *= (MAX_SPEED / speed_mag)

        return True

    # Basit getter'lar (GPS kodu icin)
    def get_velocity_3d(self):
        """Return velocity vector (vx, vy, vz) in ENU."""
        return float(self.x[7]), float(self.x[8]), float(self.x[9])

    def get_speed(self):
        """Return scalar speed magnitude."""
        v = self.get_velocity_3d()
        return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


__all__ = ["KF3D_UKF", "fx", "hx"]
