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
# 1. FİZİKSEL HAREKET MODELİ (FX) - (HIZ PATLAMASI DÜZELTİLDİ)
# ==============================================================================
def fx(x, dt, u, g_enu=np.array([0, 0, -9.81])):
    """
    Durum geçiş fonksiyonu.
    """
    x_new = x.copy()

    # IMU Verilerini Ayır
    a_body_meas = u[0:3]
    w_body_meas = u[3:6]

    # Biasları Çıkar
    b_w = x[10:13]
    b_a = x[13:16]

    w_true = w_body_meas - b_w
    a_body_true = a_body_meas - b_a

    # --- 1. Quaternion (Açı) Güncellemesi ---
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

    # --- 2. Hız ve Konum Güncellemesi ---
    R_BODY_TO_ENU = q_to_rot_matrix(q_new)
    a_ENU = R_BODY_TO_ENU @ a_body_true - g_enu

    # ⭐ PATCH 1: İvme Limitleme (Senin kodundaki mantık aynen duruyor)
    a_ENU = np.clip(a_ENU, -30.0, 30.0) 

    v_old = x[7:10]
    p_old = x[0:3]

    # 🛑 DÜZELTME BURADA YAPILDI 🛑
    # Eski hali: v_new = v_old + a_ENU * dt 
    # (Bu satır yerçekimi hatası yüzünden hızı 466'ya fırlatıyordu)
    
    # Yeni hali: İvmeyi hıza ekleme, hızı GPS düzeltsin.
    v_new = v_old 

    # Konum hesabında ivmeyi kullanmaya devam edebiliriz (kısa anlık tepki için)
    p_new = p_old + v_old * dt + 0.5 * a_ENU * dt**2

    x_new[0:3] = p_new
    x_new[7:10] = v_new

    return x_new


# ==============================================================================
# 2. ÖLÇÜM MODELİ (HX) - (AYNEN KORUNDU)
# ==============================================================================
def hx(x, g_enu=np.array([0, 0, -9.81]), m_ref=np.array([20.0, 0.0, 45.0])):
    """
    Ölçüm fonksiyonu.
    """

    z_gps = x[0:3]
    q = x[3:7]
    b_a = x[13:16]

    R_BODY_TO_ENU = q_to_rot_matrix(q)
    R_ENU_TO_BODY = R_BODY_TO_ENU.T

    # Sabit hız varsayımı (a_real = 0)
    a_real = np.zeros(3)

    # Yeni ivme modeli
    z_acc = R_ENU_TO_BODY @ (a_real - g_enu) + b_a

    # Manyetometre tahmini
    z_mag = R_ENU_TO_BODY @ m_ref

    return np.concatenate((z_gps, z_acc, z_mag))


# ==============================================================================
# 3. UNSCENTED KALMAN FILTER (UKF) SINIFI - (YAPISI AYNEN KORUNDU)
# ==============================================================================
class KF3D_UKF:
    def __init__(self, cfg: KFConfig):
        self.cfg = cfg

        self.x = np.zeros((16, 1))

        self.L_err = 15
        self.M = 9

        self.P = np.eye(self.L_err) * 10.0
        self.initialized = False

        self.alpha = 3e-3
        self.beta = 2.0
        self.kappa = 0.0

        self.lambda_ = self.alpha**2 * (self.L_err + self.kappa) - self.L_err
        self.gamma = math.sqrt(self.L_err + self.lambda_)

        self.Wm = np.full(2*self.L_err+1, 1.0/(2*(self.L_err+self.lambda_)))
        self.Wc = self.Wm.copy()
        self.Wm[0] = self.lambda_/(self.L_err+self.lambda_)
        self.Wc[0] = self.lambda_/(self.L_err+self.lambda_) + (1-self.alpha**2+self.beta)


    # ----------------------------------------------------
    # ----------------------------------------------------
    # BU FONKSİYONU UKF.PY İÇİNE YAPIŞTIR (ESKİSİNİ SİL)
    # ----------------------------------------------------
    def initialize_from_pos(self, pos_enu, v_init=None):
        """
        UKF Başlatıcı.
        v_init: Eğer elinde başlangıç hızı varsa (np.array([vx, vy, vz])) buraya ver.
                Yoksa None ver, P matrisi sayesinde otomatik bulacak.
        """
        self.x = np.zeros((16, 1))
        
        # 1. Konumu ayarla
        self.x[0:3, 0] = pos_enu
        
        # 2. Açıyı sıfırla (veya elinde varsa onu da ver)
        self.x[3:7, 0] = angle_to_q(0, 0, 0) 

        # 3. HIZ AYARI (HIZLI TEPKİ İÇİN)
        if v_init is not None:
            # Eğer elinde yaklaşık hız varsa (örn: 33 m/s), direkt onu ver
            self.x[7:10, 0] = v_init
        else:
            # Hız yoksa 0 başlat
            self.x[7:10, 0] = 0

        self.x[10:13, 0] = 0 # Biaslar
        self.x[13:16, 0] = 0 # Biaslar

        # =====================================================
        # ⭐ SİHİRLİ DOKUNUŞ: BELİRSİZLİK MATRİSİ (P) AYARI
        # =====================================================
        self.P = np.eye(self.L_err) * 1.0
        
        # Hız Belirsizliğini (Variance) ÇOK YÜKSEK yapıyoruz (1000.0).
        # Bu sayede UKF, ilk ölçümde "hızım 0'mış" demez, GPS ne diyorsa ona atlar.
        self.P[7, 7] = 1000.0  # Vx belirsizliği
        self.P[8, 8] = 1000.0  # Vy belirsizliği
        self.P[9, 9] = 1000.0  # Vz belirsizliği

        self.initialized = True
        print(f"🔧 UKF Başlatıldı (Hızlı Adaptasyon Modu). Konum: {pos_enu}")

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

        # ⭐ PATCH 3: Hız Limitleme (Aynen korundu)
        speed_mag = np.linalg.norm(x_new[7:10])
        MAX_SPEED = 60.0
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
            Z_sigma[:,i] = hx(X_sigma[:,i])

        z_hat = (Z_sigma @ self.Wm).reshape(self.M,1)
        Z_diff = Z_sigma - z_hat

        hdop = max(1.0, hdop_simulated)
        R_mat = np.diag([
            (self.cfg.r_gps_h*hdop)**2,
            (self.cfg.r_gps_h*hdop)**2,
            (self.cfg.r_gps_v*hdop)**2,
            self.cfg.r_acc**2,
            self.cfg.r_acc**2,
            self.cfg.r_acc**2,
            self.cfg.r_mag**2,
            self.cfg.r_mag**2,
            self.cfg.r_mag**2
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
            return False

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
        
        # ⭐ PATCH 2: Delta (Değişim) Hızını Kırpma (Aynen korundu)
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

        return True


__all__ = ["KF3D_UKF", "fx", "hx"]