import math
import numpy as np


class UKF_CA_Model:
    """
    Constant Acceleration (CA) model.
    State: [px, py, pz, vx, vy, vz, ax, ay, az, psi, psi_dot]
    """

    def __init__(self, kf_cfg=None):
        self.n = 11
        self.x = np.zeros((self.n, 1))

        self.P = np.eye(self.n) * 10.0
        self.P[6:9, 6:9] = np.eye(3) * 5.0

        self.Q = np.diag(
            [
                0.5,
                0.5,
                0.5,
                1.0,
                1.0,
                1.0,
                8.0,
                8.0,
                3.0,
                0.1,
                0.05,
            ]
        )

        self.R = np.diag(
            [
                3.0,  # px
                3.0,  # py
                5.0,  # pz
                0.8,  # acc_x
                0.8,  # acc_y
                0.8,  # acc_z
                1.5,  # ground speed
                math.radians(5.0),  # yaw
            ]
        )

        self.alpha = 1e-3
        self.beta = 2.0
        self.kappa = 0.0
        self.lambda_ = self.alpha**2 * (self.n + self.kappa) - self.n

        self.Wm = np.full(2 * self.n + 1, 1.0 / (2 * (self.n + self.lambda_)))
        self.Wc = np.full(2 * self.n + 1, 1.0 / (2 * (self.n + self.lambda_)))
        self.Wm[0] = self.lambda_ / (self.n + self.lambda_)
        self.Wc[0] = self.lambda_ / (self.n + self.lambda_) + (1 - self.alpha**2 + self.beta)

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
        sigmas[0] = self.x.flatten()
        for i in range(self.n):
            sigmas[i + 1] = self.x.flatten() + sqrt_P[i]
            sigmas[self.n + i + 1] = self.x.flatten() - sqrt_P[i]
        return sigmas

    def f(self, x, dt: float):
        px, py, pz, vx, vy, vz, ax, ay, az, psi, psi_dot = x
        if dt <= 0:
            return x

        px_new = px + vx * dt + 0.5 * ax * dt * dt
        py_new = py + vy * dt + 0.5 * ay * dt * dt
        pz_new = pz + vz * dt + 0.5 * az * dt * dt

        vx_new = vx + ax * dt
        vy_new = vy + ay * dt
        vz_new = vz + az * dt

        ax_new = ax * 0.95
        ay_new = ay * 0.95
        az_new = az * 0.95

        psi_new = psi + psi_dot * dt
        psi_new = (psi_new + math.pi) % (2 * math.pi) - math.pi

        return np.array(
            [
                px_new,
                py_new,
                pz_new,
                vx_new,
                vy_new,
                vz_new,
                ax_new,
                ay_new,
                az_new,
                psi_new,
                psi_dot,
            ]
        )

    def h(self, x):
        px, py, pz, vx, vy, vz, ax, ay, az, psi, psi_dot = x
        ground_speed = math.sqrt(vx * vx + vy * vy)
        cos_psi = math.cos(psi)
        sin_psi = math.sin(psi)
        acc_body_x = ax * cos_psi + ay * sin_psi
        acc_body_y = -ax * sin_psi + ay * cos_psi
        acc_body_z = az + 9.81
        return np.array([px, py, pz, acc_body_x, acc_body_y, acc_body_z, ground_speed, psi])

    def predict(self, dt):
        if dt <= 0:
            return
        sig = self._sigma_points()
        for i in range(2 * self.n + 1):
            sig[i] = self.f(sig[i], dt)

        x_pred = np.zeros(self.n)
        for i in range(2 * self.n + 1):
            x_pred += self.Wm[i] * sig[i]

        P_pred = np.zeros((self.n, self.n))
        for i in range(2 * self.n + 1):
            y = sig[i] - x_pred
            y[9] = (y[9] + math.pi) % (2 * math.pi) - math.pi
            P_pred += self.Wc[i] * np.outer(y, y)

        P_pred += self.Q
        self.x = x_pred.reshape(-1, 1)
        self.P = (P_pred + P_pred.T) / 2.0

    def update(self, z_measurement, dt, hdop):
        R_adaptive = self.R.copy()
        R_adaptive[0, 0] *= max(hdop, 1.0)
        R_adaptive[1, 1] *= max(hdop, 1.0)
        R_adaptive[2, 2] *= max(hdop, 1.0) * 1.5

        try:
            sig = self._sigma_points()
        except Exception:
            return

        Z_sig = np.zeros((2 * self.n + 1, 8))
        for i in range(2 * self.n + 1):
            Z_sig[i] = self.h(sig[i])

        z_pred = np.zeros(8)
        for i in range(2 * self.n + 1):
            z_pred += self.Wm[i] * Z_sig[i]

        S = np.zeros((8, 8))
        Pxz = np.zeros((self.n, 8))

        for i in range(2 * self.n + 1):
            dz = Z_sig[i] - z_pred
            dz[7] = (dz[7] + math.pi) % (2 * math.pi) - math.pi

            dx = sig[i] - self.x.flatten()
            dx[9] = (dx[9] + math.pi) % (2 * math.pi) - math.pi

            S += self.Wc[i] * np.outer(dz, dz)
            Pxz += self.Wc[i] * np.outer(dx, dz)

        S += R_adaptive

        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            S_inv = np.linalg.pinv(S)

        K = Pxz @ S_inv
        y = z_measurement - z_pred
        y[7] = (y[7] + math.pi) % (2 * math.pi) - math.pi

        self.x = self.x + (K @ y).reshape(-1, 1)
        self.P = self.P - K @ S @ K.T
        self.P = (self.P + self.P.T) / 2.0

        MAX_ACC = 15.0
        self.x[6] = np.clip(self.x[6], -MAX_ACC, MAX_ACC)
        self.x[7] = np.clip(self.x[7], -MAX_ACC, MAX_ACC)
        self.x[8] = np.clip(self.x[8], -MAX_ACC / 2, MAX_ACC / 2)

        MAX_SPEED = 70.0
        speed = math.sqrt(self.x[3] ** 2 + self.x[4] ** 2 + self.x[5] ** 2)
        if speed > MAX_SPEED:
            scale = MAX_SPEED / speed
            self.x[3] *= scale
            self.x[4] *= scale
            self.x[5] *= scale

    def get_velocity_3d(self):
        return self.x[3:6].flatten()

    def get_acceleration_3d(self):
        return self.x[6:9].flatten()

    def get_speed(self):
        vx, vy, vz = self.x[3:6].flatten()
        return math.sqrt(vx * vx + vy * vy + vz * vz)
#yedek 