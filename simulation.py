import csv
import math
import numpy as np

from config import KFConfig, SimCfg
from geo_utils import bearing_deg, enu_to_llh, haversine_m, llh_to_enu
from quaternion_utils import angle_to_q, q_to_rot_matrix
from ukf import KF3D_UKF


def generate_truth(t_tuple, last_state):
    x_prev, y_prev, z_prev, vx_prev, vy_prev, vz_prev, ax_prev, ay_prev, az_prev = last_state
    tt = t_tuple[0]
    dt = t_tuple[1] - t_tuple[0]

    if tt < 60:
        ax_cmd, ay_cmd, az_cmd = 0.5, 0.0, 0.0
    elif tt < 120:
        ax_cmd, ay_cmd, az_cmd = 0.0, 0.3, 0.2
    elif tt < 200:
        ax_cmd, ay_cmd, az_cmd = -0.2, 0.0, 0.0
    else:
        ax_cmd, ay_cmd, az_cmd = 0.0, -0.2, -0.2

    alpha = 0.9
    ax_new = alpha * ax_prev + (1 - alpha) * ax_cmd
    ay_new = alpha * ay_prev + (1 - alpha) * ay_cmd
    az_new = alpha * az_prev + (1 - alpha) * az_cmd

    x_new = x_prev + vx_prev * dt + 0.5 * ax_new * dt * dt
    y_new = y_prev + vy_prev * dt + 0.5 * ay_new * dt * dt
    z_new = z_prev + vz_prev * dt + 0.5 * az_new * dt * dt

    vx_new = vx_prev + ax_new * dt
    vy_new = vy_prev + ay_new * dt
    vz_new = vz_prev + az_new * dt

    return np.array([x_new, y_new, z_new, vx_new, vy_new, vz_new, ax_new, ay_new, az_new], dtype=float)


def run_sim():
    sim = SimCfg()
    cfg = KFConfig()
    kf = KF3D_UKF(cfg)

    dt_imu = 1.0 / sim.fs_imu
    steps = int(sim.T * sim.fs_imu)
    t_axis = np.arange(0, steps) * dt_imu

    lat0, lon0, h0 = sim.lat0, sim.lon0, sim.h0

    state = np.array([0, 0, 0, 0, sim.v0, 0, 0, 0, 0], dtype=float)

    rows = []
    gps_timer = 0.0
    gps_period = 1.0 / sim.fs_gps

    kf_initial_enu = np.array([0.0, 0.0, 0.0])
    kf.initialize_from_pos(kf_initial_enu)

    tgt_enu = llh_to_enu(lat0, lon0, h0, sim.target_lat, sim.target_lon, sim.target_h)

    G_ENU = np.array([0.0, 0.0, -9.81])
    M_REF = np.array([20.0, 0.0, 45.0])

    roll_rad_truth, pitch_rad_truth, yaw_rad_truth = 0.0, 0.0, 0.0
    bias_w_truth = np.zeros(3)
    bias_a_truth = np.zeros(3)
    w_body_truth = np.zeros(3)

    HDOP_SIMULATED = 1.5

    for i, t in enumerate(t_axis):
        state = generate_truth((t, t + dt_imu), state)
        x_truth, y_truth, z_truth, vx_truth, vy_truth, vz_truth, ax_truth, ay_truth, az_truth = state
        a_ENU_truth = np.array([ax_truth, ay_truth, az_truth])

        pitch_rad_truth = math.radians(10 * math.sin(t * 0.1))
        roll_rad_truth = math.radians(5 * math.cos(t * 0.2))
        yaw_rate = math.radians(10 * math.cos(t * 0.05))
        yaw_rad_truth += yaw_rate * dt_imu

        w_body_truth = np.array([0.0, 0.0, yaw_rate])

        Q_TRUTH = angle_to_q(roll_rad_truth, pitch_rad_truth, yaw_rad_truth)
        R_BODY_TO_ENU = q_to_rot_matrix(Q_TRUTH)
        R_ENU_TO_BODY = R_BODY_TO_ENU.T

        a_BODY_hareket = R_ENU_TO_BODY @ a_ENU_truth
        G_BODY = R_ENU_TO_BODY @ (-G_ENU)

        z_acc_raw = a_BODY_hareket + G_BODY + bias_a_truth
        z_gyr_raw = w_body_truth + bias_w_truth

        acc_noise = np.random.normal(0.0, cfg.r_acc, size=3)
        gyr_noise = np.random.normal(0.0, cfg.r_imu, size=3)

        z_acc_noisy = z_acc_raw + acc_noise
        z_gyr_noisy = z_gyr_raw + gyr_noise

        z_imu_predict = np.concatenate((z_acc_noisy, z_gyr_noisy))

        kf.predict(dt_imu, z_imu_predict)

        gps_used = False
        if t >= gps_timer:
            gps_timer += gps_period

            gps_noise = np.array([
                np.random.normal(0.0, cfg.r_gps_h),
                np.random.normal(0.0, cfg.r_gps_h),
                np.random.normal(0.0, cfg.r_gps_v),
            ])
            z_pos = np.array([x_truth, y_truth, z_truth]) + gps_noise

            mag_noise = np.random.normal(0.0, cfg.r_mag, size=3)
            z_mag_raw = R_ENU_TO_BODY @ M_REF
            z_mag_noisy = z_mag_raw + mag_noise

            z_full = np.concatenate((z_pos, z_acc_noisy, z_mag_noisy))

            success = kf.update(z_full, dt_imu, HDOP_SIMULATED)

            if success:
                gps_used = True

        est_x, est_y, est_z = kf.x[0, 0], kf.x[1, 0], kf.x[2, 0]
        est_q = kf.x[3:7, 0]
        est_vx, est_vy, est_vz = kf.x[7, 0], kf.x[8, 0], kf.x[9, 0]
        est_bwx, est_bwy, est_bwz = kf.x[10, 0], kf.x[11, 0], kf.x[12, 0]
        est_bax, est_bay, est_baz = kf.x[13, 0], kf.x[14, 0], kf.x[15, 0]

        R_EST = q_to_rot_matrix(est_q)
        est_a_body_true = z_acc_noisy - kf.x[13:16, 0]
        est_ax_enu, est_ay_enu, est_az_enu = R_EST @ est_a_body_true - G_ENU

        est_lat, est_lon, est_h = enu_to_llh(lat0, lon0, h0, est_x, est_y, est_z)
        tgt_lat, tgt_lon, tgt_h = sim.target_lat, sim.target_lon, sim.target_h

        ground_dist = haversine_m(est_lat, est_lon, tgt_lat, tgt_lon)
        vert_dist = tgt_h - est_h
        slant_dist = math.hypot(ground_dist, vert_dist)
        brg = bearing_deg(est_lat, est_lon, tgt_lat, tgt_lon)

        rows.append([
            t,
            est_lat,
            est_lon,
            est_h,
            est_x,
            est_y,
            est_z,
            est_q[0],
            est_q[1],
            est_q[2],
            est_q[3],
            est_bwx,
            est_bwy,
            est_bwz,
            est_bax,
            est_bay,
            est_baz,
            est_vx,
            est_vy,
            est_vz,
            est_ax_enu,
            est_ay_enu,
            est_az_enu,
            ground_dist,
            vert_dist,
            slant_dist,
            brg,
            int(gps_used),
        ])

    csv_filename = "kf3d_track_log_ukf_robust_minimal_v3.csv"
    with open(csv_filename, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "t",
            "est_lat",
            "est_lon",
            "est_h",
            "est_x",
            "est_y",
            "est_z",
            "est_q0",
            "est_q1",
            "est_q2",
            "est_q3",
            "est_bwx",
            "est_bwy",
            "est_bwz",
            "est_bax",
            "est_bay",
            "est_baz",
            "est_vx",
            "est_vy",
            "est_vz",
            "est_ax",
            "est_ay",
            "est_az",
            "ground_dist_m",
            "vert_dist_m",
            "slant_dist_m",
            "bearing_deg",
            "gps_used",
        ])
        w.writerows(rows)

    print("=== UKF 3D ROBUST Q-SIM SUMMARY (V3 FIXED) ===")
    print(f"IMU steps: {len(t_axis)} (fs_imu={sim.fs_imu} Hz)")
    print(f"GPS rate:  {sim.fs_gps} Hz")
    print(f"CSV: {csv_filename}")
    print(f"Target ENU (m): {tgt_enu.round(1)}")
    print("Done.")


__all__ = ["generate_truth", "run_sim"]

