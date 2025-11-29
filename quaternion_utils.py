import math
import numpy as np


# ------------------------------------------
#  SAFE QUATERNION MULTIPLY  (MANTIK AYNI)
# ------------------------------------------
def quat_mult(q1, q2):
    q1 = np.asarray(q1, dtype=float)
    q2 = np.asarray(q2, dtype=float)

    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])


# ------------------------------------------
#  ROT MATRIX (NORMALIZATION EKLENDİ)
# ------------------------------------------
def q_to_rot_matrix(q):
    q = np.asarray(q, dtype=float)

    norm = np.linalg.norm(q)
    if norm < 1e-9:
        return np.eye(3)
    q = q / norm

    w, x, y, z = q

    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-w*z),     2*(x*z+w*y)],
        [2*(x*y+w*z),     1-2*(x*x+z*z),   2*(y*z-w*x)],
        [2*(x*z-w*y),     2*(y*z+w*x),     1-2*(x*x+y*y)]
    ])


# ------------------------------------------
#  EULER → QUATERNION  (Aynı mantık)
# ------------------------------------------
def angle_to_q(r, p, y):
    cr, sr = math.cos(r/2), math.sin(r/2)
    cp, sp = math.cos(p/2), math.sin(p/2)
    cy, sy = math.cos(y/2), math.sin(y/2)

    return np.array([
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy
    ])


# ------------------------------------------
#  q_log (EDGE CASE FIX)
# ------------------------------------------
def q_log(q):
    q = np.asarray(q, dtype=float)
    w = q[0]
    v = q[1:]
    vn = np.linalg.norm(v)

    if vn < 1e-12:
        return np.zeros(3)

    angle = math.atan2(vn, w)
    return v * (2 * angle / vn)


# ------------------------------------------
#  q_exp (EDGE CASE FIX)
# ------------------------------------------
def q_exp(v):
    v = np.asarray(v, dtype=float)
    vn = np.linalg.norm(v)

    if vn < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])

    half = vn / 2.0
    return np.array([
        math.cos(half),
        *(v / vn * math.sin(half))
    ])


# ------------------------------------------
#  WEIGHTED QUATERNION AVERAGE (STABLE)
# ------------------------------------------
def q_average(Q, W):
    qa = Q[3:7, 0].astype(float)
    norm = np.linalg.norm(qa)
    if norm > 1e-9:
        qa /= norm
    else:
        qa[:] = np.array([1,0,0,0])

    for _ in range(5):  # biraz artırıldı (daha kararlı)
        err = np.zeros(3)
        w, x, y, z = qa
        qa_conj = np.array([w, -x, -y, -z])

        for i in range(Q.shape[1]):
            qe = quat_mult(Q[3:7, i], qa_conj)
            err += W[i] * q_log(qe)

        qa = quat_mult(q_exp(err), qa)
        norm = np.linalg.norm(qa)
        if norm < 1e-9:
            return np.array([1,0,0,0])
        qa /= norm

    return qa


# ------------------------------------------
#  QUATERNION → SAFE EULER
# ------------------------------------------
def q_to_euler_bounded(q):
    q = np.asarray(q, dtype=float)

    norm = np.linalg.norm(q)
    if norm < 1e-9:
        return 0.0, 0.0, 0.0
    q = q / norm

    w, x, y, z = q

    # Roll
    roll = math.atan2(
        2*(w*x + y*z),
        1 - 2*(x*x + y*y)
    )

    # Pitch
    sinp = 2*(w*y - z*x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi/2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw
    yaw = math.atan2(
        2*(w*z + x*y),
        1 - 2*(y*y + z*z)
    )

    r_deg = max(-90, min(90, math.degrees(roll)))
    p_deg = max(-90, min(90, math.degrees(pitch)))
    y_deg = math.degrees(yaw) % 360.0

    return r_deg, p_deg, y_deg
