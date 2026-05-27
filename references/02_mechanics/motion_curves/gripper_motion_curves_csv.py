import csv
import math
from pathlib import Path


A = (-12.0, 0.0)
B0 = (-13.0, 28.5)
C0 = (-47.99, 27.83)
L1 = 45.5
L2 = 35.0
STROKE = 16.0
NPTS = 1001

V_NUT = 1.0       # mm/s, positive downward
A_NUT = 0.0       # mm/s^2, positive downward


def circle_intersections(a, r0, b, r1):
    x0, y0 = a
    x1, y1 = b
    dx = x1 - x0
    dy = y1 - y0
    d = math.hypot(dx, dy)
    if d > r0 + r1 or d < abs(r0 - r1) or d == 0:
        raise ValueError("No circle intersection")

    along = (r0 * r0 - r1 * r1 + d * d) / (2 * d)
    h = math.sqrt(max(0.0, r0 * r0 - along * along))
    xm = x0 + along * dx / d
    ym = y0 + along * dy / d
    rx = -dy * h / d
    ry = dx * h / d
    return (xm + rx, ym + ry), (xm - rx, ym - ry)


def wrap_pi(x):
    while x > math.pi:
        x -= 2.0 * math.pi
    while x < -math.pi:
        x += 2.0 * math.pi
    return x


def gradient(y, x):
    n = len(y)
    out = [0.0] * n
    out[0] = (y[1] - y[0]) / (x[1] - x[0])
    out[-1] = (y[-1] - y[-2]) / (x[-1] - x[-2])
    for i in range(1, n - 1):
        out[i] = (y[i + 1] - y[i - 1]) / (x[i + 1] - x[i - 1])
    return out


def main():
    theta0 = math.atan2(C0[1] - B0[1], C0[0] - B0[0])
    s_values = [STROKE * i / (NPTS - 1) for i in range(NPTS)]
    phi = []
    c_values = []

    for s in s_values:
        b = (B0[0], B0[1] - s)
        p1, p2 = circle_intersections(A, L1, b, L2)
        c = p1 if p1[0] <= p2[0] else p2
        theta = wrap_pi(math.atan2(c[1] - b[1], c[0] - b[0]) - theta0)
        phi.append(-theta)
        c_values.append(c)

    j = gradient(phi, s_values)
    k = gradient(j, s_values)
    omega = [x * V_NUT for x in j]
    alpha = [k_i * V_NUT * V_NUT + j_i * A_NUT for k_i, j_i in zip(k, j)]

    out_path = Path(__file__).with_name("gripper_motion_curves.csv")
    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "nut_stroke_mm",
            "jaw_closing_angle_deg",
            "dphi_ds_rad_per_mm",
            "dphi_ds_deg_per_mm",
            "d2phi_ds2_rad_per_mm2",
            "d2phi_ds2_deg_per_mm2",
            "jaw_omega_rad_per_s",
            "jaw_omega_deg_per_s",
            "jaw_alpha_rad_per_s2",
            "jaw_alpha_deg_per_s2",
            "joint3_x_mm",
            "joint3_y_mm",
        ])
        for i, s in enumerate(s_values):
            writer.writerow([
                f"{s:.6f}",
                f"{math.degrees(phi[i]):.6f}",
                f"{j[i]:.9f}",
                f"{math.degrees(j[i]):.6f}",
                f"{k[i]:.9f}",
                f"{math.degrees(k[i]):.6f}",
                f"{omega[i]:.9f}",
                f"{math.degrees(omega[i]):.6f}",
                f"{alpha[i]:.9f}",
                f"{math.degrees(alpha[i]):.6f}",
                f"{c_values[i][0]:.6f}",
                f"{c_values[i][1]:.6f}",
            ])

    print(f"Saved data: {out_path}")
    print(f"Angle range: {math.degrees(phi[0]):.3f} to {math.degrees(phi[-1]):.3f} deg")
    print(f"dphi/ds range: {math.degrees(min(j)):.3f} to {math.degrees(max(j)):.3f} deg/mm")
    print(f"d2phi/ds2 range: {math.degrees(min(k)):.3f} to {math.degrees(max(k)):.3f} deg/mm^2")


if __name__ == "__main__":
    main()
