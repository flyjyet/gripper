import math


# Units: mm, N, N*mm, MPa.


GEOM = {
    "A": (-12.0, 0.0),        # fixed joint 1, left side
    "B0": (-13.0, 28.5),      # nut/jaw joint 2 at fully open
    "C0": (-47.99, 27.83),    # link joint 3 at fully open
    "O0": (18.34, 59.78),     # R60 center on left jaw rigid body
    "L1": 45.5,               # A-C
    "L2": 35.0,               # B-C
    "jaw_radius": 60.0,
    "support_center": (0.0, 62.0),
    "support_radius": 15.0,
    "stroke": 16.0,
}

SCREW = {
    "lead": 2.0,              # mm/rev
    "thread_half_angle_deg": 15.0,
    "rated_torque_Nm": 3.5,
    "peak_torque_Nm": 12.5,
}

PIN_ASSUMPTION = {
    "pin_diameter": 3.0,
    "bearing_width": 20.0,
    "shear_planes": 2,
    "allow_shear": 80.0,
    "allow_bearing": 120.0,
}

TARGET = {
    "normal_per_side": 200.0,
    "link1_bearing_width": 5.0,
    "other_bearing_width": 20.0,
    "pin_diameter": 3.0,
}


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def mul(k, v):
    return (k * v[0], k * v[1])


def norm(v):
    return math.hypot(v[0], v[1])


def unit(v):
    n = norm(v)
    return (v[0] / n, v[1] / n)


def cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def circle_intersections(a, r0, b, r1):
    x0, y0 = a
    x1, y1 = b
    dx = x1 - x0
    dy = y1 - y0
    d = math.hypot(dx, dy)
    if d > r0 + r1 or d < abs(r0 - r1) or d == 0:
        return []

    along = (r0 * r0 - r1 * r1 + d * d) / (2 * d)
    h_sq = r0 * r0 - along * along
    h = math.sqrt(max(0.0, h_sq))
    xm = x0 + along * dx / d
    ym = y0 + along * dy / d
    rx = -dy * h / d
    ry = dx * h / d
    return [(xm + rx, ym + ry), (xm - rx, ym - ry)]


def wrap_pi(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def state(s):
    a = GEOM["A"]
    b0 = GEOM["B0"]
    c0 = GEOM["C0"]
    o0 = GEOM["O0"]
    l1 = GEOM["L1"]
    l2 = GEOM["L2"]

    b = (b0[0], b0[1] - s)
    pts = circle_intersections(a, l1, b, l2)
    if not pts:
        raise ValueError(f"No linkage solution at s={s}")

    c = min(pts, key=lambda p: (p[0] - c0[0]) ** 2 + (p[1] - c0[1]) ** 2)

    theta0 = math.atan2(c0[1] - b0[1], c0[0] - b0[0])
    theta = math.atan2(c[1] - b[1], c[0] - b[0])
    rot = wrap_pi(theta - theta0)
    co = math.cos(rot)
    si = math.sin(rot)

    bo0 = (o0[0] - b0[0], o0[1] - b0[1])
    o = (b[0] + co * bo0[0] - si * bo0[1],
         b[1] + si * bo0[0] + co * bo0[1])

    theta1 = math.atan2(c[1] - a[1], c[0] - a[0])
    return {"s": s, "A": a, "B": b, "C": c, "O": o, "jaw_rot": rot, "link1_angle": theta1}


def cable_center(diameter):
    r = diameter / 2.0
    support = GEOM["support_center"]
    # Concave support: cable center is above the lower arc by (r - support_radius).
    return (0.0, support[1] - GEOM["support_radius"] + r)


def contact_residual(s, diameter):
    r = diameter / 2.0
    o = state(s)["O"]
    p = cable_center(diameter)
    return norm(sub(o, p)) - (GEOM["jaw_radius"] - r)


def solve_contact(diameter):
    lo = 0.0
    hi = GEOM["stroke"]
    flo = contact_residual(lo, diameter)
    fhi = contact_residual(hi, diameter)
    if flo == 0:
        return lo
    if flo * fhi > 0:
        raise ValueError(f"No contact inside stroke for D={diameter}, f0={flo}, f1={fhi}")

    for _ in range(80):
        mid = (lo + hi) / 2
        fm = contact_residual(mid, diameter)
        if flo * fm <= 0:
            hi = mid
            fhi = fm
        else:
            lo = mid
            flo = fm
    return (lo + hi) / 2


def closure_rate(s, diameter):
    h = 1e-4
    return (contact_residual(s + h, diameter) - contact_residual(s - h, diameter)) / (2 * h)


def force_state(diameter, screw_force=1000.0):
    """Return one-side force state at first contact.

    screw_force is the total vertical screw/nut force shared by the two symmetric sides.
    """
    s = solve_contact(diameter)
    st = state(s)
    r = diameter / 2.0
    p = cable_center(diameter)
    o = st["O"]
    normal_dir = unit(sub(o, p))
    drds = closure_rate(s, diameter)

    # Virtual work: Fs*ds = 2*N*dr, so one-side cable normal force is:
    normal = screw_force / (2 * drds)

    # Rigid-body balance of the jaw about B. Link 1 is a two-force member, so
    # the force at C is along C->A.
    b = st["B"]
    c = st["C"]
    a = st["A"]
    contact_point = add(p, mul(-r, normal_dir))
    force_on_jaw_at_contact = mul(-normal, normal_dir)
    c_force_dir = unit(sub(a, c))
    h = -cross(sub(contact_point, b), force_on_jaw_at_contact) / cross(sub(c, b), c_force_dir)
    force_c = mul(h, c_force_dir)
    force_b = mul(-1.0, add(force_c, force_on_jaw_at_contact))

    return {
        "D": diameter,
        "s": s,
        "remaining": GEOM["stroke"] - s,
        "jaw_rot_deg": math.degrees(st["jaw_rot"]),
        "O": o,
        "cable_center": p,
        "closure_rate": drds,
        "normal_per_side": normal,
        "normal_ratio_per_side": normal / screw_force,
        "joint_A_load": abs(h),
        "joint_C_load": abs(h),
        "joint_B_load": norm(force_b),
        "joint_B_reaction": force_b,
    }


def screw_efficiency(mean_diameter, mu):
    lead = SCREW["lead"]
    alpha = math.radians(SCREW["thread_half_angle_deg"])
    lam = math.atan(lead / (math.pi * mean_diameter))
    mu_eq = mu / math.cos(alpha)
    eta = math.tan(lam) * (1 - mu_eq * math.tan(lam)) / (math.tan(lam) + mu_eq)
    self_lock = mu_eq > math.tan(lam)
    return {
        "mean_diameter": mean_diameter,
        "mu": mu,
        "lead_angle_deg": math.degrees(lam),
        "eta": eta,
        "self_lock": self_lock,
        "self_lock_margin": mu_eq / math.tan(lam),
    }


def screw_thrust(torque_nm, eta):
    # F = 2*pi*T*eta / lead
    return 2 * math.pi * torque_nm * 1000.0 * eta / SCREW["lead"]


def screw_torque(thrust, eta):
    # T = F*lead / (2*pi*eta)
    return thrust * SCREW["lead"] / (2 * math.pi * eta) / 1000.0


def pin_check(load, pin=None):
    p = dict(PIN_ASSUMPTION)
    if pin:
        p.update(pin)
    d = p["pin_diameter"]
    b = p["bearing_width"]
    n = p["shear_planes"]
    tau = load / (n * math.pi * d * d / 4.0)
    bearing = load / (d * b)
    req_d_shear = math.sqrt(4.0 * load / (n * math.pi * p["allow_shear"]))
    req_db = load / p["allow_bearing"]
    req_width = req_db / d
    return {
        "load": load,
        "tau": tau,
        "bearing": bearing,
        "req_d_shear": req_d_shear,
        "req_db": req_db,
        "req_width": req_width,
    }


def target_clamp_state(diameter, target_normal=None):
    target_normal = target_normal or TARGET["normal_per_side"]
    unit_force = force_state(diameter, 1000.0)
    screw_force = target_normal / unit_force["normal_ratio_per_side"]
    scale = screw_force / 1000.0
    return {
        "D": diameter,
        "s": unit_force["s"],
        "remaining": unit_force["remaining"],
        "ratio": unit_force["normal_ratio_per_side"],
        "screw_force": screw_force,
        "joint_A_load": unit_force["joint_A_load"] * scale,
        "joint_C_load": unit_force["joint_C_load"] * scale,
        "joint_B_load": unit_force["joint_B_load"] * scale,
        "joint_B_reaction": mul(scale, unit_force["joint_B_reaction"]),
    }


def print_contact_table():
    print("Contact and mechanical ratio")
    print("D(mm)  cableY  s_contact  remain  jaw_rot(deg)  dr/ds  N_side/Fs")
    for d in range(14, 30, 2):
        fs = force_state(float(d), 1000.0)
        print(
            f"{d:>5.0f}  {fs['cable_center'][1]:>6.2f}  {fs['s']:>9.3f}"
            f"  {fs['remaining']:>6.3f}  {fs['jaw_rot_deg']:>12.2f}"
            f"  {fs['closure_rate']:>5.3f}  {fs['normal_ratio_per_side']:>9.4f}"
        )


def print_screw_table():
    print("\nTrapezoidal screw examples, lead=2 mm, included angle=30 deg")
    print("d2(mm)  mu   lead_angle  eta    self_lock  margin")
    for d2 in (7.0, 9.0, 11.0):
        for mu in (0.10, 0.15, 0.20):
            e = screw_efficiency(d2, mu)
            print(
                f"{d2:>6.1f}  {mu:>4.2f}  {e['lead_angle_deg']:>10.2f}"
                f"  {e['eta']:>5.3f}  {str(e['self_lock']):>9}"
                f"  {e['self_lock_margin']:>6.2f}"
            )


def print_force_table(eta=0.35):
    fr = screw_thrust(SCREW["rated_torque_Nm"], eta)
    fp = screw_thrust(SCREW["peak_torque_Nm"], eta)
    print(f"\nReference force scaling, eta={eta:.2f}: rated thrust={fr:.0f} N, peak thrust={fp:.0f} N")
    print("D(mm)  N_side_rated  N_side_peak  Bpin_peak  A/Cpin_peak")
    for d in (14.0, 20.0, 28.0):
        unit_force = force_state(d, 1000.0)
        peak_scale = fp / 1000.0
        rated_scale = fr / 1000.0
        print(
            f"{d:>5.0f}  {unit_force['normal_per_side'] * rated_scale:>12.0f}"
            f"  {unit_force['normal_per_side'] * peak_scale:>11.0f}"
            f"  {unit_force['joint_B_load'] * peak_scale:>9.0f}"
            f"  {unit_force['joint_A_load'] * peak_scale:>11.0f}"
        )


def print_pin_table(eta=0.35):
    fp = screw_thrust(SCREW["peak_torque_Nm"], eta)
    worst = force_state(28.0, 1000.0)
    loads = {
        "B joint peak": worst["joint_B_load"] * fp / 1000.0,
        "A/C joint peak": worst["joint_A_load"] * fp / 1000.0,
    }
    example_pin = {
        "pin_diameter": 6.0,
        "bearing_width": 6.0,
        "shear_planes": 2,
        "allow_shear": 80.0,
        "allow_bearing": 80.0,
    }
    print(
        "\nReference peak-torque pin check example: d=6 mm, bearing width=6 mm, double shear,"
        " allowable shear=80 MPa, allowable bearing=80 MPa"
    )
    print("item            load(N)  tau(MPa)  bearing(MPa)  req_d_shear(mm)  req_width(mm)")
    for name, load in loads.items():
        c = pin_check(load, example_pin)
        print(
            f"{name:<14}  {load:>7.0f}  {c['tau']:>8.1f}"
            f"  {c['bearing']:>12.1f}  {c['req_d_shear']:>15.2f}"
            f"  {c['req_width']:>13.2f}"
        )


def print_target_force_table():
    print(f"\nTarget clamp force: {TARGET['normal_per_side']:.0f} N per side")
    print("D(mm)  s_contact  N/Fs    Fs_req(N)  T@eta.25  T@eta.35  T@eta.45  B_load  A/C_load")
    for d in range(14, 30, 2):
        ts = target_clamp_state(float(d))
        print(
            f"{d:>5.0f}  {ts['s']:>9.3f}  {ts['ratio']:>6.4f}"
            f"  {ts['screw_force']:>9.1f}"
            f"  {screw_torque(ts['screw_force'], 0.25):>8.3f}"
            f"  {screw_torque(ts['screw_force'], 0.35):>8.3f}"
            f"  {screw_torque(ts['screw_force'], 0.45):>8.3f}"
            f"  {ts['joint_B_load']:>6.1f}"
            f"  {ts['joint_A_load']:>8.1f}"
        )


def print_target_pin_table():
    print(
        "\nTarget pin check: d=3 mm, double shear, stainless steel,"
        " assumed allow shear=80 MPa, allow bearing=120 MPa"
    )
    print("D(mm)  joint  load(N)  b(mm)  tau(MPa)  p_bearing(MPa)  result")
    for d in range(14, 30, 2):
        ts = target_clamp_state(float(d))
        checks = [
            ("B", ts["joint_B_load"], TARGET["other_bearing_width"]),
            ("A/C", ts["joint_A_load"], TARGET["link1_bearing_width"]),
        ]
        for joint, load, width in checks:
            c = pin_check(
                load,
                {
                    "pin_diameter": TARGET["pin_diameter"],
                    "bearing_width": width,
                    "allow_shear": PIN_ASSUMPTION["allow_shear"],
                    "allow_bearing": PIN_ASSUMPTION["allow_bearing"],
                },
            )
            ok = c["tau"] <= PIN_ASSUMPTION["allow_shear"] and c["bearing"] <= PIN_ASSUMPTION["allow_bearing"]
            print(
                f"{d:>5.0f}  {joint:<5}  {load:>7.1f}  {width:>5.1f}"
                f"  {c['tau']:>8.1f}  {c['bearing']:>14.1f}  {'OK' if ok else 'NG'}"
            )


def main():
    print_contact_table()
    print_screw_table()
    print_force_table(eta=0.35)
    print_pin_table(eta=0.35)
    print_target_force_table()
    print_target_pin_table()


if __name__ == "__main__":
    main()
