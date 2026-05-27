import csv
import os
from pathlib import Path


BASE = Path(__file__).resolve().parent
CSV_PATH = BASE / "gripper_motion_curves.csv"
PNG_PATH = BASE / "gripper_motion_curves.png"
MPL_CONFIG = BASE / ".matplotlib"
MPL_CONFIG.mkdir(exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(MPL_CONFIG))

import matplotlib.pyplot as plt


def load_data():
    with CSV_PATH.open(encoding="utf-8") as f:
        rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]
    return rows


def col(rows, name):
    return [row[name] for row in rows]


def main():
    rows = load_data()
    s = col(rows, "nut_stroke_mm")

    fig, axes = plt.subplots(3, 1, figsize=(9.5, 10.5), constrained_layout=True)

    axes[0].plot(s, col(rows, "jaw_closing_angle_deg"), linewidth=2)
    axes[0].set_title("Jaw angle - nut stroke")
    axes[0].set_xlabel("Nut stroke s (mm)")
    axes[0].set_ylabel("Jaw closing angle phi (deg)")
    axes[0].grid(True, alpha=0.35)

    axes[1].plot(s, col(rows, "dphi_ds_deg_per_mm"), linewidth=2, label="dphi/ds (deg/mm)")
    axes[1].plot(s, col(rows, "jaw_omega_deg_per_s"), "--", linewidth=1.7, label="omega at vNut=1 mm/s (deg/s)")
    axes[1].set_title("Jaw angular velocity - nut velocity")
    axes[1].set_xlabel("Nut stroke s (mm)")
    axes[1].set_ylabel("Velocity ratio / angular velocity")
    axes[1].grid(True, alpha=0.35)
    axes[1].legend()

    axes[2].plot(s, col(rows, "d2phi_ds2_deg_per_mm2"), linewidth=2, label="d2phi/ds2 (deg/mm^2)")
    axes[2].plot(s, col(rows, "jaw_alpha_deg_per_s2"), "--", linewidth=1.7, label="alpha at vNut=1, aNut=0 (deg/s^2)")
    axes[2].set_title("Jaw angular acceleration - nut acceleration")
    axes[2].set_xlabel("Nut stroke s (mm)")
    axes[2].set_ylabel("Acceleration ratio / angular acceleration")
    axes[2].grid(True, alpha=0.35)
    axes[2].legend()

    fig.savefig(PNG_PATH, dpi=200)
    print(f"Saved figure: {PNG_PATH}")


if __name__ == "__main__":
    main()
