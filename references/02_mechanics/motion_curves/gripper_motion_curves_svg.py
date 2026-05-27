import csv
from pathlib import Path


CSV_FILE = Path(__file__).with_name("gripper_motion_curves.csv")
SVG_FILE = Path(__file__).with_name("gripper_motion_curves.svg")


def read_rows():
    with CSV_FILE.open(encoding="utf-8") as f:
        return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]


def scale(value, src_min, src_max, dst_min, dst_max):
    if src_max == src_min:
        return (dst_min + dst_max) / 2
    return dst_min + (value - src_min) * (dst_max - dst_min) / (src_max - src_min)


def nice_ticks(vmin, vmax, count=5):
    if vmax == vmin:
        return [vmin]
    return [vmin + (vmax - vmin) * i / (count - 1) for i in range(count)]


def polyline(points):
    return " ".join(f"{x:.2f},{y:.2f}" for x, y in points)


def plot_panel(rows, x_key, y_keys, y_labels, title, x, y, w, h, colors):
    pad_l = 62
    pad_r = 18
    pad_t = 34
    pad_b = 42
    px0 = x + pad_l
    px1 = x + w - pad_r
    py0 = y + pad_t
    py1 = y + h - pad_b

    xs = [r[x_key] for r in rows]
    yvals = [r[k] for k in y_keys for r in rows]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(yvals), max(yvals)
    margin = (ymax - ymin) * 0.08 if ymax > ymin else 1.0
    ymin -= margin
    ymax += margin

    out = []
    out.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="#ffffff" stroke="#d0d0d0"/>')
    out.append(f'<text x="{x + w / 2:.1f}" y="{y + 22:.1f}" text-anchor="middle" class="title">{title}</text>')

    for tick in nice_ticks(xmin, xmax, 5):
        tx = scale(tick, xmin, xmax, px0, px1)
        out.append(f'<line x1="{tx:.2f}" y1="{py0}" x2="{tx:.2f}" y2="{py1}" class="grid"/>')
        out.append(f'<text x="{tx:.2f}" y="{py1 + 18}" text-anchor="middle" class="tick">{tick:.0f}</text>')

    for tick in nice_ticks(ymin, ymax, 5):
        ty = scale(tick, ymin, ymax, py1, py0)
        out.append(f'<line x1="{px0}" y1="{ty:.2f}" x2="{px1}" y2="{ty:.2f}" class="grid"/>')
        out.append(f'<text x="{px0 - 8}" y="{ty + 4:.2f}" text-anchor="end" class="tick">{tick:.2f}</text>')

    out.append(f'<line x1="{px0}" y1="{py1}" x2="{px1}" y2="{py1}" class="axis"/>')
    out.append(f'<line x1="{px0}" y1="{py0}" x2="{px0}" y2="{py1}" class="axis"/>')
    out.append(f'<text x="{x + w / 2:.1f}" y="{y + h - 10:.1f}" text-anchor="middle" class="label">Nut stroke s (mm)</text>')
    out.append(f'<text x="{x + 14:.1f}" y="{y + h / 2:.1f}" transform="rotate(-90 {x + 14:.1f},{y + h / 2:.1f})" text-anchor="middle" class="label">{y_labels[0]}</text>')

    legend_x = x + w - 205
    legend_y = y + 26
    for idx, key in enumerate(y_keys):
        pts = []
        for r in rows:
            px = scale(r[x_key], xmin, xmax, px0, px1)
            py = scale(r[key], ymin, ymax, py1, py0)
            pts.append((px, py))
        out.append(f'<polyline points="{polyline(pts)}" fill="none" stroke="{colors[idx]}" stroke-width="2.2"/>')
        out.append(f'<line x1="{legend_x}" y1="{legend_y + 18 * idx}" x2="{legend_x + 24}" y2="{legend_y + 18 * idx}" stroke="{colors[idx]}" stroke-width="2.2"/>')
        out.append(f'<text x="{legend_x + 30}" y="{legend_y + 4 + 18 * idx}" class="legend">{y_labels[idx]}</text>')

    return "\n".join(out)


def main():
    rows = read_rows()
    width = 1000
    panel_h = 280
    gap = 18
    height = panel_h * 3 + gap * 2 + 20
    colors = ["#1f77b4", "#d62728"]

    panels = [
        plot_panel(
            rows,
            "nut_stroke_mm",
            ["jaw_closing_angle_deg"],
            ["phi (deg)"],
            "Jaw angle - nut stroke",
            20,
            10,
            960,
            panel_h,
            colors,
        ),
        plot_panel(
            rows,
            "nut_stroke_mm",
            ["dphi_ds_deg_per_mm", "jaw_omega_deg_per_s"],
            ["dphi/ds (deg/mm)", "omega at vNut=1 mm/s (deg/s)"],
            "Jaw angular velocity - nut velocity",
            20,
            10 + panel_h + gap,
            960,
            panel_h,
            colors,
        ),
        plot_panel(
            rows,
            "nut_stroke_mm",
            ["d2phi_ds2_deg_per_mm2", "jaw_alpha_deg_per_s2"],
            ["d2phi/ds2 (deg/mm^2)", "alpha at vNut=1, aNut=0 (deg/s^2)"],
            "Jaw angular acceleration - nut acceleration",
            20,
            10 + (panel_h + gap) * 2,
            960,
            panel_h,
            colors,
        ),
    ]

    svg = f'''<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<style>
  text {{ font-family: Arial, Helvetica, sans-serif; fill: #202020; }}
  .title {{ font-size: 16px; font-weight: 700; }}
  .label {{ font-size: 12px; }}
  .tick {{ font-size: 11px; fill: #555; }}
  .legend {{ font-size: 12px; }}
  .grid {{ stroke: #e8e8e8; stroke-width: 1; }}
  .axis {{ stroke: #333; stroke-width: 1.2; }}
</style>
{chr(10).join(panels)}
</svg>
'''
    SVG_FILE.write_text(svg, encoding="utf-8")
    print(f"Saved figure: {SVG_FILE}")


if __name__ == "__main__":
    main()
