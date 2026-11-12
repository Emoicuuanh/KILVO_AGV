#!/usr/bin/env python3
"""Đánh giá quỹ đạo ước lượng so với ground truth mô phỏng (không cần ROS).

  eval_ate.py <thư_mục_kết_quả> [--est aft_mapped_to_init.tum wheel_odom.tum]

Mỗi file TUM ước lượng được:
  * nội suy GT tại thời điểm ước lượng;
  * căn chỉnh 2 cách: 'first' (khớp pose đầu — đúng thực tế: AGV xuất phát từ trạm sạc biết vị trí)
    và 'se3' (Umeyama, chuẩn ATE trong các bài báo);
  * báo cáo ATE RMSE (xy, z), sai số cuối, sai số tại mã QR (cm), RMSE theo vùng.
KILVO xuất pose của IMU (đặt tại base_link + 0.10 m z, cùng hướng) trong khung camera_init
(= IMU lúc đầu, đã căn trọng lực) -> căn 'first' khử được độ lệch cố định này.
"""
import argparse
import glob
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from agv_common import load_layout, qr_positions  # noqa: E402


# ===== ĐỌC FILE & TOÁN HỌC CƠ BẢN =====
# Đọc file TUM -> mảng (N, 8), sắp theo thời gian, bỏ mốc thời gian trùng
def load_tum(p):
    a = np.loadtxt(p, ndmin=2)
    if a.size == 0:
        return None
    a = a[np.argsort(a[:, 0])]
    _, idx = np.unique(a[:, 0], return_index=True)
    return a[idx]


# Quaternion (x y z w) -> ma trận quay 3x3
def quat_to_R(q):  # q = x y z w
    x, y, z, w = q.T
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z); R[:, 0, 1] = 2 * (x * y - z * w); R[:, 0, 2] = 2 * (x * z + y * w)
    R[:, 1, 0] = 2 * (x * y + z * w); R[:, 1, 1] = 1 - 2 * (x * x + z * z); R[:, 1, 2] = 2 * (y * z - x * w)
    R[:, 2, 0] = 2 * (x * z - y * w); R[:, 2, 1] = 2 * (y * z + x * w); R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


# Nội suy vị trí + yaw của ground truth tại các thời điểm t; ok = t nằm trong khoảng GT
def interp(gt, t):
    ok = (t >= gt[0, 0]) & (t <= gt[-1, 0])
    p = np.stack([np.interp(t, gt[:, 0], gt[:, k]) for k in (1, 2, 3)], 1)
    yaw = np.unwrap(np.arctan2(2 * (gt[:, 7] * gt[:, 6] + gt[:, 4] * gt[:, 5]), 1 - 2 * (gt[:, 5] ** 2 + gt[:, 6] ** 2)))
    return p, np.interp(t, gt[:, 0], yaw), ok


# Phép quay + tịnh tiến tối ưu đưa src khớp dst (bình phương tối thiểu) — chuẩn ATE
def umeyama(src, dst):
    ms, md = src.mean(0), dst.mean(0)
    U, _, Vt = np.linalg.svd((dst - md).T @ (src - ms))
    S = np.eye(3)
    S[2, 2] = np.sign(np.linalg.det(U @ Vt))
    R = U @ S @ Vt
    return R, md - R @ ms


# ===== ĐÁNH GIÁ 1 QUỸ ĐẠO =====
def evaluate(est, gt, layout):
    # --- B1. Ghép cặp: GT nội suy tại thời điểm của ước lượng ---
    t = est[:, 0]
    g_p, g_yaw, ok = interp(gt, t)
    est, g_p, g_yaw, t = est[ok], g_p[ok], g_yaw[ok], t[ok]
    if len(t) < 10:
        return None
    # --- B2. Căn chỉnh 2 kiểu: 'first' (khớp pose đầu) và 'se3' (Umeyama) ---
    e_p, e_R = est[:, 1:4], quat_to_R(est[:, 4:8])
    # căn theo pose đầu: T_align = T_gt0 * T_est0^-1 (chỉ yaw + tịnh tiến, giữ phương trọng lực)
    e_yaw = np.unwrap(np.arctan2(e_R[:, 1, 0], e_R[:, 0, 0]))
    dyaw = g_yaw[0] - e_yaw[0]
    Rz = np.array([[np.cos(dyaw), -np.sin(dyaw), 0], [np.sin(dyaw), np.cos(dyaw), 0], [0, 0, 1]])
    p_first = (Rz @ (e_p - e_p[0]).T).T + g_p[0]
    R, tr = umeyama(e_p, g_p)
    p_se3 = (R @ e_p.T).T + tr
    res = {"n": int(len(t)), "duration_s": float(t[-1] - t[0]),
           "length_m": float(np.sum(np.linalg.norm(np.diff(g_p[:, :2], axis=0), axis=1)))}
    # --- B3. Sai số vị trí xy, z, sai số cuối, sai số yaw, độ trôi % ---
    for name, p in (("first", p_first), ("se3", p_se3)):
        exy = np.linalg.norm(p[:, :2] - g_p[:, :2], axis=1)
        ez = p[:, 2] - g_p[:, 2]
        res[name] = {"ate_xy_rmse_m": float(np.sqrt(np.mean(exy ** 2))), "ate_xy_max_m": float(exy.max()),
                     "z_rmse_m": float(np.sqrt(np.mean(ez ** 2))), "final_xy_m": float(exy[-1])}
    eyaw = (e_yaw + dyaw) - g_yaw
    res["first"]["yaw_rmse_deg"] = float(np.degrees(np.sqrt(np.mean(((eyaw + np.pi) % (2 * np.pi) - np.pi) ** 2))))
    res["first"]["drift_pct"] = 100 * res["first"]["final_xy_m"] / max(res["length_m"], 1e-6)

    # --- B4. Sai số tại mã QR ---
    # sai số tại mã QR (căn 'first'): lần robot đi qua gần tâm mã nhất trong mỗi lượt
    exy = np.linalg.norm(p_first[:, :2] - g_p[:, :2], axis=1)
    rad = layout["qr"]["capture_radius"]
    qr_err = []
    for qx, qy in qr_positions(layout["qr"]):
        d = np.hypot(g_p[:, 0] - qx, g_p[:, 1] - qy)
        inside = d < rad
        if not inside.any():
            continue
        edges = np.flatnonzero(np.diff(np.r_[0, inside.astype(int), 0]))
        for a, b in zip(edges[::2], edges[1::2]):
            k = a + int(np.argmin(d[a:b]))
            qr_err.append(exy[k])
    if qr_err:
        q = np.array(qr_err) * 100
        res["qr"] = {"reads": len(q), "mean_cm": float(q.mean()), "max_cm": float(q.max())}
    # --- B5. RMSE theo từng vùng (hành lang lặp, vùng tối, sàn trơn...) ---
    zones = {}
    for z in layout["zones"]:
        x0, x1, y0, y1 = z["box"]
        m = (g_p[:, 0] >= x0) & (g_p[:, 0] <= x1) & (g_p[:, 1] >= y0) & (g_p[:, 1] <= y1)
        if m.sum() > 5:
            zones[z["name"]] = float(np.sqrt(np.mean(exy[m] ** 2)))
    res["zone_xy_rmse_m"] = zones
    return res, t, g_p, p_first, exy


# ===== MAIN: đánh giá mọi file .tum, in bảng, lưu metrics.md/json, vẽ trajectory.png =====
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    ap.add_argument("--est", nargs="*")
    ap.add_argument("--layout", default=None)
    a = ap.parse_args()
    layout = load_layout(a.layout)
    gt = load_tum(os.path.join(a.dir, "ground_truth.tum"))
    if gt is None:
        sys.exit("ground_truth.tum rỗng")
    ests = a.est or sorted(os.path.basename(p) for p in glob.glob(os.path.join(a.dir, "*.tum")) if "ground_truth" not in p)
    allres, curves = {}, {}
    for e in ests:
        arr = load_tum(os.path.join(a.dir, e))
        if arr is None:
            print(f"{e}: rỗng (thuật toán không xuất pose?)")
            continue
        r = evaluate(arr, gt, layout)
        if r is None:
            print(f"{e}: không đủ dữ liệu trùng thời gian với GT")
            continue
        allres[e], curves[e] = r[0], r[1:]
    if not allres:
        sys.exit(1)

    # --- Bảng kết quả ---
    lines = ["| Ước lượng | ATE xy (first) | ATE xy (se3) | Cuối xy | Trôi % | z RMSE | yaw RMSE | QR TB / max |",
             "|---|---|---|---|---|---|---|---|"]
    for e, r in allres.items():
        q = r.get("qr", {})
        lines.append(f"| {e} | {r['first']['ate_xy_rmse_m']*100:.1f} cm | {r['se3']['ate_xy_rmse_m']*100:.1f} cm | "
                     f"{r['first']['final_xy_m']*100:.1f} cm | {r['first']['drift_pct']:.2f} | "
                     f"{r['first']['z_rmse_m']*100:.1f} cm | {r['first']['yaw_rmse_deg']:.2f}° | "
                     f"{q.get('mean_cm', float('nan')):.1f} / {q.get('max_cm', float('nan')):.1f} cm |")
    lines.append("\nRMSE xy theo vùng (căn first), cm:")
    for e, r in allres.items():
        lines.append(f"  {e}: " + ", ".join(f"{k}={v*100:.1f}" for k, v in r["zone_xy_rmse_m"].items()))
    report = "\n".join(lines)
    print(report)
    with open(os.path.join(a.dir, "metrics.md"), "w") as f:
        f.write(report + "\n")
    with open(os.path.join(a.dir, "metrics.json"), "w") as f:
        json.dump(allres, f, indent=2)

    # --- Hình: bên trái quỹ đạo trên mặt bằng (kệ, vùng trơn/tối), bên phải sai số theo thời gian ---
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, (ax, bx) = plt.subplots(1, 2, figsize=(15, 6), gridspec_kw={"width_ratios": [1.6, 1]})
        sh = layout["shelves"]
        for ry in sh["rows_y"]:
            ax.add_patch(plt.Rectangle((sh["x"][0], ry - sh["depth"] / 2), sh["x"][1] - sh["x"][0], sh["depth"], color="#ccc"))
        for z in layout["zones"]:
            if z["type"] in ("slippery", "dark"):
                x0, x1, y0, y1 = z["box"]
                ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, alpha=0.25,
                                           color="#3b82f6" if z["type"] == "slippery" else "#333"))
        first = True
        for e, (t, g_p, p, exy) in curves.items():
            if first:
                ax.plot(g_p[:, 0], g_p[:, 1], "k-", lw=1.5, label="ground truth")
                first = False
            ax.plot(p[:, 0], p[:, 1], lw=1, label=e)
            bx.plot(t - t[0], exy * 100, lw=1, label=e)
        ax.set_aspect("equal"); ax.legend(fontsize=8); ax.set_title("Quỹ đạo (căn pose đầu)")
        bx.set_xlabel("t (s)"); bx.set_ylabel("sai số xy (cm)"); bx.grid(alpha=0.3); bx.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(a.dir, "trajectory.png"), dpi=120)
        print("Hình:", os.path.join(a.dir, "trajectory.png"))
    except Exception as ex:  # noqa: BLE001
        print("Không vẽ được hình:", ex)


if __name__ == "__main__":
    main()
