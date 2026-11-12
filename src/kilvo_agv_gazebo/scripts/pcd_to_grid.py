#!/usr/bin/env python3
"""Chuyển bản đồ point cloud của KILVO (.pcd) thành bản đồ lưới 2D (.pgm + .yaml) cho map_server/move_base.

  pcd_to_grid.py <map.pcd> <đầu_ra_không_đuôi> [--res 0.05] [--zmin 0.08] [--zmax 1.0]

Cách làm:
  1. Đọc điểm (x, y, z) từ file .pcd (ASCII hoặc binary, như KILVO ghi).
  2. Tìm độ cao mặt sàn = đỉnh histogram z ở phần thấp (bản đồ KILVO đã căn theo trọng lực).
  3. Giữ các điểm cao hơn sàn từ zmin tới zmax (vật cản trong tầm cao của AGV), bỏ sàn và trần.
  4. Chiếu xuống mặt phẳng xy: ô có >= min_pts điểm = VẬT CẢN (đen).
     Ô nằm trong vùng robot đã quét (có điểm sàn gần đó) = TRỐNG (trắng); còn lại = CHƯA BIẾT (xám).
  5. Ghi .pgm (ảnh) + .yaml (độ phân giải, gốc tọa độ) theo chuẩn map_server.
Khung tọa độ: giữ nguyên khung camera_init của KILVO (gốc = vị trí IMU lúc khởi động ở trạm sạc).
"""
import argparse
import os

import numpy as np


# ===== 1. ĐỌC FILE .pcd =====
def read_pcd_xyz(path):
    with open(path, "rb") as f:
        header = {}
        while True:
            line = f.readline().decode("ascii", "ignore").strip()
            if not line or line.startswith("#"):
                continue
            k, *v = line.split()
            header[k.upper()] = v
            if k.upper() == "DATA":
                break
        fields, sizes, types = header["FIELDS"], list(map(int, header["SIZE"])), header["TYPE"]
        counts = list(map(int, header.get("COUNT", ["1"] * len(fields))))
        n = int(header["POINTS"][0])
        if header["DATA"][0] == "ascii":
            arr = np.loadtxt(f, ndmin=2)
            cols = np.cumsum([0] + counts)
            idx = {name: cols[i] for i, name in enumerate(fields)}
            return np.stack([arr[:, idx["x"]], arr[:, idx["y"]], arr[:, idx["z"]]], 1)
        if header["DATA"][0] != "binary":
            raise SystemExit("Chỉ hỗ trợ PCD ascii/binary (không nén)")
        kind = {"F": "f", "U": "u", "I": "i"}
        dtype = np.dtype([(name, f"<{kind[t]}{s}", (c,) if c > 1 else ())
                          for name, s, t, c in zip(fields, sizes, types, counts)])
        data = np.frombuffer(f.read(n * dtype.itemsize), dtype=dtype, count=n)
        return np.stack([data["x"], data["y"], data["z"]], 1).astype(np.float64)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pcd")
    ap.add_argument("out", help="đường dẫn đầu ra không đuôi, vd .../maps/warehouse_map")
    ap.add_argument("--res", type=float, default=0.05, help="độ phân giải ô (m)")
    ap.add_argument("--zmin", type=float, default=0.08, help="bỏ điểm thấp hơn sàn + zmin (sàn, mấp mô)")
    ap.add_argument("--zmax", type=float, default=1.0, help="bỏ điểm cao hơn sàn + zmax (cao hơn robot)")
    ap.add_argument("--min_pts", type=int, default=3, help="số điểm tối thiểu để ô là vật cản (lọc nhiễu)")
    ap.add_argument("--free_radius", type=float, default=0.6, help="ô cách điểm sàn < r (m) coi là trống")
    a = ap.parse_args()

    p = read_pcd_xyz(a.pcd)
    p = p[np.all(np.isfinite(p), axis=1)]
    print(f"Đọc {len(p)} điểm")

    # ===== 2. ĐỘ CAO MẶT SÀN =====
    lo = p[:, 2] < np.percentile(p[:, 2], 30)
    hist, edges = np.histogram(p[lo, 2], bins=200)
    floor_z = 0.5 * (edges[np.argmax(hist)] + edges[np.argmax(hist) + 1])
    print(f"Sàn ở z = {floor_z:.3f} m (khung camera_init)")

    # ===== 3. LỚP VẬT CẢN và LỚP SÀN =====
    obst = p[(p[:, 2] > floor_z + a.zmin) & (p[:, 2] < floor_z + a.zmax)]
    floor = p[np.abs(p[:, 2] - floor_z) < 0.05]

    # ===== 4. CHIẾU XUỐNG LƯỚI 2D =====
    xy_all = np.vstack([obst[:, :2], floor[:, :2]])
    xmin, ymin = xy_all.min(0) - 1.0
    xmax, ymax = xy_all.max(0) + 1.0
    W, H = int(np.ceil((xmax - xmin) / a.res)), int(np.ceil((ymax - ymin) / a.res))

    def to_cell(xy):
        return ((xy[:, 0] - xmin) / a.res).astype(int), ((xy[:, 1] - ymin) / a.res).astype(int)

    occ_cnt = np.zeros((H, W), np.int32)
    cx, cy = to_cell(obst[:, :2])
    np.add.at(occ_cnt, (cy, cx), 1)
    occupied = occ_cnt >= a.min_pts

    seen = np.zeros((H, W), bool)
    fx, fy = to_cell(floor[:, :2])
    seen[fy, fx] = True
    # nở vùng sàn đã thấy ra free_radius (lấp khe giữa các điểm sàn và vùng mù quanh robot)
    k = max(1, int(round(a.free_radius / a.res)))
    acc = np.zeros((H + 2 * k, W + 2 * k), bool)
    for dy in range(-k, k + 1):
        for dx in range(-k, k + 1):
            if dx * dx + dy * dy <= k * k:
                acc[k + dy:k + dy + H, k + dx:k + dx + W] |= seen
    free = acc[k:k + H, k:k + W] & ~occupied

    # ===== 5. GHI .pgm + .yaml (map_server: trắng 254 = trống, đen 0 = vật cản, xám 205 = chưa biết) =====
    img = np.full((H, W), 205, np.uint8)
    img[free] = 254
    img[occupied] = 0
    img = np.flipud(img)  # ảnh: hàng 0 ở trên cùng = y lớn nhất
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out + ".pgm", "wb") as f:
        f.write(f"P5\n{W} {H}\n255\n".encode())
        f.write(img.tobytes())
    with open(a.out + ".yaml", "w") as f:
        f.write(f"image: {os.path.basename(a.out)}.pgm\n"
                f"resolution: {a.res}\n"
                f"origin: [{xmin:.3f}, {ymin:.3f}, 0.0]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    print(f"Ghi {a.out}.pgm/.yaml: {W}x{H} ô ({W*a.res:.1f} x {H*a.res:.1f} m), "
          f"{occupied.sum()} ô vật cản, {free.sum()} ô trống")


if __name__ == "__main__":
    main()
