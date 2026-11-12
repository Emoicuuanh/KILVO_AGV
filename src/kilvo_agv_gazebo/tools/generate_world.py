#!/usr/bin/env python3
"""Sinh worlds/warehouse.sdf từ config/layout.yaml.

    python3 tools/generate_world.py            # ghi đè worlds/warehouse.sdf
    python3 tools/generate_world.py --seed 3   # đổi cách xếp thùng hàng trên kệ
"""
import argparse
import math
import os
import random

import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def mat(r, g, b):
    return (f"<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>"
            f"<specular>0.1 0.1 0.1 1</specular></material>")


def box(name, x, y, z, sx, sy, sz, color, collide=True, mu=None, yaw=0.0):
    pose = f"<pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}</pose>"
    geom = f"<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>"
    out = f'<visual name="{name}_v">{pose}{geom}{mat(*color)}</visual>'
    if collide:
        surf = ""
        if mu is not None:
            surf = f"<surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>"
        out += f'<collision name="{name}_c">{pose}{geom}{surf}</collision>'
    return out


def static_model(name, body):
    return f'<model name="{name}"><static>true</static><link name="link">{body}</link></model>\n'


def qr_positions(qr):
    pts = []
    for x0, y0, x1, y1, step in qr["lines"]:
        length = math.hypot(x1 - x0, y1 - y0)
        n = int(round(length / step))
        for i in range(n + 1):
            t = i / n if n else 0.0
            pts.append((round(x0 + t * (x1 - x0), 3), round(y0 + t * (y1 - y0), 3)))
    # bỏ trùng
    seen, out = set(), []
    for p in pts:
        if p not in seen:
            seen.add(p)
            out.append(p)
    return out


def build(cfg, seed, classic=False):
    rng = random.Random(seed)
    hall = cfg["hall"]
    (x0, x1), (y0, y1) = hall["x"], hall["y"]
    W, H = x1 - x0, y1 - y0
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    wh = hall["wall_height"]
    models = []

    # ---------- sàn + tường + vùng trơn + mã QR ----------
    b = box("floor", cx, cy, -0.05, W + 1, H + 1, 0.1, (0.55, 0.55, 0.52), mu=hall["floor_mu"])
    t = 0.3
    b += box("wall_n", cx, y1 + t / 2, wh / 2, W + 2 * t, t, wh, (0.7, 0.7, 0.68))
    b += box("wall_s", cx, y0 - t / 2, wh / 2, W + 2 * t, t, wh, (0.7, 0.7, 0.68))
    b += box("wall_e", x1 + t / 2, cy, wh / 2, t, H, wh, (0.7, 0.7, 0.68))
    b += box("wall_w", x0 - t / 2, cy, wh / 2, t, H, wh, (0.7, 0.7, 0.68))
    for z in cfg["zones"]:
        if z["type"] != "slippery":
            continue
        zx0, zx1, zy0, zy1 = z["box"]
        # tấm mỏng 1 mm, ma sát rất thấp, màu xanh nhạt "sàn ướt"
        b += box(f"slip_{z['name']}", (zx0 + zx1) / 2, (zy0 + zy1) / 2, 0.0005,
                 zx1 - zx0, zy1 - zy0, 0.001, (0.35, 0.5, 0.65), mu=z["mu"])
    qs = cfg["qr"]["size"]
    for i, (qx, qy) in enumerate(qr_positions(cfg["qr"])):
        b += box(f"qr{i}", qx, qy, 0.0015, qs, qs, 0.001, (0.02, 0.02, 0.02), collide=False)
        b += box(f"qr{i}_in", qx, qy, 0.0021, qs * 0.4, qs * 0.4, 0.001, (0.95, 0.95, 0.95), collide=False)
    cs = cfg["qr"]["charging_station"]
    b += box("charger", cs[0] - 0.9, cs[1], 0.3, 0.3, 0.8, 0.6, (0.1, 0.5, 0.1))
    models.append(static_model("warehouse_shell", b))

    # ---------- dãy kệ lặp lại ----------
    sh = cfg["shelves"]
    sx0, sx1 = sh["x"]
    L = sx1 - sx0
    d, hgt = sh["depth"], sh["height"]
    nb = int(round(L / sh["bay_length"]))
    for r, ry in enumerate(sh["rows_y"]):
        b = ""
        for i in range(nb + 1):
            ux = sx0 + i * sh["bay_length"]
            for side in (-1, 1):
                b += box(f"up{i}_{side+1}", ux, ry + side * (d / 2 - 0.04), hgt / 2,
                         0.08, 0.08, hgt, (0.15, 0.3, 0.7))
        for k, lz in enumerate(sh["levels"]):
            b += box(f"beam{k}", (sx0 + sx1) / 2, ry, lz, L, d, 0.04, (0.85, 0.45, 0.1))
        for i in range(nb):
            bx = sx0 + (i + 0.5) * sh["bay_length"]
            for k, lz in enumerate(sh["levels"][:-1]):
                if rng.random() < sh["pallet_fill"]:
                    ph = rng.uniform(0.35, 0.6)
                    pw = rng.uniform(1.0, 2.1)
                    b += box(f"pal{i}_{k}", bx + rng.uniform(-0.1, 0.1), ry, lz + 0.02 + ph / 2,
                             pw, d * 0.85, ph, (0.65, 0.5, 0.3))
        models.append(static_model(f"shelf_row_{r}", b))

    # ---------- hành lang tường trơn ----------
    fc = cfg["featureless_corridor"]
    fx0, fx1 = fc["x"]
    b = ""
    for j, wy in enumerate(fc["walls_y"]):
        b += box(f"fw{j}", (fx0 + fx1) / 2, wy, fc["height"] / 2, fx1 - fx0, 0.2, fc["height"],
                 (0.8, 0.8, 0.8))
    models.append(static_model("featureless_corridor", b))

    # ---------- cột + pallet rải rác (đặc trưng cho khu trống) ----------
    b = ""
    for i, px in enumerate([-16, -8, 0, 8, 16]):
        b += box(f"col_s{i}", px, -12.3, wh / 2, 0.4, 0.4, wh, (0.6, 0.6, 0.6))
        b += box(f"col_m{i}", px - 2, -7.6, wh / 2, 0.4, 0.4, wh, (0.6, 0.6, 0.6))
    for i, py in enumerate([-8, -3, 2, 7]):
        for side, px in (("w", x0 + 0.6), ("e", x1 - 0.6)):
            hh = rng.uniform(0.6, 1.5)
            b += box(f"pallet_{side}{i}", px, py + rng.uniform(-1, 1), hh / 2, 0.8, 1.2, hh,
                     (0.6, 0.45, 0.25), yaw=rng.uniform(-0.2, 0.2))
    models.append(static_model("clutter", b))

    # ---------- đèn ----------
    lc = cfg["lighting"]
    lights = ""
    n = 0
    for lx0, lx1, ly0, ly1 in lc["lit_regions"]:
        long_x = (lx1 - lx0) >= (ly1 - ly0)
        a0, a1 = (lx0, lx1) if long_x else (ly0, ly1)
        c = (ly0 + ly1) / 2 if long_x else (lx0 + lx1) / 2
        k = max(1, int(round((a1 - a0) / lc["spacing"])))
        for i in range(k):
            a = a0 + (i + 0.5) * (a1 - a0) / k
            lx, ly = (a, c) if long_x else (c, a)
            lights += (f'<light type="point" name="lamp{n}"><pose>{lx:.2f} {ly:.2f} {lc["height"]} 0 0 0</pose>'
                       f"<diffuse>0.9 0.9 0.85 1</diffuse><specular>0.1 0.1 0.1 1</specular>"
                       f"<attenuation><range>{lc['range']}</range><constant>0.4</constant>"
                       f"<linear>0.05</linear><quadratic>0.02</quadratic></attenuation>"
                       f"<cast_shadows>false</cast_shadows></light>\n")
            n += 1

    amb = lc["ambient"]
    cs0 = cfg["qr"]["charging_station"]
    if classic:
        world = f"""<?xml version="1.0"?>
<!-- TỰ SINH bởi tools/generate_world.py (Gazebo 11 classic) từ config/layout.yaml (seed={seed}). Đừng sửa tay. -->
<sdf version="1.6">
  <world name="warehouse">
    <physics name="ode_1ms" default="true" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode><solver><type>quick</type><iters>50</iters></solver></ode>
    </physics>
    <gravity>0 0 -9.81</gravity>
    <scene>
      <ambient>{amb} {amb} {amb} 1</ambient>
      <background>0.05 0.05 0.07 1</background>
      <shadows>false</shadows>
      <grid>false</grid>
    </scene>
{lights}
{''.join(models)}
  </world>
</sdf>
"""
        return world, n
    world = f"""<?xml version="1.0"?>
<!-- TỰ SINH bởi tools/generate_world.py từ config/layout.yaml (seed={seed}). Đừng sửa tay. -->
<sdf version="1.9">
  <world name="warehouse">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <gravity>0 0 -9.81</gravity>
    <scene>
      <ambient>{amb} {amb} {amb} 1</ambient>
      <background>0.05 0.05 0.07 1</background>
      <shadows>false</shadows>
      <grid>false</grid>
    </scene>
{lights}
{''.join(models)}
    <include>
      <uri>model://agv</uri>
      <name>agv</name>
      <pose>{cs0[0]} {cs0[1]} 0.105 0 0 0</pose>
    </include>
  </world>
</sdf>
"""
    return world, n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--layout", default=os.path.join(PKG, "config", "layout.yaml"))
    ap.add_argument("--out", default=os.path.join(PKG, "worlds", "warehouse.world"))
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--classic", action="store_true", help="xuất SDF 1.6 cho Gazebo 11 (ROS 1)")
    a = ap.parse_args()
    with open(a.layout) as f:
        cfg = yaml.safe_load(f)
    world, nl = build(cfg, a.seed, a.classic)
    with open(a.out, "w") as f:
        f.write(world)
    print(f"Đã ghi {a.out}  ({nl} đèn, {len(qr_positions(cfg['qr']))} mã QR)")


if __name__ == "__main__":
    main()
