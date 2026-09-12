import os
import socket
import struct
import time
import numpy as np
import matplotlib

# macOS 上通常比默认 backend 更稳
try:
    matplotlib.use("TkAgg")
except Exception:
    pass

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# =========================
# Global running flag
# =========================
RUNNING = True

def request_stop():
    global RUNNING
    RUNNING = False
    print("收到退出请求，准备保存地图并退出...")

# =========================
# UDP config
# =========================
PORT = 9999
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", PORT))
sock.settimeout(0.2)

# =========================
# Map params
# =========================
MAP_RES = 0.10
MAP_SIZE_M = 100.0
MAP_N = int(MAP_SIZE_M / MAP_RES)

MAP_X_MIN = -MAP_SIZE_M / 2.0
MAP_Z_MIN = -MAP_SIZE_M / 2.0

L_FREE = -0.35
L_OCC = +0.90
L_MIN = -5.0
L_MAX = +5.0

USE_MAX_RANGE_AS_FREE_ONLY = True

ROS_OCC_THRESH = 0.65
ROS_FREE_THRESH = 0.25

SAVE_DIR = "slam_output"
os.makedirs(SAVE_DIR, exist_ok=True)

# =========================
# Global map state
# =========================
log_odds = np.zeros((MAP_N, MAP_N), dtype=np.float32)
explored_mask = np.zeros((MAP_N, MAP_N), dtype=bool)
trajectory = []

# marker points in world coordinates
markers_world = []

# current pose for marker recording
current_pose = {
    "x": None,
    "z": None,
    "yaw": None,
}

# =========================
# Figure / UI
# =========================
plt.ion()
fig, (ax_local, ax_map) = plt.subplots(1, 2, figsize=(13, 6))

# --- Left: local lidar view ---
sc = ax_local.scatter([], [], s=4)
car_size = 0.5
car_rect = Rectangle(
    (-car_size / 2, -car_size / 2),
    car_size, car_size,
    fill=False, edgecolor="red", linewidth=2
)
ax_local.add_patch(car_rect)
heading_line, = ax_local.plot([0.0, 0.0], [0.0, 0.9], color="green", linewidth=3)

yaw_text = ax_local.text(0.02, 0.98, "", transform=ax_local.transAxes, va="top")
pose_text = ax_local.text(0.02, 0.90, "", transform=ax_local.transAxes, va="top")
layer_text = ax_local.text(0.02, 0.82, "", transform=ax_local.transAxes, va="top")

ax_local.set_aspect("equal", "box")
ax_local.set_title("Local Lidar View (All Layers)")
ax_local.set_xlabel("right")
ax_local.set_ylabel("forward")

# --- Right: global map ---
initial_vis = np.full((MAP_N, MAP_N), 0.5, dtype=np.float32)
map_img = ax_map.imshow(
    initial_vis,
    origin="lower",
    cmap="gray",
    vmin=0.0,
    vmax=1.0,
    extent=[MAP_X_MIN, MAP_X_MIN + MAP_SIZE_M, MAP_Z_MIN, MAP_Z_MIN + MAP_SIZE_M],
)
traj_line, = ax_map.plot([], [], "r-", linewidth=1)
marker_scatter = ax_map.scatter([], [], s=35, c="red", marker="o")

ax_map.set_title("Occupancy Grid Map (All Layers Stacked)")
ax_map.set_xlabel("world x")
ax_map.set_ylabel("world z")
ax_map.set_aspect("equal", "box")

# only for local display normalization
yaw0 = None

# =========================
# Keyboard / close events
# =========================
def update_marker_artist():
    if len(markers_world) == 0:
        marker_scatter.set_offsets(np.empty((0, 2)))
    else:
        marker_scatter.set_offsets(np.asarray(markers_world))

def on_key_press(event):
    key = event.key.lower() if event.key is not None else ""

    # 在地图窗口按 7 记录 marker
    if key in ("7", "cmd+7", "super+7", "meta+7"):
        if current_pose["x"] is None or current_pose["z"] is None:
            print("还没有收到有效 pose，无法打点。")
            return

        mx = float(current_pose["x"])
        mz = float(current_pose["z"])
        markers_world.append([mx, mz])

        print(f"[MARK] 已记录 marker #{len(markers_world)} at world ({mx:.2f}, {mz:.2f})")
        update_marker_artist()
        fig.canvas.draw_idle()
        return

    # q 退出并保存
    if key == "q":
        request_stop()

def on_close(event):
    request_stop()

fig.canvas.mpl_connect("key_press_event", on_key_press)
fig.canvas.mpl_connect("close_event", on_close)

# =========================
# Packet parsing
# =========================
def parse_packet(data: bytes):
    """
    Multi-layer packet format:
    0   : 4 bytes magic b"LDM3"
    4   : uint32 n_rays
    8   : uint32 n_layers
    12  : float angle_min
    16  : float angle_inc
    20  : float range_min
    24  : float range_max
    28  : float posx
    32  : float posz
    36  : float fwdx
    40  : float fwdz
    44  : float heights[n_layers]
    ... : float flat_ranges[n_layers * n_rays]
    """
    if len(data) < 44 or data[:4] != b"LDM3":
        return None

    n_rays = struct.unpack_from("<I", data, 4)[0]
    n_layers = struct.unpack_from("<I", data, 8)[0]

    angle_min, angle_inc, rmin, rmax = struct.unpack_from("<ffff", data, 12)
    posx, posz, fwdx, fwdz = struct.unpack_from("<ffff", data, 28)

    off = 44
    heights = np.frombuffer(data, dtype=np.float32, offset=off, count=n_layers).copy()
    off += 4 * n_layers

    total_ranges = n_layers * n_rays
    expected = off + 4 * total_ranges
    if len(data) != expected:
        return None

    flat_ranges = np.frombuffer(data, dtype=np.float32, offset=off, count=total_ranges).copy()
    layer_ranges = flat_ranges.reshape(n_layers, n_rays)

    return n_rays, n_layers, angle_min, angle_inc, rmin, rmax, posx, posz, fwdx, fwdz, heights, layer_ranges

# =========================
# Geometry helpers
# =========================
def rot2d(x, z, yaw):
    c = np.cos(yaw)
    s = np.sin(yaw)
    return c * x - s * z, s * x + c * z

def world_to_grid(x, z):
    gx = int(np.floor((x - MAP_X_MIN) / MAP_RES))
    gz = int(np.floor((z - MAP_Z_MIN) / MAP_RES))
    return gx, gz

def in_map(gx, gz):
    return 0 <= gx < MAP_N and 0 <= gz < MAP_N

def clamp_to_map_world(x, z):
    x = np.clip(x, MAP_X_MIN + 1e-6, MAP_X_MIN + MAP_SIZE_M - 1e-6)
    z = np.clip(z, MAP_Z_MIN + 1e-6, MAP_Z_MIN + MAP_SIZE_M - 1e-6)
    return x, z

def bresenham(gx0, gz0, gx1, gz1):
    cells = []

    dx = abs(gx1 - gx0)
    dz = abs(gz1 - gz0)
    x, z = gx0, gz0
    sx = 1 if gx1 >= gx0 else -1
    sz = 1 if gz1 >= gz0 else -1

    if dz <= dx:
        err = dx / 2
        while x != gx1:
            cells.append((x, z))
            err -= dz
            if err < 0:
                z += sz
                err += dx
            x += sx
        cells.append((gx1, gz1))
    else:
        err = dz / 2
        while z != gz1:
            cells.append((x, z))
            err -= dx
            if err < 0:
                x += sx
                err += dz
            z += sz
        cells.append((gx1, gz1))

    return cells

# =========================
# Mapping
# =========================
def update_map_with_scan(posx, posz, yaw, angles, ranges, rmin, rmax):
    gx0, gz0 = world_to_grid(posx, posz)
    if not in_map(gx0, gz0):
        return

    for a, r in zip(angles, ranges):
        if not np.isfinite(r):
            continue
        if r < rmin:
            continue

        rr = min(float(r), rmax)

        x_end = posx + rr * np.cos(yaw + a)
        z_end = posz + rr * np.sin(yaw + a)

        x_end, z_end = clamp_to_map_world(x_end, z_end)
        gx1, gz1 = world_to_grid(x_end, z_end)

        if not in_map(gx1, gz1):
            continue

        ray_cells = bresenham(gx0, gz0, gx1, gz1)
        if len(ray_cells) == 0:
            continue

        for gx, gz in ray_cells:
            if in_map(gx, gz):
                explored_mask[gz, gx] = True

        for gx, gz in ray_cells[:-1]:
            if in_map(gx, gz):
                log_odds[gz, gx] = np.clip(log_odds[gz, gx] + L_FREE, L_MIN, L_MAX)

        if USE_MAX_RANGE_AS_FREE_ONLY and rr >= (rmax - 1e-3):
            continue

        gx, gz = ray_cells[-1]
        if in_map(gx, gz):
            log_odds[gz, gx] = np.clip(log_odds[gz, gx] + L_OCC, L_MIN, L_MAX)

def logodds_to_prob(lo):
    return 1.0 - 1.0 / (1.0 + np.exp(lo))

def build_visual_map(prob_map, explored):
    # occupied -> black, free -> white, unknown -> gray
    vis = np.full(prob_map.shape, 0.5, dtype=np.float32)
    vis[explored] = 1.0 - prob_map[explored]
    return vis

# =========================
# ROS map export
# =========================
def prob_to_ros_occupancy_image(prob_map, explored, occ_thresh=0.65, free_thresh=0.25):
    """
    ROS-style map encoding:
      occupied -> 0
      free     -> 254
      unknown  -> 205
    """
    img = np.full(prob_map.shape, 205, dtype=np.uint8)

    occupied = explored & (prob_map >= occ_thresh)
    free = explored & (prob_map <= free_thresh)
    unknown = ~explored | ((prob_map > free_thresh) & (prob_map < occ_thresh))

    img[occupied] = 0
    img[free] = 254
    img[unknown] = 205

    return img

def save_pgm(path, img):
    h, w = img.shape
    with open(path, "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode("ascii"))
        f.write(img.tobytes())

def save_ros_map_full(prob_map, explored, save_dir,
                      map_res, map_x_min, map_z_min,
                      occ_thresh=0.65, free_thresh=0.25):
    ros_img = prob_to_ros_occupancy_image(prob_map, explored, occ_thresh, free_thresh)
    ros_img_to_save = np.flipud(ros_img)

    pgm_path = os.path.join(save_dir, "map.pgm")
    yaml_path = os.path.join(save_dir, "map.yaml")

    save_pgm(pgm_path, ros_img_to_save)

    yaml_text = f"""image: map.pgm
resolution: {map_res}
origin: [{map_x_min}, {map_z_min}, 0.0]
negate: 0
occupied_thresh: {occ_thresh}
free_thresh: {free_thresh}
"""

    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write(yaml_text)

    return pgm_path, yaml_path

# =========================
# Marker save helpers
# =========================
def save_markers_txt(path):
    with open(path, "w", encoding="utf-8") as f:
        f.write("id,x,z\n")
        for i, (mx, mz) in enumerate(markers_world):
            f.write(f"{i+1},{mx:.6f},{mz:.6f}\n")

def save_marked_png_full(vis_map, markers_world, out_path):
    x_max = MAP_X_MIN + MAP_SIZE_M
    z_max = MAP_Z_MIN + MAP_SIZE_M

    fig2, ax2 = plt.subplots(figsize=(8, 8))
    ax2.imshow(
        vis_map,
        origin="lower",
        cmap="gray",
        vmin=0.0,
        vmax=1.0,
        extent=[MAP_X_MIN, x_max, MAP_Z_MIN, z_max],
    )

    if len(markers_world) > 0:
        pts = np.asarray(markers_world)
        ax2.scatter(pts[:, 0], pts[:, 1], s=40, c="red", marker="o")
        for i, (mx, mz) in enumerate(markers_world):
            ax2.text(mx + 0.15, mz + 0.15, str(i + 1), color="red", fontsize=10)

    ax2.set_aspect("equal", "box")
    ax2.set_title("SLAM Map with Markers")
    ax2.set_xlabel("world x")
    ax2.set_ylabel("world z")
    fig2.tight_layout()
    fig2.savefig(out_path, dpi=200)
    plt.close(fig2)
# =========================
# Save all outputs (FULL MAP, NO CROP)
# =========================
def save_map():
    prob_map = logodds_to_prob(log_odds)
    vis_map = build_visual_map(prob_map, explored_mask)

    png_path = os.path.join(SAVE_DIR, "slam_map_floor2.png")
    marked_png_path = os.path.join(SAVE_DIR, "slam_map_marked_floor2.png")
    npz_path = os.path.join(SAVE_DIR, "slam_map_floor2.npz")
    meta_path = os.path.join(SAVE_DIR, "slam_meta_floor2.txt")
    markers_txt_path = os.path.join(SAVE_DIR, "markers_world_floor2.txt")

    plt.imsave(png_path, vis_map, cmap="gray", vmin=0.0, vmax=1.0)
    save_marked_png_full(vis_map, markers_world, marked_png_path)
    save_markers_txt(markers_txt_path)

    np.savez_compressed(
        npz_path,
        log_odds=log_odds,
        prob_map=prob_map,
        explored_mask=explored_mask.astype(np.uint8),
        vis_map=vis_map,
        markers_world=np.asarray(markers_world, dtype=np.float32),
        map_res=MAP_RES,
        map_size_m=MAP_SIZE_M,
        map_n=MAP_N,
        map_x_min=MAP_X_MIN,
        map_z_min=MAP_Z_MIN,
        trajectory=np.array(trajectory, dtype=np.float32),
    )

    with open(meta_path, "w", encoding="utf-8") as f:
        f.write(f"map_res = {MAP_RES}\n")
        f.write(f"map_size_m = {MAP_SIZE_M}\n")
        f.write(f"map_n = {MAP_N}\n")
        f.write(f"map_x_min = {MAP_X_MIN}\n")
        f.write(f"map_z_min = {MAP_Z_MIN}\n")
        f.write(f"L_FREE = {L_FREE}\n")
        f.write(f"L_OCC = {L_OCC}\n")
        f.write(f"L_MIN = {L_MIN}\n")
        f.write(f"L_MAX = {L_MAX}\n")
        f.write(f"USE_MAX_RANGE_AS_FREE_ONLY = {USE_MAX_RANGE_AS_FREE_ONLY}\n")
        f.write(f"trajectory_points = {len(trajectory)}\n")
        f.write(f"marker_count = {len(markers_world)}\n")
        f.write(f"ros_occ_thresh = {ROS_OCC_THRESH}\n")
        f.write(f"ros_free_thresh = {ROS_FREE_THRESH}\n")

    ros_dir = os.path.join(SAVE_DIR, "ros_map")
    os.makedirs(ros_dir, exist_ok=True)

    pgm_path, yaml_path = save_ros_map_full(
        prob_map,
        explored_mask,
        ros_dir,
        MAP_RES,
        MAP_X_MIN,
        MAP_Z_MIN,
        occ_thresh=ROS_OCC_THRESH,
        free_thresh=ROS_FREE_THRESH,
    )

    print("\n地图已保存（整图，不裁剪）：")
    print(f"  普通地图:      {png_path}")
    print(f"  标注地图:      {marked_png_path}")
    print(f"  marker坐标:    {markers_txt_path}")
    print(f"  数据文件:      {npz_path}")
    print(f"  参数文件:      {meta_path}")
    print(f"  ROS PGM:       {pgm_path}")
    print(f"  ROS YAML:      {yaml_path}")

# =========================
# Main loop
# =========================
print(f"Listening UDP :{PORT} ...")
print("启动后开始累计建图。")
print("在地图窗口里按 7 记录 marker；按 q 保存并退出。")

last_draw = time.time()
packet_count = 0
last_print = time.time()

try:
    while RUNNING:
        try:
            data, _ = sock.recvfrom(65535)
        except socket.timeout:
            plt.pause(0.001)
            continue
        except OSError:
            break

        parsed = parse_packet(data)
        if parsed is None:
            plt.pause(0.001)
            continue

        n, n_layers, angle_min, angle_inc, rmin, rmax, posx, posz, fwdx, fwdz, heights, layer_ranges = parsed
        packet_count += 1

        current_pose["x"] = posx
        current_pose["z"] = posz
        current_pose["yaw"] = np.arctan2(fwdz, fwdx)

        yaw = current_pose["yaw"]

        if yaw0 is None:
            yaw0 = yaw
        yaw_rel = yaw - yaw0

        angles = angle_min + np.arange(n) * angle_inc

        # 1) Update global map with ALL layers
        for k in range(n_layers):
            rr = np.clip(layer_ranges[k], rmin, rmax)
            update_map_with_scan(posx, posz, yaw, angles, rr, rmin, rmax)

        trajectory.append([posx, posz])

        # 2) Left: local lidar display with ALL layers stacked
        all_xp = []
        all_yp = []

        for k in range(n_layers):
            rr = np.clip(layer_ranges[k], rmin, rmax)

            xs_car = rr * np.sin(angles)
            zs_car = rr * np.cos(angles)

            xw_rel, zw_rel = rot2d(xs_car, zs_car, yaw_rel)
            xw = xw_rel + posx
            zw = zw_rel + posz

            xr = xw - posx
            zr = zw - posz
            xc, zc = rot2d(xr, zr, -yaw_rel)

            xp = -xc
            yp = zc

            all_xp.append(xp)
            all_yp.append(yp)

        if n_layers > 0:
            xp_all = np.concatenate(all_xp)
            yp_all = np.concatenate(all_yp)
            sc.set_offsets(np.c_[xp_all, yp_all])
        else:
            sc.set_offsets(np.empty((0, 2)))

        heading_line.set_data([0.0, 0.0], [0.0, 0.9])
        ax_local.set_xlim(-rmax, rmax)
        ax_local.set_ylim(-rmax, rmax)
        yaw_text.set_text(f"yaw_rel(deg) = {np.degrees(yaw_rel):.1f}")
        pose_text.set_text(f"pose = ({posx:.2f}, {posz:.2f})")
        layer_text.set_text(f"layers = {n_layers}, heights = {np.round(heights, 2)}")

        # 3) Right: global map display
        now = time.time()
        if now - last_draw > 0.25:
            prob_map = logodds_to_prob(log_odds)
            vis_map = build_visual_map(prob_map, explored_mask)
            map_img.set_data(vis_map)

            if len(trajectory) > 1:
                traj = np.asarray(trajectory)
                traj_line.set_data(traj[:, 0], traj[:, 1])

            update_marker_artist()
            fig.canvas.draw_idle()
            plt.pause(0.001)
            last_draw = now

        if now - last_print > 1.0:
            explored_count = int(explored_mask.sum())
            print(
                f"packets={packet_count}, "
                f"pose=({posx:.2f},{posz:.2f}), "
                f"layers={n_layers}, "
                f"heights={np.round(heights, 2)}, "
                f"explored={explored_count}, "
                f"markers={len(markers_world)}"
            )
            last_print = now

except Exception as e:
    print(f"运行过程中出现异常: {e}")

finally:
    try:
        save_map()
    finally:
        try:
            sock.close()
        except Exception:
            pass
        plt.ioff()
        plt.close("all")