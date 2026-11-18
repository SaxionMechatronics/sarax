#!/usr/bin/env python3
"""
Normal Selector
---------------
Subscribes to the full PointNormal PointCloud2 published by the
mls_normal_node or pca_normal_node, uniformly samples `num_arrows`
normals, displays them in RViz2 with index labels, and lets the
user pick one by number.

The selected normal is:
  - highlighted green in RViz
  - saved to `output_file` (default ~/selected_normal_tf.yaml)
  - printed to the terminal

Usage
-----
  pixi run select_v1                      # uses /pca_normals
  pixi run select_mls_v1                  # uses /mls_normals
  ros2 run sarax_perception normal_selector_v1.py --ros-args -p input_topic:=/mls_normals

Output YAML
-----------
  selected_index: 7
  frame_id: camera_link
  position: {x: 0.1234, y: 0.5678, z: 0.9012}
  normal:   {nx: -0.9945, ny: 0.0123, nz: 0.1034}

The v1 selection UI is retained. TF at the cloud timestamp supplies the
point-and-normal transform to target_frame (default world). The temporary
world uses the base's axes, not PX4 NED; results use a separate output file.
"""

import math
import os
import struct
import threading
import time
import numpy as np

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, JointState
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import tf2_ros

import yaml


FRESH_CLOUD_TOPIC = "/camera/points_filtered"


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────

def _parse_point_normals(msg: PointCloud2):
    """Return list of (x,y,z,nx,ny,nz) from a pcl::PointNormal cloud."""
    fields = {f.name: f.offset for f in msg.fields}
    needed = ('x', 'y', 'z', 'normal_x', 'normal_y', 'normal_z')
    if not all(n in fields for n in needed):
        return []

    ox  = fields['x']
    oy  = fields['y']
    oz  = fields['z']
    onx = fields['normal_x']
    ony = fields['normal_y']
    onz = fields['normal_z']

    pts = []
    data       = msg.data
    point_step = msg.point_step
    row_step   = msg.row_step

    for row in range(msg.height):
        for col in range(msg.width):
            base = row * row_step + col * point_step
            x  = struct.unpack_from('<f', data, base + ox)[0]
            y  = struct.unpack_from('<f', data, base + oy)[0]
            z  = struct.unpack_from('<f', data, base + oz)[0]
            nx = struct.unpack_from('<f', data, base + onx)[0]
            ny = struct.unpack_from('<f', data, base + ony)[0]
            nz = struct.unpack_from('<f', data, base + onz)[0]
            if not (math.isfinite(x) and math.isfinite(nx)):
                continue
            pts.append((x, y, z, nx, ny, nz))
    return pts


def _stamp_to_sec(stamp):
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


def _quat_to_rot(qx, qy, qz, qw):
    return np.array([
        [1.0 - 2.0*(qy*qy + qz*qz), 2.0*(qx*qy - qz*qw),       2.0*(qx*qz + qy*qw)],
        [2.0*(qx*qy + qz*qw),       1.0 - 2.0*(qx*qx + qz*qz), 2.0*(qy*qz - qx*qw)],
        [2.0*(qx*qz - qy*qw),       2.0*(qy*qz + qx*qw),       1.0 - 2.0*(qx*qx + qy*qy)],
    ])


_FOREVER = rclpy.duration.Duration(seconds=0).to_msg()


def _arrow_marker(frame_id, ns, idx, x, y, z, nx, ny, nz,
                  scale, r, g, b) -> Marker:
    m = Marker()
    m.header.frame_id = frame_id
    m.ns     = ns
    m.id     = idx
    m.type   = Marker.ARROW
    m.action = Marker.ADD
    m.points = [
        Point(x=float(x), y=float(y), z=float(z)),
        Point(x=float(x + nx * scale),
              y=float(y + ny * scale),
              z=float(z + nz * scale)),
    ]
    m.scale.x = 0.007   # shaft diameter
    m.scale.y = 0.016   # head diameter
    m.scale.z = 0.0
    m.color.r = float(r)
    m.color.g = float(g)
    m.color.b = float(b)
    m.color.a = 1.0
    m.lifetime = _FOREVER
    return m


def _text_marker(frame_id, ns, idx, x, y, z, nx, ny, nz,
                 arrow_scale, text, r=0.0, g=0.0, b=0.0) -> Marker:
    """Place the label near the arrow tip."""
    along  = arrow_scale * 0.7
    up     = 0.03
    m = Marker()
    m.header.frame_id = frame_id
    m.ns     = ns
    m.id     = idx
    m.type   = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.pose.position.x = float(x + nx * along)
    m.pose.position.y = float(y + ny * along - up)
    m.pose.position.z = float(z + nz * along)
    m.pose.orientation.w = 1.0
    m.scale.z = 0.09
    m.color.r = float(r)
    m.color.g = float(g)
    m.color.b = float(b)
    m.color.a = 1.0
    m.text    = text
    m.lifetime = _FOREVER
    return m


# ──────────────────────────────────────────────────────────────────────────────
# Node
# ──────────────────────────────────────────────────────────────────────────────

class NormalSelector(Node):

    _ARROW_NS = 'sel_arrows'
    _LABEL_NS = 'sel_labels'

    def __init__(self):
        super().__init__('normal_selector')

        self.declare_parameter('input_topic',  '/pca_normals')
        self.declare_parameter('num_arrows',   20)
        self.declare_parameter('arrow_scale',  0.15)
        self.declare_parameter('target_frame', 'world')
        self.declare_parameter('standoff', 0.05)
        self.declare_parameter('preplan_distance', 1.8)
        self.declare_parameter('preplan_alt_offset', 0.3)
        self.declare_parameter('preplan_min_alt', 0.8)
        self.declare_parameter('fresh_cloud_topic', FRESH_CLOUD_TOPIC)
        self.declare_parameter('output_file',
                               os.path.expanduser('~/selected_normal_tf.yaml'))

        self._topic       = self.get_parameter('input_topic').value
        self._fresh_cloud_topic = self.get_parameter('fresh_cloud_topic').value
        self._num_arrows  = self.get_parameter('num_arrows').value
        self._arrow_scale = self.get_parameter('arrow_scale').value
        self._target_frame = self.get_parameter('target_frame').value
        self._standoff = float(self.get_parameter('standoff').value)
        self._preplan_distance = float(self.get_parameter('preplan_distance').value)
        self._preplan_alt_offset = float(self.get_parameter('preplan_alt_offset').value)
        self._preplan_min_alt = float(self.get_parameter('preplan_min_alt').value)
        self._out_file    = self.get_parameter('output_file').value

        self._normals    = None   # list[(x,y,z,nx,ny,nz)]  after first frame
        self._frame_id   = ''
        self._markers_ma = MarkerArray()
        self._input_cloud_stamp = None
        self._input_cloud_stamp_msg = None
        self._input_cloud_frame_id = ''
        self._fresh_cloud_stamp = None
        self._fresh_cloud_frame_id = ''
        self._lock       = threading.Lock()

        # Use the captured cloud's pose, not the current camera pose.
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Latest arm joint positions from /joint_states
        self._joint_pos = {}
        self._joint_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_states_cb, 10)

        self._sub = self.create_subscription(
            PointCloud2, self._topic, self._cloud_cb, 10)

        self._fresh_cloud_sub = self.create_subscription(
            PointCloud2, self._fresh_cloud_topic, self._fresh_cloud_cb, 10)

        self._pub = self.create_publisher(
            MarkerArray, '/normal_selector/markers', 10)

        self._timer = self.create_timer(1.0, self._republish)

        threading.Thread(target=self._input_loop, daemon=True).start()

        self.get_logger().info(
            f"Waiting for PointCloud2 on '{self._topic}'; transform to "
            f"'{self._target_frame}' through TF "
            f"(freshness checked against '{self._fresh_cloud_topic}') ...")

    # ── subscriptions ─────────────────────────────────────────────────────── #

    def _joint_states_cb(self, msg: JointState):
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                self._joint_pos[name] = pos

    def _point_cloud_transform(self):
        with self._lock:
            frame_id = self._frame_id
            stamp = self._input_cloud_stamp_msg
        transform = self._tf_buffer.lookup_transform(
            self._target_frame, frame_id, rclpy.time.Time.from_msg(stamp),
            timeout=rclpy.duration.Duration(seconds=1.0))
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        t = np.array([translation.x, translation.y, translation.z], dtype=float)
        q = np.array([rotation.x, rotation.y, rotation.z, rotation.w], dtype=float)
        q /= np.linalg.norm(q)
        return t, q, _quat_to_rot(*q)

    def _fresh_cloud_cb(self, msg: PointCloud2):
        with self._lock:
            self._fresh_cloud_stamp = _stamp_to_sec(msg.header.stamp)
            self._fresh_cloud_frame_id = msg.header.frame_id or ''

    def _check_cloud_freshness(self):
        if not self._fresh_cloud_topic or self._fresh_cloud_topic == self._topic:
            return None

        wait_s = float(os.environ.get("SARAX_FRESH_CLOUD_WAIT_S", "1.0"))
        deadline = time.time() + wait_s
        while rclpy.ok():
            with self._lock:
                input_stamp = self._input_cloud_stamp
                input_frame = self._input_cloud_frame_id
                fresh_stamp = self._fresh_cloud_stamp
                fresh_frame = self._fresh_cloud_frame_id
            if fresh_stamp is not None:
                break
            if time.time() >= deadline:
                raise RuntimeError(
                    f"no live cloud received on {self._fresh_cloud_topic}; "
                    "cannot verify that the selected normal cloud is fresh"
                )
            time.sleep(0.05)

        if input_stamp is None or fresh_stamp is None:
            raise RuntimeError("missing cloud timestamps; cannot verify normal freshness")

        max_stale_s = float(os.environ.get("SARAX_MLS_MAX_STALENESS_S", "3.0"))
        live_minus_selected = fresh_stamp - input_stamp
        info = {
            'selected_topic': self._topic,
            'selected_stamp_s': round(float(input_stamp), 6),
            'selected_frame_id': input_frame,
            'live_topic': self._fresh_cloud_topic,
            'live_stamp_s': round(float(fresh_stamp), 6),
            'live_frame_id': fresh_frame,
            'live_minus_selected_s': round(float(live_minus_selected), 6),
            'max_allowed_s': round(float(max_stale_s), 6),
        }
        action = os.environ.get("SARAX_MLS_STALE_ACTION", "warn").strip().lower()
        info['status'] = 'ok'
        if live_minus_selected > max_stale_s:
            info['status'] = 'older_than_live_cloud'
            info['message'] = (
                f"{self._topic} stamp={input_stamp:.3f}s, "
                f"{self._fresh_cloud_topic} stamp={fresh_stamp:.3f}s "
                f"(older by {live_minus_selected:.1f}s; nominal limit {max_stale_s:.1f}s). "
                "This is expected if MLS spent time computing after capturing the scan. "
                "Use it only if the drone stayed at the scan pose while MLS computed."
            )
            if action == "reject":
                raise RuntimeError(
                    "selected normal cloud is older than the live camera cloud: "
                    + info['message']
                )
            if action != "ignore":
                self.get_logger().warn(info['message'])
        return info

    def _cloud_cb(self, msg: PointCloud2):
        with self._lock:
            if self._normals is not None:
                return

        if not msg.header.frame_id:
            self.get_logger().warn('Cloud has no frame_id; cannot transform it.')
            return

        pts = _parse_point_normals(msg)
        if not pts:
            self.get_logger().warn('Received empty or unreadable cloud, retrying ...')
            return

        step    = max(1, len(pts) // self._num_arrows)
        sampled = pts[::step][:self._num_arrows]

        with self._lock:
            self._normals  = sampled
            self._frame_id = msg.header.frame_id
            self._input_cloud_stamp = _stamp_to_sec(msg.header.stamp)
            self._input_cloud_stamp_msg = msg.header.stamp
            self._input_cloud_frame_id = self._frame_id

        self.get_logger().info(
            f'Sampled {len(sampled)} arrows from {len(pts)} normals.')
        self._build_markers()

    # ── marker building ───────────────────────────────────────────────────── #

    def _build_markers(self, highlight: int = -1):
        ma = MarkerArray()

        del_m = Marker()
        del_m.action = Marker.DELETEALL
        del_m.header.frame_id = self._frame_id
        ma.markers.append(del_m)

        with self._lock:
            normals   = list(self._normals)
            frame_id  = self._frame_id
            scale     = self._arrow_scale

        for i, (x, y, z, nx, ny, nz) in enumerate(normals):
            if i == highlight:
                r, g, b = 0.0, 1.0, 0.0   # green = selected
                tr, tg, tb = 1.0, 1.0, 1.0
            else:
                r, g, b = 1.0, 0.8, 0.0   # yellow = candidate
                tr, tg, tb = 0.0, 0.0, 0.0

            ma.markers.append(
                _arrow_marker(frame_id, self._ARROW_NS, i,
                              x, y, z, nx, ny, nz, scale, r, g, b))
            ma.markers.append(
                _text_marker(frame_id, self._LABEL_NS, i,
                             x, y, z, nx, ny, nz,
                             scale, str(i), tr, tg, tb))

        with self._lock:
            self._markers_ma = ma

    # ── timer: republish ──────────────────────────────────────────────────── #

    def _republish(self):
        with self._lock:
            if not self._markers_ma.markers:
                return
            ma = self._markers_ma
        self._pub.publish(ma)

    # ── interactive terminal loop ─────────────────────────────────────────── #

    def _input_loop(self):
        # Wait for first frame
        while rclpy.ok():
            with self._lock:
                ready = self._normals is not None
            if ready:
                break
            time.sleep(0.2)

        with self._lock:
            n = len(self._normals)

        sep = '─' * 56
        print(f'\n{sep}')
        print(f'  {n} normals ready — topic: {self._topic}')
        print(f'  Arrows visible in RViz on  /normal_selector/markers')
        print(f'    Add → MarkerArray → Topic: /normal_selector/markers')
        print(f'  Labels 0 – {n - 1} float above each arrow tip.')
        print(f'{sep}')

        while rclpy.ok():
            try:
                raw = input(f'  Enter index [0-{n-1}] or "q" to quit: ').strip()
            except (EOFError, KeyboardInterrupt):
                break

            if raw.lower() in ('q', 'quit', 'exit'):
                print('  Exiting selector.')
                break

            try:
                idx = int(raw)
            except ValueError:
                print(f'  Please enter a number between 0 and {n - 1}.')
                continue

            if not (0 <= idx < n):
                print(f'  Index {idx} out of range [0, {n - 1}].')
                continue

            with self._lock:
                x, y, z, nx, ny, nz = self._normals[idx]
                frame_id   = self._frame_id
                joint_snap = dict(self._joint_pos)

            self._build_markers(highlight=idx)

            result = {
                'selected_index': int(idx),
                'frame_id': frame_id,
                'position': {
                    'x': round(float(x),  6),
                    'y': round(float(y),  6),
                    'z': round(float(z),  6),
                },
                'normal': {
                    'nx': round(float(nx), 6),
                    'ny': round(float(ny), 6),
                    'nz': round(float(nz), 6),
                },
            }

            # TF supplies the source frame's pose at the captured cloud stamp.
            try:
                cloud_freshness = self._check_cloud_freshness()
                pc_t, pc_q, R_pc = self._point_cloud_transform()
                p_world = pc_t + R_pc @ np.array([x, y, z])
                n_world = R_pc @ np.array([nx, ny, nz])
                n_world = n_world / np.linalg.norm(n_world)
                standoff = self._standoff
                ee_target = p_world + standoff * n_world
                result['camera_to_world'] = {
                    'frame_convention': 'TF',
                    'source': 'tf2_at_cloud_stamp',
                    'source_frame': frame_id,
                    'target_frame': self._target_frame,
                    'cloud_stamp_s': round(float(self._input_cloud_stamp), 6),
                    'translation': {
                        'x': round(float(pc_t[0]), 6),
                        'y': round(float(pc_t[1]), 6),
                        'z': round(float(pc_t[2]), 6),
                    },
                    'quaternion': {
                        'x': round(float(pc_q[0]), 6),
                        'y': round(float(pc_q[1]), 6),
                        'z': round(float(pc_q[2]), 6),
                        'w': round(float(pc_q[3]), 6),
                    },
                }
                result['point_cloud_to_world'] = {
                    'frame_convention': 'TF',
                    'mode': 'tf',
                    'source_frame': frame_id,
                    'target_frame': self._target_frame,
                    'translation': {
                        'x': round(float(pc_t[0]), 6),
                        'y': round(float(pc_t[1]), 6),
                        'z': round(float(pc_t[2]), 6),
                    },
                    'matrix_row_major': [
                        round(float(v), 6) for v in R_pc.reshape(-1)
                    ],
                }
                if cloud_freshness is not None:
                    result['cloud_freshness'] = cloud_freshness
                result['world_frame'] = self._target_frame
                result['world_contact'] = {
                    'x': round(float(p_world[0]), 6),
                    'y': round(float(p_world[1]), 6),
                    'z': round(float(p_world[2]), 6),
                }
                result['world_normal'] = {
                    'x': round(float(n_world[0]), 6),
                    'y': round(float(n_world[1]), 6),
                    'z': round(float(n_world[2]), 6),
                }
                result['ee_target_world'] = {
                    'x': round(float(ee_target[0]), 6),
                    'y': round(float(ee_target[1]), 6),
                    'z': round(float(ee_target[2]), 6),
                }
                # Retain the geometric staging calculation in the Z-up test frame.
                suggested = p_world + self._preplan_distance * n_world
                suggested[2] = max(
                    self._preplan_min_alt,
                    float(p_world[2]) + self._preplan_alt_offset,
                )
                yaw = math.atan2(float(n_world[1]), float(n_world[0]))
                result['suggested_preplan_pose'] = {
                    'frame_id': self._target_frame,
                    'x': round(float(suggested[0]), 6),
                    'y': round(float(suggested[1]), 6),
                    'z': round(float(suggested[2]), 6),
                    'yaw_deg': round(float(math.degrees(yaw)), 3),
                    'distance_from_contact_m': round(self._preplan_distance, 3),
                    'note': 'Geometry in the Z-up target frame; not a PX4 NED flight command.',
                }
            except Exception as e:
                self.get_logger().error(f'V1 selection validation failed: {e}')
                print(f'  Could not transform selection to {self._target_frame}: {e}')
                continue

            # Arm joint angles at pick time.
            arm_joints = {}
            for jname in ('mani_joint_1', 'mani_joint_2'):
                if jname in joint_snap:
                    arm_joints[jname + '_rad'] = round(float(joint_snap[jname]), 6)
            if arm_joints:
                result['arm_joints'] = arm_joints

            os.makedirs(os.path.dirname(self._out_file) or '.', exist_ok=True)
            with open(self._out_file, 'w') as f:
                yaml.dump(result, f, default_flow_style=False, sort_keys=False)

            print(f'\n  ✓ Arrow {idx} selected:')
            print(f'      position : ({x:.4f},  {y:.4f},  {z:.4f})')
            print(f'      normal   : ({nx:.4f}, {ny:.4f}, {nz:.4f})')
            if 'camera_to_world' in result:
                ctw = result['camera_to_world']['translation']
                source = result['camera_to_world'].get('source', 'unknown')
                print(f'      cam pos  : ({ctw["x"]:.3f}, {ctw["y"]:.3f}, {ctw["z"]:.3f})  [{self._target_frame}, {source}]')
            if 'point_cloud_to_world' in result:
                print(f'      pc mode  : {result["point_cloud_to_world"]["mode"]}')
            if 'cloud_freshness' in result:
                cf = result['cloud_freshness']
                print(
                    f'      cloud dt : {cf["live_minus_selected_s"]:+.2f} s '
                    f'({cf["selected_topic"]} → {cf["live_topic"]})'
                )
                if cf.get('status') != 'ok':
                    print('      cloud note: older than live cloud; OK only if drone held scan pose during MLS')
            if 'world_contact' in result:
                wc = result['world_contact']
                wn = result['world_normal']
                et = result['ee_target_world']
                print(f'      contact  : ({wc["x"]:.3f}, {wc["y"]:.3f}, {wc["z"]:.3f})  [{self._target_frame}]')
                print(f'      normal W : ({wn["x"]:.3f}, {wn["y"]:.3f}, {wn["z"]:.3f})  [{self._target_frame}]')
                print(f'      EE target: ({et["x"]:.3f}, {et["y"]:.3f}, {et["z"]:.3f})  [{self._target_frame}, {standoff*100:.0f} cm off]')
            if 'suggested_preplan_pose' in result:
                sp = result['suggested_preplan_pose']
                print(f'      suggested pose in {self._target_frame} (geometry only):')
                print(f'        x     : {sp["x"]:+.3f} m')
                print(f'        y     : {sp["y"]:+.3f} m')
                print(f'        z     : {sp["z"]:+.3f} m')
                print(f'        yaw   : {sp["yaw_deg"]:+.1f} °')
            if arm_joints:
                for k, v in arm_joints.items():
                    print(f'      {k:<25}: {math.degrees(v):.2f} °')
            print(f'      saved to : {self._out_file}')
            print()


# ──────────────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = NormalSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
