#!/usr/bin/env python3
"""把 vio_online 的 TUM 轨迹发布成 ROS2 话题，给 rviz2 看。

支持两种模式：
  replay  读完整文件，按文件里的时间戳节奏回放（可加速/循环）
  live    边写边看：文件里每多一行就发一帧（给 vio_live / vio_offline 正在跑的时候用）

发布：
  /vio/path      nav_msgs/Path           累积轨迹
  /vio/pose      geometry_msgs/PoseStamped 当前位姿
  /vio/markers   visualization_msgs/MarkerArray  起点球 + 漂移文字
  TF             world -> base_link

注意：TUM 里的位姿是 LOCAL_FRD（x 前 / y 右 / z 下），所以 RViz 里 z 轴朝下；
      Grid 仍在 world 的 XY 平面（水平面），不影响看轨迹。加 --flu 转成 z 向上的
      FLU 显示（x 前 / y 左 / z 上，y 左右会镜像）。

运行（必须用系统 Python 3.10 + ROS2 Humble，不要在 conda 环境里跑）：
    source /opt/ros/humble/setup.bash
    /usr/bin/python3 tools/tum_to_rviz.py --tum out/session18_traj.tum --mode live
"""
from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


def read_tum(path: Path):
    rows = []
    try:
        text = path.read_text()
    except FileNotFoundError:
        return rows
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        f = line.split()
        if len(f) < 8:
            continue
        try:
            rows.append((float(f[0]), *map(float, f[1:8])))
        except ValueError:
            continue
    return rows


def yaw_deg(q) -> float:
    x, y, z, w = q
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


class TumToRviz(Node):
    def __init__(self, args):
        super().__init__("vio_tum_to_rviz")
        self.args = args
        qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.pub_path = self.create_publisher(PathMsg, "/vio/path", qos)
        self.pub_pose = self.create_publisher(PoseStamped, "/vio/pose", qos)
        self.pub_markers = self.create_publisher(MarkerArray, "/vio/markers", qos)
        self.tf = TransformBroadcaster(self)
        self.path = PathMsg()
        self.path.header.frame_id = args.frame
        self.first = None
        self.last_pub = 0.0
        self.get_logger().info(f"frame={args.frame} child={args.child} "
                               f"mode={args.mode} file={args.tum}")

    def _markers(self, t, q, drift, yaw):
        arr = MarkerArray()
        start = Marker()
        start.header.frame_id = self.args.frame
        start.header.stamp = self.get_clock().now().to_msg()
        start.ns, start.id, start.type = "vio", 0, Marker.SPHERE
        start.action = Marker.ADD
        start.pose.position.x, start.pose.position.y, start.pose.position.z = (
            self.first[0], self.first[1], self.first[2])
        start.pose.orientation.w = 1.0
        start.scale.x = start.scale.y = start.scale.z = 0.06
        start.color.r, start.color.g, start.color.b, start.color.a = 1.0, 0.35, 0.1, 0.9
        arr.markers.append(start)

        text = Marker()
        text.header.frame_id = self.args.frame
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns, text.id, text.type = "vio", 1, Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = self.first[0] + 0.35
        text.pose.position.y = self.first[1]
        text.pose.position.z = self.first[2] - 0.25
        text.pose.orientation.w = 1.0
        text.scale.z = 0.12
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = f"drift {drift*100:+.1f} cm   yaw {yaw:+.1f}°"
        arr.markers.append(text)
        return arr

    def publish(self, row):
        t, x, y, z, qx, qy, qz, qw = row
        if self.args.flu:
            # FRD -> FLU：世界系 y,z 翻转（M = diag(1,-1,-1) = 绕 x 轴 180°）
            # p' = M p ; q' = q_M ⊗ q，q_M = (x=1, y=0, z=0, w=0)
            y, z = -y, -z
            qx, qy, qz, qw = qw, -qz, qy, -qx
        now = self.get_clock().now().to_msg()
        if self.first is None:
            self.first = (x, y, z)

        ps = PoseStamped()
        ps.header.frame_id = self.args.frame
        ps.header.stamp = now
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
        ps.pose.orientation.x, ps.pose.orientation.y = qx, qy
        ps.pose.orientation.z, ps.pose.orientation.w = qz, qw
        self.pub_pose.publish(ps)

        tfm = TransformStamped()
        tfm.header.frame_id = self.args.frame
        tfm.header.stamp = now
        tfm.child_frame_id = self.args.child
        tfm.transform.translation.x, tfm.transform.translation.y = x, y
        tfm.transform.translation.z = z
        tfm.transform.rotation.x, tfm.transform.rotation.y = qx, qy
        tfm.transform.rotation.z, tfm.transform.rotation.w = qz, qw
        self.tf.sendTransform(tfm)

        self.path.header.stamp = now
        self.path.poses.append(ps)
        if len(self.path.poses) > 20000:
            self.path.poses = self.path.poses[-20000:]
        self.pub_path.publish(self.path)

        drift = math.dist((x, y, z), self.first)
        yaw = yaw_deg((qx, qy, qz, qw))
        yaw0 = yaw_deg((self.path.poses[0].pose.orientation.x,
                        self.path.poses[0].pose.orientation.y,
                        self.path.poses[0].pose.orientation.z,
                        self.path.poses[0].pose.orientation.w))
        d = (yaw - yaw0 + 180.0) % 360.0 - 180.0
        self.pub_markers.publish(self._markers(t, (qx, qy, qz, qw), drift, d))

    def run_replay(self):
        rows = read_tum(self.args.tum)
        if not rows:
            self.get_logger().error("TUM 为空或读不出")
            return
        self.get_logger().info(f"replay {len(rows)} poses, speed {self.args.speed}")
        while rclpy.ok():
            t0 = rows[0][0]
            for row in rows:
                if not rclpy.ok():
                    break
                delay = (row[0] - t0) / max(self.args.speed, 1e-3)
                # 只在时间戳是合理的连续量时按节奏；否则全速
                if 0 <= delay < 3600:
                    target = self._replay_start + delay
                    while rclpy.ok() and time.time() < target:
                        rclpy.spin_once(self, timeout_sec=0.001)
                self.publish(row)
            if not self.args.loop:
                break
            self._replay_start = time.time()
        # 最后保持 0.5 s，保证 RViz 收到
        end = time.time() + 0.5
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def run_live(self):
        path = Path(self.args.tum)
        pos = 0
        partial = ""
        self.get_logger().info("live: 等待文件增长（Ctrl-C 退出）")
        while rclpy.ok():
            if path.exists():
                size = path.stat().st_size
                if pos > size:      # 文件被重新创建/截断（VIO 重启）
                    pos = 0
                    partial = ""
                with path.open("r", errors="replace") as fh:
                    fh.seek(pos)
                    chunk = fh.read()
                    pos = fh.tell()
                partial += chunk
                lines = partial.splitlines(keepends=True)
                partial = ""
                for line in lines:
                    if not line.endswith("\n"):
                        partial = line
                        continue
                    s = line.strip()
                    if not s or s.startswith("#"):
                        continue
                    f = s.split()
                    if len(f) < 8:
                        continue
                    try:
                        row = (float(f[0]), *map(float, f[1:8]))
                    except ValueError:
                        continue
                    self.publish(row)
            rclpy.spin_once(self, timeout_sec=self.args.poll)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tum", required=True, type=Path)
    ap.add_argument("--mode", choices=["replay", "live"], default="live")
    ap.add_argument("--speed", type=float, default=1.0, help="replay 速度倍率")
    ap.add_argument("--loop", action="store_true")
    ap.add_argument("--frame", default="world")
    ap.add_argument("--child", default="base_link")
    ap.add_argument("--flu", action="store_true", help="转成 z 向上的 FLU 显示")
    ap.add_argument("--poll", type=float, default=0.05, help="live 模式轮询间隔 s")
    args = ap.parse_args()

    rclpy.init()
    node = TumToRviz(args)
    node._replay_start = time.time()
    try:
        if args.mode == "replay":
            node.run_replay()
        else:
            node.run_live()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
