#!/usr/bin/env python3
"""Random goal runner for Nav2 RIL validation.

Samples random goals from the drivable (free) area of the 2D occupancy map,
sends them one by one to Nav2, and records per-trial metrics to CSV.
"""

import argparse
import csv
import math
import os
import random
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


# Nav2 publishes OccupancyGrid on 0-100 scale:
#   253 (inscribed inflated) -> 99,  254 (lethal) -> 100.
# Use 99 so we catch both: robot body touching an obstacle.
LETHAL_THRESHOLD = 99


@dataclass
class TrialResult:
    trial: int
    goal_x: float
    goal_y: float
    goal_yaw: float
    result: str
    duration_s: float
    path_length_m: float
    straight_dist_m: float
    path_efficiency: float
    final_pos_err_m: float
    final_yaw_err_rad: float
    num_recoveries: int
    num_collisions: int
    failure_code: int


class RandomGoalRunner(Node):
    def __init__(self, args):
        super().__init__('random_goal_runner')
        self.args = args

        self.map_array, self.map_info = self._load_map(args.map_yaml)
        self.free_pixels = self._compute_free_pixels(
            self.map_array, args.robot_radius
        )
        if not self.free_pixels:
            raise RuntimeError(
                'No free pixels after erosion. '
                'Reduce robot_radius or check map.'
            )
        self.get_logger().info(
            f'Map loaded: {self.map_array.shape}, '
            f'free pixels after {args.robot_radius}m erosion: '
            f'{len(self.free_pixels)}'
        )

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # BEST_EFFORT to match sensor-data style odometry publishers
        # (lidar_localization publishes /odometry/imu as BEST_EFFORT).
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # Nav2 costmaps are published with TRANSIENT_LOCAL RELIABLE.
        costmap_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Odometry, args.odom_topic, self._odom_cb, odom_qos
        )
        self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap',
            self._local_costmap_cb, costmap_qos,
        )

        self.latest_odom = None
        self.prev_odom_pos = None
        self.path_length = 0.0
        self.collision_count = 0
        self.in_collision = False
        self.local_costmap = None
        self.trial_active = False
        self.latest_feedback_pose = None
        self.num_recoveries = 0

    # ------------------ map / sampling ------------------
    def _load_map(self, yaml_path):
        with open(yaml_path, 'r') as f:
            info = yaml.safe_load(f)
        img_path = info['image']
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(yaml_path), img_path)
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise RuntimeError(f'Failed to load map image: {img_path}')
        return img, info

    def _compute_free_pixels(self, img, robot_radius):
        res = float(self.map_info['resolution'])
        erode_px = max(1, int(math.ceil(robot_radius / res)))
        free_mask = (img > 250).astype(np.uint8) * 255
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2 * erode_px + 1, 2 * erode_px + 1)
        )
        eroded = cv2.erode(free_mask, kernel)
        ys, xs = np.where(eroded > 0)
        return list(zip(xs.tolist(), ys.tolist()))

    def _pixel_to_world(self, px, py):
        res = float(self.map_info['resolution'])
        ox, oy = self.map_info['origin'][0], self.map_info['origin'][1]
        h = self.map_array.shape[0]
        wx = ox + px * res
        wy = oy + (h - py) * res
        return wx, wy

    # ------------------ callbacks ------------------
    def _odom_cb(self, msg):
        self.latest_odom = msg
        if self.trial_active:
            p = msg.pose.pose.position
            if self.prev_odom_pos is not None:
                dx = p.x - self.prev_odom_pos[0]
                dy = p.y - self.prev_odom_pos[1]
                self.path_length += math.hypot(dx, dy)
            self.prev_odom_pos = (p.x, p.y)
        else:
            self.prev_odom_pos = None

    def _local_costmap_cb(self, msg):
        self.local_costmap = msg

    def _check_collision(self):
        # local_costmap (odom frame) vs /odometry/imu (odom frame) — same frame
        if self.local_costmap is None or self.latest_odom is None:
            return
        cm = self.local_costmap
        rx = self.latest_odom.pose.pose.position.x
        ry = self.latest_odom.pose.pose.position.y
        px = int((rx - cm.info.origin.position.x) / cm.info.resolution)
        py = int((ry - cm.info.origin.position.y) / cm.info.resolution)
        if 0 <= px < cm.info.width and 0 <= py < cm.info.height:
            idx = py * cm.info.width + px
            val = cm.data[idx]
            if val >= LETHAL_THRESHOLD:
                if not self.in_collision:
                    self.collision_count += 1
                    self.in_collision = True
                return
        self.in_collision = False

    # ------------------ main loop ------------------
    def run(self):
        output_path = self._prepare_output()
        self.get_logger().info('Waiting for Nav2 action server...')
        if not self.nav_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('Nav2 action server not available.')
            return
        self.get_logger().info(f'Output CSV: {output_path}')

        # Warm up: need odom before we can compute start pose
        deadline = time.time() + 10.0
        while self.latest_odom is None and time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_odom is None:
            self.get_logger().error(
                f'No odom on {self.args.odom_topic}. '
                'Is lidar_localization running?'
            )
            return

        results = []
        for i in range(self.args.num_goals):
            self.get_logger().info(
                f'=== Trial {i + 1}/{self.args.num_goals} ==='
            )
            result = self._run_single_trial(i + 1)
            results.append(result)
            self._append_csv(output_path, result, write_header=(i == 0))
            self.get_logger().info(
                f'Trial {i + 1}: {result.result} '
                f'(t={result.duration_s}s, dist={result.path_length_m}m, '
                f'eff={result.path_efficiency}, rec={result.num_recoveries}, '
                f'col={result.num_collisions})'
            )

        self._print_summary(results, output_path)

    def _run_single_trial(self, trial_num):
        px, py = random.choice(self.free_pixels)
        gx, gy = self._pixel_to_world(px, py)
        yaw = random.uniform(-math.pi, math.pi)

        start = self.latest_odom.pose.pose.position
        straight = math.hypot(gx - start.x, gy - start.y)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = gx
        goal_msg.pose.pose.position.y = gy
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # Reset per-trial state
        self.path_length = 0.0
        self.collision_count = 0
        self.in_collision = False
        self.prev_odom_pos = None
        self.latest_feedback_pose = None
        self.num_recoveries = 0
        self.trial_active = True

        def feedback_cb(fb):
            self.num_recoveries = fb.feedback.number_of_recoveries
            self.latest_feedback_pose = fb.feedback.current_pose
            self._check_collision()

        t0 = time.time()
        send_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=feedback_cb
        )
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.trial_active = False
            return TrialResult(
                trial_num, gx, gy, yaw, 'REJECTED',
                0.0, 0.0, round(straight, 2), 0.0, 0.0, 0.0,
                0, 0, -1,
            )

        result_future = goal_handle.get_result_async()
        timed_out = False
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            self._check_collision()
            if result_future.done():
                break
            if (time.time() - t0) > self.args.timeout:
                self.get_logger().warn(
                    f'Trial {trial_num} timed out, cancelling goal.'
                )
                goal_handle.cancel_goal_async()
                # Give the server a moment to respond to cancel
                cancel_deadline = time.time() + 5.0
                while rclpy.ok() and not result_future.done() \
                        and time.time() < cancel_deadline:
                    rclpy.spin_once(self, timeout_sec=0.1)
                timed_out = True
                break

        self.trial_active = False
        duration = time.time() - t0

        failure_code = 0
        if result_future.done():
            res = result_future.result()
            status = res.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                outcome = 'SUCCESS'
            elif status == GoalStatus.STATUS_ABORTED:
                outcome = 'ABORTED'
            elif status == GoalStatus.STATUS_CANCELED:
                outcome = 'TIMEOUT' if timed_out else 'CANCELED'
            else:
                outcome = f'STATUS_{status}'
            failure_code = int(getattr(res.result, 'error_code', 0) or 0)
        else:
            outcome = 'TIMEOUT'
            failure_code = -1

        # Final pose error: use feedback.current_pose (map frame) if available,
        # fall back to odom position (odom frame — less accurate but usable).
        if self.latest_feedback_pose is not None:
            fp = self.latest_feedback_pose.pose
            fx, fy = fp.position.x, fp.position.y
            fyaw = self._quat_to_yaw(fp.orientation)
        else:
            fp = self.latest_odom.pose.pose
            fx, fy = fp.position.x, fp.position.y
            fyaw = self._quat_to_yaw(fp.orientation)

        final_pos_err = math.hypot(fx - gx, fy - gy)
        final_yaw_err = abs(
            math.atan2(math.sin(yaw - fyaw), math.cos(yaw - fyaw))
        )

        efficiency = (straight / self.path_length) if self.path_length > 1e-3 else 0.0

        return TrialResult(
            trial=trial_num,
            goal_x=round(gx, 3),
            goal_y=round(gy, 3),
            goal_yaw=round(yaw, 3),
            result=outcome,
            duration_s=round(duration, 2),
            path_length_m=round(self.path_length, 2),
            straight_dist_m=round(straight, 2),
            path_efficiency=round(efficiency, 3),
            final_pos_err_m=round(final_pos_err, 3),
            final_yaw_err_rad=round(final_yaw_err, 3),
            num_recoveries=int(self.num_recoveries),
            num_collisions=int(self.collision_count),
            failure_code=failure_code,
        )

    @staticmethod
    def _quat_to_yaw(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    # ------------------ output ------------------
    def _prepare_output(self):
        if self.args.output_dir:
            log_dir = Path(self.args.output_dir).expanduser()
        else:
            log_dir = self._resolve_default_log_dir()
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        return log_dir / f'run_{stamp}.csv'

    @staticmethod
    def _resolve_default_log_dir():
        # Prefer source tree (dev workflow): walk up from install share dir
        # and look for src/**/nav2_navigation/package.xml.
        try:
            share = Path(get_package_share_directory('nav2_navigation'))
            for parent in share.parents:
                matches = list(parent.glob('src/**/nav2_navigation/package.xml'))
                if matches:
                    return matches[0].parent / 'scenario_logs'
                if parent == parent.parent:
                    break
        except Exception:
            pass
        return Path.home() / 'nav2_navigation_logs'

    def _append_csv(self, path, result, write_header):
        mode = 'w' if write_header else 'a'
        with open(path, mode, newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(asdict(result).keys()))
            if write_header:
                writer.writeheader()
            writer.writerow(asdict(result))

    def _print_summary(self, results, output_path):
        n = len(results)
        if n == 0:
            return
        success = [r for r in results if r.result == 'SUCCESS']
        n_succ = len(success)
        avg_eff = (
            sum(r.path_efficiency for r in success) / n_succ if n_succ else 0.0
        )
        avg_dur = (
            sum(r.duration_s for r in success) / n_succ if n_succ else 0.0
        )
        total_rec = sum(r.num_recoveries for r in results)
        total_col = sum(r.num_collisions for r in results)
        self.get_logger().info(
            f'===== Summary =====\n'
            f'  trials       : {n}\n'
            f'  success      : {n_succ}/{n} ({100.0 * n_succ / n:.1f}%)\n'
            f'  avg duration : {avg_dur:.2f}s (success only)\n'
            f'  avg efficiency: {avg_eff:.3f} (success only)\n'
            f'  recoveries   : {total_rec}\n'
            f'  collisions   : {total_col}\n'
            f'  csv          : {output_path}'
        )


def parse_args():
    p = argparse.ArgumentParser(
        description='Random goal runner for Nav2 RIL validation'
    )
    p.add_argument('--map_yaml', default=None,
                   help='Path to map yaml file '
                        '(default: nav2_navigation/map_2D/map5.yaml)')
    p.add_argument('--num_goals', type=int, default=20)
    p.add_argument('--timeout', type=float, default=300.0,
                   help='Per-goal timeout in seconds')
    p.add_argument('--robot_radius', type=float, default=0.8,
                   help='Erosion radius in meters (safety margin from walls)')
    p.add_argument('--odom_topic', type=str, default='/odometry/imu')
    p.add_argument('--output_dir', type=str, default=None,
                   help='Override CSV output directory')
    p.add_argument('--seed', type=int, default=None)
    return p.parse_args()


def _default_map_yaml():
    share = get_package_share_directory('nav2_navigation')
    return os.path.join(share, 'map_2D', 'map5.yaml')


def main():
    args = parse_args()
    if args.map_yaml is None:
        args.map_yaml = _default_map_yaml()
    if args.seed is not None:
        random.seed(args.seed)

    rclpy.init()
    node = RandomGoalRunner(args)
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
