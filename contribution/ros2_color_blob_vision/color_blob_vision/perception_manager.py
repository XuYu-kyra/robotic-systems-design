import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray


class PerceptionManager(Node):
    """
    Task-facing target selector.

    - Keeps both white/full 3D streams alive at the same time.
    - Switches target policy at runtime from /task/mode.
    - Transforms the chosen target into a robot frame for downstream pick/place.
    """

    def __init__(self):
        super().__init__("perception_manager")

        self.declare_parameter("default_mode", "search_white_bin")
        self.declare_parameter("mode_topic", "/task/mode")
        self.declare_parameter("target_color_hint_topic", "/task/target_color_hint")
        self.declare_parameter("white_topic", "/white/color_blobs_3d")
        self.declare_parameter("full_topic", "/full/color_blobs_3d")
        self.declare_parameter("target_pose_topic", "/task/current_target_pose")
        self.declare_parameter("target_label_topic", "/task/current_target_label")
        self.declare_parameter("target_visible_topic", "/task/current_target_visible")
        self.declare_parameter("target_reachable_topic", "/task/current_target_reachable")
        self.declare_parameter("target_status_topic", "/task/current_target_status")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("print_interval", 0.5)
        self.declare_parameter("target_hold_timeout", 1.0)
        self.declare_parameter("confirm_hits", 3)
        self.declare_parameter("match_dist_m", 0.08)
        self.declare_parameter("reachable_x_min_m", 0.05)
        self.declare_parameter("reachable_x_max_m", 0.45)
        self.declare_parameter("reachable_y_min_m", -0.30)
        self.declare_parameter("reachable_y_max_m", 0.30)
        self.declare_parameter("reachable_z_min_m", -0.05)
        self.declare_parameter("reachable_z_max_m", 0.35)
        self.declare_parameter("allow_fallback_to_source_frame", True)

        self._mode = str(self.get_parameter("default_mode").value)
        self._target_color_hint = ""
        self._mode_topic = str(self.get_parameter("mode_topic").value)
        self._target_color_hint_topic = str(
            self.get_parameter("target_color_hint_topic").value
        )
        self._white_topic = str(self.get_parameter("white_topic").value)
        self._full_topic = str(self.get_parameter("full_topic").value)
        self._target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        self._target_label_topic = str(self.get_parameter("target_label_topic").value)
        self._target_visible_topic = str(
            self.get_parameter("target_visible_topic").value
        )
        self._target_reachable_topic = str(
            self.get_parameter("target_reachable_topic").value
        )
        self._target_status_topic = str(
            self.get_parameter("target_status_topic").value
        )
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._print_interval = float(self.get_parameter("print_interval").value)
        self._target_hold_timeout = float(
            self.get_parameter("target_hold_timeout").value
        )
        self._confirm_hits = max(1, int(self.get_parameter("confirm_hits").value))
        self._match_dist_m = float(self.get_parameter("match_dist_m").value)
        self._reachable_x_min = float(self.get_parameter("reachable_x_min_m").value)
        self._reachable_x_max = float(self.get_parameter("reachable_x_max_m").value)
        self._reachable_y_min = float(self.get_parameter("reachable_y_min_m").value)
        self._reachable_y_max = float(self.get_parameter("reachable_y_max_m").value)
        self._reachable_z_min = float(self.get_parameter("reachable_z_min_m").value)
        self._reachable_z_max = float(self.get_parameter("reachable_z_max_m").value)
        self._allow_fallback = bool(
            self.get_parameter("allow_fallback_to_source_frame").value
        )

        self._latest_white = []
        self._latest_full = []
        self._last_white_time = 0.0
        self._last_full_time = 0.0
        self._last_print_time = 0.0
        self._stable_target = None
        self._candidate_key = None
        self._candidate_hits = 0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._target_pose_pub = self.create_publisher(
            PoseStamped,
            self._target_pose_topic,
            10,
        )
        self._target_label_pub = self.create_publisher(
            String,
            self._target_label_topic,
            10,
        )
        self._target_visible_pub = self.create_publisher(
            Bool,
            self._target_visible_topic,
            10,
        )
        self._target_reachable_pub = self.create_publisher(
            Bool,
            self._target_reachable_topic,
            10,
        )
        self._target_status_pub = self.create_publisher(
            String,
            self._target_status_topic,
            10,
        )

        self.create_subscription(
            Detection3DArray,
            self._white_topic,
            self._white_cb,
            10,
        )
        self.create_subscription(
            Detection3DArray,
            self._full_topic,
            self._full_cb,
            10,
        )
        self.create_subscription(
            String,
            self._mode_topic,
            self._mode_cb,
            10,
        )
        self.create_subscription(
            String,
            self._target_color_hint_topic,
            self._target_color_hint_cb,
            10,
        )

        self._timer = self.create_timer(0.2, self._tick)

        self.get_logger().info(
            "perception_manager started.\n"
            f"  default_mode: {self._mode}\n"
            f"  white_topic:  {self._white_topic}\n"
            f"  full_topic:   {self._full_topic}\n"
            f"  target_frame: {self._target_frame}\n"
            f"  mode_topic:   {self._mode_topic}"
        )

    def _split_class_id(self, class_id: str):
        if ":" in class_id:
            obj_type, color = class_id.split(":", 1)
            return obj_type, color
        return "unknown", class_id

    def _quat_to_yaw_deg(self, qx, qy, qz, qw):
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def _quat_multiply(self, a, b):
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )

    def _quat_conjugate(self, q):
        qx, qy, qz, qw = q
        return (-qx, -qy, -qz, qw)

    def _rotate_vector(self, q, v):
        vx, vy, vz = v
        rotated = self._quat_multiply(
            self._quat_multiply(q, (vx, vy, vz, 0.0)),
            self._quat_conjugate(q),
        )
        return rotated[0], rotated[1], rotated[2]

    def _msg_to_list(self, msg: Detection3DArray):
        objs = []
        frame_id = msg.header.frame_id
        stamp = msg.header.stamp
        for det in msg.detections:
            if not det.results:
                continue

            hyp = det.results[0]
            class_id = hyp.hypothesis.class_id
            obj_type, color = self._split_class_id(class_id)
            q = hyp.pose.pose.orientation
            yaw_deg = self._quat_to_yaw_deg(q.x, q.y, q.z, q.w)

            objs.append(
                {
                    "class_id": class_id,
                    "type": obj_type,
                    "color": color,
                    "score": float(hyp.hypothesis.score),
                    "x": float(hyp.pose.pose.position.x),
                    "y": float(hyp.pose.pose.position.y),
                    "z": float(hyp.pose.pose.position.z),
                    "rz_deg": float(yaw_deg),
                    "quat": (float(q.x), float(q.y), float(q.z), float(q.w)),
                    "frame_id": frame_id,
                    "stamp": stamp,
                }
            )
        return objs

    def _mode_cb(self, msg: String):
        mode = msg.data.strip()
        if not mode or mode == self._mode:
            return
        self._mode = mode
        self._stable_target = None
        self._candidate_key = None
        self._candidate_hits = 0
        self.get_logger().info(f"Mode switched to: {self._mode}")

    def _target_color_hint_cb(self, msg: String):
        self._target_color_hint = msg.data.strip().lower()

    def _white_cb(self, msg: Detection3DArray):
        objs = self._msg_to_list(msg)
        self._latest_white = objs
        if objs:
            self._last_white_time = time.time()

    def _full_cb(self, msg: Detection3DArray):
        objs = self._msg_to_list(msg)
        self._latest_full = objs
        if objs:
            self._last_full_time = time.time()

    def _select_target(self) -> Optional[dict]:
        now = time.time()
        white_fresh = (now - self._last_white_time) <= self._target_hold_timeout
        full_fresh = (now - self._last_full_time) <= self._target_hold_timeout

        if self._mode == "search_white_bin":
            candidates = [
                o for o in self._latest_white
                if white_fresh and o["class_id"] == "bin:white"
            ]
        elif self._mode == "search_block":
            candidates = [
                o for o in self._latest_full
                if full_fresh
                and o["type"] == "block"
                and o["color"] in ("red", "blue", "yellow")
            ]
        elif self._mode == "search_color_bin":
            candidates = [
                o for o in self._latest_full
                if full_fresh
                and o["type"] == "bin"
                and o["color"] in ("red", "blue", "yellow")
            ]
            if self._target_color_hint:
                preferred = [
                    o for o in candidates if o["color"] == self._target_color_hint
                ]
                if preferred:
                    candidates = preferred
        else:
            candidates = []

        if not candidates:
            return None

        candidates.sort(key=lambda o: (o["z"], -o["score"]))
        return candidates[0]

    def _update_stable_target(self, candidate: Optional[dict]) -> Optional[dict]:
        if candidate is None:
            self._candidate_key = None
            self._candidate_hits = 0
            self._stable_target = None
            return None

        key = candidate["class_id"]
        if self._stable_target is not None:
            same_class = self._stable_target["class_id"] == candidate["class_id"]
            dist = math.sqrt(
                (self._stable_target["x"] - candidate["x"]) ** 2
                + (self._stable_target["y"] - candidate["y"]) ** 2
                + (self._stable_target["z"] - candidate["z"]) ** 2
            )
            if same_class and dist <= self._match_dist_m:
                self._candidate_hits = min(self._candidate_hits + 1, self._confirm_hits)
                self._stable_target = candidate
                return self._stable_target

        if key == self._candidate_key:
            self._candidate_hits += 1
        else:
            self._candidate_key = key
            self._candidate_hits = 1

        if self._candidate_hits >= self._confirm_hits:
            self._stable_target = candidate
            return self._stable_target

        self._stable_target = None
        return None

    def _transform_target(self, target: dict) -> tuple[Optional[PoseStamped], bool]:
        pose = PoseStamped()
        pose.header.frame_id = target["frame_id"]
        pose.header.stamp = target["stamp"]
        pose.pose.position.x = target["x"]
        pose.pose.position.y = target["y"]
        pose.pose.position.z = target["z"]
        qx, qy, qz, qw = target["quat"]
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        if pose.header.frame_id == self._target_frame:
            return pose, True

        try:
            tf_msg = self._tf_buffer.lookup_transform(
                self._target_frame,
                pose.header.frame_id,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"TF unavailable for {pose.header.frame_id} -> {self._target_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            if self._allow_fallback:
                return pose, False
            return None, False

        tx = float(tf_msg.transform.translation.x)
        ty = float(tf_msg.transform.translation.y)
        tz = float(tf_msg.transform.translation.z)
        r = tf_msg.transform.rotation
        rq = (float(r.x), float(r.y), float(r.z), float(r.w))

        px, py, pz = self._rotate_vector(rq, (target["x"], target["y"], target["z"]))
        out = PoseStamped()
        out.header.stamp = pose.header.stamp
        out.header.frame_id = self._target_frame
        out.pose.position.x = px + tx
        out.pose.position.y = py + ty
        out.pose.position.z = pz + tz

        oq = self._quat_multiply(rq, target["quat"])
        out.pose.orientation.x = oq[0]
        out.pose.orientation.y = oq[1]
        out.pose.orientation.z = oq[2]
        out.pose.orientation.w = oq[3]
        return out, True

    def _is_reachable(self, pose: PoseStamped) -> bool:
        px = float(pose.pose.position.x)
        py = float(pose.pose.position.y)
        pz = float(pose.pose.position.z)
        return (
            self._reachable_x_min <= px <= self._reachable_x_max
            and self._reachable_y_min <= py <= self._reachable_y_max
            and self._reachable_z_min <= pz <= self._reachable_z_max
        )

    def _publish_status(self, label: str, visible: bool, reachable: bool, status: str):
        self._target_label_pub.publish(String(data=label))
        self._target_visible_pub.publish(Bool(data=visible))
        self._target_reachable_pub.publish(Bool(data=reachable))
        self._target_status_pub.publish(String(data=status))

    def _tick(self):
        now = time.time()
        candidate = self._select_target()

        if candidate is None:
            self._publish_status("", False, False, "no_target")
            if now - self._last_print_time >= self._print_interval:
                self._last_print_time = now
                self.get_logger().info(f"[mode={self._mode}] No target")
            self._candidate_key = None
            self._candidate_hits = 0
            self._stable_target = None
            return

        stable_target = self._update_stable_target(candidate)
        if stable_target is None:
            self._publish_status(candidate["class_id"], True, False, "candidate_unstable")
            return

        target_pose, transformed = self._transform_target(stable_target)
        if target_pose is None:
            self._publish_status(stable_target["class_id"], True, False, "tf_failed")
            return

        reachable = transformed and self._is_reachable(target_pose)
        self._target_pose_pub.publish(target_pose)
        status = "target_ready" if reachable else "target_visible_not_reachable"
        if not transformed and self._allow_fallback:
            status = "target_visible_tf_fallback"
        self._publish_status(stable_target["class_id"], True, reachable, status)

        if now - self._last_print_time < self._print_interval:
            return
        self._last_print_time = now

        yaw_deg = self._quat_to_yaw_deg(
            target_pose.pose.orientation.x,
            target_pose.pose.orientation.y,
            target_pose.pose.orientation.z,
            target_pose.pose.orientation.w,
        )
        self.get_logger().info(
            f"[mode={self._mode}] "
            f"class={stable_target['class_id']} | "
            f"frame={target_pose.header.frame_id} | "
            f"pos=({target_pose.pose.position.x:.3f}, "
            f"{target_pose.pose.position.y:.3f}, "
            f"{target_pose.pose.position.z:.3f}) m | "
            f"rz={yaw_deg:.1f} deg | "
            f"reachable={reachable} | "
            f"score={stable_target['score']:.4f}"
        )


def main(args=None):
    try:
        rclpy.init(args=args)
        node = PerceptionManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
