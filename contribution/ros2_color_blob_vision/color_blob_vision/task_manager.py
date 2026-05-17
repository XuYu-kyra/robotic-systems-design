import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String


class TaskManager(Node):
    """
    Minimal orchestrator that turns perception topics into a runnable task loop.

    Navigation/manipulation are exposed as simple command topics for now so the
    full pipeline can run before the real executors are wired in.
    """

    def __init__(self):
        super().__init__("task_manager")

        self.declare_parameter("mode_topic", "/task/mode")
        self.declare_parameter("state_topic", "/task/state")
        self.declare_parameter("event_topic", "/task/event")
        self.declare_parameter("decision_topic", "/task/decision")
        self.declare_parameter("nav_goal_topic", "/task/nav_goal_name")
        self.declare_parameter("arm_command_topic", "/task/arm_command")
        self.declare_parameter("arm_target_topic", "/task/arm_target_pose")
        self.declare_parameter("target_label_topic", "/task/current_target_label")
        self.declare_parameter("target_visible_topic", "/task/current_target_visible")
        self.declare_parameter("target_reachable_topic", "/task/current_target_reachable")
        self.declare_parameter("target_pose_topic", "/task/current_target_pose")
        self.declare_parameter("target_color_hint_topic", "/task/target_color_hint")
        self.declare_parameter("initial_state", "SEARCH_WHITE_BIN")
        self.declare_parameter("simulate_nav", True)
        self.declare_parameter("simulate_arm", True)
        self.declare_parameter("sim_nav_duration_sec", 2.0)
        self.declare_parameter("sim_home_nav_duration_sec", 12.0)
        self.declare_parameter("sim_arm_duration_sec", 2.0)
        self.declare_parameter("settle_duration_sec", 1.0)
        self.declare_parameter("white_bin_confirm_hits", 3)
        self.declare_parameter("block_confirm_hits", 3)
        self.declare_parameter("home_bin_confirm_hits", 2)
        self.declare_parameter("decision_log_interval_sec", 1.0)
        self.declare_parameter("require_reachable_targets", True)
        self.declare_parameter("use_spatial_white_bin_dedup", True)
        self.declare_parameter("white_bin_search_timeout_sec", 4.0)
        self.declare_parameter("explore_nav_goal", "explore_for_white_bin")
        self.declare_parameter("approach_white_bin_nav_goal", "approach_white_bin")
        self.declare_parameter("home_bin_nav_goal", "home_bin_area_pose")
        self.declare_parameter("white_bin_visit_radius_m", 0.35)
        self.declare_parameter("block_search_radius_from_white_bin_m", 0.45)
        self.declare_parameter("offline_white_bin_relock_cooldown_sec", 3.0)
        self.declare_parameter("offline_white_bin_lost_timeout_sec", 1.5)
        self.declare_parameter("slot_order", ["left", "rear", "right"])
        self.declare_parameter("red_slot", "left")
        self.declare_parameter("blue_slot", "rear")
        self.declare_parameter("yellow_slot", "right")

        self._mode_topic = str(self.get_parameter("mode_topic").value)
        self._state_topic = str(self.get_parameter("state_topic").value)
        self._event_topic = str(self.get_parameter("event_topic").value)
        self._decision_topic = str(self.get_parameter("decision_topic").value)
        self._nav_goal_topic = str(self.get_parameter("nav_goal_topic").value)
        self._arm_command_topic = str(self.get_parameter("arm_command_topic").value)
        self._arm_target_topic = str(self.get_parameter("arm_target_topic").value)
        self._target_color_hint_topic = str(
            self.get_parameter("target_color_hint_topic").value
        )
        self._state = str(self.get_parameter("initial_state").value)
        self._simulate_nav = bool(self.get_parameter("simulate_nav").value)
        self._simulate_arm = bool(self.get_parameter("simulate_arm").value)
        self._sim_nav_duration = float(
            self.get_parameter("sim_nav_duration_sec").value
        )
        self._sim_home_nav_duration = float(
            self.get_parameter("sim_home_nav_duration_sec").value
        )
        self._sim_arm_duration = float(
            self.get_parameter("sim_arm_duration_sec").value
        )
        self._settle_duration = float(
            self.get_parameter("settle_duration_sec").value
        )
        self._white_bin_confirm_hits = max(
            1, int(self.get_parameter("white_bin_confirm_hits").value)
        )
        self._block_confirm_hits = max(
            1, int(self.get_parameter("block_confirm_hits").value)
        )
        self._home_bin_confirm_hits = max(
            1, int(self.get_parameter("home_bin_confirm_hits").value)
        )
        self._decision_log_interval = float(
            self.get_parameter("decision_log_interval_sec").value
        )
        self._require_reachable_targets = bool(
            self.get_parameter("require_reachable_targets").value
        )
        self._use_spatial_white_bin_dedup = bool(
            self.get_parameter("use_spatial_white_bin_dedup").value
        )
        self._white_bin_search_timeout = float(
            self.get_parameter("white_bin_search_timeout_sec").value
        )
        self._explore_nav_goal = str(self.get_parameter("explore_nav_goal").value)
        self._approach_white_bin_nav_goal = str(
            self.get_parameter("approach_white_bin_nav_goal").value
        )
        self._home_bin_nav_goal = str(self.get_parameter("home_bin_nav_goal").value)
        self._white_bin_visit_radius = float(
            self.get_parameter("white_bin_visit_radius_m").value
        )
        self._block_search_radius = float(
            self.get_parameter("block_search_radius_from_white_bin_m").value
        )
        self._offline_white_bin_relock_cooldown = float(
            self.get_parameter("offline_white_bin_relock_cooldown_sec").value
        )
        self._offline_white_bin_lost_timeout = float(
            self.get_parameter("offline_white_bin_lost_timeout_sec").value
        )
        self._slot_order = list(self.get_parameter("slot_order").value)
        self._color_to_slot = {
            "red": str(self.get_parameter("red_slot").value),
            "blue": str(self.get_parameter("blue_slot").value),
            "yellow": str(self.get_parameter("yellow_slot").value),
        }

        self._target_label = ""
        self._target_visible = False
        self._target_reachable = False
        self._target_pose = None
        self._held_block_color = ""
        self._state_enter_time = time.time()
        self._last_state_pub = ""
        self._last_command_sent = ""
        self._current_dump_index = 0
        self._collected_blocks = {"red": False, "blue": False, "yellow": False}
        self._slot_contents = {slot: "" for slot in self._slot_order}
        self._active_nav_goal = ""
        self._current_white_bin_pose = None
        self._visited_white_bins = []
        self._white_bin_relock_allowed_time = 0.0
        self._last_seen_white_bin_time = 0.0
        self._white_bin_candidate_hits = 0
        self._block_candidate_hits = 0
        self._home_bin_candidate_hits = 0
        self._color_bin_candidate_hits = {}
        self._last_decision_text = ""
        self._last_decision_time = 0.0

        self._mode_pub = self.create_publisher(String, self._mode_topic, 10)
        self._state_pub = self.create_publisher(String, self._state_topic, 10)
        self._event_pub = self.create_publisher(String, self._event_topic, 10)
        self._decision_pub = self.create_publisher(String, self._decision_topic, 10)
        self._nav_goal_pub = self.create_publisher(String, self._nav_goal_topic, 10)
        self._arm_command_pub = self.create_publisher(String, self._arm_command_topic, 10)
        self._arm_target_pub = self.create_publisher(PoseStamped, self._arm_target_topic, 10)
        self._target_color_hint_pub = self.create_publisher(
            String,
            self._target_color_hint_topic,
            10,
        )

        self.create_subscription(
            String,
            str(self.get_parameter("target_label_topic").value),
            self._target_label_cb,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("target_visible_topic").value),
            self._target_visible_cb,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("target_reachable_topic").value),
            self._target_reachable_cb,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("target_pose_topic").value),
            self._target_pose_cb,
            10,
        )

        self._timer = self.create_timer(0.2, self._tick)
        self._publish_mode(self._mode_for_state(self._state))
        self._publish_target_color_hint("")

        self.get_logger().info(
            "task_manager started.\n"
            f"  initial_state: {self._state}\n"
            f"  simulate_nav:  {self._simulate_nav}\n"
            f"  simulate_arm:  {self._simulate_arm}\n"
            f"  sim_home_nav_duration:  {self._sim_home_nav_duration}\n"
            f"  white_bin_confirm_hits: {self._white_bin_confirm_hits}\n"
            f"  block_confirm_hits:     {self._block_confirm_hits}\n"
            f"  home_bin_confirm_hits:  {self._home_bin_confirm_hits}\n"
            f"  require_reachable:      {self._require_reachable_targets}\n"
            f"  spatial_white_bin_dedup: {self._use_spatial_white_bin_dedup}\n"
            f"  explore_goal:     {self._explore_nav_goal}\n"
            f"  approach_goal:    {self._approach_white_bin_nav_goal}\n"
            f"  home_bin_goal:    {self._home_bin_nav_goal}"
        )

    def _target_label_cb(self, msg: String):
        self._target_label = msg.data.strip()

    def _target_visible_cb(self, msg: Bool):
        self._target_visible = bool(msg.data)

    def _target_reachable_cb(self, msg: Bool):
        self._target_reachable = bool(msg.data)

    def _target_pose_cb(self, msg: PoseStamped):
        self._target_pose = msg

    def _publish_mode(self, mode: str):
        self._mode_pub.publish(String(data=mode))

    def _publish_target_color_hint(self, color: str):
        self._target_color_hint_pub.publish(String(data=color))

    def _publish_event(self, text: str):
        self._event_pub.publish(String(data=text))

    def _publish_decision(self, text: str, force: bool = False):
        now = time.time()
        if (
            not force
            and text == self._last_decision_text
            and (now - self._last_decision_time) < self._decision_log_interval
        ):
            return
        self._decision_pub.publish(String(data=text))
        self._last_decision_text = text
        self._last_decision_time = now

    def _transition(self, new_state: str, event: str):
        if new_state == self._state:
            return
        self._state = new_state
        self._state_enter_time = time.time()
        self._last_command_sent = ""
        self._white_bin_candidate_hits = 0
        self._block_candidate_hits = 0
        self._home_bin_candidate_hits = 0
        self._color_bin_candidate_hits = {}
        self._publish_mode(self._mode_for_state(new_state))
        if new_state != "SEARCH_COLOR_BIN":
            self._publish_target_color_hint("")
        self._publish_event(event)
        self.get_logger().info(f"State -> {new_state} ({event})")

    def _mode_for_state(self, state: str) -> str:
        if state in ("SEARCH_WHITE_BIN", "NAV_EXPLORE", "APPROACH_WHITE_BIN"):
            return "search_white_bin"
        if state in ("SEARCH_BLOCK", "PICK_BLOCK", "PLACE_TO_ROBOT_SLOT"):
            return "search_block"
        if state in ("SEARCH_COLOR_BIN", "DUMP_SLOT_TO_BIN"):
            return "search_color_bin"
        return "search_white_bin"

    def _time_in_state(self) -> float:
        return time.time() - self._state_enter_time

    def _send_nav_goal_once(self, goal_name: str):
        command = f"nav:{goal_name}"
        if self._last_command_sent == command:
            return
        self._nav_goal_pub.publish(String(data=goal_name))
        self._publish_event(command)
        self._last_command_sent = command

    def _send_arm_command_once(self, command_name: str):
        if self._last_command_sent == command_name:
            return
        self._arm_command_pub.publish(String(data=command_name))
        if self._target_pose is not None:
            self._arm_target_pub.publish(self._target_pose)
        self._publish_event(command_name)
        self._last_command_sent = command_name

    def _current_target_color(self) -> str:
        if ":" not in self._target_label:
            return ""
        return self._target_label.split(":", 1)[1]

    def _publish_state(self):
        if self._last_state_pub == self._state:
            return
        self._state_pub.publish(String(data=self._state))
        self._last_state_pub = self._state

    def _all_blocks_collected(self) -> bool:
        return all(self._collected_blocks.values())

    def _desired_slot_for_color(self, color: str) -> str:
        return self._color_to_slot.get(color, "")

    def _register_collected_block(self, color: str):
        slot = self._desired_slot_for_color(color)
        if color in self._collected_blocks:
            self._collected_blocks[color] = True
        if slot:
            self._slot_contents[slot] = color

    def _next_dump_slot(self) -> str:
        while self._current_dump_index < len(self._slot_order):
            slot = self._slot_order[self._current_dump_index]
            if self._slot_contents.get(slot):
                return slot
            self._current_dump_index += 1
        return ""

    def _expected_bin_label_for_active_slot(self) -> str:
        slot = self._next_dump_slot()
        color = self._slot_contents.get(slot, "")
        return f"bin:{color}" if color else ""

    def _remaining_slot_colors(self) -> dict[str, str]:
        remaining = {}
        for slot in self._slot_order:
            color = self._slot_contents.get(slot, "")
            if color:
                remaining[color] = slot
        return remaining

    def _white_bin_reject_reason(self) -> str:
        if not self._target_visible:
            return "reject_white_bin:not_visible"
        if self._target_label != "bin:white":
            return f"reject_white_bin:label={self._target_label or 'none'}"
        if self._target_pose is None:
            return "reject_white_bin:no_pose"
        now = time.time()
        if not self._use_spatial_white_bin_dedup and now < self._white_bin_relock_allowed_time:
            return (
                "reject_white_bin:cooldown:"
                f"{self._white_bin_relock_allowed_time - now:.1f}s"
            )
        if self._use_spatial_white_bin_dedup:
            for pose in self._visited_white_bins:
                if (
                    self._distance_between_poses(self._target_pose, pose)
                    <= self._white_bin_visit_radius
                ):
                    return "reject_white_bin:visited_nearby"
        return "accept_white_bin_candidate"

    def _block_reject_reason(self) -> str:
        if not self._target_visible:
            return "reject_block:not_visible"
        if not self._target_label.startswith("block:"):
            return f"reject_block:label={self._target_label or 'none'}"
        if self._target_pose is None:
            return "reject_block:no_pose"
        if self._require_reachable_targets and not self._target_reachable:
            return "reject_block:not_reachable"
        color = self._current_target_color()
        if not color:
            return "reject_block:no_color"
        if self._collected_blocks.get(color, False):
            return f"reject_block:already_collected:{color}"
        if self._current_white_bin_pose is None:
            return "reject_block:no_locked_white_bin"
        if not self._block_is_near_locked_white_bin():
            return "reject_block:far_from_locked_white_bin"
        return "accept_block_candidate"

    def _home_bin_detected_label(self) -> str:
        if not self._target_visible:
            return ""
        if not self._target_label.startswith("bin:"):
            return ""
        color = self._current_target_color()
        if color not in ("red", "blue", "yellow"):
            return ""
        if self._require_reachable_targets and not self._target_reachable:
            return ""
        return self._target_label

    def _home_bin_reject_reason(self) -> str:
        if not self._target_visible:
            return "reject_home_bin:not_visible"
        if not self._target_label.startswith("bin:"):
            return f"reject_home_bin:label={self._target_label or 'none'}"
        color = self._current_target_color()
        if color not in ("red", "blue", "yellow"):
            return f"reject_home_bin:color={color or 'none'}"
        if self._require_reachable_targets and not self._target_reachable:
            return "reject_home_bin:not_reachable"
        return f"accept_home_bin_candidate:{self._target_label}"

    def _matching_remaining_color_bin(self) -> tuple[str, str]:
        remaining = self._remaining_slot_colors()
        if not self._target_visible:
            return "", ""
        if not self._target_label.startswith("bin:"):
            return "", ""
        color = self._current_target_color()
        if color not in remaining:
            return "", ""
        if self._target_pose is None:
            return "", ""
        if self._require_reachable_targets and not self._target_reachable:
            return "", ""
        return color, remaining[color]

    def _color_bin_reject_reason(self) -> str:
        remaining = self._remaining_slot_colors()
        if not remaining:
            return "reject_color_bin:no_remaining_slots"
        if not self._target_visible:
            return (
                "reject_color_bin:not_visible:remaining="
                + ",".join(sorted(remaining.keys()))
            )
        if not self._target_label.startswith("bin:"):
            return f"reject_color_bin:label={self._target_label or 'none'}"
        color = self._current_target_color()
        if color not in remaining:
            return (
                f"reject_color_bin:unexpected_color={color or 'none'}:"
                f"remaining={','.join(sorted(remaining.keys()))}"
            )
        if self._target_pose is None:
            return "reject_color_bin:no_pose"
        if self._require_reachable_targets and not self._target_reachable:
            return "reject_color_bin:not_reachable"
        return f"accept_color_bin_candidate:{self._target_label}"

    def _distance_between_poses(self, a: PoseStamped, b: PoseStamped) -> float:
        dx = float(a.pose.position.x) - float(b.pose.position.x)
        dy = float(a.pose.position.y) - float(b.pose.position.y)
        dz = float(a.pose.position.z) - float(b.pose.position.z)
        return (dx * dx + dy * dy + dz * dz) ** 0.5

    def _current_target_is_new_white_bin(self) -> bool:
        if not (
            self._target_visible
            and self._target_label == "bin:white"
            and self._target_pose is not None
        ):
            return False
        now = time.time()
        self._last_seen_white_bin_time = now
        if not self._use_spatial_white_bin_dedup:
            if now < self._white_bin_relock_allowed_time:
                return False
            return True
        for pose in self._visited_white_bins:
            if self._distance_between_poses(self._target_pose, pose) <= self._white_bin_visit_radius:
                return False
        return True

    def _block_is_near_locked_white_bin(self) -> bool:
        if self._current_white_bin_pose is None or self._target_pose is None:
            return False
        return (
            self._distance_between_poses(self._target_pose, self._current_white_bin_pose)
            <= self._block_search_radius
        )

    def _tick(self):
        self._publish_state()
        now = time.time()

        if self._target_visible and self._target_label == "bin:white":
            self._last_seen_white_bin_time = now
        elif not self._use_spatial_white_bin_dedup:
            if (
                self._white_bin_relock_allowed_time > 0.0
                and now >= self._white_bin_relock_allowed_time
            ):
                pass
            elif (now - self._last_seen_white_bin_time) >= self._offline_white_bin_lost_timeout:
                self._white_bin_relock_allowed_time = now

        if self._state == "SEARCH_WHITE_BIN":
            self._publish_mode("search_white_bin")
            reason = self._white_bin_reject_reason()
            if self._current_target_is_new_white_bin():
                self._white_bin_candidate_hits += 1
                self._publish_decision(
                    f"white_bin_candidate:{self._white_bin_candidate_hits}/"
                    f"{self._white_bin_confirm_hits}"
                )
                if self._white_bin_candidate_hits >= self._white_bin_confirm_hits:
                    self._current_white_bin_pose = self._target_pose
                    self._active_nav_goal = self._approach_white_bin_nav_goal
                    self._transition("APPROACH_WHITE_BIN", "white_bin_locked")
                return
            self._white_bin_candidate_hits = 0
            self._publish_decision(reason)
            if self._time_in_state() >= self._white_bin_search_timeout:
                self._active_nav_goal = self._explore_nav_goal
                self._transition("NAV_EXPLORE", "white_bin_search_timeout")
            return

        if self._state == "NAV_EXPLORE":
            self._publish_mode("search_white_bin")
            reason = self._white_bin_reject_reason()
            if self._current_target_is_new_white_bin():
                self._white_bin_candidate_hits += 1
                self._publish_decision(
                    f"white_bin_candidate:{self._white_bin_candidate_hits}/"
                    f"{self._white_bin_confirm_hits}"
                )
                if self._white_bin_candidate_hits >= self._white_bin_confirm_hits:
                    self._current_white_bin_pose = self._target_pose
                    self._active_nav_goal = self._approach_white_bin_nav_goal
                    self._transition(
                        "APPROACH_WHITE_BIN",
                        "white_bin_locked_during_explore",
                    )
                return
            self._white_bin_candidate_hits = 0
            self._publish_decision(reason)
            self._send_nav_goal_once(self._active_nav_goal)
            if self._simulate_nav and self._time_in_state() >= self._sim_nav_duration:
                self._transition("SEARCH_WHITE_BIN", f"nav_done:{self._active_nav_goal}")
            return

        if self._state == "APPROACH_WHITE_BIN":
            self._publish_mode("search_white_bin")
            self._send_nav_goal_once(self._approach_white_bin_nav_goal)
            if self._simulate_nav and self._time_in_state() >= self._sim_nav_duration:
                self._transition("SEARCH_BLOCK", "nav_done:approach_white_bin")
            return

        if self._state == "SEARCH_BLOCK":
            self._publish_mode("search_block")
            reason = self._block_reject_reason()
            if reason == "accept_block_candidate":
                self._block_candidate_hits += 1
                self._publish_decision(
                    f"block_candidate:{self._block_candidate_hits}/"
                    f"{self._block_confirm_hits}:{self._target_label}"
                )
                if self._block_candidate_hits >= self._block_confirm_hits:
                    self._held_block_color = self._current_target_color()
                    self._transition("PICK_BLOCK", f"block_locked:{self._target_label}")
                return
            self._block_candidate_hits = 0
            self._publish_decision(reason)
            return

        if self._state == "PICK_BLOCK":
            self._publish_mode("search_block")
            self._send_arm_command_once("pick")
            if self._simulate_arm and self._time_in_state() >= self._sim_arm_duration:
                self._transition(
                    "PLACE_TO_ROBOT_SLOT",
                    f"pick_done:{self._held_block_color}",
                )
            return

        if self._state == "PLACE_TO_ROBOT_SLOT":
            self._publish_mode("search_block")
            slot = self._desired_slot_for_color(self._held_block_color)
            self._send_arm_command_once(f"place_to_robot_slot_{slot}")
            if self._simulate_arm and self._time_in_state() >= self._sim_arm_duration:
                self._register_collected_block(self._held_block_color)
                self._publish_event(
                    f"slot_filled:{slot}:{self._held_block_color}"
                )
                if self._use_spatial_white_bin_dedup and self._current_white_bin_pose is not None:
                    self._visited_white_bins.append(self._current_white_bin_pose)
                if not self._use_spatial_white_bin_dedup:
                    self._white_bin_relock_allowed_time = (
                        time.time() + self._offline_white_bin_relock_cooldown
                    )
                self._current_white_bin_pose = None
                self._held_block_color = ""
                if self._all_blocks_collected():
                    self._active_nav_goal = self._home_bin_nav_goal
                    self._transition("GO_TO_HOME_BIN_AREA", "all_blocks_collected")
                else:
                    self._active_nav_goal = self._explore_nav_goal
                    self._transition("NAV_EXPLORE", "continue_exploration")
            return

        if self._state == "GO_TO_HOME_BIN_AREA":
            self._send_nav_goal_once(self._home_bin_nav_goal)
            home_bin_label = self._home_bin_detected_label()
            if not self._use_spatial_white_bin_dedup:
                if home_bin_label:
                    self._home_bin_candidate_hits += 1
                    self._publish_decision(
                        f"home_bin_candidate:{self._home_bin_candidate_hits}/"
                        f"{self._home_bin_confirm_hits}:{home_bin_label}"
                    )
                    if self._home_bin_candidate_hits >= self._home_bin_confirm_hits:
                        self._current_dump_index = 0
                        self._transition(
                            "SEARCH_COLOR_BIN",
                            f"nav_done:{self._home_bin_nav_goal}",
                        )
                    return
                self._home_bin_candidate_hits = 0
                self._publish_decision(self._home_bin_reject_reason())
                if self._simulate_nav and self._time_in_state() >= self._sim_home_nav_duration:
                    self._current_dump_index = 0
                    self._transition(
                        "SEARCH_COLOR_BIN",
                        f"nav_timeout:{self._home_bin_nav_goal}",
                    )
                return
            if self._simulate_nav and self._time_in_state() >= self._sim_nav_duration:
                self._current_dump_index = 0
                self._transition(
                    "SEARCH_COLOR_BIN",
                    f"nav_done:{self._home_bin_nav_goal}",
                )
            return

        if self._state == "SEARCH_COLOR_BIN":
            self._publish_mode("search_color_bin")
            remaining = self._remaining_slot_colors()
            if not remaining:
                self._transition("DONE", "all_slots_dumped")
                return
            if self._time_in_state() < self._settle_duration:
                self._publish_decision(
                    f"wait_color_bin:settling:{self._time_in_state():.1f}/"
                    f"{self._settle_duration:.1f}s"
                )
                return
            color, slot = self._matching_remaining_color_bin()
            if color and slot:
                hits = self._color_bin_candidate_hits.get(color, 0) + 1
                self._color_bin_candidate_hits[color] = hits
                self._publish_target_color_hint(color)
                self._publish_decision(
                    f"color_bin_candidate:{hits}/"
                    f"{self._home_bin_confirm_hits}:{self._target_label}"
                )
                if hits >= self._home_bin_confirm_hits:
                    self._current_dump_index = self._slot_order.index(slot)
                    self._transition("DUMP_SLOT_TO_BIN", f"bin_locked:{self._target_label}")
                return
            self._color_bin_candidate_hits = {}
            hint_color = next(iter(remaining.keys()))
            self._publish_target_color_hint(hint_color)
            self._publish_decision(self._color_bin_reject_reason())
            return

        if self._state == "DUMP_SLOT_TO_BIN":
            self._publish_mode("search_color_bin")
            slot = self._next_dump_slot()
            color = self._slot_contents.get(slot, "")
            if not slot or not color:
                self._transition("SEARCH_COLOR_BIN", "dump_target_missing")
                return
            self._publish_target_color_hint(color)
            self._send_arm_command_once(f"dump_{slot}_slot_to_{color}_bin")
            if self._simulate_arm and self._time_in_state() >= self._sim_arm_duration:
                self._publish_event(f"slot_emptied:{slot}:{color}")
                self._slot_contents[slot] = ""
                self._current_dump_index += 1
                self._transition("SEARCH_COLOR_BIN", f"dump_done:{slot}:{color}")
            return

        if self._state == "DONE":
            return

        if self._state == "RECOVER":
            self._held_block_color = ""
            self._current_dump_index = 0
            self._collected_blocks = {"red": False, "blue": False, "yellow": False}
            self._slot_contents = {slot: "" for slot in self._slot_order}
            self._current_white_bin_pose = None
            self._visited_white_bins = []
            self._white_bin_relock_allowed_time = 0.0
            self._last_seen_white_bin_time = 0.0
            self._transition("SEARCH_WHITE_BIN", "recover_restart")


def main(args=None):
    try:
        rclpy.init(args=args)
        node = TaskManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(exc)


if __name__ == "__main__":
    main()
