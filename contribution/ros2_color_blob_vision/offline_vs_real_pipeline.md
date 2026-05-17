# Offline Vs Real Pipeline

This note records the current `ros_nd2` pipeline usage in two modes:

- offline ROS2 bag replay
- real robot execution

It also explains why the two modes differ, and what needs to change when moving from bag replay to the real robot.

## 1. Current offline workflow

The current offline validation bag is:

- `/home/student24/robotproject/bags/realsense_raw_007`

This bag was recorded from a RealSense camera feed and is used to replay a task-like sequence for:

- searching white bins
- finding coloured blocks
- placing collected blocks into fixed robot-mounted slots
- returning to the home area
- finding coloured bins
- dumping collected slots into matching bins

## 2. Recommended offline commands

Build and source:

```bash
cd /home/student24/robotproject/ros_nd2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select color_blob_vision
source /home/student24/robotproject/ros_nd2/install/setup.bash
```

Launch the pipeline in offline mode:

```bash
ros2 launch color_blob_vision vision_pipeline_manager.launch.py \
  pipeline_mode:=offline_bag \
  simulate_nav:=true \
  simulate_arm:=true \
  use_summary:=true
```

Replay the bag:

```bash
ros2 bag play /home/student24/robotproject/bags/realsense_raw_007 --clock
```

For more stable debugging, prefer paused and slowed replay:

```bash
ros2 bag play /home/student24/robotproject/bags/realsense_raw_007 --clock --start-paused -r 0.5
```

## 3. Useful offline debug topics

Task state:

```bash
ros2 topic echo /task/state
```

Task events:

```bash
ros2 topic echo /task/event
```

Task decisions and reject reasons:

```bash
ros2 topic echo /task/decision
```

Current selected target:

```bash
ros2 topic echo /task/current_target_label
```

Current selected pose:

```bash
ros2 topic echo /task/current_target_pose
```

## 4. What offline mode is for

The current offline mode is meant to validate:

- the task state machine
- white-bin discovery logic
- block collection ordering
- robot slot assignment
- return-home transition
- coloured-bin dumping logic

It is not meant to prove:

- correct real robot kinematics
- correct real robot navigation completion
- correct real arm execution
- strict global pose consistency in robot coordinates

## 5. Why offline mode needs special handling

The current bag does not contain the full real robot execution context.

In particular, the bag does not contain the robot's dynamic TF chain needed for true robot-frame execution. It mainly contains:

- colour images
- aligned depth images
- camera info
- limited static TF

Because of that, offline replay cannot be treated the same as a real robot run.

So the current `offline_bag` mode intentionally relaxes several things:

- navigation is simulated
- arm execution is simulated
- white-bin dedup is not based on stable robot-frame pose distance
- target reachability checks are relaxed
- final coloured-bin dumping is opportunistic, based on what appears first in view

These relaxations are deliberate and are only for bag replay robustness.

## 6. Current offline task logic

The current pipeline uses the following high-level flow:

1. Search for a white bin
2. Explore until a white bin is locked
3. Approach the white bin
4. Search for a coloured block near that white bin
5. Pick the block
6. Place the block into a fixed robot-mounted slot
7. Repeat until all required block colours are collected
8. Navigate back to the home/bin area
9. Wait until coloured bins appear in view
10. Dump whichever collected slot matches the currently visible remaining colour bin
11. Finish when all collected slots are emptied

The fixed slot mapping is currently:

- `red -> left`
- `blue -> rear`
- `yellow -> right`

## 7. Notes on the current offline bag

The current replay bag only contains one blue bin in the final home area segment.

So for this bag, the final stage is only expected to demonstrate that:

- the pipeline reaches the final dumping stage
- a matching coloured bin can be locked
- the corresponding slot can be dumped

It is not expected to fully demonstrate all three coloured bins unless a future bag includes all of them.

## 8. Real robot workflow

For the real robot, the intended launch style is:

```bash
ros2 launch color_blob_vision vision_pipeline_manager.launch.py \
  pipeline_mode:=real_robot \
  target_frame:=base_link \
  simulate_nav:=false \
  simulate_arm:=false
```

In real robot mode, the system should move away from offline approximations and depend on real robot state and feedback.

## 9. Why real robot mode is different

Real robot mode should assume:

- the robot publishes a valid TF tree
- the arm pose is real
- navigation progress is real
- execution success/failure is real

That means real robot mode should eventually use:

- real navigation completion feedback
- real arm completion feedback
- real robot-frame target poses
- real hand-eye transform values

This is why `real_robot` mode should be stricter than `offline_bag` mode.

## 10. Required TF and coordinate expectations for the real robot

The intended coordinate chain is conceptually:

- `base_link -> ... -> ee_link -> camera_link`

The exact tree depends on the robot setup, but the main requirement is:

- the camera pose relative to the robot must be known through TF

The pipeline already supports configuration of an assumed camera mount transform using launch parameters such as:

- `ee_frame`
- `camera_frame`
- `target_frame`
- `camera_mount_x`
- `camera_mount_y`
- `camera_mount_z`
- `camera_mount_roll`
- `camera_mount_pitch`
- `camera_mount_yaw`

In the current stage these are still engineering assumptions.

For real robot deployment they should be replaced with better measured or calibrated values.

## 11. What should change when moving to the real robot

### A. TF must become real

Offline mode can tolerate weak or missing robot TF context.

Real robot mode should not.

The robot should provide:

- arm TF
- base TF
- camera TF
- any required navigation frames

### B. Reachability should become strict

Offline mode currently relaxes some reachability filtering to keep bag replay usable.

Real robot mode should restore strict reachability checks so that only robot-executable targets are accepted.

### C. Simulated actions should be replaced

Offline mode currently uses simulated progression for:

- navigation
- picking
- placing
- dumping

Real robot mode should replace those with real command/result integration.

That means `task_manager` should eventually be wired to:

- real navigation interfaces
- real manipulation interfaces
- real action or service completion feedback

### D. Final dumping should use real task feedback

Offline mode currently uses visual opportunity plus simulated timing.

Real robot mode should use:

- true robot arrival confirmation
- true manipulator completion
- real error handling

## 12. Practical meaning of the two modes

### `offline_bag`

Use this when:

- validating state-machine logic
- debugging perception transitions
- replaying bagged camera data
- investigating why targets were accepted or rejected

### `real_robot`

Use this when:

- connecting to the robot
- validating real target coordinates
- validating real navigation and manipulation integration
- testing execution rather than only perception orchestration

## 13. Current status

At the current stage, the repository now supports:

- a usable offline end-to-end prototype flow
- explicit separation between offline bag behaviour and real robot behaviour
- diagnostic topics for state transitions and target rejection reasons

The offline path should now be treated as the main environment for validating task logic before final real robot integration.

## 14. Current published integration interfaces

The current pipeline already publishes task-facing integration topics for navigation and manipulation.

### Navigation-facing output

Published by `task_manager`:

- `/task/nav_goal_name` (`std_msgs/msg/String`)

Current values may include:

- `explore_for_white_bin`
- `approach_white_bin`
- `home_bin_area_pose`

This is currently a semantic navigation command, not a numeric pose.

### Manipulator-facing outputs

Published by `task_manager`:

- `/task/arm_command` (`std_msgs/msg/String`)
- `/task/arm_target_pose` (`geometry_msgs/msg/PoseStamped`)

Current command examples:

- `pick`
- `place_to_robot_slot_left`
- `place_to_robot_slot_rear`
- `place_to_robot_slot_right`
- `dump_left_slot_to_red_bin`
- `dump_rear_slot_to_blue_bin`
- `dump_right_slot_to_yellow_bin`

### Perception-facing outputs

Published by `perception_manager`:

- `/task/current_target_pose` (`geometry_msgs/msg/PoseStamped`)
- `/task/current_target_label` (`std_msgs/msg/String`)
- `/task/current_target_visible` (`std_msgs/msg/Bool`)
- `/task/current_target_reachable` (`std_msgs/msg/Bool`)
- `/task/current_target_status` (`std_msgs/msg/String`)

These are the current task-ready perception outputs that downstream executors should use.

## 15. What navigation should subscribe to now

At minimum, navigation can subscribe to:

- `/task/nav_goal_name`

Recommended interpretation:

- `explore_for_white_bin`
  - trigger patrol / exploration / waypoint sweep
- `approach_white_bin`
  - trigger local approach behaviour toward the currently locked white bin
- `home_bin_area_pose`
  - trigger return to the fixed home/bin area

In the current repository, these are semantic commands rather than hard-coded geometry outputs.

## 16. What the manipulator should subscribe to now

At minimum, the manipulator side can subscribe to:

- `/task/arm_command`
- `/task/arm_target_pose`

Recommended interpretation:

- `pick`
  - use `/task/arm_target_pose` as the grasp target
- `place_to_robot_slot_left/rear/right`
  - execute pre-defined robot-mounted slot placement motions
- `dump_<slot>_slot_to_<color>_bin`
  - execute pre-defined dump motions for the matching slot and bin colour

## 17. What is still simulated today

At the moment, `task_manager` still simulates completion timing for:

- navigation
- pick
- place-to-slot
- dump-to-bin

So the current task pipeline publishes commands outward, but does not yet wait for real executor success/failure messages.

This is why offline validation can run without a real robot stack.

## 18. Recommended future real feedback interfaces

When the navigation and manipulation owners are ready to provide real completion feedback, the next step should be to replace simulated timing with explicit execution results.

Suggested minimum feedback topics:

- `/task/nav_result`
- `/task/arm_result`

Possible simple string values:

- navigation:
  - `success`
  - `failed`
  - `aborted`
- arm:
  - `pick_success`
  - `pick_failed`
  - `place_success`
  - `place_failed`
  - `dump_success`
  - `dump_failed`

These do not need to be the final interface format. They are only the simplest contract for replacing time-based simulation.

## 19. Recommended real-robot upgrade path

The cleanest next integration path is:

1. Keep current published command topics unchanged
2. Let nav and arm teams subscribe to those topics
3. Add simple success/failure feedback topics from nav and arm
4. Replace `simulate_nav` and `simulate_arm` state advancement with real completion events
5. Later, if needed, migrate to actions or services

This keeps the current pipeline understandable while still allowing future hardening.

## 20.output for offline

(base) student24@e-24ul523g78j:~$ ros2 topic echo /task/event
data: white_bin_search_timeout
---
data: nav:explore_for_white_bin
---
data: nav_done:explore_for_white_bin
---
data: white_bin_search_timeout
---
data: nav:explore_for_white_bin
---
data: nav_done:explore_for_white_bin
---
data: white_bin_search_timeout
---
data: nav:explore_for_white_bin
---
data: nav_done:explore_for_white_bin
---
data: white_bin_locked
---
data: nav:approach_white_bin
---
data: nav_done:approach_white_bin
---
data: block_locked:block:yellow
---
data: pick
---
data: pick_done:yellow
---
data: place_to_robot_slot_right
---
data: slot_filled:right:yellow
---
data: continue_exploration
---
data: nav:explore_for_white_bin
---
data: nav_done:explore_for_white_bin
---
data: white_bin_search_timeout
---
data: nav:explore_for_white_bin
---
data: nav_done:explore_for_white_bin
---
data: white_bin_search_timeout
---
data: nav:explore_for_white_bin
---
data: nav_done:explore_for_white_bin
---
data: white_bin_search_timeout
---
data: nav:explore_for_white_bin
---
data: nav_done:explore_for_white_bin
---
data: white_bin_search_timeout
---
data: nav:explore_for_white_bin
---
data: nav_done:explore_for_white_bin
---
data: white_bin_locked
---
data: nav:approach_white_bin
---
data: nav_done:approach_white_bin
---
data: block_locked:block:red
---
data: pick
---
data: pick_done:red
---
data: place_to_robot_slot_left
---
data: slot_filled:left:red
---
data: continue_exploration
---
data: nav:explore_for_white_bin
---
data: white_bin_locked_during_explore
---
data: nav:approach_white_bin
---
data: nav_done:approach_white_bin
---
data: block_locked:block:blue
---
data: pick
---
data: pick_done:blue
---
data: place_to_robot_slot_rear
---
data: slot_filled:rear:blue
---
data: all_blocks_collected
---
data: nav:home_bin_area_pose
---
data: nav_timeout:home_bin_area_pose
---
data: bin_locked:bin:blue
---
data: dump_rear_slot_to_blue_bin
---
data: slot_emptied:rear:blue
---
data: dump_done:rear:blue
