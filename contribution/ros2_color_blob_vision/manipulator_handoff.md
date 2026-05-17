# Manipulator Handoff

This note is for the manipulator-side owner who needs to integrate with the current `ros_nd2` task pipeline.

## 1. What this package already does

The current `color_blob_vision` pipeline already contains:

- perception target selection
- task sequencing
- robot-slot colour bookkeeping
- semantic arm command publication

The manipulator side does not need to understand the image-processing pipeline internals.

## 2. What the manipulator should subscribe to

Current task-facing manipulator outputs:

- `/task/arm_command` (`std_msgs/msg/String`)
- `/task/arm_target_pose` (`geometry_msgs/msg/PoseStamped`)

These are published by:

- `color_blob_vision/task_manager.py`

## 3. Current command meanings

Examples currently published:

- `pick`
- `place_to_robot_slot_left`
- `place_to_robot_slot_rear`
- `place_to_robot_slot_right`
- `dump_left_slot_to_red_bin`
- `dump_rear_slot_to_blue_bin`
- `dump_right_slot_to_yellow_bin`

Recommended manipulator interpretation:

- `pick`
  - use `/task/arm_target_pose` as the grasp target pose
- `place_to_robot_slot_left/rear/right`
  - move to a pre-defined robot-mounted slot pose and place the held block
- `dump_<slot>_slot_to_<color>_bin`
  - execute the corresponding slot-to-bin dumping action

## 4. Which topics contain the perception result

If the manipulator owner needs to inspect perception state directly, the current outputs are:

- `/task/current_target_pose`
- `/task/current_target_label`
- `/task/current_target_visible`
- `/task/current_target_reachable`
- `/task/current_target_status`

For most integrations, the manipulator can simply rely on:

- `/task/arm_command`
- `/task/arm_target_pose`

## 5. Coordinate expectations for real robot use

For real robot deployment, the most important expectation is:

- grasp and task poses should ultimately be interpreted in a robot frame such as `base_link`

The current pipeline supports this through `perception_manager`, which can publish selected targets in a configured `target_frame`.

Real robot mode should therefore aim for:

- `pipeline_mode:=real_robot`
- `target_frame:=base_link`

This assumes the real robot provides a valid TF chain linking:

- robot base
- arm
- end effector
- camera

## 6. Current limitation

Right now, the task layer still simulates arm completion timing.

That means:

- the task layer publishes arm commands
- but it does not yet wait for real manipulator success/failure feedback

This is acceptable for offline bag replay, but not sufficient for final robot execution.

## 7. Recommended real feedback from the manipulator

The simplest useful next step is to publish a result topic such as:

- `/task/arm_result` (`std_msgs/msg/String`)

Suggested values:

- `pick_success`
- `pick_failed`
- `place_success`
- `place_failed`
- `dump_success`
- `dump_failed`

If the manipulator team prefers a richer interface, that is also fine. The key point is that task orchestration needs a clear success/failure signal.

## 8. How the task pipeline should evolve once real arm feedback exists

Current behaviour:

- manipulator completion is time-based

Desired real behaviour:

- `task_manager` should wait for real arm feedback before transitioning after:
  - pick
  - place-to-slot
  - dump-to-bin

That means future code changes should replace:

- `simulate_arm:=true`

with:

- `simulate_arm:=false`
- wait for `/task/arm_result`

## 9. Practical integration suggestion

Recommended near-term approach:

1. Subscribe to `/task/arm_command`
2. Subscribe to `/task/arm_target_pose`
3. Map the known semantic commands into your existing arm actions
4. Publish a minimal `/task/arm_result`
5. Coordinate with task-layer updates to stop using time-based arm simulation

## 10. Summary

Manipulator integration can begin immediately with:

- `/task/arm_command`
- `/task/arm_target_pose`

The most valuable next contribution from the manipulator side is one output:

- `/task/arm_result`

That is enough to move the task pipeline from simulated completion toward real closed-loop execution.
