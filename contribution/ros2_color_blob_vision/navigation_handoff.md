# Navigation Handoff

This note is for the navigation-side owner who needs to integrate with the current `ros_nd2` task pipeline.

## 1. What this package already does

The current `color_blob_vision` pipeline already contains:

- perception selection
- task-state orchestration
- semantic navigation requests

The navigation system does not need to understand image processing details.

It only needs to react to task-level navigation commands.

## 2. What navigation should subscribe to

Current task-facing navigation output:

- `/task/nav_goal_name` (`std_msgs/msg/String`)

This is published by:

- `color_blob_vision/task_manager.py`

## 3. Current command meanings

Current command values may include:

- `explore_for_white_bin`
- `approach_white_bin`
- `home_bin_area_pose`

Recommended behaviour:

- `explore_for_white_bin`
  - perform exploration, waypoint sweep, or patrol behaviour intended to discover a new white bin
- `approach_white_bin`
  - move from search/exploration behaviour into local approach behaviour toward the currently locked white bin
- `home_bin_area_pose`
  - navigate to the fixed home/bin area

## 4. What navigation does not need to do

Navigation does not need to:

- inspect raw image topics
- classify block colours
- understand white-bin or colour-bin detection details

Those are already handled upstream by perception and task orchestration.

## 5. Current limitation

Right now, the task layer still simulates navigation completion using time.

That means:

- `task_manager` publishes navigation intent
- but does not yet wait for real navigation success/failure feedback

This is enough for offline bag replay, but not enough for real robot deployment.

## 6. Recommended real feedback from navigation

The simplest useful next step is for navigation to publish a feedback topic such as:

- `/task/nav_result` (`std_msgs/msg/String`)

Suggested values:

- `success`
- `failed`
- `aborted`

If the team prefers a richer format, that is also fine. The important part is that task orchestration can distinguish success from failure.

## 7. How the task pipeline should evolve once real feedback exists

Current behaviour:

- navigation completion is time-based

Desired real behaviour:

- `task_manager` should wait for real nav feedback before transitioning from:
  - exploration
  - white-bin approach
  - return-home

That means future code changes should replace:

- `simulate_nav:=true`

with:

- `simulate_nav:=false`
- wait for `/task/nav_result`

## 8. What navigation may eventually need from perception

In the current semantic interface, navigation does not need direct geometric target data.

But for local approach refinement, navigation may later benefit from:

- `/task/current_target_pose`
- `/task/current_target_label`

Especially for:

- local approach toward a locked white bin

This is optional for the current stage, but useful for future refinement.

## 9. Practical integration suggestion

Recommended near-term approach:

1. Subscribe to `/task/nav_goal_name`
2. Map each command string to your existing navigation behaviour
3. Publish a minimal `/task/nav_result`
4. Coordinate with task-layer updates to stop using time-based nav simulation

## 10. Summary

Navigation currently only needs one input to begin integration:

- `/task/nav_goal_name`

The most valuable next contribution from the navigation side is one output:

- `/task/nav_result`

That is enough to move the task pipeline from offline-style timing simulation toward true robot execution.
