# Fixed tabletop and robot frame contract

For TRON2 tabletop work in this repository, the vertical height from the tabletop surface to the robot physical-base coordinate origin is fixed at **0.45 m (45 cm)**. This is a mounting constraint, not an optimizer variable. Effective 2026-10-04.

中文版本：[TABLETOP_FRAME_CONTRACT.zh-CN.md](TABLETOP_FRAME_CONTRACT.zh-CN.md)。

## Frame and height definitions

| Quantity | Required interpretation |
| --- | --- |
| Tabletop | Top contact surface; not table center, bottom or object origin |
| Robot origin | Physical robot base coordinate origin, resolved from the authored robot transform chain |
| Fixed height | `z_physical_base_world - z_tabletop_world = 0.45 m`, for a horizontal table in the common Z-up world frame |
| URDF/articulation root | A separate origin; apply the authored root-to-physical-base transform before checking the height |
| Hand/wrist/flange origin | Not the physical robot base origin |
| XY distance | Independent of the fixed vertical height; do not interpret 45 cm as a horizontal or Euclidean distance |


## Mandatory constraints

1. Resolve the physical-base origin and table top in one world frame before producing trajectories or configuring a scene.
2. Enforce `physical_base_z = table_top_z + 0.45`. Keep this relation fixed during optimization; do not optimize the gap or silently change table/base transforms to gain reachability, joint margin or collision clearance.
3. Keep authored robot link, flange and hand transforms unchanged. Convert between physical base and URDF root with the exact model transform; do not substitute an approximate root height.
4. If the fixed height makes a target infeasible, report that infeasibility and revise the task/trajectory within the fixed mounting constraint. Do not relax the 0.45 m rule automatically.

## Verification and existing artifacts

Before claiming mounting compatibility, report the robot/model identity, physical-base world pose, table top world height, URDF-root transform and measured height gap. The mathematical design value is exactly 0.45 m; numerical comparisons may allow only floating-point roundoff, never a configurable geometric gap.

This document establishes the requirement; a documentation update does not change runtime assets, trajectories, checkpoints, an active training run or hardware calibration. Existing artifacts that use another gap are **noncompliant with the fixed mounting height** and remain evidence only for their recorded layout until regenerated and independently verified. Collision/FK acceptance in another layout is not acceptance at 45 cm.

## Documentation coverage

This contract applies to TRON2 tabletop work in this repository. Local documentation must link to this English contract and its Chinese counterpart. This documentation defines the mounting constraint; it does not change runtime assets, trajectories, checkpoints, training runs or hardware calibration.
