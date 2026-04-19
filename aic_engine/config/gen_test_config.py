#!/usr/bin/env python3
"""
gen_test_config.py — generate a randomised ant_test_config.yaml variant.

Each run produces a fresh YAML with randomised board poses and rail
translations drawn from the same parameter ranges the competition uses.
Use a fixed --seed for reproducible regression runs.

Usage:
    python3 gen_test_config.py                       # random seed, 3 trials
    python3 gen_test_config.py --seed 42             # fixed seed
    python3 gen_test_config.py --trials 6 --seed 7  # 6 trials
    python3 gen_test_config.py --out my_config.yaml
"""

import argparse
import math
import random
import textwrap
from pathlib import Path

# ── Board pose ranges (aic_world frame) ──────────────────────────────────────
# x/y/z drawn from uniform distributions centred on the competition values.
# Yaw covers the full competition range plus ±45° to stress-test robustness.
BOARD_X_RANGE   = (0.25, 0.38)
BOARD_Y_RANGE   = (-0.12, 0.08)
BOARD_Z_RANGE   = (1.08, 1.28)
# Competition uses +17° (0.297) and −45° (−0.785); extend to ±75° for stress.
BOARD_YAW_RANGE = (-1.309, 1.309)   # ±75° in radians

# ── Rail translation limits (from task_board_limits in sample_config) ────────
NIC_RAIL_RANGE   = (-0.048,  0.036)
SC_RAIL_RANGE    = (-0.055,  0.055)
MOUNT_RAIL_RANGE = (-0.09425, 0.09425)

# ── Fixed cable attachment offset (unchanged across all scenarios) ────────────
SFP_CABLE_Z_OFFSET = 0.04245
SC_CABLE_Z_OFFSET  = 0.04045

SCORING_TOPICS = textwrap.dedent("""\
scoring:
  topics:
    - topic:
        name: "/joint_states"
        type: "sensor_msgs/msg/JointState"
    - topic:
        name: "/tf"
        type: "tf2_msgs/msg/TFMessage"
    - topic:
        name: "/tf_static"
        type: "tf2_msgs/msg/TFMessage"
        latched: true
    - topic:
        name: "/scoring/tf"
        type: "tf2_msgs/msg/TFMessage"
    - topic:
        name: "/aic/gazebo/contacts/off_limit"
        type: "ros_gz_interfaces/msg/Contacts"
    - topic:
        name: "/fts_broadcaster/wrench"
        type: "geometry_msgs/msg/WrenchStamped"
    - topic:
        name: "/aic_controller/joint_commands"
        type: "aic_control_interfaces/msg/JointMotionUpdate"
    - topic:
        name: "/aic_controller/pose_commands"
        type: "aic_control_interfaces/msg/MotionUpdate"
    - topic:
        name: "/scoring/insertion_event"
        type: "std_msgs/msg/String"
    - topic:
        name: "/aic_controller/controller_state"
        type: "aic_control_interfaces/msg/ControllerState"
""")

TASK_BOARD_LIMITS = textwrap.dedent("""\
task_board_limits:
  nic_rail:
    min_translation: -0.048
    max_translation: 0.036
  sc_rail:
    min_translation: -0.055
    max_translation: 0.055
  mount_rail:
    min_translation: -0.09425
    max_translation: 0.09425
""")

ROBOT_HOME = textwrap.dedent("""\
robot:
  home_joint_positions:
    shoulder_pan_joint: -0.1597
    shoulder_lift_joint: -1.3542
    elbow_joint: -1.6648
    wrist_1_joint: -1.6933
    wrist_2_joint: 1.5710
    wrist_3_joint: 1.4110
""")


def rng_val(lo, hi):
    return random.uniform(lo, hi)


def gen_sfp_trial(trial_id: int, seed_info: str) -> str:
    x   = rng_val(*BOARD_X_RANGE)
    y   = rng_val(*BOARD_Y_RANGE)
    z   = rng_val(*BOARD_Z_RANGE)
    yaw = rng_val(*BOARD_YAW_RANGE)
    nic_t = rng_val(*NIC_RAIL_RANGE)

    return textwrap.dedent(f"""\
  trial_{trial_id}:  # SFP  yaw={math.degrees(yaw):.1f}°  {seed_info}
    scene:
      task_board:
        pose:
          x: {x:.4f}
          y: {y:.4f}
          z: {z:.4f}
          roll: 0.0
          pitch: 0.0
          yaw: {yaw:.4f}
        nic_rail_0:
          entity_present: True
          entity_name: "nic_card_0"
          entity_pose:
            translation: {nic_t:.4f}
            roll: 0.0
            pitch: 0.0
            yaw: 0.0
        nic_rail_1:
          entity_present: False
        nic_rail_2:
          entity_present: False
        nic_rail_3:
          entity_present: False
        nic_rail_4:
          entity_present: False
        sc_rail_0:
          entity_present: False
        sc_rail_1:
          entity_present: False
      cables:
        cable_0:
          pose:
            gripper_offset:
              x: 0.0
              y: 0.015385
              z: {SFP_CABLE_Z_OFFSET}
            roll: 0.4432
            pitch: -0.4838
            yaw: 1.3303
          attach_cable_to_gripper: True
          cable_type: "sfp_sc_cable"
    tasks:
      task_1:
        cable_type: "sfp_sc"
        cable_name: "cable_0"
        plug_type: "sfp"
        plug_name: "sfp_tip"
        port_type: "sfp"
        port_name: "sfp_port_0"
        target_module_name: "nic_card_mount_0"
        time_limit: 180
""")


def gen_sc_trial(trial_id: int, seed_info: str) -> str:
    x   = rng_val(*BOARD_X_RANGE)
    y   = rng_val(*BOARD_Y_RANGE)
    z   = rng_val(*BOARD_Z_RANGE)
    yaw = rng_val(*BOARD_YAW_RANGE)
    sc_t = rng_val(*SC_RAIL_RANGE)

    return textwrap.dedent(f"""\
  trial_{trial_id}:  # SC  yaw={math.degrees(yaw):.1f}°  {seed_info}
    scene:
      task_board:
        pose:
          x: {x:.4f}
          y: {y:.4f}
          z: {z:.4f}
          roll: 0.0
          pitch: 0.0
          yaw: {yaw:.4f}
        nic_rail_0:
          entity_present: False
        nic_rail_1:
          entity_present: False
        nic_rail_2:
          entity_present: False
        nic_rail_3:
          entity_present: False
        nic_rail_4:
          entity_present: False
        sc_rail_0:
          entity_present: True
          entity_name: "sc_port_0"
          entity_pose:
            translation: {sc_t:.4f}
            roll: 0.0
            pitch: 0.0
            yaw: 0.0
        sc_rail_1:
          entity_present: False
      cables:
        cable_0:
          pose:
            gripper_offset:
              x: 0.0
              y: 0.015385
              z: {SC_CABLE_Z_OFFSET}
            roll: 0.4432
            pitch: -0.4838
            yaw: 1.3303
          attach_cable_to_gripper: True
          cable_type: "sfp_sc_cable_reversed"
    tasks:
      task_1:
        cable_type: "sfp_sc"
        cable_name: "cable_0"
        plug_type: "sc"
        plug_name: "sc_tip"
        port_type: "sc"
        port_name: "sc_port_base"
        target_module_name: "sc_port_0"
        time_limit: 180
""")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seed",   type=int, default=None,
                        help="RNG seed for reproducible configs (default: random)")
    parser.add_argument("--trials", type=int, default=3,
                        help="Number of trials to generate (default: 3)")
    parser.add_argument("--sc-fraction", type=float, default=0.33,
                        help="Fraction of trials that are SC type (default: 0.33)")
    parser.add_argument("--out",    type=str, default=None,
                        help="Output file path (default: stdout)")
    args = parser.parse_args()

    seed = args.seed if args.seed is not None else random.randint(0, 2**31)
    random.seed(seed)

    lines = [
        f"# Auto-generated by gen_test_config.py  seed={seed}\n",
        SCORING_TOPICS,
        TASK_BOARD_LIMITS,
        "trials:\n",
    ]

    n_sc  = max(1, round(args.trials * args.sc_fraction))
    n_sfp = args.trials - n_sc
    types = ["sfp"] * n_sfp + ["sc"] * n_sc
    random.shuffle(types)

    for i, t in enumerate(types, start=1):
        seed_info = f"seed={seed} trial={i}"
        if t == "sfp":
            lines.append(gen_sfp_trial(i, seed_info))
        else:
            lines.append(gen_sc_trial(i, seed_info))

    lines.append(ROBOT_HOME)

    output = "\n".join(lines)

    if args.out:
        Path(args.out).write_text(output)
        print(f"Written to {args.out}  (seed={seed})")
    else:
        print(output)


if __name__ == "__main__":
    main()
