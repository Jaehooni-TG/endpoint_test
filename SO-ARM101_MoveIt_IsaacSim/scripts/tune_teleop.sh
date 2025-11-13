#!/usr/bin/env bash
set -euo pipefail

# Simple tuning helper for SO-ARM teleop runtime parameters
# Usage: source install/setup.bash && ./scripts/tune_teleop.sh

# Defaults (override by exporting env vars before running)
ANG_GAIN=${ANG_GAIN:-2.0}
LIN_MAX=${LIN_MAX:-0.5}
ANG_MAX=${ANG_MAX:-1.5}

BRIDGE_POS_ALPHA=${BRIDGE_POS_ALPHA:-0.90}
BRIDGE_ORI_ALPHA=${BRIDGE_ORI_ALPHA:-0.85}
BRIDGE_MAX_POS_STEP=${BRIDGE_MAX_POS_STEP:-0.04}
BRIDGE_MAX_ORI_STEP=${BRIDGE_MAX_ORI_STEP:-0.52}

POSE_NODE=/so_arm_pose_to_servo

echo "[tune] Setting pose_to_servo params: angular_gain=$ANG_GAIN, max_linear=$LIN_MAX, max_angular=$ANG_MAX"
ros2 param set $POSE_NODE angular_gain $ANG_GAIN || true
ros2 param set $POSE_NODE max_linear_speed $LIN_MAX || true
ros2 param set $POSE_NODE max_angular_speed $ANG_MAX || true

echo "[tune] Recommended bridge launch args (paste into your run command):"
cat <<EOF
  --ros-args \\
    -p position_smoothing_alpha:=$BRIDGE_POS_ALPHA \\
    -p orientation_smoothing_alpha:=$BRIDGE_ORI_ALPHA \\
    -p max_position_step:=$BRIDGE_MAX_POS_STEP \\
    -p max_orientation_step:=$BRIDGE_MAX_ORI_STEP
EOF

echo "[tune] Note: MoveIt Servo thresholds edited in config require rebuild:"
echo "  colcon build --packages-select so_arm_moveit_config && source install/setup.bash"
