#!/bin/bash

# Function to display usage instructions
usage() {
    echo "Usage: $0 {0|0n|1|1b|1f|2|2f|3|3f}"
    echo "Example: $0 0"
    exit 1
}

# Check if the argument count is incorrect
if [ $# -ne 1 ]; then
    usage
fi

TAG_TYPE=$1

# 1. Common environment settings and initialization
export INTERBOTIX_ALOHA_IS_MOBILE=true # true for Mobile, false for Stationary
source /opt/ros/humble/setup.bash
source ~/interbotix_ws/install/setup.bash
cd ~/mauna_operation/internal/ || exit 1

bash slate_on.bash

# 2. Branch processing based on tag type
case "$TAG_TYPE" in
    0)
        echo "=== Running align_tag0 ==="
        python3 align_coarse_approach.py --target_frame tag_0 --target 0.0025 --dist 2.00
        python3 align_yaw.py --target_frame tag_0 --yaw_tol 1.0 --target 0.0025
        python3 align_lateral_x.py --target_frame tag_0 --target 0.0025 --precision 0.20 --yaw_tol 1.0
        python3 align_yaw.py --target_frame tag_0 --yaw_tol 1.0 --target 0.0025
        ;;
    0n)
        echo "=== Running align_tag0n ==="
        python3 align_coarse_approach.py --target_frame tag_0 --target 0.0025 --dist 1.10
        python3 align_yaw.py --target_frame tag_0 --yaw_tol 1.0 --target 0.0025
        python3 align_lateral_x.py --target_frame tag_0 --target 0.0025 --precision 0.20 --yaw_tol 1.0
        python3 align_yaw.py --target_frame tag_0 --yaw_tol 1.0 --target 0.0025
        ;;
    1)
        echo "=== Running align_tag1 ==="
        python3 align_coarse_approach.py --target_frame tag_1 --target 1.686902 --dist 1.30
        python3 align_yaw.py --target_frame tag_1 --yaw_tol 1.0 --target 1.686902
        python3 align_lateral_x.py --target_frame tag_1 --target 1.686902 --precision 0.04 --yaw_tol 1.0
        python3 align_longitudinal_y.py --target_frame tag_1 --target 1.686902 --dist 0.926311 --x_tol 0.05 --yaw_tol 1.0
        python3 align_yaw.py --target_frame tag_1 --yaw_tol 1.0 --target 1.686902
        ;;
    1b)
        echo "=== Running align_tag1b ==="
        python3 align_coarse_approach.py --target_frame tag_1 --target 1.686902 --dist 2.00
        python3 align_yaw.py --target_frame tag_1 --yaw_tol 1.0 --target 1.686902
        python3 align_lateral_x.py --target_frame tag_1 --target 1.686902 --precision 0.10 --yaw_tol 1.0
        python3 align_longitudinal_y.py --target_frame tag_1 --target 1.686902 --dist 2.00 --x_tol 0.05 --yaw_tol 1.0
        python3 align_yaw.py --target_frame tag_1 --yaw_tol 1.0 --target 1.686902
        ;;
    1f)
        echo "=== Running align_tag1f ==="
        python3 align_coarse_approach.py --target_frame tag_1 --target 1.686902 --dist 1.30
        python3 align_yaw.py --target_frame tag_1 --yaw_tol 1.0 --target 1.686902
        python3 align_lateral_x.py --target_frame tag_1 --target 1.686902 --precision 0.04 --yaw_tol 1.0
        python3 align_longitudinal_y.py --target_frame tag_1 --target 1.686902 --dist 0.926311 --x_tol 0.05 --yaw_tol 1.0
        python3 align_lateral_x.py --target_frame tag_1 --target 1.686902 --precision 0.002 --yaw_tol 0.1
        python3 align_longitudinal_y.py --target_frame tag_1 --target 1.686902 --dist 0.926311 --x_tol 0.002 --yaw_tol 0.1
        python3 align_yaw.py --target_frame tag_1 --yaw_tol 0.03 --target 1.686902
        sleep 5
        python3 align_yaw.py --target_frame tag_1 --yaw_tol 0.03 --target 1.686902
        sleep 5
        python3 align_yaw.py --target_frame tag_1 --yaw_tol 0.03 --target 1.686902
        ;;
    2)
        echo "=== Running align_tag2 ==="
        python3 align_coarse_approach.py --target_frame tag_2 --target 0.706902 --dist 1.20
        python3 align_yaw.py --target_frame tag_2 --yaw_tol 1.0 --target 0.706902
        python3 align_lateral_x.py --target_frame tag_2 --target 0.706902 --precision 0.04 --yaw_tol 1.0
        python3 align_longitudinal_y.py --target_frame tag_2 --target 0.706902 --dist 1.04 --x_tol 0.05 --yaw_tol 1.0
        python3 align_yaw.py --target_frame tag_2 --yaw_tol 0.03 --target 0.706902
        sleep 5
        python3 align_yaw.py --target_frame tag_2 --yaw_tol 0.03 --target 0.706902
        sleep 5
        python3 align_yaw.py --target_frame tag_2 --yaw_tol 0.03 --target 0.706902
        ;;
    2f)
        echo "=== Running align_tag2f1 ==="
        python3 arm_commander.py --arm left --pos 0.0  -0.96  1.16  0.0  -0.3  0.0 --time 2.0
        python3 align_lateral_x.py --target_frame tag_2 --target 0.706902 --precision 0.003 --yaw_tol 0.1
        python3 align_longitudinal_y.py --target_frame tag_2 --target 0.706902 --dist 1.04 --x_tol 0.003 --yaw_tol 0.1
        python3 align_yaw.py --target_frame tag_2 --yaw_tol 0.03 --target 0.706902
        sleep 5
        python3 align_yaw.py --target_frame tag_2 --yaw_tol 0.03 --target 0.706902
        sleep 5
        python3 align_yaw.py --target_frame tag_2 --yaw_tol 0.03 --target 0.706902
        ;;
    3)
        echo "=== Running align_tag3 ==="
        python3 align_coarse_approach.py --target_frame tag_3 --target 1.171902 --dist 1.20
        python3 align_yaw.py --target_frame tag_3 --yaw_tol 1.0 --target 1.171902
        python3 align_lateral_x.py --target_frame tag_3 --target 1.171902 --precision 0.04 --yaw_tol 1.0
        python3 align_longitudinal_y.py --target_frame tag_3 --target 1.171902 --dist 1.04 --x_tol 0.05 --yaw_tol 1.0
        python3 align_yaw.py --target_frame tag_3 --yaw_tol 1.0 --target 1.171902
        sleep 5
        python3 align_yaw.py --target_frame tag_3 --yaw_tol 1.0 --target 1.171902
        sleep 5
        python3 align_yaw.py --target_frame tag_3 --yaw_tol 1.0 --target 1.171902
        ;;
    3f)
        echo "=== Running align_tag3 ==="
        python3 align_lateral_x.py --target_frame tag_3 --target 1.171902 --precision 0.001 --yaw_tol 0.1
        python3 align_longitudinal_y.py --target_frame tag_3 --target 1.171902 --dist 1.04 --x_tol 0.001 --yaw_tol 0.1
        python3 align_yaw.py --target_frame tag_3 --yaw_tol 0.03 --target 1.171902
        sleep 5
        python3 align_yaw.py --target_frame tag_3 --yaw_tol 0.03 --target 1.171902
        sleep 5
        python3 align_yaw.py --target_frame tag_3 --yaw_tol 0.03 --target 1.171902
        ;;
    *)
        echo "Error: Unsupported tag option."
        usage
        ;;
esac
