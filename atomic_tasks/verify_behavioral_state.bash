#!/bin/bash

# Common ROS environment setup
export INTERBOTIX_ALOHA_IS_MOBILE=true
source /opt/ros/humble/setup.bash
source ~/interbotix_ws/install/setup.bash

# Run the selected verification script
#cd ~/interbotix_ws/src/aloha/scripts/ || exit 1
cd ~/mauna_operation/internal/ || exit 1

# Argument parsing variables
CUSTOM_NAME=""
OTHER_ARGS=()

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --custom)
            CUSTOM_NAME="$2"
            shift 2
            ;;
        *)
            OTHER_ARGS+=("$1")
            shift
            ;;
    esac
done

# Check if --custom was provided
if [ -n "$CUSTOM_NAME" ]; then
    # Define custom argument lists based on the provided custom name
    case "$CUSTOM_NAME" in
        "base")
            CUSTOM_ARGS=("--mode" "position" "--target_frame" "tag_0" "--target" "0.0025" "--dist" "2" "--tol_y" "0.30" "--tol_x" "0.5" "--tol_yaw" "5.0")
            ;;
        "tag_0")
            CUSTOM_ARGS=("--mode" "tag" "--target_frame" "tag_0" "--timeout" "3")
            ;;
        "tag_1")
            CUSTOM_ARGS=("--mode" "tag" "--target_frame" "tag_1" "--timeout" "3")
            ;;
        "tag_2")
            CUSTOM_ARGS=("--mode" "tag" "--target_frame" "tag_2" "--timeout" "3")
            ;;
        "tag_3")
            CUSTOM_ARGS=("--mode" "tag" "--target_frame" "tag_3" "--timeout" "3")
            ;;
        "1")
            CUSTOM_ARGS=("--mode" "position" "--target_frame" "tag_1" "--tol_yaw" "1.0" "--target" "1.686902" "--dist" "0.926331" "--tol_y" "0.04")
            ;;
        "2")
            CUSTOM_ARGS=("--mode" "position" "--target_frame" "tag_2" "--tol_yaw" "1.0" "--target" "0.706902" "--dist" "1.04" "--tol_y" "0.10")
            ;;
        "3")
            CUSTOM_ARGS=("--mode" "position" "--target_frame" "tag_3" "--tol_yaw" "1.0" "--target" "1.171902" "--dist" "1.04" "--tol_y" "0.04")
            ;;
        "plate")
            CUSTOM_ARGS=("--mode" "arm" "--arm" "right" "--pose-0" "-0.5737" "0.1933" "0.7977" "-0.0061" "-0.2899" "-0.0015" "--pose-1" "-0.0813" "0.0092" "0.9020" "-0.0046" "-0.4403" "-0.0245" "--joint-threshold" "0.90")
            ;;
        "button")
            CUSTOM_ARGS=("--mode" "arm" "--arm" "left" "--pose-0" "0.0" "-0.96" "1.16" "0.0" "-0.3" "0.0" "--pose-1" "0.0" "0.0" "0.0" "0.0" "1.45" "1.5707" "--joint-threshold" "0.90")
            ;;
        "tube")
            CUSTOM_ARGS=("--mode" "arm" "--arm" "left" "--pose-0" "0.0" "-0.96" "1.16" "0.0" "-0.3" "0.0" "--pose-1" "-0.5" "0.1608" "0.2249" "1.2853" "-0.6677" "-0.9189" "--joint-threshold" "0.90")
            ;;

        *)
            # Default custom argument list for unrecognized names
            CUSTOM_ARGS=("--default_flag" "--name" "$CUSTOM_NAME")
            ;;
    esac

    echo "Running with custom argument list for: $CUSTOM_NAME"
    python3 verify_behavioral_state.py "${CUSTOM_ARGS[@]}"
    exit $?
else
    # If --custom is not provided, pass the original arguments as they are
    python3 verify_behavioral_state.py "${OTHER_ARGS[@]}"
    exit $?
fi
