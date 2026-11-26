#!/bin/bash
echo 'Check the state of the switch at location A, and if it is red, press the switch'

cd ~/mauna_operation/atomic_tasks
if ! bash verify_behavioral_state.bash --custom tag_2 ; then
   exit 1
fi

MAX_RETRIES=5
COUNT=0

while [ $COUNT -lt $MAX_RETRIES ]
do
    # Check device status before verification
    bash check_device_status.bash

    # Run verification script with custom argument and capture the exit code
    bash verify_behavioral_state.bash --custom button; EXIT_CODE=$?

    if [ $EXIT_CODE -eq 0 ]; then
        echo "Result: [Pose 0] detected. Exiting script."
        exit 0
    elif [ $EXIT_CODE -eq 2 ]; then
        COUNT=$((COUNT + 1))
        echo "Result: [Pose 1] detected. Executing retry sequence. (Retry count: $COUNT)"

        bash fine_alignment.bash 2f
        bash press_button.bash

        echo "------------------------------------------"
    else
        echo "Matching pose not found. Retrying..."
    fi
done

echo "Maximum retry limit ($MAX_RETRIES) exceeded. Exiting."
exit 1
