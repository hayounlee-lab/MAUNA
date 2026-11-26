echo 'Place the microplate into the thermocycler at location D'

cd ~/mauna_operation/atomic_tasks
if ! bash verify_behavioral_state.bash --custom tag_1 ; then
   exit 1
fi

MAX_RETRIES=3
COUNT=0

bash fine_alignment.bash 1f
bash plate_loading.bash

while [ $COUNT -lt $MAX_RETRIES ]
do
    # Check loading status before verification
    bash inspect_loading.bash

    # Run verification script with custom argument and capture the exit code
    bash verify_behavioral_state.bash --custom plate; EXIT_CODE=$?

    if [ $EXIT_CODE -eq 0 ]; then
        echo "Result: [Pose 0] detected. Exiting script."
        bash close_lid.bash
        exit 0
    elif [ $EXIT_CODE -eq 2 ]; then
        COUNT=$((COUNT + 1))
        echo "Result: [Pose 1] detected. Executing retry sequence. (Retry count: $COUNT)"

        bash fix_plate_loading.bash

        echo "------------------------------------------"
    else
        echo "Matching pose not found. Retrying..."
    fi
done

echo "Maximum retry limit ($MAX_RETRIES) exceeded. Exiting."
exit 1
