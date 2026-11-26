echo 'Vortex the tube at location C'
cd ~/mauna_operation/atomic_tasks
if ! bash verify_behavioral_state.bash --custom tag_3 ; then
   exit 1
fi
bash fine_alignment.bash 3f
bash vortexing.bash

# Check loading status before verification
bash inspect_tube.bash

# Run verification script with custom argument and capture the exit code
bash verify_behavioral_state.bash --custom tube; EXIT_CODE=$?

if [ $EXIT_CODE -eq 0 ]; then
    echo "Result: [Pose 0] detected. Tube in place. Exiting script."
    exit 0
elif [ $EXIT_CODE -eq 2 ]; then
    echo "Result: [Pose 1] detected. Tube not in place. Error."
    exit 2
else
    echo "Matching pose not found."
exit 1
fi
