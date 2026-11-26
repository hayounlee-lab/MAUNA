echo 'Go to location B'

cd ~/mauna_operation/atomic_tasks
if bash verify_behavioral_state.bash --custom base ; then
   exit 0
fi

while ! bash verify_behavioral_state.bash --custom tag_0 ; do

   if ! bash move_to_base.bash ; then
      exit 1
   fi

done

if bash verify_behavioral_state.bash --custom base ; then
    exit 0
fi

bash fine_alignment.bash 0
