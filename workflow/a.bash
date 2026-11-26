echo 'Go to location A'

cd ~/mauna_operation/atomic_tasks
if bash verify_behavioral_state.bash --custom 2 ; then
   exit 0
fi

if ! bash verify_behavioral_state.bash --custom base ; then
   while ! bash verify_behavioral_state.bash --custom tag_0 ; do

      if ! bash move_to_base.bash ; then
         exit 1
      fi

   done
fi
bash fine_alignment.bash 0n

bash move_to_tag.bash 2

bash fine_alignment.bash 2
