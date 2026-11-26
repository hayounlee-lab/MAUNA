echo 'Go to location D'

cd ~/mauna_operation/atomic_tasks
if bash verify_behavioral_state.bash --custom 1 ; then
   exit 0
fi

if ! bash verify_behavioral_state.bash --custom base ; then
   while ! bash verify_behavioral_state.bash --custom tag_0; do

      if ! bash move_to_base.bash ; then
         exit 1
      fi

   done
   bash fine_alignment.bash 0
fi

bash move_to_tag.bash 1

bash fine_alignment.bash 1
