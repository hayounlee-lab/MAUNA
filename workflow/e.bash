echo 'check whiteboard at location E'
cd ~/mauna_operation/atomic_tasks
if ! bash verify_behavioral_state.bash --custom base ; then
   while ! bash verify_behavioral_state.bash --custom tag_0; do

      if ! bash move_to_base.bash ; then
	 exit 1
      fi

   done
   bash fine_alignment.bash 0
fi

bash move_to_tag.bash 1

bash fine_alignment.bash 1b

bash read_whiteboard.bash
