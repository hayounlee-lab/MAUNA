# Repo Structure

* mauna (Requires the upstream [ALOHA}(https://github.com/Interbotix/aloha) repository to be installed)
  * ``act`` Modified scripts for ACT. (Requires the upstream [act_plus_plus](https://github.com/Interbotix/act_plus_plus) repository to be installed)
  * ``atomic_tasks`` Definition of basic, reusable robotic atomic tasks
  * ``workflow`` High-level task sequencing and logic orchestration
  * ``launch`` System startup and configuration scripts
  * ``internal`` Shared utilities and core internal logic
  * ``system`` System commands for startup and shutdown
  * ``voice_command`` Voice recognition and command processing component. This was forked from [Whisper_Speech_Command](https://github.com/mnyhu201/Whisper_Speech_Command) and refined to include only the necessary components for the Mauna system.
  * ``pi5_tag` Contains AprilTag detection scripts to run on the Raspberry Pi 5
