print "Installing abb_motion_program_exec"

if_feature_present -id open.wasontech.abb_motion_program_exec_robotware.motion_program_exec_multimove -label MOTION_PROGRAM_MULTIMOVE
print -text "abb_motion_program_exec installing for single robot"
config -filename $BOOTPATH/config/SYS.cfg -domain SYS -replace
config -filename $BOOTPATH/config/EIO.cfg -domain EIO -replace
goto -label COPY_FILES
#MOTION_PROGRAM_MULTIMOVE
print -text "abb_motion_program_exec installing for multimove robot"
config -filename $BOOTPATH/config_multimove/SYS.cfg -domain SYS -replace
config -filename $BOOTPATH/config_multimove/EIO.cfg -domain EIO -replace

#COPY_FILES


copy -from $BOOTPATH/RAPID/motion_program_exec.mod -to $HOME/motion_program_exec.mod
copy -from $BOOTPATH/RAPID/motion_program_logger.mod -to $HOME/motion_program_logger.mod
copy -from $BOOTPATH/RAPID/error_reporter.mod -to $HOME/error_reporter.mod

copy -from $BOOTPATH/RAPID/motion_program_shared.sys -to $HOME/motion_program_shared.sys
