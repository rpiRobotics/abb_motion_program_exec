echo "Installing abb_motion_program_exec"

config -filename $BOOTPATH/config/SYS.cfg -domain SYS -replace
config -filename $BOOTPATH/config/EIO.cfg -domain EIO -replace

copy -from $BOOTPATH/RAPID/motion_program_exec.mod -to $HOME/motion_program_exec.mod
copy -from $BOOTPATH/RAPID/motion_program_logger.mod -to $HOME/motion_program_logger.mod
copy -from $BOOTPATH/RAPID/error_reporter.mod -to $HOME/error_reporter.mod

copy -from $BOOTPATH/RAPID/mp.sys -to $HOME/mp.sys
