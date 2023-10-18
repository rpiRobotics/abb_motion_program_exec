# Copyright 2022 Wason Technology, LLC
#                     Rensselaer Polytechnic Institute
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import re
from typing import Callable, NamedTuple, Any, List, Union, TYPE_CHECKING
import struct
import numpy as np
import io
import time
import datetime
from abb_robot_client.rws import RWS
from .commands.rapid_types import *
from .commands import util
from .commands import commands
from .commands.command_base import command_append_method
from .commands import egm_commands
from .commands.egm_commands import EGMStreamConfig, EGMJointTargetConfig, EGMPoseTargetConfig, EGMPathCorrectionConfig, \
    egm_minmax, egmframetype

MOTION_PROGRAM_FILE_VERSION = 10011

class MotionProgramResultLog(NamedTuple):
    timestamp: str
    column_headers: List[str]
    data: np.array

def _unpack_motion_program_result_log(b: bytes):
    f = io.BytesIO(b)
    file_ver = util.read_num(f)
    assert file_ver == MOTION_PROGRAM_FILE_VERSION
    timestamp_str = util.read_str(f)
    header_str = util.read_str(f)
    headers = header_str.split(",")
    data_flat = np.frombuffer(b[f.tell():], dtype=np.float32)
    data = data_flat.reshape((-1,len(headers)))
    return MotionProgramResultLog(timestamp_str, headers, data)

def _get_motion_program_file(path: str, motion_program: "MotionProgram", task="T_ROB1", preempt_number=None, seqno = None):
    b = motion_program.get_program_bytes(seqno)
    assert len(b) > 0, "Motion program must not be empty"
    ramdisk = path
    filename = f"{ramdisk}/motion_program"
    if task != "T_ROB1":
        task_m = re.match(r"^.*[A-Za-z_](\d+)$",task)
        if task_m:
            filename_ind = int(task_m.group(1))
            filename = f"{filename}{filename_ind}"
    if preempt_number is not None:
        filename = f"{filename}_p{preempt_number}"
    filename = f"{filename}.bin"
    return filename, b

tool0 = tooldata(True,pose([0,0,0],[1,0,0,0]),loaddata(0.001,[0,0,0.001],[1,0,0,0],0,0,0))
wobj0 = wobjdata(False, True, "", pose([0,0,0],[1,0,0,0]), pose([0,0,0],[1,0,0,0]))
load0 = loaddata(0.001,[0,0,0.001],[1,0,0,0],0,0,0)

class MotionProgram:
    """
    Class representing a Motion Program. A Motion Program is a sequences of robot motion primitives that
    can be executed by the interpreter program running on the robot. This program must be installed before the
    motion program can be executed.

    Motion commands are appended to the program by calling one of the motion program command functions. Currently 
    supported commands are ``MoveAbsJ``, ``MoveJ``, ``MoveL``, ``MoveC``, ``WaitTime``, ``CirPathMode``,
    ``SyncMoveOn``, ``SyncMoveOff``, ``EGMRunJoint``, ``EGMRunPose``, ``EGMMoveL``, and ``EGMMoveC``
        
    :param first_cmd_num: The first command number for the motion program. Defaults to 1
    :param tooldata: The tooldata to use for the motion program. Defaults to tool0
    :param wobj: The wobjdata to use for the motion program. Defaults to wobj0
    :param timestamp: The timestamp for the motion program. Defaults to current clock time
    :param egm_config: The EGM configuration for the motion program. Can be used to activate feedback streaming,
                       joint control, pose control, or path correction
    :param seqno: The sequence number of the command. Used by drivers commanding the robot
    :param gripload: The loaddata for the payload currently held by the robot

    """


    MoveAbsJ = command_append_method(commands.MoveAbsJCommand)
    MoveJ = command_append_method(commands.MoveJCommand)
    MoveL = command_append_method(commands.MoveLCommand)
    MoveC = command_append_method(commands.MoveCCommand)
    WaitTime = command_append_method(commands.WaitTimeCommand)
    CirPathMode = command_append_method(commands.CirPathModeCommand)
    SyncMoveOn = command_append_method(commands.SyncMoveOnCommand)
    SyncMoveOff = command_append_method(commands.SyncMoveOffCommand)

    EGMRunJoint = command_append_method(egm_commands.EGMRunJointCommand)
    EGMRunPose = command_append_method(egm_commands.EGMRunPoseCommand)
    EGMMoveL = command_append_method(egm_commands.EGMMoveLCommand)
    EGMMoveC = command_append_method(egm_commands.EGMMoveCCommand)

    def __init__(self,first_cmd_num: int=1, tool: tooldata = None, wobj: wobjdata = None, timestamp: str = None, 
        egm_config: Union[EGMStreamConfig,EGMJointTargetConfig,EGMPoseTargetConfig,EGMPathCorrectionConfig] = None, 
        seqno: int = 0, gripload: loaddata = None):

        self._commands = []

        if timestamp is None:
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S-%f")[:-2]
        assert re.match(r"^\d{4}\-\d{2}\-\d{2}-\d{2}\-\d{2}\-\d{2}\-\d{4}$", timestamp)

        self._timestamp = timestamp
        
        self.tool = tool
        self.wobj = wobj
        self.gripload = gripload
        if self.tool is None:
            self.tool = tool0
        if self.wobj is None:
            self.wobj = wobj0
        if self.gripload is None:
            self.gripload = load0

        self._first_cmd_num=first_cmd_num

        self._egm_config = egm_config
        self._seqno = seqno

    def _append_command(self, cmd):
        self._commands.append(cmd)

    def write_program(self, f: io.IOBase, seqno = None):
        """
        Write binary motion program to binary file. The robot controller program will interpret the binary
        file to execute the motion program.

        :param f: The target file to write program
        :param seqno: The seqno of the program. Used by drivers, can be ignored for normal use
        """
        # Version number
        f.write(util.num_to_bin(MOTION_PROGRAM_FILE_VERSION))
        
        f.write(util.tooldata_to_bin(self.tool))
        f.write(util.wobjdata_to_bin(self.wobj))
        f.write(util.loaddata_to_bin(self.gripload))
        f.write(util.str_to_bin(self._timestamp))
        if seqno is not None:
            f.write(util.num_to_bin(seqno))
        else:
            f.write(util.num_to_bin(self._seqno))

        egm_commands.write_egm_config(f,self._egm_config)

        for i in range(len(self._commands)):
            cmd = self._commands[i]
            cmd_num = i + self._first_cmd_num
            f.write(util.num_to_bin(cmd_num))
            f.write(util.num_to_bin(cmd.command_opcode))

            cmd.write_params(f)    

    def get_program_bytes(self, seqno = None) -> bytes:
        """
        Return binary motion program

        :param seqno: The seqno of the program. Used by drivers, can be ignored for normal use
        """
        f = io.BytesIO()
        self.write_program(f, seqno)
        return f.getvalue()

    def write_program_rapid(self, f: io.TextIOBase, module_name="motion_program_exec_gen", sync_move=False):
        """
        Write equivalent RAPID program of the motion program. Useful for debugging motion programs.

        :param f: The target file to write text RAPID program
        :param module_name: The name of the RAPID module to write
        :param sync_move: Set to True if the program is being used with a MulitMove synchronous program
        """

        ver = MOTION_PROGRAM_FILE_VERSION
        tooldata_str = self.tool.to_rapid()
        wobjdata_str = self.wobj.to_rapid()
        timestamp_str = self._timestamp


        print(f"MODULE {module_name}", file=f)
        print(f"    ! abb_motion_program_exec format version {ver}", file=f)
        print(f"    ! abb_motion_program_exec timestamp {timestamp_str}", file=f)
        print(f"    TASK PERS tooldata motion_program_tool := {tooldata_str};", file=f)
        print(f"    TASK PERS wobjdata motion_program_wobj := {wobjdata_str};", file=f)
        if sync_move:
             print("    PERS tasks task_list{2} := [ [\"T_ROB1\"], [\"T_ROB2\"] ];", file=f)
             print("    VAR syncident motion_program_sync1;", file=f)
             print("    VAR syncident motion_program_sync2;", file=f)

        print(f"    PROC main()", file=f)

        for i in range(len(self._commands)):
            cmd = self._commands[i]
            cmd_num = i + self._first_cmd_num

            print(f"        ! cmd_num = {cmd_num}",file=f)

            print(f"        {cmd.to_rapid(sync_move=sync_move, cmd_num=cmd_num)}", file=f)
        
        print("    ENDPROC", file=f)
        print("ENDMODULE", file=f)
        
    def get_program_rapid(self, module_name="motion_program_exec_gen", sync_move=False) -> str:
        """
        Returns equivalent RAPID program of the motion program. Useful for debugging motion programs.

        :param module_name: The name of the RAPID module to write
        :param sync_move: Set to True if the program is being used with a MulitMove synchronous program
        """
        o = io.StringIO()
        self.write_program_rapid(o, module_name, sync_move)
        return o.getvalue()

    def get_timestamp(self) -> str:
        """Get the timestamp of the motion program"""
        return self._timestamp

class MotionProgramExecClient:
    """
    Client to execute motion programs an ABB IRC5 controller using Robot Web Services (RWS)

    :ivar abb_client: Instance of ``abb_robot_client.rws.RWS`` used execute commands

    :param base_url: Base URL of the robot. For Robot Studio instances, this should be http://127.0.0.1:80,
                     the default value. For a real robot, 127.0.0.1 should be replaced with the IP address
                     of the robot controller. The WAN port ethernet must be used, not the maintenance port.
    :param username: The HTTP username for the robot. Defaults to 'Default User'
    :param password: The HTTP password for the robot. Defaults to 'robotics'    
    """
    def __init__(self, base_url='http://127.0.0.1:80', username='Default User', password='robotics', abb_client = None):
        if abb_client is None:
            self.abb_client: RWS = RWS(base_url, username, password)
        else:
            self.abb_client: RWS = abb_client

    def execute_motion_program(self, motion_program: MotionProgram, task: str="T_ROB1", 
        wait : bool = True, seqno: int = None) -> Union[MotionProgramResultLog,int]:
        """
        Execute a motion program. If ``wait`` is True, executes the following steps:

        #. Convert `motion_program` to bytes
        #. Upload binary motion program to controller as a temporary file on the controller ramdisk
        #. Check the last sequence number of the controller event log before starting the program
        #. Start executing the motion program by starting the RAPID task on the controller
        #. Wait for the RAPID task to complete
        #. Read the event log to check for errors, and find the filename of the saved joint log
        #. Read the joint log file from the controller
        #. Parse and return the joint log

        If `wait` is set to False, the process stops after retrieving the most recent sequence number and returns
        that value.

        :param motion_program: The motion program to execute
        :param task: The RAPID Task to use to execute the program. Defaults to ``T_ROB1``
        :param wait: If True, wait for the program to complete. Else, return once the program has been started.
        :param seqno: Optional motion program seqno override
        """
        filename, b = _get_motion_program_file(self.abb_client.get_ramdisk_path(), motion_program, task, seqno = seqno)
        def _upload():            
            self.abb_client.upload_file(filename, b)

        prev_seqnum = self._download_and_start_motion_program([task], _upload)
        if not wait:
            return prev_seqnum
        self.wait_motion_program_complete()
        return self.read_motion_program_result_log(prev_seqnum)

    def preempt_motion_program(self, motion_program: MotionProgram, task: str="T_ROB1", preempt_number: int = 1, 
        preempt_cmdnum : int = -1, seqno: int = None):
        """
        Preempt a running motion program. Preempting works by downloading a replacement motion program file
        to the controller, and then switching to the new file at a specified command number. Multiple preemptions
        can occur on the same motion program, as long as each preempting has an increasing crossover command number.

        Preempting can only occur after the `currently queued` command. The ABB controller will read ahead up to motion
        three motion commands. This means that there may be some delay before the preemption can occur.

        :param motion_program: The new motion program. The ``first-cmd_num`` parameter of the motion program must be 
                               specified to be one greater than the ``preempt_cmdnum``.
        :param task: The task to preempt
        :param preempt_number: The number of the preemption. The first preemption should set this to 1
        :param preempt_cmdnum: The command number to switch to the new motion program. Must be greater than the currently
                         queued command number.
        :param seqno: Optional override of the motion program seqno
        """

        filename, b = _get_motion_program_file(self.abb_client.get_ramdisk_path(), motion_program, task, preempt_number, seqno = seqno)
        self.abb_client.upload_file(filename, b)
        self.abb_client.set_analog_io("motion_program_preempt_cmd_num", preempt_cmdnum)
        self.abb_client.set_analog_io("motion_program_preempt", preempt_number)

    def get_current_cmdnum(self) -> int:
        """Get the currently executing ``cmdnum``"""
        return self.abb_client.get_analog_io("motion_program_current_cmd_num")

    def get_queued_cmdnum(self) -> int:
        """
        Get the currently queued ``cmdnum``. Motion commands are queued by the controller before executing, so the
        controller may queue up to three commands before executing them.
        """
        return self.abb_client.get_analog_io("motion_program_queued_cmd_num")

    def get_current_preempt_number(self) -> int:
        """Get the current preempt_number"""
        return self.abb_client.get_analog_io("motion_program_preempt_current") 
        

    def execute_multimove_motion_program(self, motion_programs: List[MotionProgram], tasks: List[str]=None, 
        wait : bool = True, seqno: int = None):
        """
        Execute a motion program on a MultiMove system with multiple robots. Same as 
        :meth:`MotionProgramExecClient.execute_motion_program()` but takes lists of motion programs and tasks.

        Motion programs should use ``SyncMoveOn()`` command when using MultiMove.

        :param motion_programs: A list of motion programs. Number of motion programs must correspond to number of robots.
        :param tasks: A list of RAPID Tasks. Defaults to T_ROBn, where n is the default task number for the robots.
        :param wait: If True, wait for the program to complete. Else, return once the program has been started.
        :param seqno: Optional motion program seqno override        
        """

        if tasks is None:
            tasks = [f"T_ROB{i+1}" for i in range(len(motion_programs))]        

        assert len(motion_programs) == len(tasks), \
            "Motion program list and task list must have some length"

        assert len(tasks) > 1, "Multimove program must have at least two tasks"

        b = []
        filenames = []
        ramdisk = self.abb_client.get_ramdisk_path()

        for mp, task in zip(motion_programs, tasks):
            filename1, b1 = _get_motion_program_file(ramdisk, mp, task, seqno = seqno)
            filenames.append(filename1)
            b.append(b1)

        assert len(b) > 0, "Motion program must not be empty"
        assert len(filenames) == len(b)
        def _upload():
            for i in range(len(filenames)):
                self.abb_client.upload_file(filenames[i], b[i])

        prev_seqnum = self._download_and_start_motion_program(tasks, _upload)
        if not wait:
            return prev_seqnum
        self.wait_motion_program_complete()
        return self.read_motion_program_result_log(prev_seqnum)

    def preempt_multimove_motion_program(self, motion_programs: List[MotionProgram], tasks: List[str]=None, 
        preempt_number: int = 1, preempt_cmdnum : int = -1, seqno: int = None):
        """
        Preempt a motion program on a MultiMove system with multiple robots. Same as 
        :meth:`MotionProgramExecClient.preempt_motion_program()` but takes lists of motion programs and tasks.
        
        :param motion_program: List of new motion programs. The ``first-cmd_num`` parameter of the motion program must be 
                               specified to be one greater than the ``preempt_cmdnum``.
        :param task: The tasks to preempt
        :param preempt_number: The number of the preemption. The first preemption should set this to 1
        :param preempt_cmdnum: The command number to switch to the new motion program. Must be greater than the currently
                         queued command number.
        :param seqno: Optional override of the motion program seqno
        """
        if tasks is None:
            tasks = [f"T_ROB{i+1}" for i in range(len(motion_programs))]        

        assert len(motion_programs) == len(tasks), \
            "Motion program list and task list must have some length"

        assert len(tasks) > 1, "Multimove program must have at least two tasks"

        b = []
        filenames = []
        ramdisk = self.abb_client.get_ramdisk_path()

        for mp, task in zip(motion_programs, tasks):
            filename1, b1 = _get_motion_program_file(ramdisk, mp, task, preempt_number, seqno = seqno)
            filenames.append(filename1)
            b.append(b1)

        for filename, b in zip(filenames, b):        
            self.abb_client.upload_file(filename, b)
        self.abb_client.set_analog_io("motion_program_preempt_cmd_num", preempt_cmdnum)
        self.abb_client.set_analog_io("motion_program_preempt", preempt_number)
    
    def _download_and_start_motion_program(self, tasks, upload_fn: Callable[[],None]):
        
        exec_state = self.abb_client.get_execution_state()
        assert exec_state.ctrlexecstate == "stopped"
        #assert exec_state.cycle == "once"
        ctrl_state = self.abb_client.get_controller_state()
        assert ctrl_state == "motoron"

        log_before = self.abb_client.read_event_log()
        prev_seqnum = log_before[0].seqnum

        self.abb_client.resetpp()
        upload_fn()

        self.abb_client.start(cycle='once',tasks=tasks)

        return prev_seqnum

    def is_motion_program_running(self) -> bool:
        """Returns True if motion program is running"""
        exec_state = self.abb_client.get_execution_state()
        return exec_state.ctrlexecstate == "running"

    def wait_motion_program_complete(self):
        """Wait for motion program to complete"""
        
        while True:
            exec_state = self.abb_client.get_execution_state()
            if exec_state.ctrlexecstate != "running":
                break
            time.sleep(0.05)

    def read_motion_program_result_log(self, prev_seqnum: int) -> MotionProgramResultLog:
        """
        Read a motion program result log after motion program completes. This function is called by
        :meth:`MotionProgramExecClient.execute_motion_program()` if ``wait`` is True. If ``wait`` is False,
        it can be called directly.

        :param prev_seqnum: The previous seqnum, returned by ``execute_motion_program()`` if ``wait`` is False.
        :return: The result log
        """

        log_after_raw = self.abb_client.read_event_log()
        log_after = []
        for l in log_after_raw:
            if l.seqnum > prev_seqnum:
                log_after.append(l)
            elif prev_seqnum > 61440 and l.seqnum < 4096:
                # Handle uint16 wraparound
                log_after.append(l)
            else:
                break
        
        failed = False
        for l in log_after:
            if l.msgtype >= 2:
                if len(l.args) > 0 and l.args[0].lower() == "motion program failed":
                    assert False, l.args[1] + " " + l.args[2] + " " + l.args[3] + " " + l.args[4]
            if l.msgtype >= 3:
                failed = True

        if failed:
            assert False, "Motion Program Failed, see robot error log for details"

        found_log_open = False
        found_log_close = False
        log_filename = ""

        for l in reversed(log_after):
            if l.code == 80003:
                if l.args[0].lower() == "motion program log file closed":
                    if found_log_open:
                        assert not found_log_close, "Found more than one log closed message"
                        found_log_close = True
                
                if l.args[0].lower() == "motion program log file opened":
                    assert not found_log_open, "Found more than one log opened message"
                    found_log_open = True
                    log_filename_m = re.search(r"(log\-[\d\-]+\.bin)",l.args[1])
                    assert log_filename_m, "Invalid log opened message"
                    log_filename = log_filename_m.group(1)

        assert found_log_open and found_log_close and len(log_filename) > 0, "Could not find log file messages in robot event log"

        ramdisk = self.abb_client.get_ramdisk_path()
        log_contents = self.abb_client.read_file(f"{ramdisk}/{log_filename}")
        try:
            self.abb_client.delete_file(f"{ramdisk}/{log_filename}")
        except:
            pass
        return _unpack_motion_program_result_log(log_contents)

    def stop_motion_program(self):
        """Stop a motion program. Motion programs will normally stop when complete, so this is not normally necessary"""
        self.abb_client.stop()

    def stop_egm(self):
        """Stop a long running EGM command. This will cause the program to complete normally"""
        self.abb_client.set_digital_io("motion_program_stop_egm", 1)

    def enable_motion_logging(self):
        """Enable motion logging"""
        self.abb_client.set_digital_io("motion_program_log_motion", 1)

    def disable_motion_logging(self):
        """Disable motion logging"""
        self.abb_client.set_digital_io("motion_program_log_motion", 0)

    def get_motion_logging_enabled(self) -> bool:
        """Return if motion logging is enabled"""
        return self.abb_client.get_digital_io("motion_program_log_motion") > 0
