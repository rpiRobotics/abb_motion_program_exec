# Copyright 2022 Wason Technology LLC, Rensselaer Polytechnic Institute
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

from .abb_motion_program_exec_client import MotionProgram, MotionProgramResultLog, _get_motion_program_file, \
    _unpack_motion_program_result_log
from typing import Callable, NamedTuple, Any, List, Union, TYPE_CHECKING
from abb_robot_client.rws_aio import RWS_AIO
import asyncio
import re

class MotionProgramExecClientAIO:
    """
    Client to execute motion programs an ABB IRC5 controller using Robot Web Services (RWS) using AsyncIO

    This class is functionally identical to :class:`abb_motion_program_exec.MotionProgramExecClient` except it uses 
    AsyncIO instead of synchronous blocking operations.

    :ivar abb_client_aio: Instance of ``abb_robot_client.rws.RWS_AIO`` used execute commands

    :param base_url: Base URL of the robot. For Robot Studio instances, this should be http://127.0.0.1:80,
                     the default value. For a real robot, 127.0.0.1 should be replaced with the IP address
                     of the robot controller. The WAN port ethernet must be used, not the maintenance port.
    :param username: The HTTP username for the robot. Defaults to 'Default User'
    :param password: The HTTP password for the robot. Defaults to 'robotics' 
    """
    def __init__(self, base_url='http://127.0.0.1:80', username='Default User', password='robotics', 
        abb_client_aio = None):

        if abb_client_aio is None:
            self.abb_client_aio = RWS_AIO(base_url, username, password)
        else:
            self.abb_client_aio = abb_client_aio

    async def execute_motion_program(self, motion_program: MotionProgram, task: str="T_ROB1", wait : bool = True, 
        seqno: int = None) -> Union[MotionProgramResultLog,int]:
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
        filename, b = _get_motion_program_file(await self.abb_client_aio.get_ramdisk_path(), 
            motion_program, task, seqno = seqno)
        async def _upload():            
            await self.abb_client_aio.upload_file(filename, b)

        prev_seqnum = await self._download_and_start_motion_program([task], _upload)
        if not wait:
            return prev_seqnum
        await self.wait_motion_program_complete()
        return await self.read_motion_program_result_log(prev_seqnum)

    async def preempt_motion_program(self, motion_program: MotionProgram, task: str="T_ROB1", preempt_number: int = 1, 
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
        filename, b = _get_motion_program_file(await self.abb_client_aio.get_ramdisk_path(), motion_program, task, preempt_number, seqno = seqno)
        await self.abb_client_aio.upload_file(filename, b)
        await self.abb_client_aio.set_analog_io("motion_program_preempt_cmd_num", preempt_cmdnum)
        await self.abb_client_aio.set_analog_io("motion_program_preempt", preempt_number)

    async def get_current_cmdnum(self) -> int:
        """Get the currently executing ``cmdnum``"""
        return await self.abb_client_aio.get_analog_io("motion_program_current_cmd_num")

    async def get_queued_cmdnum(self) -> int:
        """
        Get the currently queued ``cmdnum``. Motion commands are queued by the controller before executing, so the
        controller may queue up to three commands before executing them.
        """
        return await self.abb_client_aio.get_analog_io("motion_program_queued_cmd_num")

    async def get_current_preempt_number(self) -> int:
        """Get the current preempt_number"""
        return await self.abb_client_aio.get_analog_io("motion_program_preempt_current") 
        

    async def execute_multimove_motion_program(self, motion_programs: List[MotionProgram], tasks: List[str]=None, 
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
        ramdisk = await self.abb_client_aio.get_ramdisk_path()

        for mp, task in zip(motion_programs, tasks):
            filename1, b1 = _get_motion_program_file(ramdisk, mp, task, seqno = seqno)
            filenames.append(filename1)
            b.append(b1)

        assert len(b) > 0, "Motion program must not be empty"
        assert len(filenames) == len(b)
        async def _upload():
            for i in range(len(filenames)):
                await self.abb_client_aio.upload_file(filenames[i], b[i])

        prev_seqnum = await self._download_and_start_motion_program(tasks, _upload)
        if not wait:
            return prev_seqnum
        await self.wait_motion_program_complete()
        return await self.read_motion_program_result_log(prev_seqnum)

    async def preempt_multimove_motion_program(self, motion_programs: List[MotionProgram], tasks: List[str]=None, 
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
        ramdisk = await self.abb_client_aio.get_ramdisk_path()

        for mp, task in zip(motion_programs, tasks):
            filename1, b1 = _get_motion_program_file(ramdisk, mp, task, preempt_number, seqno = seqno)
            filenames.append(filename1)
            b.append(b1)

        for filename, b in zip(filenames, b):        
            await self.abb_client_aio.upload_file(filename, b)
        await self.abb_client_aio.set_analog_io("motion_program_preempt_cmd_num", preempt_cmdnum)
        await self.abb_client_aio.set_analog_io("motion_program_preempt", preempt_number)
    
    async def _download_and_start_motion_program(self, tasks, upload_fn: Callable[[],None]):
        
        exec_state = await self.abb_client_aio.get_execution_state()
        assert exec_state.ctrlexecstate == "stopped"
        #assert exec_state.cycle == "once"
        ctrl_state = await self.abb_client_aio.get_controller_state()
        assert ctrl_state == "motoron"

        log_before = await self.abb_client_aio.read_event_log()
        prev_seqnum = log_before[0].seqnum

        await self.abb_client_aio.resetpp()
        await upload_fn()

        await self.abb_client_aio.start(cycle='once',tasks=tasks)

        return prev_seqnum

    async def is_motion_program_running(self) -> bool:
        """Returns True if motion program is running"""
        exec_state = await self.abb_client_aio.get_execution_state()
        return exec_state.ctrlexecstate == "running"

    async def wait_motion_program_complete(self):
        """Wait for motion program to complete"""
        
        while True:
            exec_state = await self.abb_client_aio.get_execution_state()
            if exec_state.ctrlexecstate != "running":
                break
            await asyncio.sleep(0.05)

    async def read_motion_program_result_log(self, prev_seqnum: int) -> MotionProgramResultLog:
        """
        Read a motion program result log after motion program completes. This function is called by
        :meth:`MotionProgramExecClient.execute_motion_program()` if ``wait`` is True. If ``wait`` is False,
        it can be called directly.

        :param prev_seqnum: The previous seqnum, returned by ``execute_motion_program()`` if ``wait`` is False.
        :return: The result log
        """

        log_after_raw = await  self.abb_client_aio.read_event_log()
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

        ramdisk = await self.abb_client_aio.get_ramdisk_path()
        log_contents = await self.abb_client_aio.read_file(f"{ramdisk}/{log_filename}")
        try:
            await self.abb_client_aio.delete_file(f"{ramdisk}/{log_filename}")
        except:
            pass
        return _unpack_motion_program_result_log(log_contents)

    async def stop_motion_program(self):
        """Stop a motion program. Motion programs will normally stop when complete, so this is not normally necessary"""
        await self.abb_client_aio.stop()

    async def stop_egm(self):
        """Stop a long running EGM command. This will cause the program to complete normally"""
        await self.abb_client_aio.set_digital_io("motion_program_stop_egm", 1)

    async def enable_motion_logging(self):
        """Enable motion logging"""
        await self.abb_client_aio.set_digital_io("motion_program_log_motion", 1)

    async def disable_motion_logging(self):
        """Disable motion logging"""
        await self.abb_client_aio.set_digital_io("motion_program_log_motion", 0)

    async def get_motion_logging_enabled(self):
        """Return if motion logging is enabled"""
        return await self.abb_client_aio.get_digital_io("motion_program_log_motion") > 0