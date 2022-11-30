from .abb_motion_program_exec_client import MotionProgram, _get_motion_program_file, \
    _unpack_motion_program_result_log
from typing import Callable, NamedTuple, Any, List
from abb_robot_client.rws_aio import RWS_AIO
import asyncio
import re

class MotionProgramExecClientAIO:
    def __init__(self, base_url='http://127.0.0.1:80', username='Default User', password='robotics', 
        abb_client_aio = None):

        if abb_client_aio is None:
            self.abb_client_aio = RWS_AIO(base_url, username, password)
        else:
            self.abb_client_aio = abb_client_aio

    async def execute_motion_program(self, motion_program: MotionProgram, task="T_ROB1", wait : bool = True, seqno: int = None):
        filename, b = _get_motion_program_file(await self.abb_client_aio.get_ramdisk_path(), 
            motion_program, task, seqno = seqno)
        async def _upload():            
            await self.abb_client_aio.upload_file(filename, b)

        prev_seqnum = await self._download_and_start_motion_program([task], _upload)
        if not wait:
            return prev_seqnum
        await self.wait_motion_program_complete()
        return await self.read_motion_program_result_log(prev_seqnum)

    async def preempt_motion_program(self, motion_program: MotionProgram, task="T_ROB1", preempt_number: int = 1, 
        preempt_cmdnum : int = -1, seqno: int = None):
        filename, b = _get_motion_program_file(await self.abb_client_aio.get_ramdisk_path(), motion_program, task, preempt_number, seqno = seqno)
        await self.abb_client_aio.upload_file(filename, b)
        await self.abb_client_aio.set_analog_io("motion_program_preempt_cmd_num", preempt_cmdnum)
        await self.abb_client_aio.set_analog_io("motion_program_preempt", preempt_number)

    async def get_current_cmdnum(self):
        return await self.abb_client_aio.get_analog_io("motion_program_current_cmd_num")

    async def get_queued_cmdnum(self):
        return await self.abb_client_aio.get_analog_io("motion_program_queued_cmd_num")

    async def get_current_preempt_number(self):
        return await self.abb_client_aio.get_analog_io("motion_program_preempt_current") 
        

    async def execute_multimove_motion_program(self, motion_programs: List[MotionProgram], tasks=None, wait : bool = True,
        seqno: int = None):

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

    async def preempt_multimove_motion_program(self, motion_programs: List[MotionProgram], tasks=None, 
        preempt_number: int = 1, preempt_cmdnum : int = -1, seqno: int = None):
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

    async def is_motion_program_running(self):
        exec_state = await self.abb_client_aio.get_execution_state()
        return exec_state.ctrlexecstate == "running"

    async def wait_motion_program_complete(self):
        
        while True:
            exec_state = await self.abb_client_aio.get_execution_state()
            if exec_state.ctrlexecstate != "running":
                break
            await asyncio.sleep(0.05)

    async def read_motion_program_result_log(self, prev_seqnum):

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
        await self.abb_client_aio.stop()

    async def stop_egm(self):
        await self.abb_client_aio.set_digital_io("motion_program_stop_egm", 1)

    async def enable_motion_logging(self):
        await self.abb_client_aio.set_digital_io("motion_program_log_motion", 1)

    async def disable_motion_logging(self):
        await self.abb_client_aio.set_digital_io("motion_program_log_motion", 0)

    async def get_motion_logging_enabled(self):
        return await self.abb_client_aio.get_digital_io("motion_program_log_motion") > 0