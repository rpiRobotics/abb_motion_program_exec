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
from typing import Callable, NamedTuple, Any, List
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

MOTION_PROGRAM_FILE_VERSION = 10008

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

def _get_motion_program_file(path: str, motion_program: "MotionProgram", task="T_ROB1", preempt_number=None):
    b = motion_program.get_program_bytes()
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

class MotionProgram:

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

    def __init__(self,first_cmd_num: int=1, tool: tooldata = None, wobj: wobjdata = None, timestamp: str = None, egm_config = None):

        self._commands = []

        if timestamp is None:
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S-%f")[:-2]
        assert re.match(r"^\d{4}\-\d{2}\-\d{2}-\d{2}\-\d{2}\-\d{2}\-\d{4}$", timestamp)

        self._timestamp = timestamp
        
        self.tool = tool
        self.wobj = wobj
        if self.tool is None:
            self.tool = tool0
        if self.wobj is None:
            self.wobj = wobj0

        self._first_cmd_num=first_cmd_num

        self._egm_config = egm_config

    def _append_command(self, cmd):
        self._commands.append(cmd)

    def write_program(self, f: io.IOBase):
        # Version number
        f.write(util.num_to_bin(MOTION_PROGRAM_FILE_VERSION))
        
        f.write(util.tooldata_to_bin(self.tool))
        f.write(util.wobjdata_to_bin(self.wobj))
        f.write(util.str_to_bin(self._timestamp))

        egm_commands.write_egm_config(f,self._egm_config)

        for i in range(len(self._commands)):
            cmd = self._commands[i]
            cmd_num = i + self._first_cmd_num
            f.write(util.num_to_bin(cmd_num))
            f.write(util.num_to_bin(cmd.command_opcode))

            cmd.write_params(f)    

    def get_program_bytes(self):
        f = io.BytesIO()
        self.write_program(f)
        return f.getvalue()

    def write_program_rapid(self, f: io.TextIOBase, module_name="motion_program_exec_gen", sync_move=False):
            
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
        
    def get_program_rapid(self, module_name="motion_program_exec_gen", sync_move=False):
        o = io.StringIO()
        self.write_program_rapid(o, module_name, sync_move)
        return o.getvalue()

    def get_timestamp(self):
        return self._timestamp


class MotionProgramExecClient:
    def __init__(self, base_url='http://127.0.0.1:80', username='Default User', password='robotics', abb_client = None):
        if abb_client is None:
            self.abb_client = RWS(base_url, username, password)
        else:
            self.abb_client = abb_client

    def execute_motion_program(self, motion_program: MotionProgram, task="T_ROB1", wait : bool = True):
        filename, b = _get_motion_program_file(self.abb_client.get_ramdisk_path(), motion_program, task)
        def _upload():            
            self.abb_client.upload_file(filename, b)

        prev_seqnum = self._download_and_start_motion_program([task], _upload)
        if not wait:
            return prev_seqnum
        self.wait_motion_program_complete()
        return self.read_motion_program_result_log(prev_seqnum)

    def preempt_motion_program(self, motion_program: MotionProgram, task="T_ROB1", preempt_number: int = 1, 
        preempt_cmdnum : int = -1):
        filename, b = _get_motion_program_file(self.abb_client.get_ramdisk_path(), motion_program, task, preempt_number)
        self.abb_client.upload_file(filename, b)
        self.abb_client.set_analog_io("motion_program_preempt_cmd_num", preempt_cmdnum)
        self.abb_client.set_analog_io("motion_program_preempt", preempt_number)

    def get_current_cmdnum(self):
        return self.abb_client.get_analog_io("motion_program_current_cmd_num")

    def get_queued_cmdnum(self):
        return self.abb_client.get_analog_io("motion_program_queued_cmd_num")

    def get_current_preempt_number(self):
        return self.abb_client.get_analog_io("motion_program_preempt_current") 
        

    def execute_multimove_motion_program(self, motion_programs: List[MotionProgram], tasks=None, wait : bool = True):

        if tasks is None:
            tasks = [f"T_ROB{i+1}" for i in range(len(motion_programs))]        

        assert len(motion_programs) == len(tasks), \
            "Motion program list and task list must have some length"

        assert len(tasks) > 1, "Multimove program must have at least two tasks"

        b = []
        filenames = []
        ramdisk = self.abb_client.get_ramdisk_path()

        for mp, task in zip(motion_programs, tasks):
            filename1, b1 = _get_motion_program_file(ramdisk, mp, task)
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

    def preempt_multimove_motion_program(self, motion_programs: List[MotionProgram], tasks=None, 
        preempt_number: int = 1, preempt_cmdnum : int = -1):
        if tasks is None:
            tasks = [f"T_ROB{i+1}" for i in range(len(motion_programs))]        

        assert len(motion_programs) == len(tasks), \
            "Motion program list and task list must have some length"

        assert len(tasks) > 1, "Multimove program must have at least two tasks"

        b = []
        filenames = []
        ramdisk = self.abb_client.get_ramdisk_path()

        for mp, task in zip(motion_programs, tasks):
            filename1, b1 = _get_motion_program_file(ramdisk, mp, task, preempt_number)
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

    def is_motion_program_running(self):
        exec_state = self.abb_client.get_execution_state()
        return exec_state.ctrlexecstate == "running"

    def wait_motion_program_complete(self):
        
        while True:
            exec_state = self.abb_client.get_execution_state()
            if exec_state.ctrlexecstate != "running":
                break
            time.sleep(0.05)

    def read_motion_program_result_log(self, prev_seqnum):

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
        self.abb_client.stop()

    def stop_egm(self):
        self.abb_client.set_digital_io("motion_program_stop_egm", 1)

def main():
    j1 = jointtarget([10,20,30,40,50,60],[0]*6)
    j2 = jointtarget([90,-91,60,-93,94,-95],[0]*6)
    j3 = jointtarget([-80,81,-82,83,-84,85],[0]*6)

    mp = MotionProgram()
    mp.MoveAbsJ(j1,v1000,fine)
    mp.MoveAbsJ(j2,v5000,fine)
    mp.MoveAbsJ(j3,v500,fine)
    mp.MoveAbsJ(j2,v5000,z50)
    mp.MoveAbsJ(j3,v500,z200)
    mp.MoveAbsJ(j2,v5000,fine)
    mp.WaitTime(1)

    r1 = robtarget([0.1649235*1e3, 0.1169957*1e3, 0.9502961*1e3], [ 0.6776466, -0.09003431, 0.6362069, 0.3576725 ], confdata(0,0,0,0),[0]*6)
    r2 = robtarget([ 0.6243948*1e3, -0.479558*1e3 ,  0.7073749*1e3], [ 0.6065634, -0.2193409,  0.6427138, -0.4133877], confdata(-1,-1,0,1),[0]*6)

    r3 = robtarget([417.9236, 276.9956, 885.2959], [ 0.8909725 , -0.1745558 ,  0.08864544,  0.4096832 ], confdata( 0.,  1., -2.,  0.),[0]*6)
    r4 = robtarget([417.9235 , -11.00438, 759.2958 ], [0.7161292 , 0.1868255 , 0.01720813, 0.6722789 ], confdata( 0.,  2., -2.,  0.),[0]*6)
    r5 = robtarget([ 417.9235, -173.0044,  876.2958], [0.6757616, 0.3854275, 0.2376617, 0.5816431], confdata(-1.,  1., -1.,  0.),[0]*6)

    mp.MoveJ(r1,v500,fine)
    mp.MoveJ(r2,v400,fine)
    mp.MoveJ(r1,v500,z100)
    mp.MoveJ(r2,v400,z100)
    mp.MoveJ(r1,v500,fine)
    mp.WaitTime(1.5)

    mp.MoveJ(r3,v5000,fine)
    mp.MoveL(r4,v200,fine)
    mp.MoveL(r3,v200,fine)
    mp.MoveL(r4,v1000,z100)
    mp.MoveL(r3,v1000,z100)
    mp.MoveL(r4,v1000,fine)
    mp.WaitTime(2.5)

    mp.MoveJ(r3,v5000,fine)

    mp.CirPathMode(CirPathModeSwitch.CirPointOri)
    mp.MoveC(r4,r5,v200,z10)
    mp.MoveC(r4,r3,v50,fine)

    print(mp.get_program_rapid())

    client = MotionProgramExecClient()
    log_results = client.execute_motion_program(mp)

    # log_results.data is a numpy array
    import matplotlib.pyplot as plt
    import matplotlib.ticker as plt_ticker
    fig, ax1 = plt.subplots()
    lns1 = ax1.plot(log_results.data[:,0], log_results.data[:,2:])
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Joint angle (deg)")
    ax2 = ax1.twinx()
    lns2 = ax2.plot(log_results.data[:,0], log_results.data[:,1], '-k')
    ax2.set_ylabel("Command number")
    ax2.set_yticks(range(-1,int(max(log_results.data[:,1]))+1))
    ax1.legend(lns1 + lns2, log_results.column_headers[2:] + ["cmdnum"])
    ax1.set_title("Joint motion")
    plt.show()

if __name__ == "__main__":
    main()