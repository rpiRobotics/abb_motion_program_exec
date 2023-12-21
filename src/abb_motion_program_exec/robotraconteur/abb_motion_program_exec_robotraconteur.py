import sys
import numpy as np
import argparse
import threading
from .. import abb_motion_program_exec_client as abb_client
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import RobotRaconteurCompanion as RRC
from RobotRaconteurCompanion.Util.InfoFileLoader import InfoFileLoader
from RobotRaconteurCompanion.Util.AttributesUtil import AttributesUtil
from RobotRaconteurCompanion.Util.TaskGenerator import SyncTaskGenerator
from RobotRaconteurCompanion.Util.RobDef import register_service_types_from_resources
from RobotRaconteurCompanion.Util.RobotUtil import RobotUtil
from ._motion_program_conv import rr_motion_program_to_abb2

import traceback
import time
import io
import random
import drekar_launch_process

class MotionExecImpl:
    def __init__(self, mp_robot_info, base_url, username, password):

        self.mp_robot_info = mp_robot_info
        self._abb_client = abb_client.MotionProgramExecClient(base_url, username, password)

        self.device_info = mp_robot_info.robot_info.device_info
        self.robot_info = mp_robot_info.robot_info
        self.motion_program_robot_info = mp_robot_info
        self._recordings = {}

        self.param_changed = RR.EventHook()

        self._robot_util = RobotUtil(RRN)
        try:
            self._rox_robots = []
            for chain_i in range(len(self.robot_info.chains)):
                self._rox_robots.append(self._robot_util.robot_info_to_rox_robot(self.robot_info,chain_i))
        except:
            traceback.print_exc()
            raise ValueError("invalid robot_info, could not populate GeneralRoboticsToolbox.Robot")


    def execute_motion_program(self, program, queue):

        assert queue is False, "Motion program queue not supported"

        abb_program, is_multimove, tasks = rr_motion_program_to_abb2(program, self._rox_robots)

        gen = ExecuteMotionProgramGen(self, self._abb_client, abb_program, is_multimove, tasks)

        return gen

    def execute_motion_program_record(self, program, queue):

        assert queue is False, "Motion program queue not supported"

        abb_program, is_multimove, tasks = rr_motion_program_to_abb2(program, self._rox_robots)

        gen = ExecuteMotionProgramGen(self, self._abb_client, abb_program, is_multimove, tasks, save_recording = True)

        return gen

    def read_recording(self, recording_handle):
        robot_recording_np = self._recordings.pop(recording_handle)
        return RobotRecordingGen(robot_recording_np)

    def clear_recordings(self):
        self._recordings.clear()

    def get_param(self, param_name):
        raise RR.InvalidArgumentException("Unknown parameter")
    
    def set_param(self, param_name, value):
        raise RR.InvalidArgumentException("Unknown parameter")
    
    def enable_motion_program_mode(self):
        pass

    def disable_motion_program_mode(self):
        pass


class ExecuteMotionProgramGen(SyncTaskGenerator):
    def __init__(self, parent, abb_client, motion_program, is_multimove, tasks, save_recording = False):
        super().__init__(RRN, RRN.GetStructureType("experimental.robotics.motion_program.MotionProgramStatus"), 1, -1)
        self._parent = parent
        self._abb_client = abb_client
        self._motion_program = motion_program
        self._recording_handle = 0
        self._save_recording = save_recording
        self._is_multimove = is_multimove
        self._tasks = tasks

    def RunTask(self):        
        print("Start Motion Program!")
        if not self._is_multimove:
            robot_recording_data = self._abb_client.execute_motion_program(self._motion_program, task=self._tasks)
        else:
            robot_recording_data = self._abb_client.execute_multimove_motion_program(self._motion_program, tasks=self._tasks)
        if self._save_recording:
            recording_handle = random.randint(0,0xFFFFFFF)
            self._parent._recordings[recording_handle] = robot_recording_data
            self._recording_handle = recording_handle
        print("Motion Program Complete!")
        res = self._status_type()
        res.action_status = self._action_const["ActionStatusCode"]["complete"]
        res.recording_handle = self._recording_handle
        return res       


class RobotRecordingGen:
    def __init__(self, robot_recording_np, ):
        self.robot_rec_np = robot_recording_np
        self.closed = False
        self.aborted = False
        self.lock = threading.Lock()
        self._mp_recording_part = RRN.GetStructureType("experimental.robotics.motion_program.MotionProgramRecordingPart")

    def Next(self):
        with self.lock:
            if self.aborted:
                raise RR.OperationAbortedException("Recording aborted")

            if self.closed:
                raise RR.StopIterationException()

            ret = self._mp_recording_part()

            # All current paths expect to be within 10 MB limit
            ret.time = self.robot_rec_np.data[:,0].flatten().astype(np.float64)
            ret.command_number = self.robot_rec_np.data[:,1].flatten().astype(np.int32)
            # TODO: prismatic joints
            ret.joints = np.deg2rad(self.robot_rec_np.data[:,2:].astype(np.float64))
            ret.column_headers = self.robot_rec_np.column_headers

            self.closed = True
            return ret

    def Abort(self):
        self.aborted = True

    def Close(self):
        self.closed = True

def main():

    parser = argparse.ArgumentParser(description="ABB Robot motion program driver service for Robot Raconteur")
    parser.add_argument("--mp-robot-info-file", type=argparse.FileType('r'),default=None,required=True,help="Motion program robot info file (required)")
    parser.add_argument("--mp-robot-base-url", type=str, default='http://127.0.0.1:80', help="robot controller ws base url (default http://127.0.0.1:80)")
    parser.add_argument("--mp-robot-username",type=str,default='Default User',help="robot controller username (default 'Default User')")
    parser.add_argument("--mp-robot-password",type=str,default='robotics',help="robot controller password (default 'robotics')")

    args, _ = parser.parse_known_args()

    RRC.RegisterStdRobDefServiceTypes(RRN)
    register_service_types_from_resources(RRN, __package__, ["experimental.robotics.motion_program", "experimental.abb_robot.motion_program"])

    with args.mp_robot_info_file:
        mp_robot_info_text = args.mp_robot_info_file.read()

    info_loader = InfoFileLoader(RRN)
    mp_robot_info, mp_robot_ident_fd = info_loader.LoadInfoFileFromString(mp_robot_info_text, "experimental.robotics.motion_program.MotionProgramRobotInfo", "mp_robot")

    attributes_util = AttributesUtil(RRN)
    mp_robot_attributes = attributes_util.GetDefaultServiceAttributesFromDeviceInfo(mp_robot_info.robot_info.device_info)

    mp_exec_obj = MotionExecImpl(mp_robot_info,args.mp_robot_base_url,args.mp_robot_username,args.mp_robot_password)

    with RR.ServerNodeSetup("experimental.robotics.motion_program",59843,argv=sys.argv):

        service_ctx = RRN.RegisterService("mp_robot","experimental.robotics.motion_program.MotionProgramRobot",mp_exec_obj)
        service_ctx.SetServiceAttributes(mp_robot_attributes)
        service_ctx.AddExtraImport("experimental.abb_robot.motion_program")

        print("Press ctrl+c to quit")
        drekar_launch_process.wait_exit()

if __name__ == "__main__":
    main()

