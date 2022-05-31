import sys
import numpy as np
import argparse
import threading
import abb_motion_program_exec_client as abb_client
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import RobotRaconteurCompanion as RRC
from RobotRaconteurCompanion.Util.InfoFileLoader import InfoFileLoader
from RobotRaconteurCompanion.Util.AttributesUtil import AttributesUtil

import traceback
import time

def rr_pose_to_abb(rr_pose):
    a = RRN.NamedArrayToArray(rr_pose)
    return abb_client.pose(a[0][4:7]*1000.0,a[0][0:4])

def rr_tool_to_abb(rr_tool_info):
    tcp = rr_pose_to_abb(rr_tool_info.tcp)
    mass = rr_tool_info.inertia[0]["m"]
    com = RRN.NamedArrayToArray(rr_tool_info.inertia[0]["com"])
    #TODO: figure out aom term in loaddata
    aom = np.array([1,0,0,0],dtype=np.float64)
    ix = rr_tool_info.inertia[0]["ixx"]
    iy = rr_tool_info.inertia[0]["iyy"]
    iz = rr_tool_info.inertia[0]["izz"]
    ld = abb_client.loaddata(mass, com, aom, ix, iy, iz)
    return abb_client.tooldata(True,tcp,ld)

def rr_zone_to_abb(rr_fine_point,rr_blend_radius):
    r = rr_blend_radius * 1000.0
    #TODO use "extended" fields for external axes
    return abb_client.zonedata(rr_fine_point,r,r,r,r,r,r)

def rr_speed_to_abb(rr_velocity):
    #TODO use "extended" for angular velocity and external axes?
    return abb_client.speeddata(rr_velocity*1000.0, rr_velocity*10000.0, 1000, 1000)

def rr_joints_to_abb(rr_joints, rr_joint_units):
    #TODO: joint units
    #TODO: use "extended" for external axes
    return abb_client.jointtarget(np.rad2deg(rr_joints), [6e5]*6)

def rr_robot_pose_to_abb(rr_robot_pose):
    #TODO: joint units
    #TODO: use "extended" for external axes
    p = rr_pose_to_abb(rr_robot_pose.tcp_pose)
    cd = abb_client.confdata(
        np.floor(rr_robot_pose.joint_position_seed[0]/(np.pi/2)),
        np.floor(rr_robot_pose.joint_position_seed[3]/(np.pi/2)),
        np.floor(rr_robot_pose.joint_position_seed[5]/(np.pi/2)),
        0
    )
    return abb_client.robtarget(p.trans,p.rot,cd, [6e5]*6)

def rr_motion_program_to_abb(rr_mp):
    tool_cmd = rr_mp.motion_program_commands[0]
    assert tool_cmd.datatype.endswith(".SetTool"), "First command for ABB must be SetTool"
    abb_tool = rr_tool_to_abb(tool_cmd.data.tool_info)

    mp = abb_client.MotionProgram(tool=abb_tool)
    for cmd in rr_mp.motion_program_commands[1:]:
        if cmd.datatype.endswith("MoveAbsJ"):
            zd = rr_zone_to_abb(cmd.data.fine_point,cmd.data.blend_radius)
            sd = rr_speed_to_abb(cmd.data.tcp_velocity)
            jt = rr_joints_to_abb(cmd.data.joint_position, cmd.data.joint_units)
            mp.MoveAbsJ(jt, sd, zd)
        elif cmd.datatype.endswith("MoveJ"):
            zd = rr_zone_to_abb(cmd.data.fine_point,cmd.data.blend_radius)
            sd = rr_speed_to_abb(cmd.data.tcp_velocity)
            rt = rr_robot_pose_to_abb(cmd.data.tcp_pose)
            mp.MoveJ(rt, sd, zd)
        elif cmd.datatype.endswith("MoveL"):
            zd = rr_zone_to_abb(cmd.data.fine_point,cmd.data.blend_radius)
            sd = rr_speed_to_abb(cmd.data.tcp_velocity)
            rt = rr_robot_pose_to_abb(cmd.data.tcp_pose)
            mp.MoveL(rt, sd, zd)
        elif cmd.datatype.endswith("MoveC"):
            zd = rr_zone_to_abb(cmd.data.fine_point,cmd.data.blend_radius)
            sd = rr_speed_to_abb(cmd.data.tcp_velocity)
            rt = rr_robot_pose_to_abb(cmd.data.tcp_pose)
            rt2 = rr_robot_pose_to_abb(cmd.data.tcp_via_pose)
            mp.MoveC(rt2, rt,  sd, zd)
        elif cmd.datatype.endswith("WaitTime"):
            mp.WaitTime(cmd.data.time)
        else:
            assert False, f"Invalid motion program command type \"{cmd.datatype}\""
    
    return mp


class MotionExecImpl:
    def __init__(self, mp_robot_info, base_url, username, password):

        self.mp_robot_info = mp_robot_info
        self._abb_client = abb_client.MotionProgramExecClient(base_url, username, password)

        self.device_info = mp_robot_info.robot_info.device_info
        self.robot_info = mp_robot_info.robot_info
        self.motion_program_robot_info = mp_robot_info

    def execute_motion_program(self, program):

        abb_program = rr_motion_program_to_abb(program)

        gen = ExecuteMotionProgramGen(self._abb_client, abb_program)

        return gen



class ExecuteMotionProgramGen:

    def __init__(self, abb_client, motion_program):
        self._abb_client = abb_client
        self._motion_program = motion_program
        self._action_status_code = RRN.GetConstants("com.robotraconteur.action")["ActionStatusCode"]
        self._status = self._action_status_code["queued"]
        self._thread = None
        self._wait_evt = threading.Event()
        self._thread_exp = None
        self._mp_status = RRN.GetStructureType("experimental.robotics.motion_program.MotionProgramStatus")

    def Next(self):
        if self._thread_exp is not None:
            raise self._thread_exp
        ret = self._mp_status()
        ret.current_command = -1
        if self._status == self._action_status_code["queued"]:
            self._thread = threading.Thread(target=self._run)
            self._status = self._action_status_code["running"]
            self._thread.start()
            ret.action_status = self._status
            return ret
        self._wait_evt.wait(timeout=1)
        if self._thread.is_alive():
            ret = self._action_status_code["running"]
        else:
            if self._thread_exp:
                raise self._thread_exp
            raise RR.StopIterationException()

    def Close(self):
        pass

    def Abort(self):
        if self._status == self._action_status_code["queued"] or self._status == self._action_status_code["running"]:
            self._abb_client.stop()

    def _run(self):
        try:
            print("Start Motion Program!")
            self._abb_client.execute_motion_program(self._motion_program)
            print("Motion Program Complete!")
        except BaseException as e:
            self._thread_exp = e
            traceback.print_exc()
        self._wait_evt.set()



def main():

    parser = argparse.ArgumentParser(description="ABB Robot motion program driver service for Robot Raconteur")
    parser.add_argument("--mp-robot-info-file", type=argparse.FileType('r'),default=None,required=True,help="Motion program robot info file (required)")
    parser.add_argument("--mp-robot-base-url", type=str, default='http://127.0.0.1:80', help="robot controller ws base url (default http://127.0.0.1:80)")
    parser.add_argument("--mp-robot-username",type=str,default='Default User',help="robot controller username (default 'Default User')")
    parser.add_argument("--mp-robot-password",type=str,default='robotics',help="robot controller password (default 'robotics')")
    parser.add_argument("--wait-signal",action='store_const',const=True,default=False, help="wait for SIGTERM orSIGINT (Linux only)")

    args, _ = parser.parse_known_args()

    RRC.RegisterStdRobDefServiceTypes(RRN)
    RRN.RegisterServiceTypeFromFile('experimental.robotics.motion_program')

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

        if args.wait_signal:  
            #Wait for shutdown signal if running in service mode          
            print("Press Ctrl-C to quit...")
            import signal
            signal.sigwait([signal.SIGTERM,signal.SIGINT])
        else:
            #Wait for the user to shutdown the service
                input("Server started, press enter to quit...")

if __name__ == "__main__":
    main()

