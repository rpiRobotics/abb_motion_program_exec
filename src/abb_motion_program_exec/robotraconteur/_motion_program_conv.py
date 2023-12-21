from contextlib import suppress
from RobotRaconteur.RobotRaconteurPythonUtil import NamedArrayToArray
import numpy as np
from .. import abb_motion_program_exec_client as abb_exec
import RobotRaconteur as RR
import general_robotics_toolbox as rox
import re

def rr_pose_to_abb(rr_pose):
    a = NamedArrayToArray(rr_pose)
    return abb_exec.pose(a[0][4:7]*1000.0,a[0][0:4])

def rr_inertia_to_loaddata_abb(rr_inertia, rr_inertia_pose = None):
    ii = NamedArrayToArray(rr_inertia)[0]
    # ii[0] = m
    # ii[1:4] = com
    # ii[4] = Ixx
    # ii[5] = Ixy
    # ii[6] = Ixz
    # ii[7] = Iyy
    # ii[8] = Iyz
    # ii[9] = Izz
    mass = ii[0]
    com = ii[1:4]
    I = np.array([[ii[4],ii[5],ii[6]],[ii[5],ii[7],ii[8]],[ii[6],ii[8],ii[9]]])
    w_eig,v_eig = np.linalg.eig(I)
    R = np.column_stack([v_eig[:,0], v_eig[:,1], np.cross(v_eig[:,0], v_eig[:,1])])
    aom = rox.R2q(R)
    ix, iy, iz = w_eig

    if rr_inertia_pose is not None:
        com = np.add(com, NamedArrayToArray(rr_inertia_pose[0]["position"]))
        aom = rox.quatproduct(NamedArrayToArray(rr_inertia_pose[0]["orientation"])) @ aom
    
    return abb_exec.loaddata(mass, com, aom, ix, iy, iz)

def rr_tool_to_abb(rr_tool_info):
    tcp = rr_pose_to_abb(rr_tool_info.tcp)
    ld = rr_inertia_to_loaddata_abb(rr_tool_info.inertia)
    return abb_exec.tooldata(True,tcp,ld)

def rr_payload_to_abb(rr_payload_info, rr_payload_pose):
    return rr_inertia_to_loaddata_abb(rr_payload_info.inertia, rr_payload_pose)

def rr_zone_to_abb(rr_fine_point,rr_blend_radius):
    r = rr_blend_radius * 1000.0
    #TODO use "extended" fields for external axes
    return abb_exec.zonedata(rr_fine_point,r,r,r,r,r,r)

def rr_speed_to_abb(rr_velocity):
    #TODO use "extended" for angular velocity and external axes?
    return abb_exec.speeddata(rr_velocity*1000.0, rr_velocity*10000.0, 1000, 1000)

def rr_joints_to_abb(rr_joints, rr_joint_units):
    #TODO: joint units
    #TODO: use "extended" for external axes
    return abb_exec.jointtarget(np.rad2deg(rr_joints), [6e5]*6)

def rr_robot_pose_to_abb(rr_robot_pose, cfx_robot, confdata_extra = None):
    #TODO: joint units
    #TODO: use "extended" for external axes
    p = rr_pose_to_abb(rr_robot_pose.tcp_pose)
    if not confdata_extra:
        wrist_vs_axis1 = rox.fwdkin(cfx_robot, [0,rr_robot_pose.joint_position_seed[1],
            rr_robot_pose.joint_position_seed[2],0,0,0]).p[0] < 0
        wrist_vs_lower_arm = rox.fwdkin(cfx_robot, [0,0,rr_robot_pose.joint_position_seed[2],0,0,0]).p[0] < 0
        axis_5_sign = rr_robot_pose.joint_position_seed[4] < 0
        cfx = np.packbits([axis_5_sign, wrist_vs_lower_arm, wrist_vs_axis1], bitorder='little').item()
        cd = abb_exec.confdata(
            np.floor(rr_robot_pose.joint_position_seed[0]/(np.pi/2)),
            np.floor(rr_robot_pose.joint_position_seed[3]/(np.pi/2)),
            np.floor(rr_robot_pose.joint_position_seed[5]/(np.pi/2)),
            cfx
        )
    else:
        cd = abb_exec.confdata(confdata_extra.data[0], confdata_extra.data[1], confdata_extra.data[2], confdata_extra.data[3])
    return abb_exec.robtarget(p.trans,p.rot,cd, [6e5]*6)

cmd_get_arg_sentinel = object()
cmd_arg_no_default = object()

def cmd_get_arg(cmd, arg_name, default_value = cmd_arg_no_default):
    if isinstance(cmd, RR.VarValue):
        cmd = cmd.data

    val = getattr(cmd, arg_name, cmd_get_arg_sentinel)
    if val is cmd_get_arg_sentinel:
        freeform_args = getattr(cmd, "command_args", cmd_get_arg_sentinel)
        assert freeform_args is not cmd_get_arg_sentinel, f"Invalid command type, missing argument {arg_name}"

        val = freeform_args.get(arg_name, cmd_get_arg_sentinel)
        if val is cmd_get_arg_sentinel and default_value is not cmd_arg_no_default:
            return default_value
        assert val is not cmd_get_arg_sentinel, f"Invalid command type, missing argument {arg_name}"

    if isinstance(val, RR.VarValue):
        val = val.data

    return val

def cmd_get_extended(cmd, extended_name, default_value = None):
    if isinstance(cmd, RR.VarValue):
        cmd = cmd.data
    if not cmd.extended:
        return default_value
    val = cmd.extended.get(extended_name, default_value)
    return val

class MoveAbsJCommandConv:
    rr_types = ["experimental.robotics.motion_program.MoveAbsJCommand"]
    freeform_names = ["MoveAbsJ", "MoveAbsJCommand", "experimental.robotics.motion_program.MoveAbsJCommand"]

    def apply_rr_command(self, cmd, mp, **kwargs):
        zd = rr_zone_to_abb(cmd_get_arg(cmd,"fine_point"),cmd_get_arg(cmd,"blend_radius"))
        sd = rr_speed_to_abb(cmd_get_arg(cmd, "tcp_velocity"))
        jt = rr_joints_to_abb(cmd_get_arg(cmd,"joint_position"), cmd_get_arg(cmd,"joint_units", []))
        mp.MoveAbsJ(jt, sd, zd)

class MoveJCommandConv:
    rr_types = ["experimental.robotics.motion_program.MoveJCommand"]
    freeform_names = ["MoveJ","MoveJCommand","experimental.robotics.motion_program.MoveJCommand"]

    def apply_rr_command(self, cmd, mp, cfx_robot, **kwargs):
        zd = rr_zone_to_abb(cmd_get_arg(cmd,"fine_point"),cmd_get_arg(cmd,"blend_radius"))
        sd = rr_speed_to_abb(cmd_get_arg(cmd,"tcp_velocity"))
        rt = rr_robot_pose_to_abb(cmd_get_arg(cmd,"tcp_pose"), cfx_robot, cmd_get_extended(cmd, "confdata"))
        mp.MoveJ(rt, sd, zd)

class MoveLCommandConv:
    rr_types = ["experimental.robotics.motion_program.MoveLCommand"]
    freeform_names = ["MoveL","MoveLCommand","experimental.robotics.motion_program.MoveLCommand"]

    def apply_rr_command(self, cmd, mp, cfx_robot, **kwargs):
        zd = rr_zone_to_abb(cmd_get_arg(cmd,"fine_point"),cmd_get_arg(cmd,"blend_radius"))
        sd = rr_speed_to_abb(cmd_get_arg(cmd,"tcp_velocity"))
        rt = rr_robot_pose_to_abb(cmd_get_arg(cmd,"tcp_pose"), cfx_robot, cmd_get_extended(cmd, "confdata"))
        mp.MoveL(rt, sd, zd)

class MoveCCommandConv:
    rr_types = ["experimental.robotics.motion_program.MoveCCommand"]
    freeform_names = ["MoveC","MoveCCommand","experimental.robotics.motion_program.MoveCCommand"]

    def apply_rr_command(self, cmd, mp, cfx_robot, **kwargs):
        zd = rr_zone_to_abb(cmd_get_arg(cmd,"fine_point"),cmd_get_arg(cmd,"blend_radius"))
        sd = rr_speed_to_abb(cmd_get_arg(cmd,"tcp_velocity"))
        rt = rr_robot_pose_to_abb(cmd_get_arg(cmd,"tcp_pose"), cfx_robot, cmd_get_extended(cmd, "confdata"))
        rt2 = rr_robot_pose_to_abb(cmd_get_arg(cmd,"tcp_via_pose"), cfx_robot, cmd_get_extended("confdata_via"))
        mp.MoveC(rt2, rt,  sd, zd)

class WaitTimeCommandConv:
    rr_types = ["experimental.robotics.motion_program.WaitTimeCommand"]
    freeform_names = ["WaitTime", "WaitTimeCommand", "experimental.robotics.motion_program.WaitTimeCommand"]

    def apply_rr_command(self, cmd, mp, **kwargs):
        mp.WaitTime(cmd_get_arg(cmd, "time"))

class SetToolCommandConv:
    rr_types = ["experimental.robotics.motion_program.SetToolCommand"]
    freeform_names = ["SetTool", "SetToolCommand", "experimental.robotics.motion_program.SetToolCommand"]

    def apply_rr_command(self, cmd, mp, **kwargs):
        assert False, "Unsupported in motion command section"

    def add_setup_args(self, cmd, setup_args):
        abb_tool = rr_tool_to_abb(cmd_get_arg(cmd,"tool_info"))
        setup_args["tool"] = abb_tool

class SetPayloadCommandConv:
    rr_types = ["experimental.robotics.motion_program.SetPayloadCommand"]
    freeform_names = ["SetPayload", "SetPayloadCommand", "experimental.robotics.motion_program.SetPayloadCommand"]

    def apply_rr_command(self, cmd, mp, **kwargs):
        assert False, "Unsupported in motion command section"

    def add_setup_args(self, cmd, setup_args):
        abb_payload = rr_payload_to_abb(cmd_get_arg(cmd,"payload_info"),cmd_get_arg(cmd,"payload_pose"))
        setup_args["gripload"] = abb_payload

# ABB Commands

class CirPathModeCommandConv:
    rr_types = ["experimental.abb_robot.motion_program.CirPathModeCommand"]
    freeform_names = ["CirPathMode", "CirPathModeCommand", "experimental.abb_robot.motion_program.CirPathModeCommand"]

    def apply_rr_command(self, cmd, mp, **kwargs):
        rr_switch = cmd_get_arg(cmd, "switch")
        if rr_switch == 1 or rr_switch == "PathFrame":
            switch = abb_exec.CirPathModeSwitch.PathFrame
        elif rr_switch == 2 or rr_switch == "ObjectFrame":
            switch = abb_exec.CirPathModeSwitch.ObjectFrame
        elif rr_switch == 3 or rr_switch == "CirPointOri":
            switch = abb_exec.CirPathModeSwitch.CirPointOri
        elif rr_switch == 4 or rr_switch == "Wrist45":
            switch = abb_exec.CirPathModeSwitch.Wrist45
        elif rr_switch == 5 or rr_switch == "Wrist46":
            switch = abb_exec.CirPathModeSwitch.Wrist46
        elif rr_switch == 6 or rr_switch == "Wrist56":
            switch = abb_exec.CirPathModeSwitch.Wrist56
        else:
            assert False, f"Invalid CirPathModeSwitch value: {rr_switch}"
        mp.CirPathMode(switch)

class SyncMoveOnCommandConv:
    rr_types = ["experimental.abb_robot.motion_program.SyncMoveOnCommand"]
    freeform_names = ["SyncMoveOn", "SyncMoveOnCommand", "experimental.abb_robot.motion_program.SyncMoveOnCommand"]

    def apply_rr_command(self, cmd, mp, **kwargs):
        mp.SyncMoveOn()

class SyncMoveOffCommandConv:
    rr_types = ["experimental.abb_robot.motion_program.SyncMoveOffCommand"]
    freeform_names = ["SyncMoveOff", "SyncMoveOffCommand", "experimental.abb_robot.motion_program.SyncMoveOffCommand"]

    def apply_rr_command(self, cmd, mp, **kwargs):
        mp.SyncMoveOff()

# ABB EGM Commands

class EGMStreamConfigCommandConv:
    rr_types = ["experimental.abb_robot.motion_program.EGMStreamConfigCommand"]
    freeform_names = ["EGMStreamConfig", "EGMStreamConfigCommand", "experimental.abb_robot.motion_program.EGMStreamConfigCommand"]

    def apply_rr_command(self, cmd, mp, **kwargs):
        assert False, "Unsupported in motion command section"

    def add_setup_args(self, cmd, setup_args):
        setup_args["egm_config"] = abb_exec.EGMStreamConfig()

def rr_egmframetype_to_abb(rr_egmframetype):
    if rr_egmframetype == 0 or rr_egmframetype == "EGM_FRAME_BASE":
        return abb_exec.egmframetype.EGM_FRAME_BASE
    elif rr_egmframetype == 1 or rr_egmframetype == "EGM_FRAME_TOOL":
        return abb_exec.egmframetype.EGM_FRAME_TOOL
    elif rr_egmframetype == 2 or rr_egmframetype == "EGM_FRAME_WOBJ":
        return abb_exec.egmframetype.EGM_FRAME_WOBJ
    elif rr_egmframetype == 3 or rr_egmframetype == "EGM_FRAME_WORLD":
        return abb_exec.egmframetype.EGM_FRAME_WORLD
    elif rr_egmframetype == 4 or rr_egmframetype == "EGM_FRAME_JOINT":
        return abb_exec.egmframetype.EGM_FRAME_JOINT
    else:
        assert False, f"Invalid egmframetype value: {rr_egmframetype}"

def rr_egmminmax_to_abb(rr_egmminmax):
    if not isinstance(rr_egmminmax, RR.VarValue):
        min_ = rr_egmminmax.min
        max_ = rr_egmminmax.max
        return abb_exec.egm_minmax(min_, max_)
    else:
        min_ = rr_egmminmax.data["min"].data
        max_ = rr_egmminmax.data["max"].data
        return abb_exec.egm_minmax(min_, max_)

class EGMJointTargetConfigCommandConv:
    rr_types = ["experimental.abb_robot.motion_program.EGMJointTargetConfigCommand"]
    freeform_names = ["EGMJointTargetConfig", "EGMJointTargetConfigCommand", "experimental.abb_robot.motion_program.EGMJointTargetConfigCommand"]

    def apply_rr_command(self, cmd, mp, **kwargs):
        assert False, "Unsupported in motion command section"

    def add_setup_args(self, cmd, setup_args):
        J1 = rr_egmminmax_to_abb(cmd_get_arg(cmd, "J1"))
        J2 = rr_egmminmax_to_abb(cmd_get_arg(cmd, "J2"))
        J3 = rr_egmminmax_to_abb(cmd_get_arg(cmd, "J3"))
        J4 = rr_egmminmax_to_abb(cmd_get_arg(cmd, "J4"))
        J5 = rr_egmminmax_to_abb(cmd_get_arg(cmd, "J5"))
        J6 = rr_egmminmax_to_abb(cmd_get_arg(cmd, "J6"))
        max_position_deviation = cmd_get_arg(cmd, "max_position_deviation")
        max_speed_deviation = cmd_get_arg(cmd, "max_speed_deviation")
        setup_args["egm_config"] = abb_exec.EGMJointTargetConfig(J1, J2, J3, J4, J5, J6, max_position_deviation,
            max_speed_deviation)

class EGMPoseTargetConfigCommandConv:
    rr_types = ["experimental.abb_robot.motion_program.EGMPoseTargetConfigCommand"]
    freeform_names = ["EGMPoseTargetConfig", "EGMPoseTargetConfigCommand", "experimental.abb_robot.motion_program.EGMPoseTargetConfigCommand"]

    def apply_rr_command(self, cmd, mp, **kwargs):
        assert False, "Unsupported in motion command section"

    def add_setup_args(self, cmd, setup_args):
        corr_frame = rr_pose_to_abb(cmd_get_arg(cmd, "corr_frame"))
        corr_fr_type = rr_egmframetype_to_abb(cmd_get_arg(cmd, "corr_fr_type"))
        sensor_frame = rr_pose_to_abb(cmd_get_arg(cmd, "sensor_frame"))
        sensor_fr_type = rr_egmframetype_to_abb(cmd_get_arg(cmd, "sensor_fr_type"))
        x = rr_egmminmax_to_abb(cmd_get_arg(cmd, "x"))
        y = rr_egmminmax_to_abb(cmd_get_arg(cmd, "y"))
        z = rr_egmminmax_to_abb(cmd_get_arg(cmd, "z"))
        rx = rr_egmminmax_to_abb(cmd_get_arg(cmd, "rx"))
        ry = rr_egmminmax_to_abb(cmd_get_arg(cmd, "ry"))
        rz = rr_egmminmax_to_abb(cmd_get_arg(cmd, "rz"))
        max_position_deviation = cmd_get_arg(cmd, "max_position_deviation")
        max_speed_deviation = cmd_get_arg(cmd, "max_speed_deviation")
        setup_args["egm_config"] = abb_exec.EGMPoseTargetConfig(corr_frame, corr_fr_type, sensor_frame, sensor_fr_type,
            x, y, z, rx, ry, rz, max_position_deviation, max_speed_deviation)

class EGMPathCorrectionConfigCommandConv:
    rr_types = ["experimental.abb_robot.motion_program.EGMPathCorrectionConfigCommand"]
    freeform_names = ["EGMPathCorrectionConfig", "EGMPathCorrectionConfigCommand", "experimental.abb_robot.motion_program.EGMPathCorrectionConfigCommand"]

    def apply_rr_command(self, cmd, mp, **kwargs):
        assert False, "Unsupported in motion command section"

    def add_setup_args(self, cmd, setup_args):
        sensor_frame = rr_pose_to_abb(cmd_get_arg(cmd, "sensor_frame"))
        setup_args["egm_config"] = abb_exec.EGMPathCorrectionConfig(sensor_frame)

class EGMRunJointCommandConv:
    rr_types = ["experimental.abb_robot.motion_program.EGMRunJointCommand"]
    freeform_names = ["EGMRunJoint", "EGMRunJointCommand", "experimental.abb_robot.motion_program.EGMRunJointCommand"]

    def apply_rr_command(self, cmd, mp, **kwargs):
        cond_time = cmd_get_arg(cmd, "cond_time")
        ramp_in_time = cmd_get_arg(cmd, "ramp_in_time")
        ramp_out_time = cmd_get_arg(cmd, "ramp_out_time")
        mp.EGMRunJoint(cond_time, ramp_in_time, ramp_out_time)

class EGMRunPoseCommandConv:
    rr_types = ["experimental.abb_robot.motion_program.EGMRunPoseCommand"]
    freeform_names = ["EGMRunPose", "EGMRunPoseCommand", "experimental.abb_robot.motion_program.EGMRunPoseCommand"]

    def apply_rr_command(self, cmd, mp, cfx_robot, **kwargs):
        cond_time = cmd_get_arg(cmd, "cond_time")
        ramp_in_time = cmd_get_arg(cmd, "ramp_in_time")
        ramp_out_time = cmd_get_arg(cmd, "ramp_out_time")
        offset = rr_pose_to_abb(cmd_get_arg(cmd, "offset"))
        mp.EGMRunPose(cond_time, ramp_in_time, ramp_out_time, offset)

class EGMMoveLCommandConv:
    rr_types = ["experimental.abb_robot.motion_program.EGMMoveLCommand"]
    freeform_names = ["EGMMoveL","EGMMoveLCommand","experimental.abb_robot.motion_program.EGMMoveLCommand"]

    def apply_rr_command(self, cmd, mp, cfx_robot, **kwargs):
        zd = rr_zone_to_abb(cmd_get_arg(cmd,"fine_point"),cmd_get_arg(cmd,"blend_radius"))
        sd = rr_speed_to_abb(cmd_get_arg(cmd,"tcp_velocity"))
        rt = rr_robot_pose_to_abb(cmd_get_arg(cmd,"tcp_pose"),cfx_robot, cmd_get_extended(cmd, "confdata"))
        mp.EGMMoveL(rt, sd, zd)

class EGMMoveCCommandConv:
    rr_types = ["experimental.abb_robot.motion_program.EGMMoveCCommand"]
    freeform_names = ["EGMMoveC","EGMMoveCCommand","experimental.abb_robot.motion_program.EGMMoveCCommand"]

    def apply_rr_command(self, cmd, mp, cfx_robot,**kwargs):
        zd = rr_zone_to_abb(cmd_get_arg(cmd,"fine_point"),cmd_get_arg(cmd,"blend_radius"))
        sd = rr_speed_to_abb(cmd_get_arg(cmd,"tcp_velocity"))
        rt = rr_robot_pose_to_abb(cmd_get_arg(cmd,"tcp_pose"),cfx_robot,cmd_get_extended(cmd, "confdata"))
        rt2 = rr_robot_pose_to_abb(cmd_get_arg(cmd,"tcp_via_pose"),cfx_robot,cmd_get_extended(cmd,"confdata_via"))
        mp.EGMMoveC(rt2, rt,  sd, zd)

#ABB Setup Commands

def rr_workobject_to_abb(rr_wobj_info):
    robhold = rr_wobj_info.robhold
    ufprog = rr_wobj_info.ufprog
    ufmec = rr_wobj_info.ufmec
    uframe = rr_pose_to_abb(rr_wobj_info.uframe)
    oframe = rr_pose_to_abb(rr_wobj_info.oframe)
    return abb_exec.wobjdata(robhold, ufprog, ufmec, uframe, oframe)

class SetWorkObjectCommandConv:
    rr_types = ["experimental.abb_robot.motion_program.SetWorkObjectCommand"]
    freeform_names = ["SetWorkObject", "SetWorkObjectCommand", "experimental.abb_robot.motion_program.SetWorkObjectCommand"]

    def apply_rr_command(self, cmd, mp, **kwargs):
        assert False, "Unsupported in motion command section"

    def add_setup_args(self, cmd, setup_args):
        abb_wobj = rr_workobject_to_abb(cmd_get_arg(cmd,"workobject_info"))
        setup_args["wobj"] = abb_wobj

_command_convs = dict()
_freeform_command_convs = dict()

conv_types = [
    MoveAbsJCommandConv,
    MoveJCommandConv,
    MoveLCommandConv,
    MoveCCommandConv,
    WaitTimeCommandConv,
    SetToolCommandConv,
    SetPayloadCommandConv,
    CirPathModeCommandConv,
    SyncMoveOnCommandConv,
    SyncMoveOffCommandConv,
    EGMStreamConfigCommandConv,
    EGMJointTargetConfigCommandConv,    
    EGMPoseTargetConfigCommandConv,
    EGMPathCorrectionConfigCommandConv,
    EGMRunJointCommandConv,
    EGMRunPoseCommandConv,
    EGMMoveLCommandConv,
    EGMMoveCCommandConv,
    SetWorkObjectCommandConv
]

def _init_convs():
    for c in conv_types:
        c_inst = c()
        for x in c_inst.rr_types:
            _command_convs[x] = c_inst
        for y in c_inst.freeform_names:
            _freeform_command_convs[y] = c_inst

_init_convs()

class OptionalCommandException(Exception):
    def __init__(self, message):
        super().__init__(message=message)

def get_command_conv(cmd):
    if cmd.datatype == "experimental.robotics.motion_program.FreeformCommand":
        conv = _freeform_command_convs.get(cmd.data.command_name, None)
        if conv is None:
            if cmd.data.optional:
                raise OptionalCommandException(f"Optional command {cmd.data.command_name}")
            else:
                assert False, f"Unknown command {cmd.data.command_name}"
        return conv
    else:
        conv = _command_convs.get(cmd.datatype, None)
        assert conv is not None, f"Unknown command {cmd.datatype}"
        return conv

def apply_rr_motion_command_to_mp(cmd, mp, **kwargs):
    conv = get_command_conv(cmd)
    conv.apply_rr_command(cmd, mp, **kwargs)


def rr_motion_program_to_abb(rr_mp, robot):

    # Robot structure for computing confdata.cfx from joint seed
    P_cfx_w = np.copy(robot.P)
    P_cfx_w[:,0] = 0.0
    P_cfx_w[:,5:7] = 0.0    
    cfx_robot = rox.Robot(robot.H, P_cfx_w, robot.joint_type)

    setup_args = dict()
    if rr_mp.motion_setup_commands is not None:
        for setup_cmd in rr_mp.motion_setup_commands:
            #with suppress(OptionalCommandException):
                conv = get_command_conv(setup_cmd)
                conv.add_setup_args(setup_cmd, setup_args)
      
    if rr_mp.extended is not None:
        first_cmd_num_rr = rr_mp.extended.get("first_command_number", None)
        if first_cmd_num_rr is not None:
            setup_args["first_cmd_num"] = int(first_cmd_num_rr.data)
    mp = abb_exec.MotionProgram(**setup_args)
    for cmd in rr_mp.motion_program_commands:
        #with suppress(OptionalCommandException):
            apply_rr_motion_command_to_mp(cmd, mp, robot=robot, cfx_robot=cfx_robot)
        
    return mp

def is_rr_motion_program_multimove(rr_mp):
    if rr_mp.extended is None:
        return False
    groups =rr_mp.extended.get("groups", None)
    if groups is None:
        groups = rr_mp.extended.get("tasks", None)
    if groups is None:
        return False
    
    if groups.datatype == "string":
        return False
    assert groups.datatype == "varvalue{list}", "Invalid groups type"
    return len(groups.data) > 1

def get_rr_motion_program_task(rr_mp, default_task = "T_ROB1"):
    if rr_mp.extended is None:
        return default_task
    groups =rr_mp.extended.get("groups", None)
    if groups is None:
        groups = rr_mp.extended.get("tasks", None)
    if groups is None:
        return default_task
    
    if groups.datatype == "string":
        return groups.data
    else:
        assert groups.datatype == "varvalue{list}"
        assert len(groups) == 1, "Multiple tasks not expected"
        if groups.data[0].datatype == "string":
            return groups.data[0].data
        elif groups.data[0].datatype == "int32[]" or groups.data[0].datatype == "uint32[]":
            return f"T_ROB{groups.data[0].data[0]+1}"
        else:
            assert False, "Invalid task type"
    
def rr_motion_program_to_abb2(program, robots):
    if (is_rr_motion_program_multimove(program)):
        return rr_multimove_motion_program_to_abb(program, robots)
    task = get_rr_motion_program_task(program)

    robot_ind_match = re.match(r"T_ROB(\d+)", task)
    assert robot_ind_match is not None, "Invalid task name"
    robot_ind = int(robot_ind_match.group(1))-1

    rox_robot = robots[robot_ind]
    mp = rr_motion_program_to_abb(program, rox_robot)
    return mp, False, task

def rr_multimove_motion_program_to_abb(program, robots):
    motion_programs = [program]
    multi_programs = program.extended.get("multi_motion_programs", None)
    assert multi_programs is not None, "Invalid multimove motion program"
    assert multi_programs.datatype == "varvalue{list}", "Invalid multimove motion program"
    for mp in multi_programs.data:
        assert mp.datatype == "experimental.robotics.motion_program.MotionProgram", "Invalid multimove motion program"
        motion_programs.append(mp.data)

    groups = program.extended.get("groups", None)
    assert groups is not None, "Invalid multimove motion program"
    if groups.datatype == "int32[]" or groups.datatype == "uint32[]":
        tasks = [f"T_ROB{x+1}" for x in groups.data]
    elif groups.datatype == "varvalue{list}":
        tasks = [x.data for x in groups.data]
    else:
        assert False, "Invalid multimove motion program"
        
    programs = []
    for i in range(len(motion_programs)):
        robot_ind_match = re.match(r"T_ROB(\d+)", tasks[i])
        assert robot_ind_match is not None, "Invalid task name"
        robot_ind = int(robot_ind_match.group(1))-1
        rox_robot = robots[robot_ind]
        programs.append(rr_motion_program_to_abb(motion_programs[i], rox_robot))

    return programs, True, tasks
