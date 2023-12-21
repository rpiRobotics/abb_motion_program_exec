from RobotRaconteur.Client import *
import numpy as np

c = RRN.ConnectService("rr+tcp://localhost:59843?service=mp_robot")

robot_pose_type = RRN.GetStructureType("experimental.robotics.motion_program.RobotPose",c)
moveabsj_type = RRN.GetStructureType("experimental.robotics.motion_program.MoveAbsJCommand",c)
movej_type = RRN.GetStructureType("experimental.robotics.motion_program.MoveJCommand",c)
movel_type = RRN.GetStructureType("experimental.robotics.motion_program.MoveLCommand",c)
movec_type = RRN.GetStructureType("experimental.robotics.motion_program.MoveCCommand",c)
settool_type = RRN.GetStructureType("experimental.robotics.motion_program.SetToolCommand",c)
motionprogram_type = RRN.GetStructureType("experimental.robotics.motion_program.MotionProgram",c)
toolinfo_type = RRN.GetStructureType("com.robotraconteur.robotics.tool.ToolInfo",c)
transform_dt = RRN.GetNamedArrayDType("com.robotraconteur.geometry.Transform",c)
spatialinertia_dt = RRN.GetNamedArrayDType("com.robotraconteur.geometry.SpatialInertia",c)

sync_move_on_type = RRN.GetStructureType("experimental.abb_robot.motion_program.SyncMoveOnCommand",c)
sync_move_off_type = RRN.GetStructureType("experimental.abb_robot.motion_program.SyncMoveOnCommand",c)

toolinfo = toolinfo_type()
toolinfo.tcp = RRN.ArrayToNamedArray([1,0,0,0,0,0,0.1],transform_dt)
toolinfo.inertia = RRN.ArrayToNamedArray([0.1,0,0,0.01,.001,0,0,.001,0,.001],spatialinertia_dt)

settool = settool_type()
settool.tool_info = toolinfo

toolinfo_2 = toolinfo_type()
toolinfo_2.tcp = RRN.ArrayToNamedArray([1,0,0,0,0,0,0.5],transform_dt)
toolinfo_2.inertia = RRN.ArrayToNamedArray([0.1,0,0,0.01,.001,0,0,.001,0,.001],spatialinertia_dt)

settool_2 = settool_type()
settool_2.tool_info = toolinfo_2

def robot_pose(p,q,conf):
    ret = robot_pose_type()
    ret.tcp_pose[0]["orientation"]["w"] = q[0]
    ret.tcp_pose[0]["orientation"]["x"] = q[1]
    ret.tcp_pose[0]["orientation"]["y"] = q[2]
    ret.tcp_pose[0]["orientation"]["z"] = q[3]
    ret.tcp_pose[0]["position"]["x"] = p[0]*1e-3
    ret.tcp_pose[0]["position"]["y"] = p[1]*1e-3
    ret.tcp_pose[0]["position"]["z"] = p[2]*1e-3

    # ret.joint_position_seed=np.zeros((6,))
    # ret.joint_position_seed[0] = conf[0]*np.pi/2
    # ret.joint_position_seed[3] = conf[1]*np.pi/2
    # ret.joint_position_seed[5] = conf[2]*np.pi/2

    # Using JointSeed is the most portable way to specify robot configuration, but can be unreliable.
    # Use confdata instead for ABB robots in the waypoint extended field
    confdata = RR.VarValue(np.array(conf,dtype=np.float64),"double[]")
    return ret, confdata

j1 = np.deg2rad(np.array([5,-20,30,27,-11,-27],dtype=np.float64))
t1 = robot_pose([575,-200,1280],[0,-.707,0,.707],(0,0,-1,1),)
t2 = robot_pose([575,200,1480],[0,-.707,0,.707],(-1,-1,0,1),)
t3 = robot_pose([575,0,1280],[0,-.707,0,.707],(-1,-1,0,1),)

j1_2 = np.deg2rad(np.array([1,1,40,2,-40,-2],dtype=np.float64))
t1_2 = robot_pose([250,-200,1280],[.707,0,.707,0],(-1,-1,0,1))
t2_2 = robot_pose([250,200,1480],[.707,0,.707,0],(0,0,-1,1))
t3_2 = robot_pose([250,0,1280],[.707,0,.707,0],(0,0,0,1))


setup_cmds = []
mp_cmds = []

setup_cmds_2 = []
mp_cmds_2 = []

def moveabsj(j,velocity,blend_radius,fine_point):
    cmd = moveabsj_type()
    cmd.joint_position = j
    cmd.tcp_velocity = velocity
    cmd.blend_radius = blend_radius
    cmd.fine_point = fine_point
    return RR.VarValue(cmd,"experimental.robotics.motion_program.MoveAbsJCommand")

def movel(robot_pose,velocity,blend_radius,fine_point):
    cmd = movel_type()
    cmd.tcp_pose = robot_pose[0]
    cmd.tcp_velocity = velocity
    cmd.blend_radius = blend_radius
    cmd.fine_point = fine_point
    cmd.extended = {
        "confdata": robot_pose[1]
    }
    return RR.VarValue(cmd,"experimental.robotics.motion_program.MoveLCommand")

def movej(robot_pose,velocity,blend_radius,fine_point):
    cmd = movej_type()
    cmd.tcp_pose = robot_pose[0]
    cmd.tcp_velocity = velocity
    cmd.blend_radius = blend_radius
    cmd.fine_point = fine_point
    cmd.extended = {
        "confdata": robot_pose[1]
    }
    return RR.VarValue(cmd,"experimental.robotics.motion_program.MoveJCommand")

def movec(robot_pose,robot_via_pose,velocity,blend_radius,fine_point):
    cmd = movec_type()
    cmd.tcp_pose = robot_pose[0]
    cmd.tcp_via_pose = robot_via_pose[0]
    cmd.tcp_velocity = velocity
    cmd.blend_radius = blend_radius
    cmd.fine_point = fine_point
    cmd.extended = {
        "confdata": robot_pose[1],
        "confdata_via": robot_via_pose[1]
    }
    return RR.VarValue(cmd,"experimental.robotics.motion_program.MoveCCommand")

def sync_move_on():
    cmd = sync_move_on_type()
    return RR.VarValue(cmd,"experimental.abb_robot.motion_program.SyncMoveOnCommand")

def sync_move_off():
    cmd = sync_move_off_type()
    return RR.VarValue(cmd,"experimental.abb_robot.motion_program.SyncMoveOffCommand")

setup_cmds.append(RR.VarValue(settool,"experimental.robotics.motion_program.SetToolCommand"))
mp_cmds.append(sync_move_on())
mp_cmds.append(moveabsj(j1,0.5,0.2,True))
mp_cmds.append(movej(t1,0.5,0.02,True))
mp_cmds.append(movej(t2,0.5,0.2,True))
mp_cmds.append(movej(t3,0.5,0.2,True))

setup_cmds_2.append(RR.VarValue(settool_2,"experimental.robotics.motion_program.SetToolCommand"))
mp_cmds_2.append(sync_move_on())
mp_cmds_2.append(moveabsj(j1_2,0.5,0.2,True))
mp_cmds_2.append(movej(t1_2,0.5,0.02,True))
mp_cmds_2.append(movej(t2_2,0.5,0.2,True))
mp_cmds_2.append(movej(t3_2,0.5,0.2,True))


# mp_cmds.append(movec(r4,r5,0.5,0.2,True))

mp = motionprogram_type()

mp.motion_program_commands = mp_cmds
mp.motion_setup_commands = setup_cmds

# Build up second motion program
mp_2 = motionprogram_type()
mp_2.motion_program_commands = mp_cmds_2
mp_2.motion_setup_commands = setup_cmds_2

# Set the groups and add the second motion program using "extended" field
mp.extended = {
    "multi_motion_programs": RR.VarValue([mp_2], "experimental.robotics.motion_program.MotionProgram{list}"),
    "groups": RR.VarValue(["T_ROB1", "T_ROB2"], "string{list}")
}

# Optionally use integers for groups. Zero indexed
# mp.extended = {
#     "groups": RR.VarValue([0, 1], "int32{list}")
# }

mp_gen = c.execute_motion_program_record(mp, False)
res = None

status = None
while True:
    res, status1 = mp_gen.TryNext()
    if not res:
        break
    status = status1
    print(status)
    

print(f"recording_handle: {status.recording_handle}")

robot_recording = c.read_recording(status.recording_handle).NextAll()[0]

print(robot_recording.time)
print(robot_recording.command_number)
print(robot_recording.joints)

c.clear_recordings()

print("Done!")