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

toolinfo = toolinfo_type()
toolinfo.tcp = RRN.ArrayToNamedArray([1,0,0,0,0,0,0.001],transform_dt)
toolinfo.inertia = RRN.ArrayToNamedArray([0.1,0,0,0.01,.001,0,0,.001,0,.001],spatialinertia_dt)

settool = settool_type()
settool.tool_info = toolinfo

j1 = np.deg2rad(np.array([10,20,30,40,50,60],dtype=np.float64))
j2 = np.deg2rad(np.array([-10,15,35,10,95,-95],dtype=np.float64))
j3 = np.deg2rad(np.array([15,-5,25,83,-84,85],dtype=np.float64))


def robot_pose(p,q,conf):
    ret = robot_pose_type()
    ret.tcp_pose[0]["orientation"]["w"] = q[0]
    ret.tcp_pose[0]["orientation"]["x"] = q[1]
    ret.tcp_pose[0]["orientation"]["y"] = q[2]
    ret.tcp_pose[0]["orientation"]["z"] = q[3]
    ret.tcp_pose[0]["position"]["x"] = p[0]*1e-3
    ret.tcp_pose[0]["position"]["y"] = p[1]*1e-3
    ret.tcp_pose[0]["position"]["z"] = p[2]*1e-3

    ret.joint_position_seed=np.zeros((6,))
    ret.joint_position_seed[0] = conf[0]*np.pi/2
    ret.joint_position_seed[3] = conf[1]*np.pi/2
    ret.joint_position_seed[5] = conf[2]*np.pi/2

    return ret


r1 = robot_pose([ 350., -100., 600. ], [ 0.0868241, -0.0868241, 0.9924039, 0.0075961 ], (-1,0,-1,0))
r2 = robot_pose([ 370., 120., 620. ], [ 0.0868241, 0.0868241, 0.9924039, -0.0075961 ], (0,-1,0,0))

r3 = robot_pose([400., -200., 500.], [ 0.7071068, 0., 0.7071068, 0. ], ( -1.,  -3., 2.,  0. ))
r4 = robot_pose([400., 0., 580. ], [0.7071068, 0., 0.7071068, 0. ], ( 0.,  -3, -2.,  0.))
r5 = robot_pose([ 400., 200., 500.], [0.7071068, 0., 0.7071068, 0.], (0.,  -2., 1.,  0.))

setup_cmds = []
mp_cmds = []

def moveabsj(j,velocity,blend_radius,fine_point):
    cmd = moveabsj_type()
    cmd.joint_position = j
    cmd.tcp_velocity = velocity
    cmd.blend_radius = blend_radius
    cmd.fine_point = fine_point
    return RR.VarValue(cmd,"experimental.robotics.motion_program.MoveAbsJCommand")

def movel(robot_pose,velocity,blend_radius,fine_point):
    cmd = movel_type()
    cmd.tcp_pose = robot_pose
    cmd.tcp_velocity = velocity
    cmd.blend_radius = blend_radius
    cmd.fine_point = fine_point
    return RR.VarValue(cmd,"experimental.robotics.motion_program.MoveLCommand")

def movej(robot_pose,velocity,blend_radius,fine_point):
    cmd = movej_type()
    cmd.tcp_pose = robot_pose
    cmd.tcp_velocity = velocity
    cmd.blend_radius = blend_radius
    cmd.fine_point = fine_point
    return RR.VarValue(cmd,"experimental.robotics.motion_program.MoveJCommand")

def movec(robot_pose,robot_via_pose,velocity,blend_radius,fine_point):
    cmd = movec_type()
    cmd.tcp_pose = robot_pose
    cmd.tcp_via_pose = robot_via_pose
    cmd.tcp_velocity = velocity
    cmd.blend_radius = blend_radius
    cmd.fine_point = fine_point
    return RR.VarValue(cmd,"experimental.robotics.motion_program.MoveCCommand")

setup_cmds.append(RR.VarValue(settool,"experimental.robotics.motion_program.SetToolCommand"))
mp_cmds.append(moveabsj(j1,0.5,0.2,False))
mp_cmds.append(moveabsj(j2,1,0.1,True))
mp_cmds.append(moveabsj(j3,2,0.3,False))
mp_cmds.append(moveabsj(j1,0.5,0.2,True))

mp_cmds.append(movel(r1,0.5,0.02,False))
mp_cmds.append(movel(r2,0.5,0.2,True))

mp_cmds.append(movej(r1,0.5,0.02,False))
mp_cmds.append(movej(r3,0.5,0.2,True))

mp_cmds.append(movec(r5,r4,0.5,0.2,True))

mp = motionprogram_type()

mp.motion_program_commands = mp_cmds
mp.motion_setup_commands = setup_cmds

# If multimove is available, optionally use a different robot other than first robot.
# See multimove example for demonstration of synchronizing multiple robots
# mp.extended = {
#     "groups": RR.VarValue("T_ROB2", "string")
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