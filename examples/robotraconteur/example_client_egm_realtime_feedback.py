# This example demonstrates running a motion program and receiving real-time feedback using EGM
# and abb-robot-client. Both the abb-motion-program-robotraconteur and abb-robot-client-robotraconteur
# services must be started. The abb-robot-client-robotraconteur service provides low-level RWS interface
# and EGM interface to the robot. The abb-motion-program-robotraconteur service generates and
# commands motion programs.
#
# See the abb-robot-client GitHub repository for more information: https://github.com/rpiRobotics/abb_robot_client
#
# Run the two services in separate terminals:
#
# abb-motion-program-exec-robotraconteur --mp-robot-base-url=http://127.0.0.1:80 --mp-robot-info-file=abb_motion_program_exec/config/abb_1200_5_90_motion_program_robot_default_config.yml
#
# abb-robot-client-robotraconteur --robot-url=http://127.0.0.1:80 --robot-info-file=abb_robot_client/config/abb_1200_5_90_rws_default_config.yml
#
# Replace the IP address with the IP address of the robot controller.
# The EGM must be configured to send UDP data to the IP address running the robot client on the port 6510.

from RobotRaconteur.Client import *
import numpy as np
import time

c = RRN.ConnectService("rr+tcp://localhost:59843?service=mp_robot")
c_abb_client = RRN.ConnectService('rr+tcp://localhost:59926?service=robot')

# Connect the robot_state and egm_state wires
robot_state_wire = c_abb_client.robot_state.Connect()
egm_state_wire = c_abb_client.egm_state.Connect()

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
j2 = np.deg2rad(np.array([90,-91,60,-93,94,-95],dtype=np.float64))
j3 = np.deg2rad(np.array([-80,81,-82,83,-84,85],dtype=np.float64))


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


r1 = robot_pose([0.1649235*1e3, 0.1169957*1e3, 0.9502961*1e3], [ 0.6776466, -0.09003431, 0.6362069, 0.3576725 ], (0,0,0,0))
r2 = robot_pose([ 0.6243948*1e3, -0.479558*1e3 ,  0.7073749*1e3], [ 0.6065634, -0.2193409,  0.6427138, -0.4133877], (-1,-1,0,1))

r3 = robot_pose([417.9236, 276.9956, 885.2959], [ 0.8909725 , -0.1745558 ,  0.08864544,  0.4096832 ], ( 0.,  1., -2.,  0.))
r4 = robot_pose([417.9235 , -11.00438, 759.2958 ], [0.7161292 , 0.1868255 , 0.01720813, 0.6722789 ], ( 0.,  2., -2.,  0.))
r5 = robot_pose([ 417.9235, -173.0044,  876.2958], [0.6757616, 0.3854275, 0.2376617, 0.5816431], (-1.,  1., -1.,  0.))

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

# Add an "EGM start streaming" command to setup_cmds
egm_streaming_cmd = RRN.NewStructure("experimental.abb_robot.motion_program.EGMStreamConfigCommand",c)
setup_cmds.append(RR.VarValue(egm_streaming_cmd, "experimental.abb_robot.motion_program.EGMStreamConfigCommand"))

mp_cmds.append(moveabsj(j1,0.5,0.2,False))
mp_cmds.append(moveabsj(j2,1,0.1,True))
mp_cmds.append(moveabsj(j3,2,0.3,False))
mp_cmds.append(moveabsj(j1,0.5,0.2,True))

mp_cmds.append(movel(r1,0.5,0.02,False))
mp_cmds.append(movel(r2,0.5,0.2,True))

mp_cmds.append(movej(r1,0.5,0.02,False))
mp_cmds.append(movej(r3,0.5,0.2,True))

# mp_cmds.append(movec(r4,r5,0.5,0.2,True))

mp = motionprogram_type()

mp.motion_program_commands = mp_cmds
mp.motion_setup_commands = setup_cmds

# If multimove is available, optionally use a different robot other than first robot.
# See multimove example for demonstration of synchronizing multiple robots
# mp.extended = {
#     "groups": RR.VarValue("T_ROB2", "string")
# }

mp_gen = c.execute_motion_program(mp, False)
res = None


# Use AsyncNext to communicate with executor generator. A thread could also be used, but this
# solution is usually more efficient.
def next_cb(res1, err):
    global done
    if err is not None:
        if isinstance(err, RR.StopIterationException):
            # Normal end of generator
            done = True
            return
        else:
            # Error in generator
            done = True
            # In a real program, handle the error. For this example, just print
            print(err)
            return
    print(res1)
    # Call AsyncNext again to continue
    mp_gen.AsyncNext(None, next_cb)

done = False

# Call AsyncNext to start the generator
mp_gen.AsyncNext(None, next_cb)

while not done:
    print(robot_state_wire.InValue)
    print(egm_state_wire.InValue)
    time.sleep(0.05)

print("Done!")