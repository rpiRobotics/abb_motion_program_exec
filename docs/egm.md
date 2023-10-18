# EGM Support

Externally Guided Motion (EGM) is an optional feature for ABB robots to provide real-time feedback and position
control. A UDP and protobuf based protocol is used to communicate with an external computer, with feedback
data sent to the computer from the computer, and commands sent to robot, typically at an update rate of 250 Hz. 
EGM is a complex feature, and users should refer to the official documentation *Application Manual Externall Guided
Motion Document ID: 3HAC073318-001* before attempting to use EGM.

**Warning: Using EGM bypasses many safety features of the robot, including collision avoidance. Use of EGM can
be extremely hazardous if done carelessly. Software should be tested in simulation in RobotStudio before being run on
a physical robot.**

`abb_motion_program_exec` provides commands to initiate and execute EGM RAPID commands. The `abb_robot_client` `EGM`
class is used to communicate with EGM. When configuring the robot for EGM, the configuration files from the
`<repo_root>/robot/config_params_egm` must be loaded instead of the normal configuration 
`<repo_root>/robot/config_params`.

The EGM is configured in two places under the controller configuration tree. "Communication" -> "Transmission Protocol".
There should be a `UCdevice" listed. Double click on the row, and set the "Remote Address" to the IP address
of the computer running `abb_motion_program_exec`. You may need to disable the firewall of the computer to allow
incoming UDP packets. The computer will automatically send response packets back to the robot without needing
to configure the robot's information.

The second configuration is under "Motion" -> "External Motion Interface Data". This table configures profiles for the
performance of the EGM. Three profiles are loaded during `abb_motion_program_exec` setup: `joint`, `pose`, 
and `path_corr`. These are used for joint target, pose target, and path correction respectively. The joint and pose
targets are configured to be raw setpoint control with minimal filtering. These parameters can be modified depending
on project requirements.

## EGM Operational Modes

EGM can operate in three modes:
* Joint target: Joint targets are sent to the robot
* Pose target: Pose targets are sent to the robot, using a specified tool and work object frame
* Path correction: Special commands `EGMMoveL` and `EGMMoveC` are used to create motion programs. The EGM can then
correct the position of the end effector by providing an offset in path coordinates.

### EGM Joint Target control

The simplest control method is joint control. The computer streams an array of joint array every 4 ms to the 
robot, and the robot will track the requested joint target. The settings loaded during setup will attempt to minimize
the latency and filtering, but due to the limitations of EGM there may be significant latency, and this should
be considered when designing an operation using EGM. Feedback controllers may become unstable due to unexpectedly high
latency. The `joint` EGM config is used.

See `examples/egm/egm_joint_target_example.py` for an example using joint control.

Joint target control is initialized by passing `EGMJointTargetConfig` to the constructor of `MotionProgram`:

```python
mm = abb.egm_minmax(-1e-3,1e-3)
egm_config = abb.EGMJointTargetConfig(
    mm, mm, mm, mm, mm ,mm, 1000, 1000
)
mp = abb.MotionProgram(egm_config = egm_config)
```

This configuration will initialize the EGM for use when the motion program is loaded on the controller. It uses
the ABB Rapid commands `EGMSetupUC` and `EGMActJoint` to prepare for joint target control. See 
`EGMActJoint` in the ABB manual for the meaning of the parameters sent to `EGMJointTargetConfig`.

Next, run the EGM operation using `EGMRunJoint` and non-blocking motion program execution. It is recommended
to move the robot to a known-safe location before starting EGM. The `EGM` class from `abb_robot_client` is used
to receive robot state and send the joint command. Note that joint units are in degrees! The example
sends a sine wave into the joints. Once done, call `client.stop_egm()` to break out of EGM control. The rest of 
this snippet waits for the program to stop and collects the log results.

```python
mp.MoveAbsJ(j1,abb.v5000,abb.fine)
mp.EGMRunJoint(10, 0.05, 0.05)

client = abb.MotionProgramExecClient(base_url="http://127.0.0.1:80")
lognum = client.execute_motion_program(mp, wait=False)

t1 = time.perf_counter()

egm = EGM()
t2 = t1
while (t2 - t1) < 5 :
    t2 = time.perf_counter()
    res, feedback = egm.receive_from_robot(timeout=0.05)
    if res:
        egm.send_to_robot(np.ones((6,))*np.sin(t2-t1)*5)


client.stop_egm()

while client.is_motion_program_running():
    time.sleep(0.05)

log_results = client.read_motion_program_result_log(lognum)
```

### EGM Pose Target control

The pose target control mode streams a desired robot pose every 4 ms. The robot will use the "closest" robot joint
configuration. The user must be careful to avoid singularities. The settings loaded during setup will attempt to minimize
the latency and filtering, but due to the limitations of EGM there may be significant latency, and this should
be considered when designing an operation using EGM. Feedback controllers may become unstable due to unexpectedly high
latency. The `pose` EGM config is used.

See `examples/egm/egm_pose_target_example.py` for an example using joint control.

Pose target control is initialized by passing `EGMPoseTargetConfig` to the constructor of `MotionProgram`:

```python
mm = abb.egm_minmax(-1e-3,1e-3)

corr_frame = abb.pose([0,0,0],[1,0,0,0])
corr_fr_type = abb.egmframetype.EGM_FRAME_WOBJ
sense_frame = abb.pose([0,0,0],[1,0,0,0])
sense_fr_type = abb.egmframetype.EGM_FRAME_WOBJ
egm_offset = abb.pose([0,0,0],[1,0,0,0])

egm_config = abb.EGMPoseTargetConfig(corr_frame, corr_fr_type, sense_frame, sense_fr_type,
    mm, mm, mm, mm, mm ,mm, 1000, 1000
)

mp = abb.MotionProgram(egm_config = egm_config)
```

This configuration will initialize the EGM for use when the motion program is loaded on the controller. It uses
the ABB Rapid commands `EGMSetupUC` and `EGMActPose` to prepare for joint target control. See 
`EGMActPose` in the ABB manual for the meaning of the parameters sent to `EGMPoseTargetConfig`. These parameters
are fairly complicated, and the user should carefully review the ABB manual before using pose control. The
pose uses the tool frame and workobj frame provided as parameters to constructor of `MotionProgram`. By default
the tool is the robot flange, and the work object is the robot base.

Next, run the EGM operation using `EGMRunPose` and non-blocking motion program execution. It is recommended
to move the robot to a known-safe location before starting EGM. The `EGM` class from `abb_robot_client` is used
to receive robot state and send the pose command using `egm.send_to_robot_cart()` The first parameters is the
position in millimeters, and the second parameter is the orientation in quaternians. Both are numpy arrays or python 
lists. The quaternion is in `[w,x,y,z]` format. Once done, call `client.stop_egm()` to break out of EGM control. 
The rest of this snippet waits for the program to stop and collects the log results. The example moves the robot
along a line:

```python
client = abb.MotionProgramExecClient(base_url="http://127.0.0.1:80")
lognum = client.execute_motion_program(mp, wait=False)

t1 = time.perf_counter()

r2 = copy.copy(r1)

egm = EGM()
t2 = t1
while (t2 - t1) < 5 :
    t2 = time.perf_counter()
    res, feedback = egm.receive_from_robot(timeout=0.05)
    if res:
        r2.trans[1]=(t2-t1)*100.
        egm.send_to_robot_cart(r2.trans, r2.rot)


client.stop_egm()

while client.is_motion_program_running():
    time.sleep(0.05)

log_results = client.read_motion_program_result_log(lognum)
```

### EGM Path Correction

EGM path correction allows for a nominal path to be programed using RAPID commands, and then use EGM to "correct"
the position of the tool using an offset streamed to the robot. The commands `EGMMoveL` and `EGMMoveC` provide
similar behavior to the normal `MoveL` and `MoveC` commands, but add the offset provided by EGM. The offset uses
"path coordinates", which relate the direction of movement of the end effector. See `CorrConn` command in 
*Technical reference manual RAPID Instructions, Functions and Data types* for a detailed description of path 
coordinates.

EGM correction operates a relatively slow update period of 48 ms. EGM may stream faster, but the controller receives
updates at a relatively slow rate.

The `pose` EGM config is used. Unlike joint and pose mode, the parameters are left as the ABB default when loaded.

See `examples/egm/egm_path_correction_example.py` for an example using joint control.

Joint target control is initialized by passing `EGMPathCorrectionConfig` to the constructor of `MotionProgram`:

```python
sensor_frame = abb.pose([0,0,0],[1,0,0,0])

egm_config = abb.EGMPathCorrectionConfig(sensor_frame)

mp = abb.MotionProgram(egm_config = egm_config)
```

This configuration will initialize the EGM for use when the motion program is loaded on the controller. It uses
the ABB Rapid commands `EGMSetupUC` and `EGMActPose` to prepare for path correction control. See 
`EGMActMove` in the ABB manual for the meaning of the parameters sent to `EGMPathCorrectionConfig`. The default 48 ms
update period is used.

Next, run an EGM program containing `EGMMoveL` and/or `EGMMoveC` commands. Execute the motion program
in non-blocking mode. The `EGM` class from `abb_robot_client` is used to receive robot state and send the path 
correction command. The function `egm.send_to_robot_path_corr()` is used to send the path correction. The
first parameter is the path correction offset, in millimeters, in "path" coordinates. There is no need to stop
the EGM process, since th program will complete once all commands have executed.

```python
r1 = abb.robtarget([400,0,600],[0.7071068, 0., 0.7071068, 0.],abb.confdata(0,0,0,1),[0]*6)
r2 = abb.robtarget([400,200,600],[0.7071068, 0., 0.7071068, 0.],abb.confdata(0,0,0,1),[0]*6)
r3 = abb.robtarget([400,400,800],[0.7071068, 0., 0.7071068, 0.],abb.confdata(0,0,0,1),[0]*6)

mp.MoveJ(r1,abb.v5000,abb.fine)
# Using a fine point between EGMMove* will cause a controller error?
mp.EGMMoveL(r2,abb.v50,abb.z1)
mp.EGMMoveL(r1,abb.v50,abb.z1)
mp.EGMMoveC(r2,r3,abb.v50,abb.z1)
client = abb.MotionProgramExecClient(base_url="http://127.0.0.1:80")
lognum = client.execute_motion_program(mp, wait=False)
t1 = time.perf_counter()

egm = EGM()
t2 = t1
while (t2 - t1) < 20:
    t2 = time.perf_counter()
    res, feedback = egm.receive_from_robot(timeout=0.1)
    if res:
        # Inject a sine wave correction
        # `pos` is in "ABB Path Coordinates" See "CorrConn" RAPID command documentation
        egm.send_to_robot_path_corr([0,math.sin((t2-t1)*10)*10.,0])

while client.is_motion_program_running():
    time.sleep(0.05)

log_results = client.read_motion_program_result_log(lognum)
```

This example injects a small sine wave offset overlaying the `EGMMoveC` and `EGMMoveL` commands. A real world use
case would use sensor or feedback data to compute the offset value. Note that there is a very high latency for
corrections, and this **must** be taken into account for feedback control, or the system may become unstable.
