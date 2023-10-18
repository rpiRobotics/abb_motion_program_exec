# abb_motion_program_exec

[![](https://img.shields.io/badge/python-3.6+-blue.svg)](https://github.com/rpiRobotics/abb_motion_program_exec)
[![](https://img.shields.io/pypi/v/abb-motion-program-exec)](https://pypi.org/project/abb-motion-program-exec/)

`abb_motion_program_exec` provides a simple way to download and run a sequence of
`MoveAbsJ`, `MoveJ`, `MoveL`, `MoveC`, and `WaitTime` commands on
an ABB IRC5 robot controller. This program is intended to be a proof of
concept for more sophisticated controller interfaces. Multi-move control of two robots is also
supported. Externally Guided Motion (EGM) is also supported for joint target, pose target, and path correction modes.

Documentation can be found at: https://abb-motion-program-exec.readthedocs.io/

## Installation

`abb-motion-program-exec` is avaliable on PyPi.

```
pip install abb-motion-program-exec
```

Begin by installing the software for the robot controller. This software can be
installed manually by copying files to the robot controller and importing configuration files.

* See [robot_setup_manual.md](docs/robot_setup_manual.md) for setup instructions.
* See [robot_multimove_setup_manual.md](docs/robot_multimove_setup_manual.md) for ABB Multi-Move
  setup to control two robots. See later sections of this doc for more information on Multi-Move.

This contains the robot-side code, that reads
and executes the contents of `motion_program.bin`. `motion_program.bin`
contains the sequence of instructions to run, encoded in binary
format for fast interpretation.

**Only one instance of a Robot Studio virtual controller can be run at a time.** Only
instances of Robot Studio can be run at a time running a single virtual controller. This is due to
the controller using TCP port 80 on the local computer to accept REST commands from Python. If
more than one controller is started, TCP port 80 will already be in use and can cause unpredictable
behavior. Restart the computer if connections cannot be made from Python to the controller. Multiple
real robots can be used concurrently since they will each have a unique IP address to bind port 80.

### Python 3.6 Linux Install (Ubuntu Bionic)

Older versions of Python are not supported by the currently available protobuf package. Use the apt version instead.

```
sudo apt install python3-virtualenv python3-protobuf python3-numpy python3-wheel python3-setuptools
python3 -m pip install --user abb-motion-program-exec
```

## Usage

Once the `abb_motion_program_exec.mod` has been loaded on the controller,
the Python module can be used to command motion sequences. The class
`MotionProgram` contains is used to build the sequence of motions. It has
the following commands of interest:

* `MoveAbsJ(to_joint_pos: jointtarget, speed: speeddata, zone: zonedata)` - Move the
  robot to a specified joint waypoint.
* `MoveJ(to_point: robtarget, speed: speeddata, zone: zonedata)` - Move the
  robot to the specified Cartesian target using joint interpolation.
* `MoveL(to_point: robtarget, speed: speeddata, zone: zonedata)` - Move
  the robot to the specified Cartesian target using linear interpolation.
* `MoveC(cir_point: robtarget, to_point: robtarget, speed: speeddata, zone: zonedata)` -
  Move the robot to the specified Cartesian target circularly using an intermediate
  waypoint.
* `WaitTime(t: float)` - Wait a specified time in seconds.

Calling each of these functions adds the command to the sequence.

The constructor for `MotionProgram` optionally takes a `tool` parameter.
This parameter is expected to be type `tooldata` and will be passed
to each of the move commands. Because the tool is expected to be a
`PERS` type by the ABB software, it can't be modified for each
motion command without a significant performance penalty.

```python
my_motion_program = MotionProgram(tool=my_tool)
```

The following types are defined as subclasses of `NamedTuple`:

```python
class speeddata(NamedTuple):
    v_tcp: float
    v_ori: float
    v_leax: float
    v_reax: float

class zonedata(NamedTuple):
    finep: bool
    pzone_tcp: float
    pzone_ori: float
    pzone_eax: float
    zone_ori: float
    zone_leax: float
    zone_reax: float

class jointtarget(NamedTuple):
    robax: np.ndarray # shape=(6,)
    extax: np.ndarray # shape=(6,)

class pose(NamedTuple):
    trans: np.ndarray # [x,y,z]
    rot: np.ndarray # [qw,qx,qy,qz]

class confdata(NamedTuple):
    cf1: float
    cf4: float
    cf6: float
    cfx: float

class robtarget(NamedTuple):
    trans: np.ndarray # [x,y,z]
    rot: np.ndarray # [qw,qx,qy,qz]
    robconf: confdata # 
    extax: np.ndarray # shape=(6,)

class loaddata(NamedTuple):
    mass: float
    cog: np.ndarray # shape=(3,)
    aom: np.ndarray # shape=(4,)
    ix: float
    iy: float
    iz: float

class tooldata(NamedTuple):
    robhold: bool
    tframe: pose
    tload : loaddata

```

See the ABB Robotics manual "Technical reference manual RAPID 
Instructions, Functions and Data types" for more details on these data
types. Note that `pos`, `orient`, `robjoint`, and `extjoint` are
implemented using numpy arrays or lists.

The following standard `speeddata` are available in the module:
`v5`, `v10`, `v20`, `v30`, `v40`, `v50`, `v60`, `v80`, `v100`,
`v200`, `v300`, `v400`, `v500`, `v600`, `v800`, `v1000`, `v1500`,
`v2000`, `v2500`, `v3000`, `v4000`, `v5000`, `v6000`, `v7000`,
`vmax`.

The following standard `zonedata` are available in the module:
`fine`, `z0`, `z1`, `z5`, `z10`, `z15`, `z20`, `z30`, `z40`,
`z50`, `z60`, `z80`, `z100`, `z150`, `z200`.

The following `tooldata` are available in the module: `tool0`

Once the program is complete, it can be executed on the robot using
`MotionProgramExecClient`. The constructor is by default:

```
mp_client = MotionProgramClient(base_url='http://127.0.0.1:80', username='Default User', password='robotics')
```

The `base_url`, `username`, and `password` should be adjusted to the actual robot. The
client using ABB Web Services. `base_url` must be set to the IP address of the
robot, or using `localhost` if using the simulator.

Once the client is constructed, it can be used to execute the program:

```python
log_results = mp_client.execute_motion_program(mp)
```

`log_results` is a tuple containing the results of the motion:

```python
class MotionProgramResultLog(NamedTuple):
    timestamp: str
    column_headers: List[str]
    data: np.array
```

`timestamp` is a string timestamp for the data. `column_headers` is a list of strings containing the labels of the
columns of data. `data` contains a float32 numpy 2D array of the data, with each row being a sample.

For a single robot, the data has the following columns:

* `timestamp` - The time of the row. This is time from the startup of the logger task in seconds.
  Subtract the initial time from all samples to get a 0 start time for the program.
* `cmd_num` - The currently executing command number. Use `get_program_rapid()` to print out
  the program with command numbers annotated.
* `J1` - Joint 1 position in degrees
* `J2` - Joint 2 position in degrees
* `J3` - Joint 3 position in degrees
* `J4` - Joint 4 position in degrees
* `J5` - Joint 5 position in degrees
* `J6` - Joint 6 position in degrees

The field `column_headers` contains a list of the column headers.

## Python module installation

The `abb_motion_program_exec` module is available on PyPI and can be installed using pip:

```
pip install abb_motion_program_exec
```

For development, use of a `virtualenv` is recommended. Use editable install with the virtualenv:

```
pip install -e .
```

## Externally Guided Motion (EGM)

See [egm.md](docs/egm.md) for instructions on using EGM.

## Example

```python
import abb_motion_program_exec as abb

j1 = abb.jointtarget([10,20,30,40,50,60],[0]*6)
j2 = abb.jointtarget([-10,15,35,10,95,-95],[0]*6)
j3 = abb.jointtarget([15,-5,25,83,-84,85],[0]*6)

my_tool = abb.tooldata(True,abb.pose([0,0,0.1],[1,0,0,0]),abb.loaddata(0.001,[0,0,0.001],[1,0,0,0],0,0,0)) 

mp = abb.MotionProgram(tool=my_tool)
mp.MoveAbsJ(j1,abb.v1000,abb.fine)
mp.MoveAbsJ(j2,abb.v5000,abb.fine)
mp.MoveAbsJ(j3,abb.v500,abb.fine)
mp.MoveAbsJ(j2,abb.v5000,abb.z50)
mp.MoveAbsJ(j3,abb.v500,abb.z200)
mp.MoveAbsJ(j2,abb.v5000,abb.fine)
mp.WaitTime(1)

r1 = abb.robtarget([350., -100., 600.], [ 0.0868241, -0.0868241, 0.9924039, 0.0075961 ], abb.confdata(-1,0,-1,0),[0]*6)
r2 = abb.robtarget([370., 120., 620. ], [ 0.0868241, 0.0868241, 0.9924039, -0.0075961], abb.confdata(0,-1,0,0),[0]*6)

r3 = abb.robtarget([400., -200., 500.], [0.7071068, 0., 0.7071068, 0.], abb.confdata( -1.,  -3., 2.,  0.),[0]*6)
r4 = abb.robtarget([400., 0., 580.], [0.7071068, 0., 0.7071068, 0.], abb.confdata(0.,  -3., 2.,  0.), [0]*6)
r5 = abb.robtarget([400., 200., 500.], [0.7071068, 0., 0.7071068, 0.], abb.confdata(0.,  -2., 1.,  0.),[0]*6)

mp.MoveJ(r1,abb.v500,abb.fine)
mp.MoveJ(r2,abb.v400,abb.fine)
mp.MoveJ(r1,abb.v500,abb.z100)
mp.MoveJ(r2,abb.v400,abb.z100)
mp.MoveJ(r1,abb.v500,abb.fine)
mp.WaitTime(1.5)

mp.MoveJ(r3,abb.v5000,abb.fine)
mp.MoveL(r4,abb.v200,abb.fine)
mp.MoveL(r3,abb.v200,abb.fine)
mp.MoveL(r4,abb.v1000,abb.z100)
mp.MoveL(r3,abb.v1000,abb.z100)
mp.MoveL(r4,abb.v1000,abb.fine)
mp.WaitTime(2.5)

mp.MoveJ(r3,abb.v5000,abb.fine)

mp.MoveC(r4,r5,abb.v200,abb.z10)
mp.MoveC(r4,r3,abb.v50,abb.fine)

# Print out RAPID module of motion program for debugging
print(mp.get_program_rapid())

# Execute the motion program on the robot
# Change base_url to the robot IP address
client = abb.MotionProgramExecClient(base_url="http://127.0.0.1:80")
log_results = client.execute_motion_program(mp)

# log_results.data is a numpy array
import matplotlib.pyplot as plt
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

```


## Multi-Move Robot Example

Two robots can be controlled using ABB Multi-Move. See 
[robot_multimove_setup_manual.md](docs/robot_multimove_setup_manual.md) for setup instructions.

They must have exactly the same number of motion commands. The commands
are passed with the `\ID` parameter corresponding to the command number. `SyncMoveOn` is activated
to cause the robots to move in sync. The `execute_multimove_motion_program()` function
of `MotionProgramExecClient` is used to send multi-move programs to the robot.

```python
import abb_motion_program_exec as abb

# Fill motion program for T_ROB1
t1 = abb.robtarget([575,-200,1280],[0,-.707,0,.707],abb.confdata(0,0,-1,1),[0]*6)
t2 = abb.robtarget([575,200,1480],[0,-.707,0,.707],abb.confdata(-1,-1,0,1),[0]*6)
t3 = abb.robtarget([575,0,1280],[0,-.707,0,.707],abb.confdata(-1,-1,0,1),[0]*6)

my_tool = abb.tooldata(True,abb.pose([0,0,0.1],[1,0,0,0]),abb.loaddata(0.001,[0,0,0.001],[1,0,0,0],0,0,0)) 

mp = abb.MotionProgram(tool=my_tool)
mp.SyncMoveOn()
mp.MoveAbsJ(abb.jointtarget([5,-20,30,27,-11,-27],[0]*6),abb.v1000,abb.fine)
mp.MoveL(t1,abb.v1000,abb.fine)
mp.MoveJ(t2,abb.v5000,abb.fine)
mp.MoveL(t3,abb.v500,abb.fine)
mp.WaitTime(1)
mp.MoveL(t1,abb.v5000,abb.z50)
mp.MoveJ(t2,abb.v500,abb.z200)
mp.MoveL(t3,abb.v5000,abb.fine)

# Fill motion program for T_ROB2. Both programs must have
# same number of commands
t1_2 = abb.robtarget([250,-200,1280],[.707,0,.707,0],abb.confdata(-1,-1,0,1),[0]*6)
t2_2 = abb.robtarget([250,200,1480],[.707,0,.707,0],abb.confdata(0,0,-1,1),[0]*6)
t3_2 = abb.robtarget([250,0,1280],[.707,0,.707,0],abb.confdata(0,0,0,1),[0]*6)

my_tool2 = abb.tooldata(True,abb.pose([0,0,0.5],[1,0,0,0]),abb.loaddata(0.1,[0,0,0.1],[1,0,0,0],0,0,0)) 

mp2 = abb.MotionProgram(tool=my_tool2)
mp2.SyncMoveOn()
mp2.MoveAbsJ(abb.jointtarget([1,1,40,2,-40,-2],[0]*6),abb.v1000,abb.fine)
mp2.MoveJ(t1_2,abb.v1000,abb.fine)
mp2.MoveL(t2_2,abb.v5000,abb.fine)
mp2.MoveL(t3_2,abb.v500,abb.fine)
mp2.WaitTime(1)
mp2.MoveL(t1_2,abb.v5000,abb.z50)
mp2.MoveL(t2_2,abb.v500,abb.z200)
mp2.MoveL(t3_2,abb.v5000,abb.fine)


# Execute the motion program on the robot
# Change base_url to the robot IP address
client = abb.MotionProgramExecClient(base_url="http://127.0.0.1:80")

# Execute both motion programs simultaneously
log_results = client.execute_multimove_motion_program([mp,mp2])

# log_results.data is a numpy array
import matplotlib.pyplot as plt
fig, ax1 = plt.subplots()
lns1 = ax1.plot(log_results.data[:,0], log_results.data[:,2:8])
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Joint angle (deg)")
ax2 = ax1.twinx()
lns2 = ax2.plot(log_results.data[:,0], log_results.data[:,1], '-k')
ax2.set_ylabel("Command number")
ax2.set_yticks(range(-1,int(max(log_results.data[:,1]))+1))
ax1.legend(lns1 + lns2, log_results.column_headers[2:8] + ["cmdnum"])
ax1.set_title("Robot 1 joint motion")
fig, ax1 = plt.subplots()
lns1 = ax1.plot(log_results.data[:,0], log_results.data[:,8:])
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Joint angle (deg)")
ax2 = ax1.twinx()
lns2 = ax2.plot(log_results.data[:,0], log_results.data[:,1], '-k')
ax2.set_ylabel("Command number")
ax2.set_yticks(range(-1,int(max(log_results.data[:,1]))+1))
ax1.legend(lns1 + lns2, log_results.column_headers[8:] + ["cmdnum"])
ax1.set_title("Robot 2 joint motion")
plt.show()
```

Multi-Move example using relative work object:

```python
# Multi-Move example using relative robot end effector poses

import abb_motion_program_exec as abb


# Fill motion program for T_ROB1

# Hold constant relative position for this example
t1 = abb.robtarget([0,0,-200],[1,0,0,0],abb.confdata(0,0,1,1),[0]*6)
t2 = t1
t3 = t1


my_tool = abb.tooldata(True,abb.pose([0,0,0],[1,0,0,0]),abb.loaddata(0.001,[0,0,0.001],[1,0,0,0],0,0,0))
my_wobj = abb.wobjdata(False,False,"ROB_2",abb.pose([0,0,0],[1,0,0,0]),abb.pose([0,0,0],[0,0,1,0]))

mp = abb.MotionProgram(tool=my_tool,wobj=my_wobj)
mp.SyncMoveOn()
mp.MoveAbsJ(abb.jointtarget([5,-20,30,27,-11,172],[0]*6),abb.v1000,abb.fine)
mp.WaitTime(0.1)
mp.MoveJ(t1,abb.v1000,abb.fine)
mp.MoveJ(t1,abb.v1000,abb.fine)
mp.MoveL(t1,abb.v1000,abb.fine)

# Fill motion program for T_ROB2. Both programs must have
# same number of commands
t1_2 = abb.robtarget([250,-200,1280],[.707,0,.707,0],abb.confdata(-1,-1,0,1),[0]*6)
t2_2 = abb.robtarget([250,200,1480],[.707,0,.707,0],abb.confdata(0,0,-1,1),[0]*6)
t3_2 = abb.robtarget([250,0,1280],[.707,0,.707,0],abb.confdata(0,0,0,1),[0]*6)

my_tool2 = abb.tooldata(True,abb.pose([0,0,0.5],[1,0,0,0]),abb.loaddata(0.1,[0,0,0.1],[1,0,0,0],0,0,0)) 

mp2 = abb.MotionProgram(tool=my_tool2)
mp2.SyncMoveOn()
mp2.MoveAbsJ(abb.jointtarget([1,1,40,2,-40,-2],[0]*6),abb.v1000,abb.fine)
mp2.WaitTime(0.1)
mp2.MoveL(t1_2,abb.v1000,abb.fine)
mp2.MoveL(t2_2,abb.v5000,abb.fine)
mp2.MoveL(t3_2,abb.v500,abb.fine)

# Execute the motion program on the robot
# Change base_url to the robot IP address
client = abb.MotionProgramExecClient(base_url="http://127.0.0.1:80")

# Execute both motion programs simultaneously
log_results = client.execute_multimove_motion_program([mp,mp2])

# log_results.data is a numpy array
import matplotlib.pyplot as plt
fig, ax1 = plt.subplots()
lns1 = ax1.plot(log_results.data[:,0], log_results.data[:,2:8])
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Joint angle (deg)")
ax2 = ax1.twinx()
lns2 = ax2.plot(log_results.data[:,0], log_results.data[:,1], '-k')
ax2.set_ylabel("Command number")
ax2.set_yticks(range(-1,int(max(log_results.data[:,1]))+1))
ax1.legend(lns1 + lns2, log_results.column_headers[2:8] + ["cmdnum"])
ax1.set_title("Robot 1 joint motion")
fig, ax1 = plt.subplots()
lns1 = ax1.plot(log_results.data[:,0], log_results.data[:,8:])
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Joint angle (deg)")
ax2 = ax1.twinx()
lns2 = ax2.plot(log_results.data[:,0], log_results.data[:,1], '-k')
ax2.set_ylabel("Command number")
ax2.set_yticks(range(-1,int(max(log_results.data[:,1]))+1))
ax1.legend(lns1 + lns2, log_results.column_headers[8:] + ["cmdnum"])
ax1.set_title("Robot 2 joint motion")
plt.show()

```

## License

Apache 2.0 License, Copyright 2022 Wason Technology, LLC, Rensselaer Polytechnic Institute

## Acknowledgment

This work was supported in part by Subaward No. ARM-TEC-21-02-F19 from the Advanced Robotics for Manufacturing ("ARM") Institute under Agreement Number W911NF-17-3-0004 sponsored by the Office of the Secretary of Defense. ARM Project Management was provided by Christopher Adams. The views and conclusions contained in this document are those of the authors and should not be interpreted as representing the official policies, either expressed or implied, of either ARM or the Office of the Secretary of Defense of the U.S. Government. The U.S. Government is authorized to reproduce and distribute reprints for Government purposes, notwithstanding any copyright notation herein.

This work was supported in part by the New York State Empire State Development Division of Science, Technology and Innovation (NYSTAR) under contract C160142. 

![](docs/figures/arm_logo.jpg) ![](docs/figures/nys_logo.jpg)