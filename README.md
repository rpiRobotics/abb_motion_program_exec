# abb_motion_program_exec

`abb_motion_program_exec` provides a simple way to download and run a sequence of
`MoveAbsJ`, `MoveJ`, `MoveL`, `MoveC`, and `WaitTime` commands on
an ABB IRC5 robot controller. This program is intended to be a proof of
concept for more sophisticated controller interfaces. Multi-move control of two robots is also
supported.

## Installation

Begin by installing the software for the robot controller. This software can be
installed manually by copying files to the robot controller and importing configuration files,
or by using a RobotWare Add-In. The RobotWare Add-In is cleaner and probably more reliable,
but is also less flexible and requires using the Installation Manager which can be confusing. If
you aren't sure which to use, try using the manual installation first.

* See [robot_setup_manual.md](doc/robot_setup_manual.md) for manual setup instructions.
* See [robot_setup_robotware_addin.md](doc/robot_setup_robotware_addin.md) for RobotWare Add-In 
  setup instructions
* See [robot_multimove_setup_manual.md](doc/robot_multimove_setup_manual.md) for ABB Multi-Move
  setup to control two robots. See later sections of this doc for more information on Multi-Move.

This contains the robot-side code, that reads
and executes the contents of `motion_program.bin`. `motion_program.bin`
contains the sequence of instructions to run, encoded in binary
format for fast interpretation.

The Python client module is `abb_motion_program_exec_client.py`. This
script can be executed to run a sample motion sequence on an ABB 1200
robot. (Simulation only due to the choice of waypoints!) The
module can also be included in another Python program.

**Only one instance of a Robot Studio virtual controller can be run at a time.** Only
instances of Robot Studio can be run at a time running a single virtual controller. This is due to
the controller using TCP port 80 on the local computer to accept REST commands from Python. If
more than one controller is started, TCP port 80 will already be in use and can cause unpredictable
behavior. Restart the computer if connections cannot be made from Python to the controller. Multiple
real robots can be used concurrently since they will each have a unique IP address to bind port 80.

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
log_csv_bin = mp_client.execute_motion_program(mp)
```

`log_csv_bin` will contain a CSV file in binary format. This can either be saved to a binary
file directly, or converted to a string and used in Python. To convert to a string, use:

```python
log_csv = log_csv_bin.decode('ascii')
```

The CSV data has the following columns:

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

The first line of the CSV data contains column headers.

## Python module installation

The `abb_motion_program_exec_client` module can be installed into the local Python
installation using the following command executed in the project root directory:

```
pip install --user .
```

## Example

```python
import abb_motion_program_exec_client as abb

j1 = abb.jointtarget([10,20,30,40,50,60],[0]*6)
j2 = abb.jointtarget([90,-91,60,-93,94,-95],[0]*6)
j3 = abb.jointtarget([-80,81,-82,83,-84,85],[0]*6)

my_tool = abb.tooldata(True,abb.pose([0,0,0.1],[1,0,0,0]),abb.loaddata(0.001,[0,0,0.001],[1,0,0,0],0,0,0)) 

mp = abb.MotionProgram(tool=my_tool)
mp.MoveAbsJ(j1,abb.v1000,abb.fine)
mp.MoveAbsJ(j2,abb.v5000,abb.fine)
mp.MoveAbsJ(j3,abb.v500,abb.fine)
mp.MoveAbsJ(j2,abb.v5000,abb.z50)
mp.MoveAbsJ(j3,abb.v500,abb.z200)
mp.MoveAbsJ(j2,abb.v5000,abb.fine)
mp.WaitTime(1)

r1 = abb.robtarget([0.1649235*1e3, 0.1169957*1e3, 0.9502961*1e3], [ 0.6776466, -0.09003431, 0.6362069, 0.3576725 ], abb.confdata(0,0,0,0),[0]*6)
r2 = abb.robtarget([ 0.6243948*1e3, -0.479558*1e3 ,  0.7073749*1e3], [ 0.6065634, -0.2193409,  0.6427138, -0.4133877], abb.confdata(-1,-1,0,1),[0]*6)

r3 = abb.robtarget([417.9236, 276.9956, 885.2959], [ 0.8909725 , -0.1745558 ,  0.08864544,  0.4096832 ], abb.confdata( 0.,  1., -2.,  0.),[0]*6)
r4 = abb.robtarget([417.9235 , -11.00438, 759.2958 ], [0.7161292 , 0.1868255 , 0.01720813, 0.6722789 ], abb.confdata( 0.,  2., -2.,  0.),[0]*6)
r5 = abb.robtarget([ 417.9235, -173.0044,  876.2958], [0.6757616, 0.3854275, 0.2376617, 0.5816431], abb.confdata(-1.,  1., -1.,  0.),[0]*6)

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

# Write log csv to file
with open("log.csv","wb") as f:
    f.write(log_results)

# Or convert to string and use in memory
log_results_str = log_results.decode('ascii')
print(log_results_str)

```

Example log CSV data (truncated):

```
timestamp, cmd_num, J1, J2, J3, J4, J5, J6
85.34, 1, 30.5081, 4.1176, 5.80734, 161.136, 87.3982, -162.344
85.344, 1, 30.5081, 4.1176, 5.80734, 161.136, 87.3982, -162.344
85.348, 1, 30.5081, 4.1176, 5.80734, 161.136, 87.3982, -162.344
85.352, 1, 30.5081, 4.1176, 5.80734, 161.136, 87.3982, -162.344
```

## Multi-Move Robot Example

Two robots can be controlled using ABB Multi-Move. See 
[robot_multimove_setup_manual.md](doc/robot_multimove_setup_manual.md) for setup instructions.

They must have exactly the same number of motion commands. The commands
are passed with the `\ID` parameter corresponding to the command number. `SyncMoveOn` is activated
to cause the robots to move in sync. The `execute_multimove_motion_program()` function
of `MotionProgramExecClient` is used to send multi-move programs to the robot.

```python
import abb_motion_program_exec_client as abb


# Fill motion program for T_ROB1
t1 = abb.robtarget([575,200,780],[.707,0,.707,0],abb.confdata(0,0,-1,1),[0]*6)
t2 = abb.robtarget([575,-200,980],[.707,0,.707,0],abb.confdata(0,0,-1,1),[0]*6)
t3 = abb.robtarget([575,0,780],[.707,0,.707,0],abb.confdata(-1,-1,0,1),[0]*6)

my_tool = abb.tooldata(True,abb.pose([0,0,0.1],[1,0,0,0]),abb.loaddata(0.001,[0,0,0.001],[1,0,0,0],0,0,0)) 

mp = abb.MotionProgram(tool=my_tool)
mp.MoveAbsJ(abb.jointtarget([5,-20,30,27,-11,-27],[0]*6),abb.v1000,abb.fine)
mp.MoveL(t1,abb.v1000,abb.fine)
mp.MoveL(t2,abb.v5000,abb.fine)
mp.MoveL(t3,abb.v500,abb.fine)
mp.WaitTime(1)
mp.MoveL(t1,abb.v5000,abb.z50)
mp.MoveL(t2,abb.v500,abb.z200)
mp.MoveL(t3,abb.v5000,abb.fine)

# Fill motion program for T_ROB2. Both programs must have
# same number of commands
t1_2 = abb.robtarget([1750,-200,1280],[.707,0,.707,0],abb.confdata(-1,-1,0,1),[0]*6)
t2_2 = abb.robtarget([1750,200,1480],[.707,0,.707,0],abb.confdata(0,0,-1,1),[0]*6)
t3_2 = abb.robtarget([1750,0,1280],[.707,0,.707,0],abb.confdata(0,0,0,1),[0]*6)

my_tool2 = abb.tooldata(True,abb.pose([0,0,0.5],[1,0,0,0]),abb.loaddata(0.1,[0,0,0.1],[1,0,0,0],0,0,0)) 

mp2 = abb.MotionProgram(tool=my_tool2)
mp2.MoveAbsJ(abb.jointtarget([1,1,40,2,-40,-2],[0]*6),abb.v1000,abb.fine)
mp2.MoveL(t1_2,abb.v1000,abb.fine)
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

# Write log csv to file
with open("log.csv","wb") as f:
   f.write(log_results)

# Or convert to string and use in memory
log_results_str = log_results.decode('ascii')
print(log_results_str)

```

## License

Apache 2.0 License, Copyright 2022 Wason Technology, LLC, Rensselaer Polytechnic Institute
