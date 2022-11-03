# Single robot preemption example

import abb_motion_program_exec_client as abb
import time

j1 = abb.jointtarget([10,20,30,40,50,60],[0]*6)
j2 = abb.jointtarget([90,-91,60,-93,94,-95],[0]*6)
j3 = abb.jointtarget([-80,81,-82,83,-84,85],[0]*6)

my_tool = abb.tooldata(True,abb.pose([0,0,0.1],[1,0,0,0]),abb.loaddata(0.001,[0,0,0.001],[1,0,0,0],0,0,0)) 

# mp is the nominal motion program

mp = abb.MotionProgram(tool=my_tool)
mp.MoveAbsJ(j2,abb.v5000,abb.fine)


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

# Create preempting motion program. This may be generated in real time based on sensor feedback.
# first_cmd_num must be one increment after preemption number

mp_p1 = abb.MotionProgram(first_cmd_num=5)
mp_p1.MoveJ(r3,abb.v5000,abb.fine)

mp_p1.MoveC(r4,r5,abb.v200,abb.z10)
mp_p1.MoveC(r4,r3,abb.v50,abb.fine)

# Print out RAPID module of nominal motion program for debugging
print(mp.get_program_rapid())

# Execute the motion program on the robot
# Change base_url to the robot IP address
client = abb.MotionProgramExecClient(base_url="http://127.0.0.1:80")

# Each preempting must have an incrementing preempt_number
preempt_number = 1

# Set wait=False so execute_motion_program returns immediately
lognum = client.execute_motion_program(mp, wait=False)
time.sleep(0.25)

# Request preemption after command number 4
client.preempt_motion_program(mp_p1, preempt_number=1, preempt_cmdnum=4)
while client.is_motion_program_running():
    # Currently executing command number
    cmd_num = client.get_current_cmdnum()
    # Current preemption number
    preempt_num = client.get_current_preempt_number()
    # Current command number that has been queued for execution
    queued_num = client.get_queued_cmdnum()
    print(f"preempt_num={preempt_num}, cmd_num={cmd_num}, queued_cmd_num={queued_num}")
    time.sleep(0.05)

# Read the results once motion program stops running
log_results = client.read_motion_program_result_log(lognum)

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