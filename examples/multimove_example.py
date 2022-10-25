import abb_motion_program_exec_client as abb

# Fill motion program for T_ROB1
t1 = abb.robtarget([575,-200,1280],[0,-.707,0,.707],abb.confdata(0,0,-1,1),[0]*6)
t2 = abb.robtarget([575,200,1480],[0,-.707,0,.707],abb.confdata(-1,-1,0,1),[0]*6)
t3 = abb.robtarget([575,0,1280],[0,-.707,0,.707],abb.confdata(-1,-1,0,1),[0]*6)

my_tool = abb.tooldata(True,abb.pose([0,0,0.1],[1,0,0,0]),abb.loaddata(0.001,[0,0,0.001],[1,0,0,0],0,0,0)) 

mp = abb.MotionProgram(tool=my_tool)
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
import matplotlib.ticker as plt_ticker
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


