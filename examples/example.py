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
