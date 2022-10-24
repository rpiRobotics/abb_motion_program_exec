# Multi-Move example using relative robot end effector poses

import abb_motion_program_exec_client as abb


# Fill motion program for T_ROB1

# Hold constant relative position for this example
t1 = abb.robtarget([0,0,-200],[1,0,0,0],abb.confdata(0,0,1,1),[0]*6)
t2 = t1
t3 = t1


my_tool = abb.tooldata(True,abb.pose([0,0,0],[1,0,0,0]),abb.loaddata(0.001,[0,0,0.001],[1,0,0,0],0,0,0))
my_wobj = abb.wobjdata(False,False,"ROB_2",abb.pose([0,0,0],[1,0,0,0]),abb.pose([0,0,0],[0,0,1,0]))

mp = abb.MotionProgram(tool=my_tool,wobj=my_wobj)
mp.MoveAbsJ(abb.jointtarget([5,-20,30,27,-11,172],[0]*6),abb.v1000,abb.fine)
mp.WaitTime(0.1)
mp.MoveJ(t1,abb.v1000,abb.fine)
mp.MoveL(t1,abb.v1000,abb.fine)
mp.MoveL(t1,abb.v1000,abb.fine)

# Fill motion program for T_ROB2. Both programs must have
# same number of commands
t1_2 = abb.robtarget([250,-200,1280],[.707,0,.707,0],abb.confdata(-1,-1,0,1),[0]*6)
t2_2 = abb.robtarget([250,200,1480],[.707,0,.707,0],abb.confdata(0,0,-1,1),[0]*6)
t3_2 = abb.robtarget([250,0,1280],[.707,0,.707,0],abb.confdata(0,0,0,1),[0]*6)

my_tool2 = abb.tooldata(True,abb.pose([0,0,0.5],[1,0,0,0]),abb.loaddata(0.1,[0,0,0.1],[1,0,0,0],0,0,0)) 

mp2 = abb.MotionProgram(tool=my_tool2)
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

# Write log csv to file
with open("log.csv","wb") as f:
   f.write(log_results)

# Or convert to string and use in memory
log_results_str = log_results.decode('ascii')
print(log_results_str)


