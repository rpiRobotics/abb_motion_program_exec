from abb_robot_client.egm import EGM
import abb_motion_program_exec as abb
import time
import numpy as np
import copy
import math

sensor_frame = abb.pose([0,0,0],[1,0,0,0])

egm_config = abb.EGMPathCorrectionConfig(sensor_frame)

r1 = abb.robtarget([400,0,600],[0.7071068, 0., 0.7071068, 0.],abb.confdata(0,0,0,1),[0]*6)
r2 = abb.robtarget([400,200,600],[0.7071068, 0., 0.7071068, 0.],abb.confdata(0,0,0,1),[0]*6)
r3 = abb.robtarget([400,400,800],[0.7071068, 0., 0.7071068, 0.],abb.confdata(0,0,0,1),[0]*6)

mp = abb.MotionProgram(egm_config = egm_config)
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