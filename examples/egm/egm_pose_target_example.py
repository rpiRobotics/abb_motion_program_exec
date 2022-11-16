from abb_robot_client.egm import EGM
import abb_motion_program_exec as abb
import time
import numpy as np
import copy

mm = abb.egm_minmax(-1e-3,1e-3)

corr_frame = abb.pose([0,0,0],[1,0,0,0])
corr_fr_type = abb.egmframetype.EGM_FRAME_WOBJ
sense_frame = abb.pose([0,0,0],[1,0,0,0])
sense_fr_type = abb.egmframetype.EGM_FRAME_WOBJ
egm_offset = abb.pose([0,0,0],[1,0,0,0])

egm_config = abb.EGMPoseTargetConfig(corr_frame, corr_fr_type, sense_frame, sense_fr_type,
    mm, mm, mm, mm, mm ,mm, 1000, 1000
)

r1 = abb.robtarget([400,0,600],[0.7071068, 0., 0.7071068, 0.],abb.confdata(0,0,0,1),[0]*6)

mp = abb.MotionProgram(egm_config = egm_config)
mp.MoveJ(r1,abb.v5000,abb.fine)
mp.EGMRunPose(10, 0.05, 0.05, egm_offset)
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