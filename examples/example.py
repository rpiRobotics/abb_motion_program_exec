import abb_motion_program_exec_client as abb

def run_program():

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
    #with open("log.csv","wb") as f:
    #    f.write(log_results)

    # Or convert to string and use in memory
    log_results_str = log_results.decode('ascii')
    print(log_results_str)

while True:
    run_program()