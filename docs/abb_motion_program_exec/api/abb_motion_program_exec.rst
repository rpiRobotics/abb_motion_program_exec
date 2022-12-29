abb_motion_program_exec
=======================

ABB IRC5 Controller Motion Program command client.

This module contains types found on the ABB Robot Controller. Documentation is taken from the
ABB Robotics manual "Technical reference manual RAPID Instructions, Functions and Data types, 
Document ID: 3HAC 16581-1". Note that most of the following data structures are all direct copies of the 
underlying ABB types.

abb_motion_program_exec
-----------------------

.. automodule:: abb_motion_program_exec
    :members: speeddata, zonedata, jointtarget, pose, confdata, robtarget, loaddata, CirPathModeSwitch, tooldata,
              wobjdata, egm_minmax, EGMStreamConfig, EGMJointTargetConfig, egmframetype, EGMPoseTargetConfig,
              EGMPathCorrectionConfig, MotionProgramExecClient

.. autoclass:: MotionProgram
    :members:

    .. method:: MoveAbsJ(to_joint_pos: jointtarget, speed: speeddata, zone: zonedata)

        Append Move Absolute Joint command to motion program

        :param to_joint_pos: The destination absolute joint position of the robot and external axes.
        :type to_joint_pos: jointtarget
        :param speed: The speed data that applies to movements.
        :type speed: speeddata
        :param zone: Zone data for the movement.
        :type zone: zonedata
            
    .. method:: MoveJ(to_point: robtarget, speed: speeddata, zone: zonedata)

        Append move to position in joint space command to motion program

        :param to_point: The destination point of the robot and external axes.
        :type to_point: robtarget
        :param speed: The speed data that applies to movements.
        :type speed: speeddata
        :param zone: Zone data for the movement.
        :type zone: zonedata

    .. method:: MoveL(to_point: robtarget, speed: speeddata, zone: zonedata)

        Append move to position in straight line command to motion program

        :param to_point: The destination point of the robot and external axes.
        :type to_point: robtarget
        :param speed: The speed data that applies to movements.
        :type speed: speeddata
        :param zone: Zone data for the movement.
        :type zone: zonedata

    .. method:: MoveC(cir_point: robtarget, to_point: robtarget, speed: speeddata, zone: zonedata)

        Append move to position circularly through a point command to motion program

        :param cir_point: The circle point of the robot and external axes.
        :type cir_point: robtarget
        :param to_point: The destination point of the robot and external axes.
        :type to_point: robtarget
        :param speed: The speed data that applies to movements.
        :type speed: speeddata
        :param zone: Zone data for the movement.
        :type zone: zonedata

    .. method:: WaitTime(t: float)

        Append wait for a specified time in seconds command

        :param t: The time to wait in seconds
        :type t: float

    .. method:: CirPathMode(switch: CirPathModeSwitch)

        Append select reorientation method during circular moves command

        :param switch: The selected circular reorientation method
        :type switch: CirPathModeSwitch

    .. method:: SyncMoveOn()

        Append enable synchronous move command. Only valid on MultiMove systems

    .. method:: SyncMoveOff()

        Append disable synchronous move command. Only valid on MultiMove system

    .. method:: EGMRunJoint(cond_time: float, ramp_in_time: float, ramp_out_time: float)

        Append run EGM joint target streaming motion command. Use ``MotionProgramExecClient.stop_egm()`` to 
        exit EGM command mode.

        :param cond_time: Condition time for the move. Set to a very large number to continue running until stopped.
        :type cond_time: float
        :param ramp_in_time: Motion ramp in time. Normally set to minimum value of 0.05
        :type ramp_in_tme: float
        :param ramp_out_time: Motion ramp out time. Normally set to minimum value of 0.05
        :type ramp_out_tme: float

    .. method:: EGMRunPose(cond_time: float, ramp_in_time: float, ramp_out_time: float)

        Apnned run EGM pose target streaming motion command. Use ``MotionProgramExecClient.stop_egm()`` to exit 
        EGM command mode.

        :param cond_time: Condition time for the move. Set to a very large number to continue running until stopped.
        :type cond_time: float
        :param ramp_in_time: Motion ramp in time. Normally set to minimum value of 0.05
        :type ramp_in_tme: float
        :param ramp_out_time: Motion ramp out time. Normally set to minimum value of 0.05
        :type ramp_out_tme: float
        :param offset: Offset of commanded pose. Typically set to zero offset and no rotation
        :type offset: pose

    .. method:: EGMMoveL(to_point: robtarget, speed: speeddata, zone: zonedata)

        Append move to position in straight line with EGM correction command to motion program

        :param to_point: The destination point of the robot and external axes.
        :type to_point: robtarget
        :param speed: The speed data that applies to movements.
        :type speed: speeddata
        :param zone: Zone data for the movement.
        :type zone: zonedata


    .. method:: EGMMoveC(cir_point: robtarget, to_point: robtarget, speed: speeddata, zone: zonedata)

        Append move to position circularly through a point with EGM correction command to motion program

        :param cir_point: The circle point of the robot and external axes.
        :type cir_point: robtarget
        :param to_point: The destination point of the robot and external axes.
        :type to_point: robtarget
        :param speed: The speed data that applies to movements.
        :type speed: speeddata
        :param zone: Zone data for the movement.
        :type zone: zonedata

    