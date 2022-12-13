MODULE motion_program_exec_egm

    CONST num egm_sample_rate:=4;

    TASK PERS bool egm_symbol_fail:=FALSE;
    TASK PERS bool have_egm:=TRUE;
    LOCAL VAR egmident egmID1;
    LOCAL VAR egmstate egmSt1;

    LOCAL VAR errnum ERR_NO_EGM:=-1;

    CONST num MOTION_PROGRAM_CMD_EGMJOINT:=50001;
    CONST num MOTION_PROGRAM_CMD_EGMPOSE:=50002;
    CONST num MOTION_PROGRAM_CMD_EGMMOVEL:=50003;
    CONST num MOTION_PROGRAM_CMD_EGMMOVEC:=50004;

    PROC motion_program_egm_init()

        BookErrNo ERR_NO_EGM;

        IF egm_symbol_fail THEN
            RETURN ;
        ENDIF

        EGMReset egmID1;
        WaitTime 0.005;
        EGMGetId egmID1;
        egmSt1:=EGMGetState(egmID1);


        EGMSetupUC ROB_1,egmID1,"joint","UCdevice"\Joint\CommTimeout:=100000;


        !IF egmST1=EGM_STATE_RUNNING THEN
        !    EGMStop egmID1,EGM_STOP_HOLD;
        !ENDIF

        EGMStreamStop egmID1;

        IF NOT have_egm THEN
            have_egm:=TRUE;
        ENDIF
    ENDPROC

    PROC motion_program_egm_start_stream()
        IF have_egm THEN
            EGMStreamStart egmID1\SampleRate:=egm_sample_rate;
        ENDIF
    ENDPROC
    
    PROC motion_program_egm_enable()
        VAR num egm_cmd;
        IF NOT try_motion_program_read_num(egm_cmd) THEN
            RAISE ERR_INVALID_MP_FILE;
        ENDIF

        ! EGM no command specified, enable streaming
        IF egm_cmd=0 THEN
            IF have_egm THEN
                EGMStreamStart egmID1\SampleRate:=egm_sample_rate;
            ENDIF
            RETURN ;
        ENDIF

        IF NOT have_egm THEN
            RAISE ERR_NO_EGM;
        ENDIF

        ! EGM joint position command
        IF egm_cmd=1 THEN
            motion_program_egm_enable_joint;
            RETURN ;
        ENDIF

        ! EGM pose command
        IF egm_cmd=2 THEN
            motion_program_egm_enable_pose;
            RETURN ;
        ENDIF

        ! EGM pose command
        IF egm_cmd=3 THEN
            motion_program_egm_enable_corr;
            RETURN ;
        ENDIF

        RAISE ERR_INVALID_OPCODE;

    ENDPROC

    PROC motion_program_egm_enable_joint()
        VAR egm_minmax minmax{6};
        VAR num pos_dev;
        VAR num speed_dev;
        VAR num i;
        VAR jointtarget joints;
        FOR i FROM 1 TO 6 DO
            IF NOT try_read_egm_minmax(minmax{i}) THEN
                RAISE ERR_INVALID_MP_FILE;
            ENDIF
        ENDFOR

        IF NOT try_motion_program_read_num(pos_dev) AND try_motion_program_read_num(speed_dev) THEN
            RAISE ERR_INVALID_MP_FILE;
        ENDIF

        ! "Move" a tiny amount to start EGM
        joints:=CJointT();
        joints.robax.rax_6:=joints.robax.rax_6+.0001;
        MoveAbsj joints,v1000,fine,motion_program_tool\WObj:=motion_program_wobj;

        EGMActJoint egmID1,\Tool:=motion_program_tool,\WObj:=motion_program_wobj\J1:=minmax{1}
        \J2:=minmax{2}\J3:=minmax{3}\J4:=minmax{4}\J5:=minmax{5}\J6:=minmax{6}\MaxPosDeviation:=pos_dev\MaxSpeedDeviation:=speed_dev;
    ENDPROC

    PROC motion_program_egm_enable_pose()
        VAR pose posecor;
        VAR egmframetype posecor_frtype;
        VAR pose posesens;
        VAR egmframetype posesens_frtype;
        VAR egm_minmax minmax{6};
        VAR num pos_dev;
        VAR num speed_dev;
        VAR num i;
        VAR jointtarget joints;

        IF NOT (
            try_motion_program_read_pose(posecor)
            AND try_read_egm_frtype(posecor_frtype)
            AND try_motion_program_read_pose(posesens)
            AND try_read_egm_frtype(posesens_frtype)
        ) THEN
            RAISE ERR_INVALID_MP_FILE;
        ENDIF

        FOR i FROM 1 TO 6 DO
            IF NOT try_read_egm_minmax(minmax{i}) THEN
                RAISE ERR_INVALID_MP_FILE;
            ENDIF
        ENDFOR

        IF NOT try_motion_program_read_num(pos_dev) AND try_motion_program_read_num(speed_dev) THEN
            RAISE ERR_INVALID_MP_FILE;
        ENDIF

        EGMSetupUC ROB_1,egmID1,"pose","UCdevice"\Pose\CommTimeout:=100000;

        ! "Move" a tiny amount to start EGM
        joints:=CJointT();
        joints.robax.rax_6:=joints.robax.rax_6+.0001;
        MoveAbsj joints,v1000,fine,motion_program_tool\WObj:=motion_program_wobj;

        EGMActPose egmID1,\StreamStart,\Tool:=motion_program_tool,\WObj:=motion_program_wobj,posecor,posecor_frtype,posesens,posesens_frtype,
            \X:=minmax{1}\Y:=minmax{2}\Z:=minmax{3}\Rx:=minmax{4}\Ry:=minmax{5}\Rz:=minmax{6}\MaxPosDeviation:=pos_dev\MaxSpeedDeviation:=speed_dev;
    ENDPROC

    PROC motion_program_egm_enable_corr()
        VAR pose posesens;
        VAR jointtarget joints;
        IF NOT try_motion_program_read_pose(posesens) THEN
            RAISE ERR_INVALID_MP_FILE;
        ENDIF

        EGMSetupUC ROB_1,egmID1,"path_corr","UCdevice"\PathCorr\APTR;

        joints:=CJointT();
        joints.robax.rax_6:=joints.robax.rax_6+.0001;
        MoveAbsj joints,v1000,fine,motion_program_tool\WObj:=motion_program_wobj;

        EGMActMove egmID1,posesens\SampleRate:=48;
        
    ENDPROC

    FUNC bool try_read_egm_minmax(INOUT egm_minmax minmax)
        RETURN (try_motion_program_read_num(minmax.min) AND try_motion_program_read_num(minmax.max));
    ENDFUNC

    FUNC bool try_read_egm_frtype(INOUT egmframetype frtype)
        VAR num frtype_code;
        IF NOT try_motion_program_read_num(frtype_code) THEN
            RETURN FALSE;
        ENDIF

        IF frtype_code=1 THEN
            frtype:=EGM_FRAME_TOOL;
        ELSEIF frtype_code=2 THEN
            frtype:=EGM_FRAME_WOBJ;
        ELSEIF frtype_code=3 THEN
            frtype:=EGM_FRAME_WORLD;
        ELSEIF frtype_code=4 THEN
            frtype:=EGM_FRAME_JOINT;
        ELSE
            frtype:=EGM_FRAME_BASE;
        ENDIF
        RETURN TRUE;
    ENDFUNC

    FUNC bool try_motion_program_egmrunjoint()
        VAR num condtime;
        VAR num rampin;
        VAR num rampout;

        IF NOT (
            try_motion_program_read_num(condtime)
            AND try_motion_program_read_num(rampin)
            AND try_motion_program_read_num(rampout)
        ) THEN
            RETURN FALSE;
        ENDIF

        SetDO motion_program_stop_egm,0;
        EGMRunJoint egmID1,EGM_STOP_HOLD,\NoWaitCond\J1\J2\J3\J4\J5\J6\CondTime:=condtime\RampInTime:=rampin;
        WaitDO motion_program_stop_egm,1;
        EGMStop egmID1,EGM_STOP_HOLD\RampOutTime:=rampout;
        EGMStreamStart egmID1\SampleRate:=egm_sample_rate;

        RETURN TRUE;
    ENDFUNC

    FUNC bool try_motion_program_egmrunpose()
        VAR num condtime;
        VAR num rampin;
        VAR num rampout;
        VAR pose offset;

        IF NOT (
            try_motion_program_read_num(condtime)
            AND try_motion_program_read_num(rampin)
            AND try_motion_program_read_num(rampout)
            AND try_motion_program_read_pose(offset)
        ) THEN
            RETURN FALSE;
        ENDIF

        SetDO motion_program_stop_egm,0;
        EGMRunPose egmID1,EGM_STOP_HOLD,\NoWaitCond\x\y\z\Rx\Ry\Rz\CondTime:=condtime\RampInTime:=rampin\RampOutTime:=rampout\Offset:=offset;
        WaitDO motion_program_stop_egm,1;
        EGMStop egmID1,EGM_STOP_HOLD\RampOutTime:=rampout;
        EGMStreamStart egmID1\SampleRate:=egm_sample_rate;
        RETURN TRUE;
    ENDFUNC

    FUNC bool try_motion_program_run_egmmovel(num cmd_num)
        VAR robtarget rt;
        VAR speeddata sd;
        VAR zonedata zd;
        IF NOT (
        try_motion_program_read_rt(rt)
        AND try_motion_program_read_sd(sd)
        AND try_motion_program_read_zd(zd)
        ) THEN
            RETURN FALSE;
        ENDIF

        EGMMoveL egmID1,rt,sd,zd,motion_program_tool\WObj:=motion_program_wobj;

        RETURN TRUE;

    ENDFUNC

    FUNC bool try_motion_program_run_egmmovec(num cmd_num)
        VAR robtarget rt1;
        VAR robtarget rt2;
        VAR speeddata sd;
        VAR zonedata zd;
        IF NOT (
        try_motion_program_read_rt(rt1)
        AND try_motion_program_read_rt(rt2)
        AND try_motion_program_read_sd(sd)
        AND try_motion_program_read_zd(zd)
        ) THEN
            RETURN FALSE;
        ENDIF

        EGMMoveC egmID1,rt1,rt2,sd,zd,motion_program_tool\WObj:=motion_program_wobj;

        RETURN TRUE;

    ENDFUNC

    FUNC bool try_motion_program_run_egm_cmd(num cmd_num,num cmd_op)
        TEST cmd_op
        CASE MOTION_PROGRAM_CMD_EGMJOINT:
            RETURN try_motion_program_egmrunjoint();
        CASE MOTION_PROGRAM_CMD_EGMPOSE:
            RETURN try_motion_program_egmrunpose();
        CASE MOTION_PROGRAM_CMD_EGMMOVEL:
            RETURN try_motion_program_run_egmmovel(cmd_num);
        CASE MOTION_PROGRAM_CMD_EGMMOVEC:
            RETURN try_motion_program_run_egmmovec(cmd_num);
        DEFAULT:
            RAISE ERR_INVALID_OPCODE;
        ENDTEST
    ENDFUNC

ENDMODULE