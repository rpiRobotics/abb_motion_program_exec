MODULE motion_program_exec

    CONST num motion_program_file_version:=10006;

    PERS motion_program_state_type motion_program_state{2};

    LOCAL VAR iodev motion_program_io_device;
    LOCAL VAR rawbytes motion_program_bytes;
    LOCAL VAR num motion_program_bytes_offset;

    PERS tooldata motion_program_tool1;
    PERS tooldata motion_program_tool2;
    PERS wobjdata motion_program_wobj1;
    PERS wobjdata motion_program_wobj2;

    VAR rmqslot logger_rmq;

    VAR intnum motion_trigg_intno;
    VAR triggdata motion_trigg_data;
    VAR num motion_cmd_num_history{128};
    VAR num motion_current_cmd_ind;
    VAR num motion_max_cmd_ind;
    VAR num task_ind;

    PERS tasks task_list{2}:=[["T_ROB1"],["T_ROB2"]];

    VAR syncident sync1;
    
    VAR errnum ERR_INVALID_MP_VERSION := -1;
    VAR errnum ERR_INVALID_MP_FILE := -1;

    PROC main()
        VAR zonedata z:=fine;
        VAR string taskname;
        VAR string filename;
        BookErrNo ERR_INVALID_MP_VERSION;
        BookErrNo ERR_INVALID_MP_FILE;
        SyncMoveOn sync1,task_list;
        taskname:=GetTaskName();
        IF taskname="T_ROB1" THEN
            filename:="motion_program.bin";
            task_ind:=1;
        ELSEIF taskname="T_ROB2" THEN
            filename:="motion_program2.bin";
            task_ind:=2;
        ENDIF
        motion_program_state{task_ind}.current_cmd_num:=-1;
        CONNECT motion_trigg_intno WITH motion_trigg_trap;
        RMQFindSlot logger_rmq,"RMQ_logger";
        TriggInt motion_trigg_data,0.001,\Start,motion_trigg_intno;
        run_motion_program_file(filename);
        ErrWrite\I,"Motion Program Complete","Motion Program Complete";
        IDelete motion_trigg_intno;
    ENDPROC

    PROC run_motion_program_file(string filename)
        ErrWrite\I,"Motion Program Begin","Motion Program Begin";
        close_motion_program_file;
        open_motion_program_file(filename);
        ErrWrite\I,"Motion Program Start Program","Motion Program Start Program timestamp: "+motion_program_state{task_ind}.program_timestamp;
        IF task_ind=1 THEN
            motion_program_req_log_start;
        ENDIF
        motion_program_run;
        close_motion_program_file;
        IF task_ind=1 THEN
            motion_program_req_log_end;
        ENDIF
    ENDPROC

    PROC open_motion_program_file(string filename)
        VAR num ver;
        VAR tooldata mtool;
        VAR wobjdata mwobj;
        VAR string timestamp;
        motion_program_state{task_ind}.motion_program_filename:=filename;
        Open "RAMDISK:"\File:=filename,motion_program_io_device,\Read\Bin;
        IF NOT try_motion_program_read_num(ver) THEN
            RAISE ERR_FILESIZE;
        ENDIF

        IF ver<>motion_program_file_version THEN
            ErrWrite"Invalid Motion Program","Invalid motion program file version";
            RAISE ERR_INVALID_MP_VERSION;
        ENDIF

        IF NOT try_motion_program_read_td(mtool) THEN
            ErrWrite"Invalid Motion Program Tool","Invalid motion program tool";
            RAISE ERR_INVALID_MP_FILE;
        ENDIF
        
        IF NOT try_motion_program_read_wd(mwobj) THEN
            ErrWrite"Invalid Motion Program Wobj","Invalid motion program wobj";
            RAISE ERR_INVALID_MP_FILE;
        ENDIF

        IF NOT try_motion_program_read_string(timestamp) THEN
            ErrWrite"Invalid Motion Program Timestamp","Invalid motion program timestamp";
            RAISE ERR_INVALID_MP_FILE;
        ENDIF

        motion_program_state{task_ind}.program_timestamp:=timestamp;
        motion_current_cmd_ind:=0;
        motion_max_cmd_ind:=0;

        ErrWrite\I,"Motion Program Opened","Motion Program Opened with timestamp: "+timestamp;
        IF task_ind=1 THEN
            motion_program_tool1:=mtool;
        ELSE
            motion_program_tool2:=mtool;
        ENDIF
        IF task_ind=1 THEN
            motion_program_wobj1:=mwobj;
        ELSE
            motion_program_wobj2:=mwobj;
        ENDIF
    ENDPROC

    PROC close_motion_program_file()
        motion_program_state{task_ind}.motion_program_filename:="";
        Close motion_program_io_device;
    ERROR
        SkipWarn;
        TRYNEXT;
    ENDPROC

    PROC motion_program_run()

        VAR bool keepgoing:=TRUE;
        VAR num cmd_num;
        VAR num cmd_op;
        motion_program_state{task_ind}.current_cmd_num:=-1;
        motion_program_state{task_ind}.running:=TRUE;
        WHILE keepgoing DO
            keepgoing:=try_motion_program_run_next_cmd(cmd_num,cmd_op);
        ENDWHILE
        WaitRob\ZeroSpeed;
        motion_program_state{task_ind}.running:=FALSE;
    ERROR
        motion_program_state{task_ind}.running:=FALSE;
        RAISE ;
    ENDPROC

    FUNC bool try_motion_program_run_next_cmd(INOUT num cmd_num,INOUT num cmd_op)

        VAR num local_cmd_ind;

        IF NOT (try_motion_program_read_num(cmd_num) AND try_motion_program_read_num(cmd_op)) THEN
            RETURN FALSE;
        ENDIF

        !motion_program_state.current_cmd_num:=cmd_num;
        motion_max_cmd_ind:=motion_max_cmd_ind+1;
        local_cmd_ind:=((motion_max_cmd_ind-1) MOD 128)+1;

        TEST cmd_op
        CASE 0:
            RETURN TRUE;
        CASE 1:
            motion_cmd_num_history{local_cmd_ind}:=-1;
            RETURN try_motion_program_run_moveabsj(cmd_num);
        CASE 2:
            motion_cmd_num_history{local_cmd_ind}:=cmd_num;
            RETURN try_motion_program_run_movej(cmd_num);
        CASE 3:
            motion_cmd_num_history{local_cmd_ind}:=cmd_num;
            RETURN try_motion_program_run_movel(cmd_num);
        CASE 4:
            motion_cmd_num_history{local_cmd_ind}:=cmd_num;
            RETURN try_motion_program_run_movec(cmd_num);
        CASE 5:
            motion_cmd_num_history{local_cmd_ind}:=-1;
            RETURN try_motion_program_wait(cmd_num);
        CASE 6:
            motion_cmd_num_history{local_cmd_ind}:=-1;
            RETURN try_motion_program_set_cirmode(cmd_num);
        DEFAULT:
            RAISE ERR_WRONGVAL;
        ENDTEST


    ENDFUNC

    FUNC bool try_motion_program_run_moveabsj(num cmd_num)
        VAR jointtarget j;
        VAR speeddata sd;
        VAR zonedata zd;
        IF NOT (
        try_motion_program_read_jt(j)
        AND try_motion_program_read_sd(sd)
        AND try_motion_program_read_zd(zd)
        ) THEN
            RETURN FALSE;
        ENDIF
        IF IsSyncMoveOn() THEN
            IF task_ind=1 THEN
                MoveAbsJ j,\ID:=cmd_num,sd,zd,motion_program_tool1\Wobj:=motion_program_wobj1;
            ELSE
                MoveAbsJ j,\ID:=cmd_num,sd,zd,motion_program_tool2\Wobj:=motion_program_wobj2;
            ENDIF
        ELSE
            IF task_ind=1 THEN
                MoveAbsJ j,sd,zd,motion_program_tool1\Wobj:=motion_program_wobj1;
            ELSE
                MoveAbsJ j,sd,zd,motion_program_tool2\Wobj:=motion_program_wobj2;
            ENDIF
        ENDIF
        RETURN TRUE;
    ENDFUNC

    FUNC bool try_motion_program_run_movej(num cmd_num)
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
        IF IsSyncMoveON() THEN
            IF task_ind=1 THEN
                TriggJ rt,\ID:=cmd_num,sd,motion_trigg_data,zd,motion_program_tool1\Wobj:=motion_program_wobj1;
            ELSE
                TriggJ rt,\ID:=cmd_num,sd,motion_trigg_data,zd,motion_program_tool2\Wobj:=motion_program_wobj2;
            ENDIF
        ELSE
            IF task_ind=1 THEN
                TriggJ rt,sd,motion_trigg_data,zd,motion_program_tool1\Wobj:=motion_program_wobj1;
            ELSE
                TriggJ rt,sd,motion_trigg_data,zd,motion_program_tool2\Wobj:=motion_program_wobj2;
            ENDIF
        ENDIF
        RETURN TRUE;

    ENDFUNC

    FUNC bool try_motion_program_run_movel(num cmd_num)
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
        IF IsSyncMoveOn() THEN
            IF task_ind=1 THEN
                TriggL rt,\ID:=cmd_num,sd,motion_trigg_data,zd,motion_program_tool1\Wobj:=motion_program_wobj1;
            ELSE
                TriggL rt,\ID:=cmd_num,sd,motion_trigg_data,zd,motion_program_tool2\Wobj:=motion_program_wobj2;
            ENDIF
        ELSE
            IF task_ind=1 THEN
                TriggL rt,sd,motion_trigg_data,zd,motion_program_tool1\Wobj:=motion_program_wobj1;
            ELSE
                TriggL rt,sd,motion_trigg_data,zd,motion_program_tool2\Wobj:=motion_program_wobj2;
            ENDIF
        ENDIF
        RETURN TRUE;

    ENDFUNC

    FUNC bool try_motion_program_run_movec(num cmd_num)
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
        IF IsSyncMoveOn() THEN
            IF task_ind=1 THEN
                TriggC rt1,rt2,\ID:=cmd_num,sd,motion_trigg_data,zd,motion_program_tool1\Wobj:=motion_program_wobj1;
            ELSE
                TriggC rt1,rt2,\ID:=cmd_num,sd,motion_trigg_data,zd,motion_program_tool2\Wobj:=motion_program_wobj2;
            ENDIF
        ELSE
            IF task_ind=1 THEN
                TriggC rt1,rt2,sd,motion_trigg_data,zd,motion_program_tool1\Wobj:=motion_program_wobj1;
            ELSE
                TriggC rt1,rt2,sd,motion_trigg_data,zd,motion_program_tool2\Wobj:=motion_program_wobj2;
            ENDIF
        ENDIF
        RETURN TRUE;

    ENDFUNC

    FUNC bool try_motion_program_wait(num cmd_num)
        VAR num t;
        IF NOT try_motion_program_read_num(t) THEN
            RETURN FALSE;
        ENDIF
        WaitRob\ZeroSpeed;
        motion_program_state{task_ind}.current_cmd_num:=cmd_num;
        WaitTime t;
        RETURN TRUE;
    ENDFUNC

    FUNC bool try_motion_program_set_cirmode(num cmd_num)
        VAR num switch;
        IF NOT try_motion_program_read_num(switch) THEN
            RETURN FALSE;
        ENDIF
        motion_program_state{task_ind}.current_cmd_num:=cmd_num;
        TEST switch
        CASE 1:
            CirPathMode\PathFrame;
        CASE 2:
            CirPathMode\ObjectFrame;
        CASE 3:
            CirPathMode\CirPointOri;
        CASE 4:
            CirPathMode\Wrist45;
        CASE 5:
            CirPathMode\Wrist46;
        CASE 6:
            CirPathMode\Wrist56;
        ENDTEST
        RETURN TRUE;
    ENDFUNC

    FUNC bool try_motion_program_read_jt(INOUT jointtarget j)
        IF NOT (
        try_motion_program_read_num(j.robax.rax_1)
        AND try_motion_program_read_num(j.robax.rax_2)
        AND try_motion_program_read_num(j.robax.rax_3)
        AND try_motion_program_read_num(j.robax.rax_4)
        AND try_motion_program_read_num(j.robax.rax_5)
        AND try_motion_program_read_num(j.robax.rax_6)
        AND try_motion_program_read_num(j.extax.eax_a)
        AND try_motion_program_read_num(j.extax.eax_b)
        AND try_motion_program_read_num(j.extax.eax_c)
        AND try_motion_program_read_num(j.extax.eax_d)
        AND try_motion_program_read_num(j.extax.eax_e)
        AND try_motion_program_read_num(j.extax.eax_f)
        ) THEN
            RETURN FALSE;
        ENDIF
        RETURN TRUE;
    ENDFUNC

    FUNC bool try_motion_program_read_sd(INOUT speeddata sd)
        IF NOT (
        try_motion_program_read_num(sd.v_tcp)
        AND try_motion_program_read_num(sd.v_ori)
        AND try_motion_program_read_num(sd.v_leax)
        AND try_motion_program_read_num(sd.v_reax)
        ) THEN
            RETURN FALSE;
        ENDIF
        RETURN TRUE;
    ENDFUNC

    FUNC bool try_motion_program_read_zd(INOUT zonedata zd)
        VAR num finep_num:=0;
        IF NOT (
        try_motion_program_read_num(finep_num)
        AND try_motion_program_read_num(zd.pzone_tcp)
        AND try_motion_program_read_num(zd.pzone_ori)
        AND try_motion_program_read_num(zd.pzone_eax)
        AND try_motion_program_read_num(zd.zone_ori)
        AND try_motion_program_read_num(zd.zone_leax)
        AND try_motion_program_read_num(zd.zone_reax)
        ) THEN
            RETURN FALSE;
        ENDIF
        zd.finep:=finep_num<>0;
        RETURN TRUE;
    ENDFUNC

    FUNC bool try_motion_program_read_rt(INOUT robtarget rt)
        IF NOT (
            try_motion_program_read_num(rt.trans.x)
            AND try_motion_program_read_num(rt.trans.y)
            AND try_motion_program_read_num(rt.trans.z)
            AND try_motion_program_read_num(rt.rot.q1)
            AND try_motion_program_read_num(rt.rot.q2)
            AND try_motion_program_read_num(rt.rot.q3)
            AND try_motion_program_read_num(rt.rot.q4)
            AND try_motion_program_read_num(rt.robconf.cf1)
            AND try_motion_program_read_num(rt.robconf.cf4)
            AND try_motion_program_read_num(rt.robconf.cf6)
            AND try_motion_program_read_num(rt.robconf.cfx)
            AND try_motion_program_read_num(rt.extax.eax_a)
            AND try_motion_program_read_num(rt.extax.eax_b)
            AND try_motion_program_read_num(rt.extax.eax_c)
            AND try_motion_program_read_num(rt.extax.eax_d)
            AND try_motion_program_read_num(rt.extax.eax_e)
            AND try_motion_program_read_num(rt.extax.eax_f)
        ) THEN
            RETURN FALSE;
        ENDIF
        RETURN TRUE;
    ENDFUNC

    FUNC bool try_motion_program_read_td(INOUT tooldata td)
        VAR num robhold_num;
        IF NOT (
            try_motion_program_read_num(robhold_num)
            AND try_motion_program_read_num(td.tframe.trans.x)
            AND try_motion_program_read_num(td.tframe.trans.y)
            AND try_motion_program_read_num(td.tframe.trans.z)
            AND try_motion_program_read_num(td.tframe.rot.q1)
            AND try_motion_program_read_num(td.tframe.rot.q2)
            AND try_motion_program_read_num(td.tframe.rot.q3)
            AND try_motion_program_read_num(td.tframe.rot.q4)
            AND try_motion_program_read_num(td.tload.mass)
            AND try_motion_program_read_num(td.tload.cog.x)
            AND try_motion_program_read_num(td.tload.cog.y)
            AND try_motion_program_read_num(td.tload.cog.z)
            AND try_motion_program_read_num(td.tload.aom.q1)
            AND try_motion_program_read_num(td.tload.aom.q2)
            AND try_motion_program_read_num(td.tload.aom.q3)
            AND try_motion_program_read_num(td.tload.aom.q4)
            AND try_motion_program_read_num(td.tload.ix)
            AND try_motion_program_read_num(td.tload.iy)
            AND try_motion_program_read_num(td.tload.iz)
        )
        THEN
            RETURN FALSE;
        ENDIF
        td.robhold:=robhold_num<>0;
        RETURN TRUE;
    ENDFUNC
    
    FUNC bool try_motion_program_read_wd(INOUT wobjdata wd)
        VAR num robhold_num;
        VAR num ufprog_num;
        IF NOT (
            try_motion_program_read_num(robhold_num)
            AND try_motion_program_read_num(ufprog_num)
            AND try_motion_program_read_string(wd.ufmec)
            AND try_motion_program_read_num(wd.uframe.trans.x)
            AND try_motion_program_read_num(wd.uframe.trans.y)
            AND try_motion_program_read_num(wd.uframe.trans.z)
            AND try_motion_program_read_num(wd.uframe.rot.q1)
            AND try_motion_program_read_num(wd.uframe.rot.q2)
            AND try_motion_program_read_num(wd.uframe.rot.q3)
            AND try_motion_program_read_num(wd.uframe.rot.q4)
            AND try_motion_program_read_num(wd.oframe.trans.x)
            AND try_motion_program_read_num(wd.oframe.trans.y)
            AND try_motion_program_read_num(wd.oframe.trans.z)
            AND try_motion_program_read_num(wd.oframe.rot.q1)
            AND try_motion_program_read_num(wd.oframe.rot.q2)
            AND try_motion_program_read_num(wd.oframe.rot.q3)
            AND try_motion_program_read_num(wd.oframe.rot.q4)
        )
        THEN
            RETURN FALSE;
        ENDIF
        wd.robhold:=robhold_num<>0;
        wd.ufprog:=ufprog_num<>0;
        RETURN TRUE;
    ENDFUNC

    FUNC bool try_motion_program_fill_bytes()
        IF RawBytesLen(motion_program_bytes)=0 OR motion_program_bytes_offset>RawBytesLen(motion_program_bytes) THEN
            ClearRawBytes motion_program_bytes;
            motion_program_bytes_offset:=1;
            ReadRawBytes motion_program_io_device,motion_program_bytes;
            IF RawBytesLen(motion_program_bytes)=0 THEN
                RETURN FALSE;
            ENDIF
        ENDIF
        RETURN TRUE;
    ERROR
        IF ERRNO=ERR_RANYBIN_EOF THEN
            SkipWarn;
            TRYNEXT;
        ENDIF
    ENDFUNC

    FUNC bool try_motion_program_read_num(INOUT num val)
        IF NOT try_motion_program_fill_bytes() THEN
            val:=0;
            RETURN FALSE;
        ENDIF
        UnpackRawBytes motion_program_bytes,motion_program_bytes_offset,val,\Float4;
        motion_program_bytes_offset:=motion_program_bytes_offset+4;
        RETURN TRUE;
    ENDFUNC

    FUNC bool try_motion_program_read_string(INOUT string val)
        VAR num str_len;
        IF NOT (
            try_motion_program_fill_bytes()
            AND try_motion_program_read_num(str_len)
        )
        THEN
            val:="";
            RETURN FALSE;
        ENDIF
        IF RawBytesLen(motion_program_bytes)<motion_program_bytes_offset+str_len THEN
            RAISE ERR_WRONGVAL;
        ENDIF
        UnpackRawBytes motion_program_bytes,motion_program_bytes_offset,val,\ASCII:=str_len;
        motion_program_bytes_offset:=motion_program_bytes_offset+str_len;
        RETURN TRUE;
    ENDFUNC

    PROC motion_program_req_log_start()
        VAR string msg;
        msg:=motion_program_state{task_ind}.program_timestamp;
        RMQSendMessage logger_rmq,msg;
    ENDPROC

    PROC motion_program_req_log_end()
        VAR string msg:="";
        RMQSendMessage logger_rmq,msg;
    ENDPROC

    TRAP motion_trigg_trap
        VAR num cmd_ind;
        VAR num local_cmd_ind;
        VAR num cmd_num:=-1;
        WHILE cmd_num=-1 AND (NOT motion_current_cmd_ind>motion_max_cmd_ind) DO
            motion_current_cmd_ind:=motion_current_cmd_ind+1;
            IF motion_current_cmd_ind>motion_max_cmd_ind THEN
                cmd_ind:=motion_max_cmd_ind;
            ELSE
                cmd_ind:=motion_current_cmd_ind;
            ENDIF
            local_cmd_ind:=((cmd_ind-1) MOD 128)+1;
            cmd_num:=motion_cmd_num_history{local_cmd_ind};
        ENDWHILE

        IF cmd_num<>-1 THEN
            motion_program_state{task_ind}.current_cmd_num:=cmd_num;
        ENDIF
    ENDTRAP

ENDMODULE