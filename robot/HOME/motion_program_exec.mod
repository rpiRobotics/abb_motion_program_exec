MODULE motion_program_exec
      
    PERS motion_program_state_type motion_program_state;
   
    LOCAL VAR iodev motion_program_io_device;
    LOCAL VAR rawbytes motion_program_bytes;
    LOCAL VAR num motion_program_bytes_offset;
    
    PERS tooldata motion_program_tool;
    
    PERS string motion_program_timestamp;
    VAR rmqslot logger_rmq;
                
    PROC main()
        VAR zonedata z := fine;
        motion_program_state.current_cmd_num:=-1;
        RMQFindSlot logger_rmq, "RMQ_logger";        
        run_motion_program_file("motion_program.bin");        
        ErrWrite \I, "Motion Program Complete", "Motion Program Complete";        
        
    ENDPROC
    
    PROC run_motion_program_file(string filename)
        ErrWrite \I, "Motion Program Begin", "Motion Program Begin";
        close_motion_program_file;
        open_motion_program_file(filename);
        motion_program_req_log_start;
        ErrWrite \I, "Motion Program Start Program", "Motion Program Start Program timestamp: " + motion_program_timestamp;
        motion_program_run;
        close_motion_program_file;
        motion_program_req_log_end;
    ENDPROC
    
    PROC open_motion_program_file(string filename)
        VAR num ver;
        VAR tooldata mtool;
        VAR string timestamp;
        motion_program_state.motion_program_filename:=filename;
        Open "TEMP:" \File:=filename, motion_program_io_device, \Read \Bin;
        IF NOT try_motion_program_read_num(ver) THEN
            RAISE ERR_FILESIZE;
        ENDIF
        
        IF ver <> 10003 THEN
            RAISE ERR_WRONGVAL;
        ENDIF
        
        IF NOT try_motion_program_read_td(mtool) THEN
            RAISE ERR_FILESIZE;
        ENDIF
        
        IF NOT try_motion_program_read_string(timestamp) THEN
            RAISE ERR_FILESIZE;
        ENDIF
        
        motion_program_timestamp := timestamp;
        
        ErrWrite \I, "Motion Program Opened", "Motion Program Opened with timestamp: " + timestamp;
        
        motion_program_tool:=mtool;
    ENDPROC
    
    PROC close_motion_program_file()
        motion_program_state.motion_program_filename:="";
        Close motion_program_io_device;
    ERROR
        SkipWarn;
        TRYNEXT;
    ENDPROC
    
    PROC motion_program_run()
        
        VAR bool keepgoing := TRUE;
        VAR num cmd_num;
        VAR num cmd_op;
        motion_program_state.current_cmd_num:=-1;
        motion_program_state.running:=TRUE;
        WHILE keepgoing DO
            keepgoing := try_motion_program_run_next_cmd(cmd_num, cmd_op);
        ENDWHILE
        motion_program_state.running:=FALSE;
    ERROR
        motion_program_state.running:=FALSE;
        RAISE;
    ENDPROC
            
    FUNC bool try_motion_program_run_next_cmd(INOUT num cmd_num, INOUT num cmd_op)
        
        IF NOT (try_motion_program_read_num(cmd_num) AND try_motion_program_read_num(cmd_op)) THEN
            RETURN FALSE;
        ENDIF
        
        motion_program_state.current_cmd_num:=cmd_num;
        
        TEST cmd_op
        CASE 0:
            RETURN TRUE;
        CASE 1:
            RETURN try_motion_program_run_moveabsj();
        CASE 2:
            RETURN try_motion_program_run_movej();
        CASE 3:
            RETURN try_motion_program_run_movel();
        CASE 4:
            RETURN try_motion_program_run_movec();
        CASE 5:
            RETURN try_motion_program_wait();
        DEFAULT:
            RAISE ERR_WRONGVAL;
        ENDTEST
    ENDFUNC
    
    FUNC bool try_motion_program_run_moveabsj()
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
        MoveAbsJ j, sd, zd, motion_program_tool;
        RETURN TRUE;   
    ENDFUNC
    
    FUNC bool try_motion_program_run_movej()
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
        MoveJ rt, sd, zd, motion_program_tool;
        RETURN TRUE;
        
    ENDFUNC
    
    FUNC bool try_motion_program_run_movel()
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
        MoveL rt, sd, zd, motion_program_tool;
        RETURN TRUE;
        
    ENDFUNC
    
    FUNC bool try_motion_program_run_movec()
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
        MoveC rt1, rt2, sd, zd, motion_program_tool;
        RETURN TRUE;
        
    ENDFUNC
    
    FUNC bool try_motion_program_wait()
        VAR num t;
        IF NOT try_motion_program_read_num(t) THEN
            RETURN FALSE;
        ENDIF
        WaitTime t;
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
        VAR num finep_num := 0;
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
        zd.finep := finep_num <> 0;
        RETURN TRUE;
    ENDFUNC
    
    FUNC bool try_motion_program_read_rt(INOUT robtarget rt)
        IF NOT(
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
        td.robhold := robhold_num <> 0;
        RETURN TRUE;
    ENDFUNC
    
    FUNC bool try_motion_program_fill_bytes()
        IF RawBytesLen(motion_program_bytes) = 0 OR motion_program_bytes_offset > RawBytesLen(motion_program_bytes) THEN
            ClearRawBytes motion_program_bytes;
            motion_program_bytes_offset:=1;
            ReadRawBytes motion_program_io_device, motion_program_bytes;
            IF RawBytesLen(motion_program_bytes) = 0 THEN                
                RETURN FALSE;
            ENDIF
        ENDIF
        RETURN TRUE;
    ERROR
        IF ERRNO = ERR_RANYBIN_EOF THEN
            SkipWarn;
            TRYNEXT;
        ENDIF
    ENDFUNC
    
    FUNC bool try_motion_program_read_num(INOUT num val)
        IF NOT try_motion_program_fill_bytes() THEN
            val:= 0;
            RETURN FALSE;
        ENDIF
        UnpackRawBytes motion_program_bytes, motion_program_bytes_offset, val, \Float4;
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
            val:= "";
            RETURN FALSE;
        ENDIF
        IF RawBytesLen(motion_program_bytes) < motion_program_bytes_offset + str_len THEN
            RAISE ERR_WRONGVAL;
        ENDIF
        UnpackRawBytes motion_program_bytes, motion_program_bytes_offset, val, \ASCII:=str_len;
        motion_program_bytes_offset:=motion_program_bytes_offset+str_len;
        RETURN TRUE;
    ENDFUNC
    
    PROC motion_program_req_log_start()
        RMQSendMessage logger_rmq, motion_program_timestamp;
    ENDPROC
    
    PROC motion_program_req_log_end()
        VAR string msg := "";
        RMQSendMessage logger_rmq, msg;
    ENDPROC

ENDMODULE