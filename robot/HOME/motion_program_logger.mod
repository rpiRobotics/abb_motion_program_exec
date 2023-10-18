MODULE motion_program_logger
            
    LOCAL VAR bool log_file_open:=FALSE;
    LOCAL VAR iodev log_io_device;
    LOCAL VAR clock time_stamp_clock;
    
    LOCAL VAR intnum rmqint_open;
    LOCAL VAR string rmq_timestamp;
    LOCAL VAR string rmq_filename;
    LOCAL VAR string rmq_req{2};
    
    LOCAL VAR intnum logger_err_interrupt;
    LOCAL VAR num robot_count:=0;
        
    
    PROC motion_program_logger_main()
        
        VAR num clk_now;
        VAR num c:=0;
        VAR num loop_count:=0;
        VAR num clk_diff;
        
        VAR num mechunit_listnum:=0;
        VAR string mechunit_name:="";
                
        CONNECT rmqint_open WITH rmq_message_string;
        IRMQMessage rmq_req, rmqint_open;
        
        CONNECT logger_err_interrupt WITH err_handler;
        IError COMMON_ERR, TYPE_ERR, logger_err_interrupt;
        
        WHILE GetNextMechUnit(mechunit_listnum, mechunit_name) DO
            robot_count:=robot_count+1;
        ENDWHILE
       
        ClkReset time_stamp_clock;
        ClkStart time_stamp_clock;
        WHILE TRUE DO
            clk_now:=ClkRead(time_stamp_clock \HighRes);
            motion_program_state{1}.clk_time:=clk_now;
            motion_program_state{1}.joint_position:=CJointT(\TaskRef:=T_ROB1Id);
            IF robot_count >= 2 THEN
                motion_program_state{2}.joint_position:=CJointT(\TaskName:="T_ROB2");    
            ENDIF
            IF log_file_open THEN               
                clk_diff:= loop_count*0.004 - clk_now;
                loop_count := loop_count+1;
                IF clk_diff > 0 THEN
                    WaitTime clk_diff;
                ENDIF
            ELSE
                loop_count := 0;
                ClkStop time_stamp_clock;
                ClkReset time_stamp_clock;
                WaitTime 0.004;
                ClkStart time_stamp_clock;
            ENDIF
            
            IDisable;
            IF motion_program_executing <> 0 THEN
                IF log_file_open THEN
                    motion_program_log_data;
                ENDIF
            ENDIF
            IEnable;
            
        ENDWHILE
    ENDPROC
    
    PROC pack_num(num val, VAR rawbytes b)
        PackRawBytes val, b, (RawBytesLen(b)+1)\Float4;
    ENDPROC
    
    PROC pack_str(string val, VAR rawbytes b)
        PackRawBytes val, b, (RawBytesLen(b)+1)\ASCII;
    ENDPROC
    
    PROC pack_jointtarget(jointtarget jt, VAR rawbytes b)
        pack_num jt.robax.rax_1, b;
        pack_num jt.robax.rax_2, b;
        pack_num jt.robax.rax_3, b;
        pack_num jt.robax.rax_4, b;
        pack_num jt.robax.rax_5, b;
        pack_num jt.robax.rax_6, b;
    ENDPROC
    
    PROC motion_program_log_open()
        VAR string log_filename;
        VAR string header_str:="timestamp,cmdnum,J1,J2,J3,J4,J5,J6";
        VAR num header_str_len;
        VAR num rmq_timestamp_len;
        VAR rawbytes header_bytes;
        
        IF robot_count >= 2 THEN
            header_str:="timestamp,cmdnum,J1,J2,J3,J4,J5,J6,J1_2,J2_2,J3_2,J4_2,J5_2,J6_2";
        ENDIF
        
        header_str_len:=StrLen(header_str);
        rmq_timestamp_len:=StrLen(rmq_timestamp);
        log_filename := "log-" + rmq_filename + ".bin";
        
        pack_num motion_program_file_version, header_bytes;
        pack_num rmq_timestamp_len, header_bytes;
        pack_str rmq_timestamp, header_bytes;
        pack_num header_str_len, header_bytes;
        pack_str header_str, header_bytes;
        Open "RAMDISK:" \File:=log_filename, log_io_device, \Write\Bin;
        WriteRawBytes log_io_device, header_bytes;
        ErrWrite \I, "Motion Program Log File Opened", "Motion Program Log File Opened with filename: " + log_filename;        
        log_file_open := TRUE;
    ENDPROC
    
    PROC motion_program_log_close()
        Close log_io_device;
        log_file_open := FALSE;
        ErrWrite \I, "Motion Program Log File Closed", "Motion Program Log File Closed";
    ERROR
        SkipWarn;
        TRYNEXT;
    ENDPROC
    
    PROC motion_program_log_data()
        VAR rawbytes data_bytes;
        pack_num ClkRead(time_stamp_clock \HighRes), data_bytes;
        pack_num motion_program_state{1}.current_cmd_num, data_bytes;
        pack_jointtarget motion_program_state{1}.joint_position, data_bytes;
        IF robot_count >= 2 THEN
            pack_jointtarget motion_program_state{2}.joint_position, data_bytes;
        ENDIF
        WriteRawBytes log_io_device, data_bytes;
    ENDPROC
    
    TRAP rmq_message_string
        VAR rmqmessage rmqmsg;
        IDisable;
        RMQGetMessage rmqmsg;
        RMQGetMsgData rmqmsg, rmq_req;
        rmq_timestamp:=rmq_req{1};
        rmq_filename:=rmq_req{2};
        IF log_file_open THEN
            motion_program_log_close;
        ENDIF
        IF StrLen(rmq_timestamp) > 0 AND motion_program_log_motion > 0 THEN
            motion_program_log_open;
        ENDIF
        IEnable;
    ENDTRAP
    
    
    TRAP err_handler
                
        VAR trapdata err_data;
        VAR errdomain err_domain;    
        VAR num err_number;    
        VAR errtype err_type;
        IDisable;
        ISleep logger_err_interrupt;
        
        GetTrapData err_data;
        ReadErrData err_data, err_domain, err_number, err_type;
        IF log_file_open THEN
            motion_program_log_close;
        ENDIF
        
        IWatch logger_err_interrupt;
        IEnable;
        
    ENDTRAP
    
ENDMODULE