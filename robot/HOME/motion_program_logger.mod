MODULE motion_program_logger
            
    CONST num motion_program_file_version:=10006;
    
    PERS motion_program_state_type motion_program_state;
        
    VAR bool log_file_open:=FALSE;
    VAR iodev log_io_device;
    VAR clock time_stamp_clock;
    
    VAR intnum rmqint_open;
    VAR string rmq_timestamp;
    
    VAR intnum logger_err_interrupt;
    
    PROC main()
        
        VAR num clk_now;
        VAR num c:=0;
        VAR num loop_count:=0;
        VAR num clk_diff;
        
        CONNECT rmqint_open WITH rmq_message_string;
        IRMQMessage rmq_timestamp, rmqint_open;
        
        CONNECT logger_err_interrupt WITH err_handler;
        IError COMMON_ERR, TYPE_ERR, logger_err_interrupt;
       
        ClkReset time_stamp_clock;
        ClkStart time_stamp_clock;
        WHILE TRUE DO
            clk_now:=ClkRead(time_stamp_clock \HighRes);
            motion_program_state.clk_time:=clk_now;
            motion_program_state.joint_position:=CJointT(\TaskRef:=T_ROB1Id);
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
    
    PROC motion_program_log_open()
        VAR string log_filename;
        VAR string header_str:="timestamp,cmdnum,J1,J2,J3,J4,J5,J6";
        VAR num header_str_len;
        VAR num rmq_timestamp_len;
        VAR rawbytes header_bytes;
        header_str_len:=StrLen(header_str);
        rmq_timestamp_len:=StrLen(rmq_timestamp);
        log_filename := "log-" + rmq_timestamp + ".bin";
        
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
        VAR num J1;    
        VAR num J2;        
        VAR num J3;    
        VAR num J4;    
        VAR num J5;    
        VAR num J6;
        VAR rawbytes data_bytes;
        GetJointData 1\Position:=J1;
        GetJointData 2\Position:=J2;
        GetJointData 3\Position:=J3;
        GetJointData 4\Position:=J4;
        GetJointData 5\Position:=J5;
        GetJointData 6\Position:=J6;
        pack_num ClkRead(time_stamp_clock \HighRes), data_bytes;
        pack_num motion_program_state.current_cmd_num, data_bytes;
        pack_num J1, data_bytes;
        pack_num J2, data_bytes;
        pack_num J3, data_bytes;
        pack_num J4, data_bytes;
        pack_num J5, data_bytes;
        pack_num J6, data_bytes;
        WriteRawBytes log_io_device, data_bytes;
    ENDPROC
    
    TRAP rmq_message_string
        VAR rmqmessage rmqmsg;
        IDisable;
        RMQGetMessage rmqmsg;
        RMQGetMsgData rmqmsg, rmq_timestamp;
        IF log_file_open THEN
            motion_program_log_close;
        ENDIF
        IF StrLen(rmq_timestamp) > 0 THEN
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