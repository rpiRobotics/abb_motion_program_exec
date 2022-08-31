MODULE motion_program_logger
        
    CONST num log_sample_period:=0.004;
    PERS motion_program_state_type motion_program_state{2};
        
    VAR bool log_file_open:=FALSE;
    VAR iodev log_io_device;
    VAR clock time_stamp_clock;
    
    VAR intnum rmqint_open;
    VAR string rmq_timestamp;
    
    VAR intnum logger_err_interrupt;
    
    VAR num logger_ind;
    
    PROC main()
        
        VAR num clk_now;
        VAR num c:=0;
        VAR num loop_count:=0;
        VAR num clk_diff;
        
        CONNECT rmqint_open WITH rmq_message_string;
        IRMQMessage rmq_timestamp, rmqint_open;
        
        CONNECT logger_err_interrupt WITH err_handler;
        !IError COMMON_ERR, TYPE_ERR, logger_err_interrupt;
       
        ClkReset time_stamp_clock;
        ClkStart time_stamp_clock;
        WHILE TRUE DO
            clk_now:=ClkRead(time_stamp_clock \HighRes);
            motion_program_state{1}.clk_time:=clk_now;
            motion_program_state{2}.clk_time:=clk_now;
            motion_program_state{1}.joint_position:=CJointT(\TaskRef:=T_ROB1Id);
            motion_program_state{2}.joint_position:=CJointT(\TaskRef:=T_ROB2Id);
            
            IDisable;
            IF motion_program_executing <> 0 THEN
                IF log_file_open THEN
                    motion_program_log_data;
                ENDIF
            ENDIF
            IEnable;
            
            IF log_file_open THEN               
                clk_diff:= loop_count*log_sample_period - clk_now;
                loop_count := loop_count+1;
                IF clk_diff > 0 THEN
                    WaitTime clk_diff;
                ENDIF
            ELSE
                loop_count := 0;
                ClkStop time_stamp_clock;
                ClkReset time_stamp_clock;
                WaitTime log_sample_period;
                ClkStart time_stamp_clock;
            ENDIF
           
        ENDWHILE
    ENDPROC
    
    PROC motion_program_log_open()
        VAR string log_filename;
        log_filename := "log-" + rmq_timestamp + ".csv";
        Open "RAMDISK:" \File:=log_filename, log_io_device, \Write;
        Write log_io_device,"timestamp, "\NoNewLine;
        Write log_io_device,"cmd_num, "\NoNewLine;
        Write log_io_device,"J1, "\NoNewLine;
        Write log_io_device,"J2, "\NoNewLine;
        Write log_io_device,"J3, "\NoNewLine;
        Write log_io_device,"J4, "\NoNewLine;
        Write log_io_device,"J5, "\NoNewLine;
        Write log_io_device,"J6, "\NoNewLine;
        Write log_io_device,"J1_2, "\NoNewLine;
        Write log_io_device,"J2_2, "\NoNewLine;
        Write log_io_device,"J3_2, "\NoNewLine;
        Write log_io_device,"J4_2, "\NoNewLine;
        Write log_io_device,"J5_2, "\NoNewLine;
        Write log_io_device,"J6_2";
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
        VAR jointtarget jt1;
        VAR jointtarget jt2;
        jt1:=motion_program_state{1}.joint_position;
        jt2:=motion_program_state{2}.joint_position;        
        
        Write log_io_device, "" \Num:=ClkRead(time_stamp_clock \HighRes)\NoNewLine;
        Write log_io_device,", "\Num:=motion_program_state{1}.current_cmd_num\NoNewLine;
        Write log_io_device,", "\Num:=jt1.robax.rax_1\NoNewLine;
        Write log_io_device,", "\Num:=jt1.robax.rax_2\NoNewLine;
        Write log_io_device,", "\Num:=jt1.robax.rax_3\NoNewLine;
        Write log_io_device,", "\Num:=jt1.robax.rax_4\NoNewLine;
        Write log_io_device,", "\Num:=jt1.robax.rax_5\NoNewLine;
        Write log_io_device,", "\Num:=jt1.robax.rax_6\NoNewLine;
        Write log_io_device,", "\Num:=jt2.robax.rax_1\NoNewLine;
        Write log_io_device,", "\Num:=jt2.robax.rax_2\NoNewLine;
        Write log_io_device,", "\Num:=jt2.robax.rax_3\NoNewLine;
        Write log_io_device,", "\Num:=jt2.robax.rax_4\NoNewLine;
        Write log_io_device,", "\Num:=jt2.robax.rax_5\NoNewLine;
        Write log_io_device,", "\Num:=jt2.robax.rax_6;
    ENDPROC
    
    PROC logger2_main()
        
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