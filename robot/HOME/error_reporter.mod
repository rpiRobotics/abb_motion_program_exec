MODULE error_reporter
    VAR intnum motion_program_err_interrupt;
    
    PERS motion_program_state_type motion_program_state;
            
    PROC main()
        CONNECT motion_program_err_interrupt WITH motion_program_trap_err;
        IError COMMON_ERR, TYPE_ERR, motion_program_err_interrupt;
        WHILE TRUE DO
            WaitTime 1;
        ENDWHILE
        
    ENDPROC
    
    TRAP motion_program_trap_err
        VAR trapdata err_data;
        VAR errdomain err_domain;    
        VAR num err_number;    
        VAR errtype err_type;
        VAR string titlestr;
        VAR string string1;
        VAR string string2;
        
        GetTrapData err_data;
        ReadErrData err_data, err_domain, err_number, err_type \Title:=titlestr \Str1:=string1 \Str2:=string2;

        ErrWrite \W, "Motion Program Failed", "Motion Program Failed at command number " + NumToStr(motion_program_state.current_cmd_num,0) 
            \RL2:= "with error code " + NumToStr(err_domain*10000+err_number,0)
            \RL3:= "error title '" + titlestr + "'"
            \RL4:= "error string '" + string1 + "'";
    ENDTRAP
ENDMODULE