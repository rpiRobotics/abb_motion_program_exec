MODULE error_reporter
        
    VAR intnum motion_program_err_interrupt;
                
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
        VAR num seqno;
        VAR string err_filename;
        VAR iodev err_file;
                
        GetTrapData err_data;
        ReadErrData err_data, err_domain, err_number, err_type \Title:=titlestr \Str1:=string1 \Str2:=string2;

        ErrWrite \W, "Motion Program Failed", "Motion Program Failed at command number " + NumToStr(motion_program_state{1}.current_cmd_num,0) 
            \RL2:= "with error code " + NumToStr(err_domain*10000+err_number,0)
            \RL3:= "error title '" + titlestr + "'"
            \RL4:= "error string '" + string1 + "'";
        
        seqno := motion_program_seqno_started;
        IF seqno > 0 THEN
            err_filename := "motion_program_err---seqno-" + NumToStr(seqno,0) + ".json";
            Open "RAMDISK:"\File:=err_filename,err_file\Write;
            Write err_file, "{""seqno"": " + NumToStr(seqno,0)\NoNewLine;
            Write err_file, ",""error_domain"": " + NumToStr(err_domain,0)\NoNewLine;
            Write err_file, ",""error_number"": " + NumToStr(err_number,0)\NoNewLine;
            Write err_file, ",""error_title"": """ + titlestr + """"\NoNewLine; 
            Write err_file, ",""error_string"": """ + string1 + """"\NoNewLine; 
            Write err_file, ",""error_string2"": """ + string2 + """"\NoNewLine; 
            Write err_file, "}";
            Close err_file;
        ENDIF
            
        
    ENDTRAP
ENDMODULE