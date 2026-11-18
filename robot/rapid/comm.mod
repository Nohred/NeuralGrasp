MODULE ServidorVision
    VAR socketdev servidor;
    VAR socketdev cliente;
    VAR num args{10};
    VAR num pos_coma;
    
    PERS tooldata tVision := [TRUE,[[0,0,100],[1,0,0,0]],[1,[0,0,1],[1,0,0,0],0,0,0]];

    ! Posicion Home que extrajiste usando el Jog
    CONST robtarget pHome := [[281.85, 5.73, 310.95], [0.044738964, -0.811528945, 0.58255236, -0.007207268], [0, 0, 0, 0], [9E9, 9E9, 9E9, 9E9, 9E9, 9E9]];

    PROC main()
        SocketCreate servidor;
        SocketBind servidor, "127.0.0.1", 55555;
        SocketListen servidor;

        WHILE TRUE DO
            TPWrite "Esperando pipeline de vision...";
            SocketAccept servidor, cliente \Time:=WAIT_MAX;
            SocketSend cliente \Str:="Conectado a RAPID Vision";
            
            AtenderVision;
            Set GripperOpen;
            WaitTime 1;
        ENDWHILE
    ERROR
        SocketClose cliente;
        SocketClose servidor;
    ENDPROC

    PROC AtenderVision()
        VAR string msg;
        VAR string cmd;
        VAR string payload;
        VAR robtarget p_destino;
        VAR robtarget p_actual;

        WHILE TRUE DO
            SocketReceive cliente \Str:=msg \Time:=WAIT_MAX;
            pos_coma := StrFind(msg, 1, ",");
            
            ! ----------------- COMANDOS SIMPLES -----------------
            IF msg = "HOME" THEN
                MoveJ pHome, v500, z50, tVision \WObj:=wobj0;
                SocketSend cliente \Str:="OK";

            ! Pose actual del TCP (tVision) en wobj0, para calibracion mano-ojo.
            ! Respuesta: POSE,x,y,z,q1,q2,q3,q4  (mm con 2 decimales, cuaternion RAPID
            ! [w,x,y,z] con 6 decimales). Peor caso ~71 caracteres (< 80 de un string RAPID).
            ELSEIF msg = "GET_POSE" THEN
                p_actual := CRobT(\Tool:=tVision \WObj:=wobj0);
                SocketSend cliente \Str:="POSE," + NumToStr(p_actual.trans.x,2) + "," + NumToStr(p_actual.trans.y,2) + "," + NumToStr(p_actual.trans.z,2) + "," + NumToStr(p_actual.rot.q1,6) + "," + NumToStr(p_actual.rot.q2,6) + "," + NumToStr(p_actual.rot.q3,6) + "," + NumToStr(p_actual.rot.q4,6);

            ! ----------------- COMANDOS CON PARAMETROS -----------------
            ELSEIF pos_coma > 0 THEN
                cmd := StrPart(msg, 1, pos_coma - 1);
                payload := StrPart(msg, pos_coma + 1, StrLen(msg) - pos_coma);
                
                ! Clonar posicion actual (solo para comandos de movimiento)
                IF cmd = "MOVE_POSE" OR cmd = "MOVE" THEN
                    p_destino := CRobT(\Tool:=tVision \WObj:=wobj0);
                ENDIF

                IF cmd = "MOVE_POSE" THEN
                    IF ParseDatos(payload, 6) THEN
                        p_destino.trans := [args{1}, args{2}, args{3}];
                        p_destino.rot := OrientZYX(args{6}, args{5}, args{4});
                        MoveL p_destino, v100, fine, tVision \WObj:=wobj0;
                        SocketSend cliente \Str:="OK";
                    ELSE
                        SocketSend cliente \Str:="ERR_FORMATO";
                    ENDIF

                ELSEIF cmd = "GRASP" THEN
                    IF ParseDatos(payload, 5) THEN
                        p_destino.trans := [args{1}, args{2}, args{3}];
                        p_destino.rot := OrientZYX(args{5}, 0, 180);
                        MoveL p_destino, v100, fine, tVision \WObj:=wobj0;
                        SocketSend cliente \Str:="OK";
                    ELSE
                        SocketSend cliente \Str:="ERR_FORMATO";
                    ENDIF

                ELSEIF cmd = "GRIPPER" THEN
                    ! 1 = Cerrar/Attach, 0 = Abrir/Detach
                    IF ParseDatos(payload, 1) THEN
                        IF args{1} = 1 THEN
                            TPWrite "Garra CERRADA (Objeto Atrapado)";
                            WaitTime 0.5; 
                            !SetDO do_Gripper, 1; ! Descomentar cuando configures la senal
                            SetDO GripperOpen, 0;
                            WaitTime 1;
                        ELSE
                            TPWrite "Garra ABIERTA (Objeto Suelto)";
                            WaitTime 0.5;
                            !SetDO do_Gripper, 0; ! Descomentar cuando configures la senal
                            Set GripperOpen;
                            WaitTime 1;
                        ENDIF
                        SocketSend cliente \Str:="OK";
                    ELSE
                        SocketSend cliente \Str:="ERR_FORMATO";
                    ENDIF
                    
                ELSE
                    SocketSend cliente \Str:="ERR_CMD_NO_RECONOCIDO";
                ENDIF
            ELSE
                SocketSend cliente \Str:="ERR_SINTAXIS";
            ENDIF
        ENDWHILE
    ERROR
        IF ERRNO = ERR_SOCK_CLOSED THEN
            TPWrite "Desconexion del cliente.";
            SocketClose cliente;
            RETURN;
        ENDIF
    ENDPROC

    FUNC bool ParseDatos(string data_str, num num_esperados)
        VAR num start_idx := 1;
        VAR num comma_idx;
        VAR string token;
        VAR bool success;
        data_str := data_str + ",";
        FOR i FROM 1 TO num_esperados DO
            comma_idx := StrFind(data_str, start_idx, ",");
            IF comma_idx = 0 RETURN FALSE;
            token := StrPart(data_str, start_idx, comma_idx - start_idx);
            success := StrToVal(token, args{i});
            IF NOT success RETURN FALSE;
            start_idx := comma_idx + 1;
        ENDFOR
        RETURN TRUE;
    ENDFUNC
ENDMODULE