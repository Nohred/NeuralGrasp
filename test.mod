MODULE ServidorPython
    VAR socketdev servidor;
    VAR socketdev cliente;

    ! 1. Procedimiento principal: Administra la vida del servidor
    PROC main()
        SocketCreate servidor;
        SocketBind servidor, "127.0.0.1", 55555;
        SocketListen servidor;

        WHILE TRUE DO
            TPWrite "Esperando conexion de Python...";
            SocketAccept servidor, cliente \Time:=WAIT_MAX;
            TPWrite "Cliente conectado!";
            
            SocketSend cliente \Str:="Conexion con ABB correcta";

            ! Cede el control a la rutina de atencion
            AtenderCliente;
        ENDWHILE

    ERROR
        ! Cierre de seguridad general en caso de fallo crítico
        SocketClose cliente;
        SocketClose servidor;
    ENDPROC

    ! 2. Procedimiento secundario: Atiende las peticiones del cliente activo
    PROC AtenderCliente()
        VAR string msg;
        VAR robtarget p;

        WHILE TRUE DO
            ! Lee el mensaje entrante
            SocketReceive cliente \Str:=msg \Time:=WAIT_MAX;

            IF msg = "PING" THEN
                SocketSend cliente \Str:="PONG";
            ELSEIF msg = "GET_POSE" THEN
                p := CRobT(\Tool:=tool0 \WObj:=wobj0);
                SocketSend cliente \Str:="POSE," + NumToStr(p.trans.x,2) + "," + NumToStr(p.trans.y,2) + "," + NumToStr(p.trans.z,2);
            ELSE
                SocketSend cliente \Str:="Comando desconocido";
            ENDIF
        ENDWHILE

    ERROR
        ! Captura cuando Python cierra la conexion de forma normal (s.close)
        IF ERRNO = ERR_SOCK_CLOSED THEN
            TPWrite "Python se desconecto. Liberando socket...";
            SocketClose cliente;
            ! Retorna al procedimiento main() para volver a esperar en SocketAccept
            RETURN;
        ENDIF
    ENDPROC
ENDMODULE