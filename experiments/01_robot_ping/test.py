import socket

# 1. Configuracion de la IP y Puerto (debe coincidir con RAPID)
HOST = "127.0.0.1"
PORT = 55555

try:
    # 2. Crear el socket y conectar
    print("Conectando a RobotStudio...")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    
    # 3. Recibir el saludo del robot
    saludo_bytes = s.recv(1024)
    print("Robot dice:", saludo_bytes.decode())
    
    # 4. Enviar comando PING
    print("\nEnviando comando: PING")
    s.sendall(b"PING") 
    # Se envia en bytes el comando PING, sendall garantiza que se envien todos los bytes
    respuesta = s.recv(1024).decode()
    print("Respuesta:", respuesta)
    
    # 5. Enviar comando GET_POSE
    print("\nEnviando comando: GET_POSE")
    s.sendall(b"GET_POSE")
    respuesta = s.recv(1024).decode()
    print("Respuesta:", respuesta)

except ConnectionRefusedError:
    print("Error: No se pudo conectar. ¿Esta corriendo el programa en RobotStudio?")
finally:
    # 6. Cerrar conexion
    print("\nCerrando conexion...")
    s.close()