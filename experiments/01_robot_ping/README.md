# 01 — Ping al robot (prueba de conectividad)

## Qué es

Primera prueba de sockets entre Python y RAPID: conecta, lee el saludo, manda `PING` y `GET_POSE`, y cierra. `test.py` es el antiguo `test.py` de la raíz, **sin cambios**. El servidor es `robot/rapid/test.mod` (módulo `ServidorPython`).

## Requisitos

- RobotStudio con **`robot/rapid/test.mod`** cargado en T_ROB1 y en ejecución (y **no** `comm.mod`).
- Python 3 (solo librería estándar).

## Cómo ejecutar

```powershell
python experiments/01_robot_ping/test.py
```

## Qué esperar

```
Conectando a RobotStudio...
Robot dice: Conexion con ABB correcta

Enviando comando: PING
Respuesta: PONG

Enviando comando: GET_POSE
Respuesta: POSE,<x>,<y>,<z>
```

`GET_POSE` devuelve la posición de **`tool0`** (brida), no de `tVision`, y sin orientación.

**Estado:** probado por el usuario (según CLAUDE.md). Sin verificar tras moverlo de carpeta (el archivo no cambió).

## Limitaciones y problemas conocidos

- `test.mod` y `comm.mod` no pueden estar cargados a la vez (mismo `main`, mismo puerto).
- Ver `docs/known_issues.md`: R10 (`GET_POSE` incompleto) y R12 (`NameError` en el `finally` si falla la creación del socket).

## Siguiente paso

Reemplazar esta prueba por un `GET_POSE` completo en `comm.mod`, útil para grabar poses de calibración.
