# robot/client — cliente Python de sockets para RAPID

## Qué es

`rapid_client.py` contiene la clase `RobotVisionController`, que habla con `robot/rapid/comm.mod`. Es la clase que antes vivía en `robot_comms.py` (raíz), sin cambios, más el método nuevo `get_pose()`. Las secuencias de pick & place se separaron a `experiments/02_robot_pick_place/run.py`.

| Método | Comando enviado | Devuelve |
|---|---|---|
| `connect()` | — (lee el saludo) | `True`/`False` |
| `send_command(cmd)` | `cmd` tal cual | respuesta en texto (`"ERR"` si no hay conexión) |
| `go_home()` | `HOME` | `True` si `OK` |
| `move_pose_euler(x,y,z,rx,ry,rz)` | `MOVE_POSE,x,y,z,rx,ry,rz` (2 decimales) | `True` si `OK` |
| `set_gripper(close)` | `GRIPPER,1` / `GRIPPER,0` | `True` si `OK` |
| `get_pose()` | `GET_POSE` | `(x, y, z, q1, q2, q3, q4)` (mm, cuaternión RAPID) o `None`; para 4x4: `calibration.pose_rapid_to_T(*pose)` |
| `disconnect()` | cierra el socket | — |

No hay método para `GRASP` (ver `docs/known_issues.md`, R6).

## Requisitos

- Python 3 (solo librería estándar).
- Para mover el robot: RobotStudio con `comm.mod` en ejecución (ver `robot/rapid/README.md`).

## Cómo ejecutar

Uso como librería, desde la raíz del repo:

```python
from robot.client import RobotVisionController

robot = RobotVisionController(host="127.0.0.1", port=55555, timeout=15.0)
if robot.connect():
    robot.go_home()
    robot.move_pose_euler(300, 0, 200, 0, 180, 0)
    robot.disconnect()
```

Pruebas sin RobotStudio, con un servidor falso que imita `comm.mod`:

```powershell
conda activate neuralgrasp
python -m pytest tests/test_rapid_client.py -v
```

## Qué esperar

- **Con RobotStudio:** `[Robot] Conectado: Conectado a RAPID Vision` y los movimientos en la estación (**sin verificar** tras la reorganización; el código de la clase no cambió).
- **Tests con el servidor falso:** pasan (**ejecutado por Claude Code**). Comprueban el formato exacto de los mensajes, las respuestas y que la secuencia de pick & place cabe en 80 caracteres por mensaje.

## Limitaciones y problemas conocidos

- Sin delimitador de mensajes: no mandes varios comandos sin leer la respuesta de cada uno (R7).
- `connect()` atrapa cualquier excepción e imprime el error; `send_command` no tiene manejo de errores.
- El servidor falso (`tests/fake_rapid_server.py`) imita el **formato** de las respuestas, no el movimiento ni los errores de RAPID (alcance, configuración).

## Siguiente paso

Verificar `get_pose()` contra RobotStudio (está **sin verificar**; solo se probó contra el servidor falso). Cuando se haga la integración, añadir un método `grasp(...)`.
