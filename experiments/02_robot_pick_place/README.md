# 02 — Pick & place de cubo y cilindro (RobotStudio)

## Qué es

Secuencia de pick & place en la estación de RobotStudio, con coordenadas obtenidas con jog. `run.py` es la parte de "secuencias" del antiguo `robot_comms.py`: mismas coordenadas, mismo orden y mismos `z_offset`. El cliente de sockets se importa de `robot/client/rapid_client.py`.

Por cada objeto: approach (z + 80 mm) → pick → `GRIPPER,1` → subir → approach place → place → `GRIPPER,0` → retirarse. Todo con `rx=0, ry=180, rz=0` (herramienta boca abajo). Antes de empezar abre la pinza y va a `HOME`; también vuelve a `HOME` entre el cubo y el cilindro.

| Objeto | Pick (mm) | Place (mm) |
|---|---|---|
| Cubo | 323.38, 58.88, 38.31 | -1.58, 274.70, 46.57 |
| Cilindro | 278.83, -23.65, 45.69 | 14.77, -279.44, 45.69 |

## Requisitos

- RobotStudio con la estación `RobotStudio/NeuralGrasp.rsstn` (o la que tenga los objetos) y **`robot/rapid/comm.mod`** cargado y en ejecución.
- La señal `GripperOpen` configurada en el controlador.
- Python 3 (solo librería estándar).

## Cómo ejecutar

Desde la raíz del repo (o desde cualquier carpeta; el script añade la raíz a `sys.path`):

```powershell
python experiments/02_robot_pick_place/run.py
```

## Qué esperar

En la consola, `[Robot] Conectado: Conectado a RAPID Vision`, luego los pasos 1-8 para `CUBO` y para `CILINDRO`, y al final `[Robot] Desconectado.`. En RobotStudio, el robot recoge y deja ambos objetos.

**Estado:** probado por el usuario (según CLAUDE.md, como `robot_comms.py`). **Sin verificar en RobotStudio** tras la separación. Sí se ejecutó (**Claude Code**) la secuencia del cubo contra el servidor falso (`tests/test_rapid_client.py::test_pick_and_place_sequence_messages`), que comprueba los mismos 8 mensajes.

## Limitaciones y problemas conocidos

- El script no revisa las respuestas: si un `MOVE_POSE` falla (`ERR_*`), la secuencia sigue.
- El cubo de `points.mod` (`z = 62.77`) no coincide con el de este script (`z = 38.31`) (R9).
- Errores de configuración posibles con `MoveL` (R3).

## Siguiente paso

Cuando se integre la red, reemplazar las coordenadas fijas por las que predice GR-ConvNet transformadas con `T_base_cam` (fuera de alcance aquí).
