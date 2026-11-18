# robot/rapid — módulos RAPID (servidor de sockets)

## Qué es

Módulos RAPID para el controlador virtual **OmniCore C30, RobotWare 7.21.5, IRB 1100** en RobotStudio.

| Archivo | Módulo | Función | Estado |
|---|---|---|---|
| `comm.mod` | `ServidorVision` | Servidor **activo**: `HOME`, `MOVE_POSE`, `GRASP`, `GRIPPER` | probado por el usuario (según CLAUDE.md) |
| `test.mod` | `ServidorPython` | Prueba de conectividad: `PING`, `GET_POSE` | probado por el usuario (según CLAUDE.md) |
| `points.mod` | (sin `MODULE`) | `robtarget` obtenidos con jog: `pHome`, `pPlace1/2`, `cylinder`, `cube` | referencia; no carga tal cual (ver abajo) |

Las estaciones están en `RobotStudio/` (raíz del repo): `NeuralGrasp.rsstn`, `NeuralTest.rsstn`.

## Requisitos

- RobotStudio con el controlador virtual de la estación en marcha.
- Para `comm.mod`: la señal digital **`GripperOpen`** debe existir en la configuración de I/O del controlador. Si no existe, el módulo no carga.
- El puerto **55555** en `127.0.0.1` debe estar libre.

## Cómo ejecutar

No se pueden cargar desde la terminal. En RobotStudio:

1. **RAPID → T_ROB1 → Load Module…** y elige `comm.mod` (o `test.mod`, **nunca los dos**: ambos definen `main` y usan el mismo puerto).
2. **PP to Main** (mover el puntero de programa a `main`) y **Play**.
3. En el FlexPendant/TPWrite debe aparecer `Esperando pipeline de vision...` (o `Esperando conexion de Python...` con `test.mod`).
4. Ejecuta el cliente correspondiente (ver `experiments/01_robot_ping` y `experiments/02_robot_pick_place`).

Después de editar un `.mod` hay que recargarlo y volver a hacer **PP to Main**.

## Protocolo de `comm.mod`

- Servidor en `127.0.0.1:55555`. Al aceptar la conexión envía `Conectado a RAPID Vision`.
- Request/response síncrono, texto ASCII **sin delimitador**: un `sendall` → un `recv`. RAPID recibe en un `string` de **máx. 80 caracteres**.
- Unidades: mm y grados, en `wobj0`, con la herramienta `tVision` (TCP a z = 100 mm).

| Comando | Acción RAPID | Respuesta |
|---|---|---|
| `HOME` | `MoveJ pHome, v500, z50, tVision` | `OK` |
| `MOVE_POSE,x,y,z,rx,ry,rz` | clona `CRobT`; `rot := OrientZYX(rz, ry, rx)`; `MoveL v100 fine` | `OK` / `ERR_FORMATO` |
| `GRASP,x,y,z,?,ang` | `rot := OrientZYX(ang, 0, 180)`; `MoveL` (4.º valor sin usar; **no clona `CRobT`**) | `OK` / `ERR_FORMATO` |
| `GRIPPER,1` / `GRIPPER,0` | `SetDO GripperOpen, 0` / `Set GripperOpen` | `OK` / `ERR_FORMATO` |
| otro `CMD,...` | — | `ERR_CMD_NO_RECONOCIDO` |
| texto sin coma ≠ `HOME` | — | `ERR_SINTAXIS` |

Cuando el cliente cierra el socket, el `ERROR` handler (`ERR_SOCK_CLOSED`) vuelve a `SocketAccept`.

Las convenciones de orientación y cuaterniones están en [docs/coordinate_frames.md](../../docs/coordinate_frames.md).

## Qué esperar

- Con `test.mod`: `PING` → `PONG`; `GET_POSE` → `POSE,x,y,z` (con `tool0`).
- Con `comm.mod`: el robot se mueve en la estación y cada comando responde `OK`.

## Limitaciones y problemas conocidos

Detalle completo en [docs/known_issues.md](../../docs/known_issues.md) (R1-R11). Lo principal:
- La pinza usa `GripperOpen`; `do_Gripper` está comentado (R1).
- `GRASP` no clona `CRobT`, así que `robconf` y `extax` no se heredan (R2). `MOVE_POSE` puede dar errores de configuración con cambios grandes de orientación (R3).
- `points.mod` no tiene `MODULE … ENDMODULE` (R8), y su cubo no coincide con el script de pick & place (R9).
- Ningún comando de `comm.mod` devuelve la pose completa (R10).

Ninguno de estos problemas se corrigió en la reorganización: el comportamiento es el mismo de antes.

## Siguiente paso

Añadir a `comm.mod` un `GET_POSE` que devuelva `x,y,z,q1..q4` de `tVision` (propuesta en `experiments/04_handeye_synthetic/README.md`), para grabar poses reales para la calibración mano-ojo.
