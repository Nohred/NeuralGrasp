# robot/rapid — módulos RAPID (servidor de sockets)

## Qué es

Módulos RAPID para el controlador virtual **OmniCore C30, RobotWare 7.21.5, IRB 1100** en RobotStudio.

| Archivo | Módulo | Función | Estado |
|---|---|---|---|
| `comm.mod` | `ServidorVision` | Servidor **activo**: `HOME`, `GET_POSE`, `MOVE_POSE`, `GRASP`, `GRIPPER` | probado por el usuario (según CLAUDE.md); **`GET_POSE` sin verificar en RobotStudio** (añadido el 2026-11-17, probado solo contra el servidor falso) |
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
| `GET_POSE` | `CRobT(\Tool:=tVision \WObj:=wobj0)` (no mueve nada) | `POSE,x,y,z,q1,q2,q3,q4`: mm con 2 decimales, cuaternión RAPID `[w,x,y,z]` con 6 decimales; ≤ 71 caracteres |
| `MOVE_POSE,x,y,z,rx,ry,rz` | clona `CRobT`; `rot := OrientZYX(rz, ry, rx)`; `MoveL v100 fine` | `OK` / `ERR_FORMATO` |
| `GRASP,x,y,z,?,ang` | `rot := OrientZYX(ang, 0, 180)`; `MoveL` (4.º valor sin usar; **no clona `CRobT`**) | `OK` / `ERR_FORMATO` |
| `GRIPPER,1` / `GRIPPER,0` | `SetDO GripperOpen, 0` / `Set GripperOpen` | `OK` / `ERR_FORMATO` |
| otro `CMD,...` | — | `ERR_CMD_NO_RECONOCIDO` |
| texto sin coma ≠ `HOME` | — | `ERR_SINTAXIS` |

Cuando el cliente cierra el socket, el `ERROR` handler (`ERR_SOCK_CLOSED`) vuelve a `SocketAccept`.

Las convenciones de orientación y cuaterniones están en [docs/coordinate_frames.md](../../docs/coordinate_frames.md).

## Qué esperar

- Con `test.mod`: `PING` → `PONG`; `GET_POSE` → `POSE,x,y,z` (con `tool0`).
- Con `comm.mod`: el robot se mueve en la estación y cada comando responde `OK`. Recién cargado, en `pHome`, `GET_POSE` debería responder algo como `POSE,281.85,5.73,310.95,0.044739,-0.811529,0.582552,-0.007207`. `NumToStr` puede omitir ceros finales; el cliente lo acepta.

## Limitaciones y problemas conocidos

Detalle completo en [docs/known_issues.md](../../docs/known_issues.md) (R1-R11). Lo principal:
- La pinza usa `GripperOpen`; `do_Gripper` está comentado (R1).
- `GRASP` no clona `CRobT`, así que `robconf` y `extax` no se heredan (R2). `MOVE_POSE` puede dar errores de configuración con cambios grandes de orientación (R3).
- `points.mod` no tiene `MODULE … ENDMODULE` (R8). Su cubo no coincide con el del script; por decisión del usuario **vale la pose del script** y `points.mod` se ignora para el cubo (R9).

Ninguno de estos problemas se corrigió en la reorganización. El único cambio en `comm.mod` es el comando **nuevo** `GET_POSE`; los demás comandos se comportan igual que antes.

## Siguiente paso

1. Recargar `comm.mod` en RobotStudio (Load Module + PP to Main) y probar `GET_POSE`, por ejemplo con `python -c "from robot.client import RobotVisionController as R; r=R(); r.connect(); print(r.get_pose()); r.disconnect()"`.
2. Correr `python experiments/04_handeye_synthetic/run.py --poses robotstudio`, que además verifica la convención `OrientZYX` comparando la pose pedida con la leída.
