# Convenciones de marcos de referencia

Este documento fija las convenciones que usan `calibration/`, los experimentos y la documentación. Cada punto dice de dónde sale: si se **verificó en el código**, si sale **del manual de RAPID** (sin verificar en RobotStudio) o si es una **convención propia** de este proyecto.

## Unidades

| Magnitud | Unidad | Fuente |
|---|---|---|
| Posición | **mm** | `robtarget.trans` en RAPID; `robot/client/rapid_client.py` manda `MOVE_POSE` en mm |
| Ángulos en comandos | **grados** | `MOVE_POSE`/`GRASP` en `robot/rapid/comm.mod` (`OrientZYX` recibe grados) |
| Orientación interna | matriz 3x3 / cuaternión | `calibration/transforms.py` |

`calibration/` trabaja **siempre** en mm, grados y matrices homogéneas 4x4. Ojo: el código heredado de GR-ConvNet (`inference/grasp_generator.py`, `hardware/calibrate_camera.py`) trabaja en **metros** (ver [known_issues.md](known_issues.md)).

## Notación `T_a_b`

`T_a_b` es la **pose del marco `b` expresada en el marco `a`**. Transforma puntos de `b` a `a`:

```
p_a = T_a_b @ p_b          T_a_c = T_a_b @ T_b_c          T_b_a = inv(T_a_b)
```

## Marcos

| Marco | Qué es | Definición | Fuente |
|---|---|---|---|
| `base` | Marco del robot | `wobj0` (= base del IRB 1100): x hacia el frente, z hacia arriba | `comm.mod` usa `\WObj:=wobj0` en todos los movimientos (verificado en el código) |
| `gripper` | TCP de la herramienta | `tVision`: TCP a **z = 100 mm** de la brida, sin rotación (`[1,0,0,0]`) | `comm.mod:7` `PERS tooldata tVision := [TRUE,[[0,0,100],[1,0,0,0]],...]` (verificado en el código) |
| `flange` | Brida (tool0) | Marco de la brida | `test.mod:40` (`GET_POSE` de la prueba) devuelve la pose de `tool0`. El `GET_POSE` de `comm.mod` devuelve la de **`tVision`** (`gripper`) |
| `cam` | Marco óptico de la cámara | Convención OpenCV: **x a la derecha, y hacia abajo, z hacia adelante** (eje óptico) | Convención de OpenCV (`solvePnP`, `estimatePoseCharucoBoard`) |
| `target` | Tablero ChArUco | Convención OpenCV: origen en una esquina, x/y en el plano del tablero, **z normal al tablero, apuntando hacia atrás de la cara impresa** | Convención de OpenCV; propia en `calibration/synthetic.py` |

**Montaje del tablero (convención propia, escena sintética):** el tablero va a 50 mm sobre el eje z de la herramienta, con la cara impresa mirando en +z de la herramienta. Por eso `T_gripper_target = [Rx(180°), (0, 0, 50)]`: el z del tablero es −z de la herramienta. Cuando exista el montaje físico hay que medir `T_gripper_target` de verdad. La calibración no lo necesita, pero el residuo de consistencia lo estima.

**Calibración eye-to-hand:** la cámara es fija y el tablero va en el gripper. Las incógnitas son `T_base_cam` (lo que se busca) y `T_gripper_target` (constante). En cada pose `i` se cumple:

```
T_gripper_target = inv(T_base_gripper_i) @ T_base_cam @ T_cam_target_i     (igual para toda i)
```

## Cuaterniones: RAPID vs. scipy

| | Orden | Ejemplo: 180° alrededor de y |
|---|---|---|
| **RAPID** (`orient`, `robtarget.rot`) | `[q1, q2, q3, q4] = [w, x, y, z]`, escalar **primero** | `[0, 0, 1, 0]` |
| **scipy** (`Rotation.as_quat()` por defecto) | `[x, y, z, w]`, escalar **al final** | `[0, 1, 0, 0]` |

Las conversiones están en `calibration/transforms.py` (`quat_rapid_to_R`, `R_to_quat_rapid`, `pose_rapid_to_T`, `T_to_pose_rapid`). `R_to_quat_rapid` devuelve siempre `w >= 0`; `q` y `-q` son la misma rotación. Se comprueba en `tests/test_transforms.py` (`[0,0,1,0]` ↔ Ry(180°) y el cuaternión de `pHome` de `comm.mod`).

## Orientación en `MOVE_POSE` y `GRASP`

Verificado en el código (`robot/rapid/comm.mod`):

```rapid
! MOVE_POSE,x,y,z,rx,ry,rz   (línea 59)
p_destino.rot := OrientZYX(args{6}, args{5}, args{4});      ! = OrientZYX(rz, ry, rx)

! GRASP,x,y,z,?,ang           (línea 69)
p_destino.rot := OrientZYX(args{5}, 0, 180);                ! = OrientZYX(ang, 0, 180)
```

Según el manual de referencia de RAPID, `OrientZYX(ZAngle, YAngle, XAngle)` arma la orientación como rotaciones **intrínsecas**: primero Z, luego Y' y luego X''. Es decir:

```
R = Rz(rz) @ Ry(ry) @ Rx(rx)          (scipy: Rotation.from_euler("ZYX", [rz, ry, rx], degrees=True))
```

Lo implementa `calibration.transforms.euler_rapid_to_R(rx, ry, rz)`, y su inversa `R_to_euler_rapid` equivale a `EulerZYX`. **Estado: sale del manual de RAPID y está probado en Python, pero sin verificar en RobotStudio.** Para confirmarlo de forma automática: `python experiments/04_handeye_synthetic/run.py --poses robotstudio` manda `MOVE_POSE` a varias poses, lee el cuaternión real con `GET_POSE` (TCP `tVision` en `wobj0`) y avisa si difiere de lo esperado en más de 0.1°. A mano: `MOVE_POSE,300,0,300,10,20,30`, luego `GET_POSE`, y comparar con `R_to_quat_rapid(euler_rapid_to_R(10, 20, 30))`.

Casos útiles:
- `rx=0, ry=180, rz=0` (lo que usa el pick & place) → `R = Ry(180°)` → cuaternión RAPID `[0, 0, 1, 0]`: herramienta boca abajo.
- `GRASP` con `ang`: `R = Rz(ang) @ Rx(180°)`. También es boca abajo, pero con `Rx(180)` en lugar de `Ry(180)`, así que **no da la misma orientación** que `MOVE_POSE` con `ry=180` para el mismo giro en z (difieren en 180° alrededor de z).

## Marco óptico de la cámara (OpenCV)

- x a la derecha de la imagen, y hacia abajo y z hacia adelante, saliendo del lente hacia la escena.
- Un punto 3D `(X, Y, Z)` en `cam` se proyecta en `u = fx·X/Z + cx`, `v = fy·Y/Z + cy`.
- `cv2.solvePnP` y la estimación de pose de ChArUco devuelven `T_cam_target` en este marco, con la unidad que se use para el tamaño de los cuadros. **Usar mm** para que encaje con el resto.
- La escena sintética coloca la cámara con `calibration.synthetic.look_at`, que construye un marco OpenCV válido: el "arriba" del mundo queda hacia −y de la imagen.
