# 04 — Calibración mano-ojo eye-to-hand con datos sintéticos

## Qué es

Estudio de la calibración `T_base_cam` (cámara fija, tablero ChArUco en el gripper) con `calibration/` y los 5 métodos de `cv2.calibrateHandEye`, sin hardware. Las poses del robot y las detecciones del tablero se simulan con verdad de terreno conocida (escena descrita en `calibration/README.md`).

1. **Prueba sin ruido:** el error debe ser ~0. Tolerancia: **1e-6 mm y 1e-6°** (en la práctica sale ~1e-13 a 1e-10; el margen absorbe redondeo de float64 sin ocultar errores de convención, que dan cientos de mm).
2. **Monte Carlo** (200 repeticiones por condición; cada repetición tiene sus propias poses y ruido, compartidos entre métodos para compararlos en pareja):
   - (a) error vs. **nivel de ruido** de la detección, con 15 poses;
   - (b) error vs. **número de poses** (5, 10, 15, 20, 30), con el ruido nominal de 1 mm / 0.25°.
3. **Caso degenerado:** poses `varied` vs. `low_rotation` (inclinación y giro ≤ 3°) vs. `translation_only` (orientación idéntica en todas las poses).

### Fuentes de poses (`--poses`)

| Modo | Poses del robot | Detecciones | Estado |
|---|---|---|---|
| `synthetic` (por defecto) | muestreadas en la caja de trabajo, con semilla | simuladas | **ejecutado por Claude Code** |
| `csv` | `--csv archivo` con columnas `x,y,z,q1,q2,q3,q4` (mm, cuaternión RAPID `[w,x,y,z]`), p. ej. grabadas con jog en RobotStudio con **`tVision` activa** | simuladas con la cámara sintética; se descartan las poses con tablero no visible (salvo `--no-visibility-filter`) | **ejecutado por Claude Code** con `poses_example.csv` (poses sintéticas, no de RobotStudio) |
| `robotstudio` | — | — | **stub**: imprime qué hace falta y sale con código 2 |

**Qué haría falta para `robotstudio`:** `comm.mod` no tiene un comando que devuelva la pose (y no se modificó). `test.mod` tiene `GET_POSE`, pero con `tool0` y solo `x,y,z`. Se necesitaría en `comm.mod` algo como:

```rapid
ELSEIF msg = "GET_POSE" THEN
    p_actual := CRobT(\Tool:=tVision \WObj:=wobj0);
    SocketSend cliente \Str:="POSE," + NumToStr(p_actual.trans.x,2) + "," + NumToStr(p_actual.trans.y,2) + ","
        + NumToStr(p_actual.trans.z,2) + "," + NumToStr(p_actual.rot.q1,5) + "," + NumToStr(p_actual.rot.q2,5) + ","
        + NumToStr(p_actual.rot.q3,5) + "," + NumToStr(p_actual.rot.q4,5);
```

Ojo con el límite de 80 caracteres de un `string` RAPID: la respuesta ronda los 70-75. Si no cabe, hay que partirla en dos respuestas o usar `\RawData`. El modo también tendría que mover el robot (`MOVE_POSE`) por una lista de poses y leer `GET_POSE` en cada una.

## Requisitos

Entorno conda `neuralgrasp` (ver `requirements.txt` de la raíz). Ni RobotStudio, ni cámara, ni GPU.

## Cómo ejecutar

Desde la raíz del repo:

```powershell
conda activate neuralgrasp

# Estudio completo (200 reps, ~2.5 min)
python experiments/04_handeye_synthetic/run.py

# Variantes
python experiments/04_handeye_synthetic/run.py --n-reps 50 --methods tsai,park --seed 1
python experiments/04_handeye_synthetic/run.py --noise-trans-mm 0,1,3 --noise-rot-deg 0,0.2,0.6 --n-poses 20
python experiments/04_handeye_synthetic/run.py --reach 580
python experiments/04_handeye_synthetic/run.py --poses csv --csv experiments/04_handeye_synthetic/poses_example.csv
python experiments/04_handeye_synthetic/run.py --help
```

Salidas en `experiments/04_handeye_synthetic/results/` (ignorado por git):
- `raw.csv`: una fila por repetición y método.
- `summary.csv`: media, desviación, mediana y p95 por condición (barridos a y b).
- `degenerate.csv`: lo mismo, para el caso degenerado.
- `error_vs_noise.png` y `error_vs_nposes.png`: mediana (sólida) y p95 (punteada), en escala log.

## Qué esperar

**Ejecutado por Claude Code** (`--n-reps 200 --seed 0`, alcance 475 mm, OpenCV 5.1.0, ~130 s).

**[1] Sin ruido:** OK con los 5 métodos. Tsai, Park y Horaud dan ~1e-12 mm; Andreff y Daniilidis, ~1e-11 mm.

**[2a] Error vs. ruido** (15 poses; mediana [p95]):

| Ruido σt / σr | Tsai | Park | Horaud | Andreff | Daniilidis |
|---|---|---|---|---|---|
| 0.5 mm / 0.1° | 1.07 [2.1] mm, 0.08 [0.15]° | 1.06 [2.0] mm, 0.07 [0.14]° | 1.06 [2.0] mm, 0.07 [0.14]° | 1.77 [5.3] mm, 0.11 [0.21]° | 1.04 [2.0] mm, 0.11 [0.25]° |
| 1 mm / 0.25° | 2.53 [4.9] mm, 0.20 [0.41]° | 2.52 [5.0] mm, 0.18 [0.40]° | 2.50 [5.0] mm, 0.18 [0.40]° | 5.0 [12.3] mm, 0.26 [0.49]° | 2.25 [4.7] mm, 0.25 [0.60]° |
| 2 mm / 0.5° | 4.9 [9.8] mm, 0.38 [0.84]° | 4.7 [9.6] mm, 0.34 [0.75]° | 4.7 [9.6] mm, 0.33 [0.74]° | 10.5 [27.0] mm, 0.50 [0.98]° | 4.9 [9.3] mm, 0.55 [1.14]° |
| 4 mm / 1° | 10.8 [20.3] mm, 0.82 [1.76]° | 10.5 [19.4] mm, 0.68 [1.48]° | 10.5 [19.5] mm, 0.69 [1.45]° | 25.2 [65.9] mm, 1.07 [2.02]° | 8.8 [69.2] mm, 1.07 [9.2]° |

**[2b] Error vs. número de poses** (1 mm / 0.25°): la mediana de traslación de Park baja de 5.0 mm con 5 poses a 3.0 / 2.6 / 2.1 / 1.7 mm con 10 / 15 / 20 / 30 poses. La rotación pasa de 0.37° a 0.12°. La mejora se aplana después de ~15-20 poses.

**Conclusiones (para esta escena simulada):**
- **Park ≈ Horaud** (resultados prácticamente idénticos; sus líneas se superponen en las gráficas) son los más consistentes. Tsai queda muy cerca. **Andreff** duplica el error de traslación.
- **Daniilidis** tiene la mejor mediana de traslación en la mayoría de condiciones (no en todas), pero **fallos catastróficos ocasionales**: 28 de 2200 corridas (1.3%) con error de ~180° y cientos a miles de mm. Por eso su media y desviación son enormes y en las tablas y gráficas se usan mediana y p95. El **residuo de consistencia** (sin verdad de terreno) detecta esos fallos con valores de decenas de miles de mm, lo que confirma que sirve con datos reales.
- El error de traslación es aproximadamente lineal en el ruido: ~2.5 mm de mediana por cada 1 mm / 0.25° de ruido, con 15 poses.

**[3] Caso degenerado** (15 poses, 1 mm / 0.25°; mediana):

| Poses | Park: traslación / rotación | Tsai | Andreff | Daniilidis |
|---|---|---|---|---|
| `varied` | 2.6 mm / 0.19° | 2.5 mm / 0.20° | 4.8 mm / 0.25° | 2.3 mm / 0.26° |
| `low_rotation` (≤ 3°) | 8.7 mm / 0.91° | 10.9 mm / 1.35° (9 groseros) | 9.5 mm / 2.1° | 7.3 mm / 0.74° |
| `translation_only` | **falla en las 200** (NaN / SVD) | 1208 mm / 129° | 1208 mm / 0.84° | 877 mm / 0.53° |

Con poca variedad de rotación, el error de rotación se multiplica por ~5 y el de traslación por ~3-4. Con orientación fija, **la traslación de `T_base_cam` no es observable con ningún método**. La rotación sí falla con Tsai, Park y Horaud, pero no del todo con Andreff y Daniilidis, que la recuperan a ~1° gracias a la parte traslacional de la ecuación. Esto matiza la hipótesis inicial de que "la rotación se vuelve poco confiable": lo que se pierde por completo es la traslación.

## Limitaciones y problemas conocidos

- **Escena supuesta:** cámara en `(1100, 0, 500)` mm, tablero a 50 mm del TCP y caja de trabajo proporcional al alcance. Sin cinemática inversa ni colisiones.
- **Ruido gaussiano** independiente por pose y sin error del robot (repetibilidad o calibración cinemática). Con datos reales, los errores serán mayores.
- La tabla impresa en consola puede verse con caracteres raros (`Ã³`) al usar `conda run` en PowerShell. Los CSV están en UTF-8 y se ven bien. Para evitarlo: `conda activate neuralgrasp` y `$env:PYTHONIOENCODING="utf-8"`.
- OpenCV escribe `Hand-eye calibration failed! Not enough informative motions` en casos degenerados; el script silencia el log de OpenCV y cuenta esos casos como fallos.
- `poses_example.csv` contiene poses **sintéticas** generadas por la librería (semilla 123), **no** grabadas en RobotStudio.

## Siguiente paso

1. Grabar 15-30 poses reales con jog (con `tVision` activa y orientaciones variadas, inclinaciones de decenas de grados) y correr `--poses csv`.
2. Implementar el `GET_POSE` completo en `comm.mod` y el modo `robotstudio`.
3. Usar `calibration.perturb_transform` sobre `T_base_cam` para el experimento de error de calibración vs. éxito de agarre.
