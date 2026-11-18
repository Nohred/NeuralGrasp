"""Experimento 04: calibración mano-ojo eye-to-hand con datos sintéticos.

Pasos:
  1. Prueba sin ruido: el error debe ser ~0 (tolerancia NOISE_FREE_TOL, ver abajo).
  2. Monte Carlo (>= 200 repeticiones por condición):
       a) error vs. nivel de ruido (con --n-poses poses),
       b) error vs. número de poses (5, 10, 15, 20, 30) con el ruido nominal,
     para cada método de cv2.calibrateHandEye. Media, desviación y percentil 95.
  3. Caso degenerado: poses con orientación fija (solo traslación) y con poca
     variedad de rotación, comparadas con poses variadas.

Salidas en --out (por defecto experiments/04_handeye_synthetic/results/, ignorado por git):
  raw.csv, summary.csv, degenerate.csv, error_vs_noise.png, error_vs_nposes.png.

Ejemplo:
  python experiments/04_handeye_synthetic/run.py --n-reps 200 --seed 0
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
import time
from collections import defaultdict

import numpy as np

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

import cv2  # noqa: E402

from calibration import (  # noqa: E402
    METHODS,
    SceneConfig,
    SyntheticScene,
    calibrate_eye_to_hand,
    consistency_residual,
    R_to_euler_rapid,
    T_to_pose_rapid,
    pose_error,
    pose_rapid_to_T,
    rotation_angle_deg,
    summarize,
)

# Tolerancia de la prueba sin ruido. En la práctica los 5 métodos dan ~1e-10 mm/°;
# 1e-6 deja margen para errores de redondeo en float64 sin ocultar errores reales.
NOISE_FREE_TOL = 1e-6

# Umbral para contar un resultado como "error grosero" (fallo catastrófico del método).
GROSS_TRANS_MM = 100.0
GROSS_ROT_DEG = 10.0

# Paleta categórica validada (orden fijo, nunca ciclado) + marcador por método,
# para que el método no se identifique solo por color.
METHOD_STYLE = {
    "tsai": ("#2a78d6", "o"),
    "park": ("#eb6834", "s"),
    "horaud": ("#1baf7a", "^"),
    "andreff": ("#eda100", "D"),
    "daniilidis": ("#e87ba4", "v"),
}

POSES_CSV_HEADER = "x,y,z,q1,q2,q3,q4"

# Diferencia máxima aceptable entre la pose pedida con MOVE_POSE y la leída con GET_POSE.
# GET_POSE redondea a 0.01 mm y 1e-6 en el cuaternión (~1e-4°); MoveL ... fine llega
# exacto en el simulador. Diferencias mayores indican otra convención de OrientZYX.
CMD_VS_READ_TRANS_MM = 0.5
CMD_VS_READ_ROT_DEG = 0.1


# ------------------------------------------------------------------ utilidades
def parse_float_list(text: str) -> list:
    return [float(v) for v in str(text).split(",") if v.strip() != ""]


def load_poses_csv(path: str) -> list:
    """Lee poses ``x,y,z,q1,q2,q3,q4`` (mm, cuaternión en orden RAPID w,x,y,z).

    Ignora líneas vacías, comentarios (#) y encabezados no numéricos.
    """
    poses = []
    with open(path, newline="", encoding="utf-8") as fh:
        for lineno, row in enumerate(csv.reader(fh), start=1):
            if not row or row[0].strip().startswith("#"):
                continue
            try:
                vals = [float(v) for v in row[:7]]
            except ValueError:
                continue  # encabezado
            if len(vals) != 7:
                raise ValueError(f"{path}:{lineno}: se esperaban 7 columnas x,y,z,q1,q2,q3,q4")
            q = np.array(vals[3:])
            if not np.isclose(np.linalg.norm(q), 1.0, atol=1e-3):
                raise ValueError(f"{path}:{lineno}: cuaternión no unitario (|q|={np.linalg.norm(q):.4f})")
            poses.append(pose_rapid_to_T(*vals[:3], *(q / np.linalg.norm(q))))
    return poses


def save_poses_csv(path: str, poses_T, comment: str = "") -> None:
    """Guarda poses 4x4 como ``x,y,z,q1,q2,q3,q4`` (mm, cuaternión RAPID)."""
    with open(path, "w", newline="\n", encoding="utf-8") as fh:
        if comment:
            for line in comment.splitlines():
                fh.write(f"# {line}\n")
        fh.write(POSES_CSV_HEADER + "\n")
        for T in poses_T:
            fh.write(",".join(f"{v:.6f}" for v in T_to_pose_rapid(T)) + "\n")


def order_by_rotation(poses_T, start_R):
    """Ordena las poses (vecino más cercano) para minimizar el cambio de orientación
    entre movimientos consecutivos: MOVE_POSE hereda robconf de la pose actual."""
    remaining = list(poses_T)
    ordered, R = [], start_R
    while remaining:
        i = min(range(len(remaining)), key=lambda k: rotation_angle_deg(R, remaining[k][:3, :3]))
        T = remaining.pop(i)
        ordered.append(T)
        R = T[:3, :3]
    return ordered


def collect_robotstudio_poses(args, scene: SyntheticScene) -> list:
    """Mueve el robot de RobotStudio por poses sintéticas y lee la pose real con GET_POSE.

    Requiere robot/rapid/comm.mod (con GET_POSE) en ejecución. Devuelve las poses
    LEÍDAS (4x4) de los movimientos que respondieron OK, y las guarda en CSV.
    """
    from robot.client import RobotVisionController

    rng = np.random.default_rng([args.seed, 7])
    targets = scene.sample_gripper_poses(args.rs_n_poses, rng)
    robot = RobotVisionController(host=args.host, port=args.port, timeout=args.rs_timeout)

    if args.rs_dry_run:
        print("[robotstudio] --rs-dry-run: comandos que se enviarían (sin conectar):")
        for T in order_by_rotation(targets, np.eye(3)):
            rx, ry, rz = R_to_euler_rapid(T[:3, :3])
            print(f"    MOVE_POSE,{T[0, 3]:.2f},{T[1, 3]:.2f},{T[2, 3]:.2f},{rx:.2f},{ry:.2f},{rz:.2f}")
        raise SystemExit(0)

    if not robot.connect():
        raise SystemExit("[robotstudio] No se pudo conectar. ¿Está comm.mod (con GET_POSE) en ejecución?")
    read_poses, rows = [], []
    try:
        start = robot.get_pose()
        if start is None:
            raise SystemExit("[robotstudio] GET_POSE no respondió POSE,... ¿Recargaste comm.mod con GET_POSE?")
        targets = order_by_rotation(targets, pose_rapid_to_T(*start)[:3, :3])
        for i, T in enumerate(targets, start=1):
            rx, ry, rz = R_to_euler_rapid(T[:3, :3])
            try:
                ok = robot.move_pose_euler(T[0, 3], T[1, 3], T[2, 3], rx, ry, rz)
                pose = robot.get_pose() if ok else None
            except OSError as exc:  # RAPID se detuvo (p. ej. error de movimiento) y cerró el socket
                print(f"[robotstudio] Conexión perdida en la pose {i}: {exc}. Se usan las {len(read_poses)} anteriores.")
                break
            if pose is None:
                print(f"[robotstudio] Pose {i}/{len(targets)}: MOVE_POSE sin OK; se omite.")
                continue
            T_read = pose_rapid_to_T(*pose)
            dt, dr = pose_error(T_read, T)
            rows.append((i, dt, dr))
            read_poses.append(T_read)
            print(f"[robotstudio] Pose {i}/{len(targets)}: pedida vs. leída = {dt:.3f} mm, {dr:.4f}°")
    finally:
        robot.disconnect()

    if rows:
        max_dt = max(r[1] for r in rows)
        max_dr = max(r[2] for r in rows)
        ok_conv = max_dt <= CMD_VS_READ_TRANS_MM and max_dr <= CMD_VS_READ_ROT_DEG
        print(f"[robotstudio] Máx. diferencia pedida vs. leída: {max_dt:.3f} mm, {max_dr:.4f}° -> "
              + ("convención OrientZYX/cuaternión CONSISTENTE" if ok_conv else
                 "¡REVISAR! la convención de orientación no coincide (ver docs/coordinate_frames.md)"))
    os.makedirs(args.out, exist_ok=True)
    csv_path = os.path.join(args.out, "robotstudio_poses.csv")
    save_poses_csv(csv_path, read_poses,
                   "Poses LEÍDAS con GET_POSE (tVision, wobj0) tras MOVE_POSE a poses sintéticas.\n"
                   f"semilla {args.seed}, alcance {args.reach} mm. Reutilizable con --poses csv.")
    print(f"[robotstudio] {len(read_poses)} poses guardadas en {csv_path}")
    return read_poses


def make_pose_source(args, scene: SyntheticScene):
    """Devuelve ``sample(n, rng, mode) -> lista de T_base_gripper`` según --poses."""
    if args.poses == "synthetic":
        return lambda n, rng, mode="varied": scene.sample_gripper_poses(n, rng, mode=mode)

    if args.poses == "robotstudio":
        all_poses = collect_robotstudio_poses(args, scene)
        tag = "robotstudio"
    else:
        if not args.csv:
            raise SystemExit("--poses csv requiere --csv <archivo>")
        all_poses = load_poses_csv(args.csv)
        tag = "csv"
    if not args.no_visibility_filter:
        visible = [T for T in all_poses if scene.is_visible(T)]
        print(f"[{tag}] {len(visible)}/{len(all_poses)} poses con el tablero visible "
              f"según la cámara simulada (usa --no-visibility-filter para desactivar).")
        all_poses = visible
    if len(all_poses) < 3:
        raise SystemExit(f"[{tag}] Se necesitan al menos 3 poses utilizables.")

    def sample(n, rng, mode="varied"):
        if mode != "varied":
            raise ValueError("El caso degenerado solo existe en modo synthetic")
        k = min(n, len(all_poses))
        idx = rng.choice(len(all_poses), size=k, replace=False)
        return [all_poses[i] for i in idx]

    sample.n_available = len(all_poses)
    return sample


def run_trial(scene, poses, methods, rng, nt, nr):
    """Una repetición: mismas poses y mismas detecciones para todos los métodos."""
    det = scene.detect(poses, rng, nt, nr)
    out = {}
    for m in methods:
        try:
            T_est = calibrate_eye_to_hand(poses, det, m)
            if not np.all(np.isfinite(T_est)):
                raise FloatingPointError("resultado no finito")
            et, er = pose_error(T_est, scene.T_base_cam)
            cr = consistency_residual(poses, det, T_est)
            out[m] = (et, er, cr["trans_rms_mm"], cr["rot_rms_deg"], 0)
        except Exception:  # cv2 puede lanzar o devolver NaN en casos degenerados
            out[m] = (np.nan, np.nan, np.nan, np.nan, 1)
    return out


def trial_rng(seed, *keys):
    """RNG determinista por condición y repetición, independiente del orden de ejecución."""
    return np.random.default_rng([seed, *keys])


# ------------------------------------------------------------------ pasos
def noise_free_check(scene, sample, methods, n_poses, seed):
    print(f"\n[1] Prueba sin ruido ({n_poses} poses, tolerancia {NOISE_FREE_TOL:g} mm / grados)")
    rng = trial_rng(seed, 99)
    poses = sample(n_poses, rng)
    res = run_trial(scene, poses, methods, rng, 0.0, 0.0)
    ok_all = True
    for m in methods:
        et, er, *_ , failed = res[m]
        ok = (not failed) and et < NOISE_FREE_TOL and er < NOISE_FREE_TOL
        ok_all &= ok
        print(f"    {m:<11} trans={et:.2e} mm  rot={er:.2e} deg  {'OK' if ok else 'FALLA'}")
    return ok_all


def monte_carlo(scene, sample, args, methods, levels, nominal, pose_counts):
    rows = []
    t0 = time.time()
    print(f"\n[2a] Error vs. ruido: {len(levels)} niveles x {args.n_reps} reps, {args.n_poses} poses")
    for li, (nt, nr) in enumerate(levels):
        for rep in range(args.n_reps):
            rng = trial_rng(args.seed, 1, li, rep)
            res = run_trial(scene, sample(args.n_poses, rng), methods, rng, nt, nr)
            rows += _rows("noise", "varied", args.n_poses, nt, nr, rep, res)
    print(f"[2b] Error vs. número de poses {pose_counts}: ruido nominal "
          f"{nominal[0]} mm / {nominal[1]} deg, {args.n_reps} reps")
    for ci, n in enumerate(pose_counts):
        for rep in range(args.n_reps):
            rng = trial_rng(args.seed, 2, ci, rep)
            res = run_trial(scene, sample(n, rng), methods, rng, *nominal)
            rows += _rows("n_poses", "varied", n, *nominal, rep, res)
    print(f"    ({time.time() - t0:.1f} s)")
    return rows


def degenerate(scene, sample, args, methods, nominal):
    rows = []
    modes = ["varied", "low_rotation", "translation_only"]
    print(f"\n[3] Caso degenerado: {modes}, {args.n_poses} poses, ruido nominal, {args.n_reps} reps")
    for mi, mode in enumerate(modes):
        for rep in range(args.n_reps):
            rng = trial_rng(args.seed, 3, mi, rep)
            res = run_trial(scene, sample(args.n_poses, rng, mode), methods, rng, *nominal)
            rows += _rows("degenerate", mode, args.n_poses, *nominal, rep, res)
    return rows


def _rows(sweep, mode, n, nt, nr, rep, res):
    return [dict(sweep=sweep, pose_mode=mode, method=m, n_poses=n, noise_trans_mm=nt,
                 noise_rot_deg=nr, rep=rep, trans_err_mm=v[0], rot_err_deg=v[1],
                 consist_trans_rms_mm=v[2], consist_rot_rms_deg=v[3], failed=v[4])
            for m, v in res.items()]


def summarize_rows(rows):
    groups = defaultdict(list)
    for r in rows:
        key = (r["sweep"], r["pose_mode"], r["method"], r["n_poses"], r["noise_trans_mm"], r["noise_rot_deg"])
        groups[key].append(r)
    out = []
    for key, rs in groups.items():
        st = summarize([r["trans_err_mm"] for r in rs])
        sr = summarize([r["rot_err_deg"] for r in rs])
        sc = summarize([r["consist_trans_rms_mm"] for r in rs])
        gross = sum(1 for r in rs if r["trans_err_mm"] > GROSS_TRANS_MM or r["rot_err_deg"] > GROSS_ROT_DEG)
        out.append(dict(zip(("sweep", "pose_mode", "method", "n_poses", "noise_trans_mm", "noise_rot_deg"), key),
                        reps=len(rs), failures=sum(r["failed"] for r in rs), gross_errors=gross,
                        trans_mean_mm=st["mean"], trans_std_mm=st["std"], trans_median_mm=st["median"],
                        trans_p95_mm=st["p95"],
                        rot_mean_deg=sr["mean"], rot_std_deg=sr["std"], rot_median_deg=sr["median"],
                        rot_p95_deg=sr["p95"],
                        consist_trans_rms_median_mm=sc["median"]))
    return out


def write_csv(path, rows):
    if not rows:
        return
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def print_table(summary, sweep, title):
    rows = [s for s in summary if s["sweep"] == sweep]
    if not rows:
        return
    print(f"\n{title}")
    hdr = (f"{'modo':<17}{'método':<11}{'poses':>5}{'σt mm':>7}{'σr °':>6} | "
           f"{'trans media±std [mediana, p95] mm':>40} | {'rot media±std [mediana, p95] °':>38} | "
           f"groseros | fallos")
    print(hdr)
    print("-" * len(hdr))
    for s in rows:
        print(f"{s['pose_mode']:<17}{s['method']:<11}{s['n_poses']:>5}{s['noise_trans_mm']:>7.2f}"
              f"{s['noise_rot_deg']:>6.2f} | "
              f"{s['trans_mean_mm']:>10.4g}±{s['trans_std_mm']:<9.3g}[{s['trans_median_mm']:>8.4g},{s['trans_p95_mm']:>9.4g}] | "
              f"{s['rot_mean_deg']:>9.4g}±{s['rot_std_deg']:<8.3g}[{s['rot_median_deg']:>8.4g},{s['rot_p95_deg']:>8.4g}] | "
              f"{s['gross_errors']:>8} | {s['failures']}/{s['reps']}")
    print(f"(groseros: error > {GROSS_TRANS_MM:g} mm o > {GROSS_ROT_DEG:g}°; fallos: excepción o resultado NaN)")


def plot_sweep(summary, sweep, xkey, xlabel, title, path, methods, xticklabels=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rows = [s for s in summary if s["sweep"] == sweep]
    # El nivel de ruido 0 (error ~1e-12) aplasta la escala log; ya lo cubre la prueba [1].
    rows = [s for s in rows if s["noise_trans_mm"] > 0 or s["noise_rot_deg"] > 0]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2), constrained_layout=True)
    fig.patch.set_facecolor("#fcfcfb")
    # Mediana (sólida) y p95 (punteada): la media se distorsiona con fallos catastróficos raros.
    for ax, (mean_k, p95_k, ylab) in zip(axes, [("trans_median_mm", "trans_p95_mm", "Error de traslación (mm)"),
                                               ("rot_median_deg", "rot_p95_deg", "Error de rotación (°)")]):
        ax.set_facecolor("#fcfcfb")
        for m in methods:
            rs = sorted((r for r in rows if r["method"] == m), key=lambda r: r[xkey])
            x = [r[xkey] for r in rs]
            color, marker = METHOD_STYLE.get(m, ("#52514e", "o"))
            ax.plot(x, [r[mean_k] for r in rs], color=color, marker=marker, lw=2, ms=6, label=m)
            ax.plot(x, [r[p95_k] for r in rs], color=color, lw=1, ls="--", alpha=0.8)
        ax.set_yscale("log")
        ax.set_xlabel(xlabel, color="#52514e")
        ax.set_ylabel(ylab, color="#52514e")
        ax.grid(True, which="major", color="#e4e3df", lw=0.8)
        ax.tick_params(colors="#52514e")
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        if xticklabels:
            ax.set_xticks(list(xticklabels.keys()))
            ax.set_xticklabels(list(xticklabels.values()))
    axes[0].legend(frameon=False, fontsize=8, title="sólida = mediana, punteada = p95", title_fontsize=8)
    fig.suptitle(title, color="#0b0b0b")
    fig.savefig(path, dpi=150)
    plt.close(fig)


# ------------------------------------------------------------------ main
def build_parser():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--poses", choices=["synthetic", "csv", "robotstudio"], default="synthetic",
                   help="Origen de las poses del robot (las detecciones siempre se simulan).")
    p.add_argument("--csv", help="Archivo x,y,z,q1,q2,q3,q4 (mm, orden RAPID) para --poses csv.")
    p.add_argument("--no-visibility-filter", action="store_true",
                   help="En modo csv/robotstudio, no descartar poses con el tablero no visible.")
    rs = p.add_argument_group("modo robotstudio (requiere comm.mod con GET_POSE en ejecución)")
    rs.add_argument("--host", default="127.0.0.1")
    rs.add_argument("--port", type=int, default=55555)
    rs.add_argument("--rs-n-poses", type=int, default=20,
                    help="Poses sintéticas a las que se mueve el robot (se leen con GET_POSE).")
    rs.add_argument("--rs-timeout", type=float, default=60.0,
                    help="Timeout del socket (s); un MoveL largo tarda varios segundos.")
    rs.add_argument("--rs-dry-run", action="store_true",
                    help="Solo imprime los MOVE_POSE que se enviarían, sin conectar.")
    p.add_argument("--n-poses", type=int, default=15, help="Poses por calibración en [1], [2a] y [3].")
    p.add_argument("--pose-counts", default="5,10,15,20,30", help="Barrido de número de poses [2b].")
    p.add_argument("--noise-trans-mm", default="0,0.5,1,2,4",
                   help="Niveles de ruido de traslación (lista separada por comas).")
    p.add_argument("--noise-rot-deg", default="0,0.1,0.25,0.5,1",
                   help="Niveles de ruido de rotación, emparejados con --noise-trans-mm "
                        "(si uno tiene 1 valor se repite). El nivel nominal para [2b] y [3] "
                        "es el del medio de la lista.")
    p.add_argument("--n-reps", type=int, default=200, help="Repeticiones Monte Carlo por condición.")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--methods", default=",".join(METHODS), help=f"Subconjunto de {list(METHODS)}.")
    p.add_argument("--reach", type=int, choices=[475, 580], default=475,
                   help="Variante del IRB 1100 (alcance en mm) -> caja de trabajo.")
    p.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "results"))
    p.add_argument("--no-plots", action="store_true")
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:  # cv2 imprime "Hand-eye calibration failed" en casos degenerados; se cuentan como fallos.
        cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_SILENT)
    except AttributeError:
        pass

    methods = [m.strip() for m in args.methods.split(",") if m.strip()]
    unknown = set(methods) - set(METHODS)
    if unknown:
        raise SystemExit(f"Métodos desconocidos: {sorted(unknown)}")
    nts, nrs = parse_float_list(args.noise_trans_mm), parse_float_list(args.noise_rot_deg)
    if len(nts) == 1:
        nts = nts * len(nrs)
    if len(nrs) == 1:
        nrs = nrs * len(nts)
    if len(nts) != len(nrs):
        raise SystemExit("--noise-trans-mm y --noise-rot-deg deben tener la misma longitud (o 1 valor)")
    levels = list(zip(nts, nrs))
    nominal = levels[len(levels) // 2]
    pose_counts = [int(v) for v in parse_float_list(args.pose_counts)]

    scene = SyntheticScene.from_config(SceneConfig(reach_mm=float(args.reach)))
    sample = make_pose_source(args, scene)
    if args.poses != "synthetic":
        too_many = [n for n in pose_counts + [args.n_poses] if n > sample.n_available]
        if too_many:
            print(f"[{args.poses}] Aviso: se piden {too_many} poses pero solo hay {sample.n_available}; se usan todas.")

    os.makedirs(args.out, exist_ok=True)
    print(f"Escena: alcance {args.reach} mm, cámara en {scene.config.cam_position_mm} mm, "
          f"tablero a {scene.config.board_offset_mm} mm sobre z de la herramienta; poses: {args.poses}")

    ok = noise_free_check(scene, sample, methods, args.n_poses, args.seed)
    rows = monte_carlo(scene, sample, args, methods, levels, nominal, pose_counts)
    if args.poses == "synthetic":
        rows += degenerate(scene, sample, args, methods, nominal)
    else:
        print("\n[3] Caso degenerado omitido (solo aplica a poses sintéticas).")

    summary = summarize_rows(rows)
    write_csv(os.path.join(args.out, "raw.csv"), rows)
    write_csv(os.path.join(args.out, "summary.csv"), [s for s in summary if s["sweep"] != "degenerate"])
    write_csv(os.path.join(args.out, "degenerate.csv"), [s for s in summary if s["sweep"] == "degenerate"])

    print_table(summary, "noise", f"=== Error vs. ruido ({args.n_poses} poses, {args.n_reps} reps) ===")
    print_table(summary, "n_poses", f"=== Error vs. número de poses (ruido {nominal[0]} mm / {nominal[1]}°) ===")
    print_table(summary, "degenerate", "=== Caso degenerado ===")

    if not args.no_plots:
        idx = {nt: f"{nt:g} mm\n{nr:g}°" for nt, nr in levels if nt > 0 or nr > 0}
        plot_sweep(summary, "noise", "noise_trans_mm", "Ruido de la detección (σ traslación / σ rotación)",
                   f"Eye-to-hand sintético: error vs. ruido ({args.n_poses} poses, {args.n_reps} reps)",
                   os.path.join(args.out, "error_vs_noise.png"), methods, xticklabels=idx)
        plot_sweep(summary, "n_poses", "n_poses", "Número de poses",
                   f"Eye-to-hand sintético: error vs. número de poses (ruido {nominal[0]:g} mm / {nominal[1]:g}°)",
                   os.path.join(args.out, "error_vs_nposes.png"), methods)
    print(f"\nResultados en: {args.out}")
    print(f"Prueba sin ruido: {'OK' if ok else 'FALLA'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
