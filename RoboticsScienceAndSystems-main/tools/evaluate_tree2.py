"""Replay held-out turns and stress both compiled firmware adapters, without motor I/O."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import subprocess

import numpy as np

from train_tree2 import DEADBAND, GAIN, knots, measured_prediction, metrics, raw_prediction, write_json
from tree2_data import DATA, MODEL, ROOT, load_experiments


def replay(program, angle, signal, left, right):
    angle, signal, left, right = np.broadcast_arrays(angle, signal, left, right)
    data = "".join(f"{a:.17g} {int(s)} {int(l)} {int(r)}\n"
                   for a, s, l, r in zip(angle.flat, signal.flat, left.flat, right.flat))
    result = subprocess.run([str(program.resolve())], input=data, text=True, capture_output=True, check=True)
    values = np.fromstring(result.stdout, sep=" ").reshape(angle.size, 5)
    if not np.isfinite(values).all():
        raise ValueError("Nonfinite C++ prediction")
    return values


def from_difference(previous):
    left = -(np.asarray(previous) // 2)
    return left, left + previous


def integer_wheels(angle, signal, differential, measured):
    base = np.where(signal > 0, 45., -35.) * np.clip(np.cos(angle), 0., 1.)
    if measured:
        nominal = np.minimum(GAIN*(np.abs(angle.astype(np.float32))-DEADBAND), 140.)
        actual = np.abs(differential.astype(np.float32))
        base *= np.divide(actual, nominal, out=np.ones_like(actual), where=(signal > 0) & (actual < nominal))
    limit = 2*(100-np.abs(base))
    half = np.clip(differential, -limit, limit) / 2
    wheels = np.column_stack((base-half, base+half))
    return np.sign(wheels)*np.floor(np.abs(wheels)+.5)


def reference(angle, signal, left, right, leaves):
    a = angle.astype(np.float32)
    sign = -np.sign(a)
    target = np.clip(GAIN*(np.abs(a)-DEADBAND), 0., 140.)
    aligned = np.clip(sign*(right-left), 0., target)
    reverse = sign*(target + .65*(aligned-target))
    return np.where(signal > 0, measured_prediction(a, leaves), reverse)


def check_pair(programs, leaves, angle, signal, left, right):
    angle, signal, left, right = np.broadcast_arrays(angle, signal, left, right)
    old, new = [replay(p, angle, signal, left, right) for p in programs]
    np.testing.assert_array_equal(old[:, :2], new[:, :2])
    np.testing.assert_array_equal(new[:, 4].astype(np.float32), raw_prediction(angle, leaves))
    desired = reference(angle, signal, left, right, leaves)
    error = float(np.max(np.abs(desired - new[:, 1])))
    np.testing.assert_allclose(new[:, 1], desired, atol=2e-5, rtol=0)
    for column, output in enumerate((old, new)):
        p = output[:, column].astype(np.float32).astype(float)
        np.testing.assert_array_equal(output[:, 2:4], integer_wheels(angle, signal, p, measured=(column == 1)))
    return old, new, error


def jump(values):
    return float(np.max(np.abs(np.diff(values))))


def zero_crossing_replay(program):
    # Command-level transient with feedback of the previous integer wheel commands.
    # Angles are prescribed; this experiment has no simulated plant or sensors.
    left, right = -35, 35
    trace = []
    for deg in (75., 5., 2., 0., 0., 0., .2, -.2, .2, -.2, 0.):
        result = replay(program, np.deg2rad(deg), 1, left, right)[0]
        left, right = int(result[2]), int(result[3])
        trace.append(dict(angle_deg=deg, left=left, right=right, difference=right-left))
    return trace


def verify_archived_python(directory, exe, test_features):
    """Optional independent refit using the original October trainer and its synthetic CSV."""
    archive = json.loads((ROOT/"StampC3_Template/Tree2TrainingReport.json").read_text())
    csv = directory/"tree2_data.csv"
    if hashlib.sha256(csv.read_bytes()).hexdigest() != archive["csv_sha256"]:
        raise ValueError("Legacy reference CSV does not match the archived firmware training report")
    spec = importlib.util.spec_from_file_location("archived_tree2", directory/"main.py")
    trainer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(trainer)
    df = trainer.load_training_data(csv)
    x = trainer.make_features(df)
    tree = trainer.DecisionTreeRegressor(max_depth=4, min_samples_leaf=20, random_state=42)
    target = (df.pwm_r_target - df.pwm_l_target).to_numpy()
    tree.fit(x, target)
    coefficients = trainer.fit_leaf_models(tree, x, target)
    rng = np.random.default_rng(42)
    extra = np.column_stack((rng.uniform(-np.pi, np.pi, 4000), rng.choice([-1, 1], 4000),
                             rng.integers(-200, 201, 4000)))
    features = np.vstack((x, test_features, extra)).astype(np.float32)
    a, signal, d = features.T
    values = replay(exe, a, signal, *from_difference(d.astype(int)))[:, 0]
    reference_pwm = trainer.model_predict(tree, coefficients, features)
    np.testing.assert_allclose(values, reference_pwm, atol=2e-5, rtol=0)
    return dict(cases=len(values), max_abs_pwm=float(np.abs(values-reference_pwm).max()),
                synthetic_csv_sha256=archive["csv_sha256"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--legacy", type=Path, required=True, help="Legacy tree2_replay executable")
    parser.add_argument("--measured", type=Path, required=True, help="Default tree2_replay executable")
    parser.add_argument("--legacy-trainer", type=Path, help="Optional original trainer directory with main.py and tree2_data.csv")
    parser.add_argument("--data", type=Path, default=DATA)
    args = parser.parse_args()
    programs = [args.legacy, args.measured]
    for exe, version in zip(programs, ("legacy", "measured")):
        if subprocess.check_output([str(exe.resolve()), "--version"], text=True).strip() != version:
            raise ValueError(f"Expected {version} build at {exe}")
    rows, _ = load_experiments(args.data)
    test = rows[rows.split.eq("test")].reset_index(drop=True)
    leaves = json.loads(MODEL.read_text())["leaves"]
    angle = np.deg2rad(test.error_deg.to_numpy())
    label = test.pwm_right_minus_left.to_numpy()
    left, right = test.prev_l.to_numpy(), test.prev_r.to_numpy()
    old, new, err = check_pair(programs, leaves, angle, 1, left, right)
    x, y = knots(leaves)
    predictions = dict(legacy=old[:, 0], cart=raw_prediction(angle, leaves),
                       interpolated=np.interp(angle.astype(np.float32), x, y), deployed=new[:, 1],
                       legacy_integer=old[:, 3]-old[:, 2], deployed_integer=new[:, 3]-new[:, 2])
    def score(mask):
        return {k: metrics(label[mask], p[mask]) for k, p in predictions.items()}
    slices = {}
    magnitude = np.abs(test.error_deg.to_numpy())
    for lo, hi in [(0, 5), (5, 15), (15, 45), (45, 70), (70, 80), (80, 91)]:
        mask = (magnitude > lo) & (magnitude <= hi)
        if mask.any():
            slices[f"({lo},{hi}]"] = score(mask)
    macro = {k: float(np.mean([np.abs(label[ix]-p[ix]).mean()
                               for ix in test.groupby(["session", "trial"]).indices.values()]))
             for k, p in predictions.items()}
    report = dict(test=score(slice(None)), by_abs_remaining_deg=slices,
                  by_direction={d: score(test.direction.eq(d).to_numpy()) for d in ("L", "R")},
                  episode_macro_mae=macro)
    cases, max_error = len(test), err

    # Include all possible signed previous-wheel differences at the reported large angle.
    previous = np.tile(np.arange(-200, 201), 4)
    signal = np.repeat([1, 1, -1, -1], 401)
    deg = np.repeat([75., -75., 75., -75.], 401)
    old, new, err = check_pair(programs, leaves, np.deg2rad(deg), signal, *from_difference(previous))
    cases += len(deg)
    max_error = max(max_error, err)
    at75 = []
    for mode in (1, -1):
        for side in (75, -75):
            use = (deg == side) & (signal == mode)
            wrong = use & (old[:, 0]*deg > 0)
            examples = {str(d): [float(old[i, 0]), float(new[i, 1]), int(old[i, 2]), int(old[i, 3]),
                                  int(new[i, 2]), int(new[i, 3])]
                        for d in (0, 20, 70, 100, -70, -100)
                        for i in np.flatnonzero(use & (previous == d))}
            at75.append(dict(signal=mode, angle=side, n=int(use.sum()), old_wrong=int(wrong.sum()),
                             old_wrong_history_range=[int(previous[wrong].min()), int(previous[wrong].max())]
                             if wrong.any() else [], new_wrong=int((new[use, 1]*side > 0).sum()),
                             examples_old_new_pwmL_pwmR=examples))
    report["at75"] = at75

    # Dense around zero, broad across the full domain; exact float32 neighbors of every
    # old angle split, new CART split, centroid and deadband expose discontinuities.
    header = (ROOT/"StampC3_Template/Tree2Model.h").read_text()
    splits = [float(v) for v in re.findall(r"double\(x\[0\]\) <= ([-+.\deE]+)", header)]
    cuts = np.asarray(splits + [-s for s in splits] + [r["upper_rad"] for r in leaves[:-1]]
                      + list(x) + [-DEADBAND, DEADBAND], dtype=np.float32)
    dense = np.deg2rad(np.arange(-500, 501)/100)
    # The adapter's -pi tie is +pi. Use the open negative endpoint for a one-sided sweep.
    grid = np.unique(np.r_[np.deg2rad(np.arange(-1799, 1801)/10), dense, cuts,
                           np.nextafter(cuts, np.float32(-np.inf)), np.nextafter(cuts, np.float32(np.inf))])
    sweeps = []
    for mode in (1, -1):
        for d in (-200, -100, -20, 0, 20, 100, 200):
            old, new, err = check_pair(programs, leaves, grid, mode, *from_difference(d))
            cases += len(grid)
            max_error = max(max_error, err)
            small = np.isin(grid, dense)
            zero = list(grid).index(0.)
            sweep = dict(signal=mode, previous_difference=d, n=len(grid))
            for name, diff, wheels in (("legacy", old[:, 0], old[:, 3]-old[:, 2]),
                                      ("deployed", new[:, 1], new[:, 3]-new[:, 2])):
                near_equal = small & (wheels == 0)
                around_zero = grid[small]
                absdiff = np.abs(np.diff(diff))
                i = int(absdiff.argmax())
                sweep[name] = dict(opposite=int((diff*grid > 0).sum()), zero_error_pwm=float(diff[zero]),
                                   max_grid_jump=float(absdiff[i]), max_jump_pair_deg=np.rad2deg(grid[i:i+2]).tolist(),
                                   small_float_jump=jump(diff[small]), small_integer_jump=jump(wheels[small]),
                                   small_opposite=int((diff[small]*around_zero > 0).sum()),
                                   small_pwm_range=[float(diff[small].min()), float(diff[small].max())],
                                   equal_wheel_range_deg=np.rad2deg([grid[near_equal].min(), grid[near_equal].max()]).tolist()
                                   if near_equal.any() else [])
            if mode == 1:
                raw = raw_prediction(grid, leaves)
                sweep["cart_max_jump"] = jump(raw)
                sweep["cart_small_jump"] = jump(raw[small])
            if sweep["deployed"]["opposite"] or sweep["deployed"]["zero_error_pwm"] != 0:
                raise AssertionError("Current policy failed a direction/zero-error invariant")
            sweeps.append(sweep)
    report.update(sweeps=sweeps, parity=dict(cases_per_version=cases, python_cpp_max_abs_pwm=max_error,
                  integer_wheel_mismatches=0, model_sha256=hashlib.sha256(MODEL.read_bytes()).hexdigest()))
    report["zero_crossing_replay"] = {version: zero_crossing_replay(exe)
                                      for version, exe in zip(("legacy", "deployed"), programs)}
    if args.legacy_trainer:
        historical = np.column_stack((angle, np.ones(len(test)), right-left))
        report["archived_python_cpp"] = verify_archived_python(args.legacy_trainer, args.legacy, historical)
    write_json(ROOT / "docs/tree2_evaluation.json", report)
    print(json.dumps(dict(test=report["test"], parity=report["parity"]), indent=2))


if __name__ == "__main__":
    main()
