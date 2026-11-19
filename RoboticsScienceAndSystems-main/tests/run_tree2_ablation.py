"""Compare forward Tree2 input features on recorded turns and policies on eight Processing routes."""
import argparse
from collections import Counter
import csv
from hashlib import sha256
import json
import os
from pathlib import Path
import subprocess
import sys
import uuid

import numpy as np
from sklearn.tree import DecisionTreeRegressor
from run_processing_simulation_checks import ROOT, compile_processing_tests

sys.path.insert(0, str(ROOT / "tools"))
from export_processing_models import generate
from train_tree2 import angle_input, metrics, raw_prediction
from tree2_data import MODEL, load_experiments


def digest(path):
    return sha256(path.read_bytes()).hexdigest()


def compare_features():
    rows, _ = load_experiments()
    model = json.loads(MODEL.read_text(encoding="utf-8"))
    train = rows.split.eq("train").to_numpy()
    validation = rows.split.eq("validation").to_numpy()
    test = rows.split.eq("test").to_numpy()
    first = rows.groupby(["session", "trial"]).cumcount().eq(0).to_numpy()
    angles = angle_input(rows)
    previous = (rows.prev_r - rows.prev_l).to_numpy(dtype=np.float32)
    # Both experiment directions are forward arcs; the old q denotes motion mode, not left/right.
    inputs = np.column_stack((angles, np.ones_like(angles), previous))
    labels = rows.pwm_right_minus_left.to_numpy()

    def score(p):
        return {name: metrics(labels[mask], p[mask]) for name, mask in (
            ("validation", validation), ("test", test), ("test_first_turn_sample", test & first))}

    results = {"previous_command": score(previous)}
    for name, columns in (("angle", [0]), ("angle_mode", [0, 1]), ("angle_mode_history", [0, 1, 2])):
        estimator = DecisionTreeRegressor(max_depth=model["selected"]["depth"],
                                         min_samples_leaf=model["selected"]["min_samples_leaf"], random_state=42)
        x = inputs[:, columns]
        estimator.fit(x[train], labels[train])
        prediction = estimator.predict(x)
        if name in ("angle", "angle_mode"):
            np.testing.assert_allclose(prediction, raw_prediction(angles, model["leaves"]), rtol=0, atol=2e-6)
        results[name] = dict(leaves=int(estimator.get_n_leaves()), **score(prediction))
        d = np.arange(-200, 201)
        stress = {}
        for angle_deg in (-75, 75):
            probes = np.column_stack((np.full(len(d), np.deg2rad(angle_deg)), np.ones(len(d)), d))
            p = estimator.predict(probes[:, columns])
            stress[str(angle_deg)] = int(np.count_nonzero(p * angle_deg > 0))
        results[name]["opposite_at_75_of_401"] = stress
    return dict(train_rows=int(train.sum()), validation_rows=int(validation.sum()), test_rows=int(test.sum()),
                random_state=42, motion_signal=1, history="last recorded PWM difference within the trial, before TURN filtering",
                max_depth=model["selected"]["depth"], min_samples_leaf=model["selected"]["min_samples_leaf"],
                previous_difference_train_range=[int(previous[train].min()), int(previous[train].max())],
                results=results)


def check_raw_forward(java, classpath):
    leaves = json.loads(MODEL.read_text(encoding="utf-8"))["leaves"]
    edges = np.asarray([leaf["upper_rad"] for leaf in leaves[:-1]], dtype=np.float32)
    centers = np.asarray([leaf["center_rad"] for leaf in leaves], dtype=np.float32)
    angles = np.unique(np.concatenate((edges, np.nextafter(edges, np.float32(-np.inf)),
                                       np.nextafter(edges, np.float32(np.inf)), centers,
                                       np.deg2rad(np.array([-90, -75, 0, 75, 90], dtype=np.float32)))))
    arguments = [format(float(v), ".9g") for pair in zip(angles, raw_prediction(angles, leaves)) for v in pair]
    subprocess.run([str(java), "-Dtree2.legacy=true", "-cp", classpath, "Tree2AblationProbe", *arguments], check=True)
    return dict(angles=len(angles), previous_differences=[-200, 0, 200], cases=3 * len(angles))


def read_route(directory, row):
    path = directory / (row["route"] + ".csv")
    modes = Counter()
    first = None
    last_forward = None
    flips = pairs = 0
    max_abs_y = 0.0
    with path.open(newline="") as stream:
        for sample in csv.DictReader(stream):
            mode = sample["action"]
            left, right = int(sample["left_pwm"]), int(sample["right_pwm"])
            if first is None:
                first = [left, right]
            modes[mode] += 1
            diff = right - left
            if mode == "FORWARD" and last_forward is not None:
                pairs += 1
                flips += int(last_forward * diff < 0)
            last_forward = diff if mode == "FORWARD" else None
            if row["route"] == "straight":
                max_abs_y = max(max_abs_y, abs(float(sample["y_mm"])))
    if first is None:
        raise ValueError(f"Empty trace: {path}")
    result = dict(success=row["success"] == "true", terminal=row["terminal"] == "true",
                  seconds=float(row["seconds"]), goal_error_mm=float(row["goal_error_mm"]),
                  remaining_mm=float(row["remaining_mm"]), reason=row["reason"], actions=dict(modes),
                  first_pwm=first, forward_pairs=pairs, forward_sign_flips=flips, trace_sha256=digest(path))
    if row["route"] == "straight":
        result["max_abs_y_mm"] = max_abs_y
    return result


def run_group(java, classpath, legacy, directory):
    result = subprocess.run([str(java), "-Dtree2.legacy=" + str(legacy).lower(), "-cp", classpath,
                             "ProcessingSimulationTests", str(directory)])
    if result.returncode not in (0, 2):
        raise RuntimeError(f"Processing harness failed for {directory.name}: {result.returncode}")
    with (directory / "rollouts.csv").open(newline="") as stream:
        routes = {r["route"]: read_route(directory, r) for r in csv.DictReader(stream)}
    passed = sum(r["success"] for r in routes.values())
    if len(routes) != 8 or result.returncode != (0 if passed == len(routes) else 2):
        raise ValueError("Incomplete or inconsistent rollout summary")
    return dict(passed=passed, total=len(routes), harness_exit=result.returncode, routes=routes)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--processing-root", type=Path, default=ROOT/".tools/processing4/Processing")
    parser.add_argument("--report", type=Path, default=ROOT/"docs/tree2_ablation.json")
    args = parser.parse_args()
    # Each invocation keeps its own complete traces; publish the summary only after all arms finish.
    run_dir = ROOT / "build-host" / "tree2-ablation" / uuid.uuid4().hex
    run_dir.mkdir(parents=True)
    jdk, cp = compile_processing_tests(args.processing_root.resolve(), run_dir/"sketch")
    override = run_dir/"test-model"
    override.mkdir()
    generate(ablation_dir=override)
    subprocess.run([str(jdk/"bin/javac.exe"), "-d", str(override), str(override/"TreeModels.java"),
                    str(ROOT/"tests/Tree2AblationProbe.java")], check=True)
    test_cp = str(override) + os.pathsep + cp
    java = jdk/"bin/java.exe"
    probe = check_raw_forward(java, test_cp)
    groups = {}
    for name, classpath, legacy, forward, speed, reverse in (
        ("A", cp, True, "legacy", "legacy", "legacy"),
        ("B", test_cp, True, "raw CART", "legacy", "legacy"),
        ("C", cp, False, "interpolated CART with deadband and envelope", "measured", "guarded teacher"),
        ("C_legacy_reverse", test_cp, False, "interpolated CART with deadband and envelope", "measured", "legacy"),
    ):
        groups[name] = dict(forward=forward, forward_speed=speed, reverse=reverse,
                            **run_group(java, classpath, legacy, run_dir/name))
    # Check the isolating controls on entire time series, including mode and position.
    def trace(group, route):
        return groups[group]["routes"][route]["trace_sha256"]
    if trace("B", "reverse") != trace("A", "reverse"):
        raise AssertionError("Changing the forward tree altered the reverse-only reference route")
    for route, data in groups["C"]["routes"].items():
        if not data["actions"].get("REVERSE", 0) and trace("C", route) != trace("C_legacy_reverse", route):
            raise AssertionError(f"Reverse-only substitution changed {route} without a REVERSE decision")
    sources = [*sorted((ROOT/"DigitalTwin").glob("*.pde")), *sorted((ROOT/"DigitalTwin").glob("*.java")),
               ROOT/"tests/ProcessingSimulationTests.java", ROOT/"tests/Tree2AblationProbe.java",
               ROOT/"tests/run_processing_simulation_checks.py",
               Path(__file__).resolve(), ROOT/"tools/export_processing_models.py", ROOT/"tools/train_tree2.py",
               ROOT/"tools/tree2_data.py", ROOT/"StampC3_Template/Tree2MeasuredModel.h",
               ROOT/"StampC3_Template/Tree2ControlPolicy.h", ROOT/"StampC3_Template/Tree2Model.h",
               ROOT/"StampC3_Template/tree1_model.hpp", ROOT/"StampC3_Template/tree1_config.hpp",
               MODEL, ROOT/"docs/tree2_training.json"]
    report = dict(physics_step_ms=10, decision_step_ms=50, timeout_s=300, goal_tolerance_mm=5,
                  raw_forward_probe=probe, feature_ablation=compare_features(),
                  model="measured-forward-v1", groups=groups,
                  source_sha256={p.relative_to(ROOT).as_posix(): digest(p) for p in sources},
                  test_class_source_sha256=digest(override/"TreeModels.java"))
    destination = args.report.resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(destination.name + "." + uuid.uuid4().hex + ".tmp")
    temporary.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8", newline="\n")
    temporary.replace(destination)
    print("Passed routes:", {name: g["passed"] for name, g in groups.items()})
    print("Traces:", run_dir)
    print("Report:", destination)


if __name__ == "__main__":
    main()
