"""Read the turn experiment triplets. Keep each complete acquisition in one split."""
from collections import Counter
import hashlib
from pathlib import Path
import re

import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "controldata"
MODEL = ROOT / "models/tree2_measured_v1.json"
RAW = ("timestamp_ms trial direction target_angle_deg phase trial_elapsed_ms straight_travel_mm "
       "straight_end_mm turn_angle_deg target_left_mm_s target_right_mm_s left_mm_s right_mm_s "
       "center_mm_s left_pwm right_pwm pwm_right_minus_left left_count right_count x_mm y_mm theta_rad").split()
SUMMARY = ("trial direction target_deg status straight_end_mm final_angle_deg angle_error_deg "
           "turn_sample_count mean_left_pwm mean_right_pwm mean_pwm_difference mean_center_mm_s").split()
BINS = "trial angle_bin_start_deg sample_count mean_left_pwm mean_right_pwm mean_pwm_difference".split()

# Preassigned target blocks include both directions and every repeat at each target.
VALIDATION = (15, 45, 80)
TEST = (20, 50, 75, 85)


def split_name(target):
    return "test" if target in TEST else "validation" if target in VALIDATION else "train"


def read_table(path, columns, empty_columns=()):
    frame = pd.read_csv(path)
    if list(frame.columns) != columns or frame.empty:
        raise ValueError(f"Unexpected schema or empty file: {path}")
    empty_bins = frame.sample_count.eq(0) if empty_columns else np.zeros(len(frame), dtype=bool)
    for column in columns:
        expected_missing = empty_bins if column in empty_columns else np.zeros(len(frame), dtype=bool)
        if not np.array_equal(frame[column].isna(), expected_missing):
            raise ValueError(f"Unexpected missing value in {path}: {column}")
        if column not in ("direction", "phase", "status"):
            if not np.isfinite(frame[column].dropna().to_numpy(dtype=float)).all():
                raise ValueError(f"Nonfinite number in {path}: {column}")
    return frame


def historical_response(raw, summary):
    """Original acquisition controller only. Observe through the first WAIT/DONE after braking."""
    results = []
    for trial, group in raw[raw.trial > 0].groupby("trial", sort=False):
        terminal = group[group.phase.isin(["WAIT", "DONE"])].iloc[0]
        settle = group[group.phase.eq("SETTLE")]
        trace = pd.concat([settle, terminal.to_frame().T])
        t = trace.timestamp_ms.to_numpy(dtype=float) - settle.timestamp_ms.iloc[0]
        error = trace.turn_angle_deg.to_numpy(dtype=float) - terminal.target_angle_deg
        # Last entry to +/-2 degrees, sustained for >=200 ms through the observation endpoint.
        outside = np.flatnonzero(np.abs(error) > 2)
        start = int(outside[-1] + 1) if len(outside) else 0
        settled = start < len(t) and t[-1] - t[start] >= 200
        # Count transitions between excursions above +0.36 and below -0.36 degrees.
        sides = np.sign(error[np.abs(error) > .36])
        reversals = int(np.count_nonzero(np.diff(sides)))
        results.append(dict(direction=terminal.direction, final_error=float(error[-1]),
                            peak_overshoot=max(0., float(error.max())),
                            settling_ms=float(t[start]) if settled else None,
                            crossings=reversals, window_ms=float(t[-1]),
                            terminal_vs_last_settle=float(error[-1] - error[-2])))
        if abs(terminal.turn_angle_deg - summary.loc[trial, "final_angle_deg"]) > .0011:
            raise ValueError("Summary final angle disagrees with first post-SETTLE sample")
    return results


def stats(values):
    a = np.asarray(values, dtype=float)
    return dict(min=float(a.min()), mean=float(a.mean()), median=float(np.median(a)),
                p95=float(np.quantile(a, .95)), max=float(a.max()))


def load_experiments(directory=DATA):
    files = {p.name: p for p in Path(directory).iterdir() if p.is_file()}
    digests = {n: hashlib.sha256(p.read_bytes()).hexdigest() for n, p in files.items()}
    copies, sessions = {}, {}
    for name, path in sorted(files.items()):
        canonical = re.sub(r"\(\d+\)(?=\.csv$)", "", name)
        if name != canonical:
            if digests.get(canonical) != digests[name]:
                raise ValueError(f"Conflicting copy: {name}")
            copies[name] = canonical
            continue
        match = re.fullmatch(r"(\d{8}_\d{6}_\d+_([LR])_(\d+))(_summary|_angle_bins)?\.csv", name)
        if not match:
            raise ValueError(f"Unrecognized experiment file: {name}")
        stem, direction, target, kind = match.groups()
        sessions.setdefault((stem, direction, int(target)), {})[kind or "raw"] = path
    frames, response, manifest, gaps = [], [], [], []
    phases, totals = Counter(), Counter()
    max_yaw_mismatch = 0.
    for (stem, direction, target), paths in sessions.items():
        raw = read_table(paths["raw"], RAW)
        summary = read_table(paths["_summary"], SUMMARY).set_index("trial", verify_integrity=True)
        bins = read_table(paths["_angle_bins"], BINS, BINS[3:])
        if (not raw.direction.eq(direction).all() or not raw.target_angle_deg.eq(target).all()
                or not summary.direction.eq(direction).all() or not summary.target_deg.eq(target).all()
                or not summary.status.eq("completed").all() or not raw.timestamp_ms.diff().dropna().gt(0).all()):
            raise ValueError(f"Inconsistent session identity, status or time: {stem}")
        if not (raw.right_pwm - raw.left_pwm).eq(raw.pwm_right_minus_left).all():
            raise ValueError(f"PWM difference mismatch: {stem}")
        wheels = raw[["left_pwm", "right_pwm"]].to_numpy()
        if np.any(np.abs(wheels) > 100) or np.any(wheels != np.trunc(wheels)):
            raise ValueError(f"Out-of-range or noninteger wheel PWM: {stem}")
        # Shift before phase filtering. First TURN uses the last STRAIGHT sample of this trial.
        raw[["prev_l", "prev_r"]] = raw.groupby("trial")[["left_pwm", "right_pwm"]].shift()
        turn = raw[raw.phase.eq("TURN")].copy()
        if turn[["prev_l", "prev_r"]].isna().any().any():
            raise ValueError(f"Missing causal command history: {stem}")
        by_trial = turn.groupby("trial")
        if not by_trial.size().equals(summary.turn_sample_count) or set(bins.trial) != set(summary.index):
            raise ValueError(f"Trial/count mismatch: {stem}")
        for src, dst in [("left_pwm", "mean_left_pwm"), ("right_pwm", "mean_right_pwm"),
                         ("pwm_right_minus_left", "mean_pwm_difference"), ("center_mm_s", "mean_center_mm_s")]:
            if not np.allclose(by_trial[src].mean(), summary[dst], atol=1e-8, rtol=0):
                raise ValueError(f"Inconsistent summary: {stem} {dst}")
        if not np.allclose(summary.angle_error_deg, summary.final_angle_deg - target, atol=1e-8, rtol=0):
            raise ValueError(f"Inconsistent final angle error: {stem}")
        indexed = bins.set_index(["trial", "angle_bin_start_deg"], verify_integrity=True)
        nonnegative = turn[turn.turn_angle_deg >= 0]
        binned = nonnegative.groupby([nonnegative.trial, np.floor(nonnegative.turn_angle_deg).astype(int)])
        counts = binned.size().reindex(indexed.index, fill_value=0)
        if not counts.equals(indexed.sample_count) or counts.sum() != len(nonnegative):
            raise ValueError(f"Inconsistent angle bins: {stem}")
        for src, dst in [("left_pwm", "mean_left_pwm"), ("right_pwm", "mean_right_pwm"),
                         ("pwm_right_minus_left", "mean_pwm_difference")]:
            means = binned[src].mean().reindex(indexed.index)
            if not np.allclose(means, indexed[dst], atol=1e-8, rtol=0, equal_nan=True):
                raise ValueError(f"Inconsistent angle-bin mean: {stem} {dst}")
        ccw = 1 if direction == "L" else -1
        if (turn[["target_left_mm_s", "target_right_mm_s"]] <= 0).any().any() or not (
                ccw * (turn.target_right_mm_s - turn.target_left_mm_s) > 0).all():
            raise ValueError(f"Unrecognized forward-turn protocol: {stem}")
        yaw_delta = np.rad2deg(by_trial.theta_rad.diff()) * ccw
        mismatch = (yaw_delta - by_trial.turn_angle_deg.diff()).abs().max()
        max_yaw_mismatch = max(max_yaw_mismatch, float(mismatch))
        turn["error_deg"] = -ccw * (target - turn.turn_angle_deg)
        turn["session"] = stem
        turn["split"] = split_name(target)
        frames.append(turn)
        response.extend(historical_response(raw, summary))
        gaps.extend(by_trial.timestamp_ms.diff().dropna().tolist())
        phases.update(raw.phase)
        totals.update(raw_rows=len(raw), summary_rows=len(summary), bin_rows=len(bins),
                      empty_bins=int(bins.sample_count.eq(0).sum()), bin_members=int(bins.sample_count.sum()),
                      negative_turn_progress=int(turn.turn_angle_deg.lt(0).sum()))
        manifest.append(dict(session=stem, split=split_name(target), trials=len(summary),
                             raw=len(raw), turn=len(turn), raw_sha256=digests[paths["raw"].name]))
    if not frames:
        raise ValueError("No complete experiment triplets")
    turns = pd.concat(frames, ignore_index=True)
    history = pd.DataFrame(response)
    motion = {}
    for direction, g in history.groupby("direction"):
        motion[direction] = dict(trials=len(g), final_error_deg=stats(g.final_error),
                                 peak_overshoot_deg=stats(g.peak_overshoot),
                                 settled_trials=int(g.settling_ms.notna().sum()),
                                 settling_ms=stats(g.settling_ms.dropna()) if g.settling_ms.notna().any() else None,
                                 crossing_trials=int(g.crossings.gt(0).sum()), max_crossings=int(g.crossings.max()),
                                 settle_window_ms=stats(g.window_ms))
    direction_checks = {}
    for direction, g in turns.groupby("direction"):
        trial = g.groupby(["session", "trial"])
        count_delta = ((trial.right_count.last() - trial.right_count.first())
                       - (trial.left_count.last() - trial.left_count.first()))
        direction_checks[direction] = dict(rows=len(g), mean_left_mm_s=float(g.left_mm_s.mean()),
                                           mean_right_mm_s=float(g.right_mm_s.mean()),
                                           turn_encoder_difference_range=[int(count_delta.min()), int(count_delta.max())],
                                           near_zero_command=stats(g.loc[g.error_deg.abs() <= 5, "pwm_right_minus_left"]))
    audit = dict(files=len(files), duplicates=copies, totals=dict(totals), phases=dict(phases),
                 sessions=manifest, turn_gap_ms=stats(gaps), yaw_increment_max_error_deg=max_yaw_mismatch,
                 terminal_vs_last_settle_different=int(history.terminal_vs_last_settle.abs().gt(.0011).sum()),
                 recorded_motion=motion, direction_checks=direction_checks, remaining_abs_deg=stats(turns.error_deg.abs()),
                 turn_pwm_by_direction={d: stats(g.pwm_right_minus_left) for d, g in turns.groupby("direction")},
                 measured_wrong_direction=int((turns.pwm_right_minus_left * turns.error_deg > 0).sum()))
    return turns, audit
