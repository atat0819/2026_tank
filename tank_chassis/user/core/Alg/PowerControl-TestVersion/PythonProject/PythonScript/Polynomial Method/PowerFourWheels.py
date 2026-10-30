"""Fit the firmware's six per-motor coefficients from 15-channel VOFA captures.

P_total = 4*K0 + K1*sum(I) + K2*sum(abs(w)) + K3*sum(I*w)
        + K4*sum(I**2) + K5*sum(w**2).
Inputs: I0=measured total W; I1/3/5/7=motor A; I2/4/6/8=rotor rad/s.
Use: python PowerFourWheels.py capture1.csv capture2.csv --output-dir results
Optional --file-weights assigns a per-row weight to each CSV; all default to 1.
Requires numpy, pandas, matplotlib. Does not edit firmware or input CSVs.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re

import numpy as np
import pandas as pd


def build_features(current, speed):
    return np.column_stack([
        np.full(len(current), 4.0), current.sum(axis=1), np.abs(speed).sum(axis=1),
        (current * speed).sum(axis=1), (current**2).sum(axis=1), (speed**2).sum(axis=1),
    ])


def least_squares(matrix, power, weights=None):
    if weights is not None:
        weights = np.asarray(weights, dtype=float)
        if (weights.shape != (len(power),) or not np.isfinite(weights).all()
                or np.any(weights < 0) or not np.any(weights > 0)):
            raise ValueError("weights must be finite, nonnegative, nonzero and match sample count")
        root_weight = np.sqrt(weights)
        matrix = matrix * root_weight[:, None]
        power = power * root_weight
    # Column scaling improves numerical conditioning without changing the objective.
    scale = np.sqrt(np.mean(matrix**2, axis=0))
    if np.any(scale == 0):
        raise ValueError("Insufficient excitation: feature matrix rank is below 6")
    scaled = matrix / scale
    coefficients, _, rank, singular_values = np.linalg.lstsq(scaled, power, rcond=None)
    if rank != 6:
        raise ValueError(f"Insufficient excitation: feature matrix rank {rank}, expected 6")
    return coefficients / scale, {
        "rank": int(rank), "scaled_condition_number": float(singular_values[0] / singular_values[-1]),
    }


def metrics(actual, predicted):
    error = predicted - actual
    total = np.sum((actual - actual.mean())**2)
    return {"rmse_w": float(np.sqrt(np.mean(error**2))),
            "mae_w": float(np.mean(abs(error))), "bias_prediction_minus_actual_w": float(error.mean()),
            "r2": float(1 - np.sum(error**2) / total) if total else None,
            "absolute_error_p95_w": float(np.quantile(abs(error), .95)),
            "absolute_error_max_w": float(np.max(abs(error)))}


def load_capture(path):
    frame = pd.read_csv(path)
    if list(frame.columns) != [f"I{i}" for i in range(15)]:
        raise ValueError(f"{path}: expected VOFA columns I0..I14")
    raw = frame.apply(pd.to_numeric, errors="coerce").to_numpy(dtype=float)
    finite = np.isfinite(raw).all(axis=1)
    selected = raw[finite & (raw[:, 13] == 15)]
    if len(selected) == 0:
        raise ValueError(f"{path}: no finite samples with all four motors online")
    sequence = raw[np.isfinite(raw[:, 14]), 14]
    increments = np.diff(sequence)
    if np.any(increments <= 0) or np.any(np.diff(selected[:, 10]) <= 0):
        raise ValueError(f"{path}: repeated/reset sequence or timestamp; split the capture first")
    audit = {
        "path": str(path.resolve()), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "rows_total": len(raw), "rows_used": len(selected),
        "nonfinite_rows": int((~finite).sum()), "offline_rows": int((finite & (raw[:, 13] != 15)).sum()),
        "negative_power_rows": int((selected[:, 0] < 0).sum()),
        "sequence_gap_count": int((increments > 1).sum()),
        "missing_sequence_count": int(np.maximum(increments - 1, 0).sum()),
        "time_start_s": float(selected[0, 10]), "time_end_s": float(selected[-1, 10]),
        "power_range_w": [float(selected[:, 0].min()), float(selected[:, 0].max())],
        "current_range_a": [float(selected[:, [1, 3, 5, 7]].min()), float(selected[:, [1, 3, 5, 7]].max())],
        "rotor_speed_range_rad_s": [float(selected[:, [2, 4, 6, 8]].min()), float(selected[:, [2, 4, 6, 8]].max())],
    }
    return {"data": selected, "power": selected[:, 0],
            "matrix": build_features(selected[:, [1, 3, 5, 7]], selected[:, [2, 4, 6, 8]]), "audit": audit}


def read_old_coefficients():
    root = next(p for p in Path(__file__).resolve().parents if (p / "RtosTask/can_send_task.cpp").is_file())
    source = (root / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8")
    body = re.search(r"float poly_coeffs\[6\]\s*=\s*\{(.*?)\}", source, re.S).group(1)
    body = re.sub(r"//[^\n]*", "", body)
    return np.array([float(value.strip().rstrip("fF")) for value in body.split(",") if value.strip()])


def plot_results(captures, coefficients, destination):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, len(captures), figsize=(7*len(captures), 7), squeeze=False)
    for index, capture in enumerate(captures):
        data, actual = capture["data"], capture["power"]
        predicted = capture["matrix"] @ coefficients
        cuts = np.r_[0, np.flatnonzero(np.diff(data[:, 14]) != 1) + 1, len(data)]
        for part, (start, end) in enumerate(zip(cuts[:-1], cuts[1:])):
            x = np.arange(start, end)
            axes[0, index].plot(x, actual[start:end], color="#303030", lw=.8,
                                label="Measured" if part == 0 else None)
            axes[0, index].plot(x, predicted[start:end], color="#d95f02", lw=.8, alpha=.8,
                                label="Joint least-squares fit" if part == 0 else None)
            axes[1, index].plot(x, predicted[start:end]-actual[start:end], color="#0072b2", lw=.7)
        for gap in cuts[1:-1]:
            for row in range(2):
                axes[row, index].axvline(gap, color="#999999", lw=.8, ls="--")
        axes[0, index].set_title(f"Capture {index+1}: RMSE = {metrics(actual, predicted)['rmse_w']:.3f} W")
        axes[0, index].set_ylabel("Total input power (W)")
        axes[0, index].legend()
        axes[1, index].axhline(0, color="#555555", lw=.7)
        axes[1, index].set_ylabel("Predicted - measured (W)")
        axes[1, index].set_xlabel("Recorded sample index (sequence gaps marked)")
        for row in range(2):
            axes[row, index].grid(alpha=.2)
    fig.tight_layout()
    fig.savefig(destination, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="+", type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--file-weights", nargs="+", type=float,
                        help="Per-row weight for each CSV, in input order (default: all 1)")
    args = parser.parse_args()
    captures = [load_capture(path) for path in args.csv]
    file_weights = args.file_weights if args.file_weights is not None else [1.0]*len(captures)
    if len(file_weights) != len(captures):
        parser.error("--file-weights must contain one weight per CSV")
    sample_weights = np.repeat(file_weights, [len(c["power"]) for c in captures])
    matrix = np.concatenate([c["matrix"] for c in captures])
    power = np.concatenate([c["power"] for c in captures])
    coefficients, diagnostics = least_squares(matrix, power, weights=sample_weights)
    old = read_old_coefficients()
    result = {"model": "4*K0 + K1*sum(I) + K2*sum(abs(w)) + K3*sum(I*w) + K4*sum(I^2) + K5*sum(w^2)",
              "method": "least squares on raw rows with explicit file weights; no smoothing, outlier removal, delay correction or sign constraints",
              "per_row_file_weights": file_weights,
              "coefficients_per_motor": coefficients.tolist(), "old_coefficients": old.tolist(),
              "rows_used": len(power), **diagnostics,
              "joint_fit": metrics(power, matrix @ coefficients), "old_model": metrics(power, matrix @ old),
              "float32_coefficient_metrics": metrics(power, matrix @ coefficients.astype(np.float32).astype(float)),
              "captures": []}
    for index, capture in enumerate(captures):
        x, y, data = capture["matrix"], capture["power"], capture["data"]
        individual, individual_diag = least_squares(x, y)
        item = {**capture["audit"], "per_row_weight": file_weights[index],
                "normalized_objective_share": float(len(y)*file_weights[index]/sample_weights.sum()),
                "joint_fit": metrics(y, x @ coefficients),
                "old_model": metrics(y, x @ old), "individual_coefficients": individual.tolist(),
                "individual_fit": metrics(y, x @ individual), **individual_diag,
                "old_telemetry_max_difference_w": float(np.max(abs(data[:, 9] - x @ old))),
                "meter_power_minus_voltage_current_max_w": float(np.max(abs(y - data[:, 11]*data[:, 12])))}
        if len(captures) > 1:
            train = [c for j, c in enumerate(captures) if j != index]
            train_weights = np.repeat([w for j, w in enumerate(file_weights) if j != index],
                                      [len(c["power"]) for c in train])
            held_out, _ = least_squares(np.concatenate([c["matrix"] for c in train]),
                                       np.concatenate([c["power"] for c in train]), weights=train_weights)
            item["held_out_file_validation"] = metrics(y, x @ held_out)
        result["captures"].append(item)
    labels = ["K0", "K1", "K2, fabsf(w)", "K3", "K4", "K5"]
    snippet = "float poly_coeffs[6] = {\n" + "\n".join(
        f"    {value:.12f}f{',' if i < 5 else ' ' }  // {labels[i]}"
        for i, value in enumerate(coefficients)) + "\n};\n"
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "poly_coeffs.txt").write_text(snippet, encoding="utf-8")
    (args.output_dir / "fit_results.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    plot_results(captures, coefficients, args.output_dir / "fit_comparison.png")
    print(snippet)
    print(json.dumps({key: result[key] for key in ["rows_used", "rank", "scaled_condition_number", "joint_fit", "old_model"]}, indent=2))
    print("Results:", args.output_dir.resolve())


if __name__ == "__main__":
    main()
