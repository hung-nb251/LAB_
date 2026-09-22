#!/usr/bin/env python3
"""Train a joint XYZ GRU from relative robot end-effector trajectories.

The training representation matches the co-carry predictor contract:

* each CSV trial is processed independently;
* robot EE positions are uniformly resampled;
* positions are relative to the first resampled EE pose;
* velocity features are per-sample displacements ``p[k] - p[k-1]``;
* a causal window ending at k predicts position at k + horizon.

Only GROUND_TRUTH trials are used. Existing model directories are never
overwritten.
"""

import argparse
import json
import os
import pickle
import random
import time
from pathlib import Path

os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")
os.environ.setdefault("TF_ENABLE_ONEDNN_OPTS", "0")

import numpy as np
import pandas as pd
import sklearn
import tensorflow as tf
from sklearn.preprocessing import MinMaxScaler


POSITION_COLS = ["robot_ee_x", "robot_ee_y", "robot_ee_z"]
FEATURE_NAMES = ["x", "y", "z", "vx", "vy", "vz"]
OUTPUT_NAMES = ["x", "y", "z"]
TIME_COL = "wall_time"
DEFAULT_DATA_DIR = Path("/home/hungnb/simulation_hri/cocarry_logs")
DEFAULT_OUTPUT_DIR = (
    Path(__file__).resolve().parent.parent
    / "pHRI_Models"
    / "gru_robot_ee_h5_relative_v1"
)
DEFAULT_EXCLUDED_FILES = [
    "Long/experiment_GROUND_TRUTH_20260616_160434.csv",
]


def parse_csv_list(value: str, cast):
    values = [cast(item.strip()) for item in value.split(",") if item.strip()]
    if not values:
        raise argparse.ArgumentTypeError("Danh sách không được rỗng")
    return values


def parse_args():
    parser = argparse.ArgumentParser(
        description="Train joint XYZ GRU from robot_ee GROUND_TRUTH logs"
    )
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA_DIR)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--sample-rate", type=float, default=15.0)
    parser.add_argument("--window-size", type=int, default=20)
    parser.add_argument("--horizon", type=int, default=5)
    parser.add_argument("--layers", default="1,2,3")
    parser.add_argument("--units", default="32,64")
    parser.add_argument("--seeds", default="42,43,44")
    parser.add_argument("--dropout", type=float, default=0.2)
    parser.add_argument("--learning-rate", type=float, default=0.001)
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--epochs", type=int, default=200)
    parser.add_argument("--patience", type=int, default=20)
    parser.add_argument("--lr-patience", type=int, default=10)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    args.layers = parse_csv_list(args.layers, int)
    args.units = parse_csv_list(args.units, int)
    args.seeds = parse_csv_list(args.seeds, int)
    return args


def validate_args(args):
    positive = {
        "sample_rate": args.sample_rate,
        "window_size": args.window_size,
        "horizon": args.horizon,
        "learning_rate": args.learning_rate,
        "batch_size": args.batch_size,
        "epochs": args.epochs,
        "patience": args.patience,
        "lr_patience": args.lr_patience,
    }
    invalid = [name for name, value in positive.items() if value <= 0]
    if invalid:
        raise ValueError(f"Các tham số phải > 0: {', '.join(invalid)}")
    if any(value <= 0 for value in args.layers + args.units):
        raise ValueError("layers và units phải > 0")
    if not 0.0 <= args.dropout < 1.0:
        raise ValueError("dropout phải thuộc [0, 1)")


def set_seed(seed: int):
    os.environ["PYTHONHASHSEED"] = str(seed)
    random.seed(seed)
    np.random.seed(seed)
    tf.keras.utils.set_random_seed(seed)


def load_manifest(data_dir: Path, split: str):
    manifest_path = data_dir / f"{split}_file_list.json"
    with manifest_path.open("r", encoding="utf-8") as stream:
        entries = json.load(stream)
    excluded = set(DEFAULT_EXCLUDED_FILES)
    selected = [
        name for name in entries
        if "GROUND_TRUTH" in Path(name).name.upper() and name not in excluded
    ]
    if not selected:
        raise ValueError(f"Không có trial Ground Truth hợp lệ trong {manifest_path}")
    return selected


def read_trajectory(path: Path, sample_rate_hz: float):
    frame = pd.read_csv(path, usecols=[TIME_COL, *POSITION_COLS])
    raw_rows = len(frame)
    frame[TIME_COL] = pd.to_datetime(frame[TIME_COL], errors="coerce")
    for column in POSITION_COLS:
        frame[column] = pd.to_numeric(frame[column], errors="coerce")
    frame = frame.dropna().sort_values(TIME_COL)
    frame = frame.drop_duplicates(subset=TIME_COL, keep="last")
    if len(frame) < 2:
        raise ValueError(f"Trial có ít hơn hai mẫu hợp lệ: {path}")

    times = (
        frame[TIME_COL] - frame[TIME_COL].iloc[0]
    ).dt.total_seconds().to_numpy(dtype=np.float64)
    duration = float(times[-1])
    period = 1.0 / sample_rate_hz
    if duration < period:
        raise ValueError(f"Trial quá ngắn ({duration:.3f}s): {path}")

    positions = frame[POSITION_COLS].to_numpy(dtype=np.float64)
    sample_times = np.arange(0.0, duration + period * 0.25, period)
    sampled = np.column_stack([
        np.interp(sample_times, times, positions[:, axis])
        for axis in range(3)
    ])
    sampled -= sampled[0]
    return sampled, {
        "raw_rows": raw_rows,
        "valid_rows": len(frame),
        "duration_s": duration,
        "resampled_points": len(sampled),
        "path_length_m": float(np.linalg.norm(np.diff(sampled, axis=0), axis=1).sum()),
    }


def add_delta_features(positions: np.ndarray):
    delta = np.zeros_like(positions)
    delta[1:] = positions[1:] - positions[:-1]
    return np.hstack((positions, delta))


def make_windows(positions: np.ndarray, window_size: int, horizon: int):
    features = add_delta_features(positions)
    inputs = []
    targets = []
    for index in range(len(positions) - horizon):
        start = max(0, index - window_size + 1)
        window = features[start:index + 1]
        missing = window_size - len(window)
        if missing:
            window = np.vstack((np.repeat(features[[0]], missing, axis=0), window))
        inputs.append(window)
        targets.append(positions[index + horizon])
    return np.asarray(inputs, dtype=np.float32), np.asarray(targets, dtype=np.float32)


def prepare_split(data_dir, files, sample_rate, window_size, horizon):
    all_inputs = []
    all_targets = []
    trial_stats = {}
    for relative_name in files:
        path = data_dir / relative_name
        if not path.is_file():
            raise FileNotFoundError(f"Không tìm thấy trial: {path}")
        positions, stats = read_trajectory(path, sample_rate)
        inputs, targets = make_windows(positions, window_size, horizon)
        if not len(inputs):
            raise ValueError(f"Trial không đủ dài để tạo window: {path}")
        all_inputs.append(inputs)
        all_targets.append(targets)
        trial_stats[relative_name] = stats

    return (
        np.concatenate(all_inputs),
        np.concatenate(all_targets),
        {
            "files": len(files),
            "windows": int(sum(len(item) for item in all_inputs)),
            "resampled_points": int(sum(
                item["resampled_points"] for item in trial_stats.values()
            )),
            "duration_s": float(sum(
                item["duration_s"] for item in trial_stats.values()
            )),
            "trials": trial_stats,
        },
    )


def fit_scalers(inputs, targets):
    scaler_x = {}
    for index, name in enumerate(FEATURE_NAMES):
        scaler_x[name] = MinMaxScaler().fit(
            inputs[:, :, index].reshape(-1, 1)
        )
    scaler_y = {}
    for index, name in enumerate(OUTPUT_NAMES):
        scaler_y[name] = MinMaxScaler().fit(targets[:, index].reshape(-1, 1))
    return scaler_x, scaler_y


def scale_inputs(values, scalers):
    scaled = np.empty_like(values, dtype=np.float32)
    for index, name in enumerate(FEATURE_NAMES):
        scaled[:, :, index] = scalers[name].transform(
            values[:, :, index].reshape(-1, 1)
        ).reshape(values.shape[0], values.shape[1])
    return scaled


def scale_targets(values, scalers):
    scaled = np.empty_like(values, dtype=np.float32)
    for index, name in enumerate(OUTPUT_NAMES):
        scaled[:, index] = scalers[name].transform(
            values[:, index].reshape(-1, 1)
        ).ravel()
    return scaled


def inverse_targets(values, scalers):
    restored = np.empty_like(values, dtype=np.float64)
    for index, name in enumerate(OUTPUT_NAMES):
        restored[:, index] = scalers[name].inverse_transform(
            values[:, index].reshape(-1, 1)
        ).ravel()
    return restored


def build_model(window_size, layers, units, dropout, learning_rate):
    inputs = tf.keras.Input(shape=(window_size, len(FEATURE_NAMES)), name="input_sequence")
    value = inputs
    for index in range(layers):
        value = tf.keras.layers.GRU(
            units,
            activation="tanh",
            return_sequences=index < layers - 1,
            name=f"gru_{index + 1}",
        )(value)
        if dropout:
            value = tf.keras.layers.Dropout(dropout, name=f"dropout_{index + 1}")(value)
    outputs = tf.keras.layers.Dense(3, name="position_output")(value)
    model = tf.keras.Model(inputs, outputs, name=f"gru_joint_{layers}l_{units}u")
    model.compile(
        optimizer=tf.keras.optimizers.Adam(learning_rate),
        loss="mse",
        metrics=["mae"],
    )
    return model


def metrics(targets, predictions):
    error = predictions - targets
    return {
        "rmse_m": np.sqrt(np.mean(np.square(error), axis=0)).tolist(),
        "mae_m": np.mean(np.abs(error), axis=0).tolist(),
        "mean_euclidean_error_m": float(np.mean(np.linalg.norm(error, axis=1))),
        "p95_euclidean_error_m": float(np.quantile(np.linalg.norm(error, axis=1), 0.95)),
    }


def print_metrics(label, values):
    rmse = np.asarray(values["rmse_m"]) * 1000.0
    mean_3d = values["mean_euclidean_error_m"] * 1000.0
    p95_3d = values["p95_euclidean_error_m"] * 1000.0
    print(
        f"{label}: RMSE XYZ=[{rmse[0]:.2f}, {rmse[1]:.2f}, {rmse[2]:.2f}] mm | "
        f"mean3D={mean_3d:.2f} mm | p95={p95_3d:.2f} mm",
        flush=True,
    )


def train_search(args, train_data, val_data, val_targets, scaler_y):
    x_train, y_train = train_data
    x_val, y_val = val_data
    results = []
    candidates = {}

    for layers in args.layers:
        for units in args.units:
            key = (layers, units)
            candidates[key] = []
            for seed in args.seeds:
                tf.keras.backend.clear_session()
                set_seed(seed)
                model = build_model(
                    args.window_size, layers, units, args.dropout, args.learning_rate
                )
                callbacks = [
                    tf.keras.callbacks.EarlyStopping(
                        monitor="val_loss",
                        patience=args.patience,
                        restore_best_weights=True,
                        min_delta=1e-6,
                    ),
                    tf.keras.callbacks.ReduceLROnPlateau(
                        monitor="val_loss",
                        factor=0.5,
                        patience=args.lr_patience,
                        min_lr=1e-6,
                    ),
                ]
                started = time.perf_counter()
                history = model.fit(
                    x_train,
                    y_train,
                    validation_data=(x_val, y_val),
                    batch_size=args.batch_size,
                    epochs=args.epochs,
                    callbacks=callbacks,
                    verbose=0,
                    shuffle=True,
                )
                elapsed = time.perf_counter() - started
                prediction = inverse_targets(model.predict(x_val, verbose=0), scaler_y)
                val_result = metrics(val_targets, prediction)
                record = {
                    "layers": layers,
                    "units": units,
                    "seed": seed,
                    "epochs_ran": len(history.history["loss"]),
                    "best_epoch": int(np.argmin(history.history["val_loss"]) + 1),
                    "best_val_scaled_mse": float(min(history.history["val_loss"])),
                    "validation_metrics": val_result,
                    "training_time_s": elapsed,
                }
                results.append(record)
                candidates[key].append((record, model.get_weights()))
                print_metrics(f"GRU {layers}L/{units}U seed={seed}", val_result)

    config_scores = {}
    for key, runs in candidates.items():
        config_scores[key] = float(np.mean([
            run[0]["validation_metrics"]["mean_euclidean_error_m"]
            for run in runs
        ]))
    selected_key = min(config_scores, key=config_scores.get)
    selected_record, selected_weights = min(
        candidates[selected_key],
        key=lambda run: run[0]["validation_metrics"]["mean_euclidean_error_m"],
    )
    tf.keras.backend.clear_session()
    final_model = build_model(
        args.window_size,
        selected_key[0],
        selected_key[1],
        args.dropout,
        args.learning_rate,
    )
    final_model.set_weights(selected_weights)
    return final_model, selected_record, results, config_scores


def save_json(path: Path, value):
    with path.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write("\n")


def main():
    args = parse_args()
    validate_args(args)
    data_dir = args.data_dir.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    if not data_dir.is_dir():
        raise FileNotFoundError(f"Không tìm thấy data directory: {data_dir}")
    if output_dir.exists() and not args.dry_run:
        raise FileExistsError(f"Từ chối ghi đè model directory: {output_dir}")

    manifests = {
        split: load_manifest(data_dir, split)
        for split in ("train", "val", "test")
    }
    prepared = {}
    split_stats = {}
    print("Preparing robot_ee GROUND_TRUTH trajectories...", flush=True)
    for split in ("train", "val", "test"):
        x_values, y_values, stats = prepare_split(
            data_dir,
            manifests[split],
            args.sample_rate,
            args.window_size,
            args.horizon,
        )
        prepared[split] = (x_values, y_values)
        split_stats[split] = stats
        print(
            f"{split:5s}: {stats['files']} files | {stats['resampled_points']} points | "
            f"{stats['windows']} windows | {stats['duration_s']:.1f}s",
            flush=True,
        )
    if args.dry_run:
        print("Dry-run complete; no model or files were written.")
        return

    scaler_x, scaler_y = fit_scalers(*prepared["train"])
    scaled = {
        split: (
            scale_inputs(prepared[split][0], scaler_x),
            scale_targets(prepared[split][1], scaler_y),
        )
        for split in prepared
    }
    model, selected, search_results, config_scores = train_search(
        args,
        scaled["train"],
        scaled["val"],
        prepared["val"][1],
        scaler_y,
    )

    val_prediction = inverse_targets(model.predict(scaled["val"][0], verbose=0), scaler_y)
    test_started = time.perf_counter()
    test_prediction = inverse_targets(
        model.predict(scaled["test"][0], verbose=0, batch_size=1), scaler_y
    )
    test_elapsed = time.perf_counter() - test_started
    val_metrics = metrics(prepared["val"][1], val_prediction)
    test_metrics = metrics(prepared["test"][1], test_prediction)
    baseline_metrics = metrics(
        prepared["test"][1], prepared["test"][0][:, -1, :3]
    )
    print("\nSelected configuration:", selected, flush=True)
    print_metrics("Validation", val_metrics)
    print_metrics("Test GRU", test_metrics)
    print_metrics("Test constant-position sanity baseline", baseline_metrics)

    output_dir.mkdir(parents=True, exist_ok=False)
    model_filename = f"gru_joint_Ts{args.horizon}.keras"
    model.save(output_dir / model_filename)
    with (output_dir / "scaler_x.pkl").open("wb") as stream:
        pickle.dump(scaler_x, stream)
    with (output_dir / "scaler_y.pkl").open("wb") as stream:
        pickle.dump(scaler_y, stream)
    for split, files in manifests.items():
        save_json(output_dir / f"{split}_file_list.json", files)
    save_json(output_dir / "search_results.json", search_results)

    metadata = {
        "model": "Keras joint XYZ GRU",
        "model_file": model_filename,
        "source_columns": POSITION_COLS,
        "feature_names": FEATURE_NAMES,
        "output_names": OUTPUT_NAMES,
        "coordinate_system": "base_link displacement relative to first EE pose",
        "velocity_definition": "per-sample displacement delta_p[k] = p[k] - p[k-1]",
        "ground_truth_only": True,
        "excluded_files": DEFAULT_EXCLUDED_FILES,
        "sample_rate_hz": args.sample_rate,
        "window_size": args.window_size,
        "horizon_steps": args.horizon,
        "prediction_horizon_s": args.horizon / args.sample_rate,
        "split_strategy": "existing file-level manifests; not subject-held-out",
        "split_stats": split_stats,
        "search": {
            "layers": args.layers,
            "units": args.units,
            "seeds": args.seeds,
            "dropout": args.dropout,
            "learning_rate": args.learning_rate,
            "batch_size": args.batch_size,
            "max_epochs": args.epochs,
            "patience": args.patience,
            "selection_metric": "mean validation Euclidean position error",
            "mean_validation_error_by_config_m": {
                f"{key[0]}L_{key[1]}U": value
                for key, value in config_scores.items()
            },
        },
        "selected_run": selected,
        "validation_metrics": val_metrics,
        "test_metrics": test_metrics,
        "test_constant_position_sanity_baseline": baseline_metrics,
        "batch1_test_inference_ms_mean": test_elapsed / len(test_prediction) * 1000.0,
        "tensorflow_version": tf.__version__,
        "sklearn_version": sklearn.__version__,
    }
    save_json(output_dir / "metadata.json", metadata)
    print(f"\nSaved new GRU artifact: {output_dir}", flush=True)


if __name__ == "__main__":
    main()
