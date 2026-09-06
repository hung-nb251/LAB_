#!/usr/bin/env python3
"""
Inference Worker — runs TensorFlow in a COMPLETELY ISOLATED Python process.
Launched by predictor_node using the venv Python executable.
Communicates via stdin/stdout with JSON lines.
"""
import os
import sys
import json
import pickle
import time
import traceback

# These env vars are already set, but reinforce them
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "3"
os.environ["TF_ENABLE_ONEDNN_OPTS"] = "0"
os.environ["PYTHONHASHSEED"] = "0"

import numpy as np

try:
    from .svgp_numpy import SVGPNumpyRunner
except ImportError:
    # inference_worker.py is also launched directly as a script by the ROS node.
    from svgp_numpy import SVGPNumpyRunner

_TF = None


def _load_tensorflow():
    """Load TensorFlow only for Keras/TFLite/GPflow-compatible backends."""
    global _TF
    if _TF is None:
        import tensorflow as tf
        try:
            tf.config.threading.set_intra_op_parallelism_threads(1)
            tf.config.threading.set_inter_op_parallelism_threads(1)
        except RuntimeError:
            # GPflow may already have initialized TensorFlow in a prior model.
            pass
        _TF = tf
    return _TF

def _load_pickle(path):
    if not os.path.exists(path):
        return None
    with open(path, 'rb') as f:
        return pickle.load(f)


def send_response(data):
    """Send JSON response to stdout (read by ROS node)."""
    line = json.dumps(data)
    sys.stdout.write(line + "\n")
    sys.stdout.flush()


class TFLiteRunner:
    """Small predict_on_batch adapter shared with the Keras inference path."""

    def __init__(self, model_path, window_size, num_features):
        tf = _load_tensorflow()
        self._interpreter = tf.lite.Interpreter(
            model_path=model_path,
            num_threads=1,
        )
        self._interpreter.allocate_tensors()
        inputs = self._interpreter.get_input_details()
        outputs = self._interpreter.get_output_details()
        if len(inputs) != 1 or len(outputs) != 1:
            raise ValueError('TFLite predictor must have one input and one output')
        self._input_index = inputs[0]['index']
        self._output_index = outputs[0]['index']
        expected = (1, int(window_size), int(num_features))
        actual = tuple(int(value) for value in inputs[0]['shape'])
        if actual != expected:
            raise ValueError(
                f'TFLite input shape {actual} does not match runtime {expected}')

    def predict_on_batch(self, input_batch):
        values = np.asarray(input_batch, dtype=np.float32)
        self._interpreter.set_tensor(self._input_index, values)
        self._interpreter.invoke()
        return self._interpreter.get_tensor(self._output_index)


def main():
    # Read config from stdin (first line)
    config_line = sys.stdin.readline().strip()
    config = json.loads(config_line)

    model_dir = config["model_dir"]
    model_files = config["model_files"]
    scaler_x_file = config["scaler_x_file"]
    scaler_y_file = config["scaler_y_file"]
    default_model = config["default_model"]
    num_features = config.get("num_features", 3)
    window_size = config.get("window_size", 20)

    # Load scalers
    scaler_x = _load_pickle(os.path.join(model_dir, scaler_x_file))
    scaler_y = _load_pickle(os.path.join(model_dir, scaler_y_file))

    if scaler_x is None or scaler_y is None:
        send_response({"type": "ready", "success": False,
                        "message": f"Scalers not found in {model_dir}"})
        return

    # Scaler pickle compatibility is required by every backend. Heavy model
    # frameworks are imported later only when the selected artifact needs them.
    try:
        import sklearn
        send_response({
            "type": "info",
            "message": f"sklearn {sklearn.__version__} loaded; backend dependencies are lazy",
        })
    except ImportError as ie:
        send_response({"type": "ready", "success": False,
                        "message": f"Dependency missing: {ie}"})
        return
    except Exception as e:
        send_response({"type": "ready", "success": False,
                        "message": f"Initialization failed: {e}\\n{traceback.format_exc()}"})
        return

    current_model = None
    current_model_name = ""

    def do_load_model(name):
        nonlocal current_model, current_model_name
        name = name.lower().strip()
        if name not in model_files:
            return False, f"Unknown model '{name}'. Available: {list(model_files.keys())}"
        path = os.path.join(model_dir, model_files[name])
        fallback_message = ''
        if not os.path.exists(path) and path.endswith('.npz'):
            pickle_fallback = os.path.splitext(path)[0] + '.pkl'
            if os.path.exists(pickle_fallback):
                path = pickle_fallback
                fallback_message = ' (NPZ missing; using PKL fallback)'
        if not os.path.exists(path):
            return False, f"File not found: {path}"
        try:
            if path.endswith('.npz'):
                current_model = SVGPNumpyRunner(
                    path,
                    input_dim=window_size * num_features,
                    output_dim=3,
                )
                current_model_name = name
                dummy = np.zeros(
                    (1, window_size * num_features), dtype=np.float64)
                current_model.predict_f(dummy)
                return True, f"NumPy SVGP model '{name}' loaded OK"
            elif path.endswith('.pkl'):
                import gpflow
                with open(path, 'rb') as f:
                    current_model = pickle.load(f)
                current_model_name = name
                # Warm-up inference để xác nhận model OK
                dummy = np.zeros((1, window_size * num_features), dtype=np.float64)
                current_model.predict_f(dummy)
                return True, f"GPflow model '{name}' loaded OK{fallback_message}"
            elif path.endswith('.tflite'):
                current_model = TFLiteRunner(path, window_size, num_features)
                current_model_name = name
                dummy = np.zeros(
                    (1, window_size, num_features), dtype=np.float32)
                current_model.predict_on_batch(dummy)
                return True, f"TFLite model '{name}' loaded OK"
            else:
                tf = _load_tensorflow()

                # Create a compatibility wrapper for Dense that strips new kwargs
                # (e.g. quantization_config) not recognized by older model configs
                class CompatDense(tf.keras.layers.Dense):
                    def __init__(self, *args, **kwargs):
                        kwargs.pop('quantization_config', None)
                        super().__init__(*args, **kwargs)

                class CompatGRU(tf.keras.layers.GRU):
                    def __init__(self, *args, **kwargs):
                        kwargs.pop('quantization_config', None)
                        super().__init__(*args, **kwargs)

                class CompatLSTM(tf.keras.layers.LSTM):
                    def __init__(self, *args, **kwargs):
                        kwargs.pop('quantization_config', None)
                        super().__init__(*args, **kwargs)

                class CompatSimpleRNN(tf.keras.layers.SimpleRNN):
                    def __init__(self, *args, **kwargs):
                        kwargs.pop('quantization_config', None)
                        super().__init__(*args, **kwargs)

                class CompatInputLayer(tf.keras.layers.InputLayer):
                    def __init__(self, *args, **kwargs):
                        kwargs.pop('batch_shape', None)
                        kwargs.pop('optional', None)
                        super().__init__(*args, **kwargs)

                custom_objects = {
                    'Dense': CompatDense,
                    'GRU': CompatGRU,
                    'LSTM': CompatLSTM,
                    'SimpleRNN': CompatSimpleRNN,
                    'InputLayer': CompatInputLayer,
                }

                current_model = tf.keras.models.load_model(
                    path, compile=False, custom_objects=custom_objects)
                current_model_name = name
                dummy = np.zeros((1, window_size, num_features), dtype=np.float32)
                current_model.predict_on_batch(dummy)
                return True, f"Model '{name}' loaded OK"
        except Exception as e:
            return False, f"Load error: {e}"


    def scale_input(input_batch):
        scaled = input_batch.copy().astype(np.float64)
        if isinstance(scaler_x, dict):
            features = ['x', 'y', 'z']
            if num_features == 6:
                features = ['x', 'y', 'z', 'vx', 'vy', 'vz']
                
            for i, axis in enumerate(features):
                if axis in scaler_x:
                    scaled[0, :, i] = scaler_x[axis].transform(
                        input_batch[0, :, i].reshape(-1, 1)
                    ).flatten()
            return scaled.astype(np.float32)
        else:
            # Assume it is a single scaler for all features
            batch, seq, feats = input_batch.shape
            flat = input_batch.reshape(-1, feats)
            scaled_flat = scaler_x.transform(flat)
            return scaled_flat.reshape(batch, seq, feats).astype(np.float32)

    def inverse_scale_output(pred_scaled):
        if isinstance(pred_scaled, list):
            pred_scaled = pred_scaled[0]
            
        res = []
        if isinstance(scaler_y, dict):
            for i, axis in enumerate(['x', 'y', 'z']):
                if pred_scaled.ndim == 3:
                    val_to_scale = pred_scaled[0, -1, i]
                else:
                    val_to_scale = pred_scaled[0, i]
                    
                val = scaler_y[axis].inverse_transform(
                    val_to_scale.reshape(-1, 1)
                )[0, 0]
                res.append(float(val))
        else:
            # Assume it is a single scaler for all output features (x, y, z)
            if pred_scaled.ndim == 3:
                val_to_scale = pred_scaled[0, -1, :] # shape (3,)
            else:
                val_to_scale = pred_scaled[0, :] # shape (3,)
            val = scaler_y.inverse_transform(val_to_scale.reshape(1, -1))[0]
            res = [float(x) for x in val]
            
        return res

    # Load default model
    ok, msg = do_load_model(default_model)
    send_response({"type": "ready", "success": ok, "message": msg,
                    "model_name": current_model_name})

    if not ok:
        return

    # Main loop: read commands from stdin, write results to stdout
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue

        try:
            cmd = json.loads(line)
        except json.JSONDecodeError:
            continue

        if cmd.get("cmd") == "shutdown":
            break

        elif cmd.get("cmd") == "load_model":
            ok, msg = do_load_model(cmd["model_name"])
            send_response({"type": "model_loaded", "success": ok,
                            "message": msg, "model_name": current_model_name})

        elif cmd.get("cmd") == "predict":
            if current_model is None:
                send_response({"type": "predict", "prediction": None, "inference_ms": 0.0})
                continue
            try:
                input_seq = np.array(cmd["data"], dtype=np.float32)

                # --- Bỏ Savitzky-Golay Filter ---
                # Không áp dụng bộ lọc này vì lúc train offline model không hề dùng Savitzky-Golay.
                # Lọc ở bước inference sẽ làm sai lệch phân phối (distribution) của chuỗi, 
                # đặc biệt gây sai số lớn ở phần rìa (edge) của cửa sổ thời gian (bước quan trọng nhất).
                # try:
                #     if _savgol_filter is not None and len(input_seq) >= 5:
                #         for i in range(num_features):
                #             input_seq[:, i] = _savgol_filter(input_seq[:, i], 5, 3)
                # except Exception:
                #     pass

                input_batch = input_seq.reshape(1, -1, num_features)
                
                # Check for NaNs
                if np.isnan(input_batch).any():
                    send_response({"type": "predict", "prediction": None, "inference_ms": 0.0, "error": "Input contains NaN"})
                    continue

                input_scaled = scale_input(input_batch)

                t0 = time.time()
                if hasattr(current_model, 'predict_f'):
                    # SVGP: flatten window to (1, window_size * num_features)
                    input_flat = input_scaled.flatten().reshape(1, -1).astype(np.float64)
                    mean_tensor, _ = current_model.predict_f(input_flat)
                    pred_scaled = (
                        mean_tensor.numpy()
                        if hasattr(mean_tensor, 'numpy') else np.asarray(mean_tensor)
                    )
                else:
                    # Keras models (GRU/LSTM/RNN): predict_on_batch
                    pred_tensor = current_model.predict_on_batch(input_scaled)
                    if isinstance(pred_tensor, list):
                        pred_scaled = [t.numpy() if hasattr(t, 'numpy') else t for t in pred_tensor]
                    else:
                        pred_scaled = pred_tensor.numpy() if hasattr(pred_tensor, 'numpy') else pred_tensor
                inference_ms = (time.time() - t0) * 1000.0

                prediction = inverse_scale_output(pred_scaled)
                send_response({
                    "type": "predict",
                    "prediction": prediction,
                    "inference_ms": inference_ms,
                    "model_name": current_model_name,
                    "epoch": cmd.get("epoch", -1),  # echo epoch để lọc stale response
                })
            except Exception as e:
                print(f"DEBUG: Predict error: {e}", file=sys.stderr)
                send_response({"type": "predict", "prediction": None,
                                "inference_ms": 0.0, "error": str(e)})


if __name__ == "__main__":
    main()
