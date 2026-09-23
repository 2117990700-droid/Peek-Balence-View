from __future__ import annotations

import json
import sys
from pathlib import Path

import joblib
import pandas as pd

MODEL_PATH = Path(__file__).resolve().parent / "balance_model.joblib"

bundle = None
model_mtime = None


def emit(message: dict) -> None:
    print(json.dumps(message, ensure_ascii=False), flush=True)


def load_model_if_changed() -> bool:
    global bundle, model_mtime
    if not MODEL_PATH.exists():
        bundle = None
        model_mtime = None
        return False

    current_mtime = MODEL_PATH.stat().st_mtime_ns
    if bundle is None or current_mtime != model_mtime:
        bundle = joblib.load(MODEL_PATH)
        model_mtime = current_mtime
        emit(
            {
                "type": "model_status",
                "modelReady": True,
                "demoMode": bool(bundle.get("demo_mode", False)),
                "message": (
                    f"演示模型（{bundle.get('sample_count', '?')} 条合成样本）"
                    if bundle.get("demo_mode", False)
                    else f"模型已加载（{bundle.get('sample_count', '?')} 条训练样本）"
                ),
            }
        )
    return True


try:
    if not load_model_if_changed():
        emit({"type": "model_status", "modelReady": False, "message": "模型未训练"})
except Exception as error:
    emit({"type": "model_status", "modelReady": False, "message": f"模型加载失败：{error}"})

for line in sys.stdin:
    request = None
    try:
        request = json.loads(line)
        if not load_model_if_changed():
            continue

        feature_names = bundle["features"]
        values = request.get("features", {})
        row = {name: float(values[name]) for name in feature_names}
        frame = pd.DataFrame([row], columns=feature_names)

        model = bundle["model"]
        prediction = str(model.predict(frame)[0])
        probabilities = {}
        confidence = None

        if hasattr(model, "predict_proba"):
            scores = model.predict_proba(frame)[0]
            probabilities = {
                str(label): float(score)
                for label, score in zip(model.classes_, scores)
            }
            confidence = max(probabilities.values())

        emit(
            {
                "type": "ai_prediction",
                "id": request.get("id"),
                "timestamp": request.get("timestamp"),
                "prediction": prediction,
                "confidence": confidence,
                "probabilities": probabilities,
                "demoMode": bool(bundle.get("demo_mode", False)),
            }
        )
    except Exception as error:
        emit(
            {
                "type": "prediction_error",
                "id": request.get("id") if isinstance(request, dict) else None,
                "message": str(error),
            }
        )
