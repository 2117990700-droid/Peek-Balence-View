from __future__ import annotations

import json
import argparse
import sys
from datetime import datetime, timezone
from pathlib import Path

import joblib
import pandas as pd
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score, classification_report
from sklearn.model_selection import train_test_split

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_DATASET_PATH = ROOT / "data" / "balance_dataset.csv"
MODEL_PATH = Path(__file__).resolve().parent / "balance_model.joblib"
METRICS_PATH = Path(__file__).resolve().parent / "metrics.json"

MODEL_CONFIG = {
    "algorithm": "RandomForestClassifier",
    "n_estimators": 100,
    "max_depth": 6,
    "criterion": "gini",
    "min_samples_leaf": 2,
}

FEATURES = [
    "chestSway",
    "waistSway",
    "swayDifference",
    "stepFrequency",
    "imuBalanceScore",
    "leftPressure",
    "rightPressure",
    "totalPressure",
    "lrBalance",
    "apBalance",
    "footSway",
    "footStabilityScore",
]


parser = argparse.ArgumentParser()
parser.add_argument("--dataset", type=Path, default=DEFAULT_DATASET_PATH)
parser.add_argument("--demo", action="store_true")
args = parser.parse_args()
DATASET_PATH = args.dataset.resolve()
try:
    DATASET_REFERENCE = DATASET_PATH.relative_to(ROOT).as_posix()
except ValueError:
    DATASET_REFERENCE = DATASET_PATH.name


def fail(message: str) -> None:
    print(message, file=sys.stderr)
    raise SystemExit(1)


if not DATASET_PATH.exists():
    fail(f"找不到数据集：{DATASET_PATH}")

data = pd.read_csv(DATASET_PATH)
required = FEATURES + ["label"]
missing = [column for column in required if column not in data.columns]
if missing:
    fail("数据集缺少字段：" + ", ".join(missing))

clean = data[required].copy()
for feature in FEATURES:
    clean[feature] = pd.to_numeric(clean[feature], errors="coerce")
clean["label"] = clean["label"].astype(str).str.strip().str.upper()
clean = clean.dropna(subset=required)

class_counts = clean["label"].value_counts()
if len(clean) < 30:
    fail(f"有效样本只有 {len(clean)} 条；至少采集 30 条后再训练。")
if len(class_counts) < 2:
    fail("至少需要两种不同标签的数据。")
if int(class_counts.min()) < 5:
    fail("每种标签至少需要 5 条有效样本。")

X = clean[FEATURES]
y = clean["label"]
test_count = max(len(class_counts), round(len(clean) * 0.2))

X_train, X_test, y_train, y_test = train_test_split(
    X,
    y,
    test_size=test_count,
    random_state=42,
    stratify=y,
)

model = RandomForestClassifier(
    n_estimators=MODEL_CONFIG["n_estimators"],
    max_depth=MODEL_CONFIG["max_depth"],
    criterion=MODEL_CONFIG["criterion"],
    min_samples_leaf=MODEL_CONFIG["min_samples_leaf"],
    class_weight="balanced",
    random_state=42,
    n_jobs=-1,
)
model.fit(X_train, y_train)

predicted = model.predict(X_test)
accuracy = float(accuracy_score(y_test, predicted))
report = classification_report(y_test, predicted, output_dict=True, zero_division=0)

bundle = {
    "model": model,
    "features": FEATURES,
    "classes": [str(value) for value in model.classes_],
    "trained_at": datetime.now(timezone.utc).isoformat(),
    "sample_count": int(len(clean)),
    "accuracy": accuracy,
    "model_config": MODEL_CONFIG,
    "demo_mode": bool(args.demo),
    "dataset": DATASET_REFERENCE,
}
joblib.dump(bundle, MODEL_PATH)

metrics = {
    "accuracy": accuracy,
    "demo_mode": bool(args.demo),
    "dataset": DATASET_REFERENCE,
    "sample_count": int(len(clean)),
    "model_config": MODEL_CONFIG,
    "class_counts": {str(key): int(value) for key, value in class_counts.items()},
    "classification_report": report,
    "feature_importance": {
        name: float(value)
        for name, value in sorted(
            zip(FEATURES, model.feature_importances_),
            key=lambda item: item[1],
            reverse=True,
        )
    },
}
METRICS_PATH.write_text(json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8")

print(
    f"{'演示模型' if args.demo else '模型'}训练完成：{len(clean)} 条样本，"
    f"{len(class_counts)} 种标签，测试准确率 {accuracy:.1%}。"
)
