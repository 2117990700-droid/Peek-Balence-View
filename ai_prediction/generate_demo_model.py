from __future__ import annotations

import csv
import random
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DATA_PATH = ROOT / "data" / "demo_balance_dataset.csv"
TRAIN_SCRIPT = Path(__file__).resolve().parent / "train_model.py"

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

random.seed(20260922)


def clipped_gauss(mean: float, deviation: float, low: float, high: float) -> float:
    return max(low, min(high, random.gauss(mean, deviation)))


def make_row(label: str, index: int) -> dict:
    if label == "NORMAL":
        chest = clipped_gauss(0.045, 0.018, 0.005, 0.10)
        waist = clipped_gauss(0.040, 0.016, 0.005, 0.09)
        step = clipped_gauss(1.20, 0.35, 0.35, 2.10)
        lr = clipped_gauss(0.00, 0.045, -0.12, 0.12)
        ap = clipped_gauss(0.00, 0.050, -0.14, 0.14)
        foot_sway = clipped_gauss(0.07, 0.025, 0.01, 0.15)
        imu_score = clipped_gauss(89, 6, 72, 100)
        foot_score = clipped_gauss(90, 5, 74, 100)
    elif label == "WARNING":
        chest = clipped_gauss(0.145, 0.045, 0.06, 0.28)
        waist = clipped_gauss(0.125, 0.040, 0.05, 0.25)
        step = clipped_gauss(0.72, 0.30, 0.05, 1.45)
        lr = clipped_gauss(0.13 * random.choice([-1, 1]), 0.07, -0.30, 0.30)
        ap = clipped_gauss(0.12 * random.choice([-1, 1]), 0.07, -0.30, 0.30)
        foot_sway = clipped_gauss(0.25, 0.08, 0.09, 0.48)
        imu_score = clipped_gauss(61, 10, 35, 82)
        foot_score = clipped_gauss(63, 10, 35, 82)
    else:
        chest = clipped_gauss(0.32, 0.09, 0.14, 0.58)
        waist = clipped_gauss(0.27, 0.08, 0.12, 0.52)
        step = clipped_gauss(0.25, 0.18, 0.0, 0.75)
        lr = clipped_gauss(0.30 * random.choice([-1, 1]), 0.11, -0.58, 0.58)
        ap = clipped_gauss(0.28 * random.choice([-1, 1]), 0.11, -0.58, 0.58)
        foot_sway = clipped_gauss(0.62, 0.18, 0.28, 1.15)
        imu_score = clipped_gauss(28, 12, 0, 55)
        foot_score = clipped_gauss(30, 12, 0, 58)

    total = clipped_gauss(640, 90, 390, 900)
    left = total * (1 - lr) / 2
    right = total * (1 + lr) / 2

    return {
        "timestamp": int(time.time() * 1000) + index,
        "chestSway": round(chest, 6),
        "waistSway": round(waist, 6),
        "swayDifference": round(abs(chest - waist), 6),
        "stepFrequency": round(step, 6),
        "imuBalanceScore": round(imu_score, 4),
        "leftPressure": round(left, 4),
        "rightPressure": round(right, 4),
        "totalPressure": round(total, 4),
        "lrBalance": round(lr, 6),
        "apBalance": round(ap, 6),
        "footSway": round(foot_sway, 6),
        "footStabilityScore": round(foot_score, 4),
        "label": label,
    }


DATA_PATH.parent.mkdir(parents=True, exist_ok=True)
rows = []
for label in ("NORMAL", "WARNING", "HIGH_RISK"):
    rows.extend(make_row(label, len(rows) + offset) for offset in range(20))
random.shuffle(rows)

with DATA_PATH.open("w", newline="", encoding="utf-8") as file:
    writer = csv.DictWriter(file, fieldnames=["timestamp", *FEATURES, "label"])
    writer.writeheader()
    writer.writerows(rows)

print(f"已生成 {len(rows)} 条合成演示样本：{DATA_PATH}")
raise SystemExit(
    subprocess.call(
        [sys.executable, str(TRAIN_SCRIPT), "--dataset", str(DATA_PATH), "--demo"]
    )
)
