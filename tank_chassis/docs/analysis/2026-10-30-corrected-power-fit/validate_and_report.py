"""Validate the corrected-wiring fit without changing firmware or source CSVs."""
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
script = ROOT / "user/core/Alg/PowerControl-TestVersion/PythonProject/PythonScript/Polynomial Method/PowerFourWheels.py"
spec = importlib.util.spec_from_file_location("power_fit", script)
fit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fit)
saved = json.loads((OUT / "fit_results.json").read_text(encoding="utf-8"))
captures = [fit.load_capture(Path(c["path"])) for c in saved["captures"]]
shares = np.array([c["normalized_objective_share"] for c in saved["captures"]])
np.testing.assert_allclose(shares, [.001, .999], rtol=1e-12)
for current, original in zip(captures, saved["captures"]):
    assert current["audit"]["sha256"] == original["sha256"]

matrix = np.concatenate([c["matrix"] for c in captures])
power = np.concatenate([c["power"] for c in captures])
weights = np.concatenate([np.full(len(c["power"]), s / len(c["power"]))
                          for c, s in zip(captures, shares)])
coefficients = np.array(saved["coefficients_per_motor"])
# Independent QR solve cross-checks the helper's scaled SVD solution.
weighted = matrix * np.sqrt(weights)[:, None]
scale = np.sqrt(np.mean(weighted**2, axis=0))
q, r = np.linalg.qr(weighted / scale, mode="reduced")
qr_coefficients = np.linalg.solve(r, q.T @ (power * np.sqrt(weights))) / scale
np.testing.assert_allclose(qr_coefficients, coefficients, rtol=1e-9, atol=1e-11)

predictions = [np.full(len(c["power"]), np.nan) for c in captures]
folds = []
for fold in range(5):
    train, validation = [], []
    for capture in captures:
        n = len(capture["power"])
        lo, hi = fold * n // 5, (fold + 1) * n // 5
        tick = capture["data"][:, 10]
        validation.append((np.arange(n) >= lo) & (np.arange(n) < hi))
        train.append((tick < tick[lo] - .1) | (tick > tick[hi - 1] + .1))
    training_weights = np.concatenate([np.full(mask.sum(), share / mask.sum())
                                       for mask, share in zip(train, shares)])
    trained, diagnostics = fit.least_squares(
        np.concatenate([c["matrix"][m] for c, m in zip(captures, train)]),
        np.concatenate([c["power"][m] for c, m in zip(captures, train)]),
        weights=training_weights)
    for capture, mask, prediction in zip(captures, validation, predictions):
        prediction[mask] = capture["matrix"][mask] @ trained
    folds.append({"fold": fold + 1, "training_rows": [int(m.sum()) for m in train],
                  "validation_rows": [int(m.sum()) for m in validation],
                  "coefficients": trained.tolist(), **diagnostics})

details = []
snippet = (OUT / "poly_coeffs.txt").read_text(encoding="utf-8")
rounded = np.array([float(line.strip().split("f")[0].rstrip(","))
                    for line in snippet.splitlines()[1:7]])
for capture, prediction, old in zip(captures, predictions, saved["captures"]):
    assert np.isfinite(prediction).all()
    data, x, y = capture["data"], capture["matrix"], capture["power"]
    # Match the firmware's float arithmetic and explicit four-motor sum.
    k = rounded.astype(np.float32)
    direct = np.zeros(len(data), dtype=np.float32)
    for index in range(4):
        a = data[:, 1 + 2 * index].astype(np.float32)
        w = data[:, 2 + 2 * index].astype(np.float32)
        direct += k[0] + k[1]*a + k[2]*abs(w) + k[3]*a*w + k[4]*a*a + k[5]*w*w
    double_prediction = x @ coefficients
    difference = float(np.max(abs(direct - double_prediction)))
    assert difference < .001
    worst = int(np.argmax(abs(double_prediction - y)))
    details.append({"path": old["path"], "blocked_validation": fit.metrics(y, prediction),
                    "float32_max_difference_w": difference,
                    "worst_fit_sample": {"time_s": float(data[worst, 10]),
                                         "sequence": int(data[worst, 14]),
                                         "measured_w": float(y[worst]),
                                         "predicted_w": float(double_prediction[worst])}})

validation = {"method": "Five contiguous blocks per capture; exclude 100 ms at training boundaries; preserve 0.001/0.999 group shares in each fold.",
              "qr_coefficients": qr_coefficients.tolist(), "folds": folds,
              "captures": details}
(OUT / "validation_results.json").write_text(json.dumps(validation, ensure_ascii=False, indent=2), encoding="utf-8")

labels = ["空载", "落地"]
rows = "\n".join(
    f"| {label} | {item['rows_used']:,} | {item['old_model']['rmse_w']:.4f} | "
    f"{item['joint_fit']['rmse_w']:.4f} | {detail['blocked_validation']['rmse_w']:.4f} | "
    f"{item['joint_fit']['r2']:.6f} | {item['joint_fit']['absolute_error_p95_w']:.4f} | "
    f"{item['joint_fit']['absolute_error_max_w']:.4f} |"
    for label, item, detail in zip(labels, saved["captures"], details))
sources = "\n".join(f"- `{c['path']}`：SHA256 `{c['sha256']}`。" for c in saved["captures"])
report = f"""# 正确接线新数据的四轮功率参数辨识

仅使用用户本次提供的两份新记录。按用户确认，落地总权重99.9%、空载总权重0.1%。旧接线数据未参与拟合；旧固件系数只用于误差对照。

## 参数与模型

```cpp
{snippet.strip()}
```

单电机模型：`P_i = K0 + K1*I_i + K2*abs(w_i) + K3*I_i*w_i + K4*I_i² + K5*w_i²`。
四轮输入总功率为四个模型求和，常数项为4*K0。I保留反馈正负号，单位A；w为转子端rad/s，不除以15.8；P单位W。

```text
J = 0.001/40002 * sum_air((X*K-P_meter)^2)
  + 0.999/49783 * sum_ground((X*K-P_meter)^2)
```

六参数一次联合加权最小二乘，列缩放后用SVD求解，独立QR复算一致。矩阵秩为{saved['rank']}，缩放条件数{saved['scaled_condition_number']:.4f}。这是误差目标权重，不是功率分配比例。

## 数据与误差

两份文件共{saved['rows_used']:,}行，全部四轮在线、全部有限数。空载记录缺少1个采样序号，落地记录序号连续。负功率数据保留。未平滑、未删除大残差点、未作时延补偿或系数符号约束。
两份记录中，实测功率与直流电压乘电流的最大差均小于0.000023W。这只核对文件内部一致性，不是功率计绝对精度校验。

| 数据 | 行数 | 旧系数RMSE W | 新系数拟合RMSE W | 新系数分段验证RMSE W | 拟合R² | 拟合95%绝对误差 W | 拟合最大绝对误差 W |
|---|---:|---:|---:|---:|---:|---:|---:|
{rows}

拟合误差使用参与拟合的完整记录。分段验证把每份文件分成五个连续块，逐折留出对应块，训练数据排除验证块两侧100ms；每折保持两组总权重。验证点不参与该折参数求解，但仍来自这两份采集记录，不等同于新工况的独立实车验证。

## 使用范围

旧系数在两份新记录上均有明显低估。新系数提高了对本次正确接线数据的吻合程度，但高速空载的误差大于落地工况，仍存在较大的瞬时残差。RMSE和R²不代表峰值误差保证；现有数据不足以把瞬态偏差确定归因为采样时延或模型本身。
源代码使用指令电流进行限流预测，本辨识使用CAN反馈电流；接入新参数后仍需核对两种电流及实际功率的差异。
12位小数参数按固件float运算复算，与双精度矩阵预测的最大差小于0.001W。没有修改或烧录固件。

## 数据来源

{sources}

`fit_results.json`记录参数、逐文件误差、数据范围和权重；`validation_results.json`记录各折参数、验证误差和最差样本；`fit_comparison.png`展示实测、拟合与残差。
复现：用已有PowerFourWheels.py依次传入空载、落地文件，逐行权重分别为0.001/40002与0.999/49783，再运行本目录validate_and_report.py。
"""
(OUT / "report.md").write_text(report, encoding="utf-8")
for original in saved["captures"]:
    assert hashlib.sha256(Path(original["path"]).read_bytes()).hexdigest() == original["sha256"]
print(rows)
print("PASS: independent QR, blocked validation, float32 arithmetic and unchanged input hashes")
