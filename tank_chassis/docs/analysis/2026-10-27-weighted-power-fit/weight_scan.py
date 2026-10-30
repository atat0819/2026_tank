"""Compare group weights with raw fits and five contiguous validation blocks."""
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
captures = [fit.load_capture(Path(path)) for path in
            ["E:/功率限制1.csv", "E:/功率限制2.csv", "E:/功率采集空转1.csv"]]
matrix = np.concatenate([c["matrix"] for c in captures])
power = np.concatenate([c["power"] for c in captures])
ground_count = sum(len(c["power"]) for c in captures[:2])
air_count = len(captures[2]["power"])
results = []
for alpha in [0, .0005, .001, .002, .005, .01, .02, .05, .1, .3, .5]:
    weights = np.r_[np.full(ground_count, (1-alpha)/ground_count),
                    np.full(air_count, alpha/air_count)]
    coefficients, diagnostics = fit.least_squares(matrix, power, weights=weights)
    validation_predictions = [np.empty(len(c["power"])) for c in captures]
    for fold in range(5):
        train, validation = [], []
        for capture in captures:
            count = len(capture["power"])
            lo, hi = fold*count//5, (fold+1)*count//5
            indices = np.arange(count)
            validation.append((indices >= lo) & (indices < hi))
            tick = capture["data"][:, 10]
            # Adjacent training rows within 100 ms of the test block are excluded.
            train.append((tick < tick[lo]-.1) | (tick > tick[hi-1]+.1))
        ng = sum(m.sum() for m in train[:2])
        na = train[2].sum()
        train_weights = np.r_[np.full(ng, (1-alpha)/ng), np.full(na, alpha/na)]
        trained, _ = fit.least_squares(
            np.concatenate([c["matrix"][m] for c, m in zip(captures, train)]),
            np.concatenate([c["power"][m] for c, m in zip(captures, train)]),
            weights=train_weights)
        for capture, mask, predictions in zip(captures, validation, validation_predictions):
            predictions[mask] = capture["matrix"][mask] @ trained
    results.append({"air_group_share": alpha, "coefficients": coefficients.tolist(), **diagnostics,
                    "fits": [fit.metrics(c["power"], c["matrix"] @ coefficients) for c in captures],
                    "blocked_validation": [fit.metrics(c["power"], p)
                                           for c, p in zip(captures, validation_predictions)]})

output = {"policy": "Ground driving has priority; normalize each group by its row count.",
          "objective": "(1-alpha)*MSE_ground + alpha*MSE_air",
          "blocked_validation": "Five contiguous equal-row blocks per file; 100 ms training guard at test boundaries.",
          "selected_air_share": .001, "input_audits": [c["audit"] for c in captures], "weight_scan": results}
(OUT / "weight_scan.json").write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding="utf-8")
selected = next(r for r in results if r["air_group_share"] == .001)
saved = json.loads((OUT / "fit_results.json").read_text(encoding="utf-8"))
np.testing.assert_allclose(selected["coefficients"], saved["coefficients_per_motor"], rtol=1e-10, atol=1e-12)
assert np.isclose(sum(c["normalized_objective_share"] for c in saved["captures"][:2]), .999)
assert np.isclose(saved["captures"][2]["normalized_objective_share"], .001)

rows = "\n".join(f"| {r['air_group_share']*100:g}% | " + " | ".join(f"{m['rmse_w']:.4f}" for m in r["fits"]) + " |"
                 for r in results)
validation_rows = "\n".join(f"| {label} | " + " | ".join(f"{m['rmse_w']:.4f}" for m in r["blocked_validation"]) + " |"
                            for label, r in [("仅落地", results[0]), ("落地99.9% / 空载0.1%", selected)])
report = f'''# 落地优先的加权融合

用户明确要求“绝对的优先落地行驶”。建议的候选为 **落地总权重99.9%、空载总权重0.1%**。本次只生成分析结果，未替换固件参数。

使用前两份落地记录共 {ground_count:,} 行和空载记录 {air_count:,} 行，加权最小二乘目标为：

```text
J(K) = 0.999/26730 * sum_ground((X*K-P)^2)
     + 0.001/22179 * sum_air((X*K-P)^2)
```

即 0.999*MSE_落地 + 0.001*MSE_空载。两份落地记录内部每行等权，保留此前落地拟合的相对权重。空载每行相对落地每行的权重约为 0.001206400503。此处的99.9%/0.1%是目标函数权重，不是概率、数据丢弃比例或车辆功率分配。

不是先分别拟合再平均参数；求解时对 X 和 P 的每行乘以权重平方根，并重新求同一组六个系数。三份数据全部保留，负功率、间断和异常残差的处理与此前分析相同，没有自动时移或平滑。

## 权重取舍

下表为各权重在原始三份记录上的 RMSE，W；各行都使用了参与拟合的数据，属于样本内比较。

| 空载总权重 | 落地1 | 落地2 | 空载 |
|---|---:|---:|---:|
{rows}

空载数据转速更高，平方项特征更大。即使很小的权重，也能影响高速模型。增加空载权重可继续降低空载样本误差，但会逐渐增加落地误差。50%空载并不符合本次优先落地的目标。

## 分段验证

每份记录按采样顺序分成五段，每次留出对应一段验证，训练中排除边界100 ms邻域。没有随机打乱相邻样本。验证数据不参与各折参数求解。

| 参数策略 | 落地1验证 RMSE | 落地2验证 RMSE | 空载验证 RMSE |
|---|---:|---:|---:|
{validation_rows}

0.1%空载的落地验证误差与原模型几乎相同，空载验证有所改善。落地的数值变化很小，不能据此声称统计显著提升；推荐该比例是为了在观察到的误差取舍中尽量保持落地表现，而非追求空载最低误差。权重比较使用同一批数据，仍需新的独立落地记录验证最终模型。

## 融合系数

```cpp
{(OUT / 'poly_coeffs.txt').read_text(encoding='utf-8').strip()}
```

系数仍按单电机定义，四轮常数项使用4*K0。空载最大瞬态误差仍在约129 W，轻量融合未解决采样错峰；不能把整体RMSE改善理解为瞬态限功率误差界。

## 复现

```powershell
python "user/core/Alg/PowerControl-TestVersion/PythonProject/PythonScript/Polynomial Method/PowerFourWheels.py" "E:/功率限制1.csv" "E:/功率限制2.csv" "E:/功率采集空转1.csv" --file-weights 1 1 0.0012064005030324521 --output-dir "docs/analysis/2026-10-27-weighted-power-fit"
python docs/analysis/2026-10-27-weighted-power-fit/weight_scan.py
```

`--file-weights` 是每个文件的逐行权重，脚本在JSON中同时记录归一化后的总权重。`fit_results.json` 给出拟合结果，`weight_scan.json` 给出权重扫描和分段验证，`fit_comparison.png` 为三份记录的实测/拟合与误差曲线。输入路径及SHA256也保存于JSON。三个CSV未修改，固件参数未修改。
'''
(OUT / "report.md").write_text(report, encoding="utf-8")
print(rows)
print(validation_rows)
print("PASS: normalized group shares and independent weighted solve agree with CLI output")
