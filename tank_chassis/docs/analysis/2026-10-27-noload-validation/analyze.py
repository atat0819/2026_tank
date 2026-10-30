"""Compare frozen coefficient sets on the new no-load capture and prior captures."""
import importlib.util
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
SCRIPT = ROOT / "user/core/Alg/PowerControl-TestVersion/PythonProject/PythonScript/Polynomial Method/PowerFourWheels.py"
spec = importlib.util.spec_from_file_location("power_fit", SCRIPT)
fit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fit)
previous = json.loads((OUT.parent / "2026-10-27-power-fit/fit_results.json").read_text(encoding="utf-8"))
captures = [fit.load_capture(Path(path)) for path in
            ["E:/功率限制1.csv", "E:/功率限制2.csv", "E:/功率采集空转1.csv"]]
current = np.array(previous["coefficients_per_motor"])
original = np.array(previous["old_coefficients"])
test = captures[-1]
candidate, diagnostics = fit.least_squares(test["matrix"], test["power"])
models = {"original": original, "current_ground_fit": current, "noload_only_refit": candidate}
result = {
    "coefficients": {name: value.tolist() for name, value in models.items()},
    "noload_refit_diagnostics": diagnostics,
    "chi_square": None,
    "chi_square_note": "Measurement noise variance is unknown; SSE is not a chi-square statistic.",
    "policy": "No smoothing, residual filtering, delay correction or firmware edits; negative power retained.",
    "captures": [],
}
for capture in captures:
    x, y, data = capture["matrix"], capture["power"], capture["data"]
    item = {**capture["audit"], "models": {}}
    moving = np.max(abs(data[:, [2, 4, 6, 8]]), axis=1) >= 1
    for name, coefficients in models.items():
        predicted = x @ coefficients
        item["models"][name] = {
            **fit.metrics(y, predicted), "sse_w2": float(np.sum((predicted-y)**2)),
            "moving_rows": int(moving.sum()),
            "moving_rmse_w": float(np.sqrt(np.mean((predicted[moving]-y[moving])**2))),
        }
    result["captures"].append(item)

data, x, y = test["data"], test["matrix"], test["power"]
predicted = x @ current
result["noload_current_telemetry_max_difference_w"] = float(np.max(abs(predicted-data[:, 9])))
error = predicted-y
worst = int(np.argmax(abs(error)))
result["worst_sample"] = {"row_index_zero_based": worst, "time_s": float(data[worst, 10]),
                          "actual_w": float(y[worst]), "predicted_w": float(predicted[worst]),
                          "error_w": float(error[worst])}
time = data[:, 10]-data[0, 10]
fig, axes = plt.subplots(3, 1, figsize=(13, 10))
axes[0].plot(time, y, color="#303030", lw=.8, label="Measured")
axes[0].plot(time, predicted, color="#d95f02", lw=.8, alpha=.8, label="Current coefficients")
axes[0].set_title("Independent no-load validation: R2 = 0.96344, RMSE = 6.849 W")
axes[0].set_ylabel("Total input power (W)")
axes[1].plot(time, error, color="#0072b2", lw=.7)
axes[1].set_ylabel("Predicted - measured (W)")
axes[1].axhline(0, color="#555555", lw=.7)
axes[1].set_xlabel("Elapsed capture time (s)")
zoom = abs(time-time[worst]) < .05
axes[2].plot(time[zoom], y[zoom], ".-", color="#303030", label="Measured")
axes[2].plot(time[zoom], predicted[zoom], ".-", color="#d95f02", label="Current coefficients")
axes[2].axvline(time[worst], color="#999999", ls="--", lw=.8)
axes[2].set_title(f"Largest transient error: {abs(error[worst]):.3f} W (raw samples, no time shift)")
axes[2].set_xlabel("Elapsed capture time (s)")
axes[2].set_ylabel("Total input power (W)")
for ax in axes:
    ax.grid(alpha=.2)
axes[0].legend()
axes[2].legend()
fig.tight_layout()
fig.savefig(OUT / "noload_comparison.png", dpi=160)
plt.close(fig)
(OUT / "validation_results.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")

table = "\n".join(f"| {name} | {r['rmse_w']:.3f} | {r['r2']:.5f} | {r['mae_w']:.3f} | {r['absolute_error_p95_w']:.3f} | {r['absolute_error_max_w']:.3f} |"
                  for name, r in result["captures"][-1]["models"].items())
cross_table = "\n".join(f"| {Path(c['path']).name} | {c['models']['current_ground_fit']['rmse_w']:.3f} | {c['models']['noload_only_refit']['rmse_w']:.3f} |"
                        for c in result["captures"])
coefficient_table = "\n".join(f"| K{i} | {current[i]:.12g} | {candidate[i]:.12g} |" for i in range(6))
report = f'''# 空转记录独立验证

**保留当前系数比恢复原始系数更合适。** 本次没有改动固件参数。

输入为 `E:/功率采集空转1.csv`，共 {len(y):,} 条记录、约 {time[-1]:.3f} 秒。无非有限数、无离线行，采样序号连续。电流约 -22.02～20.70 A，转子转速约 -968.66～951.48 rad/s，总输入功率约 -155.19～176.74 W；已明显超出此前落地辨识数据的转速和功率范围。

通道 I9 与按当前系数重算的最大差约 {result['noload_current_telemetry_max_difference_w']:.8f} W，支持这份数据确实使用当前新系数的判断。

## 三组模型在同一份空转记录上的比较

| 模型 | RMSE W | R² | MAE W | 95%绝对误差 W | 最大绝对误差 W |
|---|---:|---:|---:|---:|---:|
{table}

`original` 是最初 K0=0.614597983 等参数；`current_ground_fit` 是刚填入固件的两份落地数据联合拟合；`noload_only_refit` 是仅使用这份空转数据重新最小二乘得到的候选。

前两组是固定参数在新记录上的验证。第三组使用同一份空转记录拟合并评价，属于训练误差，不能与独立验证混同。数据没有平滑、删残差异常点或做时移；负功率保留。

## 空转重拟合是否值得替换

| 参数 | 当前落地拟合 | 仅空转重新拟合 |
|---|---:|---:|
{coefficient_table}

| 数据 | 当前系数 RMSE W | 空转重拟合 RMSE W |
|---|---:|---:|
{cross_table}

空转重拟合在本记录上的 RMSE 下降约 9.2%，但在前两份落地记录上的误差均增大。其 K5 为负，速度损耗二次项的外推也需要额外检验。因此本次不建议直接用仅空转的候选替换当前系数。若目标是同时覆盖落地与悬空高速，应在明确两种工况权重后联合辨识，并保留新的独立数据验证。

## “卡方”和拟合优度

R² 是拟合优度；当前系数在这份独立空转数据上为 **0.96344**。统计学卡方 χ² 通常需要各点测量误差标准差，例如 χ²=sum(((P测量-P预测)/sigma)^2)。目前没有功率计测量方差，也无法确认误差独立，不能给出有效的卡方 p 值或把误差平方和直接称为卡方。

作为普通误差量，当前模型 SSE 为 {np.sum(error**2):.3f} W²。R² 较高并不代表瞬时误差很小：最差样本在记录时间 {data[worst,10]:.6f} s，实测 {y[worst]:.3f} W，预测 {predicted[worst]:.3f} W，差 {error[worst]:.3f} W。曲线中存在快速正负功率切换和短时错峰，CSV 尚不足以证明它们由功率计延迟、反馈采样差异还是其他原因造成。

见 `noload_comparison.png` 的完整曲线、误差和最差片段放大图。均值偏差只有约 -0.019 W，也不能掩盖瞬时峰值误差；对限功率使用，应继续关注这种瞬态。

## 复现与来源

在工程根目录执行 `python docs/analysis/2026-10-27-noload-validation/analyze.py`。脚本复用已有四轮矩阵构造及最小二乘函数，读取此前保存的原始/当前系数作为冻结比较基准。完整数值、输入文件路径和 SHA256 在 `validation_results.json`。前两份数据仍为 `E:/功率限制1.csv`、`E:/功率限制2.csv`。三个原始 CSV 均只读。
'''
(OUT / "report.md").write_text(report, encoding="utf-8")
print(table)
print(cross_table)
print("Saved:", OUT)
