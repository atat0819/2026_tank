"""Read-only comparison of the logged, causal IMU filters."""
from pathlib import Path
import json

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import numpy as np
import pandas as pd
from scipy import signal

SOURCE = Path("E:/IMU\u6570\u636e2.csv")
OUT = Path(__file__).resolve().parent
A = pd.read_csv(SOURCE).to_numpy(dtype=float)
assert A.shape[1] == 6 and np.isfinite(A).all()
t = A[:, 4] - A[0, 4]
dt = np.diff(t)
assert np.all(dt > 0)
period = round(float(np.median(dt)), 3)
fs = 1 / period
regular = dt < 1.5 * period
grid = np.arange(0, t[-1] + period / 2, period)
uniform = np.column_stack([np.interp(grid, t, A[:, j]) for j in range(4)])
quiet = t >= 25
metrics = {
    "source": str(SOURCE), "rows": len(A), "duration_s": float(t[-1]),
    "nominal_sample_rate_hz": fs,
    "timestamp_dt_range_s": [float(dt.min()), float(dt.max())],
    "gaps": [{"csv_row_after_gap": int(i + 3), "relative_time_s": float(t[i + 1]),
              "dt_s": float(dt[i])} for i in np.flatnonzero(~regular)],
    "quiet_segment_s": [float(t[quiet][0]), float(t[quiet][-1])],
    "channels": {},
}

for name, raw, filtered, alpha in [("pitch", 0, 1, .09), ("pitch_rate", 2, 3, .25)]:
    innovation = A[1:, raw] - A[:-1, filtered]
    change = A[1:, filtered] - A[:-1, filtered]
    fitted = np.dot(innovation[regular], change[regular]) / np.dot(
        innovation[regular], innovation[regular])
    residual = change - alpha * innovation
    fc = fs / (2 * np.pi) * np.arccos(1 - alpha ** 2 / (2 * (1 - alpha)))
    low_delay_samples = (1 - alpha) / alpha
    freq, raw_psd = signal.welch(uniform[:, raw], fs, nperseg=2048)
    _, filtered_psd = signal.welch(uniform[:, filtered], fs, nperseg=2048)
    # Integrate PSD bin power on disjoint bands; total shares sum to 100%.
    total = float(np.sum(raw_psd) * (freq[1] - freq[0]))
    bands = []
    for low, high in [(0, 1), (1, 3), (3, 10), (10, 30), (30, 101)]:
        mask = (freq >= low) & (freq < high)
        p = float(np.sum(raw_psd[mask]) * (freq[1] - freq[0]))
        q = float(np.sum(filtered_psd[mask]) * (freq[1] - freq[0]))
        bands.append({"hz": [low, min(high, fs / 2)], "raw_power_percent": 100 * p / total,
                      "raw_rms": np.sqrt(p), "filtered_rms": np.sqrt(q),
                      "attenuation_db": 10 * np.log10(q / p)})
    correlations = [np.corrcoef(uniform[:-k or None, raw], uniform[k:, filtered])[0, 1]
                    for k in range(41)]
    delay_index = int(np.argmax(correlations))
    responses = []
    for hz in [1, 3, 5, 10, 20]:
        _, h = signal.freqz([alpha], [1, -(1 - alpha)], worN=[hz], fs=fs)
        responses.append({"hz": hz, "magnitude": float(abs(h[0])),
                          "phase_deg": float(np.angle(h[0], deg=True)),
                          "phase_delay_ms": float(-np.angle(h[0]) / (2 * np.pi * hz) * 1000)})
    second = signal.butter(2, fc, fs=fs)
    _, second_gd = signal.group_delay(second, w=[0], fs=fs)
    same_delay_fc = fs / np.pi * np.arctan(1 / (np.sqrt(2) * low_delay_samples))
    metrics["channels"][name] = {
        "alpha": alpha, "fitted_alpha": float(fitted),
        "recurrence_max_abs_error_excluding_gap": float(np.max(abs(residual[regular]))),
        "raw_range": [float(A[:, raw].min()), float(A[:, raw].max())],
        "filtered_range": [float(A[:, filtered].min()), float(A[:, filtered].max())],
        "same_time_max_abs_difference": float(np.max(abs(A[:, raw] - A[:, filtered]))),
        "cutoff_hz": float(fc), "low_frequency_group_delay_ms": low_delay_samples / fs * 1000,
        "cross_correlation_delay_ms": delay_index / fs * 1000,
        "cross_correlation": float(correlations[delay_index]),
        "quiet_raw_std": float(np.std(A[quiet, raw])),
        "quiet_filtered_std": float(np.std(A[quiet, filtered])),
        "quiet_raw_linear_detrended_rms": float(np.std(signal.detrend(A[quiet, raw]))),
        "quiet_filtered_linear_detrended_rms": float(np.std(signal.detrend(A[quiet, filtered]))),
        "bands": bands, "response": responses,
        "second_order_same_cutoff_low_frequency_delay_ms": float(second_gd[0] / fs * 1000),
        "second_order_same_dc_delay_cutoff_hz": float(same_delay_fc),
    }

font_file = Path("C:/Windows/Fonts/msyh.ttc")
if font_file.exists():
    plt.rcParams["font.family"] = FontProperties(fname=str(font_file)).get_name()
plt.rcParams.update({"axes.unicode_minus": False, "font.size": 10,
                     "axes.spines.top": False, "axes.spines.right": False})
fig, axs = plt.subplots(3, 2, figsize=(15, 11.5), constrained_layout=True)
fig.suptitle("IMU 数据2：人为扰动时，原始数据与当前一阶低通", fontsize=17)
center = t[int(np.argmax(abs(A[:, 0] - A[:, 1])))]
zoom = (t >= max(0, center - 1)) & (t <= min(t[-1], center + 1))
for col, name, raw, filtered, unit in [(0, "Pitch", 0, 1, "°"),
                                       (1, "Pitch 角速度", 2, 3, "°/s")]:
    for row, selection in [(0, np.ones(len(A), dtype=bool)), (1, zoom)]:
        ax = axs[row, col]
        ax.plot(t[selection], A[selection, raw], color="#8794A1", lw=1.0, label="原始")
        ax.plot(t[selection], A[selection, filtered], color="#087F8C", lw=1.6, label="一阶低通")
        ax.set(xlabel="距采集起点 / s", ylabel=unit,
               title=f"{name}：{'完整记录' if row == 0 else '快速运动局部，观察延迟'}")
        ax.grid(alpha=.2); ax.legend(loc="best")
        if row == 0:
            ax.axvspan(25, t[-1], color="#F4B942", alpha=.13)
    ax = axs[2, col]
    f, p = signal.welch(uniform[:, raw], fs, nperseg=2048)
    _, q = signal.welch(uniform[:, filtered], fs, nperseg=2048)
    ax.loglog(f[1:], p[1:], color="#8794A1", label="原始")
    ax.loglog(f[1:], q[1:], color="#087F8C", label="一阶低通")
    ax.set(xlabel="频率 / Hz", ylabel=f"({unit})² / Hz", title=f"{name}：Welch 功率谱",
           xlim=(.1, 100))
    ax.grid(alpha=.2, which="both"); ax.legend()
fig.savefig(OUT / "imu_data2_comparison.png", dpi=150)
plt.close(fig)

fig, axs = plt.subplots(2, 2, figsize=(14, 8), constrained_layout=True)
fig.suptitle("当前一阶与同截止频率二阶 Butterworth 的理论比较（200 Hz）", fontsize=16)
for col, name, alpha in [(0, "Pitch", .09), (1, "Pitch 角速度", .25)]:
    m = metrics["channels"]["pitch" if col == 0 else "pitch_rate"]
    f = np.geomspace(.1, 80, 600)
    candidates = [("当前一阶", ([alpha], [1, -(1-alpha)]), "#087F8C"),
                  ("同截止频率二阶", signal.butter(2, m["cutoff_hz"], fs=fs), "#C06432")]
    for label, coeff, color in candidates:
        _, h = signal.freqz(*coeff, worN=f, fs=fs)
        axs[0, col].semilogx(f, 20*np.log10(np.maximum(abs(h), 1e-12)), label=label, color=color)
        axs[1, col].semilogx(f, -np.unwrap(np.angle(h))/(2*np.pi*f)*1000, label=label, color=color)
    axs[0, col].set(title=f"{name} 幅频响应，截止约 {m['cutoff_hz']:.2f} Hz",
                    ylabel="幅度 / dB", ylim=(-70, 2), xlabel="频率 / Hz")
    axs[1, col].set(title=f"{name} 相位等效延迟（随频率变化）",
                    ylabel="相位延迟 / ms", xlabel="频率 / Hz")
    for row in [0, 1]:
        axs[row, col].grid(alpha=.2, which="both"); axs[row, col].legend()
fig.savefig(OUT / "filter_order_tradeoff.png", dpi=150)
plt.close(fig)
(OUT / "metrics.json").write_text(json.dumps(metrics, indent=2, ensure_ascii=False), encoding="utf-8")
print(json.dumps(metrics, indent=2, ensure_ascii=False))
