"""Compare four stair captures; keep raw measurements separate from filter lag."""
from pathlib import Path
import hashlib
import json

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import find_peaks

OUT = Path(__file__).resolve().parent
FS = 200.0
EXPECTED = [(0.2, 0.25, 0.09), (0.4, 0.25, 0.09),
            (0.4, 0.5, 0.09), (0.4, 0.5, 0.3)]
names = ["Pitch", "Pitch角速度", "Roll"]
captures = []
series = []

for number, expected in enumerate(EXPECTED, start=1):
    path = Path(f"E:/IMU平衡{number}.csv")
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    assert data.ndim == 2 and data.shape[1] == 6 and np.isfinite(data).all()
    series.append(data)
    filters = []
    for j, alpha_expected in zip((0, 2, 4), expected):
        x = data[1:, j] - data[:-1, j+1]
        y = np.diff(data[:, j+1])
        mask = np.abs(x) > 0.01
        alpha = float(x[mask] @ y[mask] / (x[mask] @ x[mask]))
        residual = y - alpha * x
        # Independently verify the declared experimental setting against all pairs.
        expected_error = y - alpha_expected * x
        assert abs(alpha - alpha_expected) < 1e-5
        assert np.max(np.abs(expected_error)) < 1e-5
        filters.append({
            "alpha": alpha,
            "recurrence_rmse": float(np.sqrt(np.mean(residual**2))),
            "max_recurrence_error_at_expected_alpha": float(np.max(np.abs(expected_error))),
            "low_frequency_delay_ms": (1-alpha)/alpha/FS*1000,
            "raw_filtered_rms_difference": float(np.sqrt(np.mean((data[:, j]-data[:, j+1])**2))),
        })
    peaks, _ = find_peaks(data[:, 0], height=7.9, prominence=5.0, distance=150)
    events = []
    for peak in peaks:
        # These are geometric selections, not verified B events or FSM transitions.
        stop = min(len(data), peak+81)
        trough = int(peak + np.argmin(data[peak:stop, 0]))
        after = min(len(data), trough+151)
        rebound = int(trough + np.argmax(data[trough:after, 0]))
        events.append({
            "peak_sample": int(peak), "peak_time_s": float(peak/FS),
            "peak_pitch_deg": float(data[peak, 0]),
            "trough_sample": trough, "trough_pitch_deg": float(data[trough, 0]),
            "peak_to_trough_s": float((trough-peak)/FS),
            "min_pitch_rate_dps_until_trough": float(data[peak:trough+1, 2].min()),
            "post_trough_0_75s_pitch_max_deg": float(data[rebound, 0]),
            "post_trough_0_75s_max_at_window_edge": bool(rebound == after-1),
            "post_trough_window_limit": "May include another deliberate command; not a measured overshoot.",
            "roll_peak_to_peak_deg_in_peak_window": float(np.ptp(data[max(0, peak-20):after, 4])),
        })
    captures.append({
        "number": number, "file": str(path),
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "rows": len(data), "approximate_duration_s_at_200hz": (len(data)-1)/FS,
        "raw_pitch_min_deg": float(data[:, 0].min()),
        "raw_pitch_max_deg": float(data[:, 0].max()),
        "raw_pitch_rate_min_dps": float(data[:, 2].min()),
        "raw_pitch_rate_max_dps": float(data[:, 2].max()),
        "raw_roll_min_deg": float(data[:, 4].min()),
        "raw_roll_max_deg": float(data[:, 4].max()),
        "filters": filters, "high_peak_event_count": len(events), "events": events,
        "high_peak_trough_median_deg": float(np.median([e["trough_pitch_deg"] for e in events])),
        "high_peak_fall_rate_median_dps": float(np.median([e["min_pitch_rate_dps_until_trough"] for e in events])),
        "last_100_raw_pitch_peak_to_peak_deg": float(np.ptp(data[-100:, 0])),
        "last_100_raw_roll_peak_to_peak_deg": float(np.ptp(data[-100:, 4])),
    })

result = {
    "nominal_sample_rate_hz_user_supplied": FS,
    "time_limit": "No timestamps or acquisition counters; time assumes 200 Hz without capture gaps.",
    "comparison_limit": "Repeated trials include B operations. No command/FSM/endpoint/motor channels; no controlled causal ranking.",
    "user_observed_issue_phase": "First B: front legs extend while rear legs remain in attitude control.",
    "selection_rule": "Raw Pitch local maxima >=7.9 deg, prominence >=5 deg, separation >=150 samples; minimum within next 80 samples.",
    "captures": captures,
}
(OUT / "metrics.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")

plt.rcParams.update({"font.sans-serif": ["Microsoft YaHei", "SimHei", "DejaVu Sans"],
                     "axes.unicode_minus": False, "font.size": 10})
colors = ["#475569", "#2563eb", "#059669", "#d97706"]
fig, axes = plt.subplots(2, 4, figsize=(17, 7.3), sharex=True, sharey="row", constrained_layout=True)
for col, (data, capture, color) in enumerate(zip(series, captures, colors)):
    for row, j in enumerate((0, 4)):
        ax = axes[row, col]
        for event in capture["events"]:
            peak = event["peak_sample"]
            start, stop = max(0, peak-30), min(len(data), peak+151)
            ax.plot((np.arange(start, stop)-peak)/FS, data[start:stop, j],
                    color=color, alpha=0.6, lw=1.1)
        ax.axvline(0, color="#94a3b8", lw=0.7, ls=":")
        ax.grid(alpha=0.2)
        if row == 0:
            alpha_text = "/".join(f"{f['alpha']:.2f}" for f in capture["filters"])
            ax.set_title(f"第{col+1}组：{alpha_text}\n{capture['high_peak_event_count']}次相近高峰", fontsize=11)
            ax.axhline(1.0, color="#94a3b8", lw=0.8, ls="--")
            ax.set_ylim(-0.3, 9.5)
        else:
            ax.set_xlabel("相对 Pitch 高峰时间 (s)")
            ax.set_ylim(-0.4, 2.5)
        if col == 0:
            ax.set_ylabel("原始 Pitch (°)" if row == 0 else "原始 Roll (°)")
        ax.set_xlim(-0.15, 0.75)
    lowest = min(capture["events"], key=lambda e: e["trough_pitch_deg"])
    ax = axes[0, col]
    rel = (lowest["trough_sample"]-lowest["peak_sample"])/FS
    ax.scatter(rel, lowest["trough_pitch_deg"], s=24, color="#b91c1c", zorder=4)
    ax.annotate(f"最低 {lowest['trough_pitch_deg']:.3f}°", (rel, lowest["trough_pitch_deg"]),
                xytext=(7, 15), textcoords="offset points", fontsize=9,
                arrowprops={"arrowstyle": "-", "color": "#b91c1c"})
fig.suptitle("四组实测原始姿态：约8°高峰对齐\n"
             "每条线为一次选中过程；系数顺序为Pitch/角速度/Roll；按约200Hz估时，B操作时刻未记录", fontsize=14)
fig.savefig(OUT / "matched_events.png", dpi=170)
plt.close(fig)

fig, axes = plt.subplots(3, 4, figsize=(17, 9), sharex="col", sharey="row", constrained_layout=True)
for col, (data, capture) in enumerate(zip(series, captures)):
    time = np.arange(len(data))/FS
    for row, j in enumerate((0, 2, 4)):
        ax = axes[row, col]
        ax.plot(time, data[:, j], lw=0.75, color="#94a3b8", label="原始")
        ax.plot(time, data[:, j+1], lw=0.9, color=colors[col], label="滤波")
        ax.grid(alpha=0.2)
        if row == 0:
            ax.set_title(f"第{col+1}组；{capture['rows']}点\nα="+
                         "/".join(f"{f['alpha']:.2f}" for f in capture["filters"]), fontsize=11)
            ax.legend(fontsize=8)
        if col == 0:
            ax.set_ylabel(["Pitch (°)", "Pitch角速度 (°/s)", "Roll (°)"][row])
        if row == 2:
            ax.set_xlabel("估算时间 (s)")
fig.suptitle("四份记录全程：原始值与滤波值\n不同记录时长与B操作节奏不同，全程峰谷不能直接用作调参效果排名", fontsize=14)
fig.savefig(OUT / "all_captures.png", dpi=170)
plt.close(fig)

lines = ["# IMU平衡1—4 对比", "",
         "采样频率按用户确认的约200Hz估算。各CSV均为I0/I1 Pitch原始/滤波，I2/I3角速度原始/滤波，I4/I5 Roll原始/滤波。",
         "数据均为有限值；没有时间列、按键事件、后腿状态、端点标志和电机反馈/命令。", "",
         "## 滤波参数确实生效", "",
         "使用逐点递推关系拟合α，并独立代入实验设置检查全部相邻点；最大残差均小于1e-5。",
         "延迟是单极点滤波器的低频附加群延迟，不包含IMU内部、通信及控制环延迟。", "",
         "|组|Pitch α|角速度 α|Roll α|Pitch/角速度/Roll低频延迟 ms|", "|---|---|---|---|---|"]
for capture in captures:
    f = capture["filters"]
    lines.append(f"|{capture['number']}|{f[0]['alpha']:.2f}|{f[1]['alpha']:.2f}|{f[2]['alpha']:.2f}|"+
                 "/".join(f"{x['low_frequency_delay_ms']:.1f}" for x in f)+"|")
lines += ["", "## 实际运动与反馈滞后分开判断", "",
          "|组|原始Pitch范围 °|原始Pitch角速度范围 °/s|原始Roll范围 °|约8°事件数|事件谷值中位数 °|事件回落角速度中位数 °/s|",
          "|---|---|---|---|---|---|---|"]
for c in captures:
    lines.append(f"|{c['number']}|{c['raw_pitch_min_deg']:.3f}~{c['raw_pitch_max_deg']:.3f}|"
                 f"{c['raw_pitch_rate_min_dps']:.2f}~{c['raw_pitch_rate_max_dps']:.2f}|"
                 f"{c['raw_roll_min_deg']:.3f}~{c['raw_roll_max_deg']:.3f}|{c['high_peak_event_count']}|"
                 f"{c['high_peak_trough_median_deg']:.3f}|{c['high_peak_fall_rate_median_dps']:.2f}|")
lines += ["", "选取规则：原始Pitch局部峰值>=7.9°，显著度>=5°，间隔>=150点；谷值取峰后80点内最低值。",
          "该规则只匹配几何形态，不表示FSM状态和外部输入一致；事件个数不同。不能将谷值中位数直接解释为因滤波导致的改善率。", "",
          "1. 第2组角度滤波0.4生效，反馈延迟减少，但相似高峰后的负角速度仍约-79°/s。",
          "2. 第3组角速度滤波0.5生效，角速度反馈更及时；原始回落速度量级没有继续明显下降。",
          "3. 第4组Roll滤波0.3生效，Roll附加延迟从约50.6ms减少到11.7ms；其原始Roll全程范围稍小，",
          "   但原始Pitch仍出现0.083°谷值及-84.57°/s，不能据此宣称第4组最稳，也不能将差异单独归因于Roll。",
          "4. 对齐图仍存在不同的后续运动形态。部分后谷0.75s最大值落在窗口右边界，可能是下一次动作正在开始，",
          "   因此报告没有把该值当作真实超调，亦没有从重复B操作中估计闭环阻尼比或单一振荡频率。", "",
          "## 下一步", "",
          "用户已确认关注第一次B：前腿伸出、后腿调平时的车尾晃动。滤波变更已经做到预期的减小滞后，下一轮应集中验证姿态分支。",
          "建议将第4组0.4/0.5/0.3固定作为已知基线（不是已证实最优），保持其它增益、台阶和轮速不变，",
          "仅将REAR_IMU_PITCH_DAMPING从1试增至2，比较同一动作后的原始Pitch反向摆幅、回落角速度与稳定时间。",
          "在30°/s下，该项请求由0.524增至1.047Nm；60°/s下由1.047增至2.094Nm。这是保护前计算值，不是已测力矩或保证有效的最终增益。",
          "同时记录后腿FSM状态、收腿请求及左右端点保持；即使仍在IMU模式，端点位置保持也可能将该侧姿态前馈清零。",
          "若晃动时已由端点保持接管，就应检查对应位置环及切换，继续增加Pitch阻尼不能解决该阶段的问题。",
          "当前源码的Pitch阻尼仍为1 Nm/(rad/s)，这是源码事实，不是所有采集版本或实际输出力矩的证明。",
          "本次不修改、编译或烧录固件。", "", "## 输入追溯", ""]
for c in captures:
    lines.append(f"- 第{c['number']}组：`{c['file']}`，{c['rows']}点，约{c['approximate_duration_s_at_200hz']:.3f}s；SHA-256 `{c['sha256']}`。")
(OUT / "report.md").write_text("\n".join(lines)+"\n", encoding="utf-8")
for c in captures:
    print(json.dumps({key: value for key, value in c.items()
                      if key not in ("events", "sha256", "filters")}, ensure_ascii=False))
print("PASS: all four settings independently verified against every adjacent CSV row.")
