"""Analyze the six captured IMU channels without modifying firmware or raw data."""
from pathlib import Path
import hashlib
import json
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import find_peaks

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
SOURCE = Path("E:/IMU平衡1.csv")
FS = 200.0  # User corrected the approximate acquisition rate to 200 Hz.
a = np.loadtxt(SOURCE, delimiter=",", skiprows=1)
assert a.ndim == 2 and a.shape[1] == 6 and np.isfinite(a).all()
n = len(a)
t = np.arange(n) / FS
task = (ROOT / "RtosTask/up_stair.cpp").read_text(encoding="utf-8")
fsm = (ROOT / "fsm/up_stair_behind_motor_fsm.hpp").read_text(encoding="utf-8")

def scalar(text, pattern):
    return float(re.search(pattern, text).group(1))

pitch_target = scalar(fsm, r"IMU_PITCH_TARGET_DEG\s*=\s*([\d.]+)f")
roll_target = scalar(fsm, r"ROLL_TARGET_DEG\s*=\s*([\d.]+)f")
pitch_kp = scalar(task, r"rear_6248_pitch_pid\s*=\s*\{\s*([\d.]+)f")
pitch_vmax = scalar(task, r"rear_6248_pitch_pid\s*=\s*\{\s*[\d.]+f,\s*[\d.]+f,\s*[\d.]+f,\s*([\d.]+)f")
mit_kd = scalar(task, r"REAR_IMU_MIT_KD\s*=\s*([\d.]+)f")
body_damping = scalar(task, r"REAR_IMU_PITCH_DAMPING\s*=\s*([\d.]+)f")

filters = {}
for name, j in [("pitch", 0), ("pitch_rate", 2), ("roll", 4)]:
    x = a[1:, j] - a[:-1, j + 1]
    y = np.diff(a[:, j + 1])
    mask = np.abs(x) > 0.01  # Ignore roundoff-dominated near-static points.
    alpha = float(x[mask] @ y[mask] / (x[mask] @ x[mask]))
    residual = y - alpha * x
    residual_08 = y - 0.8 * x
    filters[name] = {
        "fitted_alpha": alpha,
        "fit_points": int(mask.sum()),
        "recurrence_rmse": float(np.sqrt(np.mean(residual**2))),
        "recurrence_max_abs_error": float(np.max(np.abs(residual))),
        "alpha_0_8_recurrence_rmse": float(np.sqrt(np.mean(residual_08**2))),
        "low_frequency_delay_samples": (1 - alpha) / alpha,
        "low_frequency_delay_ms_at_200hz": (1 - alpha) / alpha / FS * 1000,
    }

def replay(raw, alpha):
    output = np.empty_like(raw)
    output[0] = raw[0]
    for i in range(1, len(raw)):
        output[i] = alpha * raw[i] + (1 - alpha) * output[i - 1]
    return output

replay_rate = replay(a[:, 2], 0.8)
replay_roll = replay(a[:, 4], 0.8)
peaks, _ = find_peaks(a[:, 0], prominence=5.0, distance=50)
peak = int(peaks[0])
trough = peak + int(np.argmin(a[peak:peak + 65, 0]))
rebounds, _ = find_peaks(a[trough:trough + 150, 0], prominence=1.0)
rebound = trough + int(rebounds[0])
zoom = slice(peak - 37, peak + 164)
velocity_request = np.clip(pitch_kp * np.deg2rad(pitch_target - a[:, 1]),
                           -pitch_vmax, pitch_vmax)
damping_request = -body_damping * np.deg2rad(a[:, 3])
metrics = {
    "input": str(SOURCE),
    "sha256": hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
    "rows": n,
    "nominal_sample_rate_hz_user_supplied": FS,
    "approximate_duration_s": float(t[-1]),
    "timing_limit": "No timestamp column; nominal time assumes no missing frames or gaps.",
    "capture_context": "Repeated stair trials with B presses; command times and FSM states were not captured.",
    "channel_min": a.min(axis=0).tolist(),
    "channel_max": a.max(axis=0).tolist(),
    "filters": filters,
    "example_event": {
        "peak_sample": peak, "peak_time_s": float(t[peak]),
        "peak_pitch_deg": float(a[peak, 0]),
        "trough_sample": trough, "trough_time_s": float(t[trough]),
        "trough_pitch_deg": float(a[trough, 0]),
        "peak_to_trough_s": float(t[trough] - t[peak]),
        "rebound_sample": rebound, "rebound_time_s": float(t[rebound]),
        "rebound_pitch_deg": float(a[rebound, 0]),
        "max_raw_minus_filtered_pitch_deg": float(np.max(np.abs(a[:, 0] - a[:, 1]))),
    },
    "current_source_nominal_attitude_path": {
        "pitch_target_deg": pitch_target, "pitch_kp_per_s": pitch_kp,
        "speed_limit_rad_s": pitch_vmax, "mit_kd": mit_kd,
        "pitch_body_damping_nm_per_rad_s": body_damping,
        "max_abs_unprotected_pitch_speed_request_rad_s": float(np.max(np.abs(velocity_request))),
        "max_abs_unprotected_body_damping_nm": float(np.max(np.abs(damping_request))),
        "one_deg_pitch_error_speed_request_rad_s": float(pitch_kp * np.pi / 180),
        "one_deg_pitch_error_velocity_torque_term_nm_at_zero_motor_speed": float(mit_kd * pitch_kp * np.pi / 180),
        "30_dps_body_damping_nm": float(body_damping * np.pi / 6),
        "limit": "Calculated from current source, not measured motor commands/torque; inactive in retract and overridden in endpoint hold.",
    },
}
(OUT / "metrics.json").write_text(json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8")

plt.rcParams.update({"font.sans-serif": ["Microsoft YaHei", "SimHei", "DejaVu Sans"],
                     "axes.unicode_minus": False, "font.size": 10})
fig, axes = plt.subplots(3, 2, figsize=(15, 9), constrained_layout=True,
                         gridspec_kw={"width_ratios": [1.3, 1]})
for row, (j, title, unit, alpha, target) in enumerate([
    (0, "Pitch", "°", 0.20, pitch_target),
    (2, "Pitch角速度", "°/s", 0.25, 0.0),
    (4, "Roll", "°", 0.09, roll_target),
]):
    for col, selection in enumerate([slice(None), zoom]):
        ax = axes[row, col]
        ax.plot(t[selection], a[selection, j], color="#94a3b8", lw=1.0, label="原始测量")
        ax.plot(t[selection], a[selection, j+1], color="#2563eb", lw=1.3,
                label=f"录制滤波 α={alpha:.2f}")
        ax.axhline(target, color="#334155", lw=0.8, ls=":")
        if j != 2:
            ax.text(0.015, target + 0.04, f"姿态分支目标 {target:g}°",
                    transform=ax.get_yaxis_transform(), fontsize=8, color="#334155")
        if col == 1 and j in (2, 4):
            replayed = replay_rate if j == 2 else replay_roll
            ax.plot(t[selection], replayed[selection], color="#e87924", lw=1.0,
                    ls="--", label="离线重放 α=0.8")
        if col == 0:
            ax.axvspan(t[zoom.start], t[zoom.stop-1], color="#f59e0b", alpha=0.15)
        ax.set_ylabel(f"{title} ({unit})")
        ax.grid(alpha=0.20)
        ax.set_title("全程：反复上台与 B 操作" if col == 0 and row == 0 else
                     "首个明显回落与再次抬升（动作时刻未知）" if row == 0 else "")
        ax.legend(loc="upper right", fontsize=8)
        if row == 2:
            ax.set_xlabel("估算时间 (s)，按用户给定约 200 Hz；CSV 无时间戳")
ax = axes[0, 1]
for idx, offset in [(peak, (-25, -28)), (trough, (12, 18)), (rebound, (8, 20))]:
    ax.scatter(t[idx], a[idx, 0], s=24, color="#b91c1c", zorder=5)
    ax.annotate(f"{a[idx,0]:.3f}°", (t[idx], a[idx,0]), textcoords="offset points",
                xytext=offset, fontsize=9, arrowprops={"arrowstyle": "-", "color": "#b91c1c"})
fig.suptitle("IMU平衡1：原始测量与录制滤波\n"
             "约200Hz；Pitch α=0.20、Pitch角速度 α=0.25、Roll α=0.09；橙线仅重放滤波，不预测车身响应", fontsize=14)
fig.savefig(OUT / "imu_balance1.png", dpi=170)
plt.close(fig)

report = f"""# IMU平衡1 数据分析

输入：`{SOURCE}`，SHA-256 `{metrics['sha256']}`。
共有 {n} 行六通道，全部为有限值。用户确认是反复上台、包含 B 操作，频率约 200 Hz。
按此频率估算总时长 {t[-1]:.3f} s；文件没有时间戳，无法检查实际间隔或丢帧。

## 确定的结果

| 量 | 原始范围 | 由数据反推的 α | 低频附加延迟（200 Hz） |
|---|---|---|---|
| Pitch | {a[:,0].min():.3f} ~ {a[:,0].max():.3f} ° | {filters['pitch']['fitted_alpha']:.8f} | 20.0 ms |
| Pitch角速度 | {a[:,2].min():.3f} ~ {a[:,2].max():.3f} °/s | {filters['pitch_rate']['fitted_alpha']:.8f} | 15.0 ms |
| Roll | {a[:,4].min():.3f} ~ {a[:,4].max():.3f} ° | {filters['roll']['fitted_alpha']:.8f} | 50.6 ms |

逐点拟合 `y[n]-y[n-1] = α*(x[n]-y[n-1])`。
三路递推残差 RMS 分别为 {filters['pitch']['recurrence_rmse']:.3g} °、
{filters['pitch_rate']['recurrence_rmse']:.3g} °/s、{filters['roll']['recurrence_rmse']:.3g} °。
因此这次采集并未采用角速度/Roll 的 0.8；不能作为 0.8 的效果测试。
当前源码 `up_stair.cpp:316-317` 仍在重建历史后恢复 0.25/0.09；102-103 行的构造值 0.8 被覆盖。
延迟是单极点滤波器低频群延迟 `(1-α)/α`，不是所有动作的固定时延，也未包含 IMU 内部和通信延迟。

首个明显过程：样本 {peak}（约 {t[peak]:.3f} s）Pitch={a[peak,0]:.3f}°，
到样本 {trough}（约 {t[trough]:.3f} s）降至 {a[trough,0]:.3f}°，历时约 {t[trough]-t[peak]:.3f} s，
随后样本 {rebound}（约 {t[rebound]:.3f} s）回升到 {a[rebound,0]:.3f}°。
样本 460 原始 Pitch=2.172°，滤波后=3.618°；这是快速回落时反馈仍偏高的一个例子。
角度变化与陀螺同向，支持存在实际转动；IMU 内部融合及车身线加速度的影响无法凭此文件排除。
反复操作意味着这些波峰不是同一次扰动的衰减包络，不能据此估计闭环阻尼比或认定自激振荡。

## 当前控制路径的限制

1. 姿态分支：Pitch P=55 /s，MIT Kd=3.5，车身 Pitch 阻尼=1 Nm/(rad/s)。
   1°角差产生约 0.960 rad/s 的轴速度请求；在零电机轴速、未保护且无端点接管的假设下，
   相应速度项约 3.360 Nm。30°/s 的独立车身阻尼仅 0.524 Nm；本次最大该阻尼请求约
   {np.max(np.abs(damping_request)):.3f} Nm。这不代表系统全部阻尼，MIT 本身也含电机轴速度反馈。
2. 收腿/收腿保持分支（`up_stair.cpp:450`）走编码器位置环，不叠加上述 IMU Pitch 阻尼或 Roll 补偿。
   因此在该阶段调大 `REAR_IMU_PITCH_DAMPING` 无效。
3. 姿态分支中的机械端点保持会用位置 P 接管目标，并将该侧姿态前馈置零。
4. CSV 未记录 B 事件、后 FSM 状态、左右端点保持、后腿角度/轴速度或最终命令，
   无法判断每次回摆属于姿态控制、收腿、端点切换还是外部台阶冲击。
   当前源码参数也不能证明录制时烧录的全部参数相同；唯有滤波系数可直接从六通道验证。

## 建议顺序

先将用户希望使用的两路 0.8 同步到初始化/恢复路径，暂时固定其它参数，形成可复现基线。
0.8 的低频附加延迟约 1.25 ms，但它只改变测量反馈，不能保证消除物理回摆；图中橙线仅为离线测量重放。
下一次固定相同台阶、轮速与按键顺序，并记录 FSM 状态、B/收腿请求及左右端点保持标志。
若回摆明确发生于姿态调平，分别试验角度 P 的降低和车身角速度阻尼的增加，一次只改一项。
若回摆发生于收腿或端点保持，应检查对应编码器位置环、目标变化与模式交接，避免误调不生效的姿态参数。
不凭本次文件给出已验证的最终增益，也不将机械台阶的必要姿态变化设为绝对零。

本次只生成分析文件，未改动、编译或烧录固件。
"""
(OUT / "report.md").write_text(report, encoding="utf-8")
print(json.dumps({"rows": n, "approximate_duration_s": t[-1], "filters": filters,
                  "example_event": metrics["example_event"],
                  "nominal_attitude_path": metrics["current_source_nominal_attitude_path"],
                  "chart": str(OUT / "imu_balance1.png")}, ensure_ascii=False, indent=2))
