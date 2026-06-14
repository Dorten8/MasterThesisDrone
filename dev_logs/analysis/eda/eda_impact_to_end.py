#!/usr/bin/env python3
"""eda_impact_to_end.py — Impact-to-Experiment-End Timing Analysis.

Compares Fixed vs Rotating Cage on:
  1. Time from obstacle impact (e_impact_timestamp_PX4) to experiment end (e_ep_timestamp_PX4)
  2. Stats table (mean, median, std, min, max)
  3. Histogram + fitted normal distribution — separate panel per condition
  4. Impact angle vs time-to-end scatter with linear fit

Outputs (to graphics/):
  - eda_impact_to_end_histograms.png
  - eda_impact_angle_vs_time_to_end.png

Usage:
  python3 dev_logs/analysis/eda/eda_impact_to_end.py

Or import from a notebook:
  from dev_logs.analysis.eda.eda_impact_to_end import (
      load_timing_data,
      plot_histograms,
      plot_angle_vs_time,
  )
"""

import sys
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from scipy import stats

# ── Path resolution ──────────────────────────────────────────────────────────
_project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
GRAPHICS_DIR = os.path.join(THIS_DIR, "..", "graphics")
os.makedirs(GRAPHICS_DIR, exist_ok=True)

# ── Professional plotting style (thesis, matches eda_angle_prediction.py) ─────
plt.rcParams.update({
    'font.family': 'sans-serif',
    'font.size': 11,
    'axes.labelsize': 12,
    'axes.titlesize': 13,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'legend.fontsize': 10,
    'grid.linestyle': '--',
    'grid.alpha': 0.5,
    'figure.titlesize': 14,
    'figure.dpi': 150,
})

# ── DB path ───────────────────────────────────────────────────────────────────
DB_PATH = os.path.join(_project_root, "dev_logs", "analysis", "experiments_summary.db")
if not os.path.exists(DB_PATH):
    # _project_root might be wrong; resolve relative to this file
    DB_PATH = os.path.join(THIS_DIR, "..", "experiments_summary.db")

CONDITION_COLORS = {
    'Rotating Cage': '#1F77B4',   # blue
    'Fixed Cage': '#D62728',      # red
}


# ══════════════════════════════════════════════════════════════════════════════
#  1.  Data Loading
# ══════════════════════════════════════════════════════════════════════════════

def load_timing_data():
    """Load impact-to-end timing data from experiments_summary.db.

    Returns
    -------
    pd.DataFrame
        Columns: flight_name, condition, impact_angle,
                 time_to_end_s (impact → experiment end),
                 total_flight_s (start → experiment end).
    """
    import sqlite3
    con = sqlite3.connect(DB_PATH)

    # Query the flights_summary table for impact-detected flights that have
    # valid PX4 timestamps. We pull:
    #   e_impact_timestamp_PX4 — moment the drone hit the obstacle
    #   e_ep_timestamp_PX4     — moment the experiment ended (end-point)
    #   e_sp_timestamp_PX4     — moment the experiment started (start-point)
    # The WHERE clause ensures we only analyze flights where both impact and
    # end timestamps are recorded (NULLs would produce meaningless durations).
    df = pd.read_sql("""
        SELECT flight_name, condition, impact_angle,
               e_impact_timestamp_PX4, e_ep_timestamp_PX4,
               e_sp_timestamp_PX4
        FROM flights_summary
        WHERE impact_detected = 1
          AND e_impact_timestamp_PX4 IS NOT NULL
          AND e_ep_timestamp_PX4 IS NOT NULL
        ORDER BY condition, flight_name
    """, con)
    con.close()

    # PX4 timestamps are in microseconds → convert to seconds by dividing by
    # 1e6 (1,000,000 µs = 1 s). The difference gives the elapsed wall-clock
    # duration between two flight events.
    df['time_to_end_s'] = (df['e_ep_timestamp_PX4'] - df['e_impact_timestamp_PX4']) / 1e6
    df['total_flight_s'] = (df['e_ep_timestamp_PX4'] - df['e_sp_timestamp_PX4']) / 1e6

    print(f"📡 Loaded {len(df)} impact flights with timing data")
    for cond in ['Fixed Cage', 'Rotating Cage']:
        n = (df['condition'] == cond).sum()
        print(f"   → {cond}: {n} flights")

    return df


# ══════════════════════════════════════════════════════════════════════════════
#  2.  Stats helper
# ══════════════════════════════════════════════════════════════════════════════

def compute_stats(df):
    """Return per-condition summary stats dict."""
    stats_dict = {}
    for cond in ['Fixed Cage', 'Rotating Cage']:
        sub = df[df['condition'] == cond]['time_to_end_s']
        stats_dict[cond] = {
            'N': len(sub),
            'mean': sub.mean(),
            'median': sub.median(),
            'std': sub.std(),
            'min': sub.min(),
            'max': sub.max(),
        }
    return stats_dict


# ══════════════════════════════════════════════════════════════════════════════
#  3.  Histogram with fitted normal — separate panel per condition
# ══════════════════════════════════════════════════════════════════════════════

def plot_histograms(df, save_path=None, show=True):
    """Two-panel figure: histogram + fitted normal curve for each condition.

    Parameters
    ----------
    df : pd.DataFrame
        Output of load_timing_data().
    save_path : str or None
        If given, saves the figure to disk at this path.
    show : bool
        If True, calls plt.show() for inline notebook display.
    """
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), dpi=150, sharey=False)

    for ax, cond in zip(axes, ['Fixed Cage', 'Rotating Cage']):
        data = df[df['condition'] == cond]['time_to_end_s'].values
        color = CONDITION_COLORS[cond]
        n = len(data)

        # ── Histogram ─────────────────────────────────────────────────────
        # density=True normalizes the histogram so the total area equals 1
        # (a probability density), not raw counts. This makes the histogram
        # directly comparable with the fitted normal PDF curve overlaying it.
        bins = 12
        counts, bin_edges, patches = ax.hist(
            data, bins=bins, density=True, alpha=0.55, color=color,
            edgecolor='white', linewidth=0.8, label=f'{cond} (N={n})'
        )

        # ── Fitted normal curve ───────────────────────────────────────────
        # MLE (Maximum Likelihood Estimate) for the normal distribution:
        #   μ̂ = sample mean,  σ̂ = sample std with Bessel's correction (ddof=1).
        # Using ddof=1 gives an unbiased estimate of the population std, which
        # is the standard MLE convention for normal-distribution fitting.
        mu, sigma = np.mean(data), np.std(data, ddof=1)
        x_smooth = np.linspace(data.min() - 0.5, data.max() + 0.5, 300)
        # stats.norm.pdf returns the Gaussian probability density at each x:
        #   f(x) = (1 / (σ√(2π))) · exp(−½ ((x−μ)/σ)²)
        pdf = stats.norm.pdf(x_smooth, mu, sigma)
        ax.plot(x_smooth, pdf, color='#333333', linewidth=2.0, linestyle='-',
                label=f'Normal fit\n$\\mu={mu:.2f}$, $\\sigma={sigma:.2f}$')

        # ── Rug plot on bottom ────────────────────────────────────────────
        # A rug plot draws a small vertical tick at each data point along the
        # bottom axis. It shows the exact sample locations and reveals gaps,
        # clusters, or potential outliers that the histogram bins might hide.
        ax.plot(data, np.zeros_like(data) - 0.02 * ax.get_ylim()[1],
                '|', color=color, markersize=6, markeredgewidth=1.0, alpha=0.6)

        # ── Stats annotation ──────────────────────────────────────────────
        # IQR (Interquartile Range) = Q75 − Q25, the spread of the middle 50%
        # of the data. It is a robust measure of dispersion, unlike std which
        # is inflated by outliers. Displaying both Q25 and Q75 gives a compact
        # summary of the distribution's central spread.
        q25, q75 = np.percentile(data, [25, 75])
        stats_text = (
            f"N = {n}\n"
            f"Mean = {mu:.2f} s\n"
            f"Median = {np.median(data):.2f} s\n"
            f"Std = {sigma:.2f} s\n"
            f"IQR = [{q25:.2f}, {q75:.2f}] s"
        )
        ax.text(0.97, 0.97, stats_text, transform=ax.transAxes,
                ha='right', va='top', fontsize=9,
                bbox=dict(boxstyle='round,pad=0.4', facecolor='white',
                          alpha=0.9, edgecolor='#CCCCCC'))

        ax.set_xlabel('Time from Impact to End (s)', fontweight='bold')
        ax.set_ylabel('Probability Density', fontweight='bold')
        ax.set_title(f'<{cond}>', fontweight='bold', fontsize=13)
        ax.legend(fontsize=9, framealpha=0.85)
        ax.grid(True, linestyle=':', alpha=0.4)
        ax.set_xlim(left=4.5)

    # ── Shared title ──────────────────────────────────────────────────────
    fig.suptitle('Impact-to-Experiment-End Time Distribution\n(Fixed vs Rotating Cage)',
                 fontweight='bold', fontsize=14, y=1.02)

    # ── Data origin ───────────────────────────────────────────────────────
    total_n = len(df)
    fig.text(0.98, 0.01,
             f"Source: experiments_summary.db / flights_summary (impact_detected=1, N={total_n})",
             ha='right', va='bottom', fontsize=7.5, fontweight='bold',
             bbox=dict(boxstyle='round,pad=0.15', facecolor='white', alpha=0.8, edgecolor='none'))

    fig.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"   💾 Saved histograms → {os.path.relpath(save_path, THIS_DIR)}")

    if show:
        plt.show()
    plt.close(fig)


# ══════════════════════════════════════════════════════════════════════════════
#  4.  Impact angle vs time-to-end scatter
# ══════════════════════════════════════════════════════════════════════════════

def plot_angle_vs_time(df, save_path=None, show=True):
    """Scatter: impact_angle (x) vs time_to_end_s (y), colored by condition.

    Includes linear fit per condition and summary stats annotation.

    Parameters
    ----------
    df : pd.DataFrame
        Output of load_timing_data().
    save_path : str or None
        If given, saves the figure to disk at this path.
    show : bool
        If True, calls plt.show() for inline notebook display.
    """
    fig, ax = plt.subplots(figsize=(9, 6), dpi=150)

    for cond in ['Fixed Cage', 'Rotating Cage']:
        sub = df[df['condition'] == cond]
        x = sub['impact_angle'].values
        y = sub['time_to_end_s'].values
        color = CONDITION_COLORS[cond]

        # ── Scatter ───────────────────────────────────────────────────────
        ax.scatter(x, y, c=color, alpha=0.7, s=65, edgecolor='white',
                   linewidth=0.5, zorder=3, label=f'{cond} (N={len(sub)})')

        # ── Linear fit ────────────────────────────────────────────────────
        # scipy.stats.linregress performs ordinary least-squares (OLS) linear
        # regression: it finds slope and intercept that minimize Σ(yᵢ − (slope·xᵢ + intercept))².
        # Returns:
        #   slope     — change in time_to_end per 1° change in impact_angle
        #   intercept — predicted time_to_end when impact_angle = 0°
        #   r_val     — Pearson r (same sign as slope, measures linear association)
        #   p_val     — two-tailed p-value for the null hypothesis that slope = 0
        if len(x) > 2:
            slope, intercept, r_val, p_val, _ = stats.linregress(x, y)
            x_smooth = np.linspace(x.min(), x.max(), 100)
            y_fit = slope * x_smooth + intercept
            ax.plot(x_smooth, y_fit, color=color, linewidth=2.0, linestyle='--',
                    alpha=0.8, zorder=4,
                    label=f'{cond} fit: r={r_val:.3f}, p={p_val:.3f}')

    ax.set_xlabel('Impact Angle (°)', fontweight='bold')
    ax.set_ylabel('Time from Impact to Experiment End (s)', fontweight='bold')
    ax.set_title('Impact Angle vs. Post-Impact Duration\n(Fixed vs Rotating Cage)',
                 fontweight='bold', fontsize=13)
    ax.legend(fontsize=9.5, framealpha=0.85, loc='upper left')
    ax.grid(True, linestyle=':', alpha=0.4)

    # ── Tick formatting ──────────────────────────────────────────────────
    ax.xaxis.set_major_locator(ticker.MultipleLocator(10))
    ax.yaxis.set_major_locator(ticker.MultipleLocator(1))

    # ── Data origin ───────────────────────────────────────────────────────
    total_n = len(df)
    fig.text(0.98, 0.01,
             f"Source: experiments_summary.db / flights_summary (impact_detected=1, N={total_n})",
             ha='right', va='bottom', fontsize=7.5, fontweight='bold',
             bbox=dict(boxstyle='round,pad=0.15', facecolor='white', alpha=0.8, edgecolor='none'))

    fig.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"   💾 Saved angle-vs-time → {os.path.relpath(save_path, THIS_DIR)}")

    if show:
        plt.show()
    plt.close(fig)


# ══════════════════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════════════════

def main():
    print("=" * 70)
    print("⏱️  Impact-to-Experiment-End Timing Analysis")
    print("=" * 70)

    # 1. Load data
    df = load_timing_data()
    print()

    # 2. Stats
    print("📊 Summary statistics:")
    stats_dict = compute_stats(df)
    for cond, s in stats_dict.items():
        print(f"  {cond}:")
        print(f"    N      = {s['N']}")
        print(f"    Mean   = {s['mean']:.2f} s")
        print(f"    Median = {s['median']:.2f} s")
        print(f"    Std    = {s['std']:.2f} s")
        print(f"    Min    = {s['min']:.2f} s")
        print(f"    Max    = {s['max']:.2f} s")
    print()

    # 3. Histograms + normal fit
    print("📊 Generating histogram + normal distribution plots...")
    plot_histograms(
        df,
        save_path=os.path.join(GRAPHICS_DIR, "eda_impact_to_end_histograms.png"),
        show=False,
    )
    print()

    # 4. Angle vs time-to-end scatter
    print("🎯 Generating impact angle vs time-to-end scatter...")
    plot_angle_vs_time(
        df,
        save_path=os.path.join(GRAPHICS_DIR, "eda_impact_angle_vs_time_to_end.png"),
        show=False,
    )
    print()

    print(f"✅ All plots saved to: {os.path.relpath(GRAPHICS_DIR, THIS_DIR)}/")
    print()

    # 5. Between-condition comparison
    from scipy.stats import ttest_ind, mannwhitneyu
    fixed_data = df[df['condition'] == 'Fixed Cage']['time_to_end_s']
    rot_data = df[df['condition'] == 'Rotating Cage']['time_to_end_s']

    # Welch's t-test (equal_var=False): a variant of the independent-samples
    # t-test that does NOT assume equal variances between groups. It uses the
    # Welch-Satterthwaite approximation for degrees of freedom, making it
    # robust when sample sizes and variances differ across conditions.
    t_stat, t_p = ttest_ind(fixed_data, rot_data, equal_var=False)
    # Mann-Whitney U test: a non-parametric rank-based test for whether one
    # distribution tends to have larger values than the other. Unlike the t-test,
    # it makes no normality assumption — it only assumes independent observations
    # and ordinal comparability. The null hypothesis is that a randomly drawn
    # value from Fixed Cage is equally likely to be larger or smaller than one
    # from Rotating Cage.
    u_stat, u_p = mannwhitneyu(fixed_data, rot_data, alternative='two-sided')

    print("=" * 70)
    print("🔬 Between-condition comparison:")
    print(f"  Welch's t-test:  t = {t_stat:.3f}, p = {t_p:.4f}")
    print(f"  Mann-Whitney U:  U = {u_stat:.0f}, p = {u_p:.4f}")
    # Cohen's d effect size: (μ₁ − μ₂) / σ_pooled, where σ_pooled = sqrt((σ₁² + σ₂²) / 2).
    # This is the standardized mean difference, independent of sample size.
    # Interpretations: |d| ≈ 0.2 = small, 0.5 = medium, 0.8 = large effect.
    print(f"  Cohen's d:       d = {(fixed_data.mean() - rot_data.mean()) / np.sqrt((fixed_data.var() + rot_data.var()) / 2):.3f}")
    print()


if __name__ == "__main__":
    main()
