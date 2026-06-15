#!/usr/bin/env python3
# ── Headless-safe backend (Agg when no display, notebook overrides) ──────────
import os, sys as _sys
if not os.environ.get("DISPLAY") and "JPY_SESSION_NAME" not in os.environ:
    import matplotlib as _mpl; _mpl.use("Agg")

"""
eda_angle_prediction.py — Exploratory Data Analysis for Impact Angle Prediction

Goal: Visually prove that impact_angle leaves a distinct, measurable footprint
in high-frequency IMU data. Supports both Fixed Cage and Rotating Cage
(parameterized via `condition` argument).

Outputs (to graphics/):
  - eda_correlation_heatmap.png     Pearson r of every IMU feature vs impact_angle
  - eda_top3_scatter.png            Top 3 features by |r|, each with Huber trendline
  - eda_parallel_coordinates.png    (Bonus) Multi-dimensional angle-group view

Usage:
  python3 dev_logs/analysis/eda/eda_angle_prediction.py

Or import from a notebook:
  from dev_logs.analysis.eda.eda_angle_prediction import (
      load_impact_data,
      plot_correlation_heatmap,
      plot_top3_scatter,
      plot_parallel_coordinates,
  )
"""

import sys
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from scipy.stats import pearsonr

# ── Path resolution ──────────────────────────────────────────────────────────
# When run as a script, resolve project root 2 levels up from dev_logs/analysis/
_project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

# Import db_manager directly (not through database/__init__.py which pulls in
# MCAP-heavy dependencies like db_pipeline / mcap_ros2)
import importlib.util
_db_manager_path = os.path.join(os.path.dirname(__file__), "..", "database", "db_manager.py")
_spec = importlib.util.spec_from_file_location("_db_manager_mod", _db_manager_path)
_db_manager_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_db_manager_mod)
get_database_df = _db_manager_mod.get_database_df

# ── Constants ────────────────────────────────────────────────────────────────
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
GRAPHICS_DIR = os.path.join(THIS_DIR, "..", "graphics")
os.makedirs(GRAPHICS_DIR, exist_ok=True)

# ── Professional plotting style (thesis) ────────────────────────────────────
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

# All IMU columns from flights_summary that capture impact dynamics
IMU_COLS = [
    # Peak accelerations (per-axis)
    'imu_peak_accel_x', 'imu_peak_accel_y', 'imu_peak_accel_z',
    # Peak gyroscope rates (per-axis)
    'imu_peak_gyro_x', 'imu_peak_gyro_y', 'imu_peak_gyro_z',
    # Integrated accel energy (per-axis)
    'imu_accel_energy_x', 'imu_accel_energy_y', 'imu_accel_energy_z',
    # Integrated gyro energy (per-axis)
    'imu_gyro_energy_x', 'imu_gyro_energy_y', 'imu_gyro_energy_z',
    # Vibration amplitude (accel axes)
    'imu_vib_ax', 'imu_vib_ay', 'imu_vib_az',
    # Vibration amplitude (gyro axes)
    'imu_vib_gx', 'imu_vib_gy', 'imu_vib_gz',
    # Settling times
    'imu_accel_settling', 'imu_gyro_settling',
    # Vibration spread during impact window
    'imu_ax_spread_impact', 'imu_ay_spread_impact', 'imu_az_spread_impact',
    # Vibration spread during regular flight
    'imu_ax_spread_regular', 'imu_ay_spread_regular', 'imu_az_spread_regular',
]

# Clean display labels for heatmap / plots
DISPLAY_NAMES = {
    'imu_peak_accel_x': 'Peak Accel X',
    'imu_peak_accel_y': 'Peak Accel Y',
    'imu_peak_accel_z': 'Peak Accel Z',
    'imu_peak_gyro_x': 'Peak Gyro X',
    'imu_peak_gyro_y': 'Peak Gyro Y',
    'imu_peak_gyro_z': 'Peak Gyro Z',
    'imu_accel_energy_x': 'Accel Energy X',
    'imu_accel_energy_y': 'Accel Energy Y',
    'imu_accel_energy_z': 'Accel Energy Z',
    'imu_gyro_energy_x': 'Gyro Energy X',
    'imu_gyro_energy_y': 'Gyro Energy Y',
    'imu_gyro_energy_z': 'Gyro Energy Z',
    'imu_vib_ax': 'Vibration Accel X',
    'imu_vib_ay': 'Vibration Accel Y',
    'imu_vib_az': 'Vibration Accel Z',
    'imu_vib_gx': 'Vibration Gyro X',
    'imu_vib_gy': 'Vibration Gyro Y',
    'imu_vib_gz': 'Vibration Gyro Z',
    'imu_accel_settling': 'Accel Settling Time',
    'imu_gyro_settling': 'Gyro Settling Time',
    'imu_ax_spread_impact': 'Accel Spread (Impact) X',
    'imu_ay_spread_impact': 'Accel Spread (Impact) Y',
    'imu_az_spread_impact': 'Accel Spread (Impact) Z',
    'imu_ax_spread_regular': 'Accel Spread (Regular) X',
    'imu_ay_spread_regular': 'Accel Spread (Regular) Y',
    'imu_az_spread_regular': 'Accel Spread (Regular) Z',
}

# Which group each feature belongs to (for heatmap coloring)
FEATURE_GROUPS = {
    'imu_peak_accel_x': 'Peak Accel', 'imu_peak_accel_y': 'Peak Accel', 'imu_peak_accel_z': 'Peak Accel',
    'imu_peak_gyro_x': 'Peak Gyro', 'imu_peak_gyro_y': 'Peak Gyro', 'imu_peak_gyro_z': 'Peak Gyro',
    'imu_accel_energy_x': 'Accel Energy', 'imu_accel_energy_y': 'Accel Energy', 'imu_accel_energy_z': 'Accel Energy',
    'imu_gyro_energy_x': 'Gyro Energy', 'imu_gyro_energy_y': 'Gyro Energy', 'imu_gyro_energy_z': 'Gyro Energy',
    'imu_vib_ax': 'Vibration Accel', 'imu_vib_ay': 'Vibration Accel', 'imu_vib_az': 'Vibration Accel',
    'imu_vib_gx': 'Vibration Gyro', 'imu_vib_gy': 'Vibration Gyro', 'imu_vib_gz': 'Vibration Gyro',
    'imu_accel_settling': 'Settling', 'imu_gyro_settling': 'Settling',
    'imu_ax_spread_impact': 'Spread Impact', 'imu_ay_spread_impact': 'Spread Impact', 'imu_az_spread_impact': 'Spread Impact',
    'imu_ax_spread_regular': 'Spread Regular', 'imu_ay_spread_regular': 'Spread Regular', 'imu_az_spread_regular': 'Spread Regular',
}

# Group colors for the heatmap sidebar
GROUP_COLORS = {
    'Peak Accel': '#1F77B4',
    'Peak Gyro': '#FF7F0E',
    'Accel Energy': '#2CA02C',
    'Gyro Energy': '#D62728',
    'Vibration Accel': '#9467BD',
    'Vibration Gyro': '#8C564B',
    'Settling': '#E377C2',
    'Spread Impact': '#7F7F7F',
    'Spread Regular': '#BCBD22',
}

# Consistent condition colors (shared with rf_angle_prediction module)
CONDITION_COLORS = {'Rotating Cage': '#1F77B4', 'Fixed Cage': '#D62728'}


# ══════════════════════════════════════════════════════════════════════════════
#  Huber Robust Regressor (reused from experiments_analysis_summary.ipynb)
# ══════════════════════════════════════════════════════════════════════════════

def huber_regressor(x, y, delta=1.345, max_iter=100, tol=1e-5):
    """
    Robust M-estimator using the Huber loss function.

    Parameters
    ----------
    x : array-like        Predictor values.
    y : array-like        Response values.
    delta : float         Huber threshold (default 1.345 = 95% efficiency under Gaussian).
    max_iter : int        Maximum IRLS iterations.
    tol : float           Convergence tolerance on coefficient norm.

    Returns
    -------
    slope, intercept : float, float
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    # Design matrix: [x, 1] for slope + intercept
    X = np.vstack([x, np.ones_like(x)]).T
    # ── Step 1: Initial OLS fit via least-squares normal equations ───────────
    # Ordinary Least Squares gives starting coefficients; these will be refined
    # iteratively by down-weighting outliers in subsequent IRLS steps.
    beta = np.linalg.lstsq(X, y, rcond=None)[0]
    # ── IRLS (Iteratively Reweighted Least Squares) loop ─────────────────────
    for _ in range(max_iter):
        predictions = X @ beta
        residuals = y - predictions
        # ── Step 2: Robust scale estimate via MAD ────────────────────────────
        # MAD (Median Absolute Deviation) is a robust alternative to std.
        # mad / 0.6745 calibrates MAD to be consistent with σ under normality
        # (i.e., for Gaussian data, MAD/0.6745 ≈ σ).
        mad = np.median(np.abs(residuals - np.median(residuals)))
        scale = mad / 0.6745 if mad > 1e-5 else 1.0
        # ── Step 3: Standardized residuals ───────────────────────────────────
        u = residuals / scale
        # ── Step 4: Huber weight function ────────────────────────────────────
        # w(u) = 1.0           if |u| ≤ delta   (inlier: full weight)
        #        delta / |u|   if |u| > delta   (outlier: down-weighted)
        # This gives OLS-like efficiency on inliers while bounding the influence
        # of outliers (delta=1.345 → 95% asymptotic efficiency under normality).
        w = np.where(np.abs(u) <= delta, 1.0, delta / np.abs(u))
        # ── Step 5: Weighted Least Squares via normal equations ──────────────
        # Solve (Xᵀ W X) β = Xᵀ W y, where W = diag(w).
        # The element-wise multiplication w[:, None] * X applies weights to each
        # row of the design matrix; beta_new minimizes Σ w_i (y_i − x_iᵀβ)².
        beta_new = np.linalg.solve(X.T @ (w[:, None] * X), X.T @ (w * y))
        # ── Step 6: Convergence check on coefficient norm ────────────────────
        # If the change in β between iterations is below tol, the IRLS has
        # converged to a stable M-estimate. Break early to save computation.
        if np.linalg.norm(beta_new - beta) < tol:
            beta = beta_new
            break
        beta = beta_new
    return float(beta[0]), float(beta[1])


# ══════════════════════════════════════════════════════════════════════════════
#  1.  Data Extraction
# ══════════════════════════════════════════════════════════════════════════════

def load_impact_data(condition='Fixed Cage'):
    """
    Load impact flights with verified impacts and all IMU columns.

    Parameters
    ----------
    condition : str
        Cage condition to filter by — 'Fixed Cage' (default) or 'Rotating Cage'.

    Returns
    -------
    pd.DataFrame
        Columns: impact_angle, battery_at_start, and all IMU_COLS.
        No NaN values in any of these columns.
    """
    print("📡 Loading flight data from SQLite database...")
    # Pull the full flights_summary table from the experiments_summary.db SQLite
    # database via the shared db_manager helper (avoids raw SQL in analysis code).
    df_all = get_database_df()

    # Filter: only collisions with detected impact for the given condition.
    # impact_detected == 1 ensures we only analyze flights where the sensing
    # pipeline confirmed an actual obstacle collision (not a missed approach).
    df = df_all.query(f"impact_detected == 1 and condition == '{condition}'").copy()
    print(f"   → {len(df)} {condition} impact flights found (out of {len(df_all)} total).")

    # Keep only relevant columns: the target variable (impact_angle), a covariate
    # (battery_at_start for coloring), and all IMU feature columns.
    keep_cols = ['impact_angle', 'battery_at_start'] + IMU_COLS
    # dropna() removes any rows where at least one IMU feature is missing,
    # ensuring every flight in the analysis has a complete sensor footprint.
    df = df[keep_cols].dropna()
    print(f"   → {len(df)} flights with complete IMU data after dropping NaN rows.")

    # Quick summary stats to verify data quality and range coverage
    print(f"   → impact_angle range: [{df['impact_angle'].min():.1f}°, {df['impact_angle'].max():.1f}°]")
    print(f"   → battery_at_start range: [{df['battery_at_start'].min():.1f}%, {df['battery_at_start'].max():.1f}%]")

    return df


# ══════════════════════════════════════════════════════════════════════════════
#  2.  Correlation Heatmap
# ══════════════════════════════════════════════════════════════════════════════

def plot_correlation_heatmap(df, condition='Fixed Cage', save_path=None, show=True,
                              figsize_width=3.85, cbar_fraction=0.06):
    """
    Single-panel heatmap showing Pearson correlation of each IMU feature
    with impact_angle. Rows are sorted by descending |r| so the strongest
    signals appear at the top.

    Features are also grouped by type (Peak Accel, Peak Gyro, ...) via
    a colored sidebar for quick visual scanning.

    Parameters
    ----------
    df : pd.DataFrame
        Data with IMU columns and impact_angle.
    condition : str
        Cage condition label for plot titles — 'Fixed Cage' (default) or 'Rotating Cage'.
    save_path : str or None
        If given, saves the figure to disk at this path.
    show : bool
        If True (default), calls plt.show() for inline notebook display.
    figsize_width : float
        Figure width in inches (default 3.85 — override from notebook).
    cbar_fraction : float
        Colorbar width fraction (default 0.06 — override from notebook).
    """
    # Compute Pearson r and p-value for every IMU column vs impact_angle.
    # Pearson r measures the linear correlation between two variables:
    #   r = Σ(x_i − x̄)(y_i − ȳ) / sqrt(Σ(x_i − x̄)² Σ(y_i − ȳ)²)
    # r ranges from −1 (perfect negative linear) to +1 (perfect positive linear),
    # with 0 meaning no linear relationship.
    records = []
    for col in IMU_COLS:
        r, p = pearsonr(df[col], df['impact_angle'])
        records.append({'feature': col, 'r': r, 'p': p, 'abs_r': abs(r),
                        'group': FEATURE_GROUPS[col],
                        'label': DISPLAY_NAMES[col]})

    corr_df = pd.DataFrame(records).sort_values('abs_r', ascending=True).reset_index(drop=True)

    # ── Build figure ────────────────────────────────────────────────────────
    n = len(corr_df)
    fig_height = max(6, n * 0.40)
    fig, ax = plt.subplots(figsize=(figsize_width, fig_height), dpi=150)

    # Color each cell by its r value using the coolwarm diverging colormap.
    # coolwarm maps −1 → cool blue (negative correlation), +1 → warm red
    # (positive correlation), and 0 → white (no correlation). The Normalize
    # object with vmin=−1, vmax=1 ensures the full colormap range is used.
    cmap = plt.cm.coolwarm
    norm = plt.Normalize(-1, 1)
    values = corr_df['r'].values.reshape(-1, 1)
    im = ax.imshow(values, cmap=cmap, vmin=-1, vmax=1, aspect='auto')

    # ── Y-axis: feature names ───────────────────────────────────────────────
    ax.set_yticks(range(n))
    ax.set_yticklabels(corr_df['label'].values, fontsize=9)

    # ── X-axis: single column, no tick labels ───────────────────────────────
    ax.set_xticks([])
    ax.set_xlim(-0.5, 1.5)

    # ── Group color sidebar on the left edge ────────────────────────────────
    # Each feature is painted with a colored strip indicating its feature
    # group (e.g., Peak Accel, Vibration Gyro). This makes it easy to visually
    # scan which categories of IMU measurements correlate most strongly.
    for i, grp in enumerate(corr_df['group']):
        ax.fill_between([-0.5, -0.34], i - 0.5, i + 0.5,
                        color=GROUP_COLORS.get(grp, '#CCCCCC'), alpha=0.7,
                        edgecolor='none', transform=ax.get_transform())

    # ── Annotate each cell with r value and significance stars ──────────────
    # Significance stars follow the standard convention:
    #   ***  p < 0.001  (highly significant)
    #   **   p < 0.01   (very significant)
    #   *    p < 0.05   (significant)
    # No star means the correlation is not statistically distinguishable from
    # zero at the 95% confidence level.
    for i, (_, row) in enumerate(corr_df.iterrows()):
        sig = ''
        if row['p'] < 0.001:
            sig = '***'
        elif row['p'] < 0.01:
            sig = '**'
        elif row['p'] < 0.05:
            sig = '*'
        label = f"r = {row['r']:.3f}{sig}"
        ax.text(0, i, label, ha='center', va='center', fontsize=8.5,
                color='white' if abs(row['r']) > 0.6 else 'black',
                fontweight='bold')

    # ── Colorbar ────────────────────────────────────────────────────────────
    cbar = fig.colorbar(im, ax=ax, fraction=cbar_fraction, pad=0.02)
    cbar.set_label('Pearson r (with Impact Angle)', fontweight='bold', fontsize=10)
    cbar.ax.tick_params(labelsize=9)

    # ── Title & labels ──────────────────────────────────────────────────────
    ax.set_title(f'<{condition}> IMU Feature Correlation with Impact Angle',
                 fontweight='bold', fontsize=13, pad=10)
    ax.set_xlabel('← Negative correlation    |    Positive correlation →',
                  fontsize=9, labelpad=5, fontstyle='italic')

    # ── Remove spines for clean thesis look ───────────────────────────
    for spine in ['top', 'right', 'left', 'bottom']:
        ax.spines[spine].set_visible(False)

    # ── Significance legend (inside axes, bottom-right of plot area) ──
    ax.text(0.98, 0.02, '* p < 0.05   ** p < 0.01   *** p < 0.001',
            transform=ax.transAxes, ha='right', va='bottom',
            fontsize=7.5, fontstyle='italic', color='#555555')

    # ── Data origin (per skill doc §4) ─────────────────────────────────
    ax.text(0.98, 0.01, f"Source: flights_summary ({condition}, impact_detected=1, N={len(df)})",
            transform=fig.transFigure, ha='right', va='bottom',
            fontsize=7.5, fontweight='bold',
            bbox=dict(boxstyle='round,pad=0.15', facecolor='white', alpha=0.8, edgecolor='none'))

    fig.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"   💾 Saved correlation heatmap → {os.path.relpath(save_path, THIS_DIR)}")

    if show:
        plt.show()
    plt.close(fig)
    return corr_df


# ══════════════════════════════════════════════════════════════════════════════
#  3.  Top-3 Scatter Plots (with Robust Trendline)
# ══════════════════════════════════════════════════════════════════════════════

def plot_top3_scatter(df, condition='Fixed Cage', save_path=None, show=True):
    """
    3-row figure: one row per top-IMU feature (by |Pearson r| with
    impact_angle).  Each row shows:
      - Scatter points colored by battery_at_start
      - Robust Huber regression trendline
      - Annotated r² and p-value
      - Axis labels in thesis style

    Parameters
    ----------
    df : pd.DataFrame
        Data with IMU columns and impact_angle.
    condition : str
        Cage condition label for plot titles — 'Fixed Cage' (default) or 'Rotating Cage'.
    save_path : str or None
        If given, saves the figure to disk at this path.
    show : bool
        If True (default), calls plt.show() for inline notebook display.
    """
    # Find top 3 features by |r| — absolute Pearson correlation with impact_angle.
    # Sorting by absolute value ensures we capture both strong negative and strong
    # positive linear relationships; these are the most promising univariate
    # predictors of impact angle.
    r_vals = {col: pearsonr(df[col], df['impact_angle'])[0] for col in IMU_COLS}
    top3 = sorted(r_vals, key=lambda c: abs(r_vals[c]), reverse=True)[:3]

    print(f"   → Top 3 features by |r|: {[DISPLAY_NAMES[c] for c in top3]}")
    for c in top3:
        r, p = pearsonr(df[c], df['impact_angle'])
        print(f"       {DISPLAY_NAMES[c]:30s}  r = {r:+.4f}   p = {p:.2e}")

    # ── Build 3-row figure ──────────────────────────────────────────────────
    fig, axes = plt.subplots(3, 1, figsize=(8, 12), dpi=150, sharex=True)

    # Colormap for battery state: viridis is a perceptually uniform sequential
    # colormap that maps low battery (dark purple) → high battery (bright yellow).
    # This lets the reader see at a glance whether battery level systematically
    # shifts the feature-vs-angle relationship.
    cmap = plt.cm.viridis
    norm = plt.Normalize(df['battery_at_start'].min(), df['battery_at_start'].max())

    for idx, (col, ax) in enumerate(zip(top3, axes)):
        x = df['impact_angle'].values
        y = df[col].values

        # ── Scatter: color by battery ───────────────────────────────────────
        sc = ax.scatter(x, y, c=df['battery_at_start'].values, cmap=cmap,
                        norm=norm, s=65, alpha=0.75, edgecolor='w',
                        linewidth=0.5, zorder=3)

        # ── Robust Huber trendline ──────────────────────────────────────────
        # The Huber M-estimator fits a line that is not pulled off-course by
        # outlier flights. Overlaying it on the scatter plot reveals the central
        # trend without letting a few extreme points dominate the slope.
        if len(x) > 2:
            slope, intercept = huber_regressor(x, y)
            x_smooth = np.linspace(x.min(), x.max(), 200)
            y_smooth = slope * x_smooth + intercept
            ax.plot(x_smooth, y_smooth, color='#444444', linestyle='--',
                    linewidth=1.8, alpha=0.8, zorder=4,
                    label=f'Huber trend (slope = {slope:.3f})')

            # Compute pseudo-R²: 1 - (sum of squared residuals / total sum of squares).
            # This is the coefficient of determination — it measures the fraction
            # of variance in the IMU feature explained by the linear trendline.
            # R² = 1 means perfect fit; R² = 0 means the line explains nothing
            # beyond the feature's own mean. Unlike the classic least-squares R²,
            # the residuals here come from the Huber (robust) fit, so the metric
            # is not inflated by outliers.
            residuals = y - (slope * x + intercept)
            ss_res = np.sum(residuals ** 2)
            ss_tot = np.sum((y - np.mean(y)) ** 2)
            r_squared = 1 - ss_res / ss_tot if ss_tot > 0 else 0.0

            # Pearson r and p for annotation: the standard linear correlation
            # coefficient and its two-tailed p-value.
            r_val, p_val = pearsonr(x, y)

            # Annotation text
            anno = f"$r = {r_val:.3f}$,  $r^2 = {r_squared:.3f}$"
            if p_val < 0.001:
                anno += '\n$p < 0.001$ ***'
            elif p_val < 0.01:
                anno += '\n$p < 0.01$ **'
            elif p_val < 0.05:
                anno += '\n$p < 0.05$ *'
            else:
                anno += f'\n$p = {p_val:.3f}$'

            ax.text(0.97, 0.95, anno, transform=ax.transAxes,
                    ha='right', va='top', fontsize=9,
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                              alpha=0.85, edgecolor='#CCCCCC'))

        # ── Styling ─────────────────────────────────────────────────────────
        ax.set_ylabel(f"{DISPLAY_NAMES[col]}", fontweight='bold', fontsize=11)
        if idx == 2:
            ax.set_xlabel('Impact Angle (°)', fontweight='bold', fontsize=12)
        ax.grid(True, linestyle=':', alpha=0.4, zorder=0)
        ax.legend(loc='lower right', fontsize=8.5, framealpha=0.9)

        # Tick marks
        ax.xaxis.set_major_locator(ticker.MultipleLocator(10))
        ax.yaxis.set_major_locator(ticker.MaxNLocator(6))

    # ── Shared colorbar ────────────────────────────────────────────────────
    cbar = fig.colorbar(sc, ax=axes, fraction=0.015, pad=0.02,
                        orientation='vertical', aspect=40)
    cbar.set_label('Battery at Start (%)', fontweight='bold', fontsize=10)
    cbar.ax.tick_params(labelsize=9)

    # ── Super title ─────────────────────────────────────────────────────────
    fig.suptitle(f'<{condition}> Top IMU Features vs. Impact Angle\n'
                 '(Points colored by battery state at start of flight)',
                 fontweight='bold', fontsize=13, y=0.98)

    # ── Data origin (per skill doc §4) ─────────────────────────────────
    fig.text(0.98, 0.02, f"Source: flights_summary ({condition}, impact_detected=1, N={len(df)})",
             ha='right', va='bottom', fontsize=8, fontweight='bold',
             bbox=dict(boxstyle='round,pad=0.15', facecolor='white', alpha=0.8, edgecolor='none'))

    fig.tight_layout(rect=[0, 0, 0.92, 0.94])

    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"   💾 Saved top-3 scatter → {os.path.relpath(save_path, THIS_DIR)}")

    if show:
        plt.show()
    plt.close(fig)
    return top3


# ══════════════════════════════════════════════════════════════════════════════
#  4.  (Bonus) Parallel Coordinates
# ══════════════════════════════════════════════════════════════════════════════

def plot_parallel_coordinates(df, condition='Fixed Cage', save_path=None, show=True):
    """
    Parallel coordinates plot: normalized IMU features connected by lines
    colored by impact-angle bin.  Quick visual check for clustering/separation
    of angle groups in multi-dimensional IMU space.

    Parameters
    ----------
    df : pd.DataFrame
        Data with IMU columns and impact_angle.
    condition : str
        Cage condition label for plot titles — 'Fixed Cage' (default) or 'Rotating Cage'.
    save_path : str or None
        If given, saves the figure to disk at this path.
    show : bool
        If True (default), calls plt.show() for inline notebook display.
    """
    from matplotlib.patches import Patch

    # Bin impact_angle into 4 groups for color-coding. Each bin represents
    # a qualitatively different impact regime: shallow grazing (<40°),
    # moderate oblique (40–60°), steep (60–75°), and near-perpendicular (75°+).
    bins = [0, 40, 60, 75, 100]
    labels = ['<40°', '40–60°', '60–75°', '75°+']
    df_plot = df.copy()
    df_plot['angle_bin'] = pd.cut(df_plot['impact_angle'], bins=bins,
                                   labels=labels, right=False)

    # Select top 10 features by |r| (to avoid visual clutter)
    r_vals = {col: abs(pearsonr(df[col], df['impact_angle'])[0]) for col in IMU_COLS}
    top_cols = sorted(r_vals, key=r_vals.get, reverse=True)[:10]
    plot_cols = top_cols + ['angle_bin']

    # Normalize features to [0, 1] via min-max scaling.
    # Min-max normalization: x_norm = (x − x_min) / (x_max − x_min).
    # This brings every feature onto the same 0-to-1 scale so they can be
    # plotted on shared parallel vertical axes. Without normalization, features
    # with large raw ranges (e.g., peak acceleration ~100 m/s²) would visually
    # dwarf those with small ranges (e.g., settling time ~0.1 s).
    # If a feature is constant (c_max == c_min), we map it to 0.5 (midline).
    df_norm = df_plot[plot_cols].copy()
    for col in top_cols:
        c_min, c_max = df_norm[col].min(), df_norm[col].max()
        if c_max > c_min:
            df_norm[col] = (df_norm[col] - c_min) / (c_max - c_min)
        else:
            df_norm[col] = 0.5

    # ── Plot ────────────────────────────────────────────────────────────────
    bin_colors = {'<40°': '#1F77B4', '40–60°': '#FF7F0E',
                  '60–75°': '#2CA02C', '75°+': '#D62728'}

    fig, ax = plt.subplots(figsize=(12, 6), dpi=150)

    x_pos = np.arange(len(top_cols))
    # Draw one polyline per flight. alpha=0.3 makes each line semi-transparent,
    # so overlapping lines accumulate visually — dense regions appear darker,
    # giving the reader a sense of where flights cluster in multi-feature space.
    # This is a classic parallel-coordinates trick for density perception.
    for _, row in df_norm.iterrows():
        color = bin_colors.get(row['angle_bin'], '#999999')
        ax.plot(x_pos, row[top_cols].values, color=color, alpha=0.3, linewidth=0.8)

    ax.set_xticks(x_pos)
    ax.set_xticklabels([DISPLAY_NAMES[c] for c in top_cols], rotation=30, ha='right', fontsize=9)
    ax.set_ylabel('Normalized value', fontweight='bold')
    ax.set_ylim(-0.05, 1.05)
    ax.set_title(f'<{condition}> Parallel Coordinates — IMU Features by Impact-Angle Group',
                 fontweight='bold', fontsize=13, pad=10)
    ax.grid(True, linestyle=':', alpha=0.3, axis='y')

    # Legend
    legend_elements = [Patch(facecolor=color, alpha=0.6, label=label)
                       for label, color in bin_colors.items()]
    ax.legend(handles=legend_elements, title='Impact Angle', fontsize=9,
              title_fontsize=10, loc='upper right')

    # ── Data origin (per skill doc §4) ─────────────────────────────────
    fig.text(0.98, 0.02, f"Source: flights_summary ({condition}, impact_detected=1, N={len(df)})",
             ha='right', va='bottom', fontsize=8, fontweight='bold',
             bbox=dict(boxstyle='round,pad=0.15', facecolor='white', alpha=0.8, edgecolor='none'))

    fig.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"   💾 Saved parallel coordinates → {os.path.relpath(save_path, THIS_DIR)}")

    if show:
        plt.show()
    plt.close(fig)


# ══════════════════════════════════════════════════════════════════════════════
#  5.  Consolidated Parallel Coordinates (Fixed + Rotating, 2×1)
# ══════════════════════════════════════════════════════════════════════════════

def plot_consolidated_parallel_coordinates(df_fix, df_rot, save_path=None, show=True,
                                           n_features=10):
    """
    Consolidated parallel coordinates — 2 rows × 1 column.

    Top panel:    Rotating Cage
    Bottom panel: Fixed Cage

    Features are ordered left-to-right by descending |Pearson r| as computed
    on the **Rotating Cage** data only — Rotating Cage defines the importance
    ordering.  Each feature's x-tick is a dual-rank label:

      • Bottom ticks (Fixed Cage panel):  Rotating Cage rank in blue
      • Top ticks (Rotating Cage panel):  Fixed Cage rank in red

    This lets the reader cross-reference: "Feature X is Rotating's #1 but only
    Fixed's #7 — the gap tells the story."

    Parameters
    ----------
    df_fix : pd.DataFrame
        Fixed Cage flights with IMU columns and impact_angle.
    df_rot : pd.DataFrame
        Rotating Cage flights with IMU columns and impact_angle.
    save_path : str or None
        If given, saves the figure to disk at this path.
    show : bool
        If True (default), calls plt.show() for inline notebook display.
    n_features : int
        Number of top features to display (default 10).
    """
    from matplotlib.patches import Patch

    # ── Compute per-cage ranks based on their OWN |Pearson r| ───────────────
    def _cage_ranked_cols(df):
        records = []
        for col in IMU_COLS:
            r, _ = pearsonr(df[col].values, df['impact_angle'].values)
            records.append({'feature': col, 'abs_r': abs(r)})
        r_df = pd.DataFrame(records).sort_values('abs_r', ascending=False)
        r_df['rank'] = range(1, len(r_df) + 1)
        return r_df.set_index('feature')['rank'].to_dict()

    rot_ranks = _cage_ranked_cols(df_rot)
    fix_ranks = _cage_ranked_cols(df_fix)

    # ── Feature ordering by Rotating Cage's |r| (descending) ───────────────
    rot_r_vals = {}
    for col in IMU_COLS:
        rot_r_vals[col] = abs(pearsonr(df_rot[col].values,
                                        df_rot['impact_angle'].values)[0])
    top_cols = sorted(rot_r_vals, key=rot_r_vals.get, reverse=True)[:n_features]

    # ── Rank-annotated tick labels ──────────────────────────────────────────
    # Bottom ticks (Fixed panel): "R#1 Accel Spread (Impact) X" in blue
    rot_tick_labels = [
        f"R#{rot_ranks[c]} {DISPLAY_NAMES[c]}" for c in top_cols
    ]
    # Top ticks (Rotating panel):  "F#7 Accel Spread (Impact) X" in red
    fix_tick_labels = [
        f"F#{fix_ranks[c]} {DISPLAY_NAMES[c]}" for c in top_cols
    ]

    # ── Helper to prepare normalized dataframe ──────────────────────────────
    def _prepare_parallel_df(df_source, feature_cols):
        """Bin angle, normalize features to [0,1]."""
        bins = [0, 40, 60, 75, 100]
        labels = ['<40°', '40–60°', '60–75°', '75°+']
        df_p = df_source.copy()
        df_p['angle_bin'] = pd.cut(df_p['impact_angle'], bins=bins,
                                    labels=labels, right=False)
        plot_data = df_p[feature_cols + ['angle_bin']].copy()
        for col in feature_cols:
            c_min, c_max = plot_data[col].min(), plot_data[col].max()
            if c_max > c_min:
                plot_data[col] = (plot_data[col] - c_min) / (c_max - c_min)
            else:
                plot_data[col] = 0.5
        return plot_data

    # ── Prepare both datasets ───────────────────────────────────────────────
    fix_plot = _prepare_parallel_df(df_fix, top_cols)
    rot_plot = _prepare_parallel_df(df_rot, top_cols)

    # Plot colors for angle bins
    bin_colors = {'<40°': '#1F77B4', '40–60°': '#FF7F0E',
                  '60–75°': '#2CA02C', '75°+': '#D62728'}

    # ── Build 2×1 figure (NO sharex — each panel needs its own tick labels) ──
    fig, axes = plt.subplots(2, 1, figsize=(7, 10), dpi=150)

    conditions = [
        ('Rotating Cage', rot_plot, len(df_rot)),
        ('Fixed Cage', fix_plot, len(df_fix)),
    ]

    for idx, (cond_name, plot_df, n) in enumerate(conditions):
        ax = axes[idx]
        x_pos = np.arange(len(top_cols))

        # ── Draw lines ──────────────────────────────────────────────────────
        for _, row in plot_df.iterrows():
            color = bin_colors.get(row['angle_bin'], '#999999')
            ax.plot(x_pos, row[top_cols].values, color=color,
                    alpha=0.3, linewidth=0.8)

        ax.set_xticks(x_pos)

        if idx == 0:  # ── Rotating Cage (top panel) — Fixed ranks on TOP ──
            ax.tick_params(top=True, labeltop=True, bottom=False, labelbottom=False)
            ax.set_xticklabels(fix_tick_labels, rotation=55, ha='left', fontsize=9)
            for lbl in ax.get_xticklabels():
                lbl.set_color(CONDITION_COLORS['Fixed Cage'])  # red
        else:          # ── Fixed Cage (bottom panel) — Rotating ranks on BOTTOM ──
            ax.tick_params(top=False, labeltop=False, bottom=True, labelbottom=True)
            ax.set_xticklabels(rot_tick_labels, rotation=55, ha='right', fontsize=9)
            for lbl in ax.get_xticklabels():
                lbl.set_color(CONDITION_COLORS['Rotating Cage'])  # blue

        # Remove X-axis margins so lines extend edge-to-edge
        ax.margins(x=0)
        ax.set_xlim(-0.05, len(top_cols) - 0.95)

        ax.set_ylabel('Normalized value', fontweight='bold')
        ax.set_ylim(-0.05, 1.05)
        ax.set_title(f'{cond_name} — Parallel Coordinates (N={n})',
                     fontweight='bold', fontsize=13, color='black')
        ax.grid(True, linestyle=':', alpha=0.3, axis='y')

    # ── Manually sync x-limits across panels ───────────────────────────────
    xlim = axes[0].get_xlim()
    axes[1].set_xlim(xlim)

    # ── Single legend for the figure ────────────────────────────────────────
    legend_elements = [Patch(facecolor=color, alpha=0.6, label=label)
                       for label, color in bin_colors.items()]
    axes[0].legend(handles=legend_elements, title='Impact Angle',
                   fontsize=9, title_fontsize=10, loc='upper right')

    # ── Global title ────────────────────────────────────────────────────────
    fig.suptitle('Parallel Coordinates — IMU Features by Impact-Angle Group\n',
                 fontweight='bold', fontsize=14, y=1.03)

    # ── Data source ─────────────────────────────────────────────────────────
    fig.text(0.02, -0.02, f"Source: flights_summary (N={len(df_rot)})",
             ha='left', va='bottom',
             fontsize=7, fontstyle='italic', color='#555555')
    fig.text(0.98, -0.02, f"Source: flights_summary (N={len(df_fix)})",
             ha='right', va='bottom',
             fontsize=7, fontstyle='italic', color='#555555')

    fig.subplots_adjust(top=0.78, hspace=0.25, bottom=0.15, left=0.06, right=0.98)

    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"   💾 Saved consolidated parallel coordinates → "
              f"{os.path.relpath(save_path, THIS_DIR)}")

    if show:
        plt.show()
    plt.close(fig)


# ══════════════════════════════════════════════════════════════════════════════
#  6.  Consolidated IMU Feature Correlation with ConnectionPatch
# ══════════════════════════════════════════════════════════════════════════════

def plot_consolidated_feature_correlation(df_fix, df_rot, save_path=None, show=True):
    """
    Mirrored slopegraph + heatmap layout showing IMU feature correlation ranks.

    Layout (5-column GridSpec):
        Rotating Keys  |  Rotating Heatmap  |  Splines  |  Fixed Heatmap  |  Fixed Keys

    Each heatmap is a 1-column array sorted by its own |Pearson r| descending.
    ConnectionPatch splines in the center axis trace how each feature's rank
    changes between the two cage configurations.

    Parameters
    ----------
    df_fix, df_rot : pd.DataFrame
        Fixed and Rotating Cage data with IMU columns and impact_angle.
    save_path : str or None
        If given, saves the figure to disk.
    show : bool
        If True, calls plt.show().
    """
    from matplotlib.patches import ConnectionPatch
    from matplotlib.gridspec import GridSpec

    # ── Compute Pearson r for each feature per cage ────────────────────────
    def _compute_corr(df):
        records = []
        for col in IMU_COLS:
            r, p = pearsonr(df[col], df['impact_angle'])
            records.append({
                'feature': col, 'r': r, 'p': p, 'abs_r': abs(r),
                'group': FEATURE_GROUPS[col], 'label': DISPLAY_NAMES[col],
            })
        return pd.DataFrame(records)

    fix_corr = _compute_corr(df_fix)
    rot_corr = _compute_corr(df_rot)

    # Sort each independently by |r| descending — each heatmap has its own
    # vertical ordering so Row 1 = highest |r| for that cage.
    fix_corr = fix_corr.sort_values('abs_r', ascending=False).reset_index(drop=True)
    rot_corr = rot_corr.sort_values('abs_r', ascending=False).reset_index(drop=True)

    n_features = len(IMU_COLS)

    # ── Build y-position + |r| lookups for splines ─────────────────────────
    # y = n_features - 1 - i  → rank 1 at top (y = n_features-1), rank N at bottom (y = 0)
    rot_y = {}    # feature → y-coordinate in left heatmap
    fix_y = {}    # feature → y-coordinate in right heatmap
    rot_abs_r = {}  # feature → |r|
    fix_abs_r = {}  # feature → |r|
    for i, (_, row) in enumerate(rot_corr.iterrows()):
        rot_y[row['feature']] = n_features - 1 - i
        rot_abs_r[row['feature']] = row['abs_r']
    for i, (_, row) in enumerate(fix_corr.iterrows()):
        fix_y[row['feature']] = n_features - 1 - i
        fix_abs_r[row['feature']] = row['abs_r']

    # ── Build figure: 5-column GridSpec ────────────────────────────────────
    # Col 0: Rotating Cage keys    (rank + feature name, right-aligned flush to heatmap)
    # Col 1: Rotating Cage heatmap (1-column, r values)
    # Col 2: Center spline axis    (empty, ConnectionPatch arcs)
    # Col 3: Fixed Cage heatmap    (1-column, r values)
    # Col 4: Fixed Cage keys       (feature name, left-aligned pushing outward)
    fig = plt.figure(figsize=(10.0, max(8, n_features * 0.34)), dpi=150)
    gs = GridSpec(1, 5, figure=fig,
                  width_ratios=[0.9, 0.7, 0.4, 0.7, 0.9],
                  wspace=0.015)
    ax_rot_keys = fig.add_subplot(gs[0, 0])
    ax_rot_heat = fig.add_subplot(gs[0, 1])
    ax_splines  = fig.add_subplot(gs[0, 2])
    ax_fix_heat = fig.add_subplot(gs[0, 3])
    ax_fix_keys = fig.add_subplot(gs[0, 4])

    # ── Shared y-limits across all 5 axes ──────────────────────────────────
    y_bottom, y_top = -0.5, n_features - 0.5
    for ax in [ax_rot_keys, ax_rot_heat, ax_splines, ax_fix_heat, ax_fix_keys]:
        ax.set_ylim(y_bottom, y_top)

    # Diverging colormap: coolwarm — -1.0 → deep blue, +1.0 → deep red,
    # white at zero.  The array is reversed (see above) so strongest |r|
    # cells appear at the top, colored by their signed r value.
    cmap = plt.cm.coolwarm
    norm = plt.Normalize(vmin=-1.0, vmax=1.0)

    # ── Figure title ───────────────────────────────────────────────────────
    fig.suptitle('IMU Feature Correlation with Impact Angle',
                 fontweight='bold', fontsize=13, y=0.99)

    # ═══════════════════════════════════════════════════════════════════════
    #  Col 0: Rotating Cage Keys  (rank + feature name, right-aligned
    #          so text is flush against the heatmap's left edge)
    # ═══════════════════════════════════════════════════════════════════════
    ax_rot_keys.axis('off')

    for i, (_, row) in enumerate(rot_corr.iterrows()):
        rank = i + 1
        y_pos = n_features - 1 - i
        # Right-aligned at x=0.98 so the rank+name is tight against the heatmap
        ax_rot_keys.text(0.98, y_pos, f"{rank}. {row['label']}",
                         fontsize=7.0, va='center', ha='right', color='#333333')

    # ═══════════════════════════════════════════════════════════════════════
    #  Col 1: Rotating Cage 1-column Heatmap
    # ═══════════════════════════════════════════════════════════════════════
    # Reverse so row 0 = weakest |r| (bottom) → row N-1 = strongest (top).
    # imshow origin='upper' maps row 0 → data y=-0.5 (bottom of shared ylim),
    # matching the text which places i=0 (best) at y=n_features-1 (top).
    rot_vals = rot_corr['r'].values[::-1].reshape(-1, 1)
    im_rot = ax_rot_heat.imshow(rot_vals, cmap=cmap, norm=norm, aspect='auto',
                                origin='upper')
    ax_rot_heat.set_xticks([])
    ax_rot_heat.set_yticks([])
    for spine in ['top', 'right', 'left', 'bottom']:
        ax_rot_heat.spines[spine].set_visible(False)

    # Sub-title centred directly above the heatmap column
    ax_rot_heat.text(0.5, y_top + 1.0, 'Rotating Cage',
                     fontweight='bold', fontsize=11, va='bottom', ha='center',
                     color='black', transform=ax_rot_heat.get_yaxis_transform())

    for i, (_, row) in enumerate(rot_corr.iterrows()):
        y_pos = n_features - 1 - i
        val = row['r']
        sig = ''
        if row['p'] < 0.001:
            sig = '***'
        elif row['p'] < 0.01:
            sig = '**'
        elif row['p'] < 0.05:
            sig = '*'
        ax_rot_heat.text(0, y_pos, f"{val:+.3f}{sig}",
                         ha='center', va='center', fontsize=7.0,
                         fontweight='bold',
                         color='white' if abs(val) > 0.55 else 'black')

    # ═══════════════════════════════════════════════════════════════════════
    #  Col 2: Center Spline Axis  (invisible canvas for ConnectionPatches)
    # ═══════════════════════════════════════════════════════════════════════
    ax_splines.axis('off')

    # ═══════════════════════════════════════════════════════════════════════
    #  Col 3: Fixed Cage 1-column Heatmap
    # ═══════════════════════════════════════════════════════════════════════
    fix_vals = fix_corr['r'].values[::-1].reshape(-1, 1)
    im_fix = ax_fix_heat.imshow(fix_vals, cmap=cmap, norm=norm, aspect='auto',
                                origin='upper')
    ax_fix_heat.set_xticks([])
    ax_fix_heat.set_yticks([])
    for spine in ['top', 'right', 'left', 'bottom']:
        ax_fix_heat.spines[spine].set_visible(False)

    # Sub-title centred directly above the heatmap column
    ax_fix_heat.text(0.5, y_top + 1.0, 'Fixed Cage',
                     fontweight='bold', fontsize=11, va='bottom', ha='center',
                     color='black', transform=ax_fix_heat.get_yaxis_transform())

    for i, (_, row) in enumerate(fix_corr.iterrows()):
        y_pos = n_features - 1 - i
        val = row['r']
        sig = ''
        if row['p'] < 0.001:
            sig = '***'
        elif row['p'] < 0.01:
            sig = '**'
        elif row['p'] < 0.05:
            sig = '*'
        ax_fix_heat.text(0, y_pos, f"{val:+.3f}{sig}",
                         ha='center', va='center', fontsize=7.0,
                         fontweight='bold',
                         color='white' if abs(val) > 0.55 else 'black')

    # ═══════════════════════════════════════════════════════════════════════
    #  Col 4: Fixed Cage Keys  (feature name, left-aligned so text pushes
    #          outward cleanly from the right edge of the heatmap)
    # ═══════════════════════════════════════════════════════════════════════
    ax_fix_keys.axis('off')

    for i, (_, row) in enumerate(fix_corr.iterrows()):
        rank = i + 1
        y_pos = n_features - 1 - i
        # Left-aligned at the inner edge (x=0.02) so text starts near the
        # heatmap and extends rightward — clean uniform left edge.
        ax_fix_keys.text(0.02, y_pos, f"{rank}. {row['label']}",
                         fontsize=7.0, va='center', ha='left', color='#333333')

    # ═══════════════════════════════════════════════════════════════════════
    #  ConnectionPatch Splines
    #  Thickness & opacity scale with mean |r| of the feature across BOTH
    #  cages — strong correlations draw thick, opaque splines; weak ones are
    #  thin and faint.
    # ═══════════════════════════════════════════════════════════════════════
    transform_a = ax_rot_heat.get_yaxis_transform()
    transform_b = ax_fix_heat.get_yaxis_transform()

    for feat in rot_y:
        if feat not in fix_y:
            continue
        y_r = rot_y[feat]
        y_f = fix_y[feat]

        # FIX 2: spline visual weight driven by |r|, NOT by rank difference
        mean_abs_r = (rot_abs_r[feat] + fix_abs_r[feat]) / 2.0

        # linewidth: scales linearly with |r|
        lw = mean_abs_r * 4.0
        # alpha: ~0.20 (|r|≈0) → ~1.0 (|r|≈1.0)
        alpha = 0.2 + mean_abs_r * 0.8

        # Colour by correlation sign using the same coolwarm colormap as the
        # heatmap cells, so splines are visually consistent with the cell
        # colours they connect: deep blue (-1.0) → white (0) → deep red (+1.0).
        mean_r = (rot_corr.loc[rot_corr['feature'] == feat, 'r'].values[0] +
                   fix_corr.loc[fix_corr['feature'] == feat, 'r'].values[0]) / 2.0
        line_color = cmap(norm(mean_r))

        con = ConnectionPatch(
            xyA=(1.0, y_r), xyB=(0.0, y_f),
            coordsA=transform_a, coordsB=transform_b,
            connectionstyle="arc3,rad=0.25",
            linewidth=lw,
            color=line_color, alpha=alpha, zorder=1,
        )
        fig.add_artist(con)

    # ═══════════════════════════════════════════════════════════════════════
    #  Shared Colorbar — dedicated axes at the far-right canvas edge
    # ═══════════════════════════════════════════════════════════════════════
    cax = fig.add_axes([0.96, 0.12, 0.012, 0.76])
    cbar = fig.colorbar(im_rot, cax=cax)
    cbar.set_label("Pearson R with Impact Angle\n← Negative  |  Positive →",
                   fontweight='bold', fontsize=8.5)
    cbar.ax.tick_params(labelsize=7)


    # ── Final layout adjustments ───────────────────────────────────────────
    fig.subplots_adjust(left=0.02, right=0.94, top=0.88, bottom=0.05)
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight', pad_inches=0.08)
        print(f"   💾 Saved consolidated feature correlation → "
              f"{os.path.relpath(save_path, THIS_DIR)}")
    if show:
        plt.show()
    plt.close(fig)


# ══════════════════════════════════════════════════════════════════════════════
#  7.  Consolidated Top IMU Features vs. Impact Angle (both cages)
# ══════════════════════════════════════════════════════════════════════════════

def plot_consolidated_top_features(df_fix, df_rot, save_path=None, show=True):
    """
    Top-3 IMU features vs Impact Angle — Rotating (left) + Fixed (right).

    Each cage gets its OWN top-3 by |Pearson r|, independently ranked.
    ``imu_ax_spread_impact`` is forced to Row 1 in BOTH columns for direct
    visual comparison.

    Layout: 3×2 scatter grid.  No heatmap/table.  BOTH columns carry Y-axis
    labels with ordinal rank + feature name.

    Parameters
    ----------
    df_fix, df_rot : pd.DataFrame
        Fixed and Rotating Cage data.
    save_path : str or None
        If given, saves the figure to disk.
    show : bool
        If True, calls plt.show().
    """
    from matplotlib.gridspec import GridSpec


    # ── Compute full |r| rankings per condition for Y-label numbering ─────
    def _full_rankings(df):
        records = []
        for col in IMU_COLS:
            r, p = pearsonr(df[col], df['impact_angle'])
            records.append({'feature': col, 'r': r, 'abs_r': abs(r)})
        r_df = pd.DataFrame(records).sort_values('abs_r', ascending=False)
        r_df['rank'] = range(1, len(r_df) + 1)  # 1-based rank (1 = strongest)
        return r_df.set_index('feature')['rank'].to_dict()

    rot_ranks = _full_rankings(df_rot)
    fix_ranks = _full_rankings(df_fix)

    # ── Compute top-3 per cage INDEPENDENTLY ───────────────────────────────
    def _top_features(df, n=3):
        records = []
        for col in IMU_COLS:
            r, p = pearsonr(df[col], df['impact_angle'])
            records.append({'feature': col, 'r': r, 'p': p, 'abs_r': abs(r),
                            'label': DISPLAY_NAMES[col]})
        return pd.DataFrame(records).sort_values('abs_r', ascending=False).head(n)

    rot_top = _top_features(df_rot, n=3).reset_index(drop=True)
    fix_top = _top_features(df_fix, n=3).reset_index(drop=True)

    # ── Force imu_ax_spread_impact to position 0 in each independent list ──
    def _enforce_top3(top_df, df_full, n=3):
        """Return n-feature list with imu_ax_spread_impact guaranteed at row 0.
        If not naturally top-n, compute its actual |r| and replace the weakest."""
        padj = list(top_df.to_dict('records'))  # already sorted |r| descending
        has_spread = any(r['feature'] == 'imu_ax_spread_impact' for r in padj)
        if not has_spread:
            r, p = pearsonr(df_full['imu_ax_spread_impact'],
                            df_full['impact_angle'])
            padj[-1] = {'feature': 'imu_ax_spread_impact', 'r': r, 'p': p,
                        'abs_r': abs(r), 'label': DISPLAY_NAMES['imu_ax_spread_impact']}
            padj.sort(key=lambda r: r['abs_r'], reverse=True)
        spread_idx = next(i for i, r in enumerate(padj)
                          if r['feature'] == 'imu_ax_spread_impact')
        if spread_idx > 0:
            padj.insert(0, padj.pop(spread_idx))
        return padj[:n]

    rot_rows = _enforce_top3(rot_top, df_rot, n=3)
    fix_rows = _enforce_top3(fix_top, df_fix, n=3)
    n_rows = 3

    # ── Per-feature y-limits ───────────────────────────────────────────────
    FEATURE_LIMITS = {
        'imu_ax_spread_impact': ((0, 0.6), 0.1),
        'imu_ay_spread_impact': ((0, 0.6), 0.1),
        'imu_az_spread_impact': ((0, 0.6), 0.1),
        'imu_accel_energy_x':   ((0, 1.0), 0.2),
        'imu_accel_energy_y':   ((0, 1.0), 0.2),
        'imu_accel_energy_z':   ((0, 1.0), 0.2),
        'imu_peak_accel_x':     ((0, 20),  4.0),
        'imu_peak_accel_y':     ((0, 20),  4.0),
        'imu_peak_accel_z':     ((0, 20),  4.0),
        'imu_vib_gy':           ((0, 0.30), 0.06),
        'imu_vib_gx':           ((0, 0.30), 0.06),
        'imu_vib_gz':           ((0, 0.30), 0.06),
        'imu_ax_spread_regular': ((0, 0.6), 0.1),
    }

    # ── Build figure: 3 rows × 2 cols scatter grid ────────────────────────
    fig = plt.figure(figsize=(12, 8.5), dpi=150)
    gs = GridSpec(3, 2, figure=fig,
                  hspace=0.20, wspace=0.18,
                  left=0.055, right=0.95, top=0.935, bottom=0.065)

    # ── Column sub-titles (overlaid above each column in figure space) ─────
    fig.text(0.25, 0.958, 'Rotating Cage', fontweight='bold', fontsize=11,
             ha='center', va='bottom', color='black')
    fig.text(0.75, 0.958, 'Fixed Cage', fontweight='bold', fontsize=11,
             ha='center', va='bottom', color='black')

    # ── Scatter plots ──────────────────────────────────────────────────────
    cmap_scatter = plt.cm.viridis
    norm_batt = plt.Normalize(
        min(df_fix['battery_at_start'].min(), df_rot['battery_at_start'].min()),
        max(df_fix['battery_at_start'].max(), df_rot['battery_at_start'].max()),
    )
    sc = None  # capture last scatter for colorbar

    for row_idx in range(n_rows):
        rot_r = rot_rows[row_idx]
        fix_r = fix_rows[row_idx]

        for col_idx, (df_cond, cond_name, feat_r) in enumerate([
            (df_rot, 'Rotating Cage', rot_r),
            (df_fix, 'Fixed Cage',   fix_r),
        ]):
            ax = fig.add_subplot(gs[row_idx, col_idx])
            feat = feat_r['feature']
            x = df_cond['impact_angle'].values
            y = df_cond[feat].values

            sc = ax.scatter(x, y, c=df_cond['battery_at_start'].values,
                            cmap=cmap_scatter, norm=norm_batt,
                            s=50, alpha=0.7, edgecolor='white', linewidth=0.4,
                            zorder=3)

            # Huber trendline
            if len(x) > 2:
                slope, intercept = huber_regressor(x, y)
                x_smooth = np.linspace(x.min(), x.max(), 200)
                y_smooth = slope * x_smooth + intercept
                ax.plot(x_smooth, y_smooth, color='#444444', linestyle='--',
                        linewidth=1.5, alpha=0.7, zorder=4)

                # r, p annotation
                r_val, p_val = pearsonr(x, y)
                anno = f'r = {r_val:.3f}'
                if p_val < 0.001:
                    anno += '\np < 0.001 ***'
                elif p_val < 0.01:
                    anno += '\np < 0.01 **'
                elif p_val < 0.05:
                    anno += '\np < 0.05 *'
                ax.text(0.97, 0.95, anno, transform=ax.transAxes,
                        ha='right', va='top', fontsize=7.5,
                        bbox=dict(boxstyle='round,pad=0.2', facecolor='white',
                                  alpha=0.8, edgecolor='#CCCCCC'))

            # ── Y-limits ────────────────────────────────────────────────────
            if feat in FEATURE_LIMITS:
                (ylow, yhigh), tick_step = FEATURE_LIMITS[feat]
                ax.set_ylim(ylow, yhigh)
                ax.set_yticks(np.arange(ylow, yhigh + 0.0001, tick_step))
            else:
                ax.yaxis.set_major_locator(ticker.MaxNLocator(5))

            # X-axis
            ax.set_xlim(0, 90)
            ax.set_xticks([0, 15, 30, 45, 60, 75, 90])

            # ── Y-axis label: actual #rank from full IMU correlation table ──
            ranks = rot_ranks if cond_name == 'Rotating Cage' else fix_ranks
            actual_rank = ranks.get(feat, row_idx + 1)
            display_name = DISPLAY_NAMES.get(feat, feat)
            # Prepend "Rank #X" so it's immediately clear the number refers to
            # the feature's position in the full 26-feature IMU correlation table.
            ax.set_ylabel(f"Rank #{actual_rank} — {display_name}",
                          fontweight='bold', fontsize=9)

            # X-axis label on bottom row only
            if row_idx == n_rows - 1:
                ax.set_xlabel('Impact Angle (°)', fontweight='bold', fontsize=10)

            ax.grid(True, linestyle=':', alpha=0.3)

    # ── Shared colorbar (bottom, centered) ──────────────────────────────────
    cbar_ax = fig.add_axes([0.42, 0.015, 0.16, 0.012])
    cbar = fig.colorbar(sc, cax=cbar_ax, orientation='horizontal')
    cbar.set_label('Battery at Start (%)', fontweight='bold', fontsize=8)
    cbar.ax.tick_params(labelsize=7)

    # ── Global title ───────────────────────────────────────────────────────
    fig.suptitle('Top IMU Features vs. Impact Angle\n'
                 '(Huber trendline, points colored by battery state)',
                 fontweight='bold', fontsize=13, y=1.03)

    # ── Source & stat footnotes (per skill §4.1, §4.1B, §4.1C) ────────────
    fig.text(0.00, 0,
             "Note: Trendlines computed via robust Huber regression\n"
             "to mitigate the influence of outliers\n"
             "(y = mx + c).",
             ha='left', va='bottom', fontsize=6.5, fontstyle='italic', color='#555555')
    fig.text(0.98, 0,
             f"flights_summary (N={len(df_rot)} Rotating, N={len(df_fix)} Fixed)",
             ha='right', va='bottom', fontsize=7.0, color='#555555')

    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"   💾 Saved consolidated top features → "
              f"{os.path.relpath(save_path, THIS_DIR)}")
    if show:
        plt.show()
    plt.close(fig)


# ══════════════════════════════════════════════════════════════════════════════
#  Main — run all analysis when called as a script
# ══════════════════════════════════════════════════════════════════════════════

def main():
    print("=" * 70)
    print("🧪 EDA: Impact Angle vs. IMU Footprint")
    print("=" * 70)

    # 1. Load data
    df = load_impact_data()
    if len(df) == 0:
        print("❌ No data to analyze. Exiting.")
        return

    print()

    # 2. Correlation heatmap
    print("📊 Generating correlation heatmap...")
    corr_df = plot_correlation_heatmap(
        df, save_path=os.path.join(GRAPHICS_DIR, "eda_correlation_heatmap.png"),
        show=False,
    )
    print()

    # 3. Top-3 scatter plots
    print("🎯 Generating top-3 scatter plots with robust trendlines...")
    plot_top3_scatter(
        df, save_path=os.path.join(GRAPHICS_DIR, "eda_top3_scatter.png"),
        show=False,
    )
    print()

    # 4. Parallel coordinates (bonus)
    print("🔗 Generating parallel coordinates plot...")
    plot_parallel_coordinates(
        df, save_path=os.path.join(GRAPHICS_DIR, "eda_parallel_coordinates.png"),
        show=False,
    )
    print()

    # 5. Print summary table
    print("=" * 70)
    print("📋 Summary: Pearson r with impact_angle (sorted by |r|)")
    print("=" * 70)
    summary = corr_df.sort_values('abs_r', ascending=False)
    for _, row in summary.iterrows():
        sig = ''
        if row['p'] < 0.001:
            sig = '***'
        elif row['p'] < 0.01:
            sig = '**'
        elif row['p'] < 0.05:
            sig = '*'
        print(f"  {row['label']:30s}  r = {row['r']:+.4f}  p = {row['p']:.4f}{sig}")
    print()
    print(f"✅ All plots saved to: {os.path.relpath(GRAPHICS_DIR, THIS_DIR)}/")


if __name__ == "__main__":
    main()
