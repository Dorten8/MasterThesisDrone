#!/usr/bin/env python3
# ── Headless-safe backend (Agg when no display, notebook overrides) ──────────
import os, sys as _sys, shutil
if not os.environ.get("DISPLAY") and "JPY_SESSION_NAME" not in os.environ:
    import matplotlib as _mpl; _mpl.use("Agg")

"""
rf_angle_prediction.py — RandomForestRegressor for Impact Angle Prediction

Builds on the EDA (eda/eda_angle_prediction.py) which showed strong but non-linear
correlations between IMU features and impact_angle. A conservative Random Forest
with 5-fold CV captures multi-feature interaction fingerprints without overfitting.

Pipeline:
  1. Broad feature selection (|r| or |ρ| > 0.3, redundancy filter at r > 0.85)
  2. 5-fold CV with angle-stratified splits
  3. Grid search over max_depth / min_samples_leaf within each fold
  4. Actual-vs-Predicted plot + MDI feature importance
  5. Cross-condition transfer: Fixed Cage → Rotating Cage
  6. Permutation importance + learning curve + Huber baseline comparison

Outputs (to graphics/):
  - rf_actual_vs_predicted.png      CV predicted vs actual, colored by fold
  - rf_feature_importance.png       MDI bars with group-color sidebar
  - rf_cross_condition.png          Fixed model applied to Rotating Cage
  - rf_learning_curve.png           Training size vs R² score
  - rf_permutation_importance.png   Drop-column importance (slower, more reliable)
  - rf_huber_baseline.png           Linear baseline vs RF comparison

Usage:
  python3 dev_logs/analysis/models/rf_angle_prediction.py

Or import from a notebook:
  from dev_logs.analysis.models.rf_angle_prediction import (
      run_rf_pipeline,
      FEATURE_GROUPS, GROUP_COLORS, DISPLAY_NAMES,
  )
"""

import sys
import os
import warnings
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from scipy.stats import pearsonr, spearmanr
from sklearn.ensemble import RandomForestRegressor
from sklearn.model_selection import StratifiedKFold, cross_val_score, learning_curve
from sklearn.metrics import r2_score, mean_absolute_error, mean_squared_error
from sklearn.inspection import permutation_importance
from matplotlib.patches import Patch

warnings.filterwarnings("ignore", category=FutureWarning)

# ── Path resolution ──────────────────────────────────────────────────────────
_project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

# Import db_manager directly (avoid MCAP-heavy database/__init__.py)
import importlib.util
_db_manager_path = os.path.join(os.path.dirname(__file__), "..", "database", "db_manager.py")
_spec = importlib.util.spec_from_file_location("_db_manager_mod_rf", _db_manager_path)
_db_manager_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_db_manager_mod)
get_database_df = _db_manager_mod.get_database_df

# Reuse EDA constants
from dev_logs.analysis.eda.eda_angle_prediction import (
    IMU_COLS, DISPLAY_NAMES, FEATURE_GROUPS, GROUP_COLORS,
    huber_regressor, GRAPHICS_DIR,
)

# ── Thesis plots output directory ─────────────────────────────────────────
THESIS_PLOTS_DIR = os.path.join(os.path.dirname(__file__), "..", "..", "..",
                                 "thesis", "plots")
THESIS_PLOTS_DIR = os.path.abspath(THESIS_PLOTS_DIR)

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

# Five consistent fold colors
FOLD_COLORS = ['#1F77B4', '#FF7F0E', '#2CA02C', '#D62728', '#9467BD']


# ══════════════════════════════════════════════════════════════════════════════
#  0.  Data Loading
# ══════════════════════════════════════════════════════════════════════════════

def load_data():
    """
    Load Fixed Cage (+ Rotating Cage) flights with impact_detected == 1
    and excluded == 0. Returns (df_fixed, df_rotating, all_features).
    """
    df_all = get_database_df()

    # Filter: only impact-detected, non-excluded flights
    df_all = df_all.query("impact_detected == 1 and excluded == 0").copy()

    df_fixed = df_all.query("condition == 'Fixed Cage'").copy()
    df_rot = df_all.query("condition == 'Rotating Cage'").copy()

    print(f"📡 Loaded {len(df_fixed)} Fixed Cage + {len(df_rot)} Rotating Cage "
          f"impact flights (excluded passes removed).")

    return df_fixed, df_rot


# ══════════════════════════════════════════════════════════════════════════════
#  1.  Broad Feature Selection
# ══════════════════════════════════════════════════════════════════════════════

def select_features(df, corr_threshold=0.3, redundancy_threshold=0.85):
    """
    Select IMU features with |Pearson r| or |Spearman ρ| > corr_threshold
    vs impact_angle, then remove redundant pairs (keep the stronger one).

    Returns
    -------
    selected : list[str]
        Column names of selected features.
    selection_df : pd.DataFrame
        Table of all features with correlations and selection flags.
    """
    print(f"\n🔍 Feature Selection (|r| or |ρ| > {corr_threshold}, "
          f"redundancy cutoff r > {redundancy_threshold}):")
    print(f"   Candidate pool: {len(IMU_COLS)} IMU features")

    y = df['impact_angle'].values
    records = []
    for col in IMU_COLS:
        if df[col].isna().all():
            continue
        x = df[col].values
        # Pearson r: measures linear correlation (sensitive to outliers, assumes
        # normally-distributed residuals for p-value validity).
        r_p, p_p = pearsonr(x, y)
        # Spearman ρ: rank-based correlation — converts raw values to ranks
        # first, then computes Pearson r on the ranks. This captures monotonic
        # (but not necessarily linear) relationships and is robust to outliers
        # because ranks are bounded regardless of extreme raw values.
        r_s, p_s = spearmanr(x, y)
        records.append({
            'feature': col, 'label': DISPLAY_NAMES[col],
            'group': FEATURE_GROUPS[col],
            'pearson_r': r_p, 'pearson_p': p_p,
            'spearman_r': r_s, 'spearman_p': p_s,
            'abs_pearson': abs(r_p), 'abs_spearman': abs(r_s),
        })

    sel_df = pd.DataFrame(records)

    # Dual filter: a feature passes if EITHER |Pearson r| > threshold OR
    # |Spearman ρ| > threshold. The OR logic ensures we catch both purely
    # linear signals (caught by Pearson) and non-linear but monotonic signals
    # (caught by Spearman). Using both is more permissive than either alone,
    # which is appropriate for a tree-based model that can exploit non-linear
    # associations.
    sel_df['passes_corr'] = (sel_df['abs_pearson'] > corr_threshold) | \
                             (sel_df['abs_spearman'] > corr_threshold)
    candidates = sel_df[sel_df['passes_corr']].copy()
    print(f"   Pass correlation filter: {len(candidates)} features")

    if len(candidates) == 0:
        # Fallback: take top 10 by |pearson_r|
        candidates = sel_df.nlargest(10, 'abs_pearson').copy()
        candidates['passes_corr'] = True
        print(f"   ⚠️  No features passed filter — falling back to top 10 by |r|")

    # ── Redundancy filter ──────────────────────────────────────────────────
    # Compute pairwise Pearson correlations among all candidates. For every
    # pair whose absolute cross-correlation exceeds redundancy_threshold
    # (default 0.85), we keep only the feature with the stronger correlation
    # to the target (impact_angle) and drop the other.
    #
    # Why: highly correlated features (e.g., Peak Accel X vs Accel Energy X)
    # carry nearly identical information. Keeping both inflates feature
    # importance estimates (MDI splits the importance between them) and adds
    # noise to permutation importance without improving prediction.
    #
    # The upper-triangle mask (np.triu, k=1) ensures each pair is checked only
    # once; the feature with the smaller |r| to impact_angle is marked for
    # removal from the final selected set.
    cand_cols = candidates['feature'].tolist()
    selected = list(cand_cols)
    if len(cand_cols) > 1:
        cand_data = df[cand_cols]
        # Pairwise absolute Pearson correlation matrix among candidate features
        corr_matrix = cand_data.corr().abs()
        # Extract the upper triangle (above diagonal), masking out the diagonal
        # and lower triangle so each pair is compared only once.
        upper = corr_matrix.where(np.triu(np.ones(corr_matrix.shape), k=1).astype(bool))
        to_drop = set()
        for i in range(len(cand_cols)):
            for j in range(i + 1, len(cand_cols)):
                if upper.iloc[i, j] > redundancy_threshold:
                    # Drop the one with weaker correlation to target
                    ci, cj = cand_cols[i], cand_cols[j]
                    ri = abs(pearsonr(df[ci], y)[0])
                    rj = abs(pearsonr(df[cj], y)[0])
                    to_drop.add(cj if ri >= rj else ci)

        selected = [c for c in selected if c not in to_drop]
        if to_drop:
            dropped_names = [DISPLAY_NAMES[c] for c in to_drop]
            print(f"   Removed {len(to_drop)} redundant (pairwise r > {redundancy_threshold}):")
            for dn in dropped_names:
                print(f"      ✂  {dn}")
    else:
        to_drop = set()

    sel_df['selected'] = sel_df['feature'].isin(selected)
    sel_df['redundant'] = sel_df['feature'].isin(to_drop)

    print(f"   ✅ Final feature set: {len(selected)} features")
    for c in selected:
        r = sel_df[sel_df['feature'] == c]['pearson_r'].values[0]
        print(f"      {DISPLAY_NAMES[c]:30s}  r = {r:+.4f}")

    return selected, sel_df


# ══════════════════════════════════════════════════════════════════════════════
#  2.  Main RF Pipeline
# ══════════════════════════════════════════════════════════════════════════════

def run_rf_pipeline(train_condition='Fixed Cage', save_to_disk=True, show_plots=True,
                    df_impacts=None, df_train=None, df_transfer=None):
    """
    Full Random Forest pipeline:
      1. Load & select features
      2. Grid search max_depth/min_samples_leaf via nested CV
      3. 5-fold CV with final model
      4. Actual-vs-Predicted plot
      5. MDI Feature Importance plot
      6. Cross-condition transfer (train → other condition)
      7. Permutation importance + learning curve + Huber baseline

    Parameters
    ----------
    train_condition : str
        Condition to train on — 'Fixed Cage' (default) or 'Rotating Cage'.
        The model is then tested on the other condition for transfer analysis.
    save_to_disk : bool
        If True, save figures to GRAPHICS_DIR.
    show_plots : bool
        If True, call plt.show() for inline notebook display.
    df_impacts : pd.DataFrame or None
        Universal impact-only dataframe from the notebook.
        Must contain columns: condition, impact_angle, flight_name, excluded,
        plus all IMU_COLS.  If None, calls load_data() which queries the DB
        directly (backward-compatible fallback).

    Returns
    -------
    results : dict
        Keys: model, features, cv_scores, predictions, importance_df, ...
    """
    transfer_condition = 'Rotating Cage' if train_condition == 'Fixed Cage' else 'Fixed Cage'
    file_prefix = '' if train_condition == 'Fixed Cage' else f"{train_condition.lower().replace(' ', '_')}_"

    print("=" * 72)
    print(f"🌲 RandomForestRegressor: Impact Angle Prediction Pipeline <{train_condition}>")
    print("=" * 72)

    # ── 1. Load data & select features ─────────────────────────────────────
    if df_impacts is not None:
        # Use the universal dataframe from the notebook
        df = df_impacts.query("excluded == 0").copy()
        df_fixed = df.query("condition == 'Fixed Cage'").copy()
        df_rot = df.query("condition == 'Rotating Cage'").copy()
        print(f"📊 Using universal dataframe: {len(df_fixed)} Fixed Cage + "
              f"{len(df_rot)} Rotating Cage impact flights.")
    elif df_train is not None and df_transfer is not None:
        # Two separate dataframes passed (e.g., from notebook's load_impact_data)
        df_fixed = df_train if train_condition == 'Fixed Cage' else df_transfer
        df_rot = df_transfer if train_condition == 'Fixed Cage' else df_train
        print(f"📊 Using pre-loaded dataframes: {len(df_fixed)} Fixed Cage + "
              f"{len(df_rot)} Rotating Cage impact flights.")
    else:
        df_fixed, df_rot = load_data()

    # Select train/transfer data based on condition
    df_train = df_fixed if train_condition == 'Fixed Cage' else df_rot
    df_transfer = df_rot if train_condition == 'Fixed Cage' else df_fixed

    # Drop rows with NaN in any IMU_COLS or impact_angle.
    # condition and flight_name are metadata — keep if present, skip if not
    # (e.g., load_impact_data() does not include them).
    keep_cols = ['impact_angle'] + IMU_COLS
    for col in ['condition', 'flight_name']:
        if col in df_train.columns:
            keep_cols.append(col)
    df_train = df_train[keep_cols].dropna(subset=IMU_COLS + ['impact_angle'])
    print(f"   → {len(df_train)} {train_condition} flights with complete IMU data")

    selected_features, sel_df = select_features(df_train)

    X_all = df_train[selected_features].values
    y_all = df_train['impact_angle'].values
    n_samples = len(y_all)

    if n_samples < 20:
        print(f"❌ Only {n_samples} samples — too few for 5-fold CV. Aborting.")
        return None

    # ── 2. Nested grid search: max_depth × min_samples_leaf ─────────────────
    # Outer CV: 5-fold stratified by angle quartile bins.
    # pd.qcut(y_all, q=5) partitions impact_angle into 5 equal-frequency bins
    # (quintiles). StratifiedKFold then ensures each fold contains roughly the
    # same proportion of flights from each angle bin. This prevents a fold from
    # accidentally containing only shallow-angle (or only steep-angle) flights,
    # which would give an overly optimistic or pessimistic R² estimate.
    angle_bins = pd.qcut(y_all, q=5, labels=False, duplicates='drop')
    outer_cv = StratifiedKFold(n_splits=min(5, n_samples // 3), shuffle=True,
                               random_state=42)

    # Nested cross-validation: the outer loop (StratifiedKFold) evaluates
    # generalization performance, while the inner loop (this grid search over
    # max_depth × min_samples_leaf) selects hyperparameters. By searching
    # within each outer fold, we avoid leaking validation information into
    # hyperparameter selection — each test fold is truly unseen during tuning.
    #   max_depth:        maximum tree depth (smaller = more regularization)
    #   min_samples_leaf: minimum samples per leaf node (larger = smoother trees)
    param_grid = [
        {'max_depth': d, 'min_samples_leaf': l}
        for d in [3, 4, 5, 6]
        for l in [3, 5, 7]
    ]

    print(f"\n🎯 Nested CV — grid search over {len(param_grid)} param combos "
          f"({outer_cv.get_n_splits()}-fold):")
    print(f"   max_depth ∈ [3,4,5,6], min_samples_leaf ∈ [3,5,7]")

    best_params = None
    best_score = -np.inf
    param_scores = []

    for params in param_grid:
        fold_scores = []
        for train_idx, val_idx in outer_cv.split(X_all, angle_bins):
            X_tr, X_val = X_all[train_idx], X_all[val_idx]
            y_tr, y_val = y_all[train_idx], y_all[val_idx]

            rf = RandomForestRegressor(
                n_estimators=200, max_features='sqrt',
                min_samples_split=max(10, params['min_samples_leaf'] * 2),
                random_state=42, n_jobs=-1,
                **params,
            )
            rf.fit(X_tr, y_tr)
            y_pred = rf.predict(X_val)
            fold_scores.append(r2_score(y_val, y_pred))

        mean_r2 = np.mean(fold_scores)
        std_r2 = np.std(fold_scores)
        param_scores.append({**params, 'r2_mean': mean_r2, 'r2_std': std_r2})
        print(f"   depth={params['max_depth']}, leaf={params['min_samples_leaf']:2d}  "
              f"→ R² = {mean_r2:.4f} ± {std_r2:.4f}")

        if mean_r2 > best_score:
            best_score = mean_r2
            best_params = params

    print(f"\n   🏆 Best: max_depth={best_params['max_depth']}, "
          f"min_samples_leaf={best_params['min_samples_leaf']}  "
          f"(R² = {best_score:.4f})")

    # ── 3. Final 5-fold CV with best params ─────────────────────────────────
    print(f"\n🔄 Final 5-Fold CV (max_depth={best_params['max_depth']}, "
          f"min_samples_leaf={best_params['min_samples_leaf']}):")

    final_cv = StratifiedKFold(n_splits=min(5, n_samples // 3), shuffle=True,
                               random_state=123)
    fold_records = []
    all_y_true, all_y_pred = [], []
    fold_ids = []

    # RandomForestRegressor configuration:
    #   n_estimators=200      — 200 trees; more trees reduce variance of the
    #                            ensemble at the cost of linear compute time.
    #   max_features='sqrt'   — each split considers √p features (p = total
    #                            features); this decorrelates trees, reducing
    #                            ensemble variance (standard Breiman recipe).
    #   max_depth             — limits tree depth to prevent overfitting (from
    #                            the grid search best params).
    #   min_samples_leaf      — minimum samples per leaf; enforces smoothness
    #                            (from grid search best params).
    #   min_samples_split     — must have ≥ this many samples to split a node
    #                            (set to max(10, leaf*2) as a floor).
    #   oob_score=True        — Out-Of-Bag score: each tree is tested on the
    #                            ~37% of samples not in its bootstrap sample,
    #                            giving a free unbiased performance estimate.
    rf_final = RandomForestRegressor(
        n_estimators=200, max_features='sqrt',
        max_depth=best_params['max_depth'],
        min_samples_leaf=best_params['min_samples_leaf'],
        min_samples_split=max(10, best_params['min_samples_leaf'] * 2),
        random_state=42, n_jobs=-1, oob_score=True,
    )

    for fold_i, (train_idx, val_idx) in enumerate(final_cv.split(X_all, angle_bins)):
        X_tr, X_val = X_all[train_idx], X_all[val_idx]
        y_tr, y_val = y_all[train_idx], y_all[val_idx]

        rf_final.fit(X_tr, y_tr)
        y_pred = rf_final.predict(X_val)

        r2 = r2_score(y_val, y_pred)
        mae = mean_absolute_error(y_val, y_pred)
        rmse = np.sqrt(mean_squared_error(y_val, y_pred))

        fold_records.append({'fold': fold_i + 1, 'n_train': len(y_tr),
                             'n_val': len(y_val), 'r2': r2,
                             'mae': mae, 'rmse': rmse})
        all_y_true.extend(y_val)
        all_y_pred.extend(y_pred)
        fold_ids.extend([fold_i] * len(y_val))

        print(f"   Fold {fold_i + 1}: R² = {r2:.4f}  MAE = {mae:.2f}°  "
              f"RMSE = {rmse:.2f}°  (n_train={len(y_tr)}, n_val={len(y_val)})")

    fold_df = pd.DataFrame(fold_records)
    overall_r2 = r2_score(all_y_true, all_y_pred)
    overall_mae = mean_absolute_error(all_y_true, all_y_pred)
    overall_rmse = np.sqrt(mean_squared_error(all_y_true, all_y_pred))

    print(f"   ──────────────────────────────────────────")
    print(f"   OVERALL CV: R² = {overall_r2:.4f}  "
          f"MAE = {overall_mae:.2f}°  RMSE = {overall_rmse:.2f}°")
    print(f"   OOB Score: {rf_final.oob_score_:.4f}")

    # ── Fit on ALL Fixed Cage data for feature importance ───────────────────
    rf_full = RandomForestRegressor(
        n_estimators=200, max_features='sqrt',
        max_depth=best_params['max_depth'],
        min_samples_leaf=best_params['min_samples_leaf'],
        min_samples_split=max(10, best_params['min_samples_leaf'] * 2),
        random_state=42, n_jobs=-1, oob_score=True,
    )
    rf_full.fit(X_all, y_all)

    # ── 4. Actual vs Predicted Plot ────────────────────────────────────────
    print(f"\n📊 Generating Actual-vs-Predicted plot...")
    all_y_true_arr = np.array(all_y_true)
    all_y_pred_arr = np.array(all_y_pred)
    fold_ids_arr = np.array(fold_ids)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6), dpi=150, sharex=True, sharey=True)

    # Left: scatter colored by fold
    for fi in range(final_cv.get_n_splits()):
        mask = fold_ids_arr == fi
        ax1.scatter(all_y_true_arr[mask], all_y_pred_arr[mask],
                    c=FOLD_COLORS[fi], s=55, alpha=0.75, edgecolor='white',
                    linewidth=0.5, label=f'Fold {fi + 1}', zorder=3)

    # Diagonal y = x
    lims = [min(all_y_true_arr.min(), all_y_pred_arr.min()) - 5,
            max(all_y_true_arr.max(), all_y_pred_arr.max()) + 5]
    ax1.plot(lims, lims, 'k--', linewidth=1.2, alpha=0.6, label='y = x', zorder=2)

    # ±5° band
    ax1.fill_between(lims, [l - 5 for l in lims], [l + 5 for l in lims],
                     alpha=0.08, color='gray', label='±5° band')

    ax1.set_xlim(0, 90)
    ax1.set_ylim(0, 90)
    ax1.set_xticks([0, 15, 30, 45, 60, 75, 90])
    ax1.set_yticks([0, 15, 30, 45, 60, 75, 90])
    ax1.set_aspect('equal', adjustable='box')
    ax1.set_xlabel('Actual Impact Angle (°)', fontweight='bold')
    ax1.set_ylabel('Predicted Impact Angle (°)', fontweight='bold')
    ax1.set_title(f'<{train_condition}> 5-Fold CV: Predicted vs Actual\n'
                  f'R² = {overall_r2:.3f}  MAE = {overall_mae:.1f}°  '
                  f'RMSE = {overall_rmse:.1f}°',
                  fontweight='bold', fontsize=11)
    ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=8, framealpha=0.85)
    ax1.grid(True, linestyle=':', alpha=0.4)

    # Right: residuals vs predicted.
    # Residual = actual − predicted (positive = model under-predicts).
    # A good model shows residuals centered at zero (no systematic bias),
    # with roughly constant spread across the predicted range (homoscedasticity).
    # The ±5° band highlights whether prediction errors stay within a
    # practically useful tolerance for impact angle estimation.
    residuals = all_y_true_arr - all_y_pred_arr
    for fi in range(final_cv.get_n_splits()):
        mask = fold_ids_arr == fi
        ax2.scatter(all_y_pred_arr[mask], residuals[mask],
                    c=FOLD_COLORS[fi], s=55, alpha=0.75, edgecolor='white',
                    linewidth=0.5, zorder=3)

    ax2.axhline(0, color='k', linestyle='--', linewidth=1.2, alpha=0.6, zorder=2)
    ax2.fill_between([all_y_pred_arr.min() - 2, all_y_pred_arr.max() + 2],
                     -5, 5, alpha=0.08, color='gray')
    ax2.set_xlim(0, 90)
    ax2.set_ylim(-45, 45)
    ax2.set_xticks([0, 15, 30, 45, 60, 75, 90])
    ax2.set_yticks([-45, -30, -15, 0, 15, 30, 45])
    ax2.set_xlabel('Predicted Impact Angle (°)', fontweight='bold')
    ax2.set_ylabel('Residual (Actual − Predicted) (°)', fontweight='bold')
    ax2.set_title(f'Residual Plot\nMean residual = {np.mean(residuals):.2f}°  '
                  f'Std = {np.std(residuals):.1f}°',
                  fontweight='bold', fontsize=11)
    ax2.grid(True, linestyle=':', alpha=0.4)

    fig.tight_layout()
    if save_to_disk:
        path = os.path.join(GRAPHICS_DIR, f"{file_prefix}rf_actual_vs_predicted.png")
        plt.savefig(path, dpi=300, bbox_inches='tight')
        print(f"   💾 Saved → {os.path.relpath(path, os.path.dirname(__file__))}")
    if show_plots:
        plt.show()
    plt.close(fig)

    # ── 5. MDI Feature Importance ──────────────────────────────────────────
    # MDI (Mean Decrease in Impurity): for each feature, sum the reduction in
    # MSE (or Gini for classification) across every split that uses that feature,
    # averaged over all 200 trees. A high MDI means the feature is frequently
    # used at splits that substantially reduce prediction error.
    # CAVEAT: MDI is biased toward high-cardinality and continuous features
    # (they offer more split points). This is why we also compute permutation
    # importance (section 7a) as a less biased alternative.
    print(f"\n📊 Generating MDI Feature Importance...")
    # rf_full.feature_importances_ returns one value per feature, normalized
    # so they sum to 1.0 across all features.
    importances = rf_full.feature_importances_
    imp_df = pd.DataFrame({
        'feature': selected_features,
        'importance': importances,
        'label': [DISPLAY_NAMES[f] for f in selected_features],
        'group': [FEATURE_GROUPS[f] for f in selected_features],
    }).sort_values('importance', ascending=True)

    fig, ax = plt.subplots(figsize=(9, max(5, len(selected_features) * 0.35)), dpi=150)

    bar_colors = [GROUP_COLORS.get(g, '#CCCCCC') for g in imp_df['group']]
    bars = ax.barh(imp_df['label'], imp_df['importance'], color=bar_colors,
                   edgecolor='black', linewidth=0.5, alpha=0.75)

    # Annotate values
    for bar, val in zip(bars, imp_df['importance']):
        ax.text(bar.get_width() + 0.001, bar.get_y() + bar.get_height() / 2,
                f'{val:.3f}', va='center', fontsize=8.5, fontweight='bold')

    ax.set_xlabel('MDI Feature Importance', fontweight='bold')
    ax.set_title(f'<{train_condition}> Random Forest Feature Importance (MDI)\n'
                 f'{len(selected_features)} features, OOB R² = {rf_full.oob_score_:.3f}',
                 fontweight='bold', fontsize=12)
    ax.grid(True, linestyle=':', alpha=0.4, axis='x')

    # Legend for group colors
    unique_groups = imp_df['group'].unique()
    legend_elements = [Patch(facecolor=GROUP_COLORS.get(g, '#CCCCCC'),
                             alpha=0.75, label=g) for g in unique_groups
                       if g in GROUP_COLORS]
    ax.legend(handles=legend_elements, fontsize=8, loc='lower right',
              title='Feature Group', title_fontsize=9)

    fig.tight_layout()
    if save_to_disk:
        path = os.path.join(GRAPHICS_DIR, f"{file_prefix}rf_feature_importance.png")
        plt.savefig(path, dpi=300, bbox_inches='tight')
        print(f"   💾 Saved → {os.path.relpath(path, os.path.dirname(__file__))}")
    if show_plots:
        plt.show()
    plt.close(fig)

    # ── 6. Cross-Condition Transfer: train → transfer ──────────────────────
    # Test generalization across experimental conditions: train the RF on one
    # cage type (e.g., Fixed Cage) and evaluate it on the other (e.g., Rotating
    # Cage) WITHOUT any retraining. This is a stronger test than within-condition
    # CV because it checks whether the learned IMU→angle mapping is invariant
    # to the mechanical change in the obstacle setup. A large drop in R² would
    # indicate that the cage mechanism changes the IMU footprint fundamentally.
    print(f"\n🔄 Cross-Condition Transfer: {train_condition} model → {transfer_condition}...")
    transfer_data = df_transfer[selected_features + ['impact_angle']].dropna()
    print(f"   {transfer_condition}: {len(transfer_data)} flights with complete data")

    results_transfer = None
    if len(transfer_data) > 0:
        X_transfer = transfer_data[selected_features].values
        y_transfer = transfer_data['impact_angle'].values
        y_transfer_pred = rf_full.predict(X_transfer)
        transfer_r2 = r2_score(y_transfer, y_transfer_pred)
        transfer_mae = mean_absolute_error(y_transfer, y_transfer_pred)
        transfer_rmse = np.sqrt(mean_squared_error(y_transfer, y_transfer_pred))

        fig, ax = plt.subplots(figsize=(7, 6), dpi=150)

        # Train condition CV points (grey, light)
        ax.scatter(all_y_true_arr, all_y_pred_arr,
                   c='#AAAAAA', s=35, alpha=0.3, edgecolor='none',
                   label=f'{train_condition} CV (R² = {overall_r2:.3f})', zorder=2)

        # Transfer condition points (strong color)
        ax.scatter(y_transfer, y_transfer_pred,
                   c='#D62728', s=70, alpha=0.8, edgecolor='white',
                   linewidth=0.8, marker='D',
                   label=f'{transfer_condition} (R² = {transfer_r2:.3f})', zorder=3)

        # y = x line
        all_vals = np.concatenate([all_y_true_arr, all_y_pred_arr, y_transfer, y_transfer_pred])
        lims = [all_vals.min() - 5, all_vals.max() + 5]
        ax.plot(lims, lims, 'k--', linewidth=1.2, alpha=0.6, label='y = x', zorder=1)

        ax.set_xlabel('Actual Impact Angle (°)', fontweight='bold')
        ax.set_ylabel('Predicted Impact Angle (°)', fontweight='bold')
        ax.set_title(f'<{train_condition}> → <{transfer_condition}> Cross-Condition Transfer\n'
                     f'{transfer_condition}: R² = {transfer_r2:.3f}  '
                     f'MAE = {transfer_mae:.1f}°  RMSE = {transfer_rmse:.1f}°',
                     fontweight='bold', fontsize=11)
        ax.legend(loc='lower right', fontsize=9, framealpha=0.85)
        ax.grid(True, linestyle=':', alpha=0.4)

        fig.tight_layout()
        if save_to_disk:
            path = os.path.join(GRAPHICS_DIR, f"{file_prefix}rf_cross_condition.png")
            plt.savefig(path, dpi=300, bbox_inches='tight')
            print(f"   💾 Saved → {os.path.relpath(path, os.path.dirname(__file__))}")
        if show_plots:
            plt.show()
        plt.close(fig)

        results_transfer = {'r2': transfer_r2, 'mae': transfer_mae, 'rmse': transfer_rmse,
                            'y_true': y_transfer, 'y_pred': y_transfer_pred,
                            'condition': transfer_condition}
        print(f"   {transfer_condition} R² = {transfer_r2:.3f}  "
              f"MAE = {transfer_mae:.1f}°  RMSE = {transfer_rmse:.1f}°")
    else:
        print(f"   ⚠️  No {transfer_condition} data with complete features — skipping transfer.")

    # ── 7a. Permutation Importance ─────────────────────────────────────────
    # Permutation importance measures how much model performance degrades when
    # a feature's values are randomly shuffled (breaking its association with
    # the target). Algorithm:
    #   1. Fit model on original data, record baseline MSE.
    #   2. Shuffle one feature column, breaking its link to impact_angle.
    #   3. Re-score model; importance = increase in MSE from baseline.
    #   4. Repeat n_repeats=20 times to get mean ± std of importance.
    # Unlike MDI, permutation importance is model-agnostic, does not favor
    # high-cardinality features, and reflects actual predictive value (not
    # just split frequency). The trade-off: it is computationally expensive
    # because the model must be re-scored 20 times per feature.
    print(f"\n📊 Computing Permutation Importance (may be slow)...")
    try:
        perm_result = permutation_importance(
            rf_full, X_all, y_all, n_repeats=20, random_state=42, n_jobs=-1,
            scoring='neg_mean_squared_error',
        )
        perm_df = pd.DataFrame({
            'feature': selected_features,
            'importance': perm_result.importances_mean,
            'std': perm_result.importances_std,
            'label': [DISPLAY_NAMES[f] for f in selected_features],
            'group': [FEATURE_GROUPS[f] for f in selected_features],
        }).sort_values('importance', ascending=True)

        fig, ax = plt.subplots(figsize=(9, max(5, len(selected_features) * 0.35)),
                               dpi=150)
        bar_colors = [GROUP_COLORS.get(g, '#CCCCCC') for g in perm_df['group']]
        ax.barh(perm_df['label'], perm_df['importance'], xerr=perm_df['std'],
                color=bar_colors, edgecolor='black', linewidth=0.5, alpha=0.75,
                capsize=3)

        ax.set_xlabel('Permutation Importance (Δ MSE)', fontweight='bold')
        ax.set_title(f'<{train_condition}> Permutation Feature Importance\n'
                     f'{len(selected_features)} features, 20 repeats',
                     fontweight='bold', fontsize=12)
        ax.grid(True, linestyle=':', alpha=0.4, axis='x')

        fig.tight_layout()
        if save_to_disk:
            path = os.path.join(GRAPHICS_DIR, f"{file_prefix}rf_permutation_importance.png")
            plt.savefig(path, dpi=300, bbox_inches='tight')
            print(f"   💾 Saved → {os.path.relpath(path, os.path.dirname(__file__))}")
        if show_plots:
            plt.show()
        plt.close(fig)
    except Exception as e:
        print(f"   ⚠️  Permutation importance failed: {e}")
        perm_df = None

    # ── 7b. Learning Curve ─────────────────────────────────────────────────
    # A learning curve plots model performance (R²) against training set size.
    # sklearn's learning_curve() trains the model on progressively larger
    # subsets of the data (30%, 40%, ..., 100%) and evaluates both training
    # and validation R² at each step via cross-validation (lc_cv).
    #
    # Diagnostic value:
    #   - Large gap (train >> val) → overfitting; more data or regularization helps.
    #   - Small gap, both low       → underfitting; model too simple.
    #   - Val R² still climbing     → more data would likely improve performance.
    #   - Val R² plateaued          → data is no longer the bottleneck.
    print(f"\n📊 Generating Learning Curve...")
    try:
        from sklearn.model_selection import KFold
        # Test 8 training sizes from 30% to 100% of the data, evenly spaced.
        train_sizes = np.linspace(0.3, 1.0, 8)
        lc_cv = KFold(n_splits=min(5, n_samples // 3), shuffle=True,
                      random_state=42)

        train_sizes_abs, train_scores, test_scores = learning_curve(
            rf_full, X_all, y_all, train_sizes=train_sizes,
            cv=lc_cv, scoring='r2', n_jobs=-1,
        )

        train_mean = np.mean(train_scores, axis=1)
        train_std = np.std(train_scores, axis=1)
        test_mean = np.mean(test_scores, axis=1)
        test_std = np.std(test_scores, axis=1)

        fig, ax = plt.subplots(figsize=(8, 5), dpi=150)
        ax.fill_between(train_sizes_abs, train_mean - train_std,
                        train_mean + train_std, alpha=0.15, color='#1F77B4')
        ax.fill_between(train_sizes_abs, test_mean - test_std,
                        test_mean + test_std, alpha=0.15, color='#FF7F0E')
        ax.plot(train_sizes_abs, train_mean, 'o-', color='#1F77B4',
                linewidth=2, markersize=6, label='Training R²')
        ax.plot(train_sizes_abs, test_mean, 's-', color='#FF7F0E',
                linewidth=2, markersize=6, label='Validation R²')

        # Overfitting gap annotation
        gap = train_mean[-1] - test_mean[-1]
        ax.axhline(y=0, color='gray', linestyle=':', alpha=0.4)
        ax.set_xlabel('Training Set Size (flights)', fontweight='bold')
        ax.set_ylabel('R² Score', fontweight='bold')
        ax.set_title(f'<{train_condition}> Learning Curve\n'
                     f'Final: Train R² = {train_mean[-1]:.3f}  '
                     f'Val R² = {test_mean[-1]:.3f}  '
                     f'Gap = {gap:.3f}',
                     fontweight='bold', fontsize=12)
        ax.legend(loc='lower right', fontsize=10)
        ax.grid(True, linestyle=':', alpha=0.4)
        ax.set_ylim(0.4, 1.0)
        ax.set_yticks([0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0])

        fig.tight_layout()
        if save_to_disk:
            path = os.path.join(GRAPHICS_DIR, f"{file_prefix}rf_learning_curve.png")
            plt.savefig(path, dpi=300, bbox_inches='tight')
            print(f"   💾 Saved → {os.path.relpath(path, os.path.dirname(__file__))}")
        if show_plots:
            plt.show()
        plt.close(fig)
    except Exception as e:
        print(f"   ⚠️  Learning curve failed: {e}")
        train_sizes_abs, train_mean, train_std, test_mean, test_std, gap = None, None, None, None, None, None

    # ── 7c. Huber Baseline Comparison ──────────────────────────────────────
    # Establish a simple linear baseline: fit a robust Huber regression using
    # only the single best (highest |r|) IMU feature, evaluated on the same
    # cross-validation folds as the RF. This answers the question: "Does the
    # multi-feature, non-linear Random Forest actually outperform a simple
    # linear model on the single best sensor channel?"
    #
    # The top feature (selected_features[0]) is the one with strongest |r|
    # against impact_angle after the redundancy filter. Using the same CV
    # splits ensures the comparison is fair (both models see identical
    # train/validation partitions).
    print(f"\n📊 Huber Linear Baseline Comparison...")
    huber_scores = []
    huber_fold_preds = {'y_true': [], 'y_pred': [], 'fold': []}

    for fold_i, (train_idx, val_idx) in enumerate(final_cv.split(X_all, angle_bins)):
        X_tr, X_val = X_all[train_idx], X_all[val_idx]
        y_tr, y_val = y_all[train_idx], y_all[val_idx]

        # Use the top feature (strongest |r| with impact_angle)
        top_f = selected_features[0]
        # Fit Huber robust regression on the single best feature column
        slope, intercept = huber_regressor(X_tr[:, 0], y_tr)
        y_pred_huber = slope * X_val[:, 0] + intercept

        huber_scores.append(r2_score(y_val, y_pred_huber))
        huber_fold_preds['y_true'].extend(y_val)
        huber_fold_preds['y_pred'].extend(y_pred_huber)
        huber_fold_preds['fold'].extend([fold_i] * len(y_val))

    huber_r2 = np.mean(huber_scores)
    huber_r2_std = np.std(huber_scores)
    huber_mae = mean_absolute_error(huber_fold_preds['y_true'],
                                     huber_fold_preds['y_pred'])
    huber_rmse = np.sqrt(mean_squared_error(huber_fold_preds['y_true'],
                                             huber_fold_preds['y_pred']))

    # Comparison bar chart: side-by-side visual of Huber (1-feature linear)
    # vs. Random Forest (multi-feature non-linear) on the same CV folds.
    # Error bars show ±1 std across folds, capturing fold-to-fold variability.
    # If the RF bar is substantially higher than the Huber bar, the extra
    # features and non-linear interactions provide real predictive value
    # beyond what a single linear channel can capture.
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5), dpi=150)

    # Bar chart: R² comparison
    models = ['Huber\n(1 feature)', f'Random Forest\n({len(selected_features)} features)']
    r2_vals = [huber_r2, overall_r2]
    r2_errs = [huber_r2_std, np.std(fold_df['r2'])]
    bars = ax1.bar(models, r2_vals, yerr=r2_errs, color=['#AAAAAA', '#2CA02C'],
                   edgecolor='black', linewidth=1.2, capsize=8, width=0.5)
    for bar, val in zip(bars, r2_vals):
        ax1.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.02,
                 f'{val:.3f}', ha='center', fontweight='bold', fontsize=13)
    ax1.set_ylabel('R² Score', fontweight='bold')
    ax1.set_xticklabels(models, rotation=45, ha='right')
    ax1.set_title(f'<{train_condition}> Model Comparison: R²', fontweight='bold', fontsize=12)
    ax1.set_ylim(0, 1.0)
    ax1.set_yticks([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0])
    ax1.grid(True, linestyle=':', alpha=0.4, axis='y')

    # Scatter: Huber predictions vs actual (all folds)
    ax2.scatter(huber_fold_preds['y_true'], huber_fold_preds['y_pred'],
                c='#1F77B4', s=55, alpha=0.7, edgecolor='white', linewidth=0.5)
    lims_h = [min(huber_fold_preds['y_true']) - 5,
              max(huber_fold_preds['y_true']) + 5]
    ax2.plot(lims_h, lims_h, 'k--', linewidth=1.2, alpha=0.6)
    ax2.set_xlabel('Actual Impact Angle (°)', fontweight='bold')
    ax2.set_ylabel('Predicted Impact Angle (°)', fontweight='bold')
    ax2.set_title(f'<{train_condition}> Huber Baseline: {DISPLAY_NAMES[top_f]}\n'
                  f'R² = {huber_r2:.3f}  MAE = {huber_mae:.1f}°  '
                  f'RMSE = {huber_rmse:.1f}°',
                  fontweight='bold', fontsize=11)
    ax2.grid(True, linestyle=':', alpha=0.4)

    fig.tight_layout()
    if save_to_disk:
        path = os.path.join(GRAPHICS_DIR, f"{file_prefix}rf_huber_baseline.png")
        plt.savefig(path, dpi=300, bbox_inches='tight')
        print(f"   💾 Saved → {os.path.relpath(path, os.path.dirname(__file__))}")
    if show_plots:
        plt.show()
    plt.close(fig)

    print(f"   Huber R² = {huber_r2:.3f} ± {huber_r2_std:.3f}  "
          f"(RF R² = {overall_r2:.3f})")
    delta = overall_r2 - huber_r2
    print(f"   ΔR² (RF − Huber) = {delta:+.3f}  "
          f"({'🎉 Multi-feature RF wins!' if delta > 0.01 else '⚠️  Linear baseline competitive'})")

    # ── Compile results ────────────────────────────────────────────────────
    results = {
        'model': rf_full,
        'features': selected_features,
        'best_params': best_params,
        'param_scores': param_scores,
        'fold_scores': fold_df,
        'overall_r2': overall_r2,
        'overall_mae': overall_mae,
        'overall_rmse': overall_rmse,
        'y_true': all_y_true_arr,
        'y_pred': all_y_pred_arr,
        'fold_ids': fold_ids_arr,
        'importance_df': imp_df,
        'permutation_df': perm_df,
        'rotating_results': results_transfer,  # backward compat (may be Fixed if training on Rotating)
        'transfer_results': results_transfer,
        'train_condition': train_condition,
        'transfer_condition': transfer_condition,
        'huber_r2': huber_r2,
        'huber_r2_std': huber_r2_std,
        'selection_df': sel_df,
        'learning_curve': dict(train_sizes=train_sizes_abs, train_mean=train_mean, train_std=train_std, test_mean=test_mean, test_std=test_std, gap=gap) if train_mean is not None else None,
        'huber_fold_preds': huber_fold_preds,
        'huber_top_feature': top_f,
        'huber_mae': huber_mae,
        'huber_rmse': huber_rmse,
        'n_features': len(selected_features),
    }

    print(f"\n{'=' * 72}")
    print(f"✅ RF Pipeline complete. Summary:")
    print(f"   Train condition:  {train_condition}")
    print(f"   Features: {len(selected_features)}")
    print(f"   Best params: max_depth={best_params['max_depth']}, "
          f"min_samples_leaf={best_params['min_samples_leaf']}")
    print(f"   CV R²: {overall_r2:.3f}  MAE: {overall_mae:.1f}°  "
          f"RMSE: {overall_rmse:.1f}°")
    print(f"   OOB R²: {rf_full.oob_score_:.3f}")
    print(f"   Huber R²: {huber_r2:.3f}")
    if results_transfer:
        print(f"   {transfer_condition} R²: {results_transfer['r2']:.3f}")
    print(f"   All plots → {os.path.relpath(GRAPHICS_DIR, os.path.dirname(__file__))}/")
    print(f"{'=' * 72}")

    return results


# ══════════════════════════════════════════════════════════════════════════════
# ══════════════════════════════════════════════════════════════════════════════
#  7d.  Consolidated Feature Importance (Fixed + Rotating, 1×2)
# ══════════════════════════════════════════════════════════════════════════════

def plot_consolidated_feature_importance(results_fix, results_rot,
                                          save_to_disk=True, show_plots=True):
    """
    Consolidated Random Forest Feature Importance — 2 rows × 1 column.

    Top:    Rotating Cage
    Bottom: Fixed Cage

    Each panel shows the features selected by that cage's own RF model,
    sorted by MDI ascending.  MDI importance is shown as a solid filled
    horizontal bar and Permutation importance as an outlined (hollow) bar
    of the same feature-group colour, making the two metrics directly
    comparable per feature.

    Parameters
    ----------
    results_fix, results_rot : dict
        Return values of run_rf_pipeline() for each cage condition.
    save_to_disk : bool
        Save PNGs to GRAPHICS_DIR.
    show_plots : bool
        Display figures inline (for notebooks).
    """
    from matplotlib.patches import Patch

    conditions = [
        ('Rotating Cage', results_rot),
        ('Fixed Cage', results_fix),
    ]

    # Collect unique groups across both cages for the legend
    all_groups = set()
    for _, res in conditions:
        if res and res.get('importance_df') is not None:
            for g in res['importance_df']['group'].unique():
                all_groups.add(g)
    all_groups = sorted(all_groups)

    # ── Build 2×1 figure ───────────────────────────────────────────────────
    n_feats = max(
        len(results_fix.get('importance_df', pd.DataFrame())),
        len(results_rot.get('importance_df', pd.DataFrame())),
    )
    n_feats = max(n_feats, 10)  # ensure reasonable minimum height
    fig, axes = plt.subplots(2, 1, figsize=(12, max(10, n_feats * 0.9)),
                              dpi=150, sharex=False)

    # Both MDI and Permutation are normalised to [0, 1] within each panel,
    # so the shared x-axis limit is fixed at 1.12 (12% headroom above 1.0).
    x_lim = 1.12

    for row_idx, (cond_name, res) in enumerate(conditions):
        ax = axes[row_idx]

        if res is None:
            ax.text(0.5, 0.5, f'{cond_name}\nNo data',
                    transform=ax.transAxes, ha='center', va='center',
                    fontsize=14, fontweight='bold', color='gray')
            continue

        imp_df = res.get('importance_df')
        perm_df = res.get('permutation_df')

        if imp_df is None or len(imp_df) == 0:
            ax.text(0.5, 0.5, f'{cond_name}\nNo importance data',
                    transform=ax.transAxes, ha='center', va='center',
                    fontsize=11, fontstyle='italic', color='gray')
            continue

        # ── Build MDI + Permutation series from THIS cage's own features ────
        # Each cage's model uses its own selected_features set; we plot only
        # those, sorted by MDI ascending (horizontal bars go bottom→top).
        # There is no cross-cage union — each panel is self-contained.
        panel_rows = imp_df.sort_values('importance', ascending=True).copy()
        labels = panel_rows['label'].tolist()
        mdi_raw = panel_rows['importance'].tolist()
        bar_colors = [GROUP_COLORS.get(g, '#CCCCCC') for g in panel_rows['group']]

        # Permutation importance for the same features
        perm_raw = []
        if perm_df is not None:
            perm_lookup = dict(zip(perm_df['feature'], perm_df['importance']))
            for f in panel_rows['feature']:
                perm_raw.append(perm_lookup.get(f, 0))
        else:
            perm_raw = [0] * len(labels)

        # Normalise both to [0, 1] so MDI (proportions, ~0.01–0.15)
        # and Permutation (MSE drops, typically 10–100× larger) share a
        # comparable visual scale on the same axis.
        mdi_max = max(mdi_raw) if mdi_raw else 1
        perm_max = max(perm_raw) if perm_raw else 1
        mdi_norm = [v / mdi_max if mdi_max > 0 else 0 for v in mdi_raw]
        perm_norm = [v / perm_max if perm_max > 0 else 0 for v in perm_raw]

        # All lists are already sorted by MDI ascending at this point

        y_pos = range(len(labels))
        bar_height = 0.35

        # MDI — solid filled bars (top half of each row)
        ax.barh([y + bar_height / 2 for y in y_pos], mdi_norm, bar_height,
                color=bar_colors, edgecolor='black', linewidth=0.5,
                alpha=0.85)

        # Permutation — outlined hollow bars (bottom half, same colour as edge)
        ax.barh([y - bar_height / 2 for y in y_pos], perm_norm, bar_height,
                color='none', edgecolor=bar_colors, linewidth=1.2,
                alpha=0.9)

        # ── Raw value labels on the right side of each bar ─────────────
        for i, (y, md_n, pm_n, md_r, pm_r) in enumerate(
                zip(y_pos, mdi_norm, perm_norm, mdi_raw, perm_raw)):
            # MDI raw value (proportion, format with 4 decimal places)
            if md_n > 0.001:
                ax.text(md_n + 0.015, y + bar_height / 2,
                        f'{md_r:.4f}',
                        va='center', ha='left', fontsize=9, fontweight='bold',
                        color='black')
            # Permutation raw value (MSE drop, format with 2 decimal places)
            if pm_n > 0.001:
                ax.text(pm_n + 0.015, y - bar_height / 2,
                        f'{pm_r:.2f}',
                        va='center', ha='left', fontsize=9, fontweight='bold',
                        color='black')

        ax.set_yticks(y_pos)
        ax.set_yticklabels(labels, fontsize=8)
        ax.set_xlim(0, x_lim)
        if row_idx == 1:
            ax.set_xlabel('Normalised Importance (prop. of max)', fontweight='bold', fontsize=9)
        ax.set_title(cond_name, fontweight='bold', fontsize=12, pad=6,
                     color='black')
        ax.grid(True, linestyle=':', alpha=0.4, axis='x')
        ax.tick_params(labelsize=8)

    # ── MDI / Permutation legend (Left) + Feature Group legend (Right) ────
    method_handles = [
        Patch(facecolor='#555555', alpha=0.85, label='MDI (solid fill)'),
        Patch(facecolor='none', edgecolor='#555555', linewidth=1.5, label='Permutation (outline)'),
    ]
    group_handles = [
        Patch(facecolor=GROUP_COLORS.get(g, '#CCCCCC'), alpha=0.75, label=g)
        for g in all_groups if g in GROUP_COLORS
    ]

    # Anchor to y=0.12 to sit safely below the x-axis label
    leg1 = fig.legend(handles=method_handles, fontsize=7.5, loc='upper right',
                      ncol=2, bbox_to_anchor=(0.38, 0.1), framealpha=0.85,
                      title='Importance Type', title_fontsize=8)
    fig.add_artist(leg1)

    fig.legend(handles=group_handles, fontsize=7.5, loc='upper left',
               ncol=min(5, len(group_handles)),
               bbox_to_anchor=(0.42, 0.1), framealpha=0.85,
               title='Feature Group', title_fontsize=8)

    # ── Global title ───────────────────────────────────────────────────────
    fig.suptitle('Random Forest Feature Importance: MDI + Permutation',
                 fontweight='bold', fontsize=14, y=0.98)

    # INCREASE bottom to 0.22 to give the legends and text room to breathe
    fig.subplots_adjust(top=0.93, bottom=0.14, hspace=0.14, left=0.28, right=0.94)

    # ── Data origin ──────────────────────────────────────────────────────
    n_rot = len(results_rot.get('y_true', []))
    n_fix = len(results_fix.get('y_true', []))

    # Move the note to y=0.02 so it sits completely underneath the legends
    fig.text(0.94, 0.02,
             f"flights_summary (N={n_rot} Rotating, N={n_fix} Fixed)",
             ha='right', va='bottom', fontsize=7.5, color='#555555')
    if save_to_disk:
        path = os.path.join(GRAPHICS_DIR, "consolidated_feature_importance.png")
        plt.savefig(path, dpi=300, bbox_inches='tight')
        print(f"   💾 Saved consolidated feature importance → "
              f"{os.path.relpath(path, os.path.dirname(__file__))}")
    if show_plots:
        plt.show()
    plt.close(fig)


# ══════════════════════════════════════════════════════════════════════════════
# ══════════════════════════════════════════════════════════════════════════════
#  8.  Consolidated Plotting Functions (Fixed + Rotating Cage side-by-side)
# ══════════════════════════════════════════════════════════════════════════════
# These take two results dicts (one per cage condition) and produce single
# figures with Fixed Cage on the left and Rotating Cage on the right.

CONDITION_COLORS = {'Rotating Cage': '#1F77B4', 'Fixed Cage': '#D62728'}


def plot_consolidated_actual_vs_predicted(results_fix, results_rot,
                                           save_to_disk=True, show_plots=True):
    """
    Consolidated Predicted vs Actual Impact Angle — 1×2 figure.

    Left panel:  Fixed Cage 5-fold CV scatter (colored by fold)
    Right panel: Rotating Cage 5-fold CV scatter (colored by fold)

    Both panels share x/y limits (0–90°), equal aspect ratio, ticks every 15°,
    and a y = x diagonal reference line with ±5° shaded band.
    """
    from matplotlib.lines import Line2D

    conditions = [
        ('Rotating Cage', results_rot),
        ('Fixed Cage', results_fix),
    ]

    fig, axes = plt.subplots(1, 2, figsize=(12, 6), dpi=150,
                              sharex=True, sharey=True)

    for col_idx, (cond_name, res) in enumerate(conditions):
        ax = axes[col_idx]
        if res is None:
            ax.text(0.5, 0.5, f'<{cond_name}>\nNo data',
                    transform=ax.transAxes, ha='center', va='center',
                    fontsize=14, fontweight='bold', color='gray')
            continue

        y_true = res['y_true']
        y_pred = res['y_pred']
        fold_ids = res['fold_ids']

        # Scatter colored by fold
        n_folds = int(fold_ids.max()) + 1 if len(fold_ids) > 0 else 1
        for fi in range(n_folds):
            mask = fold_ids == fi
            ax.scatter(y_true[mask], y_pred[mask],
                       c=FOLD_COLORS[fi % len(FOLD_COLORS)], s=55,
                       alpha=0.75, edgecolor='white', linewidth=0.5,
                       label=f'Fold {fi + 1}', zorder=3)

        # y = x diagonal
        ax.plot([0, 90], [0, 90], 'k--', linewidth=1.2, alpha=0.6,
                label='y = x', zorder=2)

        # ±5° band
        ax.fill_between([0, 90], [-5, 85], [5, 95],
                        alpha=0.08, color='gray', label='±5° band')

        # Strict formatting
        ax.set_xlim(0, 90)
        ax.set_ylim(0, 90)
        ax.set_xticks([0, 15, 30, 45, 60, 75, 90])
        ax.set_yticks([0, 15, 30, 45, 60, 75, 90])
        ax.set_aspect('equal', adjustable='box')

        ax.set_xlabel('Actual Impact Angle (°)', fontweight='bold')

        if col_idx == 0:
            ax.set_ylabel('Predicted Impact Angle (°)', fontweight='bold')

        r2 = res.get('overall_r2', 0)
        mae = res.get('overall_mae', 0)
        rmse = res.get('overall_rmse', 0)

        # Metrics overlay box (larger font for legibility)
        ax.text(0.95, 0.95,
                f'R² = {r2:.3f}\nMAE = {mae:.1f}°\nRMSE = {rmse:.1f}°',
                transform=ax.transAxes, ha='right', va='top',
                fontsize=11, fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                          alpha=0.85, edgecolor='#CCCCCC'))

        ax.set_title(cond_name, fontweight='bold', fontsize=12, color='black')
        ax.grid(True, linestyle=':', alpha=0.4)

    # ── Green/red comparison legend (compact, high contrast) ──────────────
    r2_fix, r2_rot = results_fix.get('overall_r2', 0), results_rot.get('overall_r2', 0)
    mae_fix, mae_rot = results_fix.get('overall_mae', 0), results_rot.get('overall_mae', 0)
    rmse_fix, rmse_rot = results_fix.get('overall_rmse', 0), results_rot.get('overall_rmse', 0)

    r2_better = 'Rotating Cage' if r2_rot > r2_fix else 'Fixed Cage'
    mae_better = 'Rotating Cage' if mae_rot < mae_fix else 'Fixed Cage'
    rmse_better = 'Rotating Cage' if rmse_rot < rmse_fix else 'Fixed Cage'

    legend_handles = []
    legend_handles.append(
        plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='#228B22',
                   markersize=10, label=f'Better (higher R²: {r2_better})'))
    legend_handles.append(
        plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='#DC143C',
                   markersize=10, label=f'Worse (lower R²: {"Fixed Cage" if r2_better == "Rotating Cage" else "Rotating Cage"})'))

    # Compact legend in the upper-left of the left (Rotating) plot
    axes[0].legend(handles=legend_handles, loc='upper left', fontsize=7,
                   framealpha=0.85, title='CV Performance Comparison',
                   title_fontsize=8)

    fig.suptitle('5-Fold Cross-Validation: Predicted vs Actual Impact Angle',
                 fontweight='bold', fontsize=14, y=0.98)

    fig.subplots_adjust(top=0.90, bottom=0.18, wspace=0.0)
    # ── Data origin ──────────────────────────────────────────────────────
    n_rot = len(results_rot.get('y_true', []))
    n_fix = len(results_fix.get('y_true', []))
    fig.text(0.88, 0.08,
             f"flights_summary (N={n_rot} Rotating, N={n_fix} Fixed)",
             ha='right', va='bottom', fontsize=7.5, color='#555555')
    if save_to_disk:
        path = os.path.join(GRAPHICS_DIR, "consolidated_actual_vs_predicted.png")
        plt.savefig(path, dpi=300, bbox_inches='tight')
        print(f"   💾 Saved → {os.path.relpath(path, os.path.dirname(__file__))}")
    if show_plots:
        plt.show()
    plt.close(fig)


def plot_consolidated_residuals(results_fix, results_rot,
                                 save_to_disk=True, show_plots=True):
    """
    Consolidated Residual Plots — 1×2 figure.

    Left panel:  Fixed Cage residuals (actual − predicted)
    Right panel: Rotating Cage residuals

    Both panels: x = Predicted Angle (0–90°), y = Residual (−45–45°),
    ticks every 15° on both axes, horizontal dashed line at y = 0.
    """
    conditions = [
        ('Rotating Cage', results_rot),
        ('Fixed Cage', results_fix),
    ]

    fig, axes = plt.subplots(1, 2, figsize=(12, 6), dpi=150,
                              sharex=True, sharey=True)

    for col_idx, (cond_name, res) in enumerate(conditions):
        ax = axes[col_idx]
        if res is None:
            ax.text(0.5, 0.5, f'<{cond_name}>\nNo data',
                    transform=ax.transAxes, ha='center', va='center',
                    fontsize=14, fontweight='bold', color='gray')
            continue

        y_true = res['y_true']
        y_pred = res['y_pred']
        fold_ids = res['fold_ids']
        residuals = y_true - y_pred

        # Scatter colored by fold
        n_folds = int(fold_ids.max()) + 1 if len(fold_ids) > 0 else 1
        for fi in range(n_folds):
            mask = fold_ids == fi
            ax.scatter(y_pred[mask], residuals[mask],
                       c=FOLD_COLORS[fi % len(FOLD_COLORS)], s=55,
                       alpha=0.75, edgecolor='white', linewidth=0.5, zorder=3)

        # Horizontal zero line (prominent black dashed)
        ax.axhline(0, color='black', linestyle='--', linewidth=1.5, alpha=0.7,
                   zorder=2)

# Strict formatting
        ax.set_xlim(0, 90)
        ax.set_ylim(-30, 30)
        ax.set_xticks([0, 15, 30, 45, 60, 75, 90])
        ax.set_yticks([-30, -15, 0, 15, 30])
        
        ax.set_box_aspect(1)

        ax.set_xlabel('Predicted Impact Angle (°)', fontweight='bold')

        if col_idx == 0:
            ax.set_ylabel('Residual (Actual − Predicted) (°)', fontweight='bold')

        std_res = np.std(residuals)
        mean_res = np.mean(residuals)
        ax.set_title(f'{cond_name}\n'
                     f'Mean = {mean_res:.1f}°  Std = {std_res:.1f}°',
                     fontweight='bold', fontsize=12, color='black')
        ax.grid(True, linestyle=':', alpha=0.4)

    fig.suptitle('Residual Plots — Prediction Error Distribution',
                 fontweight='bold', fontsize=14, y=1)
    fig.subplots_adjust(top=0.90, bottom=0.18, wspace=0.0)
    # ── Data origin ──────────────────────────────────────────────────────
    n_rot = len(results_rot.get('y_true', []))
    n_fix = len(results_fix.get('y_true', []))
    fig.text(0.88, 0.08,
             f"flights_summary (N={n_rot} Rotating, N={n_fix} Fixed)",
             ha='right', va='bottom', fontsize=7.5, color='#555555')
    if save_to_disk:
        path = os.path.join(GRAPHICS_DIR, "consolidated_residuals.png")
        plt.savefig(path, dpi=300, bbox_inches='tight')
        print(f"   💾 Saved → {os.path.relpath(path, os.path.dirname(__file__))}")
    if show_plots:
        plt.show()
    plt.close(fig)


def plot_consolidated_model_comparison(results_fix, results_rot,
                                        save_to_disk=True, show_plots=True):
    """
    Consolidated Model Comparison (R² bar charts) — 1×2 figure.

    Left panel:  Fixed Cage — Huber (1-feature) vs RF (multi-feature)
    Right panel: Rotating Cage — same comparison

    Both panels: y-limits (0, 1.0), ticks every 0.1, x-labels rotated 45°.
    """
    conditions = [
        ('Rotating Cage', results_rot),
        ('Fixed Cage', results_fix),
    ]

    fig, axes = plt.subplots(1, 2, figsize=(12, 6), dpi=150)

    for col_idx, (cond_name, res) in enumerate(conditions):
        ax = axes[col_idx]
        if res is None:
            ax.text(0.5, 0.5, f'<{cond_name}>\nNo data',
                    transform=ax.transAxes, ha='center', va='center',
                    fontsize=14, fontweight='bold', color='gray')
            continue

        huber_r2 = res.get('huber_r2', 0)
        huber_r2_std = res.get('huber_r2_std', 0)
        overall_r2 = res.get('overall_r2', 0)
        n_features = res.get('n_features', res.get('features', []))

        if isinstance(n_features, list):
            n_features = len(n_features)

        models = ['Huber\n(1 feature)', f'Random Forest\n({n_features} features)']
        r2_vals = [huber_r2, overall_r2]
        r2_errs = [huber_r2_std, np.std(res.get('fold_scores', pd.DataFrame({'r2': [overall_r2]}))['r2'])]

        bars = ax.bar(models, r2_vals, yerr=r2_errs,
                      color=['#AAAAAA', '#2CA02C'],
                      edgecolor='black', linewidth=1.2, capsize=8, width=0.5)

        # Annotations centered above each bar, offset vertically to clear
        # the error-bar whisker caps.
        for bar, val, err in zip(bars, r2_vals, r2_errs):
            ax.text(bar.get_x() + bar.get_width() / 2,
                    bar.get_height() + err + 0.03,
                    f'{val:.3f}', ha='center', fontweight='bold', fontsize=13)

        ax.set_ylabel('R² Score', fontweight='bold')
        ax.set_xticks(range(len(models)))
        ax.set_xticklabels(models, rotation=0, ha='center')
        ax.set_title(cond_name, fontweight='bold', fontsize=12, color='black')
        ax.set_ylim(0, 1.0)
        ax.set_yticks([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0])
        ax.grid(True, linestyle=':', alpha=0.4, axis='y')
        ax.set_box_aspect(1.25)

    fig.suptitle('Model Comparison: Huber vs Random Forest',
                 fontweight='bold', fontsize=14, y=0.98)
    fig.subplots_adjust(top=0.90, bottom=0.18, wspace=0.0)
    # ── Data origin ──────────────────────────────────────────────────────
    n_rot = len(results_rot.get('y_true', []))
    n_fix = len(results_fix.get('y_true', []))
    fig.text(0.88, 0.08,
             f"flights_summary (N={n_rot} Rotating, N={n_fix} Fixed)",
             ha='right', va='bottom', fontsize=7.5, color='#555555')
    if save_to_disk:
        path = os.path.join(GRAPHICS_DIR, "consolidated_model_comparison.png")
        plt.savefig(path, dpi=300, bbox_inches='tight')
        print(f"   💾 Saved → {os.path.relpath(path, os.path.dirname(__file__))}")
    if show_plots:
        plt.show()
    plt.close(fig)


def plot_consolidated_cross_condition_transfer(results_fix, results_rot,
                                                save_to_disk=True,
                                                show_plots=True):
    """
    Consolidated Cross-Condition Transfer — 1 row × 2 columns.

    Left:   "Rotating Cage    to    Fixed Cage"
            Model trained on Rotating Cage, tested on Fixed Cage.
            Rotating within-CV points = Blue Squares (marker='s', color='blue')
            Fixed transfer points    = Red Dots    (marker='o', color='red')

    Right:  "Fixed Cage    to    Rotating Cage"
            Model trained on Fixed Cage, tested on Rotating Cage.
            Fixed within-CV points   = Red Dots    (marker='o', color='red')
            Rotating transfer points = Blue Squares (marker='s', color='blue')

    Axes: 0–90°, ticks every 10°. Single unified legend.

    Parameters
    ----------
    results_fix, results_rot : dict
        Return values of run_rf_pipeline() for each cage condition.
    save_to_disk : bool
        Save PNGs to GRAPHICS_DIR.
    show_plots : bool
        Display figures inline (for notebooks).
    """
    from matplotlib.lines import Line2D

    fig, axes = plt.subplots(1, 2, figsize=(13, 6), dpi=150,
                              sharex=True, sharey=True)

    # ── Left panel: model trained on Rotating Cage → tested on Fixed Cage ──
    ax_l = axes[0]
    rot_res = results_rot
    fix_res = results_fix
    rot_transfer = rot_res.get('transfer_results') if rot_res else None

    if rot_res is not None and rot_transfer is not None:
        # Rotating within-CV points (Blue Squares)
        ax_l.scatter(rot_res['y_true'], rot_res['y_pred'],
                     marker='s', color='blue', s=50, alpha=0.5,
                     edgecolor='white', linewidth=0.5, zorder=3,
                     label='Rotating Cage (within CV)')
        # Fixed transfer points (Red Dots)
        ax_l.scatter(rot_transfer['y_true'], rot_transfer['y_pred'],
                     marker='o', color='red', s=65, alpha=0.8,
                     edgecolor='white', linewidth=0.5, zorder=4,
                     label=f"Fixed Cage (transfer)\nR² = {rot_transfer['r2']:.3f}")
    else:
        ax_l.text(0.5, 0.5, 'No transfer data', transform=ax_l.transAxes,
                  ha='center', va='center', fontsize=13, color='gray')

    # y = x line
    ax_l.plot([0, 90], [0, 90], 'k--', linewidth=1.2, alpha=0.6, zorder=2)
    ax_l.set_xlim(0, 90)
    ax_l.set_ylim(0, 90)
    ax_l.set_xticks(range(0, 91, 10))
    ax_l.set_yticks(range(0, 91, 10))
    ax_l.set_xlabel('Actual Impact Angle (°)', fontweight='bold')
    ax_l.set_ylabel('Predicted Impact Angle (°)', fontweight='bold')
    ax_l.set_title('Rotating Cage   →   Fixed Cage',
                   fontweight='bold', fontsize=12, color='black')
    ax_l.grid(True, linestyle=':', alpha=0.4)
    ax_l.set_box_aspect(1)

    # ── Right panel: model trained on Fixed Cage → tested on Rotating Cage ─
    ax_r = axes[1]
    fix_transfer = fix_res.get('transfer_results') if fix_res else None

    if fix_res is not None and fix_transfer is not None:
        # Fixed within-CV points (Red Dots)
        ax_r.scatter(fix_res['y_true'], fix_res['y_pred'],
                     marker='o', color='red', s=50, alpha=0.5,
                     edgecolor='white', linewidth=0.5, zorder=3,
                     label='Fixed Cage (within CV)')
        # Rotating transfer points (Blue Squares)
        ax_r.scatter(fix_transfer['y_true'], fix_transfer['y_pred'],
                     marker='s', color='blue', s=65, alpha=0.8,
                     edgecolor='white', linewidth=0.5, zorder=4,
                     label=f"Rotating Cage (transfer)\nR² = {fix_transfer['r2']:.3f}")
    else:
        ax_r.text(0.5, 0.5, 'No transfer data', transform=ax_r.transAxes,
                  ha='center', va='center', fontsize=13, color='gray')

    # y = x line
    ax_r.plot([0, 90], [0, 90], 'k--', linewidth=1.2, alpha=0.6, zorder=2)
    ax_r.set_xlim(0, 90)
    ax_r.set_ylim(0, 90)
    ax_r.set_xticks(range(0, 91, 10))
    ax_r.set_yticks(range(0, 91, 10))
    ax_r.set_xlabel('Actual Impact Angle (°)', fontweight='bold')
    ax_r.set_ylabel('Predicted Impact Angle (°)', fontweight='bold')
    ax_r.set_title('Fixed Cage   →   Rotating Cage',
                   fontweight='bold', fontsize=12, color='black')
    ax_r.grid(True, linestyle=':', alpha=0.4)
    ax_r.set_box_aspect(1)

    # ── Single unified legend (compact, large legible markers) ────────────
    legend_elements = [
        Line2D([0], [0], marker='s', color='w', markerfacecolor='blue',
               markersize=12, label='Rotating'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='red',
               markersize=12, label='Fixed'),
    ]
    axes[0].legend(handles=legend_elements, loc='upper left', fontsize=9,
                   framealpha=0.85)

    fig.suptitle('Cross Condition Transfer',
                 fontweight='bold', fontsize=14, y=0.98)
    fig.subplots_adjust(top=0.90, bottom=0.18, wspace=0.0)
    # ── Data origin ──────────────────────────────────────────────────────
    n_rot = len(results_rot.get('y_true', []))
    n_fix = len(results_fix.get('y_true', []))
    fig.text(0.88, 0.08,
             f"flights_summary (N={n_rot} Rotating, N={n_fix} Fixed)",
             ha='right', va='bottom', fontsize=7.5, color='#555555')
    if save_to_disk:
        path = os.path.join(GRAPHICS_DIR,
                            "consolidated_cross_condition_transfer.png")
        plt.savefig(path, dpi=300, bbox_inches='tight')
        print(f"   💾 Saved consolidated cross-condition transfer → "
              f"{os.path.relpath(path, os.path.dirname(__file__))}")
    if show_plots:
        plt.show()
    plt.close(fig)


def plot_consolidated_learning_curves(results_fix, results_rot,
                                       save_to_disk=True, show_plots=True):
    """
    Consolidated Learning Curves — 1×2 figure.

    Left panel:  Fixed Cage learning curve
    Right panel: Rotating Cage learning curve

    Both panels: y-limits (0.4, 1.0), ticks every 0.1,
    shows training R² (blue) vs validation R² (orange).
    """
    conditions = [
        ('Rotating Cage', results_rot),
        ('Fixed Cage', results_fix),
    ]

    fig, axes = plt.subplots(1, 2, figsize=(12, 6), dpi=150,
                              sharex=True, sharey=True)

    for col_idx, (cond_name, res) in enumerate(conditions):
        ax = axes[col_idx]
        if res is None or res.get('learning_curve') is None:
            ax.text(0.5, 0.5, f'<{cond_name}>\nNo data',
                    transform=ax.transAxes, ha='center', va='center',
                    fontsize=14, fontweight='bold', color='gray')
            continue

        lc = res['learning_curve']
        train_sizes = lc['train_sizes']
        train_mean = lc['train_mean']
        train_std = lc['train_std']
        test_mean = lc['test_mean']
        test_std = lc['test_std']
        gap = lc['gap']

        if train_mean is None:
            continue

        ax.fill_between(train_sizes, train_mean - train_std,
                        train_mean + train_std, alpha=0.15, color='#1F77B4')
        ax.fill_between(train_sizes, test_mean - test_std,
                        test_mean + test_std, alpha=0.15, color='#FF7F0E')
        ax.plot(train_sizes, train_mean, 'o-', color='#1F77B4',
                linewidth=2, markersize=6, label='Training R²')
        ax.plot(train_sizes, test_mean, 's-', color='#FF7F0E',
                linewidth=2, markersize=6, label='Validation R²')

        ax.axhline(y=0, color='gray', linestyle=':', alpha=0.4)
        ax.set_xlabel('Training Set Size (flights)', fontweight='bold')
        if col_idx == 0:
            ax.set_ylabel('R² Score', fontweight='bold')

        ax.set_title(f'{cond_name}\n'
                     f'Final: Train R² = {train_mean[-1]:.3f}  '
                     f'Val R² = {test_mean[-1]:.3f}  Gap = {gap:.3f}',
                     fontweight='bold', fontsize=10, color='black')
        ax.legend(loc='lower right', fontsize=8)
        ax.grid(True, linestyle=':', alpha=0.4)
        ax.set_ylim(-0.2, 1.0)
        ax.set_yticks([-0.2, 0.0, 0.2, 0.4, 0.6, 0.8, 1.0])
        ax.set_box_aspect(1)

    fig.suptitle('Learning Curves: Training Size vs R² Score',
                 fontweight='bold', fontsize=14, y=1)
    fig.subplots_adjust(top=0.90, bottom=0.18, wspace=0.0)
    # ── Data origin ──────────────────────────────────────────────────────
    n_rot = len(results_rot.get('y_true', []))
    n_fix = len(results_fix.get('y_true', []))
    fig.text(0.88, 0.08,
             f"flights_summary (N={n_rot} Rotating, N={n_fix} Fixed)",
             ha='right', va='bottom', fontsize=7.5, color='#555555')
    if save_to_disk:
        path = os.path.join(GRAPHICS_DIR, "consolidated_learning_curves.png")
        plt.savefig(path, dpi=300, bbox_inches='tight')
        print(f"   💾 Saved → {os.path.relpath(path, os.path.dirname(__file__))}")
    if show_plots:
        plt.show()
    plt.close(fig)


def generate_consolidated_plots(results_fix, results_rot,
                                 save_to_disk=True, show_plots=True):
    """
    Generate all 6 consolidated comparison figures from the Fixed Cage
    and Rotating Cage results dicts.

    Produces:
      1. consolidated_actual_vs_predicted.png       — 5-fold CV scatter (1×2)
      2. consolidated_residuals.png                  — residual distribution (1×2)
      3. consolidated_model_comparison.png           — Huber vs RF R² bars (1×2)
      4. consolidated_learning_curves.png            — training size vs R² (1×2)
      5. consolidated_feature_importance.png         — MDI + Permutation (2×2)
      6. consolidated_cross_condition_transfer.png   — transfer both dirs (1×2)

    Parameters
    ----------
    results_fix : dict
        Return value of run_rf_pipeline(train_condition='Fixed Cage').
    results_rot : dict
        Return value of run_rf_pipeline(train_condition='Rotating Cage').
    save_to_disk : bool
        Save PNGs to GRAPHICS_DIR.
    show_plots : bool
        Display figures inline (for notebooks).
    """
    print("\n" + "=" * 72)
    print("📊 Generating Consolidated Comparison Figures")
    print("=" * 72)

    print("\n1/6  Actual vs Predicted (Rotating × Fixed)")
    plot_consolidated_actual_vs_predicted(results_fix, results_rot,
                                           save_to_disk, show_plots)

    print("\n2/6  Residuals (Rotating × Fixed)")
    plot_consolidated_residuals(results_fix, results_rot,
                                 save_to_disk, show_plots)

    print("\n3/6  Model Comparison — Huber vs RF (Rotating × Fixed)")
    plot_consolidated_model_comparison(results_fix, results_rot,
                                        save_to_disk, show_plots)

    print("\n4/6  Learning Curves (Rotating × Fixed)")
    plot_consolidated_learning_curves(results_fix, results_rot,
                                       save_to_disk, show_plots)

    print("\n5/6  Feature Importance (Rotating × Fixed)")
    plot_consolidated_feature_importance(results_fix, results_rot,
                                          save_to_disk, show_plots)

    print("\n6/6  Cross Condition Transfer (both directions)")
    plot_consolidated_cross_condition_transfer(results_fix, results_rot,
                                                save_to_disk, show_plots)

    print(f"\n✅ All 6 consolidated figures → "
          f"{os.path.relpath(GRAPHICS_DIR, os.path.dirname(__file__))}/")

    # ── Dual-save to thesis/plots/ per §4.0.1 ────────────────────────────
    _thesis_copies = {
        "consolidated_actual_vs_predicted.png":
            "consolidated_actual_vs_predicted.png",
        "consolidated_residuals.png":
            "consolidated_residuals.png",
        "consolidated_model_comparison.png":
            "consolidated_model_comparison.png",
        "consolidated_learning_curves.png":
            "consolidated_learning_curves.png",
        "consolidated_feature_importance.png":
            "consolidated_feature_importance.png",
        "consolidated_cross_condition_transfer.png":
            "consolidated_cross_condition_transfer.png",
    }
    os.makedirs(THESIS_PLOTS_DIR, exist_ok=True)
    for src_name, dst_name in _thesis_copies.items():
        src = os.path.join(GRAPHICS_DIR, src_name)
        dst = os.path.join(THESIS_PLOTS_DIR, dst_name)
        if os.path.exists(src):
            shutil.copy2(src, dst)
            print(f"   📎 → thesis/plots/{dst_name}")
    print(f"✅ Thesis copies → {THESIS_PLOTS_DIR}/")

#  Main — run all analysis when called as a script
# ══════════════════════════════════════════════════════════════════════════════

def main():
    run_rf_pipeline(save_to_disk=True, show_plots=False)


if __name__ == "__main__":
    main()
