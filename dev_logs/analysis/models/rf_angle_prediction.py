#!/usr/bin/env python3
# ── Headless-safe backend (Agg when no display, notebook overrides) ──────────
import os, sys as _sys
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

def run_rf_pipeline(train_condition='Fixed Cage', save_to_disk=True, show_plots=True):
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
    df_fixed, df_rot = load_data()

    # Select train/transfer data based on condition
    df_train = df_fixed if train_condition == 'Fixed Cage' else df_rot
    df_transfer = df_rot if train_condition == 'Fixed Cage' else df_fixed

    # Drop rows with NaN in any IMU_COLS or impact_angle
    keep_cols = ['impact_angle', 'condition', 'flight_name'] + IMU_COLS
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

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6), dpi=150)

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

    ax1.set_xlabel('Actual Impact Angle (°)', fontweight='bold')
    ax1.set_ylabel('Predicted Impact Angle (°)', fontweight='bold')
    ax1.set_title(f'<{train_condition}> 5-Fold CV: Predicted vs Actual\n'
                  f'R² = {overall_r2:.3f}  MAE = {overall_mae:.1f}°  '
                  f'RMSE = {overall_rmse:.1f}°',
                  fontweight='bold', fontsize=11)
    ax1.legend(loc='lower right', fontsize=8, framealpha=0.85)
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
        train_mean, test_mean, gap = None, None, None

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
    ax1.set_title(f'<{train_condition}> Model Comparison: R²', fontweight='bold', fontsize=12)
    ax1.set_ylim(0, max(r2_vals) * 1.25)
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
#  Main — run all analysis when called as a script
# ══════════════════════════════════════════════════════════════════════════════

def main():
    run_rf_pipeline(save_to_disk=True, show_plots=False)


if __name__ == "__main__":
    main()
