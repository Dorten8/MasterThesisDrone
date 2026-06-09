"""Representative Flight Finder — finds the most "average" flight in a category.

Scores flights by multivariate Euclidean distance from the category centroid
after z-score standardization of selected features. The flight with the smallest
distance is the most representative of its category.

Usage (from notebook):
    from dev_logs.analysis.database.representative_finder import RepresentativeFlightFinder

    finder = RepresentativeFlightFinder(condition="Rotating Cage")
    finder.print_summary(top_n=3)
"""

import os
import re
import sqlite3
import numpy as np
import pandas as pd


# ── Default feature set (10 metrics spanning the 4 key thesis dimensions) ──
DEFAULT_FEATURES = [
    "impact_angle",          # Contact Geometry — #1 most important
    "impact_speed",          # Impact Dynamics
    "impact_accel",          # Impact Dynamics
    "before_impact_accel",   # Impact Dynamics
    "recovery_area",         # Recovery
    "max_dev_after",         # Recovery
    "avg_dev_after",         # Recovery
    "imu_peak_accel_z",      # IMU / Structural
    "imu_gyro_energy",       # IMU / Structural
    "imu_accel_settling",    # IMU / Structural
]


class RepresentativeFlightFinder:
    """Finds flights most representative of their category.

    Parameters
    ----------
    db_path : str or None
        Path to experiments_summary.db. Auto-resolved if None.
    condition : str or None
        Exact match on flights_summary.condition (e.g. "Rotating Cage").
    impact_angle_range : tuple of (float, float) or None
        Inclusive range filter on impact_angle column, e.g. (20, 35).
    extra_query : str or None
        Arbitrary pandas .query() expression for additional filtering.
    feature_columns : list of str or None
        Override the default 10-feature set. Must be columns in flights_summary.
    """

    def __init__(
        self,
        db_path=None,
        condition=None,
        impact_angle_range=None,
        extra_query=None,
        feature_columns=None,
    ):
        if db_path is None:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            db_path = os.path.abspath(os.path.join(current_dir, "..", "experiments_summary.db"))

        self.db_path = db_path
        self.condition = condition
        self.impact_angle_range = impact_angle_range
        self.extra_query = extra_query
        self.feature_columns = feature_columns or DEFAULT_FEATURES

        # ── Load & filter ──
        self._df_all = self._load()
        self._df_cat = self._apply_filters()
        self._df_ranked = None   # populated by find_representative()

    # ──────────────────────────────────────────────────────────────────────
    #  Internal helpers
    # ──────────────────────────────────────────────────────────────────────

    def _load(self):
        conn = sqlite3.connect(self.db_path, timeout=30.0)
        conn.execute("PRAGMA busy_timeout = 30000")
        df = pd.read_sql_query("SELECT * FROM flights_summary ORDER BY condition DESC, flight_name ASC", conn)
        conn.close()
        return df

    def _apply_filters(self):
        df = self._df_all.copy()

        if self.condition is not None:
            df = df.query("condition == @self.condition")

        if self.impact_angle_range is not None:
            lo, hi = self.impact_angle_range
            df = df.query("@lo <= impact_angle <= @hi")

        if self.extra_query is not None:
            df = df.query(self.extra_query)

        if df.empty:
            raise ValueError(
                "No flights match the given filters "
                f"(condition={self.condition!r}, "
                f"impact_angle_range={self.impact_angle_range!r}, "
                f"extra_query={self.extra_query!r})"
            )

        # Drop rows where any selected feature is null
        available = [c for c in self.feature_columns if c in df.columns]
        missing = set(self.feature_columns) - set(available)
        if missing:
            print(f"⚠️  Warning: columns not in DB, skipping: {missing}")

        df = df.dropna(subset=available)
        if df.empty:
            raise ValueError("No flights remain after dropping rows with null features.")

        return df

    def _parse_nominal_angle(self, flight_name):
        """Extract nominal angle (45° or 75°) from flight_name string."""
        if "45" in flight_name:
            return "45°"
        elif "75" in flight_name:
            return "75°"
        return "—"

    # ──────────────────────────────────────────────────────────────────────
    #  Public API
    # ──────────────────────────────────────────────────────────────────────

    @property
    def category_label(self):
        """Human-readable category description."""
        parts = []
        if self.condition:
            parts.append(self.condition)
        if self.impact_angle_range:
            parts.append(f"impact angle {self.impact_angle_range[0]:.0f}°–{self.impact_angle_range[1]:.0f}°")
        return " | ".join(parts) if parts else "All flights"

    @property
    def n_flights(self):
        return len(self._df_cat)

    def find_representative(self, top_n=3):
        """Compute centroid distances and return top-N ranked flights.

        Returns
        -------
        pd.DataFrame
            A copy of the filtered DataFrame with added columns:
            ``_distance`` (float, σ units) and ``_rank`` (int).
            Sorted by distance ascending.
        """
        available = [c for c in self.feature_columns if c in self._df_cat.columns]
        if not available:
            raise ValueError("None of the requested feature_columns are in the database.")

        X = self._df_cat[available].to_numpy(dtype=np.float64)

        mean = X.mean(axis=0)
        std = X.std(axis=0, ddof=1)
        # Guard against zero-variance columns
        std[std == 0] = 1.0

        X_z = (X - mean) / std
        distances = np.sqrt(np.sum(X_z ** 2, axis=1))

        df_out = self._df_cat.copy()
        df_out["_distance"] = distances
        df_out["_rank"] = df_out["_distance"].rank(method="dense").astype(int)
        df_out = df_out.sort_values("_distance").reset_index(drop=True)

        self._df_ranked = df_out
        self._mean = mean
        self._std = std
        return df_out.head(top_n)

    def compare_to_mean(self, flight_name):
        """Print a side-by-side comparison of one flight vs its category mean.

        Parameters
        ----------
        flight_name : str
            Exact flight_name as stored in the database.
        """
        if flight_name not in self._df_cat["flight_name"].values:
            print(f"❌ Flight '{flight_name}' not found in category '{self.category_label}'.")
            return

        # Ensure ranked data exists
        if self._df_ranked is None:
            self.find_representative()

        row = self._df_cat[self._df_cat["flight_name"] == flight_name].iloc[0]
        distance = self._df_ranked.loc[
            self._df_ranked["flight_name"] == flight_name, "_distance"
        ].values
        dist_str = f"{distance[0]:.3f}σ" if len(distance) > 0 else "—"

        available = [c for c in self.feature_columns if c in self._df_cat.columns]
        means = self._mean
        stds = self._std

        # ── Print ──
        print()
        print(f"   📋 {flight_name}  (distance={dist_str})")
        print(f"   {'─' * 72}")
        self._print_metadata(row)
        print(f"   {'─' * 72}")
        print(f"   {'Metric':<28} {'Flight':>10} {'Category Mean±SD':>22} {'Δ(σ)':>8}")
        print(f"   {'─' * 72}")

        for i, col in enumerate(available):
            flight_val = row[col]
            mu = means[i]
            sd = stds[i]
            z = (flight_val - mu) / sd if sd != 0 else 0.0

            # Format numbers compactly
            f_val = self._fmt(flight_val)
            f_mu = self._fmt(mu)
            f_sd = self._fmt(sd)
            print(f"   {col:<28} {f_val:>10} {f_mu + '±' + f_sd:>22} {z:>+8.2f}")

        print()

    def _print_metadata(self, row):
        """Print contextual metadata for a flight."""
        flight_name = row.get("flight_name", "—")
        nominal = self._parse_nominal_angle(str(flight_name))
        impact_angle = row.get("impact_angle")
        battery = row.get("battery_at_start")
        sweep_speed = row.get("sweep_speed")
        timestamp = row.get("timestamp", "—")

        impact_str = f"{impact_angle:.1f}°" if impact_angle is not None and not np.isnan(impact_angle) else "—"
        battery_str = f"{battery:.1f}%" if battery is not None and not np.isnan(battery) else "—"
        speed_str = f"{sweep_speed:.2f} m/s" if sweep_speed is not None and not np.isnan(sweep_speed) else "—"

        print(f"   Context:")
        print(f"     Nominal angle:     {nominal}")
        print(f"     Achieved angle:    {impact_str}")
        print(f"     Battery at start:  {battery_str}")
        print(f"     Sweep speed:       {speed_str}")
        print(f"     Recorded:          {timestamp}")

    def print_summary(self, top_n=3):
        """Full formatted report: top-N representative flights with comparisons.

        Parameters
        ----------
        top_n : int
            Number of top-ranked flights to display (default 3).
        """
        top_df = self.find_representative(top_n=top_n)

        n_feat = len([c for c in self.feature_columns if c in self._df_cat.columns])

        print()
        print(f"🔍 Representative Flights: {self.category_label}")
        print(f"   Category size: {self.n_flights} flights | Features: {n_feat}")
        print()

        medals = ["🥇", "🥈", "🥉"] + ["  "] * (top_n - 3)
        for i, (_, row) in enumerate(top_df.iterrows()):
            rank = i + 1
            dist = row["_distance"]
            name = row["flight_name"]
            print(f"{medals[i]} Rank {rank} (distance={dist:.3f}σ): {name}")
            self.compare_to_mean(str(name))

    @staticmethod
    def _fmt(val):
        """Compact numeric formatting."""
        if val is None or (isinstance(val, float) and np.isnan(val)):
            return "—"
        if isinstance(val, (int, np.integer)):
            return str(val)
        if isinstance(val, float):
            # Choose decimal places based on magnitude
            if abs(val) < 0.01:
                return f"{val:.4f}"
            elif abs(val) < 1:
                return f"{val:.3f}"
            elif abs(val) < 10:
                return f"{val:.2f}"
            elif abs(val) < 100:
                return f"{val:.2f}"
            elif abs(val) < 1000:
                return f"{val:.1f}"
            else:
                return f"{val:.0f}"
        return str(val)
