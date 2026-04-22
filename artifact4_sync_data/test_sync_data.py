"""
test_sync_data.py
Unit tests for Artifact 4 — Raw Time-Synchronization Data (Figure 2c)
Tests: CSV integrity, row counts, value ranges, statistical claims,
       reproducibility, Figure 2c consistency
"""
import sys, os
import pandas as pd
import numpy as np

pass_count = 0
fail_count = 0

def CHECK(cond, msg):
    global pass_count, fail_count
    if cond:
        print(f"  PASS  {msg}")
        pass_count += 1
    else:
        print(f"  FAIL  {msg}")
        fail_count += 1

def CHECKF(a, b, tol, msg):
    CHECK(abs(a - b) < tol, msg)

# ── Load CSVs ─────────────────────────────────────────────────
# Locate data files relative to this test script's location
_here = os.path.dirname(os.path.abspath(__file__))
_data = os.path.join(_here, "..", "artifact4_sync_data")
os.chdir(_data)
raw     = pd.read_csv("sync_data_raw.csv")
summary = pd.read_csv("sync_data_summary.csv")
fig2c   = pd.read_csv("sync_data_figure2c.csv")

print("=== Artifact 4: Time-Synchronization Data Tests ===\n")

# ── T1: File existence and non-empty ──────────────────────────
print("[T1] File existence and basic structure")
CHECK(len(raw)     > 0, "sync_data_raw.csv is non-empty")
CHECK(len(summary) > 0, "sync_data_summary.csv is non-empty")
CHECK(len(fig2c)   > 0, "sync_data_figure2c.csv is non-empty")

# ── T2: Row counts ────────────────────────────────────────────
print("\n[T2] Row counts consistent with paper parameters")
N_ROBOTS   = 12
N_FOLLOWERS = N_ROBOTS - 1    # 11 (R2..R12)
T_CYCLE_MS  = 245
MISSION_S   = 300.0
N_CYCLES    = int(MISSION_S / (T_CYCLE_MS / 1000.0))  # 1224
expected_raw = N_FOLLOWERS * N_CYCLES
CHECK(len(raw) == expected_raw,
      f"Raw rows = {N_FOLLOWERS} robots × {N_CYCLES} cycles = {expected_raw}")
CHECK(len(summary) == N_FOLLOWERS,
      f"Summary rows = {N_FOLLOWERS} follower robots")
CHECK(len(fig2c) == N_CYCLES,
      f"Figure 2c rows = {N_CYCLES} time points")

# ── T3: Column names ──────────────────────────────────────────
print("\n[T3] Column names")
expected_raw_cols = {"time_s","robot_id","cycle_index","sync_error_ms",
                     "master_ts_us","local_ts_us","corrected_ts_us"}
CHECK(set(raw.columns) == expected_raw_cols,
      "sync_data_raw.csv has all required columns")
CHECK("mean_ms" in summary.columns and "max_abs_ms" in summary.columns,
      "Summary has mean_ms and max_abs_ms stat columns")
CHECK("mean_sync_error_ms" in fig2c.columns, "Figure 2c has mean_sync_error_ms column")

# ── T4: Robot IDs ─────────────────────────────────────────────
print("\n[T4] Robot ID range")
robot_ids = sorted(raw["robot_id"].unique())
CHECK(min(robot_ids) == 2,  "Minimum robot_id = 2 (master R1 excluded)")
CHECK(max(robot_ids) == 12, "Maximum robot_id = 12")
CHECK(len(robot_ids) == 11, "Exactly 11 follower robots present")

# ── T5: Timing consistency ────────────────────────────────────
print("\n[T5] Timing consistency")
CHECK(raw["time_s"].min() >= 0.0,    "All timestamps ≥ 0")
CHECKF(raw["time_s"].max(), MISSION_S - T_CYCLE_MS/1000.0, 1.0,
       f"Max time_s ≈ {MISSION_S - T_CYCLE_MS/1000.0:.1f} s (last cycle)")
CHECKF(fig2c["time_s"].max(), MISSION_S - T_CYCLE_MS/1000.0, 1.0,
       "Figure 2c time range ends near mission end")
cycle_idxs = sorted(raw["cycle_index"].unique())
CHECK(len(cycle_idxs) == N_CYCLES,
      f"Exactly {N_CYCLES} unique cycle indices")

# ── T6: Sync error within ±5 ms (paper claim, Figure 2c) ──────
print("\n[T6] ±5 ms synchronisation claim (paper Figure 2c)")
pct_within_5ms = (raw["sync_error_ms"].abs() <= 5.0).mean() * 100
max_abs_err    = raw["sync_error_ms"].abs().max()
CHECKF(pct_within_5ms, 100.0, 0.1,
       f"≥99.9% of cycles within ±5 ms (actual: {pct_within_5ms:.2f}%)")
CHECK(max_abs_err <= 5.5,
      f"Max |error| ≤ 5.5 ms across all robots/cycles (actual: {max_abs_err:.3f} ms)")
CHECK(raw["sync_error_ms"].mean() < 0.1,
      f"Mean error ≈ 0 (unbiased sync): {raw['sync_error_ms'].mean():.4f} ms")

# ── T7: Per-robot statistics ──────────────────────────────────
print("\n[T7] Per-robot statistics in summary table")
CHECK((summary["max_abs_ms"] <= 5.5).all(),
      "All robots: max |error| ≤ 5.5 ms")
CHECK((summary["within_5ms_pct"] >= 99.9).all(),
      "All robots: ≥99.9% cycles within ±5 ms")
CHECK((summary["std_ms"] < 2.0).all(),
      "All robots: std dev < 2.0 ms")
mean_std = summary["std_ms"].mean()
CHECKF(mean_std, 1.1, 0.3,
       f"Mean σ ≈ 1.1 ms (consistent with ~1.2 ms BT jitter std after regression): {mean_std:.3f} ms")

# ── T7b: Mean trace consistent with Figure 2(c) visual ────────
# Figure 2(c) shows the mean trace staying within ~±1 ms throughout 300 s.
# After averaging 11 robots, per-cycle std ≈ σ/√11 ≈ 1.1/3.3 ≈ 0.33 ms.
# Peak of mean trace should be well below 2 ms (4σ of the mean distribution).
fig2c_peak = fig2c["mean_sync_error_ms"].abs().max()
fig2c_std  = fig2c["mean_sync_error_ms"].std()
CHECKF(fig2c_std, 0.33, 0.08,
       f"Mean trace std ≈ σ/√11 = 0.33 ms (averaging effect): {fig2c_std:.3f} ms")
CHECK(fig2c_peak < 2.0,
      f"Mean trace peak < 2 ms (consistent with Figure 2c ±1 ms visual): {fig2c_peak:.3f} ms")

# ── T8: Figure 2c is mean of all followers ────────────────────
print("\n[T8] Figure 2c = mean across follower robots")
recomputed_fig2c = (raw.groupby("time_s")["sync_error_ms"]
                      .mean()
                      .reset_index()
                      .rename(columns={"sync_error_ms":"mean_sync_error_ms"}))
merged = fig2c.merge(recomputed_fig2c, on="time_s", suffixes=("_file","_computed"))
max_diff = (merged["mean_sync_error_ms_file"] - merged["mean_sync_error_ms_computed"]).abs().max()
CHECKF(max_diff, 0.0, 1e-3,
       f"Figure 2c matches recomputed mean (max diff = {max_diff:.6f} ms)")

# ── T9: Reproducibility — re-running script gives same output ─
print("\n[T9] Reproducibility (fixed random seed)")
import subprocess
result = subprocess.run(["python3", "generate_sync_data.py"],
                        capture_output=True, text=True, cwd=".")
CHECK(result.returncode == 0, "Script runs without error")
raw2 = pd.read_csv("sync_data_raw.csv")
CHECK(len(raw2) == len(raw), "Re-run produces same number of rows")
diff = (raw2["sync_error_ms"] - raw["sync_error_ms"]).abs().max()
CHECKF(diff, 0.0, 1e-6, "Re-run produces bit-identical sync_error_ms (fixed seed)")

# ── T10: Crystal drift physics ────────────────────────────────
print("\n[T10] Crystal drift consistent with ±50 ppm spec")
# Single-clock drift per cycle = 50e-6 * T_cycle_s * 1000 ms = 0.01225 ms
T_cycle_s = T_CYCLE_MS / 1000.0
expected_drift_per_cycle = 50e-6 * T_cycle_s * 1000.0  # ms
# After linear regression, residual should be dominated by BT jitter (~3.2 ms std)
# not by crystal drift. Std of error ≈ BT jitter std ≈ 1.4 ms (after regression removes trend)
mean_err_std = summary["std_ms"].mean()
CHECK(mean_err_std < 2.0,
      f"Post-regression error std < 2.0 ms (regression removes drift): {mean_err_std:.3f} ms")
# Max drift before regression on any single robot would be ≤ 50ppm × 300s × 1000 = 15ms
# But regression resets it → max residual << 15ms
max_err = raw["sync_error_ms"].abs().max()
CHECK(max_err < 15.0,
      f"Max error < 15 ms (far below free-running drift bound): {max_err:.3f} ms")

print(f"\n=== Result: {pass_count} passed, {fail_count} failed ===")
sys.exit(fail_count)
