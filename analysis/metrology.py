"""
metrology.py
============

Re-analysis of the VEML7700 characterization runs, straight from the raw
captures published in `data/`.

What it adds to the original course analysis (`analisis_metrologia.py`):

  1. **Cleans the capture.** Three samples in `screen_sweep_veml7700.csv` report
     346,033.875 lux while their raw ALS counts are 9, 46 and 73 — i.e. the
     first reading of a burst returned the library's overflow value instead of a
     real conversion. Left in, they move the mean of the 40 % level from 2.8 lux
     to 5,770 lux. They are dropped here and the count is reported.

  2. **Recovers the sweep direction.** The screen capture is an up sweep
     (bursts 1-11, 0 % -> 100 %) followed by a down sweep (bursts 12-22), but the
     brightness labels carry no `u`/`d` prefix, so hysteresis could not be
     computed from the file as written. Direction is reconstructed from the
     burst index.

  3. **Fits the screen response three ways** (linear, quadratic, power law) and
     estimates the exponent gamma by two routes — a direct non-linear fit and
     the log-log linearization used in the report — to show how far apart they
     land on this data set.

  4. **Cross-calibrates the sensor against the reference luxmeter**, which
     measured the same brightness sweep, and asks whether the disagreement is a
     gain error (fixable with one calibration point) or a shape error.

Outputs: `results/*.csv` and `docs/img/*.png`.

Usage:
    python analysis/metrology.py
"""
from __future__ import annotations

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.optimize import curve_fit

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data"
RESULTS = ROOT / "results"
FIGS = ROOT / "docs" / "img"

OMEGA = 0.0849          # solid angle of the sensor aperture [sr]; L = E / OMEGA
RESOLUTION_LUX = 0.4608  # VEML7700 lux per count at GAIN 1/8 + IT 100 ms
OVERFLOW_LUX = 1e5      # anything above this is the library's overflow marker
N_UP_BURSTS = 11        # bursts 1..11 = 0 % -> 100 %, 12..22 = 100 % -> 0 %


# --------------------------------------------------------------- model shapes
def power_law(x, L0, k, gamma):
    return L0 + k * np.power(x, gamma)


def r_squared(y, fit):
    y = np.asarray(y, float)
    return 1.0 - np.sum((y - fit) ** 2) / np.sum((y - y.mean()) ** 2)


# ------------------------------------------------------------------- loading
def load_screen_sweep():
    """Raw VEML7700 burst capture of the computer-screen brightness sweep."""
    df = pd.read_csv(DATA / "screen_sweep_veml7700.csv", comment="#")
    df.columns = [c.strip() for c in df.columns]
    bad = df["lux"] > OVERFLOW_LUX
    dropped = df.loc[bad, ["burst_id", "sample_idx", "brightness", "lux", "als_raw"]]
    df = df.loc[~bad].copy()
    df["direction"] = np.where(df["burst_id"] <= N_UP_BURSTS, "up", "down")
    df["luminance_cd_m2"] = df["lux"] / OMEGA
    return df, dropped


def load_luxmeter(name):
    d = pd.read_csv(DATA / name)
    return d.groupby("brightness_pct")["lux_luxmeter"].mean()


# --------------------------------------------------------------- metrology
def metrology(levels_mean, levels_std, levels_n, brightness, label):
    """Sensitivity, linearity, repeatability and uncertainty for one magnitude."""
    x = np.asarray(brightness, float)
    y = np.asarray(levels_mean, float)
    slope, intercept = np.polyfit(x, y, 1)
    fit = intercept + slope * x
    span = y.max() - y.min()

    sd = np.asarray(levels_std, float)
    n = np.asarray(levels_n, float)
    snr_ok = y > 3 * np.where(sd > 0, sd, np.inf)
    cv = 100 * sd / np.where(y > 0, y, np.nan)

    u_a = np.nanmedian(sd / np.sqrt(n))
    u_b = RESOLUTION_LUX / OMEGA / (2 * np.sqrt(3)) if "cd" in label else RESOLUTION_LUX / (2 * np.sqrt(3))
    u_c = float(np.hypot(u_a, u_b))

    return {
        "magnitude": label,
        "sensitivity_per_%": slope,
        "R2_linear": r_squared(y, fit),
        "non_linearity_%FS": 100 * np.max(np.abs(y - fit)) / span,
        "repeatability_%CV": float(np.nanmean(cv[snr_ok])),
        "u_A": u_a,
        "u_B": u_b,
        "u_combined": u_c,
        "U95_k2": 2 * u_c,
        "effective_resolution": max(
            RESOLUTION_LUX / OMEGA if "cd" in label else RESOLUTION_LUX,
            float(np.sqrt(np.sum((n - 1) * sd ** 2) / np.sum(n - 1))),
        ),
    }


def hysteresis(df, col):
    piv = df.groupby(["brightness", "direction"])[col].mean().unstack("direction")
    common = piv[["up", "down"]].dropna()
    span = piv.max().max() - piv.min().min()
    diff = (common["up"] - common["down"]).abs()
    return 100 * diff.max() / span, 100 * diff.mean() / span, common


# --------------------------------------------------------------------- main
def main():
    RESULTS.mkdir(exist_ok=True)
    FIGS.mkdir(parents=True, exist_ok=True)

    df, dropped = load_screen_sweep()
    print(f"dropped {len(dropped)} overflow samples out of "
          f"{len(df) + len(dropped)} ({100*len(dropped)/(len(df)+len(dropped)):.1f} %)")
    print(dropped.to_string(index=False))
    dropped.to_csv(RESULTS / "discarded_samples.csv", index=False)

    g = df.groupby("brightness")["luminance_cd_m2"]
    lvl = g.agg(["mean", "std", "count"])
    B = lvl.index.values.astype(float)
    L = lvl["mean"].values

    # ---- 1. shape of the screen response ---------------------------------
    lin = np.polyfit(B, L, 1)
    quad = np.polyfit(B, L, 2)
    p_free, cov_free = curve_fit(power_law, B, L, p0=[L[0], 1e-5, 3.0], maxfev=400_000)
    p_fix, cov_fix = curve_fit(lambda x, k, gm: power_law(x, L[0], k, gm), B, L,
                               p0=[1e-3, 2.0], maxfev=400_000)
    pos = B > 0
    gamma_log, log_k = np.polyfit(np.log(B[pos]), np.log(L[pos] - L[0]), 1)

    shapes = pd.DataFrame([
        {"model": "linear  L = a·B + b", "R2": r_squared(L, np.polyval(lin, B)), "gamma": np.nan},
        {"model": "quadratic  L = a·B² + b·B + c", "R2": r_squared(L, np.polyval(quad, B)), "gamma": np.nan},
        {"model": "power, L0 free", "R2": r_squared(L, power_law(B, *p_free)),
         "gamma": p_free[2], "gamma_sigma": np.sqrt(cov_free[2, 2])},
        {"model": "power, L0 = L(0 %)", "R2": r_squared(L, power_law(B, L[0], *p_fix)),
         "gamma": p_fix[1], "gamma_sigma": np.sqrt(cov_fix[1, 1])},
        {"model": "log-log linearization", "R2": np.nan, "gamma": gamma_log},
    ])
    print("\n--- screen response, VEML7700 ---")
    print(shapes.to_string(index=False))
    shapes.to_csv(RESULTS / "screen_response_models.csv", index=False)

    # ---- 2. cross-calibration against the reference luxmeter -------------
    ref = load_luxmeter("reference_luxmeter_screen.csv")
    veml = df.groupby("brightness")["lux"].mean()
    common = veml.index.intersection(ref.index)
    v, r = veml.loc[common].values, ref.loc[common].values
    m = common.values >= 10                      # 0 % is at the noise floor
    log_slope, log_int = np.polyfit(np.log(r[m]), np.log(v[m]), 1)
    ratio = v[m] / r[m]
    k_cal = float(np.median(ratio))
    residual = (v[m] / k_cal - r[m]) / r[m] * 100

    cross = pd.DataFrame({"brightness_%": common[m], "veml_lux": v[m], "luxmeter_lux": r[m],
                          "ratio": ratio, "residual_after_1pt_cal_%": residual})
    print("\n--- VEML7700 vs reference luxmeter (same screen sweep) ---")
    print(cross.to_string(index=False))
    print(f"log-log slope = {log_slope:.4f} (1.0 = pure gain error)")
    print(f"gain factor   = {k_cal:.3f}  (spread {100*ratio.std()/ratio.mean():.1f} %)")
    print(f"after one-point calibration: mean |error| {np.abs(residual).mean():.2f} %, "
          f"max {np.abs(residual).max():.2f} %")
    cross.to_csv(RESULTS / "cross_calibration.csv", index=False)

    # ---- 3. metrological parameters --------------------------------------
    rows = [metrology(lvl["mean"], lvl["std"], lvl["count"], B, "Luminance [cd/m^2]")]
    lux_lvl = df.groupby("brightness")["lux"].agg(["mean", "std", "count"])
    rows.append(metrology(lux_lvl["mean"], lux_lvl["std"], lux_lvl["count"], B, "Illuminance [lux]"))
    params = pd.DataFrame(rows).set_index("magnitude").T
    h_max, h_mean, updown = hysteresis(df, "luminance_cd_m2")
    params.loc["hysteresis_max_%FS"] = h_max
    params.loc["hysteresis_mean_%FS"] = h_mean
    print("\n--- metrological parameters (screen, VEML7700) ---")
    print(params.to_string())
    params.to_csv(RESULTS / "metrology_parameters.csv")

    # ---- 4. figures -------------------------------------------------------
    bb = np.linspace(0, 100, 300)

    # screen response and the three fitted models
    plt.figure()
    plt.errorbar(B, L, yerr=lvl["std"], fmt="o", label="Measured (60 samples/level)")
    plt.plot(bb, np.polyval(lin, bb),
             label=f"Linear, R² = {r_squared(L, np.polyval(lin, B)):.3f}")
    plt.plot(bb, np.polyval(quad, bb),
             label=f"Quadratic, R² = {r_squared(L, np.polyval(quad, B)):.3f}")
    plt.plot(bb, power_law(bb, *p_free),
             label=f"Power law, γ = {p_free[2]:.2f}, R² = {r_squared(L, power_law(B, *p_free)):.3f}")
    plt.xlabel("Screen brightness setting (%)")
    plt.ylabel("Luminance (cd/m²)")
    plt.title("Screen luminance vs brightness setting (VEML7700)")
    plt.legend()
    plt.savefig(FIGS / "characterization.png")

    # hysteresis between the up and down sweeps
    plt.figure()
    plt.plot(updown.index, updown["up"], "o-", label="0 % → 100 %")
    plt.plot(updown.index, updown["down"], "s-", label="100 % → 0 %")
    plt.xlabel("Screen brightness setting (%)")
    plt.ylabel("Luminance (cd/m²)")
    plt.title(f"Hysteresis: {h_max:.1f} % FS max, {h_mean:.1f} % FS mean")
    plt.legend()
    plt.savefig(FIGS / "hysteresis.png")

    # cross-calibration against the reference luxmeter
    rr = np.linspace(r[m].min(), r[m].max(), 100)
    plt.figure()
    plt.loglog(r[m], v[m], "o", label="Measured levels")
    plt.loglog(rr, k_cal * rr, label=f"Gain × {k_cal:.2f} (median)")
    plt.loglog(rr, rr, "k--", label="1:1")
    plt.xlabel("Reference luxmeter (lux)")
    plt.ylabel("VEML7700 (lux)")
    plt.title(f"VEML7700 vs luxmeter, log-log slope {log_slope:.3f}")
    plt.legend()
    plt.savefig(FIGS / "cross_calibration.png")

    # LED reference sweep: the linear control case
    led = pd.read_csv(DATA / "reference_luxmeter_led.csv")
    up = led.iloc[:11]
    down = led.iloc[11:]
    sl, ic = np.polyfit(led["brightness_pct"], led["lux_luxmeter"], 1)
    plt.figure()
    plt.plot(up["brightness_pct"], up["lux_luxmeter"], "o", label="0 % → 100 %")
    plt.plot(down["brightness_pct"], down["lux_luxmeter"], "s", label="100 % → 0 %")
    plt.plot(bb, ic + sl * bb,
             label=f"Linear fit, R² = {r_squared(led['lux_luxmeter'], ic + sl*led['brightness_pct']):.4f}")
    plt.xlabel("LED PWM duty (%)")
    plt.ylabel("Illuminance (lux)")
    plt.title("Reference LED source (luxmeter)")
    plt.legend()
    plt.savefig(FIGS / "led_reference.png")

    print(f"\nwrote {RESULTS}/*.csv and four figures in {FIGS}/")


if __name__ == "__main__":
    main()
