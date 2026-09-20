#!/usr/bin/env python3
"""
Análisis metrológico de las mediciones del VEML7700.

Calcula, para DOS magnitudes:
    1) Iluminancia      E   [lux]
    2) Luminancia       L = E / OMEGA   [cd/m²]   (OMEGA en estereorradianes)

Métricas por magnitud:
    Sensibilidad, R², Linealidad (%), Histéresis (%), MAE, Error relativo medio (%),
    RMSE, Repetibilidad (%CV), Incertidumbre Tipo A / Tipo B / combinada / expandida (95%),
    Resolución efectiva.

Entrada : luminancia.csv  (columnas: timestamp_ms,burst_id,sample_idx,brightness,lux,als_raw,white_raw)
Salida  : impresión en consola + archivo Excel 'analisis_metrologia.xlsx'

Requisitos: pip install pandas numpy openpyxl   (scipy es opcional, mejora el factor k)
"""
import re
import numpy as np
import pandas as pd

# scipy es opcional: si está, se usa la t de Student para k; si no, k=2.
try:
    from scipy import stats as _sps
    _HAVE_SCIPY = True
except ImportError:
    _HAVE_SCIPY = False

# ======================= CONFIGURACIÓN =======================
INFILE  = "luminancia.csv"
OUTFILE = "analisis_metrologia.xlsx"

OMEGA = 0.0849            # ángulo sólido [sr]  ->  L = E / OMEGA  [cd/m²]

# --- Incertidumbre Tipo B (no estadística) ---
# Resolución del sensor en lux/cuenta para tu configuración (GAIN_1_8 + IT_100MS).
# (Tabla Vishay: cámbiala si usas otra ganancia/tiempo de integración.)
RESOLUTION_LUX = 0.4608
# Exactitud relativa adicional del sensor (fracción del lectura). 0 si no la consideras.
ACCURACY_REL   = 0.0      # p.ej. 0.10 para ±10 % de tolerancia de catálogo

# --- Referencia para sensibilidad / linealidad / errores ---
# Si tienes un instrumento PATRÓN, pon aquí {nivel_brillo: valor_verdadero_en_lux}.
# Si lo dejas vacío, se usa el nivel de brillo como entrada nominal y la
# recta de calibración ajustada como referencia (los errores miden la no linealidad).
REFERENCE = {}            # ej.: {0: 0.0, 25: 11.8, 50: 38.4, 75: 70.2, 100: 95.1}

COVERAGE = 0.95           # nivel de confianza para la incertidumbre expandida
# =============================================================


def parse_brightness(s):
    """'50' -> (50.0, 'na');  'u50' -> (50.0, 'up');  'd50' -> (50.0, 'down')."""
    s = str(s).strip()
    m = re.match(r'^([uUdD]?)\s*([-+]?\d*\.?\d+)$', s)
    if not m:
        return (np.nan, 'na')
    d = {'u': 'up', 'd': 'down', '': 'na'}[m.group(1).lower()]
    return (float(m.group(2)), d)


def k_factor(u_a, u_b, n):
    """Factor de cobertura. Welch-Satterthwaite + t de Student si hay scipy; si no, k=2."""
    if not _HAVE_SCIPY or u_a <= 0 or n <= 1:
        return 2.0
    u_c = np.hypot(u_a, u_b)
    # gl efectivos: Tipo B se asume con gl -> infinito (su término se anula)
    veff = u_c**4 / (u_a**4 / (n - 1))
    return float(_sps.t.ppf(0.5 + COVERAGE / 2.0, veff))


def compute_metrics(df, value_col, label):
    """Calcula todas las métricas para una magnitud (columna value_col)."""
    out = {"magnitud": label}

    # ---- Grupos de réplica más finos disponibles (nivel + dirección) ----
    # Repetibilidad y Tipo A se evalúan dentro de condiciones homogéneas.
    g_rep = df.groupby(["brightness_val", "direction"])[value_col]
    rep_tbl = g_rep.agg(["count", "mean", "std"]).reset_index()
    rep_tbl["cv_%"]  = 100 * rep_tbl["std"] / rep_tbl["mean"]
    rep_tbl["u_A"]   = rep_tbl["std"] / np.sqrt(rep_tbl["count"])         # Tipo A
    rep_tbl["u_B"]   = np.hypot(RESOLUTION_LUX / (2*np.sqrt(3)),          # Tipo B (resolución)
                                ACCURACY_REL * rep_tbl["mean"] / np.sqrt(3))
    if label.startswith("Luminancia"):
        rep_tbl["u_B"] = np.hypot((RESOLUTION_LUX/OMEGA) / (2*np.sqrt(3)),
                                  ACCURACY_REL * rep_tbl["mean"] / np.sqrt(3))
    rep_tbl["u_c"]   = np.hypot(rep_tbl["u_A"], rep_tbl["u_B"])           # combinada
    rep_tbl["k"]     = [k_factor(a, b, n) for a, b, n in
                        zip(rep_tbl["u_A"], rep_tbl["u_B"], rep_tbl["count"])]
    rep_tbl["U_95"]  = rep_tbl["k"] * rep_tbl["u_c"]                      # expandida

    # ---- Curva de calibración: una media por nivel (promedia direcciones) ----
    lvl = df.groupby("brightness_val")[value_col].mean().reset_index()
    lvl = lvl.sort_values("brightness_val").reset_index(drop=True)
    y = lvl[value_col].to_numpy()

    if REFERENCE:                       # x = valor patrón verdadero
        x = lvl["brightness_val"].map(REFERENCE).to_numpy()
        ref = x.copy()                  # "verdad" para los errores
        x_name = "referencia"
    else:                               # x = brillo comandado; referencia = recta ajustada
        x = lvl["brightness_val"].to_numpy()
        ref = None
        x_name = "brillo comandado"

    # Regresión lineal y = a + b·x
    b, a = np.polyfit(x, y, 1)
    fit  = a + b * x
    ss_res = np.sum((y - fit)**2)
    ss_tot = np.sum((y - y.mean())**2)
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else np.nan

    fso = y.max() - y.min()             # fondo de escala (span de salida)

    # Referencia para los errores: patrón si existe, si no la recta ajustada
    truth = ref if ref is not None else fit
    err   = y - truth
    mae   = np.mean(np.abs(err))
    rmse  = np.sqrt(np.mean(err**2))
    # Error relativo: solo donde la referencia está bien por encima del piso de
    # ruido (|truth| > 1% del fondo de escala); cerca de cero el % no tiene sentido.
    mask_rel = np.abs(truth) > 0.01 * fso
    mre = np.mean(np.abs(err[mask_rel] / truth[mask_rel])) * 100 if mask_rel.any() else np.nan

    # Linealidad (best-fit-line): máxima desviación de la recta / fondo de escala
    linearity = 100 * np.max(np.abs(y - fit)) / fso if fso > 0 else np.nan

    # ---- Histéresis: subida vs bajada en niveles comunes ----
    hyst = np.nan
    piv = df.groupby(["brightness_val", "direction"])[value_col].mean().unstack("direction")
    if {"up", "down"}.issubset(piv.columns):
        common = piv[["up", "down"]].dropna()
        if len(common) and fso > 0:
            hyst = 100 * np.max(np.abs(common["up"] - common["down"])) / fso

    # ---- Repetibilidad y Tipo A agrupadas ----
    # El %CV no tiene sentido cuando la media está en el piso de ruido (p.ej. negro):
    # se promedia solo sobre grupos con relación señal/ruido > 3.
    snr_ok = rep_tbl["mean"] > 3 * rep_tbl["std"]
    cv_mean = np.nanmean(rep_tbl.loc[snr_ok, "cv_%"]) if snr_ok.any() else np.nan
    n_excl = int((~snr_ok).sum())
    # desviación combinada (pooled) y u_A representativa
    dof = rep_tbl["count"] - 1
    pooled_var = np.sum(dof * rep_tbl["std"]**2) / np.sum(dof)
    pooled_sd  = np.sqrt(pooled_var)
    u_a_typ = np.nanmedian(rep_tbl["u_A"])
    u_b_typ = np.nanmedian(rep_tbl["u_B"])
    u_c_typ = np.nanmedian(rep_tbl["u_c"])
    U_typ   = np.nanmedian(rep_tbl["U_95"])

    # ---- Resolución efectiva ----
    res_lsb = RESOLUTION_LUX / OMEGA if label.startswith("Luminancia") else RESOLUTION_LUX
    res_eff = max(res_lsb, pooled_sd)   # limitada por ruido o por cuantización

    out.update({
        "eje_x": x_name,
        "Sensibilidad (salida/entrada)": b,
        "R2": r2,
        "Linealidad (%)": linearity,
        "Histeresis (%)": hyst,
        "MAE": mae,
        "Error relativo medio (%)": mre,
        "RMSE": rmse,
        "Repetibilidad media (%CV)": cv_mean,
        "  (niveles excluidos del %CV)": n_excl,
        "Desv. estandar combinada (pooled)": pooled_sd,
        "Incert. Tipo A (tip.)": u_a_typ,
        "Incert. Tipo B (tip.)": u_b_typ,
        "Incert. combinada (tip.)": u_c_typ,
        "Incert. expandida 95% (tip.)": U_typ,
        "Resolucion nominal (LSB)": res_lsb,
        "Resolucion efectiva (ruido)": res_eff,
    })
    return out, rep_tbl


def main():
    df = pd.read_csv(INFILE, comment="#")
    df.columns = [c.strip() for c in df.columns]

    parsed = df["brightness"].map(parse_brightness)
    df["brightness_val"] = parsed.map(lambda t: t[0])
    df["direction"]      = parsed.map(lambda t: t[1])
    df = df.dropna(subset=["brightness_val", "lux"]).copy()

    # Magnitud 2: luminancia
    df["lum_cd_m2"] = df["lux"] / OMEGA

    resumen_rows, perlevel = [], {}
    for col, name in [("lux", "Iluminancia [lux]"), ("lum_cd_m2", "Luminancia [cd/m^2]")]:
        row, tbl = compute_metrics(df, col, name)
        resumen_rows.append(row)
        perlevel[name] = tbl

    resumen = pd.DataFrame(resumen_rows).set_index("magnitud").T

    # ---- Salida en consola ----
    pd.set_option("display.float_format", lambda v: f"{v:.5g}")
    print("\n================  RESUMEN METROLÓGICO  ================")
    print(f"OMEGA = {OMEGA} sr  |  scipy={'sí' if _HAVE_SCIPY else 'no (k=2)'}  |  "
          f"referencia patrón={'sí' if REFERENCE else 'no -> recta ajustada'}")
    print(resumen.to_string())
    for name, tbl in perlevel.items():
        print(f"\n--- Por nivel: {name} ---")
        print(tbl.to_string(index=False))

    # ---- Excel ----
    with pd.ExcelWriter(OUTFILE, engine="openpyxl") as xl:
        resumen.to_excel(xl, sheet_name="resumen")
        for name, tbl in perlevel.items():
            sheet = "porNivel_" + ("lux" if "lux" in name else "cd_m2")
            tbl.to_excel(xl, sheet_name=sheet, index=False)
    print(f"\nGuardado: {OUTFILE}")


if __name__ == "__main__":
    main()
