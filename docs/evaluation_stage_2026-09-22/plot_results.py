"""Render static report figures from saved results; never run a mission search."""
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
data = json.loads((HERE / "results.json").read_text(encoding="utf-8"))
plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})


def save(fig, name):
    fig.savefig(HERE / (name + ".png"), dpi=180, bbox_inches="tight")
    fig.savefig(HERE / (name + ".pdf"), bbox_inches="tight")
    plt.close(fig)


rows = data["robustness"]
scales = [0, 1, 3]
fig, axes = plt.subplots(2, 2, figsize=(10, 8), layout="constrained")
for ax, enabled, key, title in [
    (axes[0, 0], False, "position_error_m", "Correction disabled: maximum position miss (m)"),
    (axes[0, 1], True, "position_error_m", "Correction enabled: maximum position miss (m)"),
    (axes[1, 0], True, "accepted", "Correction enabled: accepted cases / 2 seeds"),
    (axes[1, 1], True, "correction_delta_v_m_s", "Correction enabled: maximum added delta-V (m/s)"),
]:
    z = np.zeros((3, 3))
    for iy, initial in enumerate(scales):
        for ix, burn in enumerate(scales):
            group = [r for r in rows if r["initial_scale"] == initial and r["burn_scale"] == burn
                     and r["corrections_enabled"] == enabled]
            assert len(group) == 2
            z[iy, ix] = sum(r[key] for r in group) if key == "accepted" else max(r[key] for r in group)
    shown = np.log10(np.maximum(z, 1e-6)) if key == "position_error_m" else z
    bounds = {"vmin": -6, "vmax": 4.1} if key == "position_error_m" else {}
    im = ax.imshow(shown, origin="lower", cmap="viridis", aspect="auto", **bounds)
    for iy in range(3):
        for ix in range(3):
            ax.text(ix, iy, f"{z[iy, ix]:.3g}", ha="center", va="center", color="white",
                    bbox={"facecolor": "black", "alpha": .45, "edgecolor": "none", "pad": 2})
    ax.set(xticks=range(3), yticks=range(3), xticklabels=scales, yticklabels=scales,
           xlabel="Execution-error scale", ylabel="Initial-state-error scale", title=title)
    cb = fig.colorbar(im, ax=ax, label="log10(m)" if key == "position_error_m" else "count" if key == "accepted" else "m/s")
    if key == "accepted":
        cb.set_ticks([0, 1, 2])
fig.suptitle("Frozen Apollo nominal plan — perfect state knowledge\nSelected seeds 7 and 42; counts are not reliability estimates", fontsize=13)
save(fig, "orbital_robustness")

d = data["design"]
labels = [f"{r['preset'].replace('APOLLO7_PREFLIGHT_TRIM', 'Apollo').replace('HORUS_2B','HORUS')}\n{r['phase_deg']}° {r['mode']}" for r in d]
x = np.arange(len(d))
fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True, layout="constrained")
bottom = np.zeros(len(d))
for key, label, color in [("phase1_dv_m_s", "Phasing/homing", "#3366aa"),
                           ("phase2_dv_m_s", "Proximity", "#dd9933"),
                           ("deorbit_dv_m_s", "Deorbit", "#338866")]:
    values = np.array([r[key] if r[key] is not None else np.nan for r in d])
    axes[0].bar(x, values, bottom=bottom, label=label, color=color)
    bottom += values
axes[0].set(ylabel="Delivered delta-V (m/s)", title="Redesigned nominal cases: resources through entry interface")
axes[0].legend(ncol=3)
axes[0].set_ylim(0, np.nanmax(bottom) * 1.22)
axes[1].bar(x, [r["fuel_through_deorbit_kg"] or np.nan for r in d], color="#665599")
axes[1].set(ylabel="Consumed propellant (kg)", title="Resource requirement; vehicle-specific available propellant is not established")
axes[2].bar(x, [r["peak_control_force_N"] or np.nan for r in d], color="#887766")
axes[2].set(xticks=x, xticklabels=labels, ylabel="Peak controlled force (N)",
            title="Ideal final-approach actuator only; impulse peak forces are undefined")
fig.suptitle("Hypothetical integrated missions — no complete atmospheric descent validated", fontsize=13)
save(fig, "design_resources")

fig, axes = plt.subplots(2, 2, figsize=(10, 8), layout="constrained")
entries = data["entry"]
for ax, key, divisor, label in [(axes[0, 0], "endpoint_shift_m", 1000, "Endpoint shift from nominal (km)"),
                                (axes[0, 1], "peak_q_Pa", 1000, "Peak dynamic pressure (kPa)"),
                                (axes[1, 0], "peak_g", 1, "Peak acceleration (g)"),
                                (axes[1, 1], "peak_heat_W_m2", 1e6, "Peak surrogate heat flux (MW/m²)")]:
    for density, color in zip([.9, 1, 1.1], ["#3366aa", "#333333", "#cc7722"]):
        group = [r for r in entries if r["density_scale"] == density and r["ld_scale"] == 1]
        values = [(r[key] / divisor) if r[key] is not None else np.nan for r in group]
        ax.plot([r["fpa_deg"] for r in group], values, "o-", color=color, label=f"Density ×{density:g}")
    ax.set(xlabel="Prescribed air-relative entry FPA (deg)", ylabel=label, xticks=[-2.5, -2, -1.5])
    ax.grid(alpha=.25)
axes[0, 0].legend()
fig.suptitle("Standalone Apollo fixed-bank sensitivity — not the integrated entry state\nConstant 30° bank; no target/path acceptance or flight-validation claim", fontsize=13)
save(fig, "entry_sensitivity")

# Record source identity without modifying or bundling unrelated working changes.
files = list(ROOT.glob("*.m"))
for package in ROOT.glob("+*"):
    if package.is_dir():
        files.extend(package.rglob("*.m"))
files.extend((ROOT / "configs" / "reference_profiles").glob("*.json"))
files.extend((ROOT / "validation").glob("*.m"))
files.extend([HERE / "evaluate_simulator.m", HERE / "finalize_evaluation.m", HERE / "plot_results.py"])
manifest = {str(p.relative_to(ROOT)).replace("\\", "/"): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(set(files))}
(HERE / "source_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
print(f"Created 3 PNG/PDF figure pairs and SHA-256 manifest for {len(manifest)} files.")
