"""Plot the two separate ARD sources and the HORUS command; run at repo root."""
import json
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).resolve().parents[2]
folder = root / 'configs/reference_profiles'
a = json.loads((folder / 'ard_mean_aoa_2007.json').read_text())
c = json.loads((folder / 'ard_cfd_conditions_2016.json').read_text())
h = json.loads((folder / 'horus_aoa_2024.json').read_text())
fig, ax = plt.subplots(1, 2, figsize=(12, 4.5), layout='constrained')
ax[0].plot(np.array(a['altitude_m']) / 1000, a['simulator_aoa_magnitude_deg'], label='2007 mean curve, digitized')
ax[0].plot(np.array(c['altitude_m']) / 1000, c['aoa_magnitude_deg'], 'o--', label='2016 CFD table (separate)')
ax[0].invert_xaxis()
ax[0].set(xlabel='Reference altitude (km)', ylabel='AoA magnitude (deg)', title='ARD: two distinct source profiles')
ax[0].legend(fontsize=9)
ax[1].plot(h['time_since_entry_s'], h['aoa_deg'])
ax[1].set(xlabel='Time since reference entry (s)', ylabel='AoA (deg)', title='HORUS-2B: 2024 reference command', ylim=(10, 45))
for axis in ax:
    axis.grid(alpha=.3)
fig.savefig(Path(__file__).with_name('profiles.png'), dpi=150)
