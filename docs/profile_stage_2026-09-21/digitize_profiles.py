"""Reproduce digitization from pdftoppm -scale-to 2200 PNGs; no PDF redistribution.
Usage: python digitize_profiles.py ARD_PAGE12.png HORUS_PAGE16.png REPO_ROOT
Pixel calibration is specific to the supplied, SHA-identified source editions.
"""
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image

ard_path, horus_path, root = map(Path, sys.argv[1:])
folder = root / 'configs/reference_profiles'
ard = np.array(Image.open(ard_path).convert('RGB'))
assert ard.shape[:2] == (2200, 1555)
red = (ard[:, :, 0] > 170) & (ard[:, :, 1] < 100) & (ard[:, :, 2] < 100)
# Exclude the red legend sample, not the physical red mean curve.
red[1350:1420, 848:1075] = False
time = np.arange(4880, 5261, 1)
xp = 489 + (time - 4900) * (1107 - 489) / 350
signed = []
for x in xp:
    x = int(round(x))
    y = np.where(red[1334:1818, x])[0] + 1334
    assert len(y), (x, 'missing mean-curve pixel')
    signed.append(round(-15 - (float(np.median(y)) - 1333) * 10 / 485, 2))
alt_x = [452, 485, 523, 572, 689, 748, 796, 855, 916, 969, 1017, 1058, 1106, 1154]
alt = np.interp(xp, alt_x, np.arange(80, 14, -5))
mach_x = [479, 564, 674, 775, 826, 861, 937, 959, 1048, 1097]
mach = np.interp(xp, mach_x, [26, 24, 20, 15, 12, 10, 6, 5, 2, 1], left=np.nan, right=np.nan)
payload = dict(
    source='Tran et al. 2007 RTO-EN-AVT-130, p10-12 Fig15 red Mean',
    classification='GRAPH_DIGITIZED_REFERENCE_FLIGHT_HISTORY_NOT_GUIDANCE',
    source_time_s=time.tolist(), source_signed_aoa_deg=signed,
    altitude_m=np.round(alt * 1000).astype(int).tolist(),
    mach=[None if not np.isfinite(v) else round(float(v), 3) for v in mach],
    simulator_aoa_magnitude_deg=[-v for v in signed],
    estimated_reading_uncertainty=dict(aoa_deg=0.15, time_s=1.5, altitude_m=750, mach=0.3),
    uncertainty_note='Engineering digitization estimates, not statistical flight-error bounds; label interpolation adds uncertainty.',
    calibration=dict(time_pixels=[489, 1107], time_s=[4900, 5250], aoa_pixels=[1333, 1818],
                     aoa_deg=[-15, -25], altitude_tick_pixels=alt_x, mach_tick_pixels=mach_x),
    interpolation='PIECEWISE_LINEAR; altitude and Mach axes use separate nonuniform tick maps',
)
(folder / 'ard_mean_aoa_2007.json').write_text(json.dumps(payload, indent=2) + '\n')
horus = np.array(Image.open(horus_path).convert('RGB'))
assert horus.shape[:2] == (2200, 1452)
blue = (horus[:, :, 2] > 150) & (horus[:, :, 0] < 100) & (horus[:, :, 1] < 120)
htime = np.arange(0, 1251, 25)
values = []
for t in htime:
    x = round(352 + 811 * t / 1400)
    y = np.where(blue[625:680, x])[0] + 625
    assert len(y)
    values.append(round((690.5 - float(np.median(y))) / 1.345, 2))
# Published plateau is explicitly 40 degrees; preserve the textual exact value.
values = [40.0 if t <= 900 else a for t, a in zip(htime, values)]
hpayload = dict(source='Mooij Re-entry Systems 2024 Appendix B p1498 Fig B.7 lower plot',
                classification='GRAPH_DIGITIZED_REFERENCE_COMMAND_NOT_TRIM_MODEL',
                time_since_entry_s=htime.tolist(), aoa_deg=values,
                estimated_reading_uncertainty=dict(aoa_deg=0.8, time_s=5),
                calibration=dict(time_pixels=[352, 1163], time_s=[0, 1400],
                                 angle_pixels=[556, 825], angle_deg=[100, -100]),
                note='Initial 40 deg plateau pinned to p1497 text; decline digitized. No bank guidance imported.')
(folder / 'horus_aoa_2024.json').write_text(json.dumps(hpayload, indent=2) + '\n')
print('Saved ARD and HORUS digitizations:', len(time), len(htime), 'points')
