"""Bounded existing optimizer comparison; no mission archive writes.
Run from repo root. Each isolated child has a 90 s external wall-time limit.
"""
import json
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
FOLDER = Path(__file__).parent
if len(sys.argv) == 1:
    for phase in (90, 60):
        with (FOLDER / f'python_{phase}.log').open('w') as log:
            try:
                subprocess.run([sys.executable, __file__, str(phase)], cwd=ROOT,
                               stdout=log, stderr=subprocess.STDOUT, timeout=90, check=True)
            except subprocess.TimeoutExpired:
                (FOLDER / f'python_{phase}.json').write_text(json.dumps({'status': 'WALL_TIME_LIMIT', 'limit_s': 90}))
    sys.exit()

from J2PolarHohmannShooting import minimize_delta_v_on_zero_distance_manifold, make_scenario_config, make_maneuver_config
from mission_io import to_jsonable
phase = int(sys.argv[1])
scenario = make_scenario_config(initial_phase_angle=phase)
maneuver = make_maneuver_config(initial_mass_kg=4800)
start = time.perf_counter()
out = minimize_delta_v_on_zero_distance_manifold(
    objective_mode='two_impulse_total', feasibility_tolerance_km=.0005,
    stage1_max_iter=3, stage2_max_iter=2, stage1_verbose=False, stage2_verbose=False,
    scenario_config=scenario, maneuver_config=maneuver, ftol=1e-9)
record = {k: out[k] for k in ('success', 'message', 'optimized_parameters',
    'final_distance_km', 'terminal_relative_speed_m_s', 'total_two_impulse_delta_v_m_s')}
record.update(runtime_s=time.perf_counter()-start, scenario=scenario, maneuver=maneuver,
    limits={'stage1_max_iter': 3, 'stage2_max_iter': 2, 'wall_time_s': 90, 'ftol': 1e-9},
    scope='COMPARISON_ONLY_NOT_COMPATIBILITY_QUALIFIED_ARCHIVE')
(FOLDER / f'python_{phase}.json').write_text(json.dumps(to_jsonable(record), indent=2))
print(record)
