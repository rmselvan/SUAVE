## Hawkai — Entry point
## Usage: python Hawkai/run.py

import argparse
import matplotlib.pyplot as plt

import SUAVE
from SUAVE.Core import Units
from SUAVE.Input_Output.OpenVSP import write

from Hawkai.vehicle   import vehicle_setup, configs_setup
from Hawkai.analyses  import analyses_setup
from Hawkai.mission   import mission_setup, missions_setup
from Hawkai.plots     import make_plots

# ── Mission selection ─────────────────────────────────────────────────────────
# Edit this to change the default. Overridden by --mission on the command line.
MISSION = 'case2'   # choices: 'base', 'case1', 'case2', 'case3', 'case4', 'all'

MISSION_LABELS = {
    'base':  'Base Mission',
    'case1': 'Case 1 — Linear Asset Inspection (300 mi @ 60 mph)',
    'case2': 'Case 2 — Part Delivery (2 × 150 mi)',
    'case3': 'Case 3 — Dual Inspection and Delivery (300 mi @ 60 mph)',
    'case4': 'Case 4 — Spot Inspection (150 mi + 5 min hover + 150 mi)',
}

def _parse_args():
    parser = argparse.ArgumentParser(description='Hawkai mission runner')
    parser.add_argument('--mission', default=None,
                        choices=list(MISSION_LABELS) + ['all'],
                        help='Mission key to evaluate (default: MISSION variable)')
    return parser.parse_args()


def main():
    args = _parse_args()
    mission_key = args.mission if args.mission is not None else MISSION

    # ── Build vehicle & configs ───────────────────────────────────────────────
    vehicle = vehicle_setup()
    wing    = vehicle.wings.main_wing

    print("spans.projected (m)  :", wing.spans.projected)
    print("spans.projected (ft) :", wing.spans.projected / Units.feet)
    print("symmetric            :", wing.symmetric)
    print("chords.root (m)      :", wing.chords.root)
    print("chords.tip (m)       :", wing.chords.tip)
    for seg in wing.Segments:
        print(f"  Segment {seg.tag}: pct_span={seg.percent_span_location}, "
              f"root_chord_pct={seg.root_chord_percent}")

    # write(vehicle, 'Hawkai')

    configs = configs_setup(vehicle)

    # ── Analyses ──────────────────────────────────────────────────────────────
    configs_analyses  = analyses_setup(configs)
    mission           = mission_setup(configs_analyses, vehicle)
    missions_analyses = missions_setup(mission, configs_analyses, vehicle)

    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = missions_analyses

    configs.finalize()
    analyses.finalize()

    # ── Evaluate ──────────────────────────────────────────────────────────────
    keys_to_run = list(MISSION_LABELS) if mission_key == 'all' else [mission_key]

    for key in keys_to_run:
        label = MISSION_LABELS[key]
        print(f'\n=== Evaluating: {label} ===')
        results = getattr(analyses.missions, key).evaluate()
        make_plots(results, title=label)

    plt.show()


if __name__ == '__main__':
    main()
