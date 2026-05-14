# Hawkai Project History

## 2026-03-16

### Refactored `Hawkai.py` → `Hawkai/clsHawkai.py`

Created the `Hawkai/` package by extracting all functions from `Hawkai.py` into a class called `clsHawkai` in `Hawkai/clsHawkai.py`.

**Files created:**
- `__init__.py` — empty package marker
- `clsHawkai.py` — all 5 functions from `Hawkai.py` moved into `clsHawkai` as methods

**Methods in `clsHawkai`:**
- `main(self)` — internal calls updated to `self.setup_vehicle()`, `self.setup_analyses()`, etc.
- `make_plots(self, results)`
- `setup_analyses(self, vehicle)`
- `setup_vehicle(self)` — path fix applied using `os.path.dirname(__file__)` to correctly resolve airfoil file paths after the file moved into the subdirectory
- `setup_mission(self, vehicle, analyses)`

**Design decisions applied:**
- Internal cross-method calls inside `main()` use the OO pattern (`self.setup_vehicle()`, etc.) — no bare function calls
- `__name__ == "__main__"` guard preserved at the bottom of `clsHawkai.py`, updated to instantiate the class: `hawk = clsHawkai(); hawk.main(); plt.show()`
- Airfoil path construction fixed: replaced `.split('Hawkai.py')[0]` with `os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')` to correctly navigate one level up from the new file location

---

## 2026-03-16 — Modularize `clsHawkai.py` → Strategy 2 multi-file package

**Timestamp:** 2026-03-16T00:00:00 (UTC)

### Summary

Replaced the single 593-line `clsHawkai.py` monolith with a clean 7-file package following SUAVE's own reference architecture (`test_Stopped_Rotor.py` / `Stopped_Rotor.py`). Every concern now lives in its own module. All 5 original methods were de-classed (no more `self.`), and all numeric literals were extracted into named constants with `Units` in a dedicated `parameters.py`.

**Deleted:** `Hawkai/clsHawkai.py`

### What changed and where

| File | Action | Description |
|------|--------|-------------|
| `Hawkai/parameters.py` | Created | All numeric constants (masses, spans, speeds, voltages, rotor geometry, mission parameters) with `Units`. Single place to change any design value. |
| `Hawkai/vehicle.py` | Created | `vehicle_setup()` — full vehicle geometry and propulsion network. `configs_setup(vehicle)` — stub Config.Container with base config. Imports all constants from `parameters.py`. `_AIRFOIL_DIR` path constant defined at module level. |
| `Hawkai/analyses.py` | Created | `base_analysis(vehicle)` — builds a `SUAVE.Analyses.Vehicle` (weights, aero, energy, planet, atmosphere). `analyses_setup(configs)` — iterates over all configs and builds one analysis per config, returning an `Analysis.Container`. |
| `Hawkai/mission.py` | Created | `mission_setup(configs_analyses, vehicle)` — builds the 5-segment lift+cruise mission (hover climb, wing climb, cruise, wing descent, hover descent). `missions_setup(base_mission)` — stub Mission.Container with base mission. All segment parameters imported from `parameters.py`. |
| `Hawkai/plots.py` | Created | `make_plots(results, line_style)` — wraps the four standard SUAVE performance plot calls. |
| `Hawkai/run.py` | Created | Thin `main()` orchestrator: vehicle → configs → analyses → mission → evaluate → plots. `if __name__ == '__main__': main()` guard. Entry point for `python3 Hawkai/run.py`. |
| `Hawkai/__init__.py` | Updated | Now exposes `vehicle_setup` as the package's public API (`from Hawkai.vehicle import vehicle_setup`). Previously empty. |
| `Hawkai/clsHawkai.py` | Deleted | All logic redistributed into the files above. |

### Design decisions

- **`parameters.py` as single source of truth** — `LIFT_ROTOR_DESIGN_THRUST` is derived from `TAKEOFF_MASS / N_LIFT_ROTORS` so the two stay consistent automatically.
- **SUAVE-native call pattern** — `analyses_setup` iterates over `configs.items()`, `mission_setup` takes `configs_analyses` and calls `segment.analyses.extend(configs_analyses.base)`, matching `test_Stopped_Rotor.py` exactly.
- **SUAVE typo preserved** — `gearbox_effiicency` (double-i) matches the SUAVE Motor attribute name to avoid a silent no-op assignment.
- **Stub extension points** — `configs_setup()` and `missions_setup()` are thin stubs that make it trivial to add a second config or mission profile without touching any other file.

---

## 2026-03-16 — Four Mission Cases + `add_hover` Boilerplate

**Timestamp:** 2026-03-16T12:00:00 (UTC)

### Summary

Added four named mission case functions to `Hawkai/mission.py`, each representing a distinct operational scenario. Extended the existing boilerplate segment functions with optional keyword arguments to support per-mission overrides, and added a new `add_hover` boilerplate for stationary hover segments.

### What changed and where

| File | Action | Description |
|------|--------|-------------|
| `Hawkai/parameters.py` | Updated | Added `CRUISE_DISTANCE_SHORT` (150 mi), `CRUISE_DISTANCE_LONG` (300 mi), `CRUISE_SPEED_60MPH` (60 mph), and `HOVER_DURATION` (5 min) constants. |
| `Hawkai/mission.py` | Updated | Extended all five boilerplate functions with optional `tag=` kwarg for unique segment naming in multi-leg missions. Extended `add_cruise` with optional `distance=` and `air_speed=` overrides. Added `add_hover` boilerplate using `Segments.Hover.Hover` with `altitude` and `time` inputs. Added `mission_case1`–`mission_case4` functions. Updated `missions_setup` to accept `configs_analyses` and `vehicle` and populate all four cases. |
| `run.py` | Updated | Updated `missions_setup` call to pass `configs_analyses` and `vehicle`. |

### Mission cases

| Case | Tag | Profile |
|------|-----|---------|
| Case 1 | `case1_asset_inspection` | hover climb → wing climb → cruise 300 mi @ 60 mph → wing descent → hover descent |
| Case 2 | `case2_part_delivery` | Two full legs, each: hover climb → wing climb → cruise 150 mi @ 60 mph → wing descent → hover descent |
| Case 3 | `case3_dual_inspection_delivery` | Identical flight profile to Case 1; separate function for distinct operational context |
| Case 4 | `case4_spot_inspection` | hover climb → wing climb → cruise 150 mi → hover 5 min → cruise 150 mi → wing descent → hover descent |

### Design decisions

- **Optional `tag=` on all boilerplate functions** — prevents SUAVE `Data` container key collisions in multi-leg missions (Cases 2, 4) where the same segment type appears more than once.
- **`add_cruise` overrides** — `distance=` and `air_speed=` fall back to `P.CRUISE_DISTANCE` / `P.CRUISE_SPEED` when not supplied, preserving backward compatibility with the original `mission_setup`.
- **`add_hover` uses `add_lift_unknowns_and_residuals_to_segment`** — consistent with `add_hover_climb` and `add_hover_descent` since all three are lift-rotor-only operations.
- **Battery reset on Case 2 leg 2** — `add_hover_climb` initialises battery to `INITIAL_BATTERY_SOC` at the start of each leg, modelling a recharge between deliveries.

---

## 2026-03-18 — Mission Selector for `run.py`

**Timestamp:** 2026-03-18T00:00:00 (UTC)

### Summary

Extended `run.py` with a mission selector so the user can run any of the five missions (`base`, `case1`–`case4`) or all of them in sequence, without modifying `mission.py`. A `MISSION` constant at the top of the file sets the default; a `--mission` CLI argument overrides it at runtime. Plot windows generated in multi-mission runs are individually titled with the mission label.

### What changed and where

| File | Action | Description |
|------|--------|-------------|
| `run.py` | Updated | Added `import argparse`. Added module-level `MISSION = 'base'` constant and `MISSION_LABELS` dict mapping each key to a human-readable description. Added `_parse_args()` with a `--mission` argument (choices: `base`, `case1`–`case4`, `all`). `main()` now resolves `mission_key` from CLI or the `MISSION` constant, then loops over `keys_to_run` using `getattr(analyses.missions, key).evaluate()` for dynamic dispatch. `plt.show()` is called once after all evaluations complete. |
| `Hawkai/plots.py` | Updated | Added `import matplotlib.pyplot as plt`. Extended `make_plots` signature with `title=None`. When `title` is provided, captures the set of new figure numbers created during the call and applies a `suptitle` to each, prepending any existing suptitle. Fully backward-compatible — existing `make_plots(results)` calls are unaffected. |

### Usage

| Intent | Command |
|---|---|
| Run base mission (default) | `python run.py` |
| Override default in file | Set `MISSION = 'case4'` at top of `run.py` |
| Run a specific mission (CLI) | `python run.py --mission case1` |
| Run all missions | `python run.py --mission all` |

### Design decisions

- **`MISSION` constant + `--mission` flag** — the constant provides a persistent default that survives across runs; the CLI flag enables one-off overrides without editing the file.
- **`getattr(analyses.missions, key)`** — idiomatic way to index a SUAVE `Data`/`Container` object by string key, avoiding a brittle `if/elif` chain.
- **`plt.show()` deferred to end of loop** — all figure windows appear together after all evaluations complete rather than blocking after each mission.
- **`figs_before` / `figs_after` diff in `make_plots`** — safely identifies only the figures created by the current call so titles are not applied to pre-existing windows from a previous loop iteration.

---

## 2026-04-04 — Update Vehicle to Match NDARC Parameters

### Summary

Updated `Hawkai/parameters.py` and `Hawkai/vehicle.py` to faithfully reflect the Hawkai NDARC input file (`Hawkai_Input.txt`). Lift-rotor spatial positions (origins and rotations) were preserved per user instruction.

### What changed and where

| File | Parameter | Old | New | Source |
|------|-----------|-----|-----|--------|
| `parameters.py` | `TAKEOFF_MASS` | 2500 lb | **340 lb** | NDARC DGW |
| `parameters.py` | `OPERATING_EMPTY_MASS` | 2150 lb | **260 lb** | NDARC WE |
| `parameters.py` | `MAX_TAKEOFF_MASS` | 2500 lb | **340 lb** | fWMTO=1.0 |
| `parameters.py` | `MAX_PAYLOAD_MASS` | 100 lb | **80 lb** | DGW−WE |
| `parameters.py` | `CENTER_OF_GRAVITY` | `[[2.0,0,0]]` m | **`[[0.933,0,0]]`** m | SL=3.06 ft |
| `parameters.py` | `WING_ORIGIN` | `[[1.215,0,0.313]]` | **`[[0.933,0,0.244]]`** | SL=3.06 ft, WL=0.80 ft |
| `parameters.py` | `WING_INCIDENCE` *(new)* | — | **5.0°** | NDARC wing incidence |
| `parameters.py` | `HTAIL_STATION_LINE` *(new)* | — | **8.75 ft** | NDARC htail SL |
| `parameters.py` | `FUSELAGE_TOTAL_LENGTH` | 7.899 ft | **8.0 ft** | NDARC |
| `parameters.py` | `FUSELAGE_WIDTH` | 1.333 ft | **2.0 ft** | NDARC |
| `parameters.py` | `FUSELAGE_HEIGHT` | 1.333 ft | **2.0 ft** | NDARC |
| `parameters.py` | `FUSELAGE_EFF_DIAMETER` | 5.85 ft | **2.0 ft** | matches width |
| `parameters.py` | `LIFT_ROTOR_TIP_RADIUS` | 1.5 m | **0.6096 m** | 2 ft per NDARC |
| `parameters.py` | `LIFT_ROTOR_HUB_RADIUS` | 0.15 m | **0.0610 m** | 10% root cutout |
| `parameters.py` | `LIFT_ROTOR_N_BLADES` | 4 | **2** | NDARC |
| `parameters.py` | `LIFT_ROTOR_DESIGN_TIP_MACH` | 0.65 | **0.0746** | Vtip=83.3 ft/s |
| `parameters.py` | `PROP_TIP_RADIUS` | 0.9 m | **0.5090 m** | 1.67 ft per NDARC |
| `parameters.py` | `PROP_HUB_RADIUS` | 0.1 m | **0.0509 m** | 10% of tip radius |
| `parameters.py` | `PROP_N_BLADES` | 3 | **2** | NDARC |
| `parameters.py` | `PROP_RPM` | 2200 rpm | **572 rpm** | Vtip=100 ft/s, R=1.67 ft |
| `parameters.py` | `PROP_DESIGN_THRUST` | 500 lbf | **68 lbf** | scaled × (340/2500) |
| `parameters.py` | `LIFT_MOTOR_EFFICIENCY` | 0.85 | **0.90** | NDARC eta_motor |
| `parameters.py` | `PROP_MOTOR_EFFICIENCY` | 0.95 | **0.90** | NDARC eta_motor |
| `vehicle.py` | Wing segment twist (Root, Section_2, Tip) | 0° | **P.WING_INCIDENCE (5°)** | NDARC incidence |
| `vehicle.py` | Htail `wing.origin` formula | hardcoded 7.324 ft ref | **`P.HTAIL_STATION_LINE − 0.25×chord`** | NDARC SL=8.75 ft |

### What was NOT changed
- Lift-rotor `LIFT_ROTOR_ORIGINS` and `LIFT_ROTOR_ROTATIONS` — preserved per user instruction
- Boom geometry — tied to rotor lateral positions
- Wing planform (span, chord, sweep, dihedral, t/c) — already matched NDARC
- Tail planform (area, AR, taper, sweep, t/c) — already matched NDARC
- `VTAIL_ORIGIN_X` (8.75 ft) — already correct
- Battery, ESC, avionics, airfoils, mission profile

### Verification
`python3 Hawkai.py` ran to completion with exit code 0. OpenVSP "Diameter" warnings are pre-existing cosmetic issues unrelated to these changes.
