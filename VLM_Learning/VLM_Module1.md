# Module 1 — SUAVE Framework Orientation

## 1. The Three-Layer Class Hierarchy

`Fidelity_Zero` doesn't stand alone. It inherits through two layers:

```
Markup  (Aerodynamics.py → Markup.py)
  └── Fidelity_Zero
        └── compute.lift.inviscid_wings = Vortex_Lattice()
```

- **`Markup`** owns `self.process`, `self.settings`, `self.geometry`, and the `evaluate()` method.
- **`Fidelity_Zero`** populates that process with the actual aerodynamic steps.
- **`Vortex_Lattice`** is one *step* inside the lift sub-process — it is not the top-level class.

---

## 2. The `Process` Container

`Process` (`Process.py:18`) is a `ContainerOrdered` — basically an ordered dictionary that is also callable. When you call it, it iterates over its items in insertion order and calls each one:

```python
# Process.py:57-68
for tag, step in self.items():
    if hasattr(step, 'evaluate'):
        result = step.evaluate(*args, **kwarg)
    else:
        result = step(*args, **kwarg)
```

So `self.process.compute(state, settings, geometry)` runs every registered step in sequence, passing the same three arguments to each. This is how `Fidelity_Zero` wires up its whole pipeline declaratively in `__defaults__`:

```python
# Fidelity_Zero.py:84-110
compute.lift = Process()
compute.lift.inviscid_wings = Vortex_Lattice()   # ← step 1 of lift
compute.lift.fuselage       = fuselage_correction # ← step 2
compute.lift.total          = aircraft_total      # ← step 3

compute.drag = Process()
compute.drag.parasite = Process()
compute.drag.parasite.wings.wing = parasite_drag_wing
# ... etc.
```

**The key insight:** adding a step to the process is just a dict assignment. The ordering of steps is the ordering of insertion.

---

## 3. The `evaluate()` Entry Point

```python
# Markup.py:84
results = self.process.compute(state, settings, geometry)
```

`state` contains `conditions`, which is where all flight-state arrays live. The `settings` and `geometry` come from the analysis object itself. From this single call, all of lift, drag, and moment calculations cascade.

---

## 4. The `conditions` Object — Shape Convention

This is critical and comes up everywhere in later modules. All aerodynamic quantities in `conditions` are stored as **2D NumPy arrays of shape `[n_conditions, 1]`**:

```python
# Vortex_Lattice.py:221-222
AoA  = conditions.aerodynamics.angle_of_attack.T[0]  # shape [n]
Mach = conditions.freestream.mach_number.T[0]         # shape [n]
```

The `.T[0]` squeezes the column vector to a 1D array before computation, then results are packed back with `np.atleast_2d(...).T`. You'll see this pattern constantly. When VLM solves multiple flight conditions in one call, `n_conditions` can be 160 at once (10 AoA × 16 Mach during surrogate training).

---

## 5. Two Evaluation Modes

`initialize()` sets `self.evaluate` to one of two functions at startup — this is a function pointer assignment, not subclassing:

```python
# Vortex_Lattice.py:166-176
if use_surrogate == True:
    self.sample_training()   # run VLM on 10×16 = 160 conditions
    self.build_surrogate()   # fit RectBivariateSpline
    self.evaluate = self.evaluate_surrogate
else:
    self.evaluate = self.evaluate_no_surrogate
```

| Mode | When called | What it does |
|------|-------------|--------------|
| `evaluate_surrogate` | Every mission point during flight | Interpolates `RectBivariateSpline(AoA, Mach)` — microseconds |
| `evaluate_no_surrogate` | Once per training point, or when `use_surrogate=False` | Calls `calculate_VLM()` → `VLM()` — milliseconds to seconds |

For your lift+cruise work: **set `use_surrogate = False`** in `Fidelity_Zero.__defaults__` (`line 74`) to always run the direct path. The surrogate cannot capture propeller slipstream effects (Module 10 will explain why).

---

## 6. The `calculate_VLM` Helper

At the bottom of `Vortex_Lattice.py:637`, there's a module-level helper `calculate_VLM()`. This is the only place `VLM()` is called. Its job is:

1. Call `VLM(conditions, settings, geometry)` — gets raw results
2. **Re-dimensionalize** per-wing coefficients using `geometry.vortex_distribution.wing_areas`
3. **Re-normalize** by each wing's own reference area (`wing.areas.reference`)

This matters because VLM internally normalizes everything by the *vehicle* reference area. The per-wing CL you get back from `wing_lifts[wing.tag]` is normalized by that wing's own area — a different denominator.

---

## 7. Surrogate Training Grid

```python
# Vortex_Lattice.py:92-94
training.angle_of_attack = [[-5, -2, 0, 2, 5, 8, 10, 12, 45, 75]]°
training.Mach            = [[0.0, 0.1, 0.2, 0.3, 0.5, 0.75, 0.85, 0.9,
                              1.3, 1.35, 1.5, 2.0, 2.25, 2.5, 3.0, 3.5]]
```

Note `Fidelity_Zero.__defaults__` overrides the Mach grid to just 8 subsonic points (`line 113`) — the supersonic columns are dropped for a subsonic-only analysis. If you're modeling a supersonic vehicle you'd restore the full 16-point grid.

The transonic surrogate (`CL_surrogate_trans`, built at `line 576`) uses `RegularGridInterpolator` (linear) bridging the last subsonic point and first two supersonic points — a thin 3-column slab over [0.9, 1.3, 1.35]. The smooth blending across this slab uses `Cubic_Spline_Blender` (weights `h_sub`, `h_sup`) in `evaluate_surrogate:259-270`.

---

## Summary — What to Hold Onto for Later Modules

| Concept | Where it matters again |
|---------|----------------------|
| `conditions` as `[n, 1]` arrays | Every subsequent module |
| `settings.number_spanwise_vortices` (default 15) | Module 3 — sets panel count |
| `settings.use_VORLAX_matrix_calculation` | Modules 5, 6 — alternate AIC/RHS path |
| `geometry.vortex_distribution` gets populated by VLM, then read back here | Module 3 onward |
| `calculate_VLM` re-normalizes per-wing coefficients | Module 8 — force integration |
