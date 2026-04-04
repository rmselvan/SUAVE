# Module 10 — The Surrogate Model

**Primary file:** `Vortex_Lattice.py`
**Prerequisites:** All prior modules

---

## Overview

The VLM linear solve is too expensive to call at every mission point in a trajectory optimization or full mission simulation — at 1–100 ms per call for a fine mesh, a 1000-point mission would take minutes per vehicle evaluation. The surrogate model replaces the VLM with a fast 2D spline interpolation over `(alpha, Mach)` that executes in microseconds.

This module covers: why the surrogate is structured the way it is, what it captures, what it misses, and how to validate it.

---

## 1. Why a Surrogate?

A SUAVE mission simulation calls `Fidelity_Zero.evaluate()` at every mission segment point. For a climb segment with 50 points, a cruise with 100 points, and descent with 30 points, that's 180 VLM calls per mission evaluation. In a gradient-based optimizer this could happen hundreds of times per optimization run.

The surrogate trades one-time setup cost (training: ~160 VLM calls at initialization) for near-zero per-call cost. For a subsonic-only vehicle (8 Mach training points × 10 AoA = 80 calls), initialization takes ~8 seconds and each mission point takes ~10 µs — a ~100,000× speedup per call.

---

## 2. The Training Grid

### 2.1 Default AoA Points

```python
# Vortex_Lattice.py:92
training.angle_of_attack = [[-5, -2, 0, 2, 5, 8, 10, 12, 45, 75]]° * Units.deg
```

10 points covering a wide range including extreme angles (45°, 75°) for Polhamus delta-wing vortex lift. The non-uniform spacing concentrates points at low angles (flight regime) while capturing the highly nonlinear high-AoA behavior.

### 2.2 Mach Points

Full grid (Vortex_Lattice defaults):
```python
# Vortex_Lattice.py:93-94
training.Mach = [[0.0, 0.1, 0.2, 0.3, 0.5, 0.75, 0.85, 0.9,
                   1.3, 1.35, 1.5, 2.0, 2.25, 2.5, 3.0, 3.5]]
```

Fidelity_Zero override (subsonic only):
```python
# Fidelity_Zero.py:113
compute.lift.inviscid_wings.training.Mach = [[0.0, 0.1, 0.2, 0.3, 0.5, 0.75, 0.85, 0.9]]
```

`Fidelity_Zero` is labeled "Subsonic" in its docstring, so it restricts the Mach grid to M < 1. For a vehicle that operates transonically or supersonically, you must restore the full 16-point grid.

### 2.3 Vectorized Training Evaluation

All `n_AoA × n_Mach` training conditions are evaluated in a **single call** to `calculate_VLM`:

```python
# Vortex_Lattice.py:422-434
lenM  = len(Mach)
AoAs  = np.atleast_2d(np.tile(AoA, lenM).T.flatten()).T   # [lenAoA*lenM, 1]
Machs = np.atleast_2d(np.tile(Mach, lenAoA).flatten()).T  # [lenAoA*lenM, 1]
zeros = np.zeros_like(Machs)

konditions.aerodynamics.angle_of_attack = AoAs
konditions.freestream.mach_number       = Machs
konditions.freestream.velocity          = zeros

total_lift, total_drag, ..., = calculate_VLM(konditions, settings, geometry)
```

`np.tile` creates the full Cartesian product grid. All 80 (or 160) conditions are solved simultaneously in the single `np.linalg.solve(A, RHS)` call — the batched solve across conditions is what makes this efficient.

---

## 3. Subsonic/Supersonic Split

After training, results are split at `M = 1`:

```python
# Vortex_Lattice.py:437-453
sub_sup_split = np.where(Machs < 1.0)[0][-1] + 1
CL_sub  = np.reshape(total_lift[0:sub_sup_split, 0], (len_sub_mach, lenAoA)).T
CL_sup  = np.reshape(total_lift[sub_sup_split:, 0],  (len_sup_mach, lenAoA)).T
```

The `T` transpose reshapes from `[Mach_idx, AoA_idx]` to `[AoA_idx, Mach_idx]` — this is the expected input format for `RectBivariateSpline(AoA_data, mach_data, CL_data)`.

---

## 4. The Surrogate Models

### 4.1 `RectBivariateSpline` — Subsonic and Supersonic

```python
# Vortex_Lattice.py:553-559
CL_surrogate_sub  = RectBivariateSpline(AoA_data, mach_data_sub, CL_data_sub)
CDi_surrogate_sub = RectBivariateSpline(AoA_data, mach_data_sub, CDi_data_sub)
CL_surrogate_sup  = RectBivariateSpline(AoA_data, mach_data_sup, CL_data_sup)
CDi_surrogate_sup = RectBivariateSpline(AoA_data, mach_data_sup, CDi_data_sup)
```

`RectBivariateSpline` fits a bicubic spline over a **rectangular** grid of (AoA, Mach) training points. Evaluation:

```python
CL = CL_surrogate_sub(AoA, Mach, grid=False)   # `grid=False` for point evaluation (not meshgrid)
```

With `grid=False`, `AoA` and `Mach` are 1D arrays of the same length and the surrogate evaluates each `(AoA[i], Mach[i])` pair independently. This matches the `[n_conditions, 1]` → `.T[0]` → 1D array convention from Module 1.

### 4.2 `RegularGridInterpolator` — Transonic

The transonic surrogate bridges the subsonic and supersonic splines over the gap between `M_sub[-1] = 0.9` and `M_sup[0] = 1.3`:

```python
# Vortex_Lattice.py:563-579
CL_data_trans = np.zeros((len(AoA_data), 3))
CL_data_trans[:,0] = CL_data_sub[:,-1]   # last subsonic point (M=0.9)
CL_data_trans[:,1] = CL_data_sup[:,0]    # first supersonic point (M=1.3)
CL_data_trans[:,2] = CL_data_sup[:,1]    # second supersonic point (M=1.35)

mach_data_trans = [mach_data_sub[-1], mach_data_sup[0], mach_data_sup[1]]   # [0.9, 1.3, 1.35]

CL_surrogate_trans = RegularGridInterpolator((AoA_data, mach_data_trans),
                                              CL_data_trans,
                                              method='linear',
                                              bounds_error=False,
                                              fill_value=None)
```

`RegularGridInterpolator` is used here instead of `RectBivariateSpline` because the 3-point transonic slab is too narrow for a cubic spline — linear interpolation is more appropriate. `bounds_error=False, fill_value=None` means it extrapolates (using the boundary value) rather than raising an error when a query is outside [0.9, 1.35].

---

## 5. The Transonic Blending Scheme

The three surrogates are blended using cubic spline weight functions:

```python
# Vortex_Lattice.py:259-270  (evaluate_surrogate)
sub_trans_spline = Cubic_Spline_Blender(hsub_min=0.85, hsub_max=0.95)
h_sub = lambda M: sub_trans_spline.compute(M)   # 1 → 0 over [0.85, 0.95]

sup_trans_spline = Cubic_Spline_Blender(hsup_min=1.05, hsup_max=1.25)
h_sup = lambda M: sup_trans_spline.compute(M)   # 0 → 1 over [1.05, 1.25]

CL = h_sub(M) * CL_sub(AoA,M) \
   + (h_sup(M) - h_sub(M)) * CL_trans((AoA,M)) \
   + (1 - h_sup(M)) * CL_sup(AoA,M)
```

The three blending regions:

| Mach range | `h_sub` | `h_sup` | Active component |
|-----------|---------|---------|-----------------|
| `M < 0.85` | 1 | 0 | Fully subsonic spline |
| `0.85 < M < 0.95` | 1→0 | 0 | Subsonic spline fading out |
| `0.95 < M < 1.05` | 0 | 0 | Fully transonic interpolator |
| `1.05 < M < 1.25` | 0 | 0→1 | Supersonic spline fading in |
| `M > 1.25` | 0 | 1 | Fully supersonic spline |

The weight `(h_sup - h_sub)` is the transonic weight — it peaks at 1 between [0.95, 1.05] and ramps off on both sides.

`Cubic_Spline_Blender` uses a smooth cubic polynomial `3t² - 2t³` (the standard smoothstep function) within its transition range, producing C1-continuous blending.

---

## 6. Per-Wing Surrogates

Separate surrogates are trained and stored for **each wing**:

```python
# Vortex_Lattice.py:588-616
for wing in geometry.wings.keys():
    CL_w_surrogates_sub[wing]  = RectBivariateSpline(AoA_data, mach_data_sub, CL_w_data_sub[wing])
    CDi_w_surrogates_sub[wing] = RectBivariateSpline(AoA_data, mach_data_sub, CDi_w_data_sub[wing])
    # ... and transonic, supersonic versions ...
```

This enables `evaluate_surrogate` to return per-wing lift breakdown `inviscid_wings[wing.tag]` in addition to the total CL. The wing-specific coefficients are used by the drag analysis (`induced_drag_aircraft`) and by the aerodynamic breakdown outputs.

---

## 7. What the Surrogate Captures vs. Misses

### Captured

| Quantity | Captured by surrogate? |
|----------|----------------------|
| CL vs. alpha (nonlinear) | ✓ |
| CDi vs. alpha | ✓ |
| Prandtl-Glauert compressibility effect | ✓ |
| Wave drag (via supersonic kernel) | ✓ |
| Per-wing CL breakdown | ✓ |

### Not captured

| Quantity | Missed — why |
|----------|-------------|
| Sideslip effects | Trained at β=0 only |
| Rotation rate derivatives (CLq, CLp, CLr) | Trained at p=q=r=0 |
| Propeller/rotor slipstream | No wake model during training |
| Control surface deflection effects | Surrogate trained at δ=0; deflection would require retraining |
| Sectional pressure distribution | Not stored in surrogate |
| Non-uniform inflow | Surrogate assumes uniform freestream |

For any condition that exercises these missing effects, **`use_surrogate = False` is required**.

---

## 8. Validating the Surrogate

### 8.1 Check Training Residuals

Set `use_surrogate = False` and evaluate at the training grid points. Compare to `training.lift_coefficient_sub`. The surrogate should reproduce these exactly (splines interpolate, not approximate, at training nodes).

```python
# Pseudo-code validation
for i, aoa in enumerate(training.angle_of_attack[:,0]):
    for j, mach in enumerate(training.Mach[:sub_len, 0]):
        CL_direct = training.lift_coefficient_sub[i, j]
        CL_spline = surrogates.lift_coefficient_sub(aoa, mach, grid=False)
        assert abs(CL_direct - CL_spline) < 1e-6  # should be exact at nodes
```

### 8.2 Check Midpoint Accuracy

Evaluate the surrogate at Mach and AoA values **between** training points and compare to direct VLM calls:

```python
aoa_test  = np.radians(3.0)   # between 2° and 5° training points
mach_test = 0.4               # between 0.3 and 0.5 training points
CL_direct   = calculate_VLM(konditions_at_test, settings, geometry)[0]
CL_surrogate = surrogates.lift_coefficient_sub(aoa_test, mach_test, grid=False)
```

Typical accuracy: `|ΔCL| < 0.005` for smooth, well-behaved configurations. Larger errors indicate the training grid needs more points in that region.

### 8.3 Check Transonic Blending

Plot `CL(M)` from the surrogate at fixed AoA across `M = [0.7, 1.4]`. The blended output should be smooth and monotonic (no kink at `M = 0.85` or `M = 1.05`). A visible kink indicates the transonic data slab is inconsistent with the sub/supersonic splines — often caused by numerical noise in the VLM at near-sonic Mach numbers.

### 8.4 Practical Setting for Lift+Cruise

For `tutorialLC.py` with `propeller_wake_model = True`:
- Set `use_surrogate = False` during transition analysis
- Set `use_surrogate = True` during cruise (clean configuration, no slipstream effect on wing)
- Or: use `use_surrogate = False` globally and accept the runtime cost if accuracy in transition matters

---

## 9. Three-Regime Code Structure Summary

```python
# Vortex_Lattice.py:251-270 (evaluate_surrogate — full case)

if CL_surrogate_sup == None:     # pure subsonic vehicle
    CL = CL_surrogate_sub(AoA, Mach, grid=False)

elif CL_surrogate_sub == None:   # pure supersonic vehicle
    CL = CL_surrogate_sup(AoA, Mach, grid=False)

else:                            # mixed sub/supersonic vehicle
    CL = h_sub(M)*CL_sub(AoA,M) + (h_sup(M)-h_sub(M))*CL_trans((AoA,M)) + (1-h_sup(M))*CL_sup(AoA,M)
```

The pure-subsonic and pure-supersonic branches exist because vehicles that never cross `M = 1` have no transonic training data and no transonic surrogate. For a lift+cruise vehicle that stays below `M = 0.9`, the `Fidelity_Zero` override produces a subsonic-only surrogate and the first branch applies.

---

## Summary — Key Concepts

| Concept | Detail |
|---------|--------|
| Training grid | 10 AoA × 8 (subsonic) or 16 (full) Mach points |
| `np.tile` grid construction | Vectorized Cartesian product for batch training call |
| `RectBivariateSpline` | Bicubic spline for sub/sup regimes; exact at training nodes |
| `RegularGridInterpolator` | Linear interpolation for thin transonic slab [0.9, 1.3, 1.35] |
| Cubic blending (`h_sub`, `h_sup`) | Smooth C1 transition between regimes over [0.85,0.95] and [1.05,1.25] |
| Per-wing surrogates | Separate `RectBivariateSpline` per wing for lift breakdown |
| What surrogate misses | Sideslip, rotation rates, slipstream, control deflection, non-uniform inflow |
| Validation strategy | Residual at nodes (exact), midpoint error (< 0.005), transonic smoothness |
| Lift+cruise recommendation | `use_surrogate = False` during transition; `True` acceptable in cruise |

---

## Curriculum Complete

You have now covered the full SUAVE VLM pipeline:

```
Fidelity_Zero.evaluate()                     [Module 1]
→ Vortex_Lattice.evaluate_no_surrogate()     [Module 1]
  → VLM(conditions, settings, geometry)
    → make_VLM_wings()                       [Module 2]
    → generate_wing_vortex_distribution()    [Module 3]
    → postprocess_VD()                       [Module 4]
    → deflect_control_surfaces()             [Module 9]
    → compute_wing_induced_velocity()        [Module 5] → C_mn, EW
    → compute_RHS_matrix()                   [Module 6] → RHS
    → np.linalg.solve(A, RHS) → GAMMA        [Module 7]
    → DCP, CSUC (LE suction)                 [Module 7]
    → CL, CDi, CM, CL_wing, cl_y            [Module 8]
→ Vortex_Lattice.sample_training()           [Module 10]
→ Vortex_Lattice.build_surrogate()           [Module 10]
→ Vortex_Lattice.evaluate_surrogate()        [Module 10]
```
