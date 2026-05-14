# SUAVE VLM Teaching Curriculum

## Context
This curriculum provides a structured learning path for understanding how SUAVE implements its Vortex Lattice Method (VLM) — from framework entry points down to the linear algebra and force integration. It was designed after exploring the full VLM call chain in the SUAVE codebase.

---

## Module Overview (10 Modules)

| Module | Core Topic | Primary File(s) | Prerequisites |
|--------|-----------|-----------------|---------------|
| 1 | Framework orientation — `Fidelity_Zero`, `Vortex_Lattice` class, analysis dispatch | `Fidelity_Zero.py`, `Vortex_Lattice.py` | None |
| 2 | Wing geometry preprocessing — segmented format, control surface injection | `make_VLM_wings.py` | Module 1 |
| 3 | Panel mesh generation — horseshoe layout, VD coordinate arrays | `generate_vortex_distribution.py` | Module 2 |
| 4 | Panel postprocessing — areas, normals, VORLAX geometric quantities | `generate_VD_helpers.py` | Module 3 |
| 5 | AIC matrix via Biot-Savart — subsonic kernel, supersonic kernel | `compute_wing_induced_velocity.py` | Modules 3, 4 |
| 6 | RHS boundary conditions — rotation rates, slipstream injection | `compute_RHS_matrix.py` | Modules 3, 4 |
| 7 | Linear solve & pressure coefficient — GAMMA solve, leading-edge suction | `VLM.py` lines 260–421 | Modules 5, 6 |
| 8 | Force/moment integration — strip-to-wing-to-total reduction | `VLM.py` lines 333–494 | Module 7 |
| 9 | Compressibility, control surfaces, wake coupling | `deflect_control_surface.py`, `VLM.py`, `compute_RHS_matrix.py` | Modules 5–8 |
| 10 | Surrogate training, spline interpolation, transonic blending, validation | `Vortex_Lattice.py` | All prior modules |

---

## Call Chain (for reference)

```
Fidelity_Zero.evaluate()
→ Vortex_Lattice.evaluate_no_surrogate()
  → VLM(conditions, settings, geometry)
    → generate_vortex_distribution()    # Steps 1-9: Modules 2, 3, 4
      → make_VLM_wings()
      → generate_wing_vortex_distribution()
      → postprocess_VD()
      → deflect_control_surfaces()
    → compute_wing_induced_velocity()   # Step 10A: Module 5 → C_mn, EW
    → compute_RHS_matrix()              # Step 10B: Module 6 → RHS
    → np.linalg.solve(A, RHS) → GAMMA  # Step 10C: Module 7
    → Pressure coefficient from GAMMA  # Step 11: Module 7
    → CL, CDi, CM, moments             # Step 12: Module 8
```

---

## Module Summaries

### Module 1 — SUAVE Framework Orientation

**What you will learn:** How the analysis pipeline is structured; where VLM sits within it; how `Vehicle`, `Analyses`, `Conditions`, and `Results` objects interact.

**Key files:**
- `trunk/SUAVE/Analyses/Aerodynamics/Fidelity_Zero.py` — top-level aerodynamics class; `__defaults__()`, `initialize()`, `evaluate()`
- `trunk/SUAVE/Analyses/Aerodynamics/Vortex_Lattice.py` — lines 51–176; settings defaults, surrogate vs. direct dispatch

**Key concepts:**
- `SUAVE.Core.Data` universal container — attribute access like `geometry.wings['main_wing'].chords.root`
- `Fidelity_Zero.evaluate()` runs a process chain: lift → drag → moments. VLM is responsible only for inviscid lift and induced drag; parasite drag lives separately.
- `conditions` object holds `angle_of_attack`, `freestream.mach_number`, rotation rates — as 2D arrays `[n_conditions, 1]`, not scalars. The VLM solves multiple flight conditions in one call.
- Two evaluation modes: `evaluate_surrogate` (default, fast, interpolation) vs `evaluate_no_surrogate` (calls `VLM()` directly for every condition)

---

### Module 2 — Wing Geometry Preprocessing

**What you will learn:** How SUAVE wings are parsed and converted to a VLM-ready segmented format; how control surfaces are injected into the panel topology.

**Key files:**
- `trunk/SUAVE/Methods/Aerodynamics/Common/Fidelity_Zero/Lift/make_VLM_wings.py` — full file; focus on `make_VLM_wings()`, `convert_to_segmented_wing()`, `make_span_break()`

**Key concepts:**
- Segmented vs. unsegmented wings: `convert_to_segmented_wing()` normalizes simple wings to a two-segment form before panelization
- Sweep conversion: both `sweeps.quarter_chord` and `sweeps.leading_edge` are supported; internally converted to LE-sweep coordinates
- Control surface injection: when `settings.discretize_control_surfaces = True`, `populate_control_sections()` splits wing segments at control surface boundaries; each control surface becomes a pseudo-wing with `wing.is_a_control_surface = True`
- `span_breaks`: index array marking where each wing's strips begin in the global panel list — the scaffold for force summation in Module 8 via `np.add.reduceat(..., span_breaks)`
- `vortex_lift` flag: per-wing boolean enabling Polhamus leading-edge suction (delta wings)

---

### Module 3 — Panel Mesh Generation

**What you will learn:** Exactly how the horseshoe vortex mesh is laid out on each wing; what every field in the `VD` (vortex distribution) object represents; how symmetry is handled.

**Key files:**
- `trunk/SUAVE/Methods/Aerodynamics/Common/Fidelity_Zero/Lift/generate_vortex_distribution.py` — `generate_wing_vortex_distribution()` starting at line 276; the header schematic (lines 40–58)

**Key concepts:**
- **1/4–3/4 rule**: bound vortex at 25% chord (`XAH/XBH`), control point at 75% chord (`XC/YC/ZC`). Derived from thin airfoil theory — a single vortex at 1/4c reproduces exact lift slope; the 3/4c control point satisfies the Kutta condition.
- Panel corners: `A1/B1` (LE corners), `A2/B2` (TE corners); suffix `A` = port side, `B` = starboard side
- **Cosine spanwise spacing**: `y_i = (span/2)(1 - cos(π i / n_sw))` — clusters panels near root/tip where circulation gradients are highest
- Symmetry: starboard panels generated first, then port copy appended with negated Y. This creates a block structure in the AIC matrix.
- `leading_edge_indices`, `trailing_edge_indices` — boolean masks over all panels used throughout force integration
- `panels_per_strip` (RNMAX) — chordwise panel count per strip

---

### Module 4 — Panel Postprocessing

**What you will learn:** How panel areas and unit normals are computed from four corner coordinates; the VORLAX-specific quantities SLOPE, ZETA, and chord length.

**Key files:**
- `trunk/SUAVE/Methods/Aerodynamics/Common/Fidelity_Zero/Lift/generate_VD_helpers.py` — full file (169 lines)

**Key concepts:**
- **Panel area**: split each quadrilateral into two triangles, `area = 0.5 * ||P1P2 × P1P3||` per triangle, sum both
- **Unit normal**: cross product of diagonal edge vectors, normalized; sign enforced to always point +Z (upward)
- `SLOPE`: camber slope per panel = `(Z2c - Z1c) / (X2c - X1c)` — encodes local camber and twist; used in pressure coefficient calculation
- `ZETA`: strip-level chord-line inclination from LE to TE centerline positions — used in force rotation in Module 8
- `VD.chord_lengths` (CHORD): arc-length chord along camber line (not flat projected chord)
- `is_postprocessed` guard flag — checked at VLM entry; must be re-run after any panel modification (e.g., control surface deflection)

---

### Module 5 — The Aerodynamic Influence Coefficient (AIC) Matrix

**What you will learn:** How `C_mn` is computed — the Biot-Savart law applied to horseshoe vortices, the frame rotation into the horseshoe-aligned coordinate system, and the subsonic vs. supersonic kernels.

**Key files:**
- `trunk/SUAVE/Methods/Aerodynamics/Common/Fidelity_Zero/Lift/compute_wing_induced_velocity.py` — full file; read `compute_wing_induced_velocity()`, `subsonic()`, `supersonic()` in order

**Key concepts:**
- **`C_mn` meaning**: element `C_mn[mach, i, j, :]` = velocity vector at control point `j` induced by a **unit-strength** horseshoe vortex on panel `i`. Shape: `[n_mach, n_cp, n_cp, 3]`. Geometry- and Mach-dependent; computed once per unique Mach value.
- **Frame rotation**: lines 121–131 rotate the problem so the bound vortex leg lies along a local Y-axis. `theta = arctan2(zb-za, yb-ya)`. This lets the VORLAX-derived formulas (written for unswept flat bound legs) work for arbitrary dihedral and sweep.
- **Subsonic kernel (Miranda et al. 1977)**:
  - `RAD = sqrt(X^2 - B2*(Y^2 + Z^2))` where `B2 = M^2 - 1` (negative for M < 1)
  - Prandtl-Glauert transformation is **embedded in the kernel**, not a post-hoc correction
  - Returns U, V, W velocity components per horseshoe vortex
- **Supersonic kernel**: `B2 > 0`; `RFLAG` array zeroes contributions where the control point is outside the panel's Mach cone. Applied at line 258 of `VLM.py`: `RHS = RHS * RFLAG`.
- **`EW` matrix**: normal-component of induced velocity in the VORLAX frame — used in two places: as the AIC matrix when `use_VORLAX_matrix_calculation=True`, and in the leading-edge suction term (Module 7).
- **Memory**: `C_mn` is cast to `float32`. For 300 panels: `[n_mach, 300, 300, 3]` ≈ 108 MB per unique Mach. Grows quadratically — primary memory bottleneck.

---

### Module 6 — The Right-Hand Side (RHS) Boundary Condition Vector

**What you will learn:** How the flow-tangency boundary condition is encoded as a vector; how angle of attack, sideslip, rotation rates, and propeller slipstream all enter the RHS.

**Key files:**
- `trunk/SUAVE/Methods/Aerodynamics/Common/Fidelity_Zero/Lift/compute_RHS_matrix.py` — full file; read `compute_RHS_matrix()` for propeller wake injection, then `build_RHS()` for the boundary condition itself

**Key concepts:**
- **Physical condition**: no flow passes through any surface → `V_total · n̂_panel = 0` → `RHS_n = V_total · n̂_n`
- **Velocity composition**:
  ```
  Vx = V∞ cos(α) cos(β) + Vx_rotation + Vx_wake
  Vy = V∞ cos(α) sin(β) + Vy_rotation + Vy_wake
  Vz = V∞ sin(α)        + Vz_rotation + Vz_wake
  ```
- **Rotation terms**: `Vx_rot = -q·z + r·y`, `Vy_rot = -r·x + p·z`, `Vz_rot = -p·y + q·x` (relative to rotation center). This is what makes the VLM compute pitch damping derivatives `CM_q` correctly.
- **Slipstream injection** (lines 88–112): loops over all propellers/lift rotors; `p.Wake.evaluate_slipstream()` returns `[n_ctrl_pts, n_eval_pts, 3]` induced velocities, summed into the velocity field at each control point. Key capability for lift+cruise configurations.
- **VORLAX-frame alternative** (lines 205–218): `ALOC = Vx*SCNTL + Vy*CCNTL*SID - Vz*CCNTL*COD` — useful for validation against VORLAX outputs but does NOT incorporate wake velocities.
- **`ONSET`**: x-component of rigid-body rotation velocity, normalized by `V∞`. Reused in the pressure coefficient calculation (`FACTOR = FORAXL + ONSET`).

---

### Module 7 — The Linear Solve and Pressure Coefficient

**What you will learn:** How the AIC matrix is assembled from `C_mn`; how the linear system is solved for vortex strengths GAMMA; and how GAMMA is converted to pressure coefficient — including leading-edge suction and vortex lift.

**Key files:**
- `trunk/SUAVE/Methods/Aerodynamics/Common/Fidelity_Zero/Lift/VLM.py` — lines 260–421

**Key concepts:**
- **AIC assembly** (lines 262–267):
  ```python
  A = C_mn[:,:,:,0]*sin(δ)*cos(φ)
    + C_mn[:,:,:,1]*cos(δ)*sin(φ)
    - C_mn[:,:,:,2]*cos(φ)*cos(δ)
  ```
  Projects three velocity components onto panel normals using camber angle `δ` and dihedral angle `φ`.
- **Linear solve** (line 270): `GAMMA = np.linalg.solve(A, RHS)` — dense direct solve (LAPACK `dgesv`). For n_cp=300: trivial; for n_cp=3000: dominant compute cost.
- **Pressure coefficient** (VORLAX PRESS subroutine):
  - `FACTOR = cos(α)cos(β) + ONSET`
  - `GNET = GAMMA * FACTOR * RNMAX / CHORD`
  - `DCP = 2*GNET + DCPSID` (sideslip correction via cumulative spanwise vortex sum)
- **`strip_cumsum()`** (line 584): cumulative sum of GAMMA along chord, resetting at each new strip's LE panel. Implements the Kutta-Joukowski theorem (local lift depends on total circulation from LE to that station).
- **Leading-edge suction** (lines 406–421):
  - `CLE = (EW * GAMMA).sum(axis=2) - EFFINC` (effective induced flow at LE using `EW` from Module 5)
  - `CSUC = 0.5 * π * |SPC| * CLE² * STB`
  - `STB = 0` when LE is supersonic (Mach cone tangent to LE) — suction disappears
- **Polhamus vortex lift** (delta wings): when `wing.vortex_lift = True` and subsonic, `SPC = -1` rotates the suction force direction — leading-edge vortex lift analogy

---

### Module 8 — Force and Moment Integration

**What you will learn:** How strip forces are extracted from pressure; how they are rotated from the camber-line frame to body axes; how strip integrals reduce to wing and vehicle coefficients.

**Key files:**
- `trunk/SUAVE/Methods/Aerodynamics/Common/Fidelity_Zero/Lift/VLM.py` — lines 333–494

**Key concepts:**
- **Strip normal force `CNC`** (lines 379–383): `SINF = ADC * DCP` per panel; `CNC = np.add.reduceat(SINF, chord_breaks) + CSUC * TFZ`
- **Axial force `CAXL`** (lines 391–395): `CAXL = -SINF * TX / (1 + TX²)` where `TX = SLOPE - ZETA` is local camber slope relative to strip mean
- **Body-axis force rotation** (lines 449–451):
  ```python
  BFX = -CNC*sin(ZETA) + CAXL*cos(ZETA)
  BFY = -(CNC*cos(ZETA) + CAXL*sin(ZETA))*sin(φ)
  BFZ =  (CNC*cos(ZETA) + CAXL*sin(ZETA))*cos(φ)
  ```
- **Lift/drag per strip** (lines 471–476):
  ```python
  LIFT = (BFZ*cos(α) - (BFX*cos(β) + BFY*sin(β))*sin(α)) * STRIP
  DRAG = CDC * ES
  MOMENT = STRIP * (BMY*cos(β) - BMX*sin(β))
  ```
- **Wing coefficients** (lines 481–482): `CL_wing = np.add.reduceat(LIFT, span_breaks) / SURF` — vectorized across all wings using `span_breaks` from Module 2
- **Total vehicle coefficients** (lines 486–492):
  - `CL = Σ(LIFT) / Sref`
  - `CDi = Σ(DRAG) / Sref`
  - `CM = Σ(MOMENT) / (Sref * c_bar)`

---

### Module 9 — Compressibility, Control Surfaces, and Wake Coupling

**What you will learn:** Mach-number batching in the AIC; control surface deflection kinematics; transonic blending; how propeller/rotor slipstream couples into the VLM.

**Key files:**
- `trunk/SUAVE/Methods/Aerodynamics/Common/Fidelity_Zero/Lift/deflect_control_surface.py` — first 100 lines
- `trunk/SUAVE/Analyses/Aerodynamics/Vortex_Lattice.py` — lines 257–270 (transonic blending)
- `trunk/SUAVE/Methods/Aerodynamics/Common/Fidelity_Zero/Lift/compute_RHS_matrix.py` — lines 88–117 (slipstream)

**Key concepts:**
- **Mach batching**: `np.unique(mach, return_inverse=True)` → compute AIC once per unique Mach, broadcast: `C_mn = C_mn_small[inv, :, :, :]`. For 100 conditions at 3 distinct Mach numbers: 33x AIC speedup.
- **Compressibility in kernel**: `B2 = M^2 - 1` embedded in the Miranda formula — not a post-hoc Prandtl-Glauert correction
- **Control surface deflection**: Rodrigues' rotation formula applied to panel corners about the hinge-line vector. After deflection, `postprocess_VD()` must be called to recompute normals — enforced by the `is_postprocessed` guard.
- **Transonic blending** (`Cubic_Spline_Blender`): three separate surrogates blended via `h_sub(M)` and `h_sup(M)` weights active over [0.85, 0.95] and [1.05, 1.25] Mach bands. Transonic region uses `RegularGridInterpolator` (sparser grid → bivariate spline would not converge).
- **Lift rotor slipstream**: `r.Wake.evaluate_slipstream()` returns an axially/radially varying velocity field at VLM control points. The wing sees non-uniform inflow during transition — resolves into modified spanwise loading.

---

### Module 10 — The Surrogate Model

**What you will learn:** Why the surrogate exists; how the (alpha, Mach) training grid is defined; which interpolation methods are used; and how to validate surrogate accuracy.

**Key files:**
- `trunk/SUAVE/Analyses/Aerodynamics/Vortex_Lattice.py` — `initialize()` at line 129, `sample_training()`, `build_surrogate()`, `evaluate_surrogate()` at line 179

**Key concepts:**
- **Why**: A direct VLM call at 300 panels × 100 condition points takes ~0.1–1 s. A full mission may need thousands of evaluations. Surrogate reduces each lookup to a bivariate spline evaluation — 2–3 orders of magnitude faster.
- **Default training grid**:
  - Alpha: `[-5, -2, 0, 2, 5, 8, 10, 12, 45, 75]` degrees — high-alpha sentinels prevent unrealistic extrapolation at stall
  - Mach: 16 points from 0.0 to 3.5, denser in transonic
- **Subsonic/supersonic surrogates**: `RectBivariateSpline(alpha, mach, CL_table)` — cubic in both dimensions. Evaluate with `grid=False` (paired points, not meshgrid).
- **What the surrogate captures**: only `(alpha, Mach)`. It does NOT capture sideslip, rotation rates, or propeller wake effects.
- **For lift+cruise in transition**: set `use_surrogate = False`, or build a custom three-axis surrogate (alpha, Mach, rotor RPM).
- **Validation protocol**: run `evaluate_no_surrogate` at training points → compare against surrogate (should be near machine precision). Then test at midpoints between training rows to check interpolation quality.

---

## Recommended Study Practice

1. Use `tutorialLC.py` as the reference vehicle — it exercises the full VLM pipeline on a lift+cruise configuration
2. Set `use_surrogate = False` while studying Modules 3–8 to instrument the direct VLM path
3. For Module 5, **sketch the horseshoe vortex geometry on paper** with VD coordinate labels (`XAH`, `XBH`, `XC`, `phi`, `delta`) before reading `compute_wing_induced_velocity.py` — the frame rotation is the most confusing transformation in the codebase
4. After completing all modules, set a breakpoint at `np.linalg.solve` and trace one `GAMMA` value all the way through to the final `CL` to verify understanding end-to-end
