# Module 6 — The RHS Boundary Condition Vector

**Primary file:** `compute_RHS_matrix.py`
**Prerequisites:** Modules 3, 4

---

## Overview

`compute_RHS_matrix()` assembles the right-hand side vector `RHS` of the linear system `A * GAMMA = RHS`. Each entry `RHS[condition, panel]` is the component of the total onset velocity normal to that panel's surface. The linear solve then finds the vortex strengths GAMMA that cancel this normal velocity (flow-tangency condition).

The function also computes several auxiliary arrays (`ONSET`, `YGIRO`, `ZGIRO`, `VX`, `SCNTL`, `CCNTL`, `COD`, `SID`) that are passed forward for use in the pressure coefficient and LE suction calculations in Module 7.

---

## 1. The Flow-Tangency Condition

The VLM boundary condition is:

> At every control point, the total velocity (freestream + induced by all vortices + wake + rotation) must be tangent to the wing surface.

Equivalently, its normal component must be zero. The RHS encodes the normal component of the **non-vortex** velocity at each control point. The AIC matrix encodes how GAMMA distributes the vortex-induced velocity. Solving `A*GAMMA = RHS` finds the GAMMA values that produce exactly the required normal velocity to cancel the onset flow.

---

## 2. The Two RHS Modes

`build_RHS` supports two boundary condition formulations, selectable via `settings.use_VORLAX_matrix_calculation`:

### Default Mode: Body-Frame dot product

```python
# compute_RHS_matrix.py:230-240
Vx = V_distribution * cos(aoa) * cos(PSI) + Vx_rotation + Vx_ind_total
Vy = V_distribution * cos(aoa) * sin(PSI) + Vy_rotation + Vy_ind_total
Vz = V_distribution * sin(aoa)            + Vz_rotation + Vz_ind_total

V_unit_vector    = np.array([Vx, Vy, Vz]) / V_distribution
panel_normals    = VD.normals[:, np.newaxis, :]
RHS_from_normals = sum(V_unit_vector * panel_normals, axis=2).T
```

This is the standard VLM formulation: `RHS = V̂ · n̂` where `V̂` is the unit onset velocity vector (including rotation and wake effects) and `n̂` is the panel unit normal. The division by `V_distribution` non-dimensionalizes the RHS.

### VORLAX Mode: ALOC formula

```python
# compute_RHS_matrix.py:218
ALOC = VX*SCNTL + VY*CCNTL*SID - VZ*CCNTL*COD
```

VORLAX computes the boundary condition using the camber slope (`SCNTL`, `CCNTL`) and dihedral angle (`COD`, `SID`) of the panel. This is mathematically equivalent to the dot product for the specific case of the VORLAX panel geometry, but note: **VORLAX does not include wake-induced velocities in the RHS** (it uses only freestream and rotation).

Set `use_VORLAX_matrix_calculation = True` when comparing SUAVE results to VORLAX output digit-for-digit.

---

## 3. Freestream Velocity Decomposition

The freestream velocity in body axes is:

```python
# compute_RHS_matrix.py:205-207
VX = COSCOS - PITCH*ZGIRO + YAW  *YGIRO   # x-component (non-dim)
VY = COSIN  - YAW  *XGIRO + ROLL *ZGIRO   # y-component (non-dim)
VZ = SINALF - ROLL *YGIRO + PITCH*XGIRO   # z-component (non-dim)
```

Where:
- `COSCOS = cos(α) * cos(β)` — freestream x-component
- `COSIN  = cos(α) * sin(β)` — freestream y-component (sideslip)
- `SINALF = sin(α)` — freestream z-component

These are the normalized (by `V∞`) velocity components at the **rotation center** (`XBAR`, `ZBAR`). The rotation rate terms (`PITCH*ZGIRO`, etc.) add the additional velocity due to rigid-body rotation at the specific panel location.

For the body-frame mode, the dimensional version is:
```python
# compute_RHS_matrix.py:230-232
Vx = V_distribution * cos(aoa) * cos(PSI) + Vx_rotation + Vx_ind_total
Vy = V_distribution * cos(aoa) * sin(PSI) + Vy_rotation + Vy_ind_total
Vz = V_distribution * sin(aoa)            + Vz_rotation + Vz_ind_total
```

---

## 4. The `XGIRO`, `YGIRO`, `ZGIRO` Rotation Reference

The rotation is measured relative to the reference point `(XBAR, ZBAR)` (the moment reference point, computed in `VLM.py` as the aerodynamic center or CG):

```python
# compute_RHS_matrix.py:198-200
XGIRO = X + CHORD*DELTAX - repeat(XBAR, RNMAX[LE_ind])
YGIRO = YY                                              # y-coord of control point
ZGIRO = ZZ - repeat(ZBAR, RNMAX[LE_ind])               # z-offset from ref point
```

`DELTAX = 0.5/RNMAX` shifts the x-reference from the strip LE to the midpoint of the first panel. These relative coordinates are then used in:
- The rotation-rate velocity contributions: `Vx_rotation = -PITCHQ*ZGIRO + YAWQ*YGIRO`
- `ONSET` (saved for Module 7): `ONSET = -PITCH*ZGIRO + YAW*YGIRO`

---

## 5. Rotation Rate Contributions

Body rotation rates (pitch `q`, roll `p`, yaw `r`) add to the velocity at each panel:

```python
# compute_RHS_matrix.py:226-228
Vx_rotation = -PITCHQ*ZGIRO + YAWQ  *YGIRO
Vy_rotation = -YAWQ  *XGIRO + ROLLQ *ZGIRO
Vz_rotation = -ROLLQ *YGIRO + PITCHQ*XGIRO
```

This is the classic formula for the velocity at a point in a rigidly rotating body: `v = ω × r`, where `ω = [p, q, r]` and `r = [XGIRO, YGIRO, ZGIRO]`. For trim analysis at steady flight with no rotation these terms are zero. For stability derivatives (e.g., `CLq`, `CMq`) these terms are the entire source of the derivative.

---

## 6. Propeller/Rotor Slipstream Injection

For lift+cruise vehicles with `settings.propeller_wake_model = True`:

```python
# compute_RHS_matrix.py:88-112
for network in geometry.networks:
    if propeller_wake_model:
        if 'propellers' in network.keys():
            for p in props:
                prop_V_wake_ind += p.Wake.evaluate_slipstream(p, geometry, num_ctrl_pts)
        if 'lift_rotors' in network.keys():
            for r in rots:
                rot_V_wake_ind += r.Wake.evaluate_slipstream(r, geometry, num_ctrl_pts)

Vx_ind_total = prop_V_wake_ind[:,:,0] + rot_V_wake_ind[:,:,0]
Vy_ind_total = prop_V_wake_ind[:,:,1] + rot_V_wake_ind[:,:,1]
Vz_ind_total = prop_V_wake_ind[:,:,2] + rot_V_wake_ind[:,:,2]
```

`evaluate_slipstream()` returns a `[n_conditions, n_cp, 3]` array of induced velocities at every VLM control point from the propeller/rotor actuator disk or vortex wake model. These are added to the freestream velocity before computing the RHS.

**This is the coupling mechanism between the rotor aerodynamics and the VLM wing aerodynamics.** Without this, the wing sees only undisturbed freestream; with it, the wing sees the accelerated slipstream, producing higher local dynamic pressure and altered angles of attack within the slipstream. This is why `use_surrogate = False` is required for lift+cruise transition analysis — the surrogate was trained without slipstream, so it cannot capture this effect.

---

## 7. The `SCNTL`, `CCNTL`, `COD`, `SID` Direction Cosines

These four arrays encode the panel orientation in the VORLAX frame:

```python
# compute_RHS_matrix.py:210-214
SCNTL  = VD.SLOPE / sqrt(1. + VD.SLOPE**2)    # sin of panel camber angle
CCNTL  = 1. / sqrt(1.0 + SCNTL**2)            # cos of panel camber angle
phi_LE = repeat(phi[:, LE_ind], RNMAX[LE_ind]) # dihedral angle, broadcast to all panels
COD    = cos(phi_LE)                           # cos of dihedral
SID    = sin(phi_LE)                           # sin of dihedral
```

These are saved in the `rhs` Data object and passed forward to Module 7 for use in:
- `EFFINC` (effective incidence at LE for LE suction)
- `ALOC` (VORLAX RHS formula)
- Force decomposition (`BFX`, `BFY`, `BFZ`)

---

## 8. `ONSET` — The Rigid-Body Rotation x-Velocity

```python
# compute_RHS_matrix.py:222
ONSET = -PITCH*ZGIRO + YAW*YGIRO
```

`ONSET` is the x-component of the velocity at each panel due to rigid-body rotation alone (no freestream, no sideslip). It is used in Module 7 as part of the `FACTOR` term in the pressure coefficient:

```python
# VLM.py:325
FACTOR = FORAXL + ONSET   # FORAXL = cos(α)*cos(β)
```

This captures how pitch or yaw rate modifies the effective dynamic pressure at each panel.

---

## 9. What `compute_RHS_matrix` Returns

```python
rhs.RHS            # [n_conditions, n_cp] — the boundary condition vector
rhs.ONSET          # [n_conditions, n_cp] — x-velocity from rotation
rhs.Vx_ind_total   # [n_conditions, n_cp] — x-slipstream velocity
rhs.Vz_ind_total   # [n_conditions, n_cp] — z-slipstream velocity
rhs.V_distribution # [n_conditions, n_cp] — freestream speed at each panel
rhs.YGIRO          # [n_cp]  — y-distance from rotation center
rhs.ZGIRO          # [n_cp]  — z-distance from rotation center
rhs.VX             # [n_conditions, n_cp] — x-onset velocity (normalized)
rhs.SCNTL, CCNTL   # [n_cp]  — panel camber direction cosines
rhs.COD, SID       # [n_conditions, n_cp] — panel dihedral direction cosines
```

---

## Summary — Key Concepts

| Concept | Detail |
|---------|--------|
| `RHS = V̂ · n̂` | Normal component of total onset velocity at each panel (default mode) |
| `ALOC` formula | VORLAX equivalent — uses `SCNTL`, `CCNTL`, `COD`, `SID` directly |
| `COSCOS`, `COSIN`, `SINALF` | Freestream decomposition: `cos(α)cos(β)`, `cos(α)sin(β)`, `sin(α)` |
| `XGIRO`, `YGIRO`, `ZGIRO` | Coordinates relative to moment reference center |
| Rotation rates | `Vx_rot = -q·z + r·y`, etc. — zero for steady trimmed flight |
| Slipstream injection | `p.Wake.evaluate_slipstream()` → added to velocity at each CP |
| `ONSET` | x-component of rotation velocity; saved for pressure coefficient |

---

**Next:** Module 7 — Linear Solve and Pressure Coefficient (`VLM.py:260–421`), where `A*GAMMA = RHS` is solved and `DCP` is assembled from GAMMA, ONSET, and the leading-edge suction term.
