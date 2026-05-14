# Module 9 — Compressibility, Control Surfaces, and Wake Coupling

**Primary files:** `deflect_control_surface.py`, `VLM.py`, `compute_RHS_matrix.py`
**Prerequisites:** Modules 5–8

---

## Overview

This module examines three cross-cutting concerns that don't fit neatly into the linear solve pipeline but that significantly affect results for real vehicles:

1. **Mach batching** — how compressibility is handled efficiently across many flight conditions
2. **Control surface deflection kinematics** — how ailerons, elevators, flaps, and all-moving surfaces physically rotate panels in the VD
3. **Propeller/rotor slipstream coupling** — how the lift rotor wake from a lift+cruise vehicle enters the wing's VLM

---

## Part 1: Compressibility and Mach Batching

### 1.1 Prandtl-Glauert in the Kernel (Recap)

As established in Module 5, compressibility is embedded in the Biot-Savart kernel via `B2 = M²-1`. There is **no separate Prandtl-Glauert correction step** applied to CL or CD. The corrected aerodynamics arise naturally from the corrected induced velocity field.

For subsonic flow (`B2 < 0`), the kernel uses `RAD = sqrt(X² + |B2|(Y²+Z²))`, which stretches the spanwise and vertical distances by `1/√(1-M²)` — equivalent to Gothert's rule applied to the influence function.

For supersonic flow (`B2 > 0`), the kernel zeros contributions from outside the Mach cone, implementing the supersonic lifting surface theory.

### 1.2 Unique-Mach Batching in `VLM.py`

The AIC matrix depends only on Mach number and panel geometry — not on angle of attack, sideslip, or rotation rates. SUAVE exploits this by computing `C_mn` only once per unique Mach value:

```python
# VLM.py:249-255
m_unique, inv = np.unique(mach, return_inverse=True)
m_unique      = np.atleast_2d(m_unique).T

C_mn_small, s, RFLAG_small, EW_small = compute_wing_induced_velocity(VD, m_unique, compute_EW=True)

C_mn  = C_mn_small[inv, :, :, :]   # broadcast all n_conditions
RFLAG = RFLAG_small[inv, :]
EW    = EW_small[inv, :, :]
```

**Example:** Surrogate training runs 160 conditions (10 AoA × 16 Mach). `np.unique` finds 16 unique Mach values. `compute_wing_induced_velocity` runs 16 times instead of 160. The `inv` array of shape `[160]` then indexes into `C_mn_small[16, ...]` to produce `C_mn[160, ...]`.

This is a ~10× speedup for surrogate training at no accuracy cost, since multiple AoA at the same Mach share the same AIC.

### 1.3 RFLAG: Zeroing Sonic Vortex RHS

```python
# VLM.py:258
RHS = RHS * RFLAG
```

For panels whose horseshoe is swept parallel to the Mach cone (`RFLAG = 0`), the RHS entry is zeroed before the solve. The supersonic kernel then replaces the self-influence for that row with an averaging condition (`W_self = 2`, `W_before = -1`, `W_after = -1`). This ensures the sonic vortex strength is the average of its neighbors — a regularization that prevents the solution from diverging at the sonic condition.

---

## Part 2: Control Surface Deflection Kinematics

### 2.1 When Deflection Occurs

Control surface panels are placed at zero deflection during `generate_vortex_distribution`. Deflection is then applied as a post-step:

```python
# generate_vortex_distribution.py:252-256
for wing in VD.VLM_wings:
    wing_is_all_moving = (not wing.is_a_control_surface) and issubclass(wing.wing_type, All_Moving_Surface)
    if wing.is_a_control_surface or wing_is_all_moving:
        VD, wing = deflect_control_surface(VD, wing)
```

This happens once at initialization. For missions with varying deflection (e.g., elevator trim throughout climb), `deflect_control_surfaces()` is called each time `wing.deflection` changes.

### 2.2 The Quaternion Rotation

Every panel in the control surface is rotated about its hinge line using a 3D rotation matrix (expressed as a quaternion for generality):

```python
# deflect_control_surface.py:404-418
quaternion = make_hinge_quaternion(wing.hinge_root_point, wing.hinge_vector, delta_deflection)

xi_prime_a1, y_prime_a1, zeta_prime_a1 = rotate_points_with_quaternion(
    quaternion, [xi_prime_a1, y_prime_a1, zeta_prime_a1])
# ... repeated for all 12 sets of panel coordinates ...
```

The quaternion `make_hinge_quaternion(point, direction, angle)` builds the 4×4 rotation+translation matrix for rotation by `angle` radians about the line through `point` in direction `direction`:

```python
# deflect_control_surface.py:502-520
q11 = u**2 + (v**2 + w**2)*cos
q12 = u*v*(1-cos) - w*sin
# ... Rodrigues' rotation formula entries ...
quat = np.array([[q11, q12, q13, q14],
                 [q21, q22, q23, q24],
                 [q31, q32, q33, q34],
                 [0.,  0.,  0.,  1. ]])
```

This is the standard Rodrigues' rotation formula extended to handle rotation about an arbitrary line (not passing through the origin), which is why the translation terms `q14`, `q24`, `q34` appear.

### 2.3 Finding the Hinge Line

The hinge line is computed from the **first strip** of each control surface only — this is a deliberate approximation that a control surface has a single straight hinge:

```python
# deflect_control_surface.py:357-386
if is_first_strip:
    ib_le_strip_corner = [xi_prime_a1[0], y_prime_a1[0], zeta_prime_a1[0]]
    ib_te_strip_corner = [xi_prime_a2[-1], y_prime_a2[-1], zeta_prime_a2[-1]]
    ib_hinge_point = np.interp(interp_fractions, interp_domains, interp_ranges_ib)

    ob_hinge_point = np.interp(...)
    hinge_vector = (ob_hinge_point - ib_hinge_point) / ||ob_hinge_point - ib_hinge_point||

    wing.hinge_root_point = ib_hinge_point
    wing.hinge_vector     = hinge_vector
```

`interp_fractions = [0., 2., 4.] + wing.hinge_fraction` interpolates along the chord at `hinge_fraction` (0 = LE, 1 = TE) to find the inboard and outboard hinge points.

### 2.4 Delta Deflection and Sign Conventions

The code applies a **delta deflection** — the difference from the last applied angle — rather than absolute deflection:

```python
# deflect_control_surface.py:396-401
ddeflection      = wing.deflection - wing.deflection_last
slat_multiplier  = (1 - wing.is_slat) - wing.is_slat      # -1 for slats (deflect LE-up)
sym_multiplier   = (1 - (sym_sign==-1)) - wing.sign_duplicate*(sym_sign==-1)
ver_multiplier   = (1 - vertical_wing) - 1*vertical_wing
delta_deflection = slat_multiplier * sym_multiplier * ver_multiplier * ddeflection
```

Sign convention: **positive deflection follows the right-hand rule about the outboard-pointing hinge vector**. For a standard aileron on the right wing, positive deflection deflects the TE downward. For the mirrored left-wing aileron, `sign_duplicate` controls whether it deflects in the same or opposite direction (same for flaps, opposite for ailerons in roll).

### 2.5 Must Re-call `postprocess_VD`

After rotating all panel coordinates, `postprocess_VD` must be called to recompute normals, chord lengths, SLOPE, ZETA, etc.:

```python
# deflect_control_surface.py:51
VD = postprocess_VD(VD, settings)
```

The `is_postprocessed = False` flag set before the rotation loop ensures that any attempt to use the VD without this step raises an error.

### 2.6 Vertical Wing Special Case

For a vertical surface (rudder, all-moving vertical tail), the y↔z axes are swapped before deflection and swapped back after:

```python
# deflect_control_surface.py:339-348
y_prime_a1, zeta_prime_a1 = flip_1(y_prime_a1, zeta_prime_a1, vertical_wing, inverted_wing)
# ... deflect in horizontal-wing frame ...
y_prime_a1, zeta_prime_a1 = flip_2(y_prime_a1, zeta_prime_a1, vertical_wing, inverted_wing)
```

This allows the same quaternion rotation code to handle both horizontal and vertical surfaces without branching the deflection math.

---

## Part 3: Propeller/Rotor Slipstream Wake Coupling

### 3.1 Where Slipstream Enters

The propeller and lift rotor slipstream is injected into the RHS in `compute_RHS_matrix.py`:

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

`evaluate_slipstream(rotor, geometry, n_ctrl_pts)` returns `[n_conditions, n_cp, 3]` — the 3-component induced velocity from the rotor wake at every VLM control point for every flight condition.

### 3.2 Physical Effect

In hover or low-speed transition, the lift rotors produce a downwash of approximately `v_h = sqrt(T/(2ρA))` through their disk. Any wing panels inside or behind this slipstream see:
- Increased dynamic pressure (higher local velocity magnitude)
- Altered effective angle of attack (slipstream has a predominantly downward velocity component)

Without `propeller_wake_model = True`, the VLM assumes uniform freestream at all control points — this is accurate in cruise but produces large errors during transition or when the rotors are positioned upstream of the wing.

### 3.3 Why the Surrogate Cannot Capture This

The surrogate (Module 10) is trained at zero rotation rate and zero slipstream. The training grid is only over `(alpha, Mach)`. A propeller slipstream:
1. Varies with rotor thrust (RPM, collective)
2. Varies with freestream velocity (slipstream contraction ratio changes)
3. Has a non-uniform spatial distribution across wing panels

None of these can be encoded in a 2D `(alpha, Mach)` surrogate. This is the fundamental reason **`use_surrogate = False` is required for lift+cruise transition analysis**.

### 3.4 Slipstream Effect on `RHS` and `GAMMA`

After `Vx_ind_total`, `Vy_ind_total`, `Vz_ind_total` are added in:
```python
# compute_RHS_matrix.py:230-235
Vx = V_distribution*cos(aoa)*cos(PSI) + Vx_rotation + Vx_ind_total
Vy = V_distribution*cos(aoa)*sin(PSI) + Vy_rotation + Vy_ind_total
Vz = V_distribution*sin(aoa)          + Vz_rotation + Vz_ind_total

aoa_distribution = arctan(Vz / sqrt(Vx² + Vy²))   # updated local AoA
PSI_distribution = arctan(Vy / Vx)                  # updated local sideslip
```

The effective local angle of attack at each panel is recalculated after adding the wake velocity. This is what creates the non-uniform lift distribution across the wing in transition — strips in the rotor slipstream have a higher effective AoA than strips in clean freestream.

---

## Summary — Key Concepts

| Topic | Key Detail |
|-------|-----------|
| `np.unique(mach)` batching | AIC computed once per unique Mach; broadcast via `inv` |
| `RHS *= RFLAG` | Zeros RHS for sonic vortices before solve |
| Quaternion rotation | Rodrigues' formula about arbitrary hinge line; applied as delta deflection |
| `deflection_last` | Enables delta deflections; avoids re-deflecting from zero each time |
| `postprocess_VD` after deflection | Mandatory — normals and chord lengths become stale after rotation |
| `sign_duplicate` | Controls symmetric deflection sign (same for flaps, opposite for ailerons) |
| Vertical wing flip | y↔z swap before/after deflection so horizontal-wing math applies |
| `evaluate_slipstream` → `Vx/Vy/Vz_ind_total` | Wake velocity added to onset velocity at each VLM panel |
| Why surrogate fails for lift+cruise | Slipstream depends on rotor thrust, not just (alpha, Mach) |

---

**Next:** Module 10 — The Surrogate Model, covering why the surrogate exists, how the (alpha, Mach) training grid is set up, how `RectBivariateSpline` and `RegularGridInterpolator` are used, and how to validate surrogate accuracy.
