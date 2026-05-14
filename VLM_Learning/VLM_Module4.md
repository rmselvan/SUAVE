# Module 4 — Panel Postprocessing

**Primary file:** `generate_VD_helpers.py`
**Prerequisites:** Module 3

---

## Overview

After the panel mesh is built, `postprocess_VD()` computes derived geometric quantities that are too expensive or inconvenient to recompute on every VLM solve. These include panel areas, unit normals, and the VORLAX-style geometric quantities (SLOPE, ZETA, chord length) that appear directly in the pressure coefficient and force integration formulas.

`postprocess_VD` is also the mechanism that **re-validates the VD** after any panel modification (e.g., control surface deflection). It sets `VD.is_postprocessed = True`; `VLM.py` raises an error if this flag is `False`.

---

## 1. When `postprocess_VD` Is Called

Two call sites:

1. **End of `generate_vortex_distribution`** (`generate_vortex_distribution.py:262`) — after all wings and control surfaces are discretized.
2. **End of `deflect_control_surfaces`** (`deflect_control_surface.py:51`) — any time control surface panel coordinates are updated by rotation.

Between the two calls to `deflect_control_surface`, `VD.is_postprocessed` is set to `False` to prevent stale postprocessed values from being used:
```python
# deflect_control_surface.py:260
VD.is_postprocessed = False
```

---

## 2. Panel Area Computation

Panel areas are computed in `compute_panel_area()`. Each panel is a quadrilateral (not necessarily planar), so the area is computed as the sum of two triangles:

```python
# generate_VD_helpers.py:130-136
P1P2 = [XB1-XA1, YB1-YA1, ZB1-ZA1]   # LE span vector (A to B)
P1P3 = [XA2-XA1, YA2-YA1, ZA2-ZA1]   # chord vector (LE to TE, side A)
P2P3 = [XA2-XB1, YA2-YB1, ZA2-ZB1]   # diagonal
P2P4 = [XB2-XB1, YB2-YB1, ZB2-ZB1]   # TE span vector

A_panel = 0.5 * (||P1P2 × P1P3|| + ||P2P3 × P2P4||)
```

The two cross products cover the two triangular halves of the quadrilateral. This is exact for planar quads and a good approximation for mildly warped ones.

---

## 3. Unit Normal Computation

Unit normals are computed in `compute_unit_normal()` using the same two vectors as the first triangle of the area calculation:

```python
# generate_VD_helpers.py:160-168
P1P2 = [XB1-XA1, YB1-YA1, ZB1-ZA1]
P1P3 = [XA2-XA1, YA2-YA1, ZA2-ZA1]
cross = P1P2 × P1P3
unit_normal = cross / ||cross||
```

**Sign convention:** All normals must point in the +Z direction (upward). Any panel whose computed normal has a negative Z component gets flipped:

```python
# generate_VD_helpers.py:167-168
unit_normal[unit_normal[:,2]<0, :] = -unit_normal[unit_normal[:,2]<0, :]
```

This handles inverted panels (e.g., downward-dihedral winglets, inverted tail surfaces). The normals feed directly into the RHS boundary condition in Module 6.

---

## 4. SLOPE — Camber Surface Inclination

`SLOPE` is the tangent of the camber surface angle at each panel's midpoint:

```python
# generate_VD_helpers.py:58-62
X1c   = (VD.XA1 + VD.XB1) / 2   # midpoint of LE edge
X2c   = (VD.XA2 + VD.XB2) / 2   # midpoint of TE edge
Z1c   = (VD.ZA1 + VD.ZB1) / 2
Z2c   = (VD.ZA2 + VD.ZB2) / 2
SLOPE = (Z2c - Z1c) / (X2c - X1c)
```

For a flat, untilted panel `SLOPE = 0`. For a panel with camber or angle of attack pre-built into the mesh, `SLOPE` captures that inclination. SLOPE appears in the pressure coefficient formula and in the force decomposition:
- In `VLM.py`: `TX = SLOPE - ZETA` — the panel slope relative to the strip chord line
- `CAXL = -SINF * TX / (1 + TX²)` — chordwise force component from the loading

`SLE = SLOPE[LE_ind]` stores the SLOPE values at the leading-edge panels only (one per strip), used in the LE suction computation.

---

## 5. ZETA — Strip Tangent Incidence Angle

`ZETA` (called `tangent_incidence_angle` in VD) is the tangent of the angle between the strip's chord line and the vehicle x-axis, computed using the **strip's LE and TE control point positions**:

```python
# generate_VD_helpers.py:67-71
LE_X          = X1c[LE_ind]   # x-coordinate of LE midpoint
LE_Z          = Z1c[LE_ind]   # z-coordinate of LE midpoint
TE_X          = X2c[TE_ind]   # x-coordinate of TE midpoint
TE_Z          = Z2c[TE_ind]   # z-coordinate of TE midpoint
tan_incidence = np.repeat((LE_Z - TE_Z) / (LE_X - TE_X), strip_n_cw)
```

`ZETA` captures the effect of local wing twist and camber on the effective angle each strip makes with the flow. It is used to decompose panel normal forces into body-axis components in Module 8:

```python
# VLM.py:444-451
FCOS = np.cos(ZETA)   # cos of strip chord angle
FSIN = np.sin(ZETA)   # sin of strip chord angle
BFX  = -CNC*FSIN + CAXL*FCOS   # x-body force
BFZ  =  (CNC*FCOS + CAXL*FSIN)*COD  # z-body force
```

---

## 6. CHORD — Adjusted Chord Length per Strip

The adjusted chord length is the actual distance from the LE control point to the TE control point along the strip centerline:

```python
# generate_VD_helpers.py:72
chord_adjusted = np.repeat(
    np.sqrt((TE_X-LE_X)**2 + (TE_Z-LE_Z)**2),
    strip_n_cw
)
```

Note this is the **arc length along the camber surface** (well, the straight-line distance between the LE and TE midpoints), not the projected chord. For small camber these are nearly equal. This chord value is used as the normalization length in the pressure coefficient (`CHORD` in `VLM.py`).

---

## 7. Trailing-Edge Coordinate Broadcasting

`postprocess_VD` also pre-broadcasts the trailing-edge panel coordinates to every panel in each strip:

```python
# generate_VD_helpers.py:74-83
XC_TE_wings = np.repeat(VD.XC[TE_ind], strip_n_cw)
YC_TE_wings = np.repeat(VD.YC[TE_ind], strip_n_cw)
...
```

These are stored as `VD.XC_TE`, `VD.YC_TE`, etc. They're used in `VLM.py` to compute `CORMED` — the x-distance from each panel's load point to the trailing edge — which feeds into the strip rolling-moment contribution from sideslip.

---

## 8. The `is_postprocessed` Guard

```python
# generate_VD_helpers.py:105
VD.is_postprocessed = True
```

`VLM.py:190-191` checks this:
```python
if not VD.is_postprocessed:
    raise ValueError('postprocess_VD has not been called since the panels have been modified')
```

This is a deliberate safety net — if you manually modify `VD.XA1` (e.g., during deflection), you must re-call `postprocess_VD` before solving. Forgetting this would silently produce wrong forces because normals and chord lengths would be stale.

---

## 9. `D` — Bound Vortex Half-Span

```python
# generate_VD_helpers.py:64
D = np.sqrt((VD.YAH - VD.YBH)**2 + (VD.ZAH - VD.ZBH)**2)[LE_ind]
```

`D` is the full spanwise length of the bound vortex segment for each strip's LE panel. It equals the strip width for planar wings. It is used in Module 5 to normalize the Biot-Savart kernel, and appears in the force integration as `ES = 2*s[0, LE_ind]` (the strip span, where `s` is the half-span returned by `compute_wing_induced_velocity`).

---

## Summary — What `postprocess_VD` Computes

| Variable | Location in VD | Used in |
|----------|---------------|---------|
| `panel_areas` | `VD.panel_areas` | Force integration (Module 8) |
| `normals` | `VD.normals` | RHS boundary condition (Module 6) |
| `SLOPE` | `VD.SLOPE` | Pressure coefficient, axial force (Module 7) |
| `SLE` | `VD.SLE` | LE suction term (Module 7) |
| `D` | `VD.D` | Bound vortex half-span (Module 5, 8) |
| `tangent_incidence_angle` (ZETA) | `VD.tangent_incidence_angle` | Force decomposition (Module 8) |
| `chord_lengths` | `VD.chord_lengths` | Pressure coefficient (Module 7) |
| `XC_TE`, `YC_TE`, `ZC_TE` | `VD.XC_TE` etc. | Moment arm (Module 8) |
| `Y_SW` | `VD.Y_SW` | Wing y-coordinate (Module 8) |
| `is_postprocessed` | `VD.is_postprocessed` | Safety guard (VLM.py) |

---

**Next:** Module 5 — The AIC Matrix (`compute_wing_induced_velocity.py`), where the Biot-Savart law is applied to compute the velocity induced at every control point by every horseshoe vortex.
