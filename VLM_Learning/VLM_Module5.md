# Module 5 — The AIC Matrix (Biot-Savart)

**Primary file:** `compute_wing_induced_velocity.py`
**Prerequisites:** Modules 3, 4

---

## Overview

`compute_wing_induced_velocity(VD, mach, compute_EW=False)` builds `C_mn` — the Aerodynamic Influence Coefficient tensor. `C_mn[k, i, j, :]` is the 3-component velocity vector induced at control point `j` by a horseshoe vortex of unit strength on panel `i`, at Mach number index `k`.

This is the most computationally expensive step in the VLM — it scales as O(n_cp²) in memory and O(n_cp² × n_mach) in computation.

---

## 1. The Horseshoe Vortex Model

Each panel contributes a **horseshoe vortex** consisting of:
1. A **bound vortex** along the quarter-chord line (from `[XAH,YAH,ZAH]` to `[XBH,YBH,ZBH]`)
2. Two **trailing vortex legs** extending to infinity downstream from each end of the bound vortex

The trailing legs are aligned with the freestream direction (the x-axis in body frame). The Biot-Savart law gives the velocity induced by each leg at an arbitrary receiving point.

The implementation uses the **Miranda et al. (1977)** formulation — a numerically stable kernel that handles the subsonic and supersonic cases with the same variable structure but different formulas.

---

## 2. Frame Rotation into the Horseshoe-Aligned Coordinate System

Before evaluating the kernel, the code rotates into a frame aligned with the bound vortex:

```python
# compute_wing_induced_velocity.py:120-131
theta    = np.arctan2(zb-za, yb-ya)   # dihedral angle of bound vortex
costheta = np.cos(theta)
sintheta = np.sin(theta)

# Rotate receiving point into horseshoe frame
xobar = (xo - xc)
yobar = (yo - yc)*costheta + (zo - zc)*sintheta
zobar =-(yo - yc)*sintheta + (zo - zc)*costheta
```

Here `(xc, yc, zc)` is the midpoint of the bound vortex. The rotation about the x-axis by angle `theta` transforms `(yo, zo)` so that the bound vortex lies in the new `y`-direction. After solving for `(U, V, W)` in the rotated frame, the result is rotated back:

```python
# compute_wing_induced_velocity.py:200
C_mn = np.stack([U, V*costheta - W*sintheta, V*sintheta + W*costheta], axis=-1)
```

This is the most confusing step — sketch it on paper. The rotation is a simple 2D rotation about x. The bound vortex segment always lies along the `ybar` axis in this frame, making the kernel formulas below apply.

---

## 3. Key Geometric Scalars

In the rotated horseshoe frame, `x1bar` and `y1bar` are the x and y distances from the bound vortex midpoint to its positive-y endpoint. From these, the key scalar `t` (tangent) and `s` (half-span) are defined:

```python
# compute_wing_induced_velocity.py:126-146
x1bar = (xb - xc)          # x-offset of one end of bound vortex
y1bar = (yb - yc)*costheta + (zb - zc)*sintheta  # y-offset (half-span in rotated frame)

s = np.abs(y1bar)           # half-span of the bound vortex
t = x1bar / y1bar           # tangent of bound vortex sweep

X1 = xobar + t*s            # x-distance from receiving point to left leg
Y1 = yobar + s              # y-distance from receiving point to left leg endpoint
X2 = xobar - t*s            # x-distance to right leg
Y2 = yobar - s              # y-distance to right leg endpoint
```

`t` encodes the sweep of the bound vortex. For a zero-sweep wing `t = 0`. For a swept wing the trailing legs are still x-aligned but the bound vortex is not perpendicular to the flow.

---

## 4. The Subsonic Kernel (Miranda et al.)

For `M < 1` (`B2 = M²−1 < 0`):

```python
# compute_wing_induced_velocity.py:260-285 (subsonic function)
RAD1 = sqrt(XSQ1 - RO1)   # RO1 = B2 * (Y1² + Z²)
RAD2 = sqrt(XSQ2 - RO2)

FB1  = (T*X1 - B2*Y1) / RAD1
FB2  = (T*X2 - B2*Y2) / RAD2
FT1  = (X1 + RAD1) / (RAD1 * RTV1)   # RTV1 = Y1² + Z²
FT2  = (X2 + RAD2) / (RAD2 * RTV2)

QB   = (FB1 - FB2) / DENOM  # DENOM = XTY² + (T²-B2)*Z²

U  =  Z/(4π) * QB
V  =  Z/(4π) * (FT1 - FT2 - QB*T)
W  = -(QB*XTY + FT1*Y1 - FT2*Y2) / (4π)
```

The Prandtl-Glauert compressibility correction is **embedded in the kernel** through `B2 = M²-1`. For subsonic flow `B2 < 0`, so `RO = B2*(Y²+Z²) < 0`, making `RAD = sqrt(X² - RO) = sqrt(X² + |B2|*(Y²+Z²))`. As `M→0`, `B2→-1` and the kernel reduces to the incompressible Biot-Savart formula.

This is *not* a post-hoc Prandtl-Glauert correction applied to CL — it is embedded in how the velocity field of each vortex is computed. This means the correction applies correctly to each panel's interaction with every other panel.

---

## 5. The Supersonic Kernel

For `M ≥ 1` (`B2 ≥ 0`), the kernel uses `CPI = 2π` instead of `4π`:

```python
# compute_wing_induced_velocity.py (supersonic function)
RAD1 = sqrt(XSQ1 - RO1)   # RO1 = B2*(Y1²+Z²), now RAD = sqrt(X² - B2*(Y²+Z²))
```

For supersonic flow `RAD` is real only inside the Mach cone (`X² > B2*(Y²+Z²)`). Outside the Mach cone, `RAD` is imaginary — these contributions are zeroed out via boolean masking (`bool1`, `bool2`).

**RFLAG — the sonic vortex flag:**
```python
# compute_wing_induced_velocity.py:447-448
TRANS = (B2 - T2F) * (B2 - T2A)
RFLAG[TRANS < 0] = 0
```

`RFLAG = 0` for panels whose horseshoe vortex is swept exactly parallel to the Mach cone (a "sonic vortex"). For these panels the kernel is singular; SUAVE zeroes the corresponding RHS entry and replaces the self-influence with an averaging condition (see `W[FLAG_bool_self] = 2`, `W[FLAG_bool_bef] = -1`, `W[FLAG_bool_aft] = -1`).

**WWAVE — the wave drag term:**
```python
# compute_wing_induced_velocity.py:457-463
WWAVE[B2_full > T2] = -0.5 * sqrt(B2_full - T2) / COX
W = W + WWAVE
```

This adds the wave pressure contribution from supersonic panels to their own self-influence.

---

## 6. Output: `C_mn` Shape and Meaning

```python
# compute_wing_induced_velocity.py:200
C_mn = np.stack([U, V*costheta - W*sintheta, V*sintheta + W*costheta], axis=-1)
# shape: [n_mach, n_cp, n_cp, 3]
```

- Axis 0: Mach number index (one slice per unique Mach number)
- Axis 1: sending panel index (which horseshoe vortex)
- Axis 2: receiving panel index (which control point)
- Axis 3: velocity component `[u, v, w]` in vehicle body frame

`C_mn[k, i, j, :]` = velocity at control point `j` from unit-strength horseshoe on panel `i` at Mach `k`.

---

## 7. `EW` — Normal-Component for LE Suction

When `compute_EW=True`, the function also returns:

```python
# compute_wing_induced_velocity.py:206-210
COS1 = np.cos(DL.T - DL)   # relative dihedral angle between panels
SIN1 = np.sin(DL.T - DL)
EW   = (W*COS1 - V*SIN1)   # normal velocity in VORLAX's horseshoe frame
```

`EW[k, i, j]` is the velocity component normal to the horseshoe plane at control point `j` induced by panel `i`. This is used in Module 7 to compute `CLE` — the total induced flow at the leading edge, which drives the LE suction magnitude.

---

## 8. Unique-Mach Batching

In `VLM.py`, the AIC is computed **only for unique Mach numbers**, then broadcast to all conditions:

```python
# VLM.py:249-255
m_unique, inv = np.unique(mach, return_inverse=True)
m_unique      = np.atleast_2d(m_unique).T
C_mn_small, s, RFLAG_small, EW_small = compute_wing_induced_velocity(VD, m_unique, compute_EW=True)

C_mn  = C_mn_small[inv, :, :, :]   # broadcast back to all conditions
RFLAG = RFLAG_small[inv, :]
EW    = EW_small[inv, :, :]
```

For surrogate training (160 conditions at 16 Mach values), this reduces 160 AIC evaluations to 16. The `return_inverse` index `inv` maps each condition back to its unique Mach.

---

## 9. Memory Budget

For a typical mesh with 15 spanwise × 5 chordwise = 75 panels per wing side, a 3-wing vehicle has ~450 panels. `C_mn` is then `[n_mach, 450, 450, 3]`. At `float32` (4 bytes), for 16 Mach values: `16 × 450 × 450 × 3 × 4 ≈ 39 MB`. For a finer mesh (30 × 10) on a 5-wing vehicle (~1500 panels): `16 × 1500 × 1500 × 3 × 4 ≈ 432 MB` — this grows quadratically and can exhaust RAM for fine meshes.

The `float32` default (`settings.floating_point_precision = np.float32`) is a deliberate memory trade-off.

---

## Summary — Key Concepts

| Concept | Detail |
|---------|--------|
| `C_mn[k,i,j,:]` | Velocity at CP `j` from unit vortex on panel `i` at Mach `k`; shape `[n_mach, n_cp, n_cp, 3]` |
| Frame rotation | Rotate into bound-vortex-aligned frame, solve `(U,V,W)`, rotate back |
| `t = x1bar/y1bar` | Tangent of bound vortex sweep — zero for unswept wings |
| `B2 = M²-1` in kernel | Prandtl-Glauert embedded in Biot-Savart, not a post-hoc correction |
| `RFLAG` | Zeros RHS for sonic vortices (`M ≥ 1`, swept parallel to Mach cone) |
| `EW` | Normal-velocity component in horseshoe frame — feeds LE suction calculation |
| Unique-Mach batching | `np.unique(mach)` reduces AIC evaluations; result broadcast via `inv` |

---

**Next:** Module 6 — The RHS Boundary Condition Vector (`compute_RHS_matrix.py`), where angle of attack, rotation rates, and propeller slipstream are assembled into the flow-tangency condition.
