# Module 7 — Linear Solve and Pressure Coefficient

**Primary file:** `VLM.py:260–421`
**Prerequisites:** Modules 5, 6

---

## Overview

With `C_mn` (AIC) and `RHS` in hand, Module 7 covers:
1. **AIC assembly** — projecting `C_mn` onto panel normals to form matrix `A`
2. **Linear solve** — `GAMMA = np.linalg.solve(A, RHS)`
3. **Pressure coefficient** — `DCP` from GAMMA, ONSET, sideslip, and LE suction

This is the mathematical core of the VLM.

---

## 1. AIC Assembly: From `C_mn` to Matrix `A`

`C_mn[k, i, j, :]` is the 3-component velocity at control point `j` from unit vortex on panel `i`. The AIC matrix `A[k, j, i]` is the **normal component** of that velocity:

```python
# VLM.py:263-265
A =   np.multiply(C_mn[:,:,:,0], np.atleast_3d(np.sin(delta)*np.cos(phi))) \
    + np.multiply(C_mn[:,:,:,1], np.atleast_3d(np.cos(delta)*np.sin(phi))) \
    - np.multiply(C_mn[:,:,:,2], np.atleast_3d(np.cos(phi)*np.cos(delta)))
```

Where:
- `delta` = mean camber surface angle at the control point (`arctan((ZC - ZCH)/(XC - XCH))`)
- `phi` = dihedral angle at the control point (`arctan((ZBC-ZAC)/(YBC-YAC))`)

This is the dot product of the induced velocity with the panel normal expressed through `delta` and `phi`:

```
n̂ = [sin(δ)cos(φ),  cos(δ)sin(φ),  -cos(φ)cos(δ)]
```

Validated against Katz & Plotkin equation 7.42.

**VORLAX mode alternative:** When `use_VORLAX_matrix_calculation = True`:
```python
# VLM.py:267
A = EW   # EW is the normal-component in the horseshoe frame computed in Module 5
```

---

## 2. The Linear Solve

```python
# VLM.py:270
GAMMA = np.linalg.solve(A, RHS)
```

- `A` is `[n_conditions, n_cp, n_cp]` — a dense square matrix per condition
- `RHS` is `[n_conditions, n_cp]`
- `GAMMA` is `[n_conditions, n_cp]` — the vortex strength on each panel

`np.linalg.solve` uses LAPACK's `dgesv` (LU decomposition). For 450 panels: `A` is 450×450 and the solve takes a few milliseconds. For 1500 panels it takes ~100ms. The system is dense — every panel influences every other panel.

**Physical meaning of GAMMA:** The bound circulation of each horseshoe vortex in units of `V∞ * [length]`. By Kutta-Joukowski, `L = ρ * V∞ * Γ` per unit span, so larger GAMMA → stronger lift on that strip.

---

## 3. Free-Stream and Onset Parameters

Before computing DCP, freestream parameters are cached as `[n_conditions, n_cp]` arrays:

```python
# VLM.py:281-291
B2     = np.tile((mach**2 - 1), n_cp)    # Prandtl-Glauert factor (per panel)
SINALF = np.sin(aoa)
COSALF = np.cos(aoa)
COSCOS = COSALF * COPSI                   # cos(α)cos(β) — x-freestream
FORLAT = COSALF * SINPSI * 2.0           # sideslip factor (2x for moment later)
FACTOR = FORAXL + ONSET                   # total effective x-velocity
```

`FACTOR = cos(α)cos(β) + ONSET` is used to non-dimensionalize GAMMA into pressure:
- For zero rotation (`ONSET = 0`): `FACTOR = cos(α)cos(β) ≈ 1` at small angles.
- For pitch rate: `ONSET = -q*z/V∞` adds a position-dependent correction.

---

## 4. Cumulative Chordwise Circulation: `GANT`

The Kutta-Joukowski theorem relates lift on a panel to the **difference** in circulation from one chordwise panel to the next:

```python
# VLM.py:316-319
GFX  = np.tile(1/CHORD, (len_mach, 1))         # 1/chord per panel
GANT = strip_cumsum(GFX * GAMMA, chord_breaks, RNMAX[LE_ind])
GANT = np.roll(GANT, 1)
GANT[:, LE_ind] = 0                             # reset at each strip's LE
```

`GANT` is the cumulative sum of `GAMMA/CHORD` from the leading edge to the current panel. Rolling by 1 and zeroing at LE gives the circulation accumulated **ahead** of the current panel — i.e., the circulation from all upstream panels in the same strip.

This feeds into the sideslip-induced pressure term `DCPSID`:
```python
# VLM.py:321-324
GLAT   = GANT*(TANA - TANB) - GFX*GAMMA*TANB
DCPSID = FORLAT * cos_DL * GLAT / (XIB - XIA)
```

`DCPSID` accounts for the lateral (sideslip) component of the swept-panel loading.

---

## 5. The Pressure Coefficient

```python
# VLM.py:328-331
GNET = GAMMA * FACTOR
GNET = GNET * RNMAX / CHORD      # normalize: multiply by n_cw/chord
DCP  = 2 * GNET + DCPSID
CP   = DCP
```

Breaking this down:
- `GAMMA * FACTOR` scales by the effective dynamic pressure direction
- `RNMAX / CHORD` = `n_cw / chord` normalizes by the panel chord length (`1/RNMAX` is one panel's chord fraction)
- `2 * GNET` is the standard VLM pressure coefficient (factor of 2 from the standard definition `Cp = ΔP / q∞`)
- `+ DCPSID` adds the sideslip correction

This matches the VORLAX `PRESS` subroutine formula.

---

## 6. Strip Panel Geometry for Force Decomposition

The panel sweep angles at the LE and TE are interpolated for each panel in the strip:

```python
# VLM.py:301-313
TAN_LE = (XB1[LE_ind] - XA1[LE_ind]) / sqrt((ZB1-ZA1)² + (YB1-YA1)²)  # LE sweep tangent
TAN_TE = (XB_TE - XA_TE) / sqrt((ZB_TE-ZA_TE)² + (YB_TE-YA_TE)²)      # TE sweep tangent

XIA = (RK-1) / RNMAX   # fraction of chord at panel leading edge
XIB = (RK  ) / RNMAX   # fraction of chord at panel trailing edge

TANA = TAN_LE*(1 - XIA) + TAN_TE*XIA    # interpolated sweep at panel LE
TANB = TAN_LE*(1 - XIB) + TAN_TE*XIB    # interpolated sweep at panel TE
```

`TANA` and `TANB` are the local sweep angles at the inboard and outboard edges of each panel. For a straight wing `TANA = TANB`. For a highly swept wing they differ significantly panel-to-panel.

These feed into `DCPSID` (above) and the LE suction calculation below.

---

## 7. Leading-Edge Suction

The LE suction captures the Kutta suction peak at the leading edge — the thrust component that arises from the leading-edge singularity in thin airfoil theory.

**Step 1:** Compute sweep parameter `STB`:
```python
# VLM.py:350-352
T2  = TLE * TLE          # tan²(LE sweep)
STB = sqrt(T2 - B2_LE)   # where B2_LE < T2 (subsonic leading edge)
STB[B2_LE >= T2] = 0.    # supersonic LE: no LE suction
```

For a subsonic leading edge (`B2 < tan²(ΛLE)`), `STB > 0` and LE suction exists. For a supersonic leading edge, `STB = 0` and LE suction is zero.

**Step 2:** Compute `CLE` via `compute_rotation_effects()`:
```python
# VLM.py:406-407
CLE = compute_rotation_effects(VD, settings, EW, GAMMA, len_mach, X, CHORD, XLE,
                                XBAR, rhs, COSINP, SINALF, PITCH, ROLL, YAW, STB, RNMAX)
```

`CLE` is the total induced flow at the leading edge from all horseshoe vortices, corrected for the onset flow and rotation:

```python
# VLM.py (compute_rotation_effects):
CLE = sum(EW * gamma, axis=2)   # sum of EW-weighted GAMMA across all panels
CLE = CLE - EFFINC[:, LE_ind]   # subtract effective incidence at LE
CLE = CLE / RNMAX[LE_ind] / STB # normalize
```

**Step 3:** The LE suction coefficient:
```python
# VLM.py:420-421
CLE  = CLE + 0.5 * DCP_LE * sqrt(XLE[LE_ind])
CSUC = 0.5 * pi * abs(SPC) * CLE**2 * STB
```

`CSUC` is the LE suction force per unit span (in coefficient form). It appears as a leading-edge thrust (negative drag) that partially cancels induced drag, and as a LE normal force for delta wings.

---

## 8. The `SPC` Leading-Edge Suction Multiplier

```python
# VLM.py:411-418
SPC  = K_SPC * np.ones_like(DCP_LE)   # default 1.0

VL   = np.repeat(VD.vortex_lift, n_sw)
m_b  = (mach[:,0] < 1.)               # subsonic conditions
SPC_cond      = VL * m_b.T
SPC[SPC_cond] = -1.                    # Polhamus: flip to +1 lift direction
SPC = SPC * exposed_leading_edge_flag  # zero if LE is covered by a slat
```

| SPC value | Case | Effect |
|-----------|------|--------|
| `+1` | Normal subsonic wing | LE suction acts as thrust (forward force) |
| `-1` | Delta wing + subsonic | Polhamus vortex lift: LE suction rotates to +z (added lift, not thrust) |
| `0` | Slat present | LE suction zeroed out (slat absorbs it) |

When `SPC < 0`, the force vector components are flipped:
```python
# VLM.py:434-435
TFX[SPC<0] = XSIN[SPC<0] * np.sign(DCP_LE)[SPC<0]    # x-force from LE suction
TFZ[SPC<0] = np.abs(XCOS)[SPC<0] * np.sign(DCP_LE)[SPC<0]  # z-force (lift) from LE suction
```

This is the Polhamus leading-edge suction analogy for delta wings.

---

## 9. `strip_cumsum` — The Vectorized Chordwise Cumulative Sum

```python
# VLM.py:584-595
def strip_cumsum(arr, chord_breaks, strip_lengths):
    cumsum  = np.cumsum(arr, axis=1)
    offsets = cumsum[:, chord_breaks-1]
    offsets[:, 0] = 0
    offsets = np.repeat(offsets, strip_lengths, axis=1)
    return cumsum - offsets
```

A standard `np.cumsum` accumulates across all panels. This function instead resets at each strip's LE by subtracting the cumulated value from the previous strip. This is equivalent to independent cumulative sums per strip, done in a single vectorized operation.

---

## Summary — Key Concepts

| Concept | Detail |
|---------|--------|
| AIC assembly | `A = C_mn·n̂`: dot product of induced velocity with panel normal via `δ`, `φ` |
| `np.linalg.solve(A, RHS)` | Dense LAPACK solve; O(n_cp³) per condition |
| `FACTOR = FORAXL + ONSET` | Effective dynamic pressure direction including rotation |
| `DCP = 2*GNET + DCPSID` | Pressure coefficient from GAMMA, normalized by panel chord |
| `GANT` (`strip_cumsum`) | Cumulative chordwise circulation; resets at each strip's LE |
| `STB = sqrt(tan²(ΛLE) - B2)` | Zero for supersonic LE; drives LE suction magnitude |
| `CSUC = 0.5π|SPC|·CLE²·STB` | LE suction coefficient |
| `SPC = -1` | Polhamus vortex lift for delta wings: LE suction rotated to lift direction |

---

**Next:** Module 8 — Force and Moment Integration (`VLM.py:333–494`), where strip-level pressure loadings are decomposed into body-axis forces and integrated to yield `CL`, `CDi`, `CM`, and per-wing coefficients.
