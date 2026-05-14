# Module 8 — Force and Moment Integration

**Primary file:** `VLM.py:333–494`
**Prerequisites:** Module 7

---

## Overview

With `DCP` (the panel pressure coefficient) and `CSUC` (the LE suction) in hand, Module 8 reduces panel-level loading to strip forces, rotates those into body axes, and then integrates over strips to produce per-wing and total vehicle aerodynamic coefficients: `CL`, `CDi`, `CM`, `CL_wing`, `CDi_wing`, `cl_y`, `cdi_y`.

The entire calculation is vectorized across all flight conditions simultaneously using `np.add.reduceat`.

---

## 1. Strip Normal Load: `SINF` and `CNC`

The first quantity is the panel's contribution to the strip normal force:

```python
# VLM.py:379
SINF = ADC * DCP   # ADC = 0.5 * PION = 0.5 * (2/RNMAX) = 1/RNMAX
```

`SINF` is the pressure loading on each panel scaled by its chordwise fraction `1/RNMAX`. `ADC` is half the non-dimensional panel chord width (since `PION = 2/RNMAX` and `ADC = 0.5*PION`).

Panels within each strip are then summed to get the strip normal force coefficient:

```python
# VLM.py:383
CNC = np.add.reduceat(SINF, chord_breaks, axis=1)
```

`np.add.reduceat(arr, indices)` sums `arr` over groups defined by consecutive `indices`. Here `chord_breaks` is the array of first-panel indices for each strip, so this is equivalent to `sum(SINF[strip_i])` for each strip — done for all conditions simultaneously.

The LE suction normal force is added in at line 440:
```python
# VLM.py:440
CNC = CNC + CSUC * sqrt(1 + T2) * TFZ
```

---

## 2. Axial Force: `CAXL`

The axial (chordwise) force on each panel comes from the slope of the loading:

```python
# VLM.py:390-391
TX   = SLOPE - ZETA                      # panel slope relative to strip chord
CAXL = -SINF * TX / (1.0 + TX**2)       # panel axial force
```

`TX` is the angle difference between the local panel inclination (`SLOPE`) and the strip chord line (`ZETA`). For a flat, uncambered wing at zero incidence `TX = 0` and `CAXL = 0`. For a cambered wing, `TX ≠ 0` and the axial force is non-zero.

Chordwise integration:
```python
# VLM.py:395-396
CAXL = np.add.reduceat(CAXL, chord_breaks, axis=1)  # sum to strips
```

The LE suction axial force is subtracted:
```python
# VLM.py:437
CAXL = CAXL - TFX * CSUC
```

---

## 3. Body-Axis Rotation: `BFX`, `BFY`, `BFZ`

Strip forces are rotated from the strip-local frame (normal and axial to the strip chord) into vehicle body axes using `ZETA` (strip chord angle) and `phi` (dihedral angle):

```python
# VLM.py:444-451
FCOS = np.cos(ZETA)   # cos of strip chord angle wrt x-axis
FSIN = np.sin(ZETA)   # sin of strip chord angle wrt x-axis

BFX = -CNC*FSIN + CAXL*FCOS    # x-body force (drag direction)
BFY = -(CNC*FCOS + CAXL*FSIN)*SID   # y-body force (side force)
BFZ =  (CNC*FCOS + CAXL*FSIN)*COD   # z-body force (lift direction)
```

Where `COD = cos(phi)` and `SID = sin(phi)` encode the dihedral angle of the strip. For a flat wing `COD = 1`, `SID = 0`, so `BFZ = CNC` and `BFY = 0`.

**Intuition:** A strip tilted at angle `ZETA` forward has its normal force pointing partly in the -x direction (producing drag) and partly in the +z direction (producing lift). The `BFX`/`BFZ` decomposition above is just this tilt decomposition.

---

## 4. Strip to Aerodynamic Force: `LIFT`, `DRAG`, `MOMENT`

Before summing, forces are multiplied by the strip planform area (`STRIP = ES * CHORD_strip`):

```python
# VLM.py:469-476
ES     = 2 * s[0, LE_ind]         # full span of strip (twice the semi-span from Biot-Savart)
STRIP  = ES * CHORD_strip          # planform area of strip

LIFT   = (BFZ*COSALF - (BFX*COPSI + BFY*SINPSI)*SINALF) * STRIP
DRAG   = CDC * ES                  # CDC = BFZ*SINALF + (BFX*COPSI+BFY*SINPSI)*COSALF * CHORD_strip
MOMENT = STRIP * (BMY*COPSI - BMX*SINPSI)
FY     = (BFY*COPSI - BFX*SINPSI) * STRIP
```

The LIFT and DRAG transforms rotate from body axes to wind axes:
- `LIFT = BFZ * cos(α) - (BFX*cos(β) + BFY*sin(β)) * sin(α)`
- `DRAG = BFZ * sin(α) + (BFX*cos(β) + BFY*sin(β)) * cos(α)`

This is the standard body-to-wind axis transformation.

---

## 5. Strip Moment: `BMX`, `BMY`, `BMZ`

Moments are computed about the moment reference point `(XBAR, ZBAR)`:

```python
# VLM.py:461-465
X = VD.XCH[LE_ind]   # load point x-coordinate (horseshoe midpoint)
Y = VD.YCH[LE_ind]
Z = VD.ZCH[LE_ind]

BMX = BFZ*Y - BFY*(Z - ZBAR) + SICPLE    # rolling moment per strip
BMY = BMLE*COD + BFX*(Z - ZBAR) - BFZ*(X - XBAR)   # pitching moment per strip
BMZ = BMLE*SID - BFX*Y + BFY*(X - XBAR)             # yawing moment per strip
```

`BMLE` is the moment contribution from the panel loading distribution about the strip LE:
```python
# VLM.py:391-396
BMLE = (XLE - XX) * SINF   # moment arm from panel center to LE
BMLE = np.add.reduceat(BMLE, chord_breaks, axis=1)
```

`SICPLE` is the rolling couple due to sideslip (product of the strip normal force and the moment arm to the trailing edge center).

---

## 6. Sectional (Strip) Coefficients

```python
# VLM.py:479-483
cl_y     = LIFT / CHORD_strip / ES   # strip CL per unit span (non-dim by strip area)
cdi_y    = DRAG / CHORD_strip / ES   # strip CDi per unit span
alpha_i  = np.hsplit(np.arctan(cdi_y / cl_y), span_breaks[1:])  # induced AoA per wing
```

`cl_y` is the spanwise lift distribution — this is the output you plot to see the elliptic distribution and compare wings. `alpha_i` is the induced angle of attack distribution, which `calculate_VLM` returns as `wing_induced_angle`.

---

## 7. Per-Wing Coefficients via `np.add.reduceat`

```python
# VLM.py:481-482
CL_wing  = np.add.reduceat(LIFT, span_breaks, axis=1) / SURF
CDi_wing = np.add.reduceat(DRAG, span_breaks, axis=1) / SURF
```

`span_breaks` is the array of strip indices at the start of each wing (from `VD.spanwise_breaks`). `np.add.reduceat` sums all strips belonging to each wing. `SURF = VD.wing_areas` is the planform area of each wing (including its mirror).

These per-wing coefficients are normalized by `SURF` (each wing's own area), not by `Sref`. The `calculate_VLM` helper in `Vortex_Lattice.py` re-dimensionalizes and re-normalizes them per wing after the solve (see Module 1, Section 6).

---

## 8. Total Vehicle Coefficients

```python
# VLM.py:486-493
CL     = np.atleast_2d(np.sum(LIFT,   axis=1) / SREF).T   # total lift coefficient
CDi    = np.atleast_2d(np.sum(DRAG,   axis=1) / SREF).T   # total induced drag
CM     = np.atleast_2d(np.sum(MOMENT, axis=1) / SREF).T / c_bar  # pitching moment
CYTOT  = np.atleast_2d(np.sum(FY,     axis=1) / SREF).T   # side force
CRTOT  = np.atleast_2d(np.sum(RM,     axis=1) / SREF).T   # rolling moment (unscaled)
CRMTOT = CRTOT / w_span * (-1)                             # rolling moment (scaled)
CNTOT  = np.atleast_2d(np.sum(YM,     axis=1) / SREF).T   # yawing moment (unscaled)
CYMTOT = CNTOT / w_span * (-1)                             # yawing moment (scaled)
```

All totals sum over all strips on all wings, normalized by vehicle reference area `Sref`. `CM` is additionally normalized by mean aerodynamic chord `c_bar`. The rolling and yawing moments are normalized by wingspan `w_span`.

The final `.T` reshapes from `[n_conditions]` back to `[n_conditions, 1]` — the standard column-vector format for `conditions` arrays (see Module 1, Section 4).

---

## 9. The Reduction Chain Summary

```
DCP[n_conditions, n_cp]
  ↓ SINF = ADC * DCP
  ↓ np.add.reduceat(SINF, chord_breaks) → CNC[n_conditions, n_strips]  (chordwise sum)
  ↓ + CSUC LE suction → modified CNC
  ↓ BFX/BFY/BFZ: rotate by ZETA, phi
  ↓ LIFT/DRAG/MOMENT/FY: multiply by strip area ES*CHORD
  ↓ cl_y/cdi_y: divide by strip area (sectional coefficients)
  ↓ np.add.reduceat(LIFT, span_breaks) / SURF → CL_wing[n_conditions, n_wings]
  ↓ sum(LIFT) / SREF → CL[n_conditions, 1]
```

---

## Summary — Key Concepts

| Concept | Detail |
|---------|--------|
| `SINF = ADC * DCP` | Panel normal load = (1/n_cw) × pressure coefficient |
| `CNC = reduceat(SINF, chord_breaks)` | Chordwise integration → strip normal force |
| `TX = SLOPE - ZETA` | Panel slope relative to strip chord → drives `CAXL` |
| `BFX/BFY/BFZ` | Body-axis force via ZETA and dihedral angle phi |
| `LIFT/DRAG = wind-axis rotation of BFX, BFZ` | Standard body → wind axis transform |
| `cl_y`, `cdi_y` | Spanwise sectional coefficients |
| `reduceat(LIFT, span_breaks)` | Per-wing lift sum |
| `sum(LIFT) / SREF` | Total vehicle CL |
| Result shape `[n_conditions, 1]` | Column vector for conditions convention |

---

**Next:** Module 9 — Compressibility, Control Surfaces, and Wake Coupling, which examines how Mach batching, control surface deflection kinematics (quaternion rotation), and rotor slipstream are integrated into the VLM for lift+cruise vehicles.
