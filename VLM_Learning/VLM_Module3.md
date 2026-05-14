# Module 3 — Panel Mesh Generation

**Primary file:** `generate_vortex_distribution.py`
**Prerequisites:** Module 2

---

## Overview

`generate_vortex_distribution()` is the mesh generator. It takes the VLM wings from Module 2 and produces `VD` — the Vortex Distribution Data object — a flat array of panel corner coordinates, bound vortex endpoints, and control point coordinates covering every panel on every wing and fuselage. Everything downstream (AIC, RHS, forces) reads from `VD`.

---

## 1. The 1/4–3/4 Rule

The entire panel layout follows from thin airfoil theory:

- **Bound vortex** placed at the **quarter-chord** line of each panel (`XAH`, `XBH`, `ZAH`, `ZBH`)
- **Control point** placed at the **three-quarter-chord** line of each panel (`XC`, `YC`, `ZC`)

This placement ensures the flow-tangency boundary condition at 3/4 chord is equivalent to satisfying the Kutta condition for a single horseshoe vortex — the core result of thin airfoil theory.

In the code these appear as:
```python
# generate_vortex_distribution.py (strip construction loop)
xah[start:stop] = xa1 + 0.25 * delta_x_a   # bound vortex at 1/4 chord
xc [start:stop] = xa1 + 0.75 * delta_x_a   # control point at 3/4 chord
```

The "delta_x" values account for chordwise panel spacing and any leading-/trailing-edge cuts imposed by control surfaces.

---

## 2. The Full Set of VD Coordinate Arrays

Each panel is a quadrilateral. SUAVE stores its geometry using the following naming convention:

```
XA1 _________________________ XB1    (leading edge corners)
   |                           |
   |       bound vortex        |
XAH|___________________________|XBH   (bound vortex line, at 1/4 chord)
   |           XCH             |      (midpoint of bound vortex)
   |                           |
   |          XC (control pt)  |      (3/4 chord, flow tangency applied here)
   |                           |
XA2|___________________________|XB2   (trailing edge corners)
```

All arrays are 1D with length `n_cp` (total panel count). The suffix convention:
- `A` = port/left side (positive-y side for a standard wing)
- `B` = starboard/right side
- `1` = leading edge row
- `2` = trailing edge row
- `H` = horseshoe bound vortex line (at 1/4 chord)
- `C` = control point (at 3/4 chord, or horseshoe midpoint for `XCH`/`YCH`/`ZCH`)

Additional arrays:
- `XAC`, `XBC` — midpoints of the A/B panel edges (used for computing dihedral angle `phi` in `VLM.py`)
- `XA_TE`, `XB_TE` — trailing-edge panel coordinates propagated to every panel in the strip via `np.repeat` in `postprocess_VD`

---

## 3. Cosine Spanwise Spacing

The default spacing (`settings.spanwise_cosine_spacing = True`) clusters panels toward the wing tips where the spanwise gradient of circulation is highest (elliptic distribution):

```python
# generate_vortex_distribution.py:330-333
n         = np.linspace(n_sw+1, 0, n_sw+1)
thetan    = n * (np.pi/2) / (n_sw+1)
y_coordinates = span * np.cos(thetan)
```

This is a half-cosine distribution from `span` (tip) to `0` (root). The cosine mapping ensures that equal angular increments in `θ` produce denser physical spacing at the tip — analogous to Chebyshev collocation points.

Linear spacing is available by setting `spanwise_cosine_spacing = False`, but it underestimates tip-region effects and LE suction magnitude.

---

## 4. Enforcing Span-Break Alignment

After the cosine spacing is computed, the code **snaps the nearest cosine point to each required span-break location** (segment boundaries, control surface edges):

```python
# generate_vortex_distribution.py:406-412
for y_req in y_coords_required:
    idx = (np.abs(y_coordinates - y_req) + shifted_idxs).argmin()
    shifted_idxs[idx] = np.inf   # mark as used, cannot be moved again
    y_coordinates[idx] = y_req   # snap to exact break location
```

This guarantees panel edges align exactly with segment discontinuities and control surface boundaries without altering the total panel count.

---

## 5. The Strip Construction Loop

The main panelization logic runs strip by strip (`idx_y` from `0` to `n_sw-1`). For each strip:

1. Interpolate chord and twist at the inboard (`y_a`) and outboard (`y_b`) strip edges using the surrounding `span_breaks`
2. Apply the `section_LE_cut` and `section_TE_cut` from the span_break to shorten the chord where a control surface is present
3. Place `n_cw` panels chordwise within the available chord extent
4. Compute all corner, bound vortex, and control point coordinates for all `n_cw` panels in the strip

For the x-positions, chordwise stations are computed by linear interpolation between the LE cut fraction and TE cut fraction:
```python
nondim_x_stations = np.interp(np.linspace(0.,1.,num=n_cw+1),
                               [0., 1.],
                               [section_LE_cut[i_break], section_TE_cut[i_break]])
x_stations_a = nondim_x_stations * wing_chord_section_a
```

The quarter-chord and three-quarter-chord points within each panel are then:
```python
delta_x_a = (x_stations_a[-1] - x_stations_a[0]) / n_cw
xah = xa1 + 0.25 * delta_x_a   # bound vortex
xc  = xa1 + 0.75 * delta_x_a   # control point
```

---

## 6. Symmetric Wing Mirroring

Symmetric wings loop over `sym_sign ∈ [+1, -1]`. The mirroring:
- For standard wings: `y` coordinates are negated (`y * sym_sign`)
- For vertical surfaces: `z` coordinates are negated instead

```python
# generate_vortex_distribution.py:446-448
signs         = np.array([1, -1])
symmetry_mask = [True, sym_para]
for sym_sign in signs[symmetry_mask]:
```

The positive side is appended first, then the mirrored side. This means the AIC matrix has a natural block structure: the top-left block describes self-induction on the right wing, the top-right block describes cross-induction from the left wing on the right wing, and so on. The AIC kernel handles both sides simultaneously — the mirrored side is not a special case, it just has negative y-coordinates.

---

## 7. Index Arrays: `leading_edge_indices`, `trailing_edge_indices`, `panels_per_strip`

These boolean/integer arrays are critical for all downstream calculations:

| Array | Size | Content |
|-------|------|---------|
| `VD.leading_edge_indices` | `[n_cp]` bool | `True` for the first panel in each strip (LE panel) |
| `VD.trailing_edge_indices` | `[n_cp]` bool | `True` for the last panel in each strip (TE panel) |
| `VD.panels_per_strip` (RNMAX) | `[n_cp]` int | Number of chordwise panels in the strip containing each panel |
| `VD.chordwise_breaks` | `[n_strips]` int | Global panel index of the first panel in each strip |
| `VD.spanwise_breaks` | `[n_wings]` int | Strip index of the first strip of each wing |
| `VD.chordwise_panel_number` (RK) | `[n_cp]` int | This panel's position within its strip (1-indexed) |

These are used pervasively:
- `LE_ind` selects the first panel per strip for strip-level quantities (SLOPE, chord length, span)
- `np.add.reduceat(arr, chord_breaks)` sums over chordwise panels → strip values
- `np.add.reduceat(strip_arr, span_breaks)` sums over strips → per-wing values
- `np.repeat(strip_val, RNMAX[LE_ind])` broadcasts strip values back to all panels

---

## 8. Control Surfaces Are Discretized Separately

Notice in `generate_vortex_distribution.py:219-227` that wings are processed in **two separate loops**:

```python
# First loop: structural wings
for wing in VD.VLM_wings:
    if not wing.is_a_control_surface:
        VD, wing = generate_wing_vortex_distribution(VD, wing, ...)

# Second loop: control surface wings
for wing in VD.VLM_wings:
    if wing.is_a_control_surface:
        VD, wing = generate_wing_vortex_distribution(VD, wing, ...)
```

This ordering guarantees that structural wing panels are indexed before control surface panels. The `surface_ID` array records which wing each panel belongs to — positive IDs for the original side, negative IDs for the mirrored symmetric side.

---

## 9. The `VD` Object After This Step

At the end of `generate_vortex_distribution`, `VD` contains:
- All panel coordinate arrays (`XA1`, `XB1`, `XAH`, `XBH`, `XC`, `YC`, `ZC`, etc.)
- `VD.n_cp` — total panel count
- `VD.n_sw` — array of strip counts per wing
- Index arrays for efficient strip/wing aggregation
- `VD.wing_areas` — area of each wing (and its mirror) used for CL normalization
- `VD.VLM_wings` — reference back to the wing geometry objects

Then `postprocess_VD()` is called (Module 4) to compute normals, areas, and VORLAX geometric quantities.

---

## Summary — Key Concepts

| Concept | Detail |
|---------|--------|
| 1/4–3/4 rule | Bound vortex at quarter-chord, control point at three-quarter-chord |
| `XA1/XB1`, `XAH/XBH`, `XC/YC/ZC` | Corner, bound vortex, and control point arrays |
| Cosine spanwise spacing | Panels cluster at tips; default `True` |
| Span-break snapping | Nearest cosine grid point snapped to each segment/CS boundary |
| `leading_edge_indices`, `RNMAX` | Boolean/integer arrays enabling vectorized strip/wing aggregation |
| `chordwise_breaks`, `spanwise_breaks` | Used with `np.add.reduceat` throughout force integration |
| Symmetric mirroring | `sym_sign` loop; mirrored panels appended after original side |

---

**Next:** Module 4 — Panel Postprocessing (`generate_VD_helpers.py`), which computes panel areas, unit normals, and the VORLAX geometric quantities SLOPE, ZETA, and chord length.
