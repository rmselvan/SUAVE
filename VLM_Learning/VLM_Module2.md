# Module 2 — Wing Geometry Preprocessing

**Primary file:** `make_VLM_wings.py`
**Prerequisites:** Module 1

---

## Overview

Before a single panel can be placed, SUAVE must convert every wing in `geometry.wings` into a uniform, VLM-ready format. `make_VLM_wings()` is that preprocessing step. It returns a flat container of "VLM wing" Data objects — one per structural wing plus one per control surface — each guaranteed to have at least two segments and all the geometric attributes the discretization loop needs.

---

## 1. Why Preprocessing Is Needed

SUAVE wing objects come in two flavors:

1. **Unsegmented** — just `wing.chords.root`, `wing.chords.tip`, `wing.sweeps.quarter_chord`, etc.
2. **Segmented** — a `wing.Segments` container with per-segment twist, chord, sweep, and dihedral.

The VLM panelization loop (`generate_wing_vortex_distribution`) only knows how to walk down a list of segments. So **all wings must be segmented first**.

`convert_to_segmented_wing()` (`make_VLM_wings.py:486`) handles case 1 — it synthesizes a root segment and tip segment from the flat wing attributes. After it runs, every wing has exactly 2 segments at minimum.

---

## 2. Sweep Conversion: Quarter-Chord → Leading Edge

The panelization loop computes panel x-positions using **leading-edge sweep**. Most wings are defined with quarter-chord sweep. The conversion is:

```python
# make_VLM_wings.py:104-107
if (i != 0) and (seg_a.sweeps.leading_edge is None):
    old_sweep = seg_a.sweeps.quarter_chord
    new_sweep = convert_sweep_segments(old_sweep, seg_a, seg_b, wing,
                                       old_ref_chord_fraction=0.25,
                                       new_ref_chord_fraction=0.0)
    seg_a.sweeps.leading_edge = new_sweep
```

This is a standard geometric conversion: the LE is forward of the quarter-chord line by `(1 - taper) * c_root / 4` projected along the span. The function is imported from `Supporting_Functions.convert_sweep_segments`.

**Why it matters:** Using the wrong sweep reference shifts every panel x-position, corrupting the AIC matrix.

---

## 3. Control Surface Injection — The `populate_control_sections` Step

If `settings.discretize_control_surfaces == True`, control surfaces are distributed from the wing level down to its segments:

```python
# make_VLM_wings.py:93
wing = populate_control_sections(wing) if discretize_cs else wing
```

After this, each wing segment carries a `.control_surfaces` container containing the portion of each control surface that overlaps that segment.

---

## 4. Control Surfaces Become Wings: `make_cs_wing_from_cs()`

The key architectural decision in SUAVE's VLM: **each control surface is modeled as a separate full wing object**, not as a modification of the parent wing's panels. This means:

- The aileron, elevator, flap, or rudder gets its own `n_sw × n_cw` panel grid
- Its panels overlay the corresponding region of the parent wing
- Its origin is computed from the parent wing's leading-edge offset at the control surface's inboard span fraction

```python
# make_VLM_wings.py:373
def make_cs_wing_from_cs(cs, seg_a, seg_b, wing, cs_ID):
    ...
    cs_wing.chords.root = wing_chord_local_at_cs_root * cs.chord_fraction
    cs_wing.chords.tip  = wing_chord_local_at_cs_tip  * cs.chord_fraction
    ...
    cs_wing.origin[0,0] += x_offset + LE_TE_cs_offset
    cs_wing.origin[0,1] += cs.span_fraction_start * wing_halfspan
```

The `LE_TE_cs_offset` shifts the origin to the hinge line: zero for slats (LE-attached), `(1 - chord_fraction) * local_chord` for flaps/ailerons/elevators (TE-attached).

---

## 5. The `span_breaks` Array — The Most Important Output

`span_breaks` is the structure that records **every spanwise discontinuity** on a wing: segment boundaries, inboard edges of control surfaces, and outboard edges of control surfaces. It drives the strip layout in `generate_wing_vortex_distribution`.

Each `span_break` Data object carries:

| Attribute | Meaning |
|-----------|---------|
| `span_fraction` | Normalized position (0 = root, 1 = tip) |
| `local_chord` | Full chord at this station (before any cuts) |
| `twist` | Local twist angle |
| `dihedral_outboard` | Dihedral angle outboard of this break |
| `sweep_outboard_LE` | LE sweep angle outboard of this break |
| `cs_IDs[2,2]` | Which control surface (if any) starts/ends here, on LE and TE |
| `cuts[2,2]` | Normalized chord position of the LE/TE cut at each side |
| `x_offset`, `dih_offset` | Cumulative x and z offsets from sweep and dihedral |

The `cuts` matrix is what enables the panelization loop to shorten the chord of strips that are adjacent to a control surface — the panels stop at the hinge line rather than the full TE.

**Building `span_breaks`:** The code does a 3-way merge sort of:
1. `seg_breaks` — one per segment boundary
2. `LE_breaks` — one per slat inboard/outboard edge
3. `TE_breaks` — one per aileron/flap/elevator/rudder inboard/outboard edge

```python
# make_VLM_wings.py:162-192
LE_breaks  = sorted(LE_breaks,  key=lambda sb: sb.span_fraction)
TE_breaks  = sorted(TE_breaks,  key=lambda sb: sb.span_fraction)
seg_breaks = sorted(seg_breaks, key=lambda sb: sb.span_fraction)
# ... merge sort ...
```

Coincident breaks (e.g., a flap that starts exactly at a segment boundary) are **superimposed** into a single `span_break` rather than duplicated.

---

## 6. The `copy_wings` Mechanism

`make_VLM_wings` does not modify `geometry.wings` in place. It creates a shallow copy via `copy_wings()` / `copy_large_container()`, extracting only the attributes needed for VLM (listed in `get_paths('wings')`). This protects the original geometry from any modifications made during panelization or control surface deflection.

```python
# make_VLM_wings.py:324-343
paths = ['tag', 'origin', 'symmetric', 'vertical', 'taper', 'dihedral',
         'thickness_to_chord', 'spans.projected', 'chords.root', 'chords.tip',
         'sweeps.quarter_chord', 'sweeps.leading_edge', 'twists.root', 'twists.tip',
         'vortex_lift', 'Airfoil', 'Segments', 'control_surfaces']
```

---

## 7. The `vortex_lift` Flag

Each wing carries a `vortex_lift` boolean (default `False`). When `True` it activates the **Polhamus leading-edge suction analogy** for delta wings. This flag is copied to `VD.vortex_lift` and later read in `VLM.py` to flip the sign of the leading-edge suction term. See Module 7 for the pressure coefficient calculation where this matters.

---

## 8. What `make_VLM_wings` Returns

A flat `Container` of Data objects with:
- All original wings (reformatted to segmented)
- All control surfaces (recast as wings with `is_a_control_surface = True`)

This container is stored as `VD.VLM_wings` and iterated twice in `generate_vortex_distribution` — first over non-control-surface wings, then over control surface wings — to guarantee correct panel ordering.

---

## Summary — Key Concepts

| Concept | Detail |
|---------|--------|
| `convert_to_segmented_wing` | Ensures all wings have ≥ 2 segments before panelization |
| LE sweep conversion | Quarter-chord → leading-edge sweep, done per-segment |
| `populate_control_sections` | Distributes wing-level control surfaces to segments |
| `make_cs_wing_from_cs` | Each control surface becomes a full wing object |
| `span_breaks` | Ordered list of spanwise discontinuities; drives strip layout |
| `cuts[2,2]` | Normalized chord cut positions for LE/TE edges of control surfaces |
| `vortex_lift` | Delta-wing Polhamus flag; propagated to VD |

---

**Next:** Module 3 — Panel Mesh Generation (`generate_vortex_distribution.py`), where `span_breaks` is consumed to place horseshoe vortices on the wing.
