# Pipeline: from an SVG to a program the robot runs

One drawing passes through three services, each handing the next a plain
JSON body.

```
drawing.svg
   │
   │  svg-to-form  (:8001, POST /convert)      SVG markup  ->  polylines in SVG units
   ▼
{"bbox": [...], "forms": [{"points": [{"x","y"}, ...], ...}]}
   │
   │  form-to-pose (:8005, POST /pose)         SVG units   ->  6-DOF poses in the robot base frame
   ▼
{"forms": [{"waypoints": [{"move": "draw"|"travel", "pose": [x,y,z,rx,ry,rz]}, ...]}]}
   │
   │  form-to-urscript (:8006, POST /urscript) poses       ->  one move line each
   ▼
drawing.script
```

---

## Stage 1: `svg-to-form`

Parses the SVG with `reify=True` so each element's transform is baked into
its geometry, then emits one form per subpath. Curves get
`round(length / SAMPLE_STEP)` chords (`SAMPLE_STEP = 5.0` SVG units), lines
and closes keep their exact endpoints, and a `Move` segment starts a new
subpath.

`_thin` enforces `THIN_STEP = 0.5 * SAMPLE_STEP` across segment boundaries.
Without it, icon art stitched from many short béziers comes out dense
enough that stage 3's blends fall under the floor and `movep` stops at
every waypoint. 

`_reseam` restarts a closed loop at the midpoint of the first edge clearing
`MIN_SEAM_EDGE`, so every real corner stays interior.

`_union_bbox` measures the points we emit, not `shape.bbox()`, so stage 2
scales by a box the drawing actually fills.

---

## Stage 2: `form-to-pose`

One scale and offset for the whole drawing (`body.bbox`), tighter axis
wins, `DEFAULT_PADDING_MM = 40` keeping the tool centre off the walls, y
flipped because SVG's y grows downward.

`CORNER_ORIGIN`, `CORNER_X` and `CORNER_Y` enable the map into the robot
base frame. Z is fixed. `DRAW_DZ = -28.14` (taught plane 103.08 mm,
drawing height 74.94 mm), `TRAVEL_DZ = 15.0` for the pen-up hover.

Each form becomes one stroke. Close points are collapsed first
(`dist > 1e-6`).

The rake is single-prong, so every waypoint holds the taught `TOOL_RX`.
The 3-prong path sits behind `ROTATE_BLADE` and is WIP.

---

## Stage 3: `form-to-urscript`

One line per waypoint. Every program starts
and ends at `HOME_POSE`, `(598.55, -129.20, 105.15)` mm at rx
`(2.249, -2.194, 0)`, taught on the pendant and clearing the rim (103.08).

| waypoint | command | why |
|---|---|---|
| `travel` | `movel` | trapezoidal profile, full stop, pen up/down |
| `draw` | `movep` | constant tool speed, blends through waypoints, so the drawing is even |

Draw moves ride the `draw_z` variable, substituted for the pose's z, at
`DRAW_VEL = 0.02`.

Blend radius: `DRAW_BLEND = 0.015` as ceiling, capped at half the distance
to the next point, and floored to 0 below `MIN_BLEND = 0.001`, which
Polyscope rejects with "blend radius too small". The last point before a
lift also gets 0 so the rake stops on the mark.

---

## Where to change what

| Symptom | Knob | 
|---|---|
| Robot stops at every waypoint | blends floored to 0 → raise `SAMPLE_STEP` | 
| `blend radius too small` on the pendant | `MIN_BLEND` is the floor; points are too dense | 
| Grooves too shallow / too deep | `DRAW_Z_MM` | 
| Drawing too small on the tray | `DEFAULT_PADDING_MM` | 
| Sand piles up at corners | `DRAW_VEL` | 
| Whole drawing is very slow | `DRAW_VEL` |
