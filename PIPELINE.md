# Pipeline

An SVG goes through three services, each passing plain JSON to the next.

```
drawing.svg
  -> svg-to-form       :8001  POST /convert    SVG markup -> polylines in SVG units
  -> form-to-pose      :8005  POST /pose       SVG units -> 6-DOF poses in the robot base frame
  -> form-to-urscript  :8006  POST /urscript   poses -> one move line each
  -> drawing.script
```

svg-to-form returns `{"bbox": [...], "forms": [{"points": [{"x","y"}, ...], ...}]}`.
form-to-pose returns `{"forms": [{"waypoints": [{"move": "draw"|"travel", "pose": [x,y,z,rx,ry,rz]}, ...]}]}`.

## svg-to-form

Parses the SVG with `reify=True` so transforms are baked in, then emits one
form per subpath. Curves are split into chords of about `SAMPLE_STEP`
(5 SVG units), lines just keep their endpoints.

Points closer than `THIN_STEP` get dropped. Icons made of lots of tiny
béziers are otherwise so dense that `movep` stops at every point. Closed
loops get restarted in the middle of an edge so no corner is at the
start/end. The bbox is taken from the emitted points, not from svgelements.

## form-to-pose

The whole drawing gets one scale and offset (`body.bbox`), and the tighter
axis wins. `DEFAULT_PADDING_MM = 40` keeps the tool centre off the walls.
y is flipped because SVG's y grows downward.

`CORNER_ORIGIN`, `CORNER_X` and `CORNER_Y` map tray coordinates into the
robot base frame. Z is fixed: `DRAW_DZ = -28.14` (taught plane 103.08 mm,
drawing height 74.94 mm) and `TRAVEL_DZ = 15.0` for the pen-up hover.

Each form becomes one stroke. Close points are collapsed first
(`dist > 1e-6`).

The rake is single-prong, so every waypoint holds the taught `TOOL_RX`.
The 3-prong path sits behind `ROTATE_BLADE` and is WIP.

## form-to-urscript

One line per waypoint. Every program starts and ends at `HOME_POSE`,
`(598.55, -129.20, 105.15)` mm at rx `(2.249, -2.194, 0)`, taught on the
pendant and clear of the rim (103.08).

`travel` waypoints become `movel`, which has a trapezoidal profile and comes
to a full stop, for lifting and lowering the rake. `draw` waypoints become
`movep`, which keeps the tool speed constant and blends through waypoints
so the groove comes out even. Draw moves use the `draw_z` variable instead
of the pose's z, at `DRAW_VEL = 0.02`.

The blend radius is at most `DRAW_BLEND = 0.015` and at most half the
distance to the next point. Anything under `MIN_BLEND = 0.001` becomes 0,
since Polyscope rejects it with "blend radius too small". The last point
before a lift also gets 0 so the rake stops on the mark.

## Tuning

If the robot stops at every waypoint or the pendant says `blend radius too
small`, the points are too dense, raise `SAMPLE_STEP`. Groove depth is
`DRAW_Z_MM` and drawing size is `DEFAULT_PADDING_MM`. `DRAW_VEL` should be changed if sand piles up at corners or the drawing is too slow.
