import json
from math import atan2, dist, pi

from fastapi import FastAPI, HTTPException
from fastapi import Form as FormField
from pydantic import BaseModel
from scipy.spatial.transform import Rotation

app = FastAPI(title="form-to-pose")

# ---------------------------------------------------------------------------
# CALIBRATION. 
#
# calibration splits into two halves:
#   - GARDEN: tray size and rake padding. The drawing gets scaled uniformly
#     and centred to fit whatever area is left usable.
#   - ROBOT: where the tray actually sits in the robot base frame, found by
#     a 3-point touch-off. 
#
# ---------------------------------------------------------------------------

# --- Garden -----------------------------------------------------------------
#   width  = |C1->C0| ≈ 384.3 and |C3->C2| ≈ 382.6  -> 383.5
#   height = |C1->C3| ≈ 388.4 and |C0->C2| ≈ 384.9  -> 386.5
TRAY_WIDTH_MM = 383.5
TRAY_HEIGHT_MM = 386.5
# Padding keeps the TOOL CENTRE this far from the walls.
DEFAULT_PADDING_MM = 40.0

# --- Robot ------------------------------------------------------------------
CORNER_ORIGIN = (796.03, 77.60, 103.08)  # tray (0, 0) = C1
CORNER_X = (411.80, 71.35, 103.08)  # tray (width, 0) = C0
CORNER_Y = (796.92, -310.75, 103.08)  # tray (0, height) = C3

# Base displacement (mm) per 1 mm moved along each tray axis. The tray sits
# level, so z is fixed at CORNER_ORIGIN's and only x,y need interpolating.
_AX = (
    (CORNER_X[0] - CORNER_ORIGIN[0]) / TRAY_WIDTH_MM,
    (CORNER_X[1] - CORNER_ORIGIN[1]) / TRAY_WIDTH_MM,
)
_AY = (
    (CORNER_Y[0] - CORNER_ORIGIN[0]) / TRAY_HEIGHT_MM,
    (CORNER_Y[1] - CORNER_ORIGIN[1]) / TRAY_HEIGHT_MM,
)

# Z offsets (mm) relative to the tray.
# form-to-urscript overrides every draw move's z with its own measured DRAW_Z_MM.
DRAW_DZ = -28.14  # taught plane 103.08 mm (tray rim); drawing height 74.94 mm
TRAVEL_DZ = 15.0  # taught plane 103.08 mm + 15 -> ~118 mm hover

# Tool orientation (axis-angle rad).
TOOL_RX = (2.230, -2.212, 0.0)

# Blade mounting calibration (rad). How the rake blade is oriented in the
# gripper.
TOOL_YAW_OFFSET = 0.0

# Single prong or three? A single prong is rotationally symmetric, so the
# blade heading has no purpose. This is not yet fully working, so dont set it to True yet.
ROTATE_BLADE = False

# Rake width (mm).
RAKE_WIDTH_MM = 30.0
# How far before/after a corner the bracket points sit (mm).
CORNER_BRACKET_MM = RAKE_WIDTH_MM
# Vertices that bend less than this aren't corners.
CORNER_MIN_TURN = 0.09  # rad, ~5 deg


def _wrap(a: float) -> float:
    """Angle to (-pi, pi]."""
    return -((pi - a) % (2 * pi) - pi)


def _fold(turn: float) -> float:
    """Shortest equivalent blade turn, in (-pi/2, pi/2].

    The rake's prongs sit in a row, so the blade is symmetric under a 180 deg
    flip. Turning by t and by t - pi look the same.
    """
    turn = _wrap(turn)
    if turn > pi / 2:
        return turn - pi
    if turn <= -pi / 2:
        return turn + pi
    return turn


def _toward(
    a: tuple[float, float], b: tuple[float, float], d: float
) -> tuple[float, float]:
    """The point d mm from a, in the direction of b."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    n = dist(a, b)
    return (a[0] + dx / n * d, a[1] + dy / n * d)


def _bracket(tray_pts: list[tuple[float, float]], seg_h: list[float]):
    """Walk the polyline, emitting (point, blade heading, blend) triples.

    At a corner we have three points. A point d back along the incoming 
    leg the vertex itself, and a point d along the
    outgoing leg.

    The two bracket points are CRISP (blend 0): they anchor the blade to the
    incoming, then the outgoing, heading so the turn can't bleed past them
    onto the legs. 
    """
    if not seg_h:  # single point: nothing to align to
        return [(tray_pts[0], 0.0, None)]

    h = seg_h[0]  # current blade heading, carried forward across folds
    out = [(tray_pts[0], h, None)]
    for i in range(1, len(tray_pts) - 1):
        turn = _fold(seg_h[i] - h)
        if abs(turn) < CORNER_MIN_TURN:
            h += turn
            out.append((tray_pts[i], h, None))
            continue
        # Clamp so the brackets stay inside their legs even on short segments.
        d = min(
            CORNER_BRACKET_MM,
            0.4 * dist(tray_pts[i - 1], tray_pts[i]),
            0.4 * dist(tray_pts[i], tray_pts[i + 1]),
        )
        out.append((_toward(tray_pts[i], tray_pts[i - 1], d), h, 0.0))
        out.append((tray_pts[i], h + turn / 2, None))
        h += turn
        out.append((_toward(tray_pts[i], tray_pts[i + 1], d), h, 0.0))
    out.append((tray_pts[-1], h, None))  # h is already the last segment's heading
    return out


class Point(BaseModel):
    x: float
    y: float


class Form(BaseModel):
    kind: str | None = None
    points: list[Point]


class PoseBody(BaseModel):
    # Mirrors svg-to-form's /convert response.
    bbox: list[float] | None
    forms: list[Form]


def _spin(heading: float) -> tuple[float, float, float]:
    """Spin the taught tool orientation about its own down-axis to face a heading.
     --- again wip, no ROTATE_BLADE True advised
    """
    r = Rotation.from_rotvec(TOOL_RX) * Rotation.from_euler(
        "z", heading + TOOL_YAW_OFFSET
    )
    return tuple(r.as_rotvec())


def orient(heading: float) -> tuple[float, float, float]:
    """Tool orientation for a waypoint: spun to the heading if the blade
    rotates, otherwise always the taught constant."""
    return _spin(heading) if ROTATE_BLADE else TOOL_RX


def _base_mm(x_mm: float, y_mm: float) -> tuple[float, float, float]:
    """Base-frame x,y,z (mm) of a tray-mm point. Tray is level, so
    only x,y are interpolated"""
    bx = CORNER_ORIGIN[0] + x_mm * _AX[0] + y_mm * _AY[0]
    by = CORNER_ORIGIN[1] + x_mm * _AX[1] + y_mm * _AY[1]
    return bx, by, CORNER_ORIGIN[2]


def _pose(
    x_mm: float, y_mm: float, dz_mm: float, tool_rx: tuple[float, float, float]
) -> list[float]:
    """Lift a tray-mm point to a 6-DOF base-frame pose [x, y, z, rx, ry, rz] (m, rad).
    """
    bx, by, bz = _base_mm(x_mm, y_mm)
    return [bx / 1000.0, by / 1000.0, (bz + dz_mm) / 1000.0, *tool_rx]


@app.post("/pose")
def to_pose(bbox: str = FormField(), forms: str = FormField()):
    # CPEE posts call arguments as form fields, each holding JSON.
    body = PoseBody(bbox=json.loads(bbox), forms=json.loads(forms))
    if body.bbox is None or not body.forms:
        raise HTTPException(status_code=422, detail="No forms to place on the tray.")

    # --- Garden: fit the whole drawing into the tray minus the padding ---
    usable_w = TRAY_WIDTH_MM - 2 * DEFAULT_PADDING_MM
    usable_h = TRAY_HEIGHT_MM - 2 * DEFAULT_PADDING_MM

    # One scale/offset for the whole image, so nothing gets distorted
    min_x, min_y, max_x, max_y = body.bbox
    draw_w = max_x - min_x
    draw_h = max_y - min_y

    # Uniform scale preserves aspect ratio
    ratios = []
    if draw_w > 0:
        ratios.append(usable_w / draw_w)
    if draw_h > 0:
        ratios.append(usable_h / draw_h)
    scale = min(ratios) if ratios else 1.0

    off_x = DEFAULT_PADDING_MM + (usable_w - draw_w * scale) / 2
    off_y = DEFAULT_PADDING_MM + (usable_h - draw_h * scale) / 2

    # --- Robot: lift each form's tray-mm points into base-frame stroke poses ---
    forms: list[dict] = []
    for i, form in enumerate(body.forms):
        if not form.points:
            continue
        tray_pts = []
        for p in form.points:
            # SVG y grows downward, so measure from max_y instead of min_y —
            # that flip is what keeps the drawing from coming out mirrored.
            x = off_x + (p.x - min_x) * scale
            y = off_y + (max_y - p.y) * scale
            tray_pts.append((x, y))
        # Collapse consecutive coincident points.
        deduped = [tray_pts[0]]
        for prev, p in zip(tray_pts, tray_pts[1:]):
            if dist(prev, p) > 1e-6:
                deduped.append(p)
        tray_pts = deduped
        if ROTATE_BLADE:
            # Blade heading per segment, in the BASE frame — the blade is
            # spun about its down-axis to this so it stays orthogonal to
            # travel. The dedupe above guarantees consecutive points differ,
            # so no segment is degenerate.
            base = []
            for x, y in tray_pts:
                base.append(_base_mm(x, y)[:2])
            seg_h = []
            for a, b in zip(base, base[1:]):
                seg_h.append(atan2(b[1] - a[1], b[0] - a[0]))
            placed = _bracket(tray_pts, seg_h)
        else:
            placed = []
            for p in tray_pts:
                placed.append((p, 0.0, None))

        # Each stroke: pen-up above the first point, drop to draw height,
        # draw through every point, then lift again. Heading carries through.
        (fx, fy), fh, _ = placed[0]
        (lx, ly), lh, _ = placed[-1]
        waypoints: list[dict] = [
            {"move": "travel", "pose": _pose(fx, fy, TRAVEL_DZ, orient(fh))},
        ]
        for (x, y), h, b in placed:
            # blend only when set (0 anchors a bracket point); None lets
            # form-to-urscript apply its default radius.
            wp = {"move": "draw", "pose": _pose(x, y, DRAW_DZ, orient(h))}
            if b is not None:
                wp["blend"] = b
            waypoints.append(wp)
        waypoints.append(
            {"move": "travel", "pose": _pose(lx, ly, TRAVEL_DZ, orient(lh))}
        )
        forms.append({"id": i, "kind": form.kind, "waypoints": waypoints})

    # tray/scale/calibration aren't read downstream — they're what you check
    # when a drawing lands in the wrong place.
    return {
        "tray": {
            "width": TRAY_WIDTH_MM,
            "height": TRAY_HEIGHT_MM,
            "padding": DEFAULT_PADDING_MM,
        },
        "scale": scale,
        "calibration": {
            "corner_origin": list(CORNER_ORIGIN),
            "corner_x": list(CORNER_X),
            "corner_y": list(CORNER_Y),
            "draw_dz": DRAW_DZ,
            "travel_dz": TRAVEL_DZ,
            "tool_rx": list(TOOL_RX),
            "rotate_blade": ROTATE_BLADE,
        },
        "forms": forms,
    }


@app.get("/health")
def health():
    return {"status": "ok"}
