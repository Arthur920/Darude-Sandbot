from io import StringIO
from math import hypot

from fastapi import FastAPI, Form
from svgelements import SVG, Close, Line, Move, Shape

app = FastAPI(title="svg-to-form")

# Target chord length, in SVG user units, for flattening curves.
#
# Tuning: the drawing gets rescaled to fit the tray, so what matters is the
# chord in MM on the sand. 
#
# The step also has to be smaller than the smallest FEATURE. 
# A rounded corner narrower than one step gets cut off
# entirely, and the polyline's corner lands off the true vertex. 
SAMPLE_STEP = 5.0

# Spacing _thin enforces. Kept below SAMPLE_STEP because flattened chords
# land on exactly SAMPLE_STEP, and thinning at that same number is a float
# coin flip that ate every other point on circle.svg, doubling the real
# spacing.
THIN_STEP = 0.5 * SAMPLE_STEP


def _sample(shape: Shape) -> list[list[dict]]:
    """Sample a shape into evenly spaced waypoints, one run per SUBPATH.

    Length comes from svgelements.

    Use shape.segments(), not Path(shape).
    Path(shape) then yields the untransformed segments while shape.bbox()
    reports the transformed extend. Rotated circle came out at 75% size.
    .segments() applies the residual transform, so the two agree.
    """
    runs: list[list[dict]] = []
    points: list[dict] = []
    for segment in shape.segments():
        if isinstance(segment, Move):
            if points:
                runs.append(points)
            points = []
            continue
        length = segment.length()
        if not length:
            continue
        # Straight segments are exact with just their endpoints; only curves
        # need subdividing into a chord approximation the robot can follow.
        if isinstance(segment, (Line, Close)):
            steps = 1
        else:
            steps = max(1, round(length / SAMPLE_STEP))
        for i in range(steps + 1):
            p = segment.point(i / steps)
            point = {"x": float(p.x), "y": float(p.y)}
            points.append(point)
    if points:
        runs.append(points)
    return runs


def _dist(a: dict, b: dict) -> float:
    return hypot(a["x"] - b["x"], a["y"] - b["y"])


def _thin(points: list[dict]) -> list[dict]:
    """Drop points closer than THIN_STEP to the last one kept.
    Needed because sampling is coarse and can leave too many points.
    Since form-to-urscript caps the blend radius at half the point spacing,
    dense points mean tiny blends, which bricks (or rather throws on) the
    cobot.
    """
    kept = points[:1]
    for p in points[1:-1]:
        if _dist(p, kept[-1]) >= THIN_STEP:
            kept.append(p)
    if len(points) > 1:
        kept.append(points[-1])
    return kept


# Shortest edge (SVG user units) allowed to host the seam. The seam is a
# dead stop, and the corner beside it blends at most
# a quarter of this edge, so a sliver would put a pivot-in-place corner
# right next to the stop. 
#
# Units, not mm: the drawing gets rescaled downstream (~0.4 mm/unit for a
# tray-filling drawing), so this is a bit under a millimetre. 
MIN_SEAM_EDGE = 2.0
assert MIN_SEAM_EDGE < THIN_STEP


def _reseam(points: list[dict]) -> list[dict]:
    """Restart a closed stroke at the midpoint of its first long-enough edge.

    We have rounded corners, so if the form is closed, the start/end corner
    should be rounded too — which means starting not at the corner but at
    the midpoint of an edge.

    Edges are taken in order and the first one clearing MIN_SEAM_EDGE is
    used.
    """
    if len(points) < 4:
        return points
    first, last = points[0], points[-1]
    if _dist(last, first) > 1e-6:
        return points  # not a closed form
    ring = points[:-1]  # drop the duplicated closing vertex
    n = len(ring)
    for i in range(n):
        p, q = ring[i], ring[(i + 1) % n]
        if _dist(q, p) >= MIN_SEAM_EDGE:
            break
    else:
        return points  # no edge long enough to seam; draw the sharp corner instead

    mid = {"x": (p["x"] + q["x"]) / 2, "y": (p["y"] + q["y"]) / 2}
    rot = ring[i + 1 :] + ring[: i + 1]  # reorder start, so we start at seam
    return [mid, *rot, mid]


def _forms(shape: Shape) -> list[dict]:
    """One form per subpath with sampled waypoints.

    Each subpath is its own stroke, so the robot lifts between a shape's
    contours.

    kind is the only metadata kept.
    """
    forms = []
    for points in _sample(shape):
        points = _reseam(_thin(points))
        forms.append(
            {
                "kind": type(shape).__name__.lower(),
                "points": points,
            }
        )
    return forms


def _union_bbox(forms: list[dict]) -> list[float] | None:
    """Overall drawing extent, used to map SVG units onto the sand area.
    """
    pts = []
    for f in forms:
        for p in f["points"]:
            pts.append(p)
    if not pts:
        return None
    xs = []
    for p in pts:
        xs.append(p["x"])
    ys = []
    for p in pts:
        ys.append(p["y"])
    return [min(xs), min(ys), max(xs), max(ys)]


@app.post("/convert")
def convert(svg: str = Form()):
    svg = SVG.parse(StringIO(svg), reify=True)
    forms = []
    for element in svg.elements():
        if isinstance(element, Shape):
            for form in _forms(element):
                forms.append(form)
    return {"bbox": _union_bbox(forms), "forms": forms}


@app.get("/health")
def health():
    return {"status": "ok"}
