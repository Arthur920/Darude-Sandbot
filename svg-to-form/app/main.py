from io import StringIO
from math import hypot

from fastapi import FastAPI, Form
from svgelements import SVG, Close, Line, Move, Shape

app = FastAPI(title="svg-to-form")

# Chord length (SVG units) for flattening curves. Has to be smaller than the
# smallest rounded corner or the corner gets cut off.
SAMPLE_STEP = 5.0

# Below SAMPLE_STEP on purpose, thinning at exactly SAMPLE_STEP is a float
# coin flip and ate every other point on circle.svg.
THIN_STEP = 0.5 * SAMPLE_STEP


def _sample(shape):
    """One run of points per subpath.

    Use shape.segments(), not Path(shape).
    Path(shape) yields the untransformed segments while shape.bbox()
    reports the transformed extend. Rotated circle came out at 75% size.
    .segments() applies the residual transform, so the two agree.
    """
    runs = []
    points = []
    for segment in shape.segments():
        if isinstance(segment, Move):
            if points:
                runs.append(points)
            points = []
            continue
        length = segment.length()
        if not length:
            continue
        # Lines only need their endpoints, curves get split into chords.
        if isinstance(segment, (Line, Close)):
            steps = 1
        else:
            steps = max(1, round(length / SAMPLE_STEP))
        for i in range(steps + 1):
            p = segment.point(i / steps)
            points.append({"x": float(p.x), "y": float(p.y)})
    if points:
        runs.append(points)
    return runs


def _dist(a, b):
    return hypot(a["x"] - b["x"], a["y"] - b["y"])


def _thin(points):
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


# Shortest edge (SVG units) the seam can go on, so the stop at the seam
# isn't right next to a corner. About 0.4 mm/unit on the tray.
MIN_SEAM_EDGE = 2.0
assert MIN_SEAM_EDGE < THIN_STEP


def _reseam(points):
    # Start closed strokes mid-edge so the start/end corner gets rounded too.
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
        return points  # no edge long enough to seam, draw the sharp corner instead

    mid = {"x": (p["x"] + q["x"]) / 2, "y": (p["y"] + q["y"]) / 2}
    rot = ring[i + 1 :] + ring[: i + 1]  # reorder start, so we start at seam
    return [mid] + rot + [mid]


def _forms(shape):
    # one form per subpath, the robot lifts between them
    kind = type(shape).__name__.lower()
    return [{"kind": kind, "points": _reseam(_thin(pts))} for pts in _sample(shape)]


def _union_bbox(forms):
    xs = [p["x"] for f in forms for p in f["points"]]
    ys = [p["y"] for f in forms for p in f["points"]]
    if not xs:
        return None
    return [min(xs), min(ys), max(xs), max(ys)]


@app.post("/convert")
def convert(svg: str = Form()):
    svg = SVG.parse(StringIO(svg), reify=True)
    forms = []
    for element in svg.elements():
        if isinstance(element, Shape):
            forms += _forms(element)
    return {"bbox": _union_bbox(forms), "forms": forms}
