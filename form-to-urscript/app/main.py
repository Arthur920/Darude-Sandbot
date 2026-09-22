import json
from pathlib import Path
from math import acos, dist, inf, pi, tan

from fastapi import FastAPI, HTTPException
from fastapi import Form as FormField
from fastapi.responses import PlainTextResponse
from pydantic import BaseModel

app = FastAPI(title="form-to-urscript")

# ---------------------------------------------------------------------------
# Renders form-to-pose output into a single flat urscript.
#
#   travel waypoint -> movel at pen-up height is faster
#   draw   waypoint -> movep at draw height gives constant tool speed
# ---------------------------------------------------------------------------

# Motion defaults (UR units: a = m/s^2, v = m/s).
TRAVEL_ACC = 0.8
TRAVEL_VEL = 0.25
DRAW_ACC = 0.5

DRAW_VEL = 0.02
# Blend radius (m) for draw moves so movep flows through waypoints instead
# of stopping at each. This is the upper bound: each draw move caps its own
# blend at half the distance to the next point (see _move).
#
# The floor matters for the orthogonal-blade rotation: movep rotates the
# tool across the blend arc, so a corner is turned while the rake rolls
# forward.
DRAW_BLEND = 0.015

# Below this the controller refuses the move ("blend radius too small").
#
# A blend that small does nothing anyway.
MIN_BLEND = 0.001

# Draw height (mm), every draw move is on this z.
DRAW_Z_MM = 74.94 

# Where the rake sits before the first stroke and after the last one.
HOME_POSE_MM = (598.55, -129.20, 105.15)
HOME_RX = (2.249, -2.194, 0.00)
_home_pose_m = []
for v in HOME_POSE_MM:
    _home_pose_m.append(v / 1000.0)
HOME_POSE = tuple(_home_pose_m) + HOME_RX


class Waypoint(BaseModel):
    move: str  # "travel" or "draw"
    # [x, y, z, rx, ry, rz] base frame, m / rad. 
    pose: tuple[float, float, float, float, float, float]
    # Per-waypoint blend cap (m). None -> use draw_blend. 
    blend: float | None = None


class Form(BaseModel):
    kind: str | None = None
    waypoints: list[Waypoint]


class ScriptBody(BaseModel):
    forms: list[Form]

DRAW_Z_VAR = "draw_z"


def _pose_literal(pose: tuple[float, ...], z_var: str | None = None) -> str:
    """Format a 6-DOF pose as a URScript p[...] literal.
    """
    x, y, z, rx, ry, rz = pose
    z_str = z_var if z_var is not None else f"{z:.6f}"
    return f"p[{x:.6f}, {y:.6f}, {z_str}, {rx:.6f}, {ry:.6f}, {rz:.6f}]"


def _drivable_blend(prev: Waypoint, wp: Waypoint, nxt: Waypoint) -> float:
    """Smallest blend radius the rake can actually be driven around this corner.

    For a corner where the direction changes by delta, a blend radius r
    rides an arc of radius r / tan(delta/2), and holding DRAW_VEL round
    that arc costs v^2 / R of sideways acceleration.
    Smallest usable blend:

        r >= (DRAW_VEL^2 / DRAW_ACC) * tan(delta / 2)

    A full 180 deg reversal creates an arc of radius zero, 
    which is the controller's "MoveP cannot maintain
    speed when reversing direction".
    """
    a = [w - q for w, q in zip(wp.pose[:3], prev.pose[:3])]
    b = [n - w for n, w in zip(nxt.pose[:3], wp.pose[:3])]
    na, nb = dist(prev.pose[:3], wp.pose[:3]), dist(wp.pose[:3], nxt.pose[:3])
    if not na or not nb:
        return 0.0
    dot = sum(i * j for i, j in zip(a, b))
    cos = dot / (na * nb)
    delta = acos(max(-1.0, min(1.0, cos)))
    if delta >= pi - 1e-9:
        return inf  # dead reversal: no arc exists, stop is the only option
    return (DRAW_VEL**2 / DRAW_ACC) * tan(delta / 2.0)


def _move(wp: Waypoint, nxt: Waypoint | None, prev: Waypoint | None) -> str:
    """One move line. Draw -> movep (constant speed, blended); travel -> movel (crisp stop).

    Draw moves keep their planned x/y but take z from DRAW_Z_VAR. 
    """
    if wp.move == "draw":
        if nxt is not None and nxt.move == "draw":
            # A waypoint may cap its own blend below DRAW_BLEND (0 on a
            # corner anchor); still never exceed half the spacing to the
            # next point.
            ceiling = DRAW_BLEND if wp.blend is None else wp.blend
            r = min(ceiling, 0.5 * dist(wp.pose[:3], nxt.pose[:3]))
            if prev is not None and r < _drivable_blend(prev, wp, nxt):
                r = 0.0  # corner too sharp to hold speed through — stop and turn
            if r < MIN_BLEND:
                r = 0.0  # too small for the controller to accept — stop instead
        else:
            r = 0.0
        return f"  movep({_pose_literal(wp.pose, DRAW_Z_VAR)}, a={DRAW_ACC}, v={DRAW_VEL}, r={r:.6f})"
    return f"  movel({_pose_literal(wp.pose)}, a={TRAVEL_ACC}, v={TRAVEL_VEL}, r=0.0)"


def _home_move() -> str:
    """The move to the park pose, same line at the top and the bottom."""
    return f"  movel({_pose_literal(HOME_POSE)}, a={TRAVEL_ACC}, v={TRAVEL_VEL}, r=0.0)"


def _render(body: ScriptBody) -> str:
    if not body.forms:
        raise HTTPException(status_code=422, detail="No forms to render.")
    # One draw height for the whole program.
    lines.append("  # home")
    lines.append(_home_move())
    for i, form in enumerate(body.forms):
        if not form.waypoints:
            continue
        label = form.kind or "form"
        lines.append(f"  # stroke {i}: {label}")
        for j, wp in enumerate(form.waypoints):
            nxt = form.waypoints[j + 1] if j + 1 < len(form.waypoints) else None
            prev = form.waypoints[j - 1] if j else None
            lines.append(_move(wp, nxt, prev))
    lines.append("  # park")
    lines.append(_home_move())

    lines.append("end \n")
    lines.append("draw_svg()")
    return "\n".join(lines) + "\n"


@app.post("/urscript", response_class=PlainTextResponse)
def to_urscript(forms: str = FormField()):
    # CPEE posts call arguments as form fields, each holding JSON.
    script = _render(ScriptBody(forms=json.loads(forms)))
    (Path.home() / "public_html" / "script.txt").write_text(script)
    return script


@app.get("/health")
def health():
    return {"status": "ok"}
