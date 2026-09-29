import json
from pathlib import Path
from math import acos, dist, inf, pi, tan

from fastapi import FastAPI, HTTPException
from fastapi import Form as FormField
from fastapi.responses import PlainTextResponse
from pydantic import BaseModel

app = FastAPI(title="form-to-urscript")

# travel waypoints become movel at hover height, draw waypoints become
# movep at draw height for a constant tool speed.

# Motion defaults (UR units: a = m/s^2, v = m/s).
TRAVEL_ACC = 0.8
TRAVEL_VEL = 0.25
DRAW_ACC = 0.5

DRAW_VEL = 0.02
# Max blend radius (m) for draw moves, also capped at half the distance to
# the next point in _move. With the 3-prong blade the tool turns along the blend.
DRAW_BLEND = 0.015

# Polyscope rejects smaller blends ("blend radius too small").
MIN_BLEND = 0.001

# Draw height (mm), every draw move is on this z.
DRAW_Z_MM = 74.94

# Where the rake sits before the first stroke and after the last one.
HOME_POSE_MM = (598.55, -129.20, 105.15)
HOME_RX = (2.249, -2.194, 0.00)
HOME_POSE = tuple(v / 1000.0 for v in HOME_POSE_MM) + HOME_RX


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


def _pose_literal(pose, z_var=None):
    x, y, z, rx, ry, rz = pose
    z_str = z_var if z_var is not None else f"{z:.6f}"
    return f"p[{x:.6f}, {y:.6f}, {z_str}, {rx:.6f}, {ry:.6f}, {rz:.6f}]"


# Smallest blend that can hold DRAW_VEL round the corner: v^2/a * tan(delta/2).
# A full reversal can't be blended at all ("MoveP cannot maintain speed
# when reversing direction").
def _drivable_blend(prev, wp, nxt):
    na = dist(prev.pose[:3], wp.pose[:3])
    nb = dist(wp.pose[:3], nxt.pose[:3])
    if not na or not nb:
        return 0.0
    dot = sum((wp.pose[k] - prev.pose[k]) * (nxt.pose[k] - wp.pose[k]) for k in range(3))
    delta = acos(max(-1.0, min(1.0, dot / (na * nb))))
    if delta >= pi - 1e-9:
        return inf  # full reversal, the only option is to stop
    return (DRAW_VEL**2 / DRAW_ACC) * tan(delta / 2.0)


def _move(wp, nxt, prev):
    # draw moves take their z from DRAW_Z_VAR
    if wp.move == "draw":
        if nxt is not None and nxt.move == "draw":
            # A waypoint may cap its own blend below DRAW_BLEND (0 on a
            # corner anchor), but never more than half the spacing to the
            # next point.
            ceiling = DRAW_BLEND if wp.blend is None else wp.blend
            r = min(ceiling, 0.5 * dist(wp.pose[:3], nxt.pose[:3]))
            if prev is not None and r < _drivable_blend(prev, wp, nxt):
                r = 0.0  # corner too sharp to keep speed, stop and turn
            if r < MIN_BLEND:
                r = 0.0  # controller won't accept it, stop instead
        else:
            r = 0.0
        return f"  movep({_pose_literal(wp.pose, DRAW_Z_VAR)}, a={DRAW_ACC}, v={DRAW_VEL}, r={r:.6f})"
    return f"  movel({_pose_literal(wp.pose)}, a={TRAVEL_ACC}, v={TRAVEL_VEL}, r=0.0)"


def _home_move():
    return f"  movel({_pose_literal(HOME_POSE)}, a={TRAVEL_ACC}, v={TRAVEL_VEL}, r=0.0)"


def _render(body):
    if not body.forms:
        raise HTTPException(status_code=422, detail="No forms to render.")
    # One draw height for the whole program.
    lines = ["def draw_svg():", f"  {DRAW_Z_VAR} = {DRAW_Z_MM / 1000.0:.6f}"]
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
