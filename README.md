# Darude-Sandbot

This Project allows an UR5 robot rake any SVG into sand by converting SVG geometry data into URScript move commands.  

https://github.com/user-attachments/assets/84fa9d08-7345-472c-8410-83129dd225ee


## Flow

CPEE is the process orchestrator. Open
<https://cpee.org/out/frames/sandbot/>, pick an SVG, and it drives the three
services in turn and sends the finished program to the robot. The model is
[Raking_final.xml](Raking_final.xml). It holds the endpoint URLs and shows
what each service is passed. The upload page it shows is
`svg-upload/upload.html`, deployed to `lehre.bpm.in.tum.de/~ge42qap/upload.html`.

`svg-to-form` flattens the SVG's curves into polylines, one form per subpath,
ordered waypoints in SVG units, plus the bounding box of the whole drawing.
`form-to-pose` scales that box to fit the sand tray with padding,
and converts every point into the robot's base frame,
with the tool pointing straight down at a fixed drawing height. `form-to-urscript`
renders each waypoint as one move line and assembles the waypoints into a UR5 readable script.


## Services

| Service | Port | Endpoint | Purpose |
|---|---|---|---|
| `svg-to-form` | 8001 | `POST /convert` | Parse an SVG into forms. The ordered waypoints of each shape. |
| `form-to-pose` | 8005 | `POST /pose` | Fit form to tray and transform to UR base frame poses. |
| `form-to-urscript` | 8006 | `POST /urscript` | translate poses into a URScript program. |


[PIPELINE.md](PIPELINE.md) walks an SVG through the whole chain function by
function: what each stage decides and why, and which knob to reach for when
the drawing comes out wrong.

## Run a service

```bash
cd <service>
pip install -r requirements.txt
uvicorn app.main:app --reload --host :: --port <port>
```
