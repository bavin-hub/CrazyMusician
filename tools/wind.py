"""
Wind control and visualization for the crazymusician Gazebo world.

set_wind(speed, direction_deg) sets Gazebo's WindEffects wind, draws wind
arrows in the 3D view (GUI markers) and records the current wind in
STATE_FILE so other scripts (e.g. the ADRC logger) can read it.

Direction convention: compass heading the wind blows TOWARD,
0 = north = PX4 NED +x, 90 = east = PX4 NED +y (= Gazebo ENU +x).
"""
import json, math, subprocess, time

WORLD = "crazymusician"
STATE_FILE = "/tmp/crazymusician_wind.json"
K_WIND = 0.3            # must match the WindEffects factor in the world SDF

# Arrow layout (Gazebo ENU frame): arrows pass beside the hover point
CENTER = (0.0, 0.0, 2.5)        # spawn point, at hover height
SIDE_OFFSETS = (-0.9, 0.9)      # m, perpendicular to the wind
HEIGHTS = (-0.5, 0.5)           # m, relative to CENTER
UPWIND_START = 1.5              # m upwind of the center where shafts begin
LEN_PER_MS = 0.25               # arrow length per m/s of wind
SHAFT_D, HEAD_R, HEAD_L = 0.03, 0.07, 0.15


_marker_proc = None      # last marker request still in flight (if any)


def _run(cmd, wait=True):
    if wait:
        return subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _run_marker(cmd, wait):
    """Marker requests must reach the GUI in order: a slow 'draw' finishing after
    a 'clear' would leave stale arrows. Wait for the previous request first."""
    global _marker_proc
    if _marker_proc is not None and _marker_proc.poll() is None:
        _marker_proc.wait()
    proc = _run(cmd, wait=False)
    if wait:
        proc.wait(); _marker_proc = None
    else:
        _marker_proc = proc
    return proc


def enu_unit(direction_deg):
    th = math.radians(direction_deg)
    return math.sin(th), math.cos(th)          # (east, north)


def ned_components(speed, direction_deg):
    th = math.radians(direction_deg)
    return speed * math.cos(th), speed * math.sin(th)   # (north, east)


def _color(speed):
    s = max(0.0, min(speed / 8.0, 1.0))        # green -> yellow -> red at 8 m/s
    r, g = (2 * s, 1.0) if s < 0.5 else (1.0, 2 * (1 - s))
    return r, g, 0.1


def _mat(r, g, b):
    c = f"r: {r:.3f} g: {g:.3f} b: {b:.3f} a: 1"
    return f"material {{ ambient {{ {c} }} diffuse {{ {c} }} emissive {{ {c} }} }}"


def _v(x, y, z):
    return f"{{ x: {x:.4f} y: {y:.4f} z: {z:.4f} }}"


def _arrow_markers(idx, base, d, length, color):
    dx, dy = d
    px, py = -dy, dx                           # horizontal perpendicular
    mat = _mat(*color)
    # shaft: cylinder along local z, rotated 90 deg about (px, py, 0) onto d
    cx, cy, cz = (base[0] + dx * length / 2, base[1] + dy * length / 2, base[2])
    s = math.sqrt(0.5)
    shaft = (f"marker {{ ns: \"wind\" id: {idx} action: ADD_MODIFY type: CYLINDER "
             f"visibility: GUI pose {{ position {_v(cx, cy, cz)} "
             f"orientation {{ x: {px * s:.4f} y: {py * s:.4f} z: 0 w: {s:.4f} }} }} "
             f"scale {_v(SHAFT_D, SHAFT_D, length)} {mat} }}")
    # head: cone as a triangle list (both windings, so it shows from any side)
    hb = (base[0] + dx * length, base[1] + dy * length, base[2])
    tip = (hb[0] + dx * HEAD_L, hb[1] + dy * HEAD_L, hb[2])
    ring = []
    for k in range(12):
        a = 2 * math.pi * k / 12
        ring.append((hb[0] + HEAD_R * (px * math.cos(a)),
                     hb[1] + HEAD_R * (py * math.cos(a)),
                     hb[2] + HEAD_R * math.sin(a)))
    pts = []
    for k in range(12):
        p, q = ring[k], ring[(k + 1) % 12]
        for tri in ((tip, p, q), (tip, q, p), (hb, q, p), (hb, p, q)):
            pts += [f"point {_v(*v)}" for v in tri]
    head = (f"marker {{ ns: \"wind\" id: {idx + 100} action: ADD_MODIFY "
            f"type: TRIANGLE_LIST visibility: GUI {mat} {' '.join(pts)} }}")
    return [shaft, head]


def draw_arrows(speed, direction_deg, wait=False):
    """Draw (or clear, for speed ~ 0) the wind arrows in the Gazebo GUI."""
    if speed < 0.05:
        req = 'ns: "wind" action: DELETE_ALL'
        return _run_marker(["gz", "service", "-s", "/marker", "--reqtype", "gz.msgs.Marker",
                            "--reptype", "gz.msgs.Boolean", "--timeout", "2000", "--req", req], wait)
    d = enu_unit(direction_deg)
    px, py = -d[1], d[0]
    length = max(0.2, LEN_PER_MS * speed)
    markers, idx = [], 1
    for so in SIDE_OFFSETS:
        for hz in HEIGHTS:
            base = (CENTER[0] + px * so - d[0] * UPWIND_START,
                    CENTER[1] + py * so - d[1] * UPWIND_START,
                    CENTER[2] + hz)
            markers += _arrow_markers(idx, base, d, length, _color(speed))
            idx += 1
    req = " ".join(markers)
    return _run_marker(["gz", "service", "-s", "/marker_array", "--reqtype", "gz.msgs.Marker_V",
                        "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req], wait)


def publish_wind(speed, direction_deg):
    """Set the WindEffects wind (Gazebo ENU components)."""
    e, n = enu_unit(direction_deg)
    _run(["gz", "topic", "-t", f"/world/{WORLD}/wind", "-m", "gz.msgs.Wind", "-p",
          f"linear_velocity: {{x: {speed * e:.3f}, y: {speed * n:.3f}}}, enable_wind: true"])


def set_wind(speed, direction_deg, draw=True):
    """Apply a wind: physics first (blocking, ~1 s), arrows in the background."""
    publish_wind(speed, direction_deg)
    with open(STATE_FILE, "w") as f:
        json.dump({"speed": speed, "direction_deg": direction_deg, "t": time.time()}, f)
    if draw:
        draw_arrows(speed, direction_deg, wait=False)


def read_state():
    """Current wind as (speed, direction_deg); (0, 0) if never set."""
    try:
        with open(STATE_FILE) as f:
            s = json.load(f)
        return float(s["speed"]), float(s["direction_deg"])
    except (OSError, ValueError, KeyError):
        return 0.0, 0.0
