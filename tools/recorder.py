"""
Screen recording of the Gazebo window with ffmpeg (x11grab).

    rec = Recorder(); rec.start(); ...; rec.stop()   -> videos/sim_YYYYmmdd_HHMMSS.mp4

Captures the Gazebo window's area of the screen in real time, so the video
shows exactly what the window shows (keep the window unobstructed while
recording). Videos go to videos/ in the repo (ignored by Git).
"""
import os, re, subprocess, time

VIDEO_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "videos")
WINDOW_TITLE = "Gazebo Sim"


def window_geometry(title=WINDOW_TITLE):
    """(x, y, w, h) of the named window on screen, or None if not found."""
    try:
        tree = subprocess.run(["xwininfo", "-root", "-tree"], capture_output=True,
                              text=True, timeout=5).stdout
    except (OSError, subprocess.TimeoutExpired):
        return None
    best = None
    for line in tree.splitlines():
        m = re.match(r'\s*(0x[0-9a-f]+) "([^"]*)".*?(\d+)x(\d+)\+-?\d+\+-?\d+\s+\+(-?\d+)\+(-?\d+)', line)
        if m and title in m.group(2):
            w, h, x, y = (int(m.group(i)) for i in (3, 4, 5, 6))
            if best is None or w * h > best[2] * best[3]:
                best = (x, y, w, h)
    return best


def screen_size():
    out = subprocess.run(["xwininfo", "-root"], capture_output=True, text=True).stdout
    w = int(re.search(r"Width:\s+(\d+)", out).group(1))
    h = int(re.search(r"Height:\s+(\d+)", out).group(1))
    return w, h


class Recorder:
    def __init__(self, fps=30):
        self.fps, self.proc, self.path, self.t0 = fps, None, None, None

    @property
    def recording(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self, name=None):
        if self.recording:
            return self.path
        os.makedirs(VIDEO_DIR, exist_ok=True)
        self.path = os.path.join(VIDEO_DIR, name or time.strftime("sim_%Y%m%d_%H%M%S.mp4"))
        geo = window_geometry()
        if geo is None:                                  # fall back to the whole screen
            sw, sh = screen_size(); geo = (0, 0, sw, sh)
            print("recorder: Gazebo window not found; recording the whole screen")
        x, y, w, h = geo
        sw, sh = screen_size()                           # clip to the screen, even sizes
        x, y = max(x, 0), max(y, 0)
        w, h = min(w, sw - x) // 2 * 2, min(h, sh - y) // 2 * 2
        display = os.environ.get("DISPLAY", ":0")
        cmd = ["ffmpeg", "-loglevel", "error", "-y", "-f", "x11grab",
               "-framerate", str(self.fps), "-video_size", f"{w}x{h}",
               "-i", f"{display}+{x},{y}", "-c:v", "libx264", "-preset", "veryfast",
               "-crf", "20", "-pix_fmt", "yuv420p", self.path]
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
        self.t0 = time.time()
        return self.path

    def stop(self):
        """Stop cleanly (ffmpeg finalizes the MP4 on 'q'); returns the file path."""
        if not self.recording:
            return None
        try:
            self.proc.communicate(b"q", timeout=10)
        except subprocess.TimeoutExpired:
            self.proc.terminate(); self.proc.wait(5)
        self.proc = None
        return self.path

    def elapsed(self):
        return time.time() - self.t0 if self.recording else 0.0
