#!/usr/bin/env python3
"""
Wind control panel: sliders for wind speed and direction, served as a web page.

Run with the simulation up (python3 tools/wind_panel.py), then open
    http://localhost:8765
in a browser on the host. The panel can also record the Gazebo window to
videos/ (Start/Stop recording). Changes are applied about once per second
(each Gazebo command-line call takes ~1 s); arrows in the 3D view show the wind.
"""
import http.server, json, os, sys, threading, time, urllib.parse
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import wind
from recorder import Recorder

rec = Recorder()

PORT = 8765
target = {"speed": 0.0, "dir": 90.0}
applied = {"speed": None, "dir": None}
lock = threading.Lock()

PAGE = """<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Wind control</title>
<style>
 :root { --bg:#f6f7f9; --fg:#1d2330; --muted:#5d6675; --card:#fff; --acc:#2f6fdf; }
 @media (prefers-color-scheme: dark) { :root { --bg:#14171c; --fg:#e8ebf0; --muted:#9aa3b2; --card:#1e232b; --acc:#6b9cff; } }
 body { font-family: system-ui, sans-serif; background:var(--bg); color:var(--fg); margin:0; padding:24px; }
 .card { max-width:460px; margin:auto; background:var(--card); border-radius:12px; padding:20px 24px;
         box-shadow:0 1px 4px rgba(0,0,0,.12); }
 h1 { font-size:20px; margin:0 0 4px; } p { color:var(--muted); margin:0 0 18px; font-size:14px; }
 label { display:flex; justify-content:space-between; font-weight:600; margin-top:16px; }
 input[type=range] { width:100%; accent-color:var(--acc); }
 .row { display:flex; gap:8px; margin-top:16px; flex-wrap:wrap; }
 button { flex:1; padding:8px; border-radius:8px; border:1px solid var(--muted); background:transparent;
          color:var(--fg); cursor:pointer; font-size:14px; }
 button.calm { border-color:#c0392b; color:#c0392b; }
 button.rec { border-color:#c0392b; color:#fff; background:#c0392b; font-weight:600; }
 button.rec.on { background:transparent; color:#c0392b; }
 #compass { display:block; margin:16px auto 0; }
 .status { margin-top:14px; font-size:13px; color:var(--muted); text-align:center; }
</style></head><body><div class="card">
<h1>Wind control</h1>
<p>Direction is where the wind blows <b>toward</b>: 0° = north (PX4 +x), 90° = east (PX4 +y).</p>
<label>Speed <span id="sv"></span></label>
<input id="speed" type="range" min="0" max="10" step="0.1" value="0">
<label>Direction <span id="dv"></span></label>
<input id="dir" type="range" min="0" max="359" step="1" value="90">
<svg id="compass" width="140" height="140" viewBox="-70 -70 140 140">
 <circle r="60" fill="none" stroke="currentColor" stroke-opacity=".3"/>
 <text y="-48" text-anchor="middle" font-size="12" fill="currentColor">N</text>
 <text x="50" y="4" text-anchor="middle" font-size="12" fill="currentColor">E</text>
 <text y="58" text-anchor="middle" font-size="12" fill="currentColor">S</text>
 <text x="-50" y="4" text-anchor="middle" font-size="12" fill="currentColor">W</text>
 <g id="arrow"><line x1="0" y1="35" x2="0" y2="-30" stroke="var(--acc)" stroke-width="5" stroke-linecap="round"/>
 <polygon points="0,-44 -10,-26 10,-26" fill="var(--acc)"/></g>
</svg>
<div class="row">
 <button onclick="setDir(0)">North</button><button onclick="setDir(90)">East</button>
 <button onclick="setDir(180)">South</button><button onclick="setDir(270)">West</button>
</div>
<div class="row"><button class="calm" onclick="calm()">Calm (0 m/s)</button></div>
<div class="row"><button id="recbtn" class="rec" onclick="toggleRec()">● Start recording</button></div>
<div class="status" id="recstatus"></div>
<div class="status" id="status">connecting…</div>
</div><script>
const sp=document.getElementById('speed'), dr=document.getElementById('dir');
function show(){ document.getElementById('sv').textContent=(+sp.value).toFixed(1)+' m/s';
  document.getElementById('dv').textContent=dr.value+'°';
  document.getElementById('arrow').setAttribute('transform','rotate('+dr.value+')'); }
let timer=null;
function send(){ show(); clearTimeout(timer); timer=setTimeout(()=>
  fetch('/set?speed='+sp.value+'&dir='+dr.value).catch(()=>{}),150); }
function setDir(d){ dr.value=d; send(); }
function calm(){ sp.value=0; send(); }
sp.oninput=send; dr.oninput=send;
async function poll(){ try{ const s=await (await fetch('/state')).json();
  document.getElementById('status').textContent = s.applied_speed===null ? 'waiting for first change' :
   'applied in Gazebo: '+s.applied_speed.toFixed(1)+' m/s toward '+Math.round(s.applied_dir)+'°'+
   (s.pending?'  (updating…)':''); }catch(e){ document.getElementById('status').textContent='panel server not reachable'; } }
async function toggleRec(){ try{ await fetch('/record'); }catch(e){} pollRec(); }
async function pollRec(){ try{ const r=await (await fetch('/recstate')).json();
  const b=document.getElementById('recbtn');
  b.textContent = r.recording ? '■ Stop recording ('+Math.floor(r.elapsed)+' s)' : '● Start recording';
  b.classList.toggle('on', r.recording);
  document.getElementById('recstatus').textContent = r.path ?
    (r.recording ? 'recording to ' : 'saved: ') + r.path.split('/').slice(-2).join('/') : ''; }catch(e){} }
setInterval(poll,500); setInterval(pollRec,500); show();
</script></body></html>"""


class Handler(http.server.BaseHTTPRequestHandler):
    def _send(self, body, ctype):
        b = body.encode()
        self.send_response(200); self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(b))); self.end_headers(); self.wfile.write(b)

    def do_GET(self):
        u = urllib.parse.urlparse(self.path)
        if u.path == "/set":
            q = urllib.parse.parse_qs(u.query)
            with lock:
                target["speed"] = max(0.0, min(float(q.get("speed", [0])[0]), 15.0))
                target["dir"] = float(q.get("dir", [90])[0]) % 360
            self._send("ok", "text/plain")
        elif u.path == "/record":
            with lock:
                if rec.recording:
                    path = rec.stop(); print(f"recording saved: {path}")
                else:
                    path = rec.start(); print(f"recording to {path}")
            self._send("ok", "text/plain")
        elif u.path == "/recstate":
            self._send(json.dumps({"recording": rec.recording, "elapsed": rec.elapsed(),
                                   "path": rec.path}), "application/json")
        elif u.path == "/state":
            with lock:
                pending = (applied["speed"], applied["dir"]) != (target["speed"], target["dir"])
                s = {"applied_speed": applied["speed"], "applied_dir": applied["dir"],
                     "pending": pending}
            self._send(json.dumps(s), "application/json")
        else:
            self._send(PAGE, "text/html; charset=utf-8")

    def log_message(self, *a):
        pass


def worker():
    last_send = 0.0
    while True:
        with lock:
            t = (target["speed"], target["dir"])
        if t != (applied["speed"], applied["dir"]):
            wind.set_wind(*t)                      # ~1 s; latest value wins
            with lock:
                applied["speed"], applied["dir"] = t
            last_send = time.time()
            print(f"wind: {t[0]:.1f} m/s toward {t[1]:.0f} deg")
        elif time.time() - last_send > 2.0:
            # A one-shot 'gz topic' publish is occasionally lost, which once left
            # 4 m/s of wind blowing while the panel showed calm; re-send the
            # current wind every 2 s so Gazebo always converges to the panel.
            wind.publish_wind(*t)
            last_send = time.time()
        time.sleep(0.1)


if __name__ == "__main__":
    wind.set_wind(0.0, 90.0)                       # start calm, arrows cleared
    applied["speed"], applied["dir"] = 0.0, 90.0
    threading.Thread(target=worker, daemon=True).start()
    srv = http.server.ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    print(f"Wind panel: open http://localhost:{PORT} in a browser on the host (Ctrl+C to stop)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if rec.recording:
            print(f"recording saved: {rec.stop()}")
        for _ in range(3):                           # make sure the calm arrives
            wind.set_wind(0.0, 90.0, draw=False)
        wind.draw_arrows(0.0, 90.0, wait=True)       # make sure the arrows are gone
        print("Wind set to calm.")
