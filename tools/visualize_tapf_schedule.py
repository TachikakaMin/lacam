#!/usr/bin/env python3
import argparse
import html
import json
import re
from pathlib import Path

from tapf_schedule_io import load_schedule


def agent_index(name):
    match = re.search(r"\d+", name)
    return int(match.group()) if match else 0


def read_movingai_map(path):
    lines = Path(path).read_text().splitlines()
    grid = []
    in_map = False
    for line in lines:
        if line.strip() == "map":
            in_map = True
            continue
        if in_map:
            grid.append(line.rstrip("\n"))
    if not grid:
        raise ValueError(f"no map section in {path}")
    return grid


def expand_path(path, makespan):
    entries = [(step["t"], [step["x"], step["y"]]) for step in path]
    dense = [None] * (makespan + 1)
    for idx, (t, pos) in enumerate(entries):
        next_t = entries[idx + 1][0] if idx + 1 < len(entries) else makespan + 1
        for tt in range(t, min(next_t, makespan + 1)):
            dense[tt] = pos
    if any(pos is None for pos in dense):
        raise ValueError("schedule does not cover full time range")
    return dense


def expand_task_timeline(entries, makespan):
    if not entries:
        return [{"task": -1, "phase": "idle"} for _ in range(makespan + 1)]
    entries = sorted(entries, key=lambda item: int(item.get("t", 0)))
    dense = [{"task": -1, "phase": "idle"} for _ in range(makespan + 1)]
    for idx, entry in enumerate(entries):
        start = max(0, int(entry.get("t", 0)))
        end = (
            int(entries[idx + 1].get("t", makespan + 1))
            if idx + 1 < len(entries)
            else makespan + 1
        )
        value = {
            "task": int(entry.get("task", -1)),
            "phase": str(entry.get("phase", "idle")),
        }
        for tt in range(start, min(end, makespan + 1)):
            dense[tt] = value
    return dense


def compute_stats(name, path, goal):
    makespan = len(path) - 1
    goal_t = list(goal)
    first = next((t for t, pos in enumerate(path) if pos == goal_t), None)

    c = makespan + 1
    while c > 0 and path[c - 1] == goal_t:
        c -= 1
    soc = c

    sol = sum(
        1
        for t in range(1, makespan + 1)
        if path[t - 1] != goal_t or path[t] != goal_t
    )
    moves = sum(1 for t in range(1, makespan + 1) if path[t] != path[t - 1])

    compressed = []
    for pos in path:
        if not compressed or compressed[-1] != pos:
            compressed.append(pos)
    aba = sum(
        1
        for i in range(2, len(compressed))
        if compressed[i] == compressed[i - 2] and compressed[i - 1] != compressed[i]
    )

    departures = []
    if first is not None:
        t = first + 1
        while t <= makespan:
            if path[t] == goal_t:
                t += 1
                continue
            start = t
            while t <= makespan and path[t] != goal_t:
                t += 1
            departures.append([start, t - 1, t - start])

    off_after = sum(item[2] for item in departures)
    hidden_wait = (
        sum(1 for t in range(first, soc) if path[t] == goal_t)
        if first is not None
        else 0
    )
    normal = first if first is not None else soc

    return {
        "name": name,
        "id": agent_index(name),
        "goal": goal_t,
        "first": first,
        "soc": soc,
        "sol": sol,
        "moves": moves,
        "aba": aba,
        "departures": departures,
        "offAfter": off_after,
        "hiddenWait": hidden_wait,
        "normal": normal,
        "isZigzag": aba >= 10 or moves >= 100,
        "isLate": hidden_wait >= 100 and off_after <= 120,
    }


HTML_TEMPLATE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{title}</title>
<style>
:root {{
  color-scheme: light;
  --bg: #f4f6f8;
  --panel: #ffffff;
  --ink: #17202a;
  --muted: #64727f;
  --line: #d6dde5;
  --wall: #20252b;
  --free: #f9fafb;
  --late: #d9472b;
  --zig: #1d7f8c;
  --both: #8a4bb8;
  --other: #5867d8;
}}
* {{ box-sizing: border-box; }}
body {{
  margin: 0;
  font-family: ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
  color: var(--ink);
  background: var(--bg);
}}
.app {{
  display: grid;
  grid-template-columns: minmax(520px, 1fr) 420px;
  gap: 16px;
  padding: 16px;
  min-height: 100vh;
}}
.stage {{
  min-width: 0;
  display: flex;
  flex-direction: column;
  gap: 12px;
}}
.toolbar, .panel {{
  background: var(--panel);
  border: 1px solid var(--line);
  border-radius: 8px;
}}
.toolbar {{
  padding: 12px;
  display: grid;
  grid-template-columns: auto 1fr auto auto auto auto;
  gap: 10px;
  align-items: center;
}}
button, select, input {{
  font: inherit;
}}
button, select {{
  height: 34px;
  border: 1px solid var(--line);
  background: #fff;
  color: var(--ink);
  border-radius: 6px;
  padding: 0 10px;
}}
button.active {{
  border-color: #222;
  box-shadow: inset 0 0 0 1px #222;
}}
input[type=range] {{
  width: 100%;
}}
.time {{
  min-width: 86px;
  text-align: right;
  color: var(--muted);
  font-variant-numeric: tabular-nums;
}}
.canvas-wrap {{
  background: var(--panel);
  border: 1px solid var(--line);
  border-radius: 8px;
  padding: 12px;
  flex: 1;
  display: grid;
  place-items: center;
  min-height: 580px;
}}
canvas {{
  width: min(100%, 760px);
  aspect-ratio: 1 / 1;
  image-rendering: crisp-edges;
}}
.side {{
  min-width: 0;
  display: flex;
  flex-direction: column;
  gap: 12px;
}}
.panel {{
  padding: 12px;
}}
.title {{
  font-size: 16px;
  font-weight: 650;
  margin-bottom: 8px;
}}
.metrics {{
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 8px;
}}
.metric {{
  border: 1px solid var(--line);
  border-radius: 6px;
  padding: 8px;
}}
.metric .label {{
  color: var(--muted);
  font-size: 12px;
}}
.metric .value {{
  margin-top: 2px;
  font-size: 20px;
  font-variant-numeric: tabular-nums;
}}
.legend {{
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 6px 10px;
  font-size: 13px;
  color: var(--muted);
}}
.swatch {{
  display: inline-block;
  width: 11px;
  height: 11px;
  border-radius: 50%;
  margin-right: 6px;
  vertical-align: -1px;
}}
.cargo-swatch {{
  display: inline-block;
  width: 12px;
  height: 10px;
  margin-right: 6px;
  vertical-align: -1px;
  border: 1px solid;
}}
.cargo-swatch.inbound {{
  background: #f2b84b;
  border-color: #6b410f;
  box-shadow: inset 0 3px 0 #ffd980;
}}
.cargo-swatch.outbound {{
  background: #43a047;
  border-color: #14532d;
  box-shadow: inset 0 3px 0 #9be7a0;
}}
.rows {{
  max-height: 44vh;
  overflow: auto;
  border: 1px solid var(--line);
  border-radius: 6px;
}}
table {{
  width: 100%;
  border-collapse: collapse;
  font-size: 12px;
  font-variant-numeric: tabular-nums;
}}
th, td {{
  padding: 6px 7px;
  border-bottom: 1px solid var(--line);
  text-align: right;
  white-space: nowrap;
}}
th {{
  position: sticky;
  top: 0;
  background: #eef2f6;
  color: #3c4955;
  z-index: 1;
}}
td:first-child, th:first-child {{
  text-align: left;
}}
tr.selected {{
  background: #fff3cf;
}}
.note {{
  color: var(--muted);
  font-size: 12px;
  line-height: 1.45;
}}
.task-info {{
  color: var(--muted);
  font-size: 13px;
  line-height: 1.5;
}}
.task-info strong {{
  color: var(--ink);
}}
@media (max-width: 980px) {{
  .app {{
    grid-template-columns: 1fr;
  }}
  .toolbar {{
    grid-template-columns: auto 1fr auto;
  }}
}}
</style>
</head>
<body>
<div class="app">
  <main class="stage">
    <div class="toolbar">
      <button id="play">Play</button>
      <input id="slider" type="range" min="0" max="{makespan}" value="0">
      <div class="time">t=<span id="time">0</span>/{makespan}</div>
      <select id="speed" aria-label="Playback speed" title="Playback speed">
        <option value="0.25">0.25x</option>
        <option value="0.5">0.5x</option>
        <option value="1" selected>1x</option>
        <option value="2">2x</option>
        <option value="4">4x</option>
      </select>
      <select id="mode">
        <option value="all">All agents</option>
        <option value="problem">Zigzag + late</option>
        <option value="zigzag">Zigzag</option>
        <option value="late">Late departure</option>
      </select>
      <button id="focus">Focus cluster</button>
    </div>
    <div class="canvas-wrap">
      <canvas id="canvas" width="1024" height="1024"></canvas>
    </div>
  </main>
  <aside class="side">
    <section class="panel">
      <div class="title">{title}</div>
      <div class="metrics">
        <div class="metric"><div class="label">SOC</div><div class="value">{soc}</div></div>
        <div class="metric"><div class="label">Sum-of-loss</div><div class="value">{sol}</div></div>
        <div class="metric"><div class="label">ABA zigzags</div><div class="value">{aba}</div></div>
        <div class="metric"><div class="label">Hidden wait</div><div class="value">{hidden}</div></div>
      </div>
    </section>
    <section class="panel">
      <div class="title">Legend</div>
      <div class="legend">
        <div><span class="swatch" style="background:var(--zig)"></span>Zigzag</div>
        <div><span class="swatch" style="background:var(--late)"></span>Late departure</div>
        <div><span class="swatch" style="background:var(--both)"></span>Both</div>
        <div><span class="swatch" style="background:var(--other)"></span>Other</div>
        <div><span class="swatch" style="background:#111;border-radius:2px"></span>Task start</div>
        <div><span class="swatch" style="background:#fff;border:2px solid #111"></span>Task goals</div>
        <div><span class="cargo-swatch inbound"></span>Inbound cargo</div>
        <div><span class="cargo-swatch outbound"></span>Outbound cargo</div>
      </div>
    </section>
    <section class="panel">
      <div class="title">Current Task</div>
      <div id="taskInfo" class="task-info"></div>
    </section>
    <section class="panel">
      <div class="title">Agent Breakdown</div>
      <div class="rows">
        <table id="table">
          <thead>
            <tr>
              <th>agent</th><th>task</th><th>phase</th><th>SOC</th><th>SOL</th><th>ABA</th>
            </tr>
          </thead>
          <tbody></tbody>
        </table>
      </div>
    </section>
    <section class="panel note">
      Hidden wait is time spent at goal before the final stable suffix; SOC counts it if the agent later leaves goal.
      Click a row to isolate an agent. Use Focus cluster for the congested (10,10)-(10,13) area.
    </section>
  </aside>
</div>
<script>
const DATA = {data_json};
const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');
const slider = document.getElementById('slider');
const timeEl = document.getElementById('time');
const playBtn = document.getElementById('play');
const speedEl = document.getElementById('speed');
const modeEl = document.getElementById('mode');
const focusBtn = document.getElementById('focus');
const tbody = document.querySelector('#table tbody');
const taskInfo = document.getElementById('taskInfo');
const taskById = new Map((DATA.tasks || []).map(task => [Number(task.id), task]));
let t = 0;
let playhead = 0;
let animationFrameId = null;
let lastFrameTime = null;
let renderedUiStep = -1;
let selected = null;
let focusCluster = false;
const millisecondsPerStep = 70;

function colorFor(agent) {{
  if (agent.isZigzag && agent.isLate) return '#8a4bb8';
  if (agent.isZigzag) return '#1d7f8c';
  if (agent.isLate) return '#d9472b';
  return '#5867d8';
}}

function visible(agent) {{
  if (selected !== null) return agent.id === selected;
  const mode = modeEl.value;
  if (mode === 'zigzag') return agent.isZigzag;
  if (mode === 'late') return agent.isLate;
  if (mode === 'problem') return agent.isZigzag || agent.isLate;
  return true;
}}

function taskState(agent) {{
  const idx = Math.min(t, agent.timeline.length - 1);
  return agent.timeline[idx] || {{task: -1, phase: 'idle'}};
}}

function interpolatedPosition(agent) {{
  const fromStep = Math.min(Math.floor(playhead), agent.path.length - 1);
  const toStep = Math.min(fromStep + 1, agent.path.length - 1);
  const progress = playhead - Math.floor(playhead);
  const [fromRow, fromCol] = agent.path[fromStep];
  const [toRow, toCol] = agent.path[toStep];
  return [
    fromRow + (toRow - fromRow) * progress,
    fromCol + (toCol - fromCol) * progress
  ];
}}

function phaseLabel(phase) {{
  if (phase === 'assigned') return 'to-start';
  if (phase === 'loaded') return 'to-goal';
  return 'idle';
}}

function taskLabel(state) {{
  return state.task >= 0 ? `T${{state.task}}` : '-';
}}

function bounds() {{
  if (!focusCluster) return {{x0: 0, y0: 0, x1: DATA.height, y1: DATA.width}};
  return {{x0: 7, y0: 7, x1: 13, y1: 16}};
}}

function inBounds(point, b) {{
  return point && point.x >= b.x0 && point.x < b.x1 && point.y >= b.y0 && point.y < b.y1;
}}

function cellCenter(point, b, cell, offX, offY) {{
  return {{
    x: offX + (point.y - b.y0 + 0.5) * cell,
    y: offY + (point.x - b.x0 + 0.5) * cell
  }};
}}

function drawTaskMarker(point, b, cell, offX, offY, color, kind, strong, label) {{
  if (!inBounds(point, b)) return;
  const p = cellCenter(point, b, cell, offX, offY);
  ctx.save();
  ctx.globalAlpha = strong ? 1 : 0.42;
  ctx.strokeStyle = color;
  ctx.fillStyle = kind === 'start' ? color : '#ffffff';
  ctx.lineWidth = Math.max(2, cell * (strong ? 0.09 : 0.06));
  if (kind === 'start') {{
    const size = cell * (strong ? 0.46 : 0.34);
    ctx.fillRect(p.x - size / 2, p.y - size / 2, size, size);
    ctx.strokeRect(p.x - size / 2, p.y - size / 2, size, size);
  }} else {{
    ctx.beginPath();
    ctx.arc(p.x, p.y, cell * (strong ? 0.35 : 0.25), 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
  }}
  if (label) {{
    ctx.globalAlpha = 1;
    ctx.fillStyle = '#111827';
    ctx.font = `${{Math.max(10, cell * 0.28)}}px ui-sans-serif`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'bottom';
    ctx.fillText(label, p.x, p.y - cell * 0.34);
  }}
  ctx.restore();
}}

function drawCurrentTasks(b, cell, offX, offY) {{
  const seen = new Set();
  for (const agent of DATA.agents) {{
    if (!visible(agent)) continue;
    const state = taskState(agent);
    if (state.task < 0) continue;
    const task = taskById.get(Number(state.task));
    if (!task) continue;
    const key = `${{agent.id}}:${{state.task}}:${{state.phase}}`;
    if (seen.has(key)) continue;
    seen.add(key);
    const color = colorFor(agent);
    const strong = selected === agent.id;
    if (state.phase === 'assigned') {{
      drawTaskMarker(task.start, b, cell, offX, offY, color, 'start', strong, strong ? `T${{task.id}} start` : '');
    }} else if (state.phase === 'loaded') {{
      for (const goal of task.goals || []) {{
        drawTaskMarker(goal, b, cell, offX, offY, color, 'goal', strong, strong ? `T${{task.id}} goal` : '');
      }}
    }}
  }}
}}

function drawCargo(x, y, cell, taskType) {{
  const outbound = taskType === 'outbound';
  const fill = outbound ? '#43a047' : '#f2b84b';
  const highlight = outbound ? '#9be7a0' : '#ffd980';
  const border = outbound ? '#14532d' : '#6b410f';
  const seam = outbound ? '#237a35' : '#9a6419';
  const width = Math.max(8, cell * 0.46);
  const height = Math.max(7, cell * 0.38);
  const left = x - width / 2;
  const top = y - cell * 0.74;
  ctx.save();
  ctx.fillStyle = fill;
  ctx.strokeStyle = border;
  ctx.lineWidth = Math.max(1.2, cell * 0.055);
  ctx.fillRect(left, top, width, height);
  ctx.strokeRect(left, top, width, height);
  ctx.fillStyle = highlight;
  ctx.fillRect(left + ctx.lineWidth / 2, top + ctx.lineWidth / 2,
               width - ctx.lineWidth, height * 0.28);
  ctx.strokeStyle = seam;
  ctx.lineWidth = Math.max(1, cell * 0.035);
  ctx.beginPath();
  ctx.moveTo(x, top);
  ctx.lineTo(x, top + height);
  ctx.stroke();
  ctx.restore();
}}

function draw(forceUi = false) {{
  t = Math.min(DATA.makespan, Math.floor(playhead));
  slider.value = String(t);
  const fraction = playhead - t;
  timeEl.textContent = fraction < 0.01 ? String(t) : playhead.toFixed(1);
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  const b = bounds();
  const rows = b.x1 - b.x0;
  const cols = b.y1 - b.y0;
  const pad = 28;
  const cell = Math.min((canvas.width - pad * 2) / cols, (canvas.height - pad * 2) / rows);
  const offX = (canvas.width - cols * cell) / 2;
  const offY = (canvas.height - rows * cell) / 2;

  ctx.fillStyle = '#ffffff';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  for (let r = b.x0; r < b.x1; r++) {{
    for (let c = b.y0; c < b.y1; c++) {{
      const x = offX + (c - b.y0) * cell;
      const y = offY + (r - b.x0) * cell;
      ctx.fillStyle = DATA.grid[r][c] === '@' ? '#20252b' : '#f9fafb';
      ctx.fillRect(x, y, cell, cell);
      ctx.strokeStyle = '#d8dee6';
      ctx.lineWidth = 1;
      ctx.strokeRect(x, y, cell, cell);
    }}
  }}

  drawCurrentTasks(b, cell, offX, offY);

  for (const agent of DATA.agents) {{
    if (!visible(agent)) continue;
    ctx.strokeStyle = colorFor(agent);
    ctx.globalAlpha = 0.35;
    ctx.lineWidth = Math.max(2, cell * 0.08);
    ctx.beginPath();
    let started = false;
    const start = Math.max(0, t - 24);
    for (let tt = start; tt <= t; tt++) {{
      const [r, c] = agent.path[tt];
      if (r < b.x0 || r >= b.x1 || c < b.y0 || c >= b.y1) continue;
      const x = offX + (c - b.y0 + 0.5) * cell;
      const y = offY + (r - b.x0 + 0.5) * cell;
      if (!started) {{
        ctx.moveTo(x, y);
        started = true;
      }} else {{
        ctx.lineTo(x, y);
      }}
    }}
    if (playhead > t) {{
      const [r, c] = interpolatedPosition(agent);
      if (r >= b.x0 && r < b.x1 && c >= b.y0 && c < b.y1) {{
        const x = offX + (c - b.y0 + 0.5) * cell;
        const y = offY + (r - b.x0 + 0.5) * cell;
        if (!started) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }}
    }}
    ctx.stroke();
    ctx.globalAlpha = 1;
  }}

  for (const agent of DATA.agents) {{
    if (!visible(agent)) continue;
    const [r, c] = interpolatedPosition(agent);
    if (r < b.x0 || r >= b.x1 || c < b.y0 || c >= b.y1) continue;
    const x = offX + (c - b.y0 + 0.5) * cell;
    const y = offY + (r - b.x0 + 0.5) * cell;
    ctx.fillStyle = colorFor(agent);
    ctx.beginPath();
    ctx.arc(x, y, cell * 0.32, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = '#fff';
    ctx.font = `${{Math.max(9, cell * 0.32)}}px ui-sans-serif`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(String(agent.id), x, y);
    const state = taskState(agent);
    if (state.phase === 'loaded') {{
      const task = taskById.get(Number(state.task));
      drawCargo(x, y, cell, task ? task.type : 'inbound');
    }}
  }}
  if (forceUi || renderedUiStep !== t) {{
    renderTable();
    renderTaskInfo();
    renderedUiStep = t;
  }}
}}

function renderTaskInfo() {{
  if (selected !== null) {{
    const agent = DATA.agents.find(a => a.id === selected);
    const state = agent ? taskState(agent) : {{task: -1, phase: 'idle'}};
    const task = taskById.get(Number(state.task));
    if (!agent || !task) {{
      taskInfo.innerHTML = '<strong>Selected agent</strong>: idle';
      return;
    }}
    const goals = (task.goals || []).map(g => `(${{g.x}},${{g.y}})`).join(' ');
    taskInfo.innerHTML = `<strong>${{agent.name}}</strong> ${{
      phaseLabel(state.phase)
    }} <strong>T${{task.id}}</strong> ${{task.type}}<br>start (${{task.start.x}},${{task.start.y}})<br>goals ${{goals}}`;
    return;
  }}
  let assigned = 0;
  let loaded = 0;
  for (const agent of DATA.agents) {{
    const state = taskState(agent);
    if (state.phase === 'assigned') assigned++;
    if (state.phase === 'loaded') loaded++;
  }}
  taskInfo.innerHTML = `<strong>t=${{t}}</strong>: ${{assigned}} assigned-to-start, ${{loaded}} loaded-to-goal. Click an agent row to show its task start and goal_set labels.`;
}}

function renderTable() {{
  const agents = [...DATA.agents].sort((a, b) => b.soc - a.soc);
  tbody.innerHTML = '';
  for (const a of agents) {{
    const state = taskState(a);
    const tr = document.createElement('tr');
    tr.dataset.id = a.id;
    if (a.id === selected) tr.classList.add('selected');
    tr.innerHTML = `<td><span class="swatch" style="background:${{colorFor(a)}}"></span>${{a.name}}</td><td>${{taskLabel(state)}}</td><td>${{phaseLabel(state.phase)}}</td><td>${{a.soc}}</td><td>${{a.sol}}</td><td>${{a.aba}}</td>`;
    tr.addEventListener('click', () => {{
      selected = selected === a.id ? null : a.id;
      draw(true);
    }});
    tbody.appendChild(tr);
  }}
}}

function stopPlayback() {{
  if (animationFrameId !== null) {{
    cancelAnimationFrame(animationFrameId);
    animationFrameId = null;
  }}
  lastFrameTime = null;
  playBtn.textContent = 'Play';
}}

function animate(timestamp) {{
  if (lastFrameTime === null) lastFrameTime = timestamp;
  const elapsed = Math.min(250, timestamp - lastFrameTime);
  lastFrameTime = timestamp;
  const advance =
      elapsed * Number(speedEl.value) / millisecondsPerStep;
  playhead = (playhead + advance) % (DATA.makespan + 1);
  draw();
  animationFrameId = requestAnimationFrame(animate);
}}

function startPlayback() {{
  if (animationFrameId !== null) return;
  playBtn.textContent = 'Pause';
  lastFrameTime = null;
  animationFrameId = requestAnimationFrame(animate);
}}

playBtn.addEventListener('click', () => {{
  if (animationFrameId !== null) {{
    stopPlayback();
  }} else {{
    startPlayback();
  }}
}});
slider.addEventListener('input', () => {{
  playhead = Number(slider.value);
  lastFrameTime = null;
  draw(true);
}});
modeEl.addEventListener('change', () => {{
  selected = null;
  draw(true);
}});
focusBtn.addEventListener('click', () => {{
  focusCluster = !focusCluster;
  focusBtn.classList.toggle('active', focusCluster);
  draw();
}});
draw(true);
</script>
</body>
</html>
"""


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map")
    parser.add_argument("schedule")
    parser.add_argument("output")
    parser.add_argument("--title", default="TAPF schedule visualization")
    args = parser.parse_args()

    grid = read_movingai_map(args.map)
    schedule_data = load_schedule(Path(args.schedule))
    makespan = int(schedule_data["statistics"]["makespan"])

    agents = []
    for name in sorted(schedule_data["schedule"], key=agent_index):
        goal = [
            schedule_data["assignments"][name]["x"],
            schedule_data["assignments"][name]["y"],
        ]
        dense = expand_path(schedule_data["schedule"][name], makespan)
        stats = compute_stats(name, dense, goal)
        stats["path"] = dense
        stats["timeline"] = expand_task_timeline(
            schedule_data.get("agent_task_timeline", {}).get(name, []), makespan
        )
        agents.append(stats)

    payload = {
        "grid": grid,
        "height": len(grid),
        "width": len(grid[0]),
        "makespan": makespan,
        "tasks": schedule_data.get("tasks", []),
        "agents": agents,
    }
    soc = sum(a["soc"] for a in agents)
    sol = sum(a["sol"] for a in agents)
    aba = sum(a["aba"] for a in agents)
    hidden = sum(a["hiddenWait"] for a in agents)

    rendered = HTML_TEMPLATE.format(
        title=html.escape(args.title),
        makespan=makespan,
        soc=soc,
        sol=sol,
        aba=aba,
        hidden=hidden,
        data_json=json.dumps(payload, separators=(",", ":")),
    )
    Path(args.output).write_text(rendered)
    print(args.output)


if __name__ == "__main__":
    main()
