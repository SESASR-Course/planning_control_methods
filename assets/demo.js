(() => {
  const kind = document.body.dataset.demo;
  if (!kind) return;
  const canvas = document.getElementById('plot');
  const ctx = canvas.getContext('2d');
  const progress = document.getElementById('progress');
  const colors = { mint: '#6ce0bd', gold: '#ffca79', blue: '#80b4ff', muted: '#5b7784' };
  let data, timer, gridState, dwaState, candidateCache, replanTicket = 0;

  function status(id, message) { document.getElementById(id).textContent = message; }
  function selectPoint(mode) {
    const state = kind === 'grid' ? gridState : dwaState;
    state.mode = mode;
    for (const name of ['start', 'goal']) {
      const button = document.getElementById(`set-${name}`);
      button.classList.toggle('active', name === mode);
      button.setAttribute('aria-pressed', String(name === mode));
    }
    status(kind === 'grid' ? 'grid-status' : 'dwa-status',
      `Click ${kind === 'grid' ? 'a free cell' : 'a clear location'} to move the ${mode}.`);
  }
  function gridGeometry(w, h, map) {
    const rows = map.grid.length, cols = map.grid[0].length;
    const cell = Math.min((w - 28) / cols, (h - 28) / rows);
    const x0 = (w - cell * cols) / 2, y0 = (h - cell * rows) / 2;
    return { cell, x0, y0, xy: p => [x0 + (p[1] + .5) * cell, y0 + (p[0] + .5) * cell],
      fromPixel: (x, y) => [Math.floor((y - y0) / cell), Math.floor((x - x0) / cell)] };
  }

  function fit() {
    const rect = canvas.getBoundingClientRect();
    const ratio = window.devicePixelRatio || 1;
    canvas.width = Math.round(rect.width * ratio);
    canvas.height = Math.round(rect.height * ratio);
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    if (data) draw();
  }
  function line(points, xy, color, width = 2, dash = []) {
    if (!points.length) return;
    ctx.beginPath();
    points.forEach((p, i) => { const [x, y] = xy(p); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
    ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash); ctx.stroke(); ctx.setLineDash([]);
  }
  function dot(x, y, radius, color) {
    ctx.beginPath(); ctx.arc(x, y, radius, 0, Math.PI * 2); ctx.fillStyle = color; ctx.fill();
  }
  function diamond(x, y, radius, color) {
    ctx.beginPath(); ctx.moveTo(x, y - radius); ctx.lineTo(x + radius, y);
    ctx.lineTo(x, y + radius); ctx.lineTo(x - radius, y); ctx.closePath();
    ctx.fillStyle = color; ctx.fill();
  }
  function background(w, h) {
    ctx.clearRect(0, 0, w, h); ctx.fillStyle = '#091a26'; ctx.fillRect(0, 0, w, h);
  }
  function drawGrid(w, h) {
    const map = data.maps[gridState.mapName];
    const rows = map.grid.length, cols = map.grid[0].length;
    const { cell, x0, y0, xy } = gridGeometry(w, h, map);
    const result = gridState.result;
    const frame = Math.min(Number(progress.value), Math.max(0, result.path.length - 1));
    for (let r = 0; r < rows; r++) for (let c = 0; c < cols; c++) {
      const value = map.costmap[r][c];
      const level = Math.max(0, Math.min(1, (value - 1) / 99));
      ctx.fillStyle = map.grid[r][c] ? '#934a83' : `hsl(203 75% ${92 - 52 * level}%)`;
      ctx.fillRect(x0 + c * cell + .6, y0 + r * cell + .6, cell - 1.2, cell - 1.2);
    }
    if (result.path.length) {
      line(result.path.slice(0, frame + 1), xy, '#493015', Math.max(6, cell * .36));
      line(result.path.slice(0, frame + 1), xy, colors.gold, Math.max(3, cell * .2));
      dot(...xy(result.path[frame]), Math.max(5, cell * .25), '#103e49');
    }
    dot(...xy(gridState.start), Math.max(5, cell * .3), colors.mint);
    diamond(...xy(gridState.goal), Math.max(6, cell * .34), '#ff9e72');
    document.getElementById('explored').textContent = result.visited.length;
    document.getElementById('path-length').textContent = result.path.length ? `${result.distance.toFixed(2)} cells` : 'No path';
  }
  function world(w, h, bounds) {
    const pad = 35, [xmin, ymin, xmax, ymax] = bounds;
    const scale = Math.min((w - 2 * pad) / (xmax - xmin), (h - 2 * pad) / (ymax - ymin));
    const ox = (w - (xmax - xmin) * scale) / 2, oy = (h - (ymax - ymin) * scale) / 2;
    const xy = p => [ox + (p[0] - xmin) * scale, h - oy - (p[1] - ymin) * scale];
    xy.fromPixel = (x, y) => [xmin + (x - ox) / scale, ymin + (h - oy - y) / scale];
    return xy;
  }
  function drawAxes(xy, bounds) {
    const [xmin, ymin, xmax, ymax] = bounds;
    ctx.strokeStyle = '#1d3848'; ctx.lineWidth = 1;
    for (let x = Math.ceil(xmin); x <= xmax; x++) line([[x, ymin], [x, ymax]], xy, '#1d3848', 1);
    for (let y = Math.ceil(ymin); y <= ymax; y++) line([[xmin, y], [xmax, y]], xy, '#1d3848', 1);
  }
  function drawRobot(p, xy, color) {
    const [x, y] = xy(p); dot(x, y, 8, color);
    ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + Math.cos(p[2]) * 18, y - Math.sin(p[2]) * 18);
    ctx.strokeStyle = '#fff'; ctx.lineWidth = 2; ctx.stroke();
  }
  function drawDwa(w, h) {
    const xy = world(w, h, [0, 0, 10, 10]); drawAxes(xy, [0, 0, 10, 10]);
    const end = Math.min(Number(progress.value), dwaState.trajectory.length - 1);
    const state = dwaState.trajectory[end];
    if (!candidateCache || candidateCache.frame !== end) {
      candidateCache = { frame: end, sample: DwaModel.candidates(state, dwaState.goal, data.obstacles,
        data.config, dwaState.weights, dwaState.simTime) };
    }
    const sample = candidateCache.sample;
    sample.paths.forEach((candidate, i) => {
      if (i !== sample.chosen) line(candidate, xy, '#80b4ff99', 1.4);
    });
    data.obstacles.forEach(p => dot(...xy(p), 7, '#e17b83'));
    const trace = dwaState.trajectory.slice(0, end + 1);
    line(trace, xy, colors.gold, 3);
    line(sample.paths[sample.chosen], xy, '#42f5a4', 3.2);
    dot(...xy(dwaState.start), 7, colors.mint);
    diamond(...xy(dwaState.goal), 10, '#ff9e72');
    drawRobot(state, xy, '#f8f8ef');
    drawDwaInset(w, h, sample, state);
    document.getElementById('position').textContent = `${state[0].toFixed(1)}, ${state[1].toFixed(1)} m`;
    document.getElementById('speed').textContent = `${state[3].toFixed(2)} m/s`;
  }
  function drawDwaInset(w, h, sample, state) {
    const width = Math.min(250, w * .42), height = Math.min(210, h * .47);
    const left = w - width - 15, top = 15;
    const farthest = Math.max(...sample.paths.map(path => {
      const end = path[path.length - 1];
      return Math.hypot(end[0] - state[0], end[1] - state[1]);
    }));
    const radius = Math.max(.16, farthest * 1.18);
    const scale = Math.min((width - 28) / (2 * radius), (height - 50) / (2 * radius));
    const local = point => [left + width / 2 + (point[0] - state[0]) * scale,
      top + height / 2 - (point[1] - state[1]) * scale];
    ctx.fillStyle = '#0a2334f2'; ctx.fillRect(left, top, width, height);
    ctx.strokeStyle = '#66869b'; ctx.lineWidth = 1; ctx.strokeRect(left + .5, top + .5, width - 1, height - 1);
    ctx.save(); ctx.beginPath(); ctx.rect(left + 5, top + 27, width - 10, height - 32); ctx.clip();
    sample.paths.forEach((path, i) => { if (i !== sample.chosen) line(path, local, '#80b4ffaa', 1.2); });
    line(sample.paths[sample.chosen], local, '#42f5a4', 3);
    dot(...local(state), 5, '#fff');
    ctx.restore();
    ctx.font = '600 12px DM Sans, sans-serif'; ctx.fillStyle = '#eaf5f4';
    ctx.fillText('Candidate detail · zoom', left + 12, top + 18);
  }
  function drawPursuit(w, h) {
    const xy = world(w, h, [-1, -1, 6, 6]); drawAxes(xy, [-1, -1, 6, 6]);
    line(data.path, xy, '#607d9b', 2, [6, 5]);
    data.waypoints.forEach(p => dot(...xy(p), 4, '#a3b6bd'));
    const end = Number(progress.value), sample = data.samples[end];
    line(data.samples.slice(0, end + 1).map(p => [p[1], p[2]]), xy, colors.mint, 3);
    const target = data.lookahead_path[sample[5]];
    line([[sample[1], sample[2]], target], xy, '#ffca7980', 1.5, [5, 4]);
    dot(...xy(target), 7, colors.gold);
    drawRobot([sample[1], sample[2], sample[3]], xy, colors.mint);
    document.getElementById('time').textContent = `${sample[0].toFixed(1)} s`;
    document.getElementById('speed').textContent = `${sample[4].toFixed(2)} m/s`;
  }
  function draw() {
    const { width: w, height: h } = canvas.getBoundingClientRect(); background(w, h);
    if (kind === 'grid') drawGrid(w, h);
    else if (kind === 'dwa') drawDwa(w, h);
    else drawPursuit(w, h);
  }
  function stop() { clearInterval(timer); timer = null; const button = document.getElementById('play'); if (button) button.textContent = 'Play ▶'; }
  function replanGrid() {
    stop();
    gridState.result = PlannerModel.plan(data.maps[gridState.mapName], gridState.start, gridState.goal,
      document.getElementById('algorithm').value, Number(document.getElementById('connectivity').value));
    progress.max = Math.max(0, gridState.result.path.length - 1);
    progress.value = 0;
    document.getElementById('play').disabled = gridState.result.path.length < 2;
    document.getElementById('reset').disabled = gridState.result.path.length < 2;
    status('grid-status', gridState.result.path.length
      ? `Route found. Click a free cell to move the ${gridState.mode}.`
      : 'No route connects these points. Move the start or goal.');
    draw();
  }
  function initGrid() {
    const maps = document.getElementById('map');
    Object.keys(data.maps).forEach(name => maps.add(new Option(name, name)));
    maps.value = data.default;
    gridState = { mapName: data.default, start: [...data.maps[data.default].start],
      goal: [...data.maps[data.default].goal], mode: 'start' };
    selectPoint('start');
    document.getElementById('set-start').addEventListener('click', () => selectPoint('start'));
    document.getElementById('set-goal').addEventListener('click', () => selectPoint('goal'));
    maps.addEventListener('change', () => {
      gridState.mapName = maps.value;
      gridState.start = [...data.maps[maps.value].start];
      gridState.goal = [...data.maps[maps.value].goal];
      replanGrid();
    });
    document.getElementById('algorithm').addEventListener('change', replanGrid);
    document.getElementById('connectivity').addEventListener('change', replanGrid);
    canvas.addEventListener('click', event => {
      const rect = canvas.getBoundingClientRect();
      const point = gridGeometry(rect.width, rect.height, data.maps[gridState.mapName])
        .fromPixel(event.clientX - rect.left, event.clientY - rect.top);
      const map = data.maps[gridState.mapName];
      if (point[0] < 0 || point[0] >= map.grid.length || point[1] < 0 || point[1] >= map.grid[0].length) return;
      if (map.grid[point[0]][point[1]]) {
        status('grid-status', 'That cell is occupied. Choose a blue free cell.'); return;
      }
      gridState[gridState.mode] = point;
      replanGrid();
    });
    replanGrid();
  }
  function updateDwaReadouts() {
    for (const id of ['weight-heading', 'weight-velocity', 'weight-obstacle']) {
      document.getElementById(`${id}-value`).textContent = Number(document.getElementById(id).value).toFixed(2);
    }
    document.getElementById('heading-value').textContent = `${document.getElementById('heading').value}°`;
    document.getElementById('sim-time-value').textContent = `${Number(document.getElementById('sim-time').value).toFixed(1)} s`;
  }
  function replanDwa() {
    stop();
    const ticket = ++replanTicket;
    dwaState.start[2] = Number(document.getElementById('heading').value) * Math.PI / 180;
    dwaState.weights = { heading: Number(document.getElementById('weight-heading').value),
      velocity: Number(document.getElementById('weight-velocity').value),
      obstacle: Number(document.getElementById('weight-obstacle').value) };
    dwaState.simTime = Number(document.getElementById('sim-time').value);
    status('dwa-status', 'Recalculating candidate motions and route…');
    requestAnimationFrame(() => {
      if (ticket !== replanTicket) return;
      const result = DwaModel.run(dwaState.start, dwaState.goal, data.obstacles,
        data.config, dwaState.weights, dwaState.simTime);
      dwaState.trajectory = result.trajectory;
      dwaState.success = result.success;
      candidateCache = null;
      progress.max = Math.max(0, result.trajectory.length - 1);
      progress.value = 0;
      document.getElementById('play').disabled = result.trajectory.length < 2;
      status('dwa-status', result.success ? 'Goal reached. Click the map or adjust a parameter to try another run.'
        : 'The goal was not reached within 300 steps. Try new points or weights.');
      draw();
    });
  }
  function initDwa() {
    dwaState = { start: [...data.start], goal: [...data.goal], trajectory: data.trajectory,
      success: data.success, weights: { heading: data.defaults.heading,
        velocity: data.defaults.velocity, obstacle: data.defaults.obstacle },
      simTime: data.defaults.simTime, mode: 'start' };
    for (const [id, value] of [['weight-heading', data.defaults.heading],
      ['weight-velocity', data.defaults.velocity], ['weight-obstacle', data.defaults.obstacle],
      ['sim-time', data.defaults.simTime]]) document.getElementById(id).value = value;
    document.getElementById('heading').value = Math.round(data.start[2] * 180 / Math.PI);
    updateDwaReadouts();
    selectPoint('start');
    document.getElementById('set-start').addEventListener('click', () => selectPoint('start'));
    document.getElementById('set-goal').addEventListener('click', () => selectPoint('goal'));
    for (const id of ['heading', 'weight-heading', 'weight-velocity', 'weight-obstacle', 'sim-time']) {
      document.getElementById(id).addEventListener('input', updateDwaReadouts);
      document.getElementById(id).addEventListener('change', replanDwa);
    }
    canvas.addEventListener('click', event => {
      const rect = canvas.getBoundingClientRect();
      const px = event.clientX - rect.left, py = event.clientY - rect.top;
      const insetWidth = Math.min(250, rect.width * .42), insetHeight = Math.min(210, rect.height * .47);
      if (px > rect.width - insetWidth - 15 && px < rect.width - 15 && py > 15 && py < insetHeight + 15) return;
      const [x, y] = world(rect.width, rect.height, [0, 0, 10, 10]).fromPixel(px, py);
      if (x < 0 || x > 10 || y < 0 || y > 10) return;
      if (data.obstacles.some(obstacle => Math.hypot(x - obstacle[0], y - obstacle[1]) < data.config.radius + data.config.collisionTol)) {
        status('dwa-status', 'That point is too close to an obstacle. Choose a clear location.'); return;
      }
      if (dwaState.mode === 'start') dwaState.start = [Number(x.toFixed(2)), Number(y.toFixed(2)), dwaState.start[2]];
      else dwaState.goal = [Number(x.toFixed(2)), Number(y.toFixed(2))];
      replanDwa();
    });
    progress.max = data.trajectory.length - 1;
    draw();
  }
  function init() {
    if (kind === 'grid') initGrid();
    else if (kind === 'dwa') initDwa();
    else progress.max = data.samples.length - 1;
    document.getElementById('play').addEventListener('click', () => {
      if (timer) return stop();
      if (Number(progress.max) < 1) return;
      if (Number(progress.value) >= Number(progress.max)) progress.value = 0;
      document.getElementById('play').textContent = 'Pause ❚❚';
      timer = setInterval(() => { progress.value = Number(progress.value) + 1; draw(); if (Number(progress.value) >= Number(progress.max)) stop(); }, kind === 'grid' ? 150 : 55);
    });
    document.getElementById('reset').addEventListener('click', () => {
      stop(); progress.value = 0; draw();
      if (kind === 'grid') document.getElementById('play').click();
    });
    progress.addEventListener('input', () => { candidateCache = null; draw(); });
    window.addEventListener('resize', fit); fit();
  }
  fetch(`data/${kind}.json`).then(response => { if (!response.ok) throw Error(`HTTP ${response.status}`); return response.json(); })
    .then(json => { data = json; init(); })
    .catch(error => { document.querySelector('.caption').textContent = `Could not load example data: ${error.message}`; });
})();
