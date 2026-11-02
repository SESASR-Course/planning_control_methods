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
    const modes = kind === 'grid' ? ['start', 'goal'] : ['start', 'goal', 'add-obstacle', 'remove-obstacle'];
    for (const name of modes) {
      const id = name === 'start' || name === 'goal' ? `set-${name}` : name;
      const button = document.getElementById(id);
      button.classList.toggle('active', name === mode);
      button.setAttribute('aria-pressed', String(name === mode));
    }
    const instruction = kind === 'grid' ? `Click a free cell to move the ${mode}.` : {
      start: 'Click a clear location to move the start.',
      goal: 'Click a clear location to move the goal.',
      'add-obstacle': 'Click a clear location to add an obstacle.',
      'remove-obstacle': 'Click a red obstacle to remove it.'
    }[mode];
    status(kind === 'grid' ? 'grid-status' : 'dwa-status', instruction);
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
    const frame = Math.min(Number(progress.value), Number(progress.max));
    const exploredVisible = Math.min(result.expanded.length, frame + 1);
    const pathVisible = Math.max(0, Math.min(result.path.length, frame - result.expanded.length + 1));
    for (let r = 0; r < rows; r++) for (let c = 0; c < cols; c++) {
      const value = map.costmap[r][c];
      const level = Math.max(0, Math.min(1, (value - 1) / 99));
      ctx.fillStyle = map.grid[r][c] ? '#4e5751' : `hsl(${80 + 72 * level} ${20 - 8 * level}% ${89 - 40 * level}%)`;
      ctx.fillRect(x0 + c * cell + .6, y0 + r * cell + .6, cell - 1.2, cell - 1.2);
    }
    result.expanded.slice(0, exploredVisible).forEach(([row, col]) => {
      ctx.fillStyle = '#d3ae8a';
      ctx.fillRect(x0 + col * cell + 1.2, y0 + row * cell + 1.2, cell - 2.4, cell - 2.4);
    });
    if (exploredVisible && exploredVisible < result.expanded.length) {
      const [row, col] = result.expanded[exploredVisible - 1];
      ctx.strokeStyle = '#7b4f3e'; ctx.lineWidth = Math.max(2, cell * .1);
      ctx.strokeRect(x0 + col * cell + 2, y0 + row * cell + 2, cell - 4, cell - 4);
    }
    if (pathVisible) {
      line(result.path.slice(0, pathVisible), xy, '#5a4136', Math.max(6, cell * .36));
      line(result.path.slice(0, pathVisible), xy, '#b76e48', Math.max(3, cell * .2));
      dot(...xy(result.path[pathVisible - 1]), Math.max(5, cell * .25), '#264d48');
    }
    dot(...xy(gridState.start), Math.max(5, cell * .3), '#4d7c66');
    diamond(...xy(gridState.goal), Math.max(6, cell * .34), '#bc634d');
    document.getElementById('explored').textContent = `${exploredVisible} / ${result.expanded.length}`;
    document.getElementById('grid-phase').textContent = !result.path.length && frame >= result.expanded.length - 1
      ? 'No route' : frame < result.expanded.length
        ? 'Exploring cells' : pathVisible < result.path.length ? 'Drawing route' : 'Complete';
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
      candidateCache = { frame: end, sample: DwaModel.candidates(state, dwaState.goal, dwaState.obstacles,
        dwaState.config, dwaState.weights, dwaState.simTime) };
    }
    const sample = candidateCache.sample;
    sample.paths.forEach((candidate, i) => {
      if (i !== sample.chosen) line(candidate, xy, '#80b4ff99', 1.4);
    });
    dwaState.obstacles.forEach(p => dot(...xy(p), 8, '#e17b83'));
    const trace = dwaState.trajectory.slice(0, end + 1);
    line(trace, xy, colors.gold, 3);
    line(sample.paths[sample.chosen], xy, '#42f5a4', 3.2);
    dot(...xy(dwaState.start), 7, colors.mint);
    diamond(...xy(dwaState.goal), 10, '#ff9e72');
    drawRobot(state, xy, '#f8f8ef');
    const limits = DwaModel.dynamicWindow(state, dwaState.config);
    document.getElementById('v-window').textContent = `${limits.linear[0].toFixed(2)}–${limits.linear[1].toFixed(2)} m/s`;
    document.getElementById('w-window').textContent = `${limits.angular[0].toFixed(2)}–${limits.angular[1].toFixed(2)} rad/s`;
    document.getElementById('candidate-count').textContent = `${dwaState.config.vSamples} × ${dwaState.config.wSamples} = ${sample.paths.length}`;
    document.getElementById('obstacle-count').textContent = dwaState.obstacles.length;
    document.getElementById('step-value').textContent = end;
    document.getElementById('position').textContent = `${state[0].toFixed(1)}, ${state[1].toFixed(1)} m`;
    document.getElementById('speed').textContent = `${state[3].toFixed(2)} m/s`;
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
  function replanGrid(autoPlay = true) {
    stop();
    gridState.result = PlannerModel.plan(data.maps[gridState.mapName], gridState.start, gridState.goal,
      document.getElementById('algorithm').value, Number(document.getElementById('connectivity').value));
    progress.max = Math.max(0, gridState.result.expanded.length + gridState.result.path.length - 1);
    progress.value = 0;
    document.getElementById('play').disabled = Number(progress.max) < 1;
    document.getElementById('reset').disabled = Number(progress.max) < 1;
    status('grid-status', gridState.result.path.length
      ? `Route found. Click a free cell to move the ${gridState.mode}.`
      : 'No route connects these points. Move the start or goal.');
    draw();
    if (autoPlay && Number(progress.max) > 0) document.getElementById('play').click();
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
        status('grid-status', 'That cell is occupied. Choose a light free cell.'); return;
      }
      gridState[gridState.mode] = point;
      replanGrid();
    });
    replanGrid(false);
  }
  function updateDwaReadouts() {
    for (const id of ['weight-heading', 'weight-velocity', 'weight-obstacle']) {
      document.getElementById(`${id}-value`).textContent = Number(document.getElementById(id).value).toFixed(2);
    }
    document.getElementById('heading-value').textContent = `${document.getElementById('heading').value}°`;
    document.getElementById('sim-time-value').textContent = `${Number(document.getElementById('sim-time').value).toFixed(1)} s`;
    document.getElementById('v-samples-value').textContent = document.getElementById('v-samples').value;
    document.getElementById('w-samples-value').textContent = document.getElementById('w-samples').value;
  }
  function replanDwa() {
    stop();
    const ticket = ++replanTicket;
    dwaState.start[2] = Number(document.getElementById('heading').value) * Math.PI / 180;
    dwaState.weights = { heading: Number(document.getElementById('weight-heading').value),
      velocity: Number(document.getElementById('weight-velocity').value),
      obstacle: Number(document.getElementById('weight-obstacle').value) };
    dwaState.simTime = Number(document.getElementById('sim-time').value);
    dwaState.config.vSamples = Number(document.getElementById('v-samples').value);
    dwaState.config.wSamples = Number(document.getElementById('w-samples').value);
    status('dwa-status', 'Recalculating candidate motions and route…');
    requestAnimationFrame(() => {
      if (ticket !== replanTicket) return;
      const result = DwaModel.run(dwaState.start, dwaState.goal, dwaState.obstacles,
        dwaState.config, dwaState.weights, dwaState.simTime);
      dwaState.trajectory = result.trajectory;
      dwaState.success = result.success;
      candidateCache = null;
      progress.max = Math.max(0, result.trajectory.length - 1);
      progress.value = Math.min(20, progress.max);
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
      simTime: data.defaults.simTime, config: { ...data.config },
      obstacles: data.obstacles.map(point => [...point]), mode: 'start' };
    for (const [id, value] of [['weight-heading', data.defaults.heading],
      ['weight-velocity', data.defaults.velocity], ['weight-obstacle', data.defaults.obstacle],
      ['sim-time', data.defaults.simTime], ['v-samples', data.config.vSamples],
      ['w-samples', data.config.wSamples]]) document.getElementById(id).value = value;
    document.getElementById('heading').value = Math.round(data.start[2] * 180 / Math.PI);
    updateDwaReadouts();
    selectPoint('start');
    document.getElementById('set-start').addEventListener('click', () => selectPoint('start'));
    document.getElementById('set-goal').addEventListener('click', () => selectPoint('goal'));
    document.getElementById('add-obstacle').addEventListener('click', () => selectPoint('add-obstacle'));
    document.getElementById('remove-obstacle').addEventListener('click', () => selectPoint('remove-obstacle'));
    document.getElementById('restore-obstacles').addEventListener('click', () => {
      dwaState.obstacles = data.obstacles.map(point => [...point]);
      replanDwa();
    });
    for (const id of ['heading', 'weight-heading', 'weight-velocity', 'weight-obstacle', 'sim-time', 'v-samples', 'w-samples']) {
      document.getElementById(id).addEventListener('input', updateDwaReadouts);
      document.getElementById(id).addEventListener('change', replanDwa);
    }
    canvas.addEventListener('click', event => {
      const rect = canvas.getBoundingClientRect();
      const px = event.clientX - rect.left, py = event.clientY - rect.top;
      const [x, y] = world(rect.width, rect.height, [0, 0, 10, 10]).fromPixel(px, py);
      if (x < 0 || x > 10 || y < 0 || y > 10) return;
      const closest = dwaState.obstacles.reduce((best, obstacle, index) => {
        const distance = Math.hypot(x - obstacle[0], y - obstacle[1]);
        return distance < best.distance ? { index, distance } : best;
      }, { index: -1, distance: Infinity });
      if (dwaState.mode === 'remove-obstacle') {
        if (closest.distance > .35) {
          status('dwa-status', 'Click directly on a red obstacle to remove it.'); return;
        }
        dwaState.obstacles.splice(closest.index, 1);
        replanDwa();
        return;
      }
      if (dwaState.mode === 'add-obstacle') {
        if (closest.distance < .35) {
          status('dwa-status', 'There is already an obstacle here.'); return;
        }
        const clearance = dwaState.config.radius + dwaState.config.collisionTol;
        if (Math.hypot(x - dwaState.start[0], y - dwaState.start[1]) < clearance ||
            Math.hypot(x - dwaState.goal[0], y - dwaState.goal[1]) < clearance) {
          status('dwa-status', 'Keep obstacles clear of the start and goal.'); return;
        }
        dwaState.obstacles.push([Number(x.toFixed(2)), Number(y.toFixed(2))]);
        replanDwa();
        return;
      }
      if (closest.distance < dwaState.config.radius + dwaState.config.collisionTol) {
        status('dwa-status', 'That point is too close to an obstacle. Choose a clear location.'); return;
      }
      if (dwaState.mode === 'start') dwaState.start = [Number(x.toFixed(2)), Number(y.toFixed(2)), dwaState.start[2]];
      else dwaState.goal = [Number(x.toFixed(2)), Number(y.toFixed(2))];
      replanDwa();
    });
    progress.max = data.trajectory.length - 1;
    progress.value = Math.min(20, progress.max);
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
      timer = setInterval(() => {
        const step = kind === 'grid' && Number(progress.value) < gridState.result.expanded.length
          ? Math.max(1, Math.ceil(gridState.result.expanded.length / 220)) : 1;
        progress.value = Math.min(Number(progress.max), Number(progress.value) + step);
        draw();
        if (Number(progress.value) >= Number(progress.max)) stop();
      }, 55);
    });
    document.getElementById('reset').addEventListener('click', () => {
      stop(); progress.value = 0; draw();
      if (kind === 'grid') document.getElementById('play').click();
    });
    progress.addEventListener('input', () => { candidateCache = null; draw(); });
    window.addEventListener('resize', fit); fit();
    if (kind === 'grid' && Number(progress.max) > 0) document.getElementById('play').click();
  }
  fetch(`data/${kind}.json`).then(response => { if (!response.ok) throw Error(`HTTP ${response.status}`); return response.json(); })
    .then(json => { data = json; init(); })
    .catch(error => { document.querySelector('.caption').textContent = `Could not load example data: ${error.message}`; });
})();
