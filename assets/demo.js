(() => {
  const kind = document.body.dataset.demo;
  if (!kind) return;
  const canvas = document.getElementById('plot');
  const ctx = canvas.getContext('2d');
  const progress = document.getElementById('progress');
  const colors = { mint: '#6ce0bd', gold: '#ffca79', blue: '#80b4ff', muted: '#5b7784' };
  let data, timer;

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
  function background(w, h) {
    ctx.clearRect(0, 0, w, h); ctx.fillStyle = '#091a26'; ctx.fillRect(0, 0, w, h);
  }
  function drawGrid(w, h) {
    const rows = data.grid.length, cols = data.grid[0].length;
    const cell = Math.min((w - 28) / cols, (h - 28) / rows);
    const x0 = (w - cell * cols) / 2, y0 = (h - cell * rows) / 2;
    const xy = p => [x0 + (p[1] + .5) * cell, y0 + (p[0] + .5) * cell];
    const result = data.results[`${document.getElementById('algorithm').value}-${document.getElementById('connectivity').value}`];
    const visible = Math.round(result.visited.length * Number(progress.value) / 100);
    for (let r = 0; r < rows; r++) for (let c = 0; c < cols; c++) {
      ctx.fillStyle = data.grid[r][c] ? '#375464' : '#112c3a';
      ctx.fillRect(x0 + c * cell + 1, y0 + r * cell + 1, cell - 2, cell - 2);
    }
    result.visited.slice(0, visible).forEach(p => { ctx.fillStyle = '#23665e'; ctx.fillRect(x0 + p[1] * cell + 1, y0 + p[0] * cell + 1, cell - 2, cell - 2); });
    if (visible === result.visited.length) line(result.path, xy, colors.gold, Math.max(3, cell * .24));
    dot(...xy(data.start), cell * .3, colors.mint);
    dot(...xy(data.goal), cell * .3, colors.gold);
    document.getElementById('explored').textContent = `${visible} / ${result.visited.length}`;
    document.getElementById('path-length').textContent = `${result.cost} cells`;
  }
  function world(w, h, bounds) {
    const pad = 35, [xmin, ymin, xmax, ymax] = bounds;
    const scale = Math.min((w - 2 * pad) / (xmax - xmin), (h - 2 * pad) / (ymax - ymin));
    const ox = (w - (xmax - xmin) * scale) / 2, oy = (h - (ymax - ymin) * scale) / 2;
    return p => [ox + (p[0] - xmin) * scale, h - oy - (p[1] - ymin) * scale];
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
    data.candidates.forEach((candidate, i) => line(candidate, xy, i === data.chosen ? colors.mint : '#345976', i === data.chosen ? 2.5 : 1));
    data.obstacles.forEach(p => dot(...xy(p), 7, '#dc826f'));
    const end = Number(progress.value), trace = data.trajectory.slice(0, end + 1);
    line(trace, xy, colors.gold, 3);
    dot(...xy(data.goal), 10, colors.gold);
    drawRobot(data.trajectory[end], xy, colors.mint);
    document.getElementById('position').textContent = `${data.trajectory[end][0].toFixed(1)}, ${data.trajectory[end][1].toFixed(1)} m`;
    document.getElementById('speed').textContent = `${data.trajectory[end][3].toFixed(2)} m/s`;
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
  function init() {
    if (kind !== 'grid') {
      progress.max = kind === 'dwa' ? data.trajectory.length - 1 : data.samples.length - 1;
      document.getElementById('play').addEventListener('click', () => {
        if (timer) return stop();
        if (Number(progress.value) >= Number(progress.max)) progress.value = 0;
        document.getElementById('play').textContent = 'Pause ❚❚';
        timer = setInterval(() => { progress.value = Number(progress.value) + 1; draw(); if (Number(progress.value) >= Number(progress.max)) stop(); }, 55);
      });
      document.getElementById('reset').addEventListener('click', () => { stop(); progress.value = 0; draw(); });
    } else {
      document.getElementById('algorithm').addEventListener('change', draw);
      document.getElementById('connectivity').addEventListener('change', draw);
    }
    progress.addEventListener('input', draw);
    window.addEventListener('resize', fit); fit();
  }
  fetch(`data/${kind}.json`).then(response => { if (!response.ok) throw Error(`HTTP ${response.status}`); return response.json(); })
    .then(json => { data = json; init(); })
    .catch(error => { document.querySelector('.caption').textContent = `Could not load example data: ${error.message}`; });
})();
