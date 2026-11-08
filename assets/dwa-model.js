/* Browser counterpart of the sampling and scoring in main/Controllers/DWA/dwa.py. */
window.DwaModel = (() => {
  const linspace = (min, max, count) => Array.from({ length: count }, (_, i) => min + (max - min) * i / (count - 1));
  const normalize = values => {
    const low = Math.min(...values), span = Math.max(...values) - low;
    return values.map(value => span < 1e-12 ? 0 : (value - low) / span);
  };
  const wrap = angle => ((angle + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI;

  function dynamicWindow(state, config) {
    const speed = state[3], turn = state[4], dt = config.dt;
    return {
      linear: [Math.max(config.minLinVel, speed - dt * config.maxLinearAcc),
               Math.min(config.maxLinVel, speed + dt * config.maxLinearAcc)],
      angular: [Math.max(config.minAngVel, turn - dt * config.maxAngAcc),
                Math.min(config.maxAngVel, turn + dt * config.maxAngAcc)]
    };
  }

  function candidates(state, goal, obstacles, config, weights, simTime) {
    const [x, y, theta] = state;
    const dt = config.dt;
    const window = dynamicWindow(state, config);
    const linear = linspace(window.linear[0], window.linear[1], config.vSamples);
    const angular = linspace(window.angular[0], window.angular[1], config.wSamples);
    const localObstacles = obstacles.filter(point => Math.hypot(x - point[0], y - point[1]) < config.obstacleMaxDist);
    const steps = Math.max(2, Math.round(simTime / config.timeGranularity));
    const paths = [], commands = [], heading = [], velocity = [], clearance = [];
    for (const w of angular) for (const v of linear) {
      const path = [[x, y]];
      let px = x, py = y, angle = theta, nearest = 2;
      for (let step = 1; step < steps; step++) {
        px += v * Math.cos(angle) * dt;
        py += v * Math.sin(angle) * dt;
        angle += w * dt;
        path.push([px, py]);
      }
      for (const obstacle of localObstacles) for (const point of path) {
        nearest = Math.min(nearest, Math.hypot(point[0] - obstacle[0], point[1] - obstacle[1]));
      }
      if (nearest < config.radius + config.collisionTol) nearest = -100;
      paths.push(path);
      commands.push([v, w]);
      heading.push(Math.PI - Math.abs(wrap(Math.atan2(goal[1] - py, goal[0] - px) - angle)));
      velocity.push(v + Math.exp(-Math.hypot(goal[0] - px, goal[1] - py) / config.goalDistTol));
      clearance.push(nearest);
    }
    const h = normalize(heading), v = normalize(velocity), o = normalize(clearance);
    let chosen = 0, best = -Infinity;
    for (let i = 0; i < paths.length; i++) {
      const score = weights.heading * h[i] + weights.velocity * v[i] + weights.obstacle * o[i];
      if (score > best) { best = score; chosen = i; }
    }
    return { paths, commands, chosen };
  }

  function run(start, goal, obstacles, config, weights, simTime) {
    const trajectory = [[start[0], start[1], start[2], 0, 0]];
    const clearance = config.radius + config.collisionTol;
    function collision(from, to) {
      const dx = to[0] - from[0], dy = to[1] - from[1];
      const lengthSq = dx * dx + dy * dy;
      for (const obstacle of obstacles) {
        const t = lengthSq ? Math.max(0, Math.min(1,
          ((obstacle[0] - from[0]) * dx + (obstacle[1] - from[1]) * dy) / lengthSq)) : 0;
        const distance = Math.hypot(from[0] + t * dx - obstacle[0], from[1] + t * dy - obstacle[1]);
        if (distance < clearance) return { obstacle, distance };
      }
      return null;
    }
    const initialCollision = collision(trajectory[0], trajectory[0]);
    if (initialCollision) return { trajectory, success: false, event: 'collision', collision: initialCollision };
    for (let step = 0; step < config.maxSteps; step++) {
      const state = trajectory[trajectory.length - 1];
      if (Math.hypot(state[0] - goal[0], state[1] - goal[1]) < config.goalDistTol) {
        return { trajectory, success: true, event: 'goal' };
      }
      const sample = candidates(state, goal, obstacles, config, weights, simTime);
      const [v, w] = sample.commands[sample.chosen];
      const next = [state[0] + v * Math.cos(state[2]) * config.dt,
                    state[1] + v * Math.sin(state[2]) * config.dt,
                    state[2] + w * config.dt, v, w];
      trajectory.push(next);
      const hit = collision(state, next);
      if (hit) return { trajectory, success: false, event: 'collision', collision: hit };
    }
    const finalState = trajectory[trajectory.length - 1];
    const success = Math.hypot(finalState[0] - goal[0], finalState[1] - goal[1]) < config.goalDistTol;
    return { trajectory, success, event: success ? 'goal' : 'step-limit' };
  }
  return { candidates, dynamicWindow, run };
})();
