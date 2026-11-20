// Browser version of Controllers/PurePursuit/pure_pursuit.py and utils.py.
const PurePursuitModel = (() => {
  const dt = 0.1;
  const steps = 1200;
  const clamp = (value, min, max) => Math.max(min, Math.min(max, value));

  function interpolate(waypoints) {
    const path = [];
    for (let i = 0; i < waypoints.length - 1; i++) {
      const [x0, y0] = waypoints[i], [x1, y1] = waypoints[i + 1];
      const count = Math.floor(Math.hypot(x1 - x0, y1 - y0) / 0.01);
      for (let j = 0; j < count; j++) {
        const fraction = j / (count - 1);
        path.push([x0 + (x1 - x0) * fraction, y0 + (y1 - y0) * fraction]);
      }
    }
    return path;
  }

  function run(waypoints, { targetVel, Lt, kp, initialYaw = 1.570 }) {
    const path = interpolate(waypoints);
    const goal = path[path.length - 1];
    const samples = [];
    let [x, y] = waypoints[0], yaw = initialYaw, v = 0, pind = 0;
    let desiredVel = targetVel, acceleration = 0, omega = 0;
    for (let i = 0; i < steps; i++) {
      if (i % 5 === 0) {
        const distanceToGoal = Math.hypot(x - goal[0], y - goal[1]);
        if (pind >= path.length - 2 && distanceToGoal < Lt) desiredVel = 0;
        acceleration = kp * (desiredVel - v);
        while (pind + 1 < path.length && Math.hypot(path[pind][0] - x, path[pind][1] - y) <= Lt) pind++;
        if (pind >= path.length - 2 && distanceToGoal < Lt) desiredVel = 0;
        const target = path[pind];
        const lateralOffset = -Math.sin(yaw) * (target[0] - x) + Math.cos(yaw) * (target[1] - y);
        omega = clamp(2 * lateralOffset / (Lt * Lt) * desiredVel, -1, 1);
      }
      v = clamp(v + clamp(acceleration, -0.5, 0.5) * dt, 0, 0.22);
      x += v * Math.cos(yaw) * dt;
      y += v * Math.sin(yaw) * dt;
      yaw = (yaw + omega * dt) % (2 * Math.PI);
      if (yaw < 0) yaw += 2 * Math.PI;
      if (yaw > Math.PI) yaw -= 2 * Math.PI;
      if (i % 5 === 0) samples.push([(i + 1) * dt, x, y, yaw, v, pind]);
    }
    return { lookahead_path: path, samples };
  }

  return { run };
})();
