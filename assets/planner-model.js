/* Browser counterpart of the four grid planners in main/Path_Planners. */
window.PlannerModel = (() => {
  const directions = [[1, 0], [1, 1], [0, 1], [-1, 1], [-1, 0], [-1, -1], [0, -1], [1, -1]];

  function plan(map, start, goal, algorithm, connectivity) {
    const grid = map.grid;
    const cols = grid[0].length;
    const key = ([row, col]) => row * cols + col;
    const position = index => [Math.floor(index / cols), index % cols];
    const valid = ([row, col]) => row >= 0 && row < grid.length && col >= 0 && col < cols && grid[row][col] === 0;
    if (!valid(start) || !valid(goal)) return null;

    const moves = connectivity === 4 ? directions.filter(([dr, dc]) => dr === 0 || dc === 0) : directions;
    const heuristic = ([row, col]) => connectivity === 4
      ? Math.abs(row - goal[0]) + Math.abs(col - goal[1])
      : Math.max(Math.abs(row - goal[0]), Math.abs(col - goal[1]));
    const parent = new Map([[key(start), null]]);
    const bestCost = new Map([[key(start), 0]]);
    const visited = [start];
    const frontier = [{ point: start, priority: 0, order: 0 }];
    let order = 1;
    let found = false;

    while (frontier.length) {
      let index = 0;
      if (algorithm !== 'bfs') {
        for (let i = 1; i < frontier.length; i++) {
          if (frontier[i].priority < frontier[index].priority ||
              (frontier[i].priority === frontier[index].priority && frontier[i].order < frontier[index].order)) index = i;
        }
      }
      const current = frontier.splice(index, 1)[0].point;
      if (key(current) === key(goal)) { found = true; break; }
      for (const [dr, dc] of moves) {
        const next = [current[0] + dr, current[1] + dc];
        if (!valid(next)) continue;
        const nextKey = key(next);
        let priority;
        if (algorithm === 'bfs' || algorithm === 'greedy') {
          if (parent.has(nextKey)) continue;
          priority = algorithm === 'bfs' ? order : heuristic(next);
        } else {
          const cost = bestCost.get(key(current)) + Math.hypot(dr, dc) * map.costmap[next[0]][next[1]];
          if (cost >= (bestCost.get(nextKey) ?? Infinity)) continue;
          bestCost.set(nextKey, cost);
          priority = cost + (algorithm === 'astar' ? heuristic(next) : 0);
        }
        if (!parent.has(nextKey)) visited.push(next);
        parent.set(nextKey, key(current));
        frontier.push({ point: next, priority, order: order++ });
      }
    }
    if (!found) return { path: [], visited, distance: 0 };
    const path = [];
    for (let at = key(goal); at !== null; at = parent.get(at)) path.push(position(at));
    path.reverse();
    const distance = path.slice(1).reduce((sum, point, index) =>
      sum + Math.hypot(point[0] - path[index][0], point[1] - path[index][1]), 0);
    return { path, visited, distance };
  }
  return { plan };
})();
