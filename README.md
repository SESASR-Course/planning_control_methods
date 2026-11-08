# Planning & Control Methods · interactive examples

A companion page to the Python examples on [`main`](https://github.com/SESASR-Course/planning_control_methods/tree/main): grid search planners, Dynamic Window Approach (DWA), and Pure Pursuit. The site uses HTML, CSS, and browser JavaScript. A Python build step runs the implementations from `main` and exports data for browser plots and animations; no Python runtime or server is needed on GitHub Pages.

Visit the [interactive Planning and Control webpage](https://sesasr-course.github.io/planning_control_methods/).

## Pages

- [Home](index.html)
- [Grid Search Planners](grid-search.html): choose from the maps in `main/2D_maps`, place start and goal on the map, compare BFS, Dijkstra, Greedy, and A* with four/eight connectivity, and replay the cell expansion followed by the route over the planner cost map.
- [DWA](dwa.html): place start and goal, add or remove obstacles, adjust scoring weights, prediction horizon, and velocity sample counts, then replay the navigation run and inspect the sampled trajectories.
- [Pure Pursuit](pure-pursuit.html): replay the controller simulation and inspect its lookahead target.