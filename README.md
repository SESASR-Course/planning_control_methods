# Planning & Control Methods · interactive examples

A static companion to the Python examples on [`main`](https://github.com/SESASR-Course/planning_control_methods/tree/main): grid search planners, Dynamic Window Approach (DWA), and Pure Pursuit. The site uses HTML, CSS, and browser JavaScript. A Python build step runs the implementations from `main` and exports data for browser plots and animations; no Python runtime or server is needed on GitHub Pages.

## Pages

- [Home](index.html)
- [Grid Search Planners](grid-search.html): BFS, Dijkstra, Greedy, and A* with four/eight connectivity.
- [DWA](dwa.html): replay a generated navigation trajectory and inspect candidate trajectories.
- [Pure Pursuit](pure-pursuit.html): replay the controller simulation and inspect its lookahead target.

## Build locally

From a checkout of `gh-pages`, with a checkout of `main` at `../planning_control_methods-main`:

```bash
python -m pip install numpy
python scripts/build_examples.py --source ../planning_control_methods-main --output data
python -m http.server 8000
```

Open `http://localhost:8000`. The generated JSON in `data/` is committed as an initial preview. The workflow rebuilds it from the latest `main` on each push to either branch, then publishes the site with GitHub Pages Actions. In repository settings, set **Pages → Build and deployment → Source** to **GitHub Actions**.
