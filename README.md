# Pure Pursuit Algorithm

Implementation of the Pure Pursuit algorithm for autonomous vehicle path following. Given a predefined path, the vehicle continuously looks ahead by a fixed distance, selects a goal point on the path, and computes the steering command needed to reach it — producing smooth, stable tracking through curves and straights.

### How It Works

1. **Nearest point** — find the closest point on the path to the vehicle's current position
2. **Lookahead** — scan forward along the path to select a goal point at the lookahead distance
3. **Transform** — convert the goal point from world frame to robot frame
4. **Curvature** — compute the required turning curvature: `κ = 2·y_r / L²` where `y_r` is the lateral offset and `L` is the lookahead distance
5. **Update** — apply the resulting angular velocity and integrate the vehicle state

The algorithm outputs cross-track error (CTE) and orientation error plots on completion.

### Installation and Running

```bash
git clone https://github.com/salilnpatil/Pure-pursuit-algorithm.git
cd Pure-pursuit-algorithm/
pip install matplotlib
python3 pure_pursuit_algorithm.py
```

Press `Esc` during the simulation to exit early.

### Demo

<p align="center">
<img src="https://github.com/salilnpatil/Pure-pursuit-algorithm/blob/main/assets/pure-pursuit-algo.gif" width="400" height="250">
</p>

### References

- [Implementation of the Pure Pursuit Path Tracking Algorithm — R. Craig Coulter, CMU RI (1992)](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf)
- [Pure Pursuit blog by vinesmsuic](https://vinesmsuic.github.io/robotics-purepersuit/index.html)
