import math
import time
import csv
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt


# ----------------------------
# Config (single source of truth)
# ----------------------------
DT = 0.05                 # [s] sim timestep
V_FWD = 0.25              # [m/s] forward speed
W_TURN = 1.8              # [rad/s] turn rate
STOP_D = 0.30             # [m] start turning if closer than this
GO_D = 0.55               # [m] resume forward if farther than this (hysteresis)
ROBOT_RADIUS = 0.06       # [m] for drawing only
OBSTACLE_RADIUS = 0.07    # [m] for drawing only
MAX_STEPS = 10_000        # safety cap


@dataclass
class Robot:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # heading [rad], 0 = +x

    def step(self, v: float, w: float, dt: float) -> None:
        # Minimal differential-drive kinematics (unicycle model)
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def distance_to(robot: Robot, ox: float, oy: float) -> float:
    return math.hypot(ox - robot.x, oy - robot.y)


def nearest_obstacle(robot: Robot, obstacles: list[tuple[float, float]]) -> tuple[float, float, float]:
    # returns (ox, oy, d_min)
    ox, oy = min(obstacles, key=lambda p: distance_to(robot, p[0], p[1]))
    dmin = distance_to(robot, ox, oy)
    return ox, oy, dmin


def choose_turn_direction(robot: Robot, ox: float, oy: float) -> float:
    """
    Minimal but smart: turn away from obstacle using bearing sign.
    If obstacle is to the left (positive relative angle), turn right (negative w), and vice versa.
    """
    bearing = math.atan2(oy - robot.y, ox - robot.x)
    rel = wrap_pi(bearing - robot.theta)
    return -W_TURN if rel > 0 else W_TURN


def decision_fsm(state: str, dmin: float, turn_w: float) -> tuple[str, float, float, str]:
    """
    Two-state FSM with hysteresis:
      - FORWARD if clear
      - TURN if too close
    """
    if state == "FORWARD":
        if dmin < STOP_D:
            return "TURN", 0.0, turn_w, "too_close"
        return "FORWARD", V_FWD, 0.0, "clear"
    else:  # TURN
        if dmin > GO_D:
            return "FORWARD", V_FWD, 0.0, "cleared"
        return "TURN", 0.0, turn_w, "turning"


def ensure_outdir() -> Path:
    out = Path("out")
    out.mkdir(exist_ok=True)
    return out


def run():
    # Scenario: two obstacles
    obstacles = [(1.0, 0.0), (1.2, 0.35)]

    r = Robot(x=0.0, y=0.0, theta=0.0)
    state = "FORWARD"

    # Logging
    out = ensure_outdir()
    csv_path = out / "telemetry.csv"

    trail_x, trail_y = [], []

    # Live plot setup
    plt.ion()
    fig, ax = plt.subplots()

    t0 = time.time()
    sim_time = 0.0

    with csv_path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "x", "y", "theta", "dmin", "state", "reason", "v", "omega", "nearest_ox", "nearest_oy"])

        for _step in range(MAX_STEPS):
            # Nearest obstacle + turn direction suggestion
            nox, noy, dmin = nearest_obstacle(r, obstacles)
            turn_w = choose_turn_direction(r, nox, noy)

            # FSM decision
            state, v, omega, reason = decision_fsm(state, dmin, turn_w)

            # Step dynamics
            r.step(v, omega, DT)
            sim_time += DT

            # Save trail
            trail_x.append(r.x)
            trail_y.append(r.y)

            # Log row
            w.writerow([sim_time, r.x, r.y, r.theta, dmin, state, reason, v, omega, nox, noy])

            # Draw
            ax.clear()
            ax.set_aspect("equal", adjustable="box")
            ax.set_xlim(-0.5, 2.0)
            ax.set_ylim(-1.0, 1.0)
            ax.grid(True, alpha=0.2)

            # Obstacles
            for (ox, oy) in obstacles:
                ax.plot(ox, oy, "rs")
                ax.add_patch(plt.Circle((ox, oy), OBSTACLE_RADIUS, fill=False))

            # Trail
            ax.plot(trail_x, trail_y, linewidth=1)

            # Robot body + heading
            ax.plot(r.x, r.y, "bo")
            ax.add_patch(plt.Circle((r.x, r.y), ROBOT_RADIUS, fill=False))
            ax.arrow(
                r.x, r.y,
                0.18 * math.cos(r.theta), 0.18 * math.sin(r.theta),
                head_width=0.05,
                length_includes_head=True
            )

            # HUD text (simple + readable)
            ax.set_title(f"state={state}  dmin={dmin:.2f} m  v={v:.2f} m/s  w={omega:.2f} rad/s  reason={reason}")

            plt.pause(DT)

            # Keep wall-clock loosely tied to sim (optional)
            # This prevents the sim from running too fast on powerful machines.
            target = t0 + sim_time
            now = time.time()
            if now < target:
                time.sleep(target - now)

    return csv_path


def make_plots(csv_path: Path):
    # Minimal plotting from saved CSV (no pandas dependency)
    t, d, v, w, state = [], [], [], [], []
    with csv_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row["t"]))
            d.append(float(row["dmin"]))
            v.append(float(row["v"]))
            w.append(float(row["omega"]))
            state.append(row["state"])

    out = csv_path.parent

    # Distance vs time
    plt.ioff()
    plt.figure()
    plt.plot(t, d)
    plt.xlabel("t [s]")
    plt.ylabel("dmin [m]")
    plt.title("Nearest obstacle distance vs time")
    plt.tight_layout()
    plt.savefig(out / "distance_vs_time.png", dpi=200)

    # Control commands vs time
    plt.figure()
    plt.plot(t, v, label="v [m/s]")
    plt.plot(t, w, label="omega [rad/s]")
    plt.xlabel("t [s]")
    plt.title("Control commands vs time")
    plt.legend()
    plt.tight_layout()
    plt.savefig(out / "controls_vs_time.png", dpi=200)


if __name__ == "__main__":
    try:
        csv_path = run()
    except KeyboardInterrupt:
        # If you Ctrl+C, you’ll still want plots from whatever is logged so far.
        csv_path = Path("out/telemetry.csv")
        print("\nInterrupted. Generating plots from existing telemetry if available...")

    if csv_path.exists():
        make_plots(csv_path)
        print(f"Saved telemetry: {csv_path}")
        print("Saved plots: out/distance_vs_time.png, out/controls_vs_time.png")
    else:
        print("No telemetry CSV found. Run the sim again.")