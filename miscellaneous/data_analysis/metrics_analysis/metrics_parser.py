import os
import json
from collections import defaultdict
import re

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from statsmodels.stats.proportion import proportion_confint

# Patterns

TRIP_TYPES = ["right_turn", "left_turn", "pass_through", "u_turn", "docking", "collision_handling"]
TRIP_OUTCOMES = ["Success", "Failure", "Invalid"]

FORWARD_ACT = "cmd_vel(0.4,0,0,0,0,0)"
L_TURN_ACT = "cmd_vel(0.2,0,0,0,0,1.1)"
U_TURN_ACT = "cmd_vel(0.2,0,0,0,0,2.1)"
SMALL_L_TURN_ACT = "cmd_vel(0.2,0,0,0,0,0.5235987755982988)"
BACKWARDS_ACT = "cmd_vel(-0.4,0,0,0,0,0)"
DOCK_ACT = "dock"

R_TURN_NAV = "WALL_FOLLOW"
PASS_THROUGH_NAV = "FORWARD"
L_TURN_NAV = "L_TURN"
U_TURN_NAV = "U_TURN"
DOCK_NAV = "DOCK"

# Constants

DATA_DIR = "./data"
COLORS = {
    TRIP_OUTCOMES[0]: "#4CAF50",
    TRIP_OUTCOMES[1]: "#F44336",
    TRIP_OUTCOMES[2]: "#FF9800"
}

# Helpers

def contains_sublist(lst, sublst):
    n, m = len(lst), len(sublst)
    return any(
        all(is_cmd_vel_valid(lst[i+j], sublst[j]) for j in range(m))
        for i in range(n - m + 1)
    )

def is_cmd_vel_valid(action, pattern):
    m = re.match(r"cmd_vel\(([^,]+),.*,([^)]+)\)", action)
    p = re.match(r"cmd_vel\(([^,]+),.*,([^)]+)\)", pattern)
    if not m:
        return False
    linear_x, angular_z = float(m.group(1)), float(m.group(2))
    return abs(linear_x - float(p.group(1))) < 0.05 and abs(angular_z - float(p.group(2))) < 0.2

# LOAD DATA

def load_trips(data_dir):
    trips = defaultdict(dict)
    for filename in os.listdir(data_dir):
        if filename.endswith(".json"):
            path = os.path.join(data_dir, filename)
            parts = filename.replace(".json", "").split("_")
            device_type = parts[-1]
            trip_type = "_".join(parts[:-1])

            with open(path, "r") as f:
                data = json.load(f)
                trips[device_type][trip_type] = data

    return trips

# COMPUTE METRICS

def assign_trip_outcome(trip, trip_type):
    actions = trip["actions"]
    navigations = trip["navigation_instructions"]
    if len(actions) < 10:
        return TRIP_OUTCOMES[2]
    
    if trip_type == "right_turn":
        if any(a in actions for a in [FORWARD_ACT, L_TURN_ACT, U_TURN_ACT]):
            return TRIP_OUTCOMES[1]
        if R_TURN_NAV not in navigations:
            return TRIP_OUTCOMES[1]
        return TRIP_OUTCOMES[0]
    elif trip_type == "pass_through":
        if any(a in actions for a in [L_TURN_ACT, U_TURN_ACT]):
            return TRIP_OUTCOMES[1]
        if PASS_THROUGH_NAV not in navigations:
            return TRIP_OUTCOMES[1]
        if  contains_sublist(actions, [FORWARD_ACT, FORWARD_ACT, FORWARD_ACT, FORWARD_ACT, FORWARD_ACT, 
                                       FORWARD_ACT, FORWARD_ACT, FORWARD_ACT, FORWARD_ACT, FORWARD_ACT]):
            return TRIP_OUTCOMES[0]
        return TRIP_OUTCOMES[1]
    elif trip_type == "left_turn":
        if any(a in actions for a in [U_TURN_ACT]):
            return TRIP_OUTCOMES[1]
        if L_TURN_NAV not in navigations:
            return TRIP_OUTCOMES[1]
        if  contains_sublist(actions, [L_TURN_ACT, L_TURN_ACT, L_TURN_ACT, L_TURN_ACT, L_TURN_ACT, 
                                       FORWARD_ACT, FORWARD_ACT, FORWARD_ACT, FORWARD_ACT, FORWARD_ACT]):
            return TRIP_OUTCOMES[0]
        return TRIP_OUTCOMES[1]
    elif trip_type == "u_turn":
        if any(a in actions for a in [L_TURN_ACT]):
            return TRIP_OUTCOMES[1]
        if U_TURN_NAV not in navigations:
            return TRIP_OUTCOMES[1]
        if contains_sublist(actions, [U_TURN_ACT, U_TURN_ACT, U_TURN_ACT, U_TURN_ACT, U_TURN_ACT, 
                                      FORWARD_ACT, FORWARD_ACT, FORWARD_ACT, FORWARD_ACT, FORWARD_ACT]):
            return TRIP_OUTCOMES[0]
        return TRIP_OUTCOMES[1]
    elif trip_type == "docking":
        if any(a in actions for a in [FORWARD_ACT, L_TURN_ACT, U_TURN_ACT]):
            return TRIP_OUTCOMES[1]
        if DOCK_NAV not in navigations:
            return TRIP_OUTCOMES[1]
        if DOCK_ACT in actions and trip["docked"]:
            return TRIP_OUTCOMES[0]
        return TRIP_OUTCOMES[1]
    elif trip_type == "collision_handling":
        if any(a in actions for a in [L_TURN_ACT, U_TURN_ACT, FORWARD_ACT]):
            return TRIP_OUTCOMES[1]
        if contains_sublist(actions, [BACKWARDS_ACT, BACKWARDS_ACT,  BACKWARDS_ACT, BACKWARDS_ACT, BACKWARDS_ACT, 
                                      SMALL_L_TURN_ACT, SMALL_L_TURN_ACT, SMALL_L_TURN_ACT, SMALL_L_TURN_ACT, SMALL_L_TURN_ACT]):
            return TRIP_OUTCOMES[0]
        return TRIP_OUTCOMES[1]
    return TRIP_OUTCOMES[2]

def compute_outcome_counts(trips):
    counts = defaultdict(lambda: defaultdict(int))

    for trip_type in TRIP_TYPES:
        if trip_type not in trips:
            continue
        for trip in trips[trip_type]:
            outcome = assign_trip_outcome(trip, trip_type)
            counts[trip_type][outcome] += 1
    return counts

def get_wall_distances(trips):
    distances = []
    for trip_type in TRIP_TYPES:
        if trip_type not in trips:
            continue
        for trip in trips[trip_type]:
            outcome = assign_trip_outcome(trip, trip_type)
            if outcome == TRIP_OUTCOMES[2]:
                continue
            for entry in trip["wall_following"]:
                if not entry["intersection"]:
                    distances.append(entry["right_wall_distance"])
    return distances

# PLOTS

def plot_behaviours(outcomes, device_type):
    valid_outcome_types = [o for o in TRIP_OUTCOMES if o != TRIP_OUTCOMES[2]]
    valid_outcomes = {
        t: {o: inner.get(o, 0) for o in valid_outcome_types}
        for t, inner in outcomes.items()
    }

    x = np.arange(len(TRIP_TYPES))
    width = 0.5
    fig, ax = plt.subplots(figsize=(8, 4))

    success_vals, failure_vals = [], []
    ci_low, ci_high = [], []

    for t in TRIP_TYPES:
        n_success = valid_outcomes[t][TRIP_OUTCOMES[0]]
        n_total   = sum(valid_outcomes[t].values())
        pct_s = (n_success / n_total * 100) if n_total > 0 else 0
        pct_f = 100 - pct_s

        lo, hi = proportion_confint(n_success, n_total, method="wilson")
        success_vals.append(pct_s)
        failure_vals.append(pct_f)
        ci_low.append(max(0, (n_success / n_total - lo) * 100))
        ci_high.append(max(0, (hi - n_success / n_total) * 100))

    _ = ax.bar(x, success_vals, width, label=TRIP_OUTCOMES[0], color=COLORS[TRIP_OUTCOMES[0]])
    _ = ax.bar(x, failure_vals, width, bottom=success_vals, label=TRIP_OUTCOMES[1], color=COLORS[TRIP_OUTCOMES[1]])

    ax.errorbar(
        x, success_vals,
        yerr=[ci_low, ci_high],
        fmt="none", ecolor="black", elinewidth=1.2, capsize=4
    )

    for xi, (sv, fv) in enumerate(zip(success_vals, failure_vals)):
        ax.text(xi, sv / 2, f"{sv:.1f}%", ha="center", va="center", fontsize=8, color="white", fontweight="bold")
        if fv > 0:
            ax.text(xi, sv + fv / 2, f"{fv:.1f}%", ha="center", va="center", fontsize=8, color="white", fontweight="bold")

    ax.set_xticks(x)
    ax.set_xticklabels([t.replace("_", " ").title() for t in TRIP_TYPES], rotation=15, ha="right")
    ax.set_ylabel("Success Rate (%)")
    ax.set_ylim(0, 115)
    ax.set_title(f"Success Rate of Individual Behaviours - {device_type.capitalize()}")

    ci_handle = Line2D([0], [0], color="black", linewidth=1.2, label="95% Wilson CI")
    handles, _ = ax.get_legend_handles_labels()
    ax.legend(
        handles=handles + [ci_handle],
        loc="upper center",
        bbox_to_anchor=(0.5, -0.18),
        ncol=3,
        frameon=True,
        framealpha=0.9,
        fontsize=9,
        columnspacing=1.2,
        handletextpad=0.6
    )

    ax.grid(axis="y", linestyle="--", alpha=0.4)
    plt.tight_layout(pad=0.5)
    plt.savefig(f"plots/behaviour_plot_{device_type}.png", dpi=150, bbox_inches="tight")
    plt.close()

def plot_wall_distances(distances, device_type):
    fig, ax = plt.subplots(figsize=(4, 4))
    q1, q3 = np.percentile(distances, [25, 75])
    iqr = q3 - q1
    filtered = [d for d in distances if q1 - 1.5 * iqr <= d <= q3 + 1.5 * iqr]

    bp = ax.boxplot(filtered, patch_artist=True,
                    medianprops={"color": "black", "linewidth": 1.5},
                    showfliers=False)

    for patch in bp["boxes"]:
        patch.set_facecolor("#90CAF9")

    ax.text(0.98, 0.02, f"mean={np.mean(filtered):.3f}m\nstd={np.std(filtered):.3f}m",
            ha="right", va="bottom", fontsize=10, color="black",
            transform=ax.transAxes)

    ax.set_xticks([])
    ax.set_ylabel("Distance (m)")
    ax.set_title(f"Right Wall Distance (m) - {device_type.capitalize()}")
    ax.grid(axis="y", linestyle="--", alpha=0.4)

    plt.tight_layout(pad=0.5)
    plt.savefig(f"plots/wall_distances_{device_type}.png", dpi=150, bbox_inches="tight")
    plt.close()

def main():
    all_trips = load_trips(DATA_DIR) 

    for device_type, trips in all_trips.items():
        outcomes = compute_outcome_counts(trips)
        distances = get_wall_distances(trips)

        plot_behaviours(outcomes=outcomes, device_type=device_type)
        plot_wall_distances(distances=distances, device_type=device_type)

if __name__ == "__main__":
    main()