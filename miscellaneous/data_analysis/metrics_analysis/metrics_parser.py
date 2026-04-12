import os
import json
from collections import defaultdict

import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from statsmodels.stats.proportion import proportion_confint

DATA_DIR = "./data"

TRIP_TYPES = ["right_turn", "left_turn", "pass_through", "u_turn"]
TRIP_OUTCOMES = ["Success", "Success with Recoverable Failure", "Failure"]

EXPECTED_NAVIGATION_INSTRUCTIONS = {
    "right_turn": {"START", "WALL_FOLLOW", "DOCK"},
    "left_turn": {"START", "L_TURN", "DOCK"},
    "pass_through": {"START", "FORWARD", "DOCK"},
    "u_turn": {"START", "U_TURN", "DOCK"}
}

COLORS = {
    TRIP_OUTCOMES[0]: "#4CAF50",
    TRIP_OUTCOMES[1]: "#FF9800",
    TRIP_OUTCOMES[2]: "#F44336"
}

mpl.rcParams.update({
    "font.family": "DejaVu Sans",   # consistent default everywhere
    "axes.titlesize": 12,
    "axes.labelsize": 11,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "legend.fontsize": 9,
    "figure.titlesize": 13
})

# LOAD DATA

def load_trips(data_dir):
    trips = {}

    for filename in os.listdir(data_dir):
        if filename.endswith(".json"):
            path = os.path.join(data_dir, filename)
            trip_type = "_".join(filename.split("_")[:2])

            with open(path, "r") as f:
                trips[trip_type] = json.load(f)

    return trips

# COMPUTE METRICS

def assign_trip_outcome(trip, trip_type):
    if not trip["docked"]:
        return TRIP_OUTCOMES[2]

    if set(trip["navigation_instructions"]) == EXPECTED_NAVIGATION_INSTRUCTIONS[trip_type]:
        return TRIP_OUTCOMES[0]

    return TRIP_OUTCOMES[1]

def compute_outcome_counts(trips):
    counts = defaultdict(lambda: defaultdict(int))

    for trip_type in TRIP_TYPES:
        for trip in trips[trip_type]:
            outcome = assign_trip_outcome(trip, trip_type)
            counts[trip_type][outcome] += 1

    return counts

def compute_total_counts(counts):
    total = defaultdict(int)

    for t in TRIP_TYPES:
        for o in TRIP_OUTCOMES:
            total[o] += counts[t][o]

    return total

def wilson_ci(count, n):
    return proportion_confint(count, n, method="wilson")

# PLOTS

def plot_trip_outcomes(counts, total_counts):
    trip_types_all = TRIP_TYPES + ["TOTAL"]
    x = np.arange(len(trip_types_all))

    width = 0.22

    fig, ax = plt.subplots(figsize=(8.2, 4.6), constrained_layout=False)

    plt.subplots_adjust(
        left=0.08,
        right=0.98,
        top=0.93,
        bottom=0.22
    )

    for i, outcome in enumerate(TRIP_OUTCOMES):

        proportions, lower_err, upper_err = [], [], []

        for trip_type in trip_types_all:

            if trip_type == "TOTAL":
                count = total_counts[outcome]
                n = sum(total_counts.values())
            else:
                count = counts[trip_type][outcome]
                n = sum(counts[trip_type].values())

            p = count / n
            low, high = wilson_ci(count, n)

            proportions.append(p)
            lower_err.append(p - low)
            upper_err.append(high - p)

        offset = (i - 1) * width

        ax.bar(
            x + offset,
            proportions,
            width,
            color=COLORS[outcome],
            edgecolor="black",
            linewidth=0.8,
            label=outcome
        )

        ax.errorbar(
            x + offset,
            proportions,
            yerr=[lower_err, upper_err],
            fmt="none",
            ecolor="black",
            capsize=3,
            linewidth=1.0,
            zorder=10
        )

    ax.set_xticks(x)
    ax.set_xticklabels(
        ["Right Turn", "Left Turn", "Pass Through", "U-Turn", "TOTAL"]
    )

    ax.set_ylim(0, 1)
    ax.set_ylabel("Proportion")

    ax.set_title(
        "Trip Outcome by Type (95% Wilson Confidence Intervals) - Simulator",
        fontsize=13,
        pad=10
    )

    handles, labels = ax.get_legend_handles_labels()

    ci_proxy = plt.Line2D(
        [0], [0],
        color="black",
        linewidth=1.5,
        label="95% Wilson CI"
    )

    handles.append(ci_proxy)

    ax.legend(
        handles=handles,
        loc="upper center",
        bbox_to_anchor=(0.5, -0.18),
        ncol=4,
        frameon=True,
        framealpha=0.9,
        fontsize=9,
        columnspacing=1.2,
        handletextpad=0.6
    )

    plt.savefig(
        "plots/trip_outcomes_simulator.png",
        dpi=300,
        pad_inches=0.05
    )

    plt.close()

def plot_behavioural_metrics(trips):

    hits = []
    wall_dist = []

    for trip_type in TRIP_TYPES:
        for trip in trips[trip_type]:

            if "bumps" in trip and isinstance(trip["bumps"], (int, float)):
                hits.append(trip["bumps"])

            if "wall_distances" in trip:
                wd = trip["wall_distances"]
                if isinstance(wd, list) and len(wd) > 0:
                    wall_dist.append(np.mean(wd))

    fig, ax = plt.subplots(
        1, 2,
        figsize=(5.8, 3.6),
        constrained_layout=False
    )

    fig.subplots_adjust(
        left=0.10,
        right=0.98,
        bottom=0.14,
        top=0.82,
        wspace=0.25
    )

    ax[0].boxplot(hits, patch_artist=True, showfliers=True)
    ax[0].set_title("Number of Bumper Hits", pad=6)
    ax[0].set_ylabel("Count")
    ax[0].set_xticks([])

    ax[1].boxplot(wall_dist, patch_artist=True, showfliers=True)
    ax[1].set_title("Wall Follow Distance", pad=6)
    ax[1].set_ylabel("Distance")
    ax[1].set_xticks([])

    fig.suptitle("Behavioural Metrics (All Trips) - Simulator", fontsize=13, y=0.95)

    plt.savefig(
        "plots/behavioural_metrics_simulator.png",
        dpi=300,
        bbox_inches="tight",
        pad_inches=0.05
    )
    plt.close()

def main():
    trips = load_trips(DATA_DIR)

    counts = compute_outcome_counts(trips)
    total_counts = compute_total_counts(counts)

    plot_trip_outcomes(counts, total_counts)
    plot_behavioural_metrics(trips)

if __name__ == "__main__":
    main()