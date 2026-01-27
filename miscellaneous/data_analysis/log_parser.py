import xml.etree.ElementTree as ET
from collections import defaultdict

import matplotlib.pyplot as plt
import numpy as np

LOG_FILE = "logs/hermes_simulator.log"
NAME = "Simulator"

def parse_log(path):
    tree = ET.parse(path)
    root = tree.getroot()

    records = []
    for r in root.findall("record"):
        millis = int(r.findtext("millis"))
        nanos = int(r.findtext("nanos", "0"))
        time_ms = millis + nanos * 1e-6

        records.append({
            "time": time_ms,
            "method": r.findtext("method", ""),
            "message": r.findtext("message", "")
        })

    records.sort(key=lambda x: x["time"])
    return records


def extract_cycles(records):
    cycles = []
    current = None

    for rec in records:
        if "Reasoning Cycle" in rec["message"] and rec["method"] == "run":
            if current:
                cycles.append(current)
            current = {
                "start": rec["time"],
                "events": []
            }

        if current:
            current["events"].append(rec)

        if current and "Reasoning Time" in rec["message"]:
            current["end"] = rec["time"]

    if current:
        cycles.append(current)

    return cycles


def compute_timings(cycles):
    cycle_durations = []
    stage_durations = defaultdict(list)

    for c in cycles:
        if "end" not in c:
            continue

        cycle_durations.append((c["end"] - c["start"]) / 1000.0)

        events = c["events"]
        for i in range(len(events) - 1):
            t0 = events[i]["time"]
            t1 = events[i + 1]["time"]
            stage = events[i]["method"]

            dt = (t1 - t0) / 1000.0
            stage_durations[stage].append(dt)

    return cycle_durations, stage_durations

def combined_violin_box(cycle_times, stage_times, out_file):
    labels = ["Total Reasoning Time"]
    data = [cycle_times]

    for stage, times in stage_times.items():
        if stage == "loadImplementation":
            continue
        labels.append(stage)
        data.append(times)

    fig, ax = plt.subplots(figsize=(10, 5))

    ax.boxplot(
        data,
        vert=True,
        whis=1.5,
        showfliers=True,
        widths=0.6
    )

    ax.set_yscale("log")

    for i, values in enumerate(data, start=1):
        values_ms = np.array(values) * 1000.0

        mean = np.mean(values_ms)
        std  = np.std(values_ms, ddof=1) if len(values_ms) > 1 else 0.0

        label = f"{mean:.3f} ± {std:.3f} ms"

        ax.text(
            i + 0.4,          
            max(values),
            label,
            fontsize=9,
            ha='right',
            va='bottom',
            color='black',
            zorder=10
        )


    ax.set_xticks(range(1, len(labels) + 1))
    ax.set_xticklabels(labels, rotation=30, ha="right")


    ax.set_ylabel("Time (s)")
    ax.set_title("Total and Per Stage Reasoning Time Distributions for " + NAME)

    plt.tight_layout()
    file_name = "plots/" + out_file + "_time_distribution.png"
    plt.savefig(file_name, dpi=300)
    plt.close()

    print(f"Saved plot to {file_name}")

def report_stats(cycle_times, stage_times):
    print("\nTiming statistics (seconds)")
    print("-" * 40)

    def stats(name, values):
        mean = np.mean(values)
        std = np.std(values, ddof=1)  # sample std
        print(f"{name:15s}  mean = {mean:.6f}  std = {std:.6f}")

    stats("Total Reasoning Time", cycle_times)

    for stage, times in stage_times.items():
        stats(stage, times)

records = parse_log(LOG_FILE)
cycles = extract_cycles(records)

cycle_times, stage_times = compute_timings(cycles)

combined_violin_box(
    cycle_times,
    stage_times,
    NAME
)

print(f"Parsed {len(cycle_times)} reasoning cycles")
print("Stages found:", list(stage_times.keys()))
report_stats(cycle_times, stage_times)
