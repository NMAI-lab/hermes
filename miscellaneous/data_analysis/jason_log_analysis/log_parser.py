import xml.etree.ElementTree as ET
from collections import defaultdict

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

SOURCES = {
    "Simulator": "logs/hermes_simulator.log",
    "Robot": "logs/hermes_robot.log",
}

COLORS = {
    "Simulator": "#4C9BE8",
    "Robot": "#E8714C",
}

OUT_FILE = "plots/jason_time_distribution.png"

def parse_log(path):
    tree = ET.parse(path)
    root = tree.getroot()

    records = []
    for r in root.findall("record"):
        millis = int(r.findtext("millis"))
        nanos  = int(r.findtext("nanos", "0"))
        records.append({
            "time":    millis + nanos * 1e-6,
            "method":  r.findtext("method", ""),
            "message": r.findtext("message", ""),
        })

    records.sort(key=lambda x: x["time"])
    return records

def extract_cycles(records):
    cycles, current = [], None
    for rec in records:
        if "Reasoning Cycle" in rec["message"] and rec["method"] == "run":
            if current:
                cycles.append(current)
            current = {"start": rec["time"], "events": []}
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
            stage = events[i]["method"]
            dt    = (events[i + 1]["time"] - events[i]["time"]) / 1000.0
            stage_durations[stage].append(dt)

    return cycle_durations, stage_durations

def filter_iqr(values, whis=1.5):
    arr = np.array(values)
    q1, q3 = np.percentile(arr, 25), np.percentile(arr, 75)
    iqr = q3 - q1
    return arr[(arr >= q1 - whis * iqr) & (arr <= q3 + whis * iqr)]

def load_all():
    result = {}
    for name, path in SOURCES.items():
        records = parse_log(path)
        cycles  = extract_cycles(records)
        ct, st  = compute_timings(cycles)
        ct_ms   = np.array(ct) * 1000
        st_ms   = {k: np.array(v) * 1000 for k, v in st.items()
                   if k != "loadImplementation"}
        result[name] = (ct_ms, st_ms)
        print(f"[{name}] {len(ct)} cycles | stages: {list(st_ms.keys())}")
    return result

def build_label_order(all_data):
    labels = ["Total Time"]
    seen   = set(labels)
    for _, (_, st) in all_data.items():
        for k in st:
            if k not in seen:
                labels.append(k)
                seen.add(k)
    return labels

def make_comparison_plot(all_data):
    labels  = build_label_order(all_data)
    n       = len(labels)
    names   = list(all_data.keys())
    k       = len(names)

    box_w   = 0.24
    offsets = np.linspace(-(k - 1) * box_w / 2, (k - 1) * box_w / 2, k)

    fig, ax = plt.subplots(figsize=(max(12, n * 2.4), 5))

    group_annotations = defaultdict(list)

    for ni, name in enumerate(names):
        ct_ms, st_ms = all_data[name]
        color        = COLORS[name]

        for li, label in enumerate(labels):
            x_center = li + 1
            x_pos    = x_center + offsets[ni]

            raw = ct_ms if label == "Total Time" else st_ms.get(label, np.array([]))
            if len(raw) == 0:
                continue

            raw_ms   = np.array(raw)
            filtered = filter_iqr(raw_ms)

            # Outliers
            outliers = raw_ms[~np.isin(raw_ms, filtered)]
            if len(outliers):
                ax.scatter(
                    [x_pos] * len(outliers), outliers,
                    color="gray", alpha=0.25, s=14, zorder=1,
                )

            ax.boxplot(
                [filtered],
                positions=[x_pos],
                widths=box_w * 0.85,
                vert=True,
                whis=1.5,
                showfliers=False,
                patch_artist=True,
                boxprops=dict(facecolor=color, alpha=0.55, linewidth=1.4),
                medianprops=dict(color="white", linewidth=2.2),
                whiskerprops=dict(color=color, linewidth=1.4),
                capprops=dict(color=color, linewidth=1.6),
            )

            mean = np.mean(filtered)
            std  = np.std(filtered, ddof=1) if len(filtered) > 1 else 0.0
            group_annotations[li].append((mean, color, f"{mean:.3f}±{std:.3f} ms"))

    # Placing labels for each graph
    for li, annotations in group_annotations.items():
        for idx, (mean, color, text) in enumerate(annotations):
            x_pos = (li + 1) + offsets[idx]
            ct_ms, st_ms = all_data[names[idx]]
            raw = ct_ms if labels[li] == "Total Time" else st_ms.get(labels[li], np.array([]))
            if len(raw) == 0:
                continue
            filtered = filter_iqr(np.array(raw))
            top = np.max(filtered)
            ax.text(
                x_pos - 0.08, top * 1.05,
                text,
                fontsize=12,
                ha="center",
                va="bottom",
                color=color,
                fontweight="bold",
                rotation=90,
                zorder=10,
            )

    ax.set_yscale("log")
    ax.set_xticks(range(1, n + 1))
    ax.set_xticklabels(labels, rotation=25, ha="right", fontsize=16)
    ax.tick_params(axis="y", labelsize=15)
    ax.set_ylabel("Time (ms)", fontsize=16)
    ax.set_title(
        "Jason's Reasoning Time Distributions - Simulator vs Robot (per stage)",
        fontsize=17, pad=10,
    )

    patches = [
        mpatches.Patch(facecolor=COLORS[nm], alpha=0.75, label=nm)
        for nm in names
    ]
    ax.legend(handles=patches, fontsize=12, loc="upper right")

    for xi in range(1, n):
        ax.axvline(xi + 0.5, color="lightgrey", linewidth=0.8, zorder=0)


    plt.tight_layout(pad=0.1)
    plt.savefig(OUT_FILE, dpi=300, bbox_inches="tight")
    plt.close()
    print(f"\nSaved plot at: {OUT_FILE}")


all_data = load_all()
make_comparison_plot(all_data)