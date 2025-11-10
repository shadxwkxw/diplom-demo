#!/usr/bin/env python3
# analyze_kpi.py: сравнение KPI между baseline и opt сценариями
# Использует tripinfos.xml, summary.xml, edgeData.xml, laneData.xml из out/<run>/

import os
import sys
import statistics as stats
import xml.etree.ElementTree as ET
from typing import Dict, Tuple, List

Run = Tuple[str, str]  # (name, path)


def pct(values: List[float], p: float) -> float:
    if not values:
        return 0.0
    values = sorted(values)
    k = (len(values) - 1) * p
    f = int(k)
    c = min(f + 1, len(values) - 1)
    if f == c:
        return values[int(k)]
    d0 = values[f] * (c - k)
    d1 = values[c] * (k - f)
    return d0 + d1


def parse_tripinfos(path: str) -> Dict[str, float]:
    m = {
        "departed": 0,
        "arrived": 0,
        "avg_travel": 0.0,
        "p95_travel": 0.0,
        "avg_wait": 0.0,
    }
    if not os.path.exists(path):
        return m
    durations, waits = [], []
    arrived = 0
    try:
        for evt in ET.iterparse(path, events=("start",)):
            tag = evt[1].tag
            if tag.endswith("tripinfo"):
                arrived += 1
                dur = float(evt[1].get("duration", "0") or 0)
                wt = float(evt[1].get("waitingTime", "0") or 0)
                durations.append(dur)
                waits.append(wt)
    except Exception:
        pass
    m["arrived"] = arrived
    m["avg_travel"] = sum(durations) / len(durations) if durations else 0.0
    m["p95_travel"] = pct(durations, 0.95)
    m["avg_wait"] = sum(waits) / len(waits) if waits else 0.0
    # departed можно получить из summary.xml; здесь приблизим как arrived
    m["departed"] = arrived
    return m


def parse_summary(path: str) -> Dict[str, float]:
    m = {
        "mean_speed": 0.0,
        "total_waiting_time": 0.0,
        "stopped_veh_avg": 0.0,
    }
    if not os.path.exists(path):
        return m
    speeds, stopped, waiting = [], [], []
    try:
        for evt in ET.iterparse(path, events=("start",)):
            tag = evt[1].tag
            if tag.endswith("step"):
                # поля зависят от версии SUMO; стараемся быть толерантными
                ms = evt[1].get("meanSpeed") or evt[1].get("mean speed")
                if ms is not None:
                    try:
                        speeds.append(float(ms))
                    except Exception:
                        pass
                st = evt[1].get("stoppedVehicles")
                if st is not None:
                    try:
                        stopped.append(float(st))
                    except Exception:
                        pass
                wt = evt[1].get("waitingTime")
                if wt is not None:
                    try:
                        waiting.append(float(wt))
                    except Exception:
                        pass
    except Exception:
        pass
    m["mean_speed"] = sum(speeds) / len(speeds) if speeds else 0.0
    m["stopped_veh_avg"] = sum(stopped) / len(stopped) if stopped else 0.0
    m["total_waiting_time"] = sum(waiting) if waiting else 0.0
    return m


def parse_lane_edge(path: str) -> Dict[str, float]:
    m = {"lane_speed_avg": 0.0, "lane_occupancy_avg": 0.0}
    if not os.path.exists(path):
        return m
    speeds, occs = [], []
    try:
        for evt in ET.iterparse(path, events=("start",)):
            tag = evt[1].tag
            if tag.endswith("interval"):
                # meandata intervals may have aggregated stats
                sp = evt[1].get("speed")
                oc = evt[1].get("occupancy")
                if sp is not None:
                    try:
                        speeds.append(float(sp))
                    except Exception:
                        pass
                if oc is not None:
                    try:
                        occs.append(float(oc))
                    except Exception:
                        pass
    except Exception:
        pass
    m["lane_speed_avg"] = sum(speeds) / len(speeds) if speeds else 0.0
    m["lane_occupancy_avg"] = sum(occs) / len(occs) if occs else 0.0
    return m


def load_run(run_path: str) -> Dict[str, Dict[str, float]]:
    return {
        "trip": parse_tripinfos(os.path.join(run_path, "tripinfos.xml")),
        "summary": parse_summary(os.path.join(run_path, "summary.xml")),
        "lane": parse_lane_edge(os.path.join(run_path, "laneData.xml")),
        # edgeData.xml можно разобрать аналогично laneData.xml при необходимости
    }


def fmt(v: float) -> str:
    return f"{v:.2f}"


def print_compare(name: str, base: float, opt: float, better_when_lower: bool = True):
    delta = opt - base
    sign = "↓" if (better_when_lower and opt < base) or ((not better_when_lower) and opt > base) else "↑" if delta != 0 else "="
    print(f"{name:24} base={fmt(base)}  opt={fmt(opt)}  diff={fmt(delta)} {sign}")


def main():
    base_dir = os.path.join("out", "baseline")
    opt_dir = os.path.join("out", "opt")
    if len(sys.argv) >= 3:
        base_dir, opt_dir = sys.argv[1], sys.argv[2]
    base = load_run(base_dir)
    opt = load_run(opt_dir)
    print("KPI comparison (baseline vs opt):")
    print_compare("arrived", base["trip"]["arrived"], opt["trip"]["arrived"], better_when_lower=False)
    print_compare("avg travel time [s]", base["trip"]["avg_travel"], opt["trip"]["avg_travel"], True)
    print_compare("p95 travel time [s]", base["trip"]["p95_travel"], opt["trip"]["p95_travel"], True)
    print_compare("avg wait [s/veh]", base["trip"]["avg_wait"], opt["trip"]["avg_wait"], True)
    print_compare("mean speed [m/s]", base["summary"]["mean_speed"], opt["summary"]["mean_speed"], better_when_lower=False)
    print_compare("stopped veh avg", base["summary"]["stopped_veh_avg"], opt["summary"]["stopped_veh_avg"], True)
    print_compare("total waiting time [s]", base["summary"]["total_waiting_time"], opt["summary"]["total_waiting_time"], True)
    print_compare("lane speed avg [m/s]", base["lane"]["lane_speed_avg"], opt["lane"]["lane_speed_avg"], better_when_lower=False)
    print_compare("lane occupancy avg", base["lane"]["lane_occupancy_avg"], opt["lane"]["lane_occupancy_avg"], True)


if __name__ == "__main__":
    main()
