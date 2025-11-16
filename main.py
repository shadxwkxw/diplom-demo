# main.py: Основной скрипт (baseline / opt), запуск SUMO, сбор метрик и оптимизация.

import os
import sys
import argparse
import csv
import traci

from config import (
    SUMO_HOME,
    SIM_STEPS,
    OPTIMIZE_INTERVAL,
    GUI,
    SUMOCFG_FILE,
    OUT_BASELINE_DIR,
    OUT_OPT_DIR,
    TRIPINFO_FILE,
    SUMMARY_FILE,
)

from utils import (
    detect_near_miss,
    optimize_phases,
    visualize_results,
    select_traffic_light,
    analyze_tlslog,
)

TLSLOG_FILE = os.path.join(os.path.dirname(__file__), 'tlslog.xml')

# Подключаем SUMO tools, если SUMO_HOME задан
if SUMO_HOME:
    tools = os.path.join(SUMO_HOME, 'tools')
    if os.path.isdir(tools):
        sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

def generate_tlslog_from_observations(observed_csv_path, output_xml_path, tls_id):
    """Генерирует tlslog.xml из CSV с наблюдаемыми переключениями фаз"""
    import xml.etree.ElementTree as ET
    from xml.dom import minidom

    root = ET.Element('tlsStates')
    cumulative_time = 0.0

    try:
        with open(observed_csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                if row['tls_id'] == tls_id:
                    state = row['state']
                    duration = float(row['observed_duration_sec'])

                    tls_state = ET.SubElement(root, 'tlsState')
                    tls_state.set('time', str(round(cumulative_time, 2)))
                    tls_state.set('id', tls_id)
                    tls_state.set('state', state)

                    cumulative_time += duration
    except Exception as e:
        print(f"Ошибка при чтении {observed_csv_path}: {e}")
        return

    xml_str = minidom.parseString(ET.tostring(root)).toprettyxml(indent="    ")
    with open(output_xml_path, 'w') as f:
        f.write(xml_str)

def start_sumo(out_dir: str):
    """Запуск SUMO симуляции с выводом KPI в out_dir"""
    os.makedirs(out_dir, exist_ok=True)

    if GUI:
        # GUI-версия (окно)
        sumo_binary = "/Library/Frameworks/EclipseSUMO.framework/Versions/Current/EclipseSUMO/share/sumo/bin/sumo-gui"
        sumo_cmd = [
            sumo_binary,
            "-c", SUMOCFG_FILE,
            "--tripinfo-output", os.path.join(out_dir, TRIPINFO_FILE),
            "--summary-output", os.path.join(out_dir, SUMMARY_FILE),
        ]
    else:
        # Консольная версия
        sumo_binary = "/Library/Frameworks/EclipseSUMO.framework/Versions/Current/EclipseSUMO/share/sumo/bin/sumo"
        sumo_cmd = [
            sumo_binary,
            "-c", SUMOCFG_FILE,
            "--no-step-log", "true",
            "-v", "false",
            "--tripinfo-output", os.path.join(out_dir, TRIPINFO_FILE),
            "--summary-output", os.path.join(out_dir, SUMMARY_FILE),
        ]

    traci.start(sumo_cmd)
    print(f"SUMO запущен с конфигом: {SUMOCFG_FILE}")
    print(f"Tripinfos/Summary выводятся в: {out_dir}")

def run_simulation():
    parser = argparse.ArgumentParser(description='SUMO Traffic Light Control Script')
    parser.add_argument('--tls', type=str, help='ID конкретного светофора для управления')
    parser.add_argument('--mode', choices=['baseline', 'opt'], default='opt',
                        help='Режим: baseline (без оптимизации) или opt (с оптимизацией)')
    args = parser.parse_args()

    enable_optimization = (args.mode != 'baseline')

    # Каталог для результатов
    if args.mode == "baseline":
        out_dir = OUT_BASELINE_DIR
    else:
        out_dir = OUT_OPT_DIR

    try:
        start_sumo(out_dir)
    except traci.TraCIException as e:
        sys.exit(f"Failed to start SUMO: {e}")

    tls_ids = traci.trafficlight.getIDList()
    print(f"Доступные светофоры (TLS IDs): {tls_ids}")
    tls_id = select_traffic_light(traci, tls_ids, args.tls)
    if not tls_id:
        traci.close()
        sys.exit("Не удалось выбрать светофор")

    if tls_id not in tls_ids:
        print(f"ВНИМАНИЕ: {tls_id} не является ID светофора в SUMO.")
        print("Будет использован первый доступный светофор.")
        tls_id = tls_ids[0] if tls_ids else None
        if not tls_id:
            traci.close()
            sys.exit("Нет доступных светофоров")

    step = 0
    total_near_miss = 0
    total_delay = 0
    risk_history = []
    interval_near_miss = 0
    interval_delay = 0
    interval_risk_sum = 0
    interval_steps = 0

    # CSV для логирования применённых длительностей фаз
    csv_path = os.path.join(os.path.dirname(__file__), 'tls_changes.csv')
    try:
        csv_file = open(csv_path, mode='w', newline='')
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["step", "tls_id", "requested_durations", "applied_durations"])
    except Exception as e:
        csv_file = None
        print(f"Не удалось открыть файл для логирования изменений светофора: {e}")

    # CSV для наблюдаемых длительностей фаз
    observed_csv_path = os.path.join(os.path.dirname(__file__), 'tls_observed.csv')
    try:
        observed_file = open(observed_csv_path, mode='w', newline='')
        observed_writer = csv.writer(observed_file)
        observed_writer.writerow(["switch_step", "tls_id", "phase_index", "state", "observed_duration_sec", "epoch"])
    except Exception as e:
        observed_file = None
        print(f"Не удалось открыть файл для наблюдаемых длительностей фаз: {e}")

    current_time = 0.0
    prev_phase_index = None
    prev_switch_time = None
    observed_stats = {}
    current_epoch = 0
    observed_stats_epochs = {}

    def get_active_logic(tls_id_local):
        try:
            active_id = traci.trafficlight.getProgram(tls_id_local)
            all_logics = traci.trafficlight.getAllProgramLogics(tls_id_local)
            for lg in all_logics:
                if lg.programID == active_id:
                    return lg
            return all_logics[0] if all_logics else None
        except Exception:
            return None

    while step < SIM_STEPS:
        try:
            traci.simulationStep()
            current_time = traci.simulation.getTime()

            near_miss, risk = detect_near_miss()
            interval_near_miss += near_miss
            total_near_miss += near_miss
            current_delay = sum(traci.vehicle.getWaitingTime(veh) for veh in traci.vehicle.getIDList())
            interval_delay += current_delay
            total_delay += current_delay
            interval_risk_sum += risk
            interval_steps += 1
            risk_history.append(risk)

            # Отслеживание смены фазы и фиксация длительности
            try:
                phase_index = traci.trafficlight.getPhase(tls_id)
                if prev_phase_index is None:
                    prev_phase_index = phase_index
                    try:
                        elapsed = traci.trafficlight.getTimeSinceLastSwitch(tls_id)
                        prev_switch_time = max(0.0, current_time - elapsed)
                    except Exception:
                        prev_switch_time = current_time

                if phase_index != prev_phase_index and prev_switch_time is not None:
                    observed_duration = max(0.0, current_time - prev_switch_time)
                    state = None
                    try:
                        logic = traci.trafficlight.getAllProgramLogics(tls_id)[0]
                        if 0 <= prev_phase_index < len(logic.phases):
                            state = logic.phases[prev_phase_index].state
                    except Exception:
                        state = None

                    if observed_file:
                        observed_writer.writerow([
                            step,
                            tls_id,
                            prev_phase_index,
                            state or "?",
                            round(observed_duration, 2),
                            current_epoch
                        ])

                    st = observed_stats.get(prev_phase_index, {"sum": 0.0, "count": 0})
                    st["sum"] += observed_duration
                    st["count"] += 1
                    observed_stats[prev_phase_index] = st

                    epoch_bucket = observed_stats_epochs.get(current_epoch, {})
                    est = epoch_bucket.get(prev_phase_index, {"sum": 0.0, "count": 0})
                    est["sum"] += observed_duration
                    est["count"] += 1
                    epoch_bucket[prev_phase_index] = est
                    observed_stats_epochs[current_epoch] = epoch_bucket

                    prev_phase_index = phase_index
                    prev_switch_time = current_time
            except Exception:
                pass

            if enable_optimization and step % OPTIMIZE_INTERVAL == 0 and step > 0:
                avg_interval_risk = interval_risk_sum / interval_steps if interval_steps else 0
                current_logic = get_active_logic(tls_id)
                try:
                    new_durations = optimize_phases(interval_near_miss, avg_interval_risk, current_logic, tls_id)
                    applied_logic = get_active_logic(tls_id)
                    applied_durations = [p.duration for p in applied_logic.phases]
                    try:
                        active_program_id = traci.trafficlight.getProgram(tls_id)
                    except Exception:
                        active_program_id = "?"
                    print(f"Step {step}: Program {active_program_id} | Optimized: {new_durations} | Applied: {applied_durations}")
                    if csv_file:
                        csv_writer.writerow([
                            step,
                            tls_id,
                            ";".join(map(str, new_durations)),
                            ";".join(map(str, applied_durations))
                        ])
                    try:
                        current_epoch += 1
                    except Exception:
                        pass
                except Exception as e:
                    print(f"Step {step}: Error optimizing phases: {e}")
                    print("Continuing with current settings")

                interval_near_miss = 0
                interval_delay = 0
                interval_risk_sum = 0
                interval_steps = 0

            step += 1
        except traci.TraCIException as e:
            print(f"Simulation step error: {e}")
            break

    traci.close()

    # tlslog.xml из наблюдений
    try:
        generate_tlslog_from_observations(observed_csv_path, TLSLOG_FILE, tls_id)
        print(f"Создан tlslog.xml на основе наблюдений TraCI: {TLSLOG_FILE}")
    except Exception as e:
        print(f"Не удалось создать tlslog.xml: {e}")

    try:
        if csv_file:
            csv_file.close()
            print(f"Лог изменений светофора сохранён: {csv_path}")
        if observed_file:
            observed_file.close()
            print(f"Лог наблюдаемых длительностей фаз сохранён: {observed_csv_path}")
    except Exception:
        pass

    print(f"Total delay: {total_delay}, Total near-miss: {total_near_miss}")
    visualize_results(risk_history)

    # Анализ tlslog.xml
    try:
        summary = analyze_tlslog(tls_id, TLSLOG_FILE)
        if summary:
            print("TLSLOG summary (avg durations by state):")
            print(f"  {summary}")
        else:
            print("tlslog.xml пуст или не содержит записи по выбранному светофору")
    except Exception as e:
        print(f"Ошибка анализа tlslog.xml: {e}")

    # Сводка наблюдаемых длительностей
    try:
        if observed_stats:
            print("Observed phase durations (avg by phase index):")
            for idx, st in sorted(observed_stats.items()):
                avg = round(st["sum"] / max(1, st["count"]), 2)
                print(f"  phase {idx}: {avg}s over {st['count']} switches")
            if observed_stats_epochs:
                print("Observed phase durations per epoch (avg by phase index):")
                for epoch in sorted(observed_stats_epochs.keys()):
                    bucket = observed_stats_epochs[epoch]
                    label = "before first optimization" if epoch == 0 else f"after optimization #{epoch}"
                    print(f"  Epoch {epoch} ({label}):")
                    for idx, st in sorted(bucket.items()):
                        avg = round(st["sum"] / max(1, st["count"]), 2)
                        print(f"    phase {idx}: {avg}s over {st['count']} switches")
        else:
            print("Недостаточно наблюдений для расчёта фактических длительностей фаз.")
    except Exception:
        pass

if __name__ == "__main__":
    run_simulation()
