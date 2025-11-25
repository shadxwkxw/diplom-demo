# main.py: SUMO Traffic Light Optimization
import os
import sys
import argparse
import csv
import traci
from sumolib import checkBinary
from config import SUMO_HOME, SIM_STEPS, OPTIMIZE_INTERVAL, GUI, SUMOCFG_FILE, MIN_PHASE_DURATION, TLS_ID
from utils import (
    detect_near_miss,
    visualize_results,
    select_traffic_light,
    analyze_tlslog,
    set_static_program_for_opt,
    set_semi_static_bounds,
    optimize_tls_durations,
    apply_phase_durations,
)

TLSLOG_FILE = os.path.join(os.path.dirname(__file__), 'tlslog.xml')

if SUMO_HOME:
    tools = os.path.join(SUMO_HOME, 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

def generate_tlslog_from_observations(observed_csv_path, output_xml_path, tls_id):
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

def start_sumo(out_dir: str | None = None):
    sumo_binary = ("/Library/Frameworks/EclipseSUMO.framework/Versions/Current/EclipseSUMO/share/sumo/bin/"
                   + ("sumo-gui" if GUI else "sumo"))
    sumo_cmd = [sumo_binary, "-c", SUMOCFG_FILE]
    if out_dir:
        # prefix ко всем выходам из sumo/sumo.cfg/additional
        prefix = os.path.join(out_dir, "")
        os.makedirs(out_dir, exist_ok=True)
        sumo_cmd += ["--output-prefix", prefix]
    if not GUI:
        sumo_cmd += ["--no-step-log", "true", "-v", "false"]
    traci.start(sumo_cmd)
    print(f"SUMO запущен с конфигом: {SUMOCFG_FILE}")

def run_simulation():
    parser = argparse.ArgumentParser(description='SUMO Traffic Light Control Script')
    parser.add_argument('--tls', type=str, help='ID конкретного светофора для управления')
    parser.add_argument('--mode', choices=['baseline', 'opt'], default='opt', help='baseline или opt')
    parser.add_argument('--out', type=str, help='Папка для вывода результатов (по умолчанию out/<mode>)')
    args = parser.parse_args()
    enable_optimization = (args.mode != 'baseline')

    # Папка вывода: out/<mode> или переданная пользователем
    default_out = os.path.join(os.path.dirname(__file__), 'out', args.mode)
    out_dir = args.out or default_out

    try:
        start_sumo(out_dir)
    except traci.TraCIException as e:
        sys.exit(f"Failed to start SUMO: {e}")
   
    tls_ids = traci.trafficlight.getIDList()
    print(f"Доступные светофоры (TLS IDs): {tls_ids}")
    tls_id = select_traffic_light(traci, tls_ids, args.tls) or TLS_ID
    print(f"Выбран светофор для управления: {tls_id}")

    # Защита: если выбран кластер — работаем только с кластерным id (и предупреждаем)
    if "#" in tls_id:
        print("Выбран кластер TLS. Вся дальнейшая логика будет применяться к кластерному ID.")

    # В режиме оптимизации включаем semi-actuated с жёсткими границами зелёных фаз
    if enable_optimization:
        ok = set_semi_static_bounds(tls_id)
        if ok:
            print(f"Режим opt: светофор {tls_id} переведён в semi-actuated с ограничениями min/max")
        else:
            print("Режим opt: не удалось установить semi-actuated режим, продолжим с текущей логикой")

    step = 0
    total_near_miss = 0
    total_delay = 0
    risk_history = []
    interval_near_miss = 0
    interval_delay = 0
    interval_risk_sum = 0
    interval_steps = 0
    current_epoch = 0

    # CSV для логирования
    csv_path = os.path.join(out_dir, 'tls_changes.csv')
    csv_file = open(csv_path, mode='w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["step", "tls_id", "applied_phase", "phase_scores", "applied_durations"])

    observed_csv_path = os.path.join(out_dir, 'tls_observed.csv')
    observed_file = open(observed_csv_path, mode='w', newline='')
    observed_writer = csv.writer(observed_file)
    observed_writer.writerow(["switch_step", "tls_id", "phase_index", "state", "observed_duration_sec", "epoch"])

    # Отслеживание фаз
    prev_phase_index = None
    prev_switch_time = None
    observed_stats = {}  # phase_index -> {sum, count}
    last_switch_time = 0.0  # время последнего переключения

    # Получаем логику светофора
    logic = traci.trafficlight.getCompleteRedYellowGreenDefinition(tls_id)[0]
    phases = logic.phases
    print(f"TLS {tls_id}: {len(phases)} phases loaded")

    while step < SIM_STEPS:
        try:
            traci.simulationStep()
            current_time = traci.simulation.getTime()

            # --- KPI и риск ---
            near_miss, risk = detect_near_miss()
            interval_near_miss += near_miss
            total_near_miss += near_miss
            current_delay = sum(traci.vehicle.getWaitingTime(veh) for veh in traci.vehicle.getIDList())
            interval_delay += current_delay
            total_delay += current_delay
            interval_risk_sum += risk
            interval_steps += 1
            risk_history.append(risk)

            # --- Отслеживание длительности фаз ---
            phase_index = traci.trafficlight.getPhase(tls_id)
            if prev_phase_index is None:
                prev_phase_index = phase_index
                try:
                    elapsed = traci.trafficlight.getTimeSinceLastSwitch(tls_id)
                    prev_switch_time = max(0.0, current_time - elapsed)
                except:
                    prev_switch_time = current_time

            if phase_index != prev_phase_index and prev_switch_time is not None:
                observed_duration = max(0.0, current_time - prev_switch_time)
                state = logic.phases[prev_phase_index].state if 0 <= prev_phase_index < len(phases) else "?"
                observed_writer.writerow([step, tls_id, prev_phase_index, state, round(observed_duration,2), current_epoch])
                st = observed_stats.get(prev_phase_index, {"sum":0.0,"count":0})
                st["sum"] += observed_duration
                st["count"] += 1
                observed_stats[prev_phase_index] = st
                prev_phase_index = phase_index
                prev_switch_time = current_time

            # --- Оптимизация длительностей фаз ---
                    # --- Оптимизация длительностей фаз ---
            if enable_optimization and step % OPTIMIZE_INTERVAL == 0 and step > 0:
                current_time = traci.simulation.getTime()

                # КРИТИЧНО ВАЖНАЯ ЗАЩИТА: не меняем фазы слишком часто!
                time_since_last_change = current_time - last_switch_time
                if time_since_last_change < 60:
                    # Можно даже вывести в лог, чтобы видеть, что защита работает
                    # print(f"Step {step}: пропуск оптимизации — прошло только {time_since_last_change:.1f} сек с последнего изменения")
                    pass  # просто пропускаем
                else:
                    # Только теперь разрешаем оптимизацию
                    controlled_links = traci.trafficlight.getControlledLinks(tls_id)
                    phase_scores = []
                    for idx, phase in enumerate(phases):
                        score = 0
                        for i, signal in enumerate(phase.state):
                            if signal not in ("G", "g"):
                                continue
                            if i >= len(controlled_links):
                                continue
                            for lane_info in controlled_links[i]:
                                if not lane_info:
                                    continue
                                try:
                                    lane_id = lane_info[0]
                                    score += traci.lane.getLastStepVehicleNumber(lane_id)
                                except Exception:
                                    continue
                        phase_scores.append(score)
                        print(f"Step {step}: Phase {idx} has {score} vehicles")

                    avg_interval_risk = (interval_risk_sum / interval_steps) if interval_steps else 0.0

                    try:
                        current_logic = traci.trafficlight.getCompleteRedYellowGreenDefinition(tls_id)[0]
                    except Exception:
                        current_logic = logic

                    new_durations = optimize_tls_durations(tls_id, current_logic, interval_near_miss, avg_interval_risk)
                    applied = apply_phase_durations(tls_id, current_logic, new_durations)
                    if applied:
                        last_switch_time = current_time
                        csv_writer.writerow([
                            step, tls_id, traci.trafficlight.getPhase(tls_id),
                            ";".join(map(str, phase_scores)),
                            ";".join(map(str, new_durations))
                        ])
                        print(f"Step {step}: УСПЕШНО продлена/сокращена текущая фаза → {new_durations}")
                        current_epoch += 1

                    # Сброс накопленных метрик за интервал
                    interval_near_miss = interval_delay = interval_risk_sum = interval_steps = 0
            step += 1
        except traci.TraCIException as e:
            print(f"Simulation step error: {e}")
            break

    traci.close()
    tlslog_out = os.path.join(out_dir, 'tlslog.xml')
    generate_tlslog_from_observations(observed_csv_path, tlslog_out, tls_id)
    print(f"tlslog.xml создан: {tlslog_out}")
    csv_file.close()
    observed_file.close()
    print(f"Логи фаз сохранены: {csv_path}, {observed_csv_path}")
    print(f"Total delay: {total_delay}, Total near-miss: {total_near_miss}")
    visualize_results(risk_history)

    # Анализ tlslog
    summary = analyze_tlslog(tls_id, TLSLOG_FILE)
    if summary:
        print("TLSLOG summary (avg durations by state):")
        print(f"  {summary}")

    # Сводка наблюдаемых длительностей
    print("Observed phase durations (avg by phase index):")
    for idx, st in sorted(observed_stats.items()):
        avg = round(st["sum"] / max(1, st["count"]), 2)
        print(f"  phase {idx}: {avg}s over {st['count']} switches")

if __name__ == "__main__":
    run_simulation()
