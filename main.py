# main.py: Основной скрипт, который запускает симуляцию, собирает метрики и оптимизирует фазы. Здесь основной цикл.
import os
import sys
import argparse
import csv
import traci
from sumolib import checkBinary
from config import SUMO_HOME, SIM_STEPS, OPTIMIZE_INTERVAL, GUI, SUMOCFG_FILE
from utils import detect_near_miss, optimize_phases, visualize_results, select_traffic_light
from utils import analyze_tlslog

TLSLOG_FILE = os.path.join(os.path.dirname(__file__), 'tlslog.xml')
if SUMO_HOME:
    tools = os.path.join(SUMO_HOME, 'tools')
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
                    
                    # Добавляем событие переключения в начало фазы
                    tls_state = ET.SubElement(root, 'tlsState')
                    tls_state.set('time', str(round(cumulative_time, 2)))
                    tls_state.set('id', tls_id)
                    tls_state.set('state', state)
                    
                    cumulative_time += duration
    except Exception as e:
        print(f"Ошибка при чтении {observed_csv_path}: {e}")
        return
    
    # Записываем XML с форматированием
    xml_str = minidom.parseString(ET.tostring(root)).toprettyxml(indent="    ")
    with open(output_xml_path, 'w') as f:
        f.write(xml_str)

def start_sumo():
    """Запуск SUMO симуляции"""
    if GUI:
        # Используем тот же путь, что и в script.py для GUI-версии
        sumo_binary = "/Library/Frameworks/EclipseSUMO.framework/Versions/Current/EclipseSUMO/share/sumo/bin/sumo-gui"
        # Для GUI-версии убираем флаги скрытия вывода, чтобы показать окно
        sumo_cmd = [sumo_binary, "-c", SUMOCFG_FILE]
    else:
        # Для не-GUI версии тоже используем проверенный путь
        sumo_binary = "/Library/Frameworks/EclipseSUMO.framework/Versions/Current/EclipseSUMO/share/sumo/bin/sumo"
        # Для не-GUI версии добавляем флаги скрытия вывода
        sumo_cmd = [sumo_binary, "-c", SUMOCFG_FILE, "--no-step-log", "true", "-v", "false"]
    
    traci.start(sumo_cmd)
    print(f"SUMO запущен с конфигом: {SUMOCFG_FILE}")

def run_simulation():
    parser = argparse.ArgumentParser(description='SUMO Traffic Light Control Script')
    parser.add_argument('--tls', type=str, help='ID конкретного светофора для управления')
    parser.add_argument('--mode', choices=['baseline', 'opt'], default='opt', help='Режим: baseline (без оптимизации) или opt (с оптимизацией)')
    args = parser.parse_args()
    enable_optimization = (args.mode != 'baseline')

    try:
        start_sumo()
    except traci.TraCIException as e:
        sys.exit(f"Failed to start SUMO: {e}")
   
    tls_ids = traci.trafficlight.getIDList()
    print(f"Доступные светофоры (TLS IDs): {tls_ids}")
    tls_id = select_traffic_light(traci, tls_ids, args.tls)
    if not tls_id:
        traci.close()
        sys.exit("Не удалось выбрать светофор")

    # Проверяем, является ли выбранный ID светофором
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

    # Подготовка CSV для логирования применённых длительностей фаз
    csv_path = os.path.join(os.path.dirname(__file__), 'tls_changes.csv')
    try:
        csv_file = open(csv_path, mode='w', newline='')
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["step", "tls_id", "requested_durations", "applied_durations"])
    except Exception as e:
        csv_file = None
        print(f"Не удалось открыть файл для логирования изменений светофора: {e}")
    # Подготовка CSV для наблюдаемых (реально отработанных) длительностей фаз
    observed_csv_path = os.path.join(os.path.dirname(__file__), 'tls_observed.csv')
    try:
        observed_file = open(observed_csv_path, mode='w', newline='')
        observed_writer = csv.writer(observed_file)
        # Добавляем epoch для разделения "до"/"после" оптимизации
        observed_writer.writerow(["switch_step", "tls_id", "phase_index", "state", "observed_duration_sec", "epoch"])
    except Exception as e:
        observed_file = None
        print(f"Не удалось открыть файл для наблюдаемых длительностей фаз: {e}")

    # Инициализация отслеживания длительности фаз по переключениям
    current_time = 0.0
    prev_phase_index = None
    prev_switch_time = None
    observed_stats = {}  # phase_index -> {sum: float, count: int}
    # Epoch: 0 = до первой оптимизации, 1 = после первой, и т.д.
    current_epoch = 0
    observed_stats_epochs = {}  # epoch -> {phase_index -> {sum: float, count: int}}

    # Вспомогательная функция: получить активную логику по текущему programID
    def get_active_logic(tls_id_local):
        try:
            active_id = traci.trafficlight.getProgram(tls_id_local)
            all_logics = traci.trafficlight.getAllProgramLogics(tls_id_local)
            for lg in all_logics:
                if lg.programID == active_id:
                    return lg
            # Фолбэк: если не нашли по programID, вернуть первую
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
           
            # Отслеживание смены фазы и фиксация фактической длительности
            try:
                phase_index = traci.trafficlight.getPhase(tls_id)
                # Инициализируем предыдущие значения при первом проходе
                if prev_phase_index is None:
                    prev_phase_index = phase_index
                    # Восстановим время последнего переключения, чтобы корректно закрыть первую фазу
                    try:
                        elapsed = traci.trafficlight.getTimeSinceLastSwitch(tls_id)
                        prev_switch_time = max(0.0, current_time - elapsed)
                    except Exception:
                        prev_switch_time = current_time
                # Если фаза сменилась, считаем длительность предыдущей фазы
                if phase_index != prev_phase_index and prev_switch_time is not None:
                    observed_duration = max(0.0, current_time - prev_switch_time)
                    # Пытаемся получить состояние предыдущей фазы из активной логики
                    state = None
                    try:
                        logic = traci.trafficlight.getAllProgramLogics(tls_id)[0]
                        if 0 <= prev_phase_index < len(logic.phases):
                            state = logic.phases[prev_phase_index].state
                    except Exception:
                        state = None
                    # Пишем в CSV
                    if observed_file:
                        observed_writer.writerow([step, tls_id, prev_phase_index, state or "?", round(observed_duration, 2), current_epoch])
                    # Агрегируем статистику
                    st = observed_stats.get(prev_phase_index, {"sum": 0.0, "count": 0})
                    st["sum"] += observed_duration
                    st["count"] += 1
                    observed_stats[prev_phase_index] = st
                    # Агрегируем по эпохам
                    epoch_bucket = observed_stats_epochs.get(current_epoch, {})
                    est = epoch_bucket.get(prev_phase_index, {"sum": 0.0, "count": 0})
                    est["sum"] += observed_duration
                    est["count"] += 1
                    epoch_bucket[prev_phase_index] = est
                    observed_stats_epochs[current_epoch] = epoch_bucket
                    # Обновляем предыдущие
                    prev_phase_index = phase_index
                    prev_switch_time = current_time
            except Exception:
                pass

            if enable_optimization and step % OPTIMIZE_INTERVAL == 0 and step > 0:
                avg_interval_risk = interval_risk_sum / interval_steps if interval_steps else 0
                current_logic = get_active_logic(tls_id)
                try:
                    # Ensure current_logic.phases is a list of traffic light phases
                    # and that individual phase durations are integers.
                    new_durations = optimize_phases(interval_near_miss, avg_interval_risk, current_logic, tls_id)
                    # Читаем обратно применённые длительности фаз из активной логики
                    applied_logic = get_active_logic(tls_id)
                    applied_durations = [p.duration for p in applied_logic.phases]
                    try:
                        active_program_id = traci.trafficlight.getProgram(tls_id)
                    except Exception:
                        active_program_id = "?"
                    print(f"Step {step}: Program {active_program_id} | Optimized: {new_durations} | Applied: {applied_durations}")
                    # Сохраняем в CSV для последующей проверки
                    if csv_file:
                        csv_writer.writerow([step, tls_id, ";".join(map(str, new_durations)), ";".join(map(str, applied_durations))])
                    # После успешной оптимизации переключаем эпоху ("после оптимизации")
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
    
    # Создаем tlslog.xml из собранных данных наблюдений
    try:
        generate_tlslog_from_observations(observed_csv_path, TLSLOG_FILE, tls_id)
        print(f"Создан tlslog.xml на основе наблюдений TraCI: {TLSLOG_FILE}")
    except Exception as e:
        print(f"Не удалось создать tlslog.xml: {e}")
    
    # Закрываем CSV-файл, если открывали
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
    # Анализ tlslog.xml: сравнение средних длительностей фаз до/после оптимизации
    try:
        summary = analyze_tlslog(tls_id, TLSLOG_FILE)
        if summary:
            print("TLSLOG summary (avg durations by state):")
            print(f"  {summary}")
        else:
            print("tlslog.xml пуст или не содержит записи по выбранному светофору")
    except Exception as e:
        print(f"Ошибка анализа tlslog.xml: {e}")

    # Сводка наблюдаемых длительностей по индексам фаз (TraCI)
    try:
        if observed_stats:
            print("Observed phase durations (avg by phase index):")
            for idx, st in sorted(observed_stats.items()):
                avg = round(st["sum"] / max(1, st["count"]), 2)
                print(f"  phase {idx}: {avg}s over {st['count']} switches")
            # Сводка по эпохам: до/после оптимизации
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