# utils.py: Вспомогательные функции
import numpy as np
import cvxpy as cp
import traci
import matplotlib.pyplot as plt

from config import (
    MIN_PHASE_DURATION,
    MAX_PHASE_DURATION,
    CYCLE_TIME,
    PROXIMITY_THRESHOLD,
)

def get_junction_info(traci, junction_id):
    """Получение информации о перекрестке"""
    try:
        position = traci.junction.getPosition(junction_id)
        junction_type = traci.junction.getType(junction_id)
        shape = traci.junction.getShape(junction_id)
        return {
            "id": junction_id,
            "position": position,
            "type": junction_type,
            "shape": shape
        }
    except Exception as e:
        print(f"Ошибка при получении информации о перекрестке {junction_id}: {e}")
        return None

def get_traffic_light_info(traci, tls_id):
    """Получение детальной информации о светофоре"""
    try:
        controlled_lanes = traci.trafficlight.getControlledLanes(tls_id)
        controlled_links = traci.trafficlight.getControlledLinks(tls_id)
        program = traci.trafficlight.getProgram(tls_id)
        complete_programs = traci.trafficlight.getCompleteRedYellowGreenDefinition(tls_id)
        phase = traci.trafficlight.getPhase(tls_id)
        state = traci.trafficlight.getRedYellowGreenState(tls_id)
        junctions = []
        if "#" in tls_id:  # признак кластера
            parts = tls_id.split("_")
            for part in parts:
                if part.isdigit():
                    junction_id = part
                    junction_info = get_junction_info(traci, junction_id)
                    if junction_info:
                        junctions.append(junction_info)
        return {
            "id": tls_id,
            "controlled_lanes": controlled_lanes,
            "controlled_links": controlled_links,
            "program": program,
            "complete_programs": complete_programs,
            "phase": phase,
            "state": state,
            "is_cluster": "#" in tls_id,
            "junctions": junctions
        }
    except Exception as e:
        print(f"Ошибка при получении информации о светофоре {tls_id}: {e}")
        return None

def extract_junctions_from_cluster(cluster_id):
    """Извлечение ID перекрестков из ID кластера"""
    if "#" not in cluster_id:
        return [cluster_id]
    parts = cluster_id.split("_")
    junctions = []
    for part in parts[1:]:
        if "#" in part:
            break
        junctions.append(part)
    return junctions

def select_traffic_light(traci, tls_ids, tls_id=None):
    """Выбор конкретного светофора из списка доступных"""
    if not tls_ids:
        print("Нет доступных светофоров в сети")
        return None
    if tls_id and tls_id in tls_ids:
        print(f"Выбран светофор: {tls_id}")
        return tls_id

    print("Доступные светофоры:")
    for i, tls in enumerate(tls_ids):
        is_cluster = "#" in tls
        cluster_info = " (кластер)" if is_cluster else ""
        print(f"{i+1}. {tls}{cluster_info}")
        if is_cluster:
            junctions = extract_junctions_from_cluster(tls)
            print(f" Перекрестки в кластере: {', '.join(junctions)}")
        try:
            state = traci.trafficlight.getRedYellowGreenState(tls)
            controlled_lanes = traci.trafficlight.getControlledLanes(tls)
            print(f" Количество контролируемых полос: {len(controlled_lanes)}")
            print(f" Текущее состояние: {state}")
        except Exception as e:
            print(f" Ошибка при получении информации о светофоре: {e}")

    try:
        choice = int(input("Выберите номер светофора (или нажмите Enter для выбора первого): ") or "1")
        if 1 <= choice <= len(tls_ids):
            selected_tls = tls_ids[choice - 1]
            print(f"Выбран светофор: {selected_tls}")
            if "#" in selected_tls:
                print("ВНИМАНИЕ: Выбран кластер светофоров.")
                print("Для корректной работы рекомендуется использовать весь кластер, а не отдельный перекресток.")
                use_cluster = input("Использовать весь кластер? (y/n, по умолчанию y): ").lower() != 'n'
                if not use_cluster:
                    junctions = extract_junctions_from_cluster(selected_tls)
                    print("Перекрестки в кластере:")
                    for i, junction in enumerate(junctions):
                        print(f"{i+1}. {junction}")
                    sub_choice = input("Выберите номер перекрестка: ")
                    if sub_choice and sub_choice.isdigit():
                        sub_choice = int(sub_choice)
                        if 1 <= sub_choice <= len(junctions):
                            junction_id = junctions[sub_choice - 1]
                            print(f"Выбран перекресток: {junction_id}")
                            print("ВНИМАНИЕ: Управление отдельным перекрестком в кластере может не работать корректно.")
                            print("Если возникнут ошибки, попробуйте использовать весь кластер.")
                            return junction_id
            return selected_tls
        else:
            print(f"Неверный выбор. Выбран первый светофор: {tls_ids[0]}")
            return tls_ids[0]
    except ValueError:
        print(f"Неверный ввод. Выбран первый светофор: {tls_ids[0]}")
        return tls_ids[0]

def detect_near_miss():
    """Детекция near-miss на основе TTC < 2 сек, с фильтром по расстоянию"""
    vehicles = traci.vehicle.getIDList()
    near_miss_count = 0
    risk_metrics = []
    for i, veh1 in enumerate(vehicles):
        pos1 = np.array(traci.vehicle.getPosition(veh1))
        speed1 = traci.vehicle.getSpeed(veh1)
        for veh2 in vehicles[i+1:]:
            pos2 = np.array(traci.vehicle.getPosition(veh2))
            dist = np.linalg.norm(pos1 - pos2)
            if dist > PROXIMITY_THRESHOLD:
                continue
            speed2 = traci.vehicle.getSpeed(veh2)
            rel_speed = abs(speed1 - speed2)
            if rel_speed > 0 and dist / rel_speed < 2.0:
                near_miss_count += 1
                risk_metrics.append(dist / rel_speed)
    avg_risk = np.mean(risk_metrics) if risk_metrics else 0
    return near_miss_count, avg_risk

# Счётчик программ для уникальных ID
_program_counter = 0

def optimize_phases(near_miss_count, avg_risk, current_logic, tls_id):
    """
    Оптимизация фаз:
    - считаем загрузку по фазам (сколько стоящих машин обслуживает каждая фаза)
    - распределяем CYCLE_TIME пропорционально этой загрузке
    - durations ≈ target при MIN/MAX и sum(durations) == CYCLE_TIME
    """
    global _program_counter
    _program_counter += 1

    num_phases = len(current_logic.phases)
    if num_phases == 0:
        print("No phases in current_logic, skipping optimization")
        return []

    # 1) Связи сигналов -> полосы
    try:
        controlled_links = traci.trafficlight.getControlledLinks(tls_id)
    except Exception as e:
        print(f"Cannot get controlled links for {tls_id}: {e}")
        return [phase.duration for phase in current_logic.phases]

    phase_loads = np.zeros(num_phases, dtype=float)

    # controlled_links: список по индексу сигнала (len == len(state))
    # каждая запись: список кортежей (from, to, via)
    for p_idx, phase in enumerate(current_logic.phases):
        state = phase.state
        for sig_idx, char in enumerate(state):
            if char not in ("G", "g"):
                continue
            if sig_idx >= len(controlled_links):
                continue
            links_for_signal = controlled_links[sig_idx]
            for link in links_for_signal:
                try:
                    via_lane = link[2]
                except Exception:
                    continue
                try:
                    q = traci.lane.getLastStepHaltingNumber(via_lane)
                except Exception:
                    q = 0
                phase_loads[p_idx] += q

    # 2) target-длительности пропорционально нагрузке
    total_load = float(phase_loads.sum())
    if total_load <= 0:
        target = np.array([CYCLE_TIME / num_phases] * num_phases, dtype=float)
    else:
        target = (phase_loads / total_load) * CYCLE_TIME

    target = np.clip(target, MIN_PHASE_DURATION, MAX_PHASE_DURATION)

    # 3) Оптимизация durations ≈ target
    durations = cp.Variable(num_phases)
    constraints = [
        durations >= MIN_PHASE_DURATION,
        durations <= MAX_PHASE_DURATION,
        cp.sum(durations) == CYCLE_TIME,
    ]
    objective = cp.Minimize(cp.sum_squares(durations - target))
    problem = cp.Problem(objective, constraints)
    problem.solve()

    if problem.status != cp.OPTIMAL:
        print(f"Optimization failed with status {problem.status}, using current durations")
        return [phase.duration for phase in current_logic.phases]

    new_durations = [int(round(d)) for d in durations.value]
    total = sum(new_durations)

    # 4) Коррекция суммы до CYCLE_TIME
    if total != CYCLE_TIME:
        diff = CYCLE_TIME - total
        i = 0
        while diff != 0 and i < num_phases:
            if diff > 0:
                add = min(diff, MAX_PHASE_DURATION - new_durations[i])
                new_durations[i] += add
                diff -= add
            else:
                sub = min(-diff, new_durations[i] - MIN_PHASE_DURATION)
                new_durations[i] -= sub
                diff += sub
            i = (i + 1) % num_phases

    # 5) Новый Logic и применение
    new_phases = []
    for i, phase in enumerate(current_logic.phases):
        new_phases.append(
            traci.trafficlight.Phase(
                new_durations[i],
                phase.state,
                new_durations[i],
                new_durations[i],
                phase.name,
            )
        )

    new_program_id = f"opt_{_program_counter}"
    new_logic = traci.trafficlight.Logic(
        new_program_id,
        current_logic.type,
        current_logic.currentPhaseIndex,
        phases=new_phases,
    )

    try:
        traci.trafficlight.setCompleteRedYellowGreenDefinition(tls_id, new_logic)
        try:
            traci.trafficlight.setProgram(tls_id, new_program_id)
        except traci.TraCIException:
            pass
        try:
            traci.trafficlight.setPhase(tls_id, 0)
        except traci.TraCIException:
            pass
    except traci.TraCIException as e:
        print(f"TraCI error when applying new logic: {e}")

    return new_durations

def visualize_results(risk_history):
    """Визуализация трендов риска"""
    plt.plot(risk_history)
    plt.xlabel('Time steps')
    plt.ylabel('Avg Risk')
    plt.title('Risk Trend')
    plt.savefig('risk_trend.png')
    print("Visualization saved to risk_trend.png")

def analyze_tlslog(tls_id, tlslog_path):
    """
    Анализ tlslog.xml: рассчитывает средние длительности по состояниям для указанного светофора.
    Возвращает словарь {state: avg_duration_seconds}.
    """
    import xml.etree.ElementTree as ET
    try:
        tree = ET.parse(tlslog_path)
    except Exception:
        return None
    root = tree.getroot()
    events = []
    for evt in root.findall('.//tlsState'):
        if evt.get('id') == tls_id:
            try:
                t = float(evt.get('time'))
                s = evt.get('state')
            except Exception:
                continue
            events.append((t, s))
    if len(events) < 2:
        return None

    durations_by_state = {}
    counts_by_state = {}
    for i in range(len(events) - 1):
        t0, s0 = events[i]
        t1, _ = events[i + 1]
        dur = max(0.0, t1 - t0)
        durations_by_state[s0] = durations_by_state.get(s0, 0.0) + dur
        counts_by_state[s0] = counts_by_state.get(s0, 0) + 1

    avg_by_state = {
        state: round(durations_by_state[state] / counts_by_state[state], 2)
        for state in durations_by_state
    }
    return avg_by_state
