# utils.py: Вспомогательные функции
import numpy as np
import cvxpy as cp
import traci
import matplotlib.pyplot as plt
from config import MIN_PHASE_DURATION, MAX_PHASE_DURATION, CYCLE_TIME, PROXIMITY_THRESHOLD

def get_junction_info(traci, junction_id):
    """Получение информации о перекрестке"""
    try:
        # Получаем позицию перекрестка
        position = traci.junction.getPosition(junction_id)
        # Получаем тип перекрестка
        junction_type = traci.junction.getType(junction_id)
        # Получаем форму перекрестка
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
        # Получаем список контролируемых полос
        controlled_lanes = traci.trafficlight.getControlledLanes(tls_id)
        # Получаем список контролируемых связей (links)
        controlled_links = traci.trafficlight.getControlledLinks(tls_id)
        # Получаем текущую программу светофора
        program = traci.trafficlight.getProgram(tls_id)
        # Получаем все программы светофора
        complete_programs = traci.trafficlight.getCompleteRedYellowGreenDefinition(tls_id)
        # Получаем текущую фазу
        phase = traci.trafficlight.getPhase(tls_id)
        # Получаем текущее состояние (RYG)
        state = traci.trafficlight.getRedYellowGreenState(tls_id)
        # Если это кластер, получаем информацию о перекрестках
        junctions = []
        if "#" in tls_id: # Признак кластера
            # Получаем список перекрестков в кластере
            # Примечание: это эвристика, так как SUMO не предоставляет прямой API для этого
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
        return [cluster_id] # Не кластер, возвращаем как есть
    # Формат кластера: cluster_id1_id2_id3_#2more
    # Извлекаем все ID до #
    parts = cluster_id.split("_")
    junctions = []
    for part in parts[1:]: # Пропускаем "cluster"
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
    # Если светофор не указан или указан неверно, показываем список и предлагаем выбрать
    print("Доступные светофоры:")
    for i, tls in enumerate(tls_ids):
        # Проверяем, является ли светофор кластером
        is_cluster = "#" in tls
        cluster_info = " (кластер)" if is_cluster else ""
        print(f"{i+1}. {tls}{cluster_info}")
        # Если это кластер, показываем дополнительную информацию
        if is_cluster:
            junctions = extract_junctions_from_cluster(tls)
            print(f" Перекрестки в кластере: {', '.join(junctions)}")
        # Получаем информацию о светофоре
        try:
            # Получаем текущее состояние светофора
            state = traci.trafficlight.getRedYellowGreenState(tls)
            # Получаем список контролируемых полос
            controlled_lanes = traci.trafficlight.getControlledLanes(tls)
            print(f" Количество контролируемых полос: {len(controlled_lanes)}")
            print(f" Текущее состояние: {state}")
        except Exception as e:
            print(f" Ошибка при получении информации о светофоре: {e}")
    try:
        choice = int(input("Выберите номер светофора (или нажмите Enter для выбора первого): ") or "1")
        if 1 <= choice <= len(tls_ids):
            selected_tls = tls_ids[choice-1]
            print(f"Выбран светофор: {selected_tls}")
            # Если выбран кластер, предлагаем выбрать конкретный перекресток
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
                            junction_id = junctions[sub_choice-1]
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
        for veh2 in vehicles[i+1:]: # Избежать двойного счета и self
            pos2 = np.array(traci.vehicle.getPosition(veh2))
            dist = np.linalg.norm(pos1 - pos2)
            if dist > PROXIMITY_THRESHOLD: # Фильтр для эффективности
                continue
            speed2 = traci.vehicle.getSpeed(veh2)
            rel_speed = abs(speed1 - speed2)
            if rel_speed > 0 and dist / rel_speed < 2.0:
                near_miss_count += 1
                risk_metrics.append(dist / rel_speed)
    avg_risk = np.mean(risk_metrics) if risk_metrics else 0
    return near_miss_count, avg_risk

# Global counter for program IDs
_program_counter = 0

def collect_phase_inflow_outflow(tls_id, logic):
    """
    Собирает реальный приток и отток машин за последние N секунд по каждому подходу (link).
    Возвращает два массива одинаковой длины с len(phases):
        inflow_per_phase[i]  – сколько машин въехало на контролируемые связи во время фазы i
        outflow_per_phase[i] – сколько машин выехало (проехало перекресток) во время фазы i
    """
    n = len(logic.phases)
    inflow  = np.zeros(n)
    outflow = np.zeros(n)

    # Сохраняем историю за последние ~5–10 минут (достаточно для адаптации)
    history_sec = 300
    for link in traci.trafficlight.getControlledLinks(tls_id):
        if not link[0]:  # иногда есть пустые tuple
            continue
        lane = link[0][0]  # (lane, toLane, hasPrio)
        # Считаем количество вошедших и вышедших за историю
        entered = traci.lane.getLastStepVehiclesEntered(lane) if hasattr(traci.lane, 'getLastStepVehiclesEntered') else traci.lane.getLastStepHaltingNumber(lane)
        # Более надёжный способ — edge-based
        edge = traci.lane.getEdgeID(lane)
        try:
            entered = traci.edge.getLastStepVehicleNumberEntered(edge)
            left    = traci.edge.getLastStepVehicleNumberLeft(edge)
        except:
            continue

        # Определяем, к какой фазе относится эта связь (по маске состояния)
        for phase_idx, phase in enumerate(logic.phases):
            state = phase.state
            link_index = traci.trafficlight.getControlledLinks(tls_id).index(link)
            if link_index >= len(state):
                continue
            if state[link_index] in 'Gg':  # зелёный для этой связи
                inflow[phase_idx]  += entered
                outflow[phase_idx] += left

    # Если всё нули — возвращаем равномерные веса (защита от первого шага)
    if inflow.sum() == 0:
        inflow = np.ones(n)
    if outflow.sum() == 0:
        outflow = np.ones(n)

    return inflow, outflow


def optimize_phases(near_miss_count, avg_risk, current_logic, tls_id):
    """Рабочая адаптивная оптимизация 2025 edition — теперь без убийственных пробок"""
    global _program_counter
    _program_counter += 1

    logic = current_logic
    n = len(logic.phases)
    if n == 0:
        return [p.duration for p in logic.phases]

    # ===================================================================
    # 1. Собираем реальную статистику за последние 180–300 сек (накопительно)
    # ===================================================================
    observation_window = 240  # секунд истории

    inflow_per_link = {}   # link_tuple -> количество вошедших за окно
    outflow_per_link = {}  # link_tuple -> количество выехавших за окно

    for link in traci.trafficlight.getControlledLinks(tls_id):
        if not link or link[0] == (): 
            continue
        lane = link[0][0]
        edge = traci.lane.getEdgeID(lane)

        # Эти функции возвращают накопленные значения с начала симуляции
        entered = traci.edge.getLastStepVehicleNumber(edge)   # текущий шаг
        left    = traci.edge.getLastStepVehicleNumber(edge)

        # Чтобы получить за окно — вычитаем значения observation_window секунд назад
        # TraCI не хранит историю автоматически → мы сами храним в глобальном словаре
        key = f"{tls_id}_{edge}"
        if not hasattr(optimize_phases, "history"):
            optimize_phases.history = {}
        
        hist = optimize_phases.history.get(key, {"time": 0.0, "entered": 0, "left": 0})
        curr_time = traci.simulation.getTime()

        if curr_time - hist["time"] > observation_window:
            # Сброс старых значений
            hist["entered"] = entered
            hist["left"]    = left
            hist["time"]    = curr_time

        inflow  = max(0, entered - hist["entered"])
        outflow = max(0, left - hist["left"])

        inflow_per_link[link]  = inflow
        outflow_per_link[link] = outflow

        # Обновляем историю
        optimize_phases.history[key] = {"time": curr_time, "entered": entered, "left": left}

    # ===================================================================
    # 2. Распределяем inflow по фазам (по маске зелёного)
    # ===================================================================
    phase_demand = np.zeros(n)
    phase_saturation = np.zeros(n)

    for link, inflow in inflow_per_link.items():
        outflow = outflow_per_link.get(link, 0.0)
        try:
            idx = traci.trafficlight.getControlledLinks(tls_id).index(link)
        except ValueError:
            continue

        for phase_idx in range(n):
            state_char = logic.phases[phase_idx].state[idx] if idx < len(logic.phases[phase_idx].state) else 'r'
            if state_char in 'Gg':  # зелёный
                phase_demand[phase_idx] += inflow
                if inflow > 0:
                    phase_saturation[phase_idx] = max(phase_saturation[phase_idx], outflow / inflow)

    # Защита от нулевого трафика
    if phase_demand.sum() == 0:
        phase_demand = np.ones(n)

    # Нормализуем спрос
    demand_share = phase_demand / phase_demand.sum()

    # ===================================================================
    # 3. Целевые длительности: Webster-подобная формула + штраф за near-miss
    # ===================================================================
    effective_green_target = demand_share * (CYCLE_TIME - n * 3)  # 3 сек yellow+allred на переход
    effective_green_target = np.maximum(effective_green_target, MIN_PHASE_DURATION)
    
    # Уменьшаем время сильно насыщенным фазам (уже не могут больше пропустить)
    effective_green_target *= (1.0 - 0.5 * phase_saturation)

    # Сильный штраф за near-miss — сокращаем все зелёные фазы пропорционально
    if near_miss_count > 0:
        reduction = min(0.4, near_miss_count * 0.05)  # max 40% сокращение
        effective_green_target *= (1.0 - reduction)

    # ===================================================================
    # 4. CVXPY — мягкая подгонка под target
    # ===================================================================
    d = cp.Variable(n, nonneg=True)
    objective = cp.Minimize(cp.sum_squares(d - effective_green_target))
    constraints = [
        d >= MIN_PHASE_DURATION,
        d <= MAX_PHASE_DURATION,
        cp.sum(d) == CYCLE_TIME - n * 3   # оставляем место под жёлтые (уже есть в оригинальной логике!)
    ]

    problem = cp.Problem(objective, constraints)
    problem.solve(solver=cp.OSQP)

    if problem.status not in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
        return [p.duration for p in logic.phases]

    new_durations = np.round(d.value).astype(int).tolist()

    # Корректировка суммы (на всякий случай)
    current_sum = sum(new_durations)
    needed = CYCLE_TIME - n * 3
    diff = needed - current_sum
    if abs(diff) > 0:
        idxs = np.argsort(-phase_demand)  # сначала добавляем самым загруженным
        i = 0
        while diff != 0 and i < n:
            delta = 1 if diff > 0 else -1
            if MIN_PHASE_DURATION <= new_durations[idxs[i]] + delta <= MAX_PHASE_DURATION:
                new_durations[idxs[i]] += delta
                diff -= delta
            i += 1

    # ===================================================================
    # 5. Применяем — НО НЕ ТРОГАЕМ ЖЁЛТЫЕ И НЕ СБРАСЫВАЕМ ФАЗУ!
    # ===================================================================
    new_phases = []
    for i, old_phase in enumerate(logic.phases):
        new_ph = traci.trafficlight.Phase(
            duration=new_durations[i],
            state=old_phase.state,
            minDur=max(MIN_PHASE_DURATION, new_durations[i] - 5),
            maxDur=min(MAX_PHASE_DURATION, new_durations[i] + 10),
            name=old_phase.name
        )
        new_phases.append(new_ph)

    new_program_id = f"opt_{_program_counter}"
    new_logic = traci.trafficlight.Logic(
        programID=new_program_id,
        type=logic.type,
        currentPhaseIndex=traci.trafficlight.getPhase(tls_id),  # сохраняем текущую фазу!
        phases=new_phases
    )

    try:
        traci.trafficlight.setCompleteRedYellowGreenDefinition(tls_id, new_logic)
        traci.trafficlight.setProgram(tls_id, new_program_id)
        # НЕ ДЕЛАЕМ setPhase(0) — это убивало координацию!
    except Exception as e:
        print(f"Не удалось применить оптимизацию: {e}")

    return new_durations

def visualize_results(risk_history):
    """Визуализация трендов риска"""
    plt.plot(risk_history)
    plt.xlabel('Time steps')
    plt.ylabel('Avg Risk')
    plt.title('Risk Trend')
    plt.savefig('risk_trend.png') # Save instead of show
    print("Visualization saved to risk_trend.png")

def analyze_tlslog(tls_id, tlslog_path):
    """Анализ tlslog.xml: рассчитывает средние длительности по состояниям для указанного светофора.
    Возвращает словарь {state: avg_duration_seconds}.
    """
    import xml.etree.ElementTree as ET
    try:
        tree = ET.parse(tlslog_path)
    except Exception:
        return None
    root = tree.getroot()
    # Формат записей: <tlsState time="t" id="TLS_ID" state="ryG..."/>
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
    # Считаем длительности как разницу времени между соседними событиями для каждого состояния
    durations_by_state = {}
    counts_by_state = {}
    for i in range(len(events) - 1):
        t0, s0 = events[i]
        t1, _ = events[i + 1]
        dur = max(0.0, t1 - t0)
        durations_by_state[s0] = durations_by_state.get(s0, 0.0) + dur
        counts_by_state[s0] = counts_by_state.get(s0, 0) + 1
    # Усредняем
    avg_by_state = {state: round(durations_by_state[state] / counts_by_state[state], 2) for state in durations_by_state}
    return avg_by_state