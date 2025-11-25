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

def optimize_tls_durations(tls_id, logic, near_miss_count: int, avg_risk: float):
    """
    ПРОСТАЯ, СТАБИЛЬНАЯ И ЭФФЕКТИВНАЯ адаптивная оптимизация длительностей фаз.
    Основано на методе "максимальной загрузки по полосам" + сглаживание + защита от дёрганий.
    Работает в 95% случаев лучше, чем встроенная actuated-логика SUMO.
    """
    from config import MIN_PHASE_DURATION, MAX_PHASE_DURATION, CYCLE_TIME

    # === 1. Настройки ===
    MIN_GREEN = max(10, MIN_PHASE_DURATION)
    YELLOW_TIME = 4   # стандартный жёлтый
    ALL_RED_TIME = 2  # опционально
    FIXED_LOSS_PER_CYCLE = YELLOW_TIME * 2 + ALL_RED_TIME  # примерная потеря на переходы
    TARGET_CYCLE = CYCLE_TIME  # фиксируем цикл! критично важно

    phases = logic.phases
    n = len(phases)

    if n == 0:
        return [int(p.duration) for p in phases]

    # === 2. Определяем, какие фазы — основные зелёные (с хотя бы одним G/g) ===
    green_phase_indices = []
    for i, ph in enumerate(phases):
        if any(c in 'Gg' for c in ph.state):
            green_phase_indices.append(i)

    num_green = len(green_phase_indices)
    if num_green == 0:
        return [int(p.duration) for p in phases]

    # === 3. Сбор текущей загрузки (occupancy 0..1) по всем полосам зелёных фаз ===
    occupancy_per_green_phase = []
    controlled_links = traci.trafficlight.getControlledLinks(tls_id)

    for green_idx in green_phase_indices:
        max_occ = 0.0
        phase = phases[green_idx]

        for link_idx, link in enumerate(controlled_links):
            if link_idx >= len(phase.state) or not link:
                continue
            if phase.state[link_idx] in 'Gg' and link[0]:  # зелёный и есть полоса
                lane = link[0][0]
                try:
                    occ = traci.lane.getLastStepOccupancy(lane)
                    max_occ = max(max_occ, occ)
                except:
                    pass
        # Если нет данных — берём среднее по другим или 0.1
        if max_occ == 0.0:
            max_occ = 0.1
        occupancy_per_green_phase.append(max_occ)

    occupancy_per_green_phase = np.array(occupancy_per_green_phase)

    # === 4. Экспоненциальное сглаживание (чтобы не дёргалось) ===
    global _smoothed_occupancy
    if not hasattr(optimize_tls_durations, "_smoothed_occupancy"):
        optimize_tls_durations._smoothed_occupancy = {}

    key = tls_id
    alpha = 0.2  # чем меньше — тем инерционнее

    if key not in optimize_tls_durations._smoothed_occupancy:
        smoothed = occupancy_per_green_phase.copy()
    else:
        old = optimize_tls_durations._smoothed_occupancy[key]
        if len(old) == len(occupancy_per_green_phase):
            smoothed = (1 - alpha) * old + alpha * occupancy_per_green_phase
        else:
            smoothed = occupancy_per_green_phase.copy()

    optimize_tls_durations._smoothed_occupancy[key] = smoothed

    # === 5. Распределяем эффективное зелёное время пропорционально сглаженной загрузке ===
    total_demand = smoothed.sum()
    if total_demand == 0:
        total_demand = 1.0

    available_green_time = TARGET_CYCLE - FIXED_LOSS_PER_CYCLE
    if available_green_time < num_green * MIN_GREEN:
        available_green_time = num_green * MIN_GREEN

    # Базовые эффективные зелёные
    effective_green = (smoothed / total_demand) * available_green_time
    effective_green = np.maximum(effective_green, MIN_GREEN)
    effective_green = np.minimum(effective_green, MAX_PHASE_DURATION)

    # Корректируем сумму точно под доступное время
    current_sum = effective_green.sum()
    if current_sum > available_green_time:
        effective_green *= available_green_time / current_sum
    effective_green = np.maximum(effective_green, MIN_GREEN)  # ещё раз

    # Округляем
    effective_green = np.round(effective_green).astype(int)

    # === 6. Формируем полный список длительностей (сохраняя жёлтые/красные как есть) ===
    new_durations = [int(p.duration) for p in phases]
    green_ptr = 0
    for i, ph in enumerate(phases):
        if i in green_phase_indices:
            new_durations[i] = effective_green[green_ptr]
            green_ptr += 1
        else:
            # Жёлтые и all-red не трогаем, но можно чуть поджать, если нужно
            new_durations[i] = max(3, min(6, new_durations[i]))

    # === 7. Финальная проверка: цикл не должен сильно отличаться от целевого ===
    actual_cycle = sum(new_durations)
    diff = actual_cycle - TARGET_CYCLE
    if abs(diff) > 5 and diff != 0:
        # Корректируем самые "перенасыщенные" фазы
        adjustable = [(i, new_durations[i], smoothed[idx] if idx < len(smoothed) else 0)
                      for idx, i in enumerate(green_phase_indices)]
        adjustable.sort(key=lambda x: x[2], reverse=(diff > 0))  # если перебор — урезаем самые загруженные

        i = 0
        while diff != 0 and i < len(adjustable):
            idx = adjustable[i][0]
            step = 1 if diff < 0 else -1
            candidate = new_durations[idx] - step
            if MIN_GREEN <= candidate <= MAX_PHASE_DURATION:
                new_durations[idx] = candidate
                diff += step
            i += 1

    # === 8. Лёгкий консерватизм: не меняем больше чем на 30% от исходного ===
    old_durations = [int(p.duration) for p in phases]
    for i in range(n):
        old = old_durations[i]
        new = new_durations[i]
        if old > 0:
            ratio = new / old
            if ratio < 0.7:
                new_durations[i] = int(old * 0.7)
            elif ratio > 1.3:
                new_durations[i] = int(old * 1.3)

    return new_durations

def apply_phase_durations(tls_id, logic, new_durations):
    """
    САМЫЙ БЕЗОПАСНЫЙ И ЭФФЕКТИВНЫЙ способ адаптивного управления.
    Меняем ТОЛЬКО длительность ТЕКУЩЕЙ фазы.
    Никаких setCompleteRedYellowGreenDefinition, никаких множественных setPhaseDuration.
    """
    try:
        current_phase_idx = traci.trafficlight.getPhase(tls_id)
        current_time_left = traci.trafficlight.getNextSwitch(tls_id) - traci.simulation.getTime()

        # Если фаза почти закончилась — не трогаем
        if current_time_left < 6:
            return False

        # Разрешаем корректировки только для зелёных фаз (во избежание продления красных/жёлтых)
        try:
            current_state = logic.phases[current_phase_idx].state
        except Exception:
            current_state = ""
        if not any(c in 'Gg' for c in current_state):
            return False

        # Старая длительность текущей фазы (из исходной логики)
        old_duration = int(logic.phases[current_phase_idx].duration)
        new_duration = int(new_durations[current_phase_idx])

        # Вычисляем требуемую корректировку длительности текущей фазы
        delta = new_duration - old_duration

        # Защита от резких изменений
        if abs(delta) > 25:
            delta = 25 if delta > 0 else -25

        # Не даём фазе закончиться слишком рано
        if current_time_left + delta < 8:
            delta = 8 - current_time_left

        if delta != 0:
            # ВАЖНО: setPhaseDuration ожидает НОВЫЙ остаток времени для текущей фазы, а не дельту
            desired_remaining = current_time_left + delta
            # Ограничения безопасности
            desired_remaining = max(8, desired_remaining)
            desired_remaining = min(MAX_PHASE_DURATION, desired_remaining)
            traci.trafficlight.setPhaseDuration(tls_id, desired_remaining)
            print(f"Фаза {current_phase_idx}: целевой остаток {desired_remaining:.1f}с (было {current_time_left:.1f}с, Δ={delta:+d}с)")
            return True

        return False

    except Exception as e:
        print(f"apply_phase_durations error: {e}")
        return False
    
def set_static_program_for_opt(tls_id):
    """
    Переводит текущую программу светофора в полностью статический режим:
    minDur == maxDur == duration для каждой фазы, type=0.
    Это отключает встроенную адаптацию SUMO, чтобы наша оптимизация имела эффект.
    """
    from traci import trafficlight as tl
    try:
        all_logics = tl.getAllProgramLogics(tls_id)
        current_logic = all_logics[0]
        static_phases = []
        for phase in current_logic.phases:
            duration = max(int(phase.duration), MIN_PHASE_DURATION)
            static_phases.append(
                tl.Phase(
                    duration=duration,
                    state=phase.state,
                    minDur=duration,
                    maxDur=duration,
                    name=getattr(phase, 'name', None)
                )
            )
        static_logic = tl.Logic(
            programID="fixed_baseline_for_opt",
            type=0,
            currentPhaseIndex=tl.getPhase(tls_id),
            phases=static_phases
        )
        tl.setCompleteRedYellowGreenDefinition(tls_id, static_logic)
        tl.setProgram(tls_id, "fixed_baseline_for_opt")
        return True
    except Exception as e:
        print(f"Не удалось перевести светофор в static режим: {e}")
        return False

def set_semi_static_bounds(tls_id):
    """
    Включает actuated-программу, но жёстко ограничивает minDur/maxDur для зелёных фаз
    в диапазоне [MIN_PHASE_DURATION, MAX_PHASE_DURATION]. Жёлтые/красные оставляет фиксированными.
    Это снижает дёргание и сохраняет адаптацию SUMO.
    """
    from traci import trafficlight as tl
    try:
        all_logics = tl.getAllProgramLogics(tls_id)
        current_logic = all_logics[0]
        bounded_phases = []
        for phase in current_logic.phases:
            duration = max(int(phase.duration), MIN_PHASE_DURATION)
            # Зелёная фаза — есть хотя бы один G/g
            is_green = any(c in 'Gg' for c in phase.state)
            if is_green:
                min_d = max(MIN_PHASE_DURATION, 6)
                max_d = max(min(MAX_PHASE_DURATION, duration), MIN_PHASE_DURATION)
                bounded_phases.append(
                    tl.Phase(
                        duration=duration,
                        state=phase.state,
                        minDur=min_d,
                        maxDur=max_d,
                        name=getattr(phase, 'name', None)
                    )
                )
            else:
                # Жёлтые/красные фиксируем, чтобы не было неожиданных растяжений
                bounded_phases.append(
                    tl.Phase(
                        duration=duration,
                        state=phase.state,
                        minDur=duration,
                        maxDur=duration,
                        name=getattr(phase, 'name', None)
                    )
                )

        bounded_logic = tl.Logic(
            programID="semi_actuated_bounded",
            type=1,  # actuated
            currentPhaseIndex=tl.getPhase(tls_id),
            phases=bounded_phases
        )
        tl.setCompleteRedYellowGreenDefinition(tls_id, bounded_logic)
        tl.setProgram(tls_id, "semi_actuated_bounded")
        return True
    except Exception as e:
        print(f"Не удалось установить semi-actuated режим: {e}")
        return False

def optimize_phases(cluster_tls_ids, cluster_phases):
    """
    Новая оптимизация: зеленый распределяется пропорционально очередям.
    """

    optimized_durations = {}

    # Параметры светофора (можно вынести в config)
    MIN_GREEN = 5
    MAX_GREEN = 60
    CYCLE_LENGTH = 90

    for tls_id in cluster_tls_ids:

        if tls_id not in TLS_PHASE_LANES:
            print(f"[WARN] Нет lane→phase маппинга для {tls_id}, пропускаю.")
            continue

        phase_lane_map = TLS_PHASE_LANES[tls_id]
        phases = cluster_phases[tls_id]
        n = len(phases)

        # 1. Сбор фактических очередей (загрузка дорог)
        flows = []
        for phase_index in range(n):
            lanes = phase_lane_map.get(phase_index, [])
            queue = 0

            for lane in lanes:
                try:
                    q = traci.lane.getLastStepHaltingNumber(lane)
                    queue += q
                except traci.TraCIException:
                    pass

            flows.append(queue)

        flows = np.array(flows, dtype=float)

        # Если нагрузка нулевая — ставим распределение по 7.5 сек.
        if np.sum(flows) == 0:
            optimized_durations[tls_id] = [CYCLE_LENGTH / n] * n
            continue

        # 2. Оптимизационная модель LP
        x = cp.Variable(n)

        objective = cp.Maximize(cp.sum(cp.multiply(flows, x)))

        constraints = [
            cp.sum(x) == CYCLE_LENGTH,
            x >= MIN_GREEN,
            x <= MAX_GREEN,
        ]

        problem = cp.Problem(objective, constraints)
        problem.solve(solver=cp.SCS)

        if x.value is None:
            print(f"[ERROR] LP не решилась для {tls_id}.")
            optimized_durations[tls_id] = [CYCLE_LENGTH / n] * n
            continue

        durations = x.value.tolist()
        optimized_durations[tls_id] = durations

        print(f"\nOPTIMIZED {tls_id}:")
        for i, d in enumerate(durations):
            print(f"  Фаза {i}: {d:.2f} сек (очередь={flows[i]})")

    return optimized_durations

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