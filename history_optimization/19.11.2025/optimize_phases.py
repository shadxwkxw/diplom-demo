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