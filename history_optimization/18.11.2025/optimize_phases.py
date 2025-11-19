def optimize_phases(near_miss_count, avg_risk, current_logic, tls_id):
    """Оптимизация фаз с cvxpy (MPC), с зависимым objective"""
    global _program_counter
    _program_counter += 1
    
    num_phases = len(current_logic.phases)
    durations = cp.Variable(num_phases, nonneg=True)
   
    constraints = [durations >= MIN_PHASE_DURATION, durations <= MAX_PHASE_DURATION, cp.sum(durations) == CYCLE_TIME]
   
    # Исправленный objective: Делаем зависимым от durations. Пример: weights per phase (симулируем риск по направлениям)
    # В реальности: Соберите phase-specific risk из данных SUMO
    phase_weights = np.linspace(1, 1.5, num_phases) # Больше веса для "опасных" фаз (e.g., повороты)
    # Избегаем депрекейтнутого умножения матриц: используем elementwise multiply
    delay_estimate = cp.sum(cp.multiply(phase_weights, durations)) # Weighted delay approx
    risk_penalty = avg_risk * cp.sum(durations) # Всегда CYCLE_TIME, но для баланса
    objective = cp.Minimize(0.5 * delay_estimate + 0.5 * risk_penalty + near_miss_count) # + const для минимизации
   
    problem = cp.Problem(objective, constraints)
    problem.solve()
   
    if problem.status != cp.OPTIMAL:
        print("Optimization failed, using current durations")
        return [phase.duration for phase in current_logic.phases]
   
    # Округляем до целых, корректируем сумму до CYCLE_TIME
    new_durations = [int(round(d)) for d in durations.value]
    total = sum(new_durations)
    if total != CYCLE_TIME:
        diff = CYCLE_TIME - total
        # Простая коррекция: распределяем разницу по фазам, не выходя за MIN/MAX
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
   
    # Копируем полные phases с новыми durations
    new_phases = []
    for i, phase in enumerate(current_logic.phases):
        # Жестко фиксируем фактическую длительность: minDur=maxDur=duration
        new_phases.append(traci.trafficlight.Phase(new_durations[i], phase.state, new_durations[i], new_durations[i], phase.name))
   
    # Create new program with unique ID to trigger tlslog.xml writing
    new_program_id = f"opt_{_program_counter}"
    new_logic = traci.trafficlight.Logic(new_program_id, current_logic.type, current_logic.currentPhaseIndex, phases=new_phases)
   
    try:
        # Обновляем полное описание программы
        traci.trafficlight.setCompleteRedYellowGreenDefinition(tls_id, new_logic)
        # Активируем новую программу (это вызовет запись в tlslog.xml)
        try:
            traci.trafficlight.setProgram(tls_id, new_program_id)
        except traci.TraCIException:
            pass
        # Перезапускаем цикл с первой фазы, чтобы новые длительности применились немедленно
        try:
            traci.trafficlight.setPhase(tls_id, 0)
        except traci.TraCIException:
            pass
    except traci.TraCIException as e:
        print(f"TraCI error: {e}")
   
    return new_durations