# config.py: Константы и настройки
import os
# Путь к SUMO_HOME (установите в переменных окружения или здесь)
SUMO_HOME = os.environ.get('SUMO_HOME', '/Users/maksim/Sumo/2025-11-01-12-22-54/venv/lib/python3.9/site-packages/sumo') # Путь к pip установке eclipse-sumo
# Параметры симуляции
SIM_STEPS = 3600 # 1 час симуляции (секунды)
OPTIMIZE_INTERVAL = 300 # Оптимизация каждые 5 мин (шаги)
GUI = True # Запускать с GUI (True) или без (False)
# Файлы SUMO
NET_FILE = "./osm.net.xml.gz"
SUMOCFG_FILE = "./osm.sumocfg"
# Параметры оптимизации
MIN_PHASE_DURATION = 5
MAX_PHASE_DURATION = 60
CYCLE_TIME = 120 # Общий цикл светофора (сек)
# Близость для near-miss (m)
PROXIMITY_THRESHOLD = 50 # Фильтр dist для эффективности