# config.py: Константы и настройки
import os
# Путь к SUMO_HOME (установите в переменных окружения или здесь)
SUMO_HOME = os.environ.get('SUMO_HOME', '/Users/maksim/Sumo/2025-11-01-12-22-54/venv/lib/python3.9/site-packages/sumo') # Путь к pip установке eclipse-sumo
# Параметры симуляции
SIM_STEPS = 3600 # 1 час симуляции (секунды)
OPTIMIZE_INTERVAL = 300 # Оптимизация каждые 2 мин (шаги)
GUI = True # Запускать с GUI (True) или без (False)
# Файлы SUMO
NET_FILE = "./osm.net.xml.gz"
SUMOCFG_FILE = "./osm.sumocfg"
# Параметры оптимизации
MIN_PHASE_DURATION = 10
MAX_PHASE_DURATION = 45
CYCLE_TIME = 120 # Общий цикл светофора (сек)
# Близость для near-miss (m)
PROXIMITY_THRESHOLD = 30 # Фильтр dist для эффективности
TLS_ID = "cluster_1781062862_1781062863_1853581175_1853581176_#2more"