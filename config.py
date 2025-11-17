# config.py: Константы и настройки
import os

# Путь к SUMO_HOME (для Python-инструментов SUMO, не сам бинарник)
# Если у тебя уже настроено SUMO_HOME в системе – можно оставить так:
SUMO_HOME = os.environ.get(
    'SUMO_HOME',
    '/Users/danilvlasuk/Desktop/diplom-demo/venv/lib/python3.9/site-packages/sumo'
)

# Параметры симуляции
SIM_STEPS = 3600          # 1 час симуляции (секунды)
OPTIMIZE_INTERVAL = 300   # Оптимизация каждые 5 мин (шаги)
GUI = True                # True = sumo-gui, False = sumo (без окна)

# Файлы SUMO (твоя сеть/конфиг)
NET_FILE = "./osm.net.xml.gz"
SUMOCFG_FILE = "./osm.sumocfg"

# Параметры оптимизации светофора
MIN_PHASE_DURATION = 5
MAX_PHASE_DURATION = 60
CYCLE_TIME = 120          # Общий цикл светофора (сек)

# Близость для near-miss (m)
PROXIMITY_THRESHOLD = 50  # Фильтр dist для эффективности

# Каталоги для результатов KPI
OUT_BASELINE_DIR = os.path.join("out", "baseline")
OUT_OPT_DIR = os.path.join("out", "opt")

# Имена файлов, которые будет писать SUMO
TRIPINFO_FILE = "tripinfos.xml"
SUMMARY_FILE = "summary.xml"
LANEDATA_FILE = "laneData.xml"  # Если настроишь вывод meandata-lane, можно использовать
