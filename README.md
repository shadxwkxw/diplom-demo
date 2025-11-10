## Создание директорий для разделения результатов с оптимизацией и без нее
```
mkdir -p out/baseline out/opt
```
## Запуск скрипта без оптимизации
```
python3 main.py --mode baseline
for f in tripinfos.xml stats.xml summary.xml edgeData.xml laneData.xml tlslog.xml tls_changes.csv tls_observed.csv risk_trend.png; do
  [ -f "$f" ] && mv -f "$f" out/baseline/;
done
```
## Запуск скрипта с оптимизацией
```
python3 main.py --mode opt
for f in tripinfos.xml stats.xml summary.xml edgeData.xml laneData.xml tlslog.xml tls_changes.csv tls_observed.csv risk_trend.png; do
  [ -f "$f" ] && mv -f "$f" out/opt/;
done
```
## Сравнение результатов
```
python3 analyze_kpi.py
```