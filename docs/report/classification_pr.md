# Precision / Recall по категориям (официальная метрика)

> Источник: запечатанные матрицы обоих движков; исключения (REVIEW/MANUAL) — штатный поток нештатных случаев, отдельная строка, не C/D.

## isaac_nominal  (routine 66, exceptions 0)

| cat | precision | recall | tp | fp | fn | → exceptions |
|---|---|---|---|---|---|---|
| B | 1.0 | 1.0 | 18 | 0 | 0 | 0 |
| C | 1.0 | 1.0 | 18 | 0 | 0 | 0 |
| D | 1.0 | 1.0 | 30 | 0 | 0 | 0 |

## isaac_all_runs  (routine 150, exceptions 8)

| cat | precision | recall | tp | fp | fn | → exceptions |
|---|---|---|---|---|---|---|
| B | 1.0 | 0.8889 | 40 | 0 | 5 | 0 |
| C | 0.9756 | 0.9756 | 40 | 1 | 1 | 6 |
| D | 0.9275 | 1.0 | 64 | 5 | 0 | 2 |

## twin_nominal  (routine 87, exceptions 1)

| cat | precision | recall | tp | fp | fn | → exceptions |
|---|---|---|---|---|---|---|
| B | 1.0 | 0.9583 | 23 | 0 | 1 | 0 |
| C | 1.0 | 1.0 | 23 | 0 | 0 | 1 |
| D | 0.9756 | 1.0 | 40 | 1 | 0 | 0 |

## twin_all_runs  (routine 231, exceptions 4)

| cat | precision | recall | tp | fp | fn | → exceptions |
|---|---|---|---|---|---|---|
| B | 1.0 | 0.6962 | 55 | 0 | 24 | 2 |
| C | 0.8594 | 1.0 | 55 | 9 | 0 | 2 |
| D | 0.8661 | 1.0 | 97 | 15 | 0 | 0 |
