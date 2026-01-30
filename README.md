# Physical AI Championship — Autonomous Box Search

Автономная система управления гуманоидным роботом Unitree G1 в симуляции MuJoCo. Робот самостоятельно находит красный куб в комнате 4x4м, приближается к нему и останавливается на расстоянии <= 0.5м.

## Содержание

- [Пререквизиты](#пререквизиты)
- [Установка](#установка)
- [Сборка](#сборка)
- [Запуск](#запуск)
- [Конфигурация](#конфигурация)
- [Архитектура](#архитектура)
- [Сцена](#сцена)
- [Датчики](#датчики)
- [Алгоритм](#алгоритм)
- [Структура файлов](#структура-файлов)

---

## Пререквизиты

### Операционная система
- Ubuntu 20.04 / 22.04 / 24.04 (x86_64 или aarch64)

### Системные пакеты

```bash
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    git \
    libopencv-dev \
    libglfw3-dev \
    libboost-program-options-dev \
    libyaml-cpp-dev \
    libfmt-dev
```

### unitree_sdk2

SDK для коммуникации с роботом через DDS. Необходимо собрать и установить:

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
sudo make install -j$(nproc)
cd ../..
```

### MuJoCo

MuJoCo 3.3.6 **скачивается автоматически** при первом `cmake` симулятора. Ручная установка не требуется.

### ONNX Runtime

Библиотека для инференса RL-модели контроллера. **Уже включена в репозиторий** для обеих архитектур (x86_64 и aarch64). Ничего устанавливать не нужно.

---

## Установка

```bash
git clone https://github.com/diaskabdualiev/physical_ai_championship.git
cd physical_ai_championship
```

---

## Сборка

### Симулятор

```bash
cd simulate
mkdir -p build && cd build
cmake ..
make -j$(nproc)
cd ../..
```

При `cmake` автоматически:
- Определяется архитектура (x86_64 / aarch64)
- Скачивается MuJoCo 3.3.6 если не найден

### Контроллер

```bash
cd g1_ctrl
mkdir -p build && cd build
cmake ..
make -j$(nproc)
cd ../..
```

Архитектура для ONNX Runtime определяется автоматически.

---

## Запуск

### Автономный режим (рекомендуемый)

Один терминал, без джойстика. Робот автоматически встаёт, включает режим ходьбы и начинает поиск:

```bash
cd simulate/build
./unitree_mujoco
```

Контроллер `g1_ctrl` запускается автоматически как дочерний процесс. При закрытии симулятора контроллер завершается автоматически.

**Что происходит автоматически:**
1. Elastic band удерживает робота в воздухе
2. Пружина постепенно опускается (~4с)
3. Отправляется команда FixStand (LT+Up) — робот встаёт
4. Отправляется команда Velocity (RB+X) — режим ходьбы
5. Пружина отключается, робот стоит самостоятельно
6. AutoSearch активируется — робот ищет куб

### Ручной режим

Для отладки или использования с геймпадом. В `config.yaml` поставить:

```yaml
auto_launch: 0
use_joystick: 1
```

**Терминал 1 — симулятор:**
```bash
cd simulate/build
./unitree_mujoco
```

**Терминал 2 — контроллер:**
```bash
cd g1_ctrl/build
./g1_ctrl --network=lo
```

**Последовательность:**
1. Клавиши `8` / `7` — опустить / поднять elastic band
2. Геймпад: `LT + Up` — FixStand (робот встаёт)
3. Геймпад: `RB + X` — Velocity (режим ходьбы)
4. Клавиша `9` — отключить elastic band
5. Клавиша `0` — включить/выключить AutoSearch

### Параметры командной строки

```bash
./unitree_mujoco --help
  -r, --robot     Тип робота (g1)
  -s, --scene     Файл сцены (scene_room.xml)
  -i, --domain_id DDS domain ID (0)
  -n, --network   Сетевой интерфейс (lo)
```

---

## Конфигурация

Файл `simulate/config.yaml`:

```yaml
# Робот
robot: "g1"
robot_scene: "scene_room.xml"

# DDS
domain_id: 0
interface: "lo"

# Джойстик (0 при auto_launch=1)
use_joystick: 0
joystick_type: "xbox"
joystick_device: "/dev/input/js1"
joystick_bits: 16

# Режим запуска
auto_launch: 1              # 1 = полностью автономный (один терминал, без геймпада)
enable_elastic_band: 1       # Виртуальная пружина для поддержки робота при старте
enable_auto_search: 0        # auto_launch включает это автоматически

# Визуализация
show_camera_window: 1        # 1 = показать окно OpenCV с камерой робота
print_scene_information: 1   # Вывести информацию о модели при запуске

# Тестирование
auto_respawn: 1              # 1 = респавн робота и куба после остановки
auto_respawn_delay: 2.0      # Секунды в режиме STOPPED до респавна
```

### Описание ключевых параметров

| Параметр | Значения | Описание |
|----------|----------|----------|
| `auto_launch` | 0 / 1 | Автоматический запуск: boot sequence + g1_ctrl subprocess |
| `use_joystick` | 0 / 1 | Использовать физический геймпад. Ставить 0 при auto_launch=1 |
| `show_camera_window` | 0 / 1 | OpenCV окно с видом камеры, bounding box, состоянием и расстоянием |
| `auto_respawn` | 0 / 1 | После остановки у куба — респавн обоих в новые позиции (для тестирования) |
| `auto_respawn_delay` | секунды | Задержка перед респавном (по умолчанию 2.0с) |

---

## Архитектура

```
MuJoCo Simulator (unitree_mujoco)          g1_ctrl (subprocess)
┌──────────────────────────────┐            ┌──────────────────────┐
│ PhysicsLoop (thread 1)       │            │                      │
│  ├─ mj_step (physics)        │            │ DDS sub:             │
│  ├─ Elastic band management  │            │  LowState            │
│  └─ Offscreen head_cam render│            │  WirelessController  │
│     └─ OpenCV HSV detection  │   DDS      │                      │
│     └─ Depth → horiz dist    │───────────>│ RL velocity policy   │
│     └─ AutoSearch FSM        │            │  joystick → velocity │
│                              │            │  → joint torques     │
│ BridgeThread (thread 2)      │            │                      │
│  ├─ Boot: button injection   │<───────────│ DDS pub:             │
│  │   (FixStand, Velocity)    │   DDS      │  LowCmd (motors)     │
│  ├─ AutoSearch → joystick    │            │                      │
│  ├─ LowCmd → mj ctrl        │            └──────────────────────┘
│  └─ Sensors → LowState pub  │
└──────────────────────────────┘
```

`g1_ctrl` не знает, что управляется алгоритмом. AutoSearch генерирует виртуальные joystick-значения (`ly` — вперёд, `rx` — поворот), которые передаются через DDS точно так же, как от настоящего геймпада.

---

## Сцена

Файл: `unitree_robots/g1/scene_room.xml`

- **Комната:** 4x4м, квадрат, ровный пол, статичные стены, статичное освещение
- **Куб:** 0.3x0.3x0.3м, красный (`rgba="0.8 0.2 0.2 1"`), free joint, рандомная позиция при каждом запуске
- **Робот:** Unitree G1 (29 DOF) в центре комнаты, случайная ориентация (yaw) при каждом запуске

---

## Датчики

| Датчик | Реализация | Назначение |
|--------|-----------|------------|
| RGB-камера | `head_cam`, offscreen render 320x240, FOV 75° | Обнаружение куба через HSV-фильтрацию |
| LiDAR | `rangefinder` на head_link + depth buffer камеры | Измерение расстояния до куба |
| IMU | `imu_quat`, `imu_gyro`, `imu_acc` | Баланс и локомоция (используется g1_ctrl) |

**Ground-truth позиция куба не используется.** Всё определяется только через камеру.

---

## Алгоритм

### Конечный автомат AutoSearch

```
SEARCHING ──(куб виден)──> CENTERING ──(куб по центру)──> APPROACHING ──(dist ≤ 0.5м)──> STOPPED
    ^                         |                               |
    └─────────────────────────┘                               |
         (куб потерян)            <───────────────────────────┘
                                      (auto_respawn → reset)
```

| Состояние | ly (вперёд) | rx (поворот) | Условие перехода |
|-----------|------------|-------------|------------------|
| SEARCHING | 0.0 | 1.0 (вращение) | Куб обнаружен в кадре |
| CENTERING | 0.0 | ±1.0 (к кубу) | \|offset\| < 15% от центра |
| APPROACHING | 0.7 (вперёд) | 0.0 | Горизонт. дистанция ≤ 0.5м |
| STOPPED | 0.0 | 0.0 | auto_respawn → новый цикл |

### Визуальное обнаружение

1. Offscreen рендер `head_cam` → RGB 320x240 + depth buffer
2. RGB → BGR → HSV
3. Красный фильтр (H: 0-10 и 170-180, S: 80-255, V: 80-255)
4. Морфологическая очистка (open + close)
5. `findContours` → наибольший контур → bounding box
6. Offset от центра кадра → направление поворота

### Оценка расстояния

Камера наклонена вниз (~56° от горизонтали). Depth buffer даёт расстояние вдоль оптической оси, а не горизонтальную дистанцию. Выполняется пересчёт:

1. По позиции пикселя куба и FOV (75°) → направление луча в системе координат камеры
2. Трансформация луча в систему координат тела через матрицу поворота камеры
3. Горизонтальная проекция (X, Y) → реальное расстояние на уровне земли
4. Остановка при горизонтальной дистанции ≤ 0.5м

---

## Структура файлов

```
physical_ai_championship/
├── README.md                           # Этот файл
├── APPROACH.md                         # Краткое описание подхода (1 страница)
├── simulate/
│   ├── config.yaml                     # Конфигурация симулятора
│   ├── CMakeLists.txt                  # Сборка (auto-download MuJoCo, auto-detect arch)
│   └── src/
│       ├── main.cc                     # Физика, offscreen рендер, boot sequence, respawn
│       ├── auto_search.h              # AutoSearch: HSV детекция + FSM + boot sequence
│       ├── unitree_sdk2_bridge.h      # DDS мост: button injection, joystick override
│       └── param.h                    # Параметры из config.yaml
├── g1_ctrl/
│   ├── CMakeLists.txt                  # Сборка контроллера (auto-detect arch)
│   ├── config/                         # RL policy конфиги и веса
│   └── src/                            # Исходники контроллера
├── unitree_robots/
│   └── g1/
│       ├── g1_29dof.xml               # Модель робота (head_cam, LiDAR, IMU)
│       ├── scene_room.xml             # Сцена: комната 4x4м + красный куб
│       └── meshes/                    # 3D-модели робота
└── .gitignore
```

---

## Окно отладки (OpenCV)

При `show_camera_window: 1` открывается окно с видом камеры робота:

- **Зелёная рамка** — bounding box обнаруженного куба
- **Статус**: SEARCHING / CENTERING / APPROACHING / STOPPED
- **depth** — расстояние от камеры до куба (по лучу)
- **horiz** — горизонтальная дистанция до куба (на уровне земли)

---

## Клавиши управления (в окне MuJoCo)

| Клавиша | Действие |
|---------|----------|
| `0` | Вкл/выкл AutoSearch |
| `7` / `Up` | Поднять elastic band |
| `8` / `Down` | Опустить elastic band |
| `9` | Вкл/выкл elastic band |
| `Backspace` | Сброс симуляции |
