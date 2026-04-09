# RobotDeliver: быстрый запуск Python-скриптов

В проекте используются 2 скрипта:

- `sim_udp_to_ros_bridge.py` — принимает UDP-данные из Unity (порт `9000`) и публикует в ROS2.
- `send_cmd_vel_udp.py` — отправляет команды движения роботу в Unity (порт `9001`).

## 1) Поток данных Unity -> ROS2

Запуск:

```bash
python sim_udp_to_ros_bridge.py --listen-port 9000 --verbose
```

Проверка топиков:

```bash
ros2 topic list
ros2 topic echo /gps
ros2 topic echo /odom
ros2 topic echo /scan --once
```

## 2) Управление роботом из Python

1. В симуляции нажми `R` (режим EXTERNAL).
2. Отправь команду:

```bash
python send_cmd_vel_udp.py --linear 0.6 --angular 0.0 --duration 4
```

Поворот:

```bash
python send_cmd_vel_udp.py --linear 0.3 --angular 0.6 --duration 3
```

Стоп:

```bash
python send_cmd_vel_udp.py --linear 0 --angular 0 --duration 1
```

## Если робот не реагирует

- Проверьте, что включен режим **EXTERNAL** (клавиша `R`).
- Проверьте, что команда уходит на `127.0.0.1:9001`.
- Убедисьте, что локальный firewall не блокирует UDP.
