# MadDrive project

## How to start (preinstallation)

1. Clone repo with `git clone --recursive https://github.com/lsd-maddrive/zaWRka-project.git` (to fetch submodules)
2. Install all dependencies with `install_pkgs.sh`
3. Install all python modules with `requirements.txt` (`pip install -r requirements.txt`)

> Don't forget to do `git submodule update --init` along with `git pull` to get updates of submodules 

## Requirements

- [ROS Melodic](http://wiki.ros.org/melodic)
- Python 2.7.x

## Contents of repository

- `wr8_description` - описание моделей робота для симуляции в Gazebo
- `wr8_ai` - ПО для распознавания разметки и знаков (на доработке)
- `wr8_gui_server` - backend (серверная часть) для коммуникации с фронтом на Android (закрыто - переименова package.xml)
- `wr8_msgs` - свои типы сообщений и сервисов 
- `wr8_software` - основные скрипты и launch-файлы проекта

Дополнительные пакеты, добавленные как подмодули:
- [ydlidar](https://github.com/EAIBOT/ydlidar) - драйвер для используемого лидара
- [hector_slam](http://wiki.ros.org/hector_slam) - пакет метода hector_slam со всеми сопутствующими
- [teb_local_planner](http://wiki.ros.org/teb_local_planner) - локальный планнер, который рассчитывает локальный маршрут с учетом минимального радиуса поворота (то ,что требуется для автомобилей)
- [rosserial](http://wiki.ros.org/rosserial) - пакет коммуникации по последовательному интерфейсу (Serial)

> Стягиваются и собираются они по причине совместимости или наличия в репозиториях (некоторых пакетов нет, а некоторые не работают при скачивании через `apt`).

Остальные:
- `controller_wr_driver` - firmware для контроллера WR, stack: ChibiOS for STM32
- `math_task` - решение математической задачи по программирования с Autonet2019
- `install_pkgs.sh` - скрипт установки и скачивания всех необходимых зависимостей
- `requirements.txt` - список Python пакетов для работы

> Все остальные файлы и папки не требуются для работы или будут удалены в скором времени

## Принцип подмены симуляцией

В основе данного принципа лежит возможность работы как с машинкой, так и с симулятором не изменяя основной составляющих и каналов базового стека:

<!-- Must be `uc` instead of `open` in link! -->
<p align="center">
<img src="https://drive.google.com/uc?id=1yt5R27NiCiPxm_0cHMFDlkMnkKW3JOjO">
</p>

## Start notes

Наиболее важные места:
- [wr8_description/launch](wr8_description/launch) - основные скрипты  системы симуляции: загрузка и запуск модели, запуск контроллера `ackermann_controller`
- [wr8_description/config](wr8_description/config) - конфигурация контроллера `ackermann_controller`
- [wr8_description/rviz](wr8_description/rviz) - настроенные параметры Rviz для просмотра модели
- [wr8_description/urdf](wr8_description/urdf) - описание модели zaWRka для симуляции
- [wr8_description/worlds](wr8_description/worlds) - "миры" для Gazebo, описание карт, в которых может производиться симуляция
---
- [wr8_software/launch](wr8_software/launch) - все скрипты, которые решают различные задачи и используются как в симуляции, так и на реальном роботе (по принципу подмены симуляцией)
    - [rviz_localization_view](wr8_software/launch/rviz_localization_view.launch), [rviz_slam_view](wr8_software/launch/rviz_slam_view.launch) - скрипты для просмотра информации в Rviz при решении задач SLAM/локализации
    - [keyboard_control](wr8_software/launch/keyboard_control.launch) - управление роботом с клавиатуры
    - [debug_launch](wr8_software/launch/debug_launch.launch), [base_launch](wr8_software/launch/base_launch.launch) - запуск всех систем на реальном роботе в рабочем и отладочном режиме (разделено на два скрипта для сохранения настройки рабочего режима)
    - [slam](wr8_software/launch/slam.launch) - запуск узлов для управления роботом и решения задачи SLAM (не включает в себя подключение к роботу/запуск симулятора)
    - [localization](wr8_software/launch/localization.launch) - запуск узлов для управления роботом и решения задачи локализации на известной карте (не включает в себя подключение к роботу/запуск симулятора)
- [wr8_software/launch/gazebo](wr8_software/launch/gazebo) - скрипты для запуска модели с симуляцией для решения задач SLAM и локализации
- [wr8_software/launch/base](wr8_software/launch/base) - более "базовые" скрипты, которые используются как в симуляции, так и на реальном роботе (по принципу подмены симуляцией)
- [wr8_software/config](wr8_software/config) - конфигурации [move_base](http://wiki.ros.org/move_base) для стека навигации (более подробно: http://wiki.ros.org/navigation/Tutorials/RobotSetup)
- [wr8_software/calib](wr8_software/calib) - калибровочные данные камер
- [wr8_software/maps](wr8_software/maps) - готовые карты, которые использует [map_server](http://wiki.ros.org/map_server) для представления и решения задачи локализации
- [wr8_software/rviz](wr8_software/rviz) - сохраненные параметры представлений Rviz
- [wr8_software/scripts](wr8_software/scripts) - папка со скриптами для Python

## How to work on kinetic?

- `teb_local_planner` must be checkouted to `kinetic-devel`

## How to work on real robot?

1) На целевой машине (заварке) запустите (через `source`) скрипт `setup_me_master.sh`.
2) На инструментальной машине (компьютере) запустите (через `source`) скрипт `setup_nuc_master.sh` (если работаете на NUC).

3) Запустите основные компоненты скриптом `wr8_software/base_start.launch`, это запускает:
    - аргумент `uc` - связь с контроллером
    - аргумент `lidar` - драйвер лидара
    - аргумент `solver` - решатель лабиринта (применяется только на соревнованиях)
    - аргумент `gui_server` - сервер для GUI на Android (не отлажен полностью)
    - аргумент `camera_s` - конкретно настроенная камера для знаков (применяется только на соревнованиях)
    - аргумент `camera_r` - конкретно настроенная камера для дороги (применяется только на соревнованиях)
    - аргумент `bag_record` - запись данных топиков в bag-файл

## Some information

Проект под командной разработкой, направленный на участие в ежегодных соревнованиях Profest AutoNet 18+ (18 - 20 марта 2020 г.Москва) 

Некоторые [правила оформления](https://github.com/serykhelena/AutoNetChallenge/blob/develop/controller_wr_driver/docs/dev_rules.md)

[Регламент соревнований 2019/2020](http://russianrobotics.ru/upload/iblock/039/039d37ea649e49ed2f50210e415bdd6c.pdf)

[Форум, посвящённый вопросам по регламенту](http://russianrobotics.ru/competition/autonet/autonet-18/)

[Список участников](https://docs.google.com/spreadsheets/d/e/2PACX-1vQQ2zzrAAbFCBXrUEgEfghzuqSvDOwywB9XMI6uXnDfj5rw4qsn_r54UXMksgU4Eq0onv_xA9ydmw2O/pubhtml?gid=363203216&single=true)
