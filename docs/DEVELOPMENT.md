# Инструкции по разработке

## Требования

- Ubuntu 20.04
- Установленный [ROS Noetic](http://wiki.ros.org/noetic/Installation)
- Готовность разбираться и улучшать систему! =)
- Установленный [плагин в VSCode](https://marketplace.visualstudio.com/items?itemName=hediet.vscode-drawio) для чтения drawio диаграмм (или используйте https://app.diagrams.net/)

## Некоторые правила разработки

- Перед началом работы над задачей (созданием ветки под задачу) сделайте `git pull` в `develop` ветке
- Каждый комит должен содержать номер таски, в рамках которой делалась работа в этом комите. Пример: `#88 designed config structure and added comments`
- Имя ветки, в которой ведется работа, должна содержать номер таски. Пример: `feature/88_radar_config`
- При создании задачи указывайте проект в меню Projects
- После завершения работы над задачей создавайте Pull Request на вивание ветки в `develop`. При создании указывайте ревьюверов (как минимум ведущего, можно и остальных), проект в Projects и связанные задачи (Linked Issues)
- После апрува сливает в `develop` ведущий разработчик


## Подготовка к работе

### Установка catkin_tools

- Установите с помощью `sudo apt install python3-catkin-tools`

или

- Создайте виртуальное окружение и в него поставьте с помощью `pip install catkin-tools`

или

- Установить в user-space командой `pip3 install -U -r requirements.txt` и добавить переменную окружения `export PATH=$PATH:$HOME/.local/bin`

### Преднастройка

- Установите требуемые пакеты командой `./scripts/install_pkgs.sh`
- Установите пакеты для сборки командой `./scripts/install_third_party.sh`
- Соберите требуемые пакеты командой `./scripts/build.sh`

