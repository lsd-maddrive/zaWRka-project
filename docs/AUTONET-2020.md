# Информация из репозитория команды MadDrive соревнований "Autonet 18+"

## 1. Установка

- Создать папку для рабочего пространства, например `catkin_ws`, и перейти в нее
<!-- - Клонировать репозиторий используя --recursive для стягивания сабмодулей `git clone --recursive <this URL>` -->
- Клонировать репозиторий `git clone <this URL>`
- Установить зависимости `./install_pkgs.sh`
<!-- - Установить зависимости для `scan_tools`: -->
<!-- `rosdep install --from-paths src --ignore-src --rosdistro melodic -r -y` -->
<!-- 5. (Для JTX) Установить зависимости `./install_jtx.sh` -->
- Установить python модули `pip install -r requirements_<version>.txt`, если используется 3й или не системный второй

<!-- > Не забывайте использовать `git submodule update --init` вместе с `git pull` для обновления сабмодулей -->
> Дополнительно требуется PyTorch, для x86 можно ставить через `pip install torch==1.4.0`, для JTX [отсюда](https://forums.developer.nvidia.com/t/pytorch-for-jetson-version-1-6-0-now-available/72048).

## 2. Сборка

Используйте `./catkin_build.sh` для сборки

## 3. Запуск на реальном роботе (в процессе)

* `start.sh` - запуск решения

> Log-файлы и Bag-файлы пишутся в директорию `$HOME/catkin_ws/src/autonet18p2020_MadDrive/data`

## 4. Запуск симуляции

`roslaunch wr8_software test_solver.launch`

Возможные аргументы:
- `world_minor_version` - вариант расположения знаков, светофоров, препятствий на скоростном участке и на парковке (от 1 до 6)
- `gz_gui` - запустить gzserver или нет, по умолчанию `false`
- `render_maze` - нарисовать solver maze или нет, по умолчанию `false`

[![Пример](https://img.youtube.com/vi/6wBE3FdzJu8/0.jpg)](https://www.youtube.com/watch?v=6wBE3FdzJu8)

## Важные ссылки

1. [Новый регламент (от 01.08.2020)](https://www.russianrobotics.ru/upload/iblock/a53/a53577af9e259e163d2cd3d45ee3626c.pdf)
2. [Самое актуальное на данный момент техническое описание робота](https://www.russianrobotics.ru/upload/iblock/877/8772baf746d238a53bf6523686eadab9.pdf)
3. [Телеграм-канал](https://t.me/joinchat/BaWT7kh4KOXaCqr8kIvlEw)
4. [Github репозиторий с примерами кода](https://github.com/ulstu/autonet18plus)
5. [rosbag файл](https://yadi.sk/d/APE45bWswA_2UA)
6. [Вебинар](https://www.youtube.com/watch?v=-LLab6G_zPk&feature=youtu.be)


## Информация с [официального github репозитория](https://github.com/ulstu/autonet18plus)
<details><summary>CLICK ME</summary>


### Публикация сообщений для движения робота

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[1.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
```

Телеуправление роботом можно осуществить с использованием библиотеки [teleop_twist_keyboard](https://github.com/ros-teleop/teleop_twist_keyboard).

Для запуска перемещения робота используется топик 'cmd_vel'. Сообщения передаются в формате [Twist](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html), имеющим два свойства linear и angular с типом [Vector3](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Vector3.html), соответствующие линейной и угловой скорости. 

Т.к. используется робот с обычной автомобильной кинематикой, то можно устанавливать свойство x для линейной скорости и свойство z для угловой. Значения для linear.x и angular.z должны быть в интервале [-1;1]:
* -1 для линейной скорости - это движение назад с максимальной скоростью, ограниченной программными средствами (~9 км/ч);
* 1 для линейной скорости - движение вперед с максимальной скоростью;
* 0 для линейной скорости - остановка робота;
* -1 для угловой скорости - поворот колес на максимально допустимый угол вправо;
* 1 для угловой скорости - поворот колес на максимально допустимый угол влево;
* 0 для угловой скорости - колеса направлены вдоль оси робота (движение вперед).


Пример публикации сообщений типа Twist для [python](https://www.programcreek.com/python/example/70251/geometry_msgs.msg.Twist) и для [C++](https://clearpathrobotics.com/blog/2014/09/ros-101-creating-node/). Необходимо помнить про названия топиков (в текущем решении это 'cmd_vel').

### Изображения с камеры

Изображения с камеры публикуются в формате [Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html) в два топика: image_top и image_bottom:
* image_top это изображение с камеры, которая "смотрит" на дорожные знаки и светофор.
* image_bottom это изображение с камеры, которая "смотрит" на линии дорожной разметки.

Для преобразования сообщения Image в тип изображения OpenCV используется библиотка [cv_bridge](http://wiki.ros.org/cv_bridge). Ее особенность в том, что она не всегда стабильно работает на python3, при этом на python2 проблем не возникает. Версии python конкретного файла можно указывать в заголовке каждого python скрипта (shebang):

Python2:
```python
#!/usr/bin/python
# -*- coding: utf-8 -*-
```

Python3:
```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
```
Пример на [C++](http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages) и на [Python](http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber)

### Кнопки запуска

На роботе предусмотрено две кнопки: зеленая для подачи команды на старт выполнения роботом задания и красная для экстренной остановки робота.

Участникам понадобится только зеленая кнопка, она двухпозиционная. При ее включении генерируется сообщение с текстом 'START' в топике 'cmd' и при ее отключении генерируется сообщение 'STOP' в этом же топике.

Робот не начнет движение ни при каких условиях, пока не нажата зеленая кнопка. Участникам соревнований необходимо предусмотреть средства для запуска своих алгоритмов при нажатии на зеленую кнопку. Пример есть в файле move_node.py.
