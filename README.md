# Projekt inżynierski - Roman Nowak
## System detekcji przeszkód z wykorzystaniem kamery zdarzeniowej

Promotor: dr Tomasz Kryjak / mgr inż. Krzysztof Błachut

Link do projektu na Overleaf:
https://www.overleaf.com/read/vdfshvbztfcv#d6c1d8

Część pisemna w Latex, wykonywana na bazie szablonu:
https://www.overleaf.com/read/bwztjqvbrvys#158783

## Konfiguracja

Kod przeznczony jest do działania na Ubuntu 22.04

Całość należy skonfigurować tak jak pokazano w instrukcji zawartej w Readme.md repozytorium https://github.com/ARK-Electronics/ROS2_PX4_Offboard_Example.

Dodatkowo konieczne będzie:
```
pip install numpy opencv-python empy==3.3.4 scikit-learn sort-tracker-py
GZ_SIM_RESOURCE_PATH=~/PX4-Autopilot/Tools/simulation/gz/models:~/PX4-Autopilot/Tools/simulation/gz/worlds
```

Należy także zainstalować:
 - narzędzie v2e oraz jego wymagania zgodnie z https://github.com/SensorsINI/v2e
 - bibliotekę gazebo transport https://gazebosim.org/api/transport/14/installation.html
 
## Sposób użycia
Po uruchomieniu nowego terminala

```
source DVS_Detekcja_przeszkod_Roman_Nowak/kod/gazabo_simulation/ros2_ws/install/setup.bash
```
Po każdych zmianach w kodzie

```
colcon build --packages-select drone_sim
```

W celu uruchomienia symulacji
```
ros2 launch drone_sim drone_sim.launch.py
```


## Wykorzystane repozytoria

https://github.com/ARK-Electronics/ROS2_PX4_Offboard_Example

https://github.com/uzh-rpg/rpg_dvs_ros

https://github.com/HBPNeurorobotics/gazebo_dvs_plugin

https://github.com/SensorsINI/v2e


