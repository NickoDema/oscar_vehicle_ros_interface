### OSCAR vehicle ROS interface

Репозиторий содержит:

* oscar_vehicle_driver - ROS-обертку над [oscar_vehicle_api](https://gitlab.com/starline/oscar_vehicle_api) - низкоуровневой системой управления автомобилем, разрабатываемой в рамках проекта [OSCAR](https://gitlab.com/starline/oscar).

* oscar_teleop - пакет для управления автомобилем через oscar_vehicle_driver с помощью геймпада ps3.


#### Установка oscar_vehicle_api

```
pip install --user oscar_vehicle_api
```

Или из исходников:

```
git clone https://gitlab.com/starline/oscar_vehicle_api.git && cd oscar_vehicle_api
pip install --user -e .
```
