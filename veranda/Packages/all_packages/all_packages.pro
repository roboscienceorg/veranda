TEMPLATE = subdirs

SUBDIRS = \
    pkg_Simulator \
    pkg_SimulatorAPI \
    pkg_DefaultRobotLoader \
    pkg_JSONLoader \
    pkg_ImageLoader \
    sensor_lidar \
    sensor_touch \
    sensor_gps \
    wheel_fixed \
    wheel_ackermann \
    wheel_swedish \
    wheel_omni \
    shape_rect \
    shape_circle \
    shape_polygon

pkg_Simulator.file = ../pkg_Simulator/simulator.pro
pkg_SimulatorAPI.file = ../pkg_SimulatorAPI/simulatorAPI.pro
pkg_DefaultRobotLoader.file = ../pkg_DefaultRobotLoader/default_robot_loader.pro
pkg_ImageLoader.file = ../pkg_ImageLoader/image_loader.pro
sensor_lidar.file = ../pkg_Sensors/lidar_sensor.pro
sensor_touch.file = ../pkg_Sensors/touch_sensor_ring.pro
sensor_gps.file = ../pkg_Sensors/gps_sensor.pro
wheel_fixed.file = ../pkg_Wheels/fixed_wheel.pro
wheel_ackermann.file = ../pkg_Wheels/ackermann_steer.pro
wheel_omni.file = ../pkg_Wheels/omni_drive.pro
wheel_swedish.file = ../pkg_Wheels/swedish_wheel.pro
shape_rect.file = ../pkg_Shapes/rectangle.pro
shape_circle.file = ../pkg_Shapes/circle.pro
shape_polygon.file = ../pkg_Shapes/polygon.pro
pkg_JSONLoader.file = ../pkg_JsonFilePlugin/json_file_plugin.pro
