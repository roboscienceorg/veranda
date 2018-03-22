TEMPLATE = subdirs

SUBDIRS = \
    pkg_Simulator \
    pkg_SimulatorAPI \
    pkg_DefaultRobotLoader \
    pkg_ImageLoader \
    sensor_lidar \
    sensor_touch \
    wheel_fixed \
    wheel_ackermann \
    shape_rect \
    shape_circle \
    shape_polygon

pkg_Simulator.file = ../pkg_Simulator/simulator.pro
pkg_SimulatorAPI.file = ../pkg_SimulatorAPI/simulatorAPI.pro
pkg_DefaultRobotLoader.file = ../pkg_DefaultRobotLoader/default_robot_loader.pro
pkg_ImageLoader.file = ../pkg_ImageLoader/image_loader.pro
sensor_lidar.file = ../pkg_Sensors/lidar_sensor.pro
sensor_touch.file = ../pkg_Sensors/touch_sensor_ring.pro
wheel_fixed.file = ../pkg_Wheels/fixed_wheel.pro
wheel_ackermann.file = ../pkg_Wheels/ackermann_steer.pro
shape_rect.file = ../pkg_Shapes/rectangle.pro
shape_circle.file = ../pkg_Shapes/circle.pro
shape_polygon.file = ../pkg_Shapes/polygon.pro
