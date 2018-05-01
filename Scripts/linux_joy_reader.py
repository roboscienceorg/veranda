# Taken from Github Gist https://gist.github.com/rdb/8864666/

# Released by rdb under the Unlicense (unlicense.org)
# Based on information from:
# https://www.kernel.org/doc/Documentation/input/joystick-api.txt

import os, struct, array, sys
from fcntl import ioctl
from multiprocessing import Process, Queue

import rclpy
from rclpy.node import Node

from sdsmt_simulator.SimTimer import SimTimer

from sensor_msgs.msg import Joy

# These constants were borrowed from linux/input.h
axis_names = {
    0x00 : 'x',
    0x01 : 'y',
    0x02 : 'z',
    0x03 : 'rx',
    0x04 : 'ry',
    0x05 : 'rz',
    0x06 : 'trottle',
    0x07 : 'rudder',
    0x08 : 'wheel',
    0x09 : 'gas',
    0x0a : 'brake',
    0x10 : 'hat0x',
    0x11 : 'hat0y',
    0x12 : 'hat1x',
    0x13 : 'hat1y',
    0x14 : 'hat2x',
    0x15 : 'hat2y',
    0x16 : 'hat3x',
    0x17 : 'hat3y',
    0x18 : 'pressure',
    0x19 : 'distance',
    0x1a : 'tilt_x',
    0x1b : 'tilt_y',
    0x1c : 'tool_width',
    0x20 : 'volume',
    0x28 : 'misc',
}

button_names = {
    0x120 : 'trigger',
    0x121 : 'thumb',
    0x122 : 'thumb2',
    0x123 : 'top',
    0x124 : 'top2',
    0x125 : 'pinkie',
    0x126 : 'base',
    0x127 : 'base2',
    0x128 : 'base3',
    0x129 : 'base4',
    0x12a : 'base5',
    0x12b : 'base6',
    0x12f : 'dead',
    0x130 : 'a',
    0x131 : 'b',
    0x132 : 'c',
    0x133 : 'x',
    0x134 : 'y',
    0x135 : 'z',
    0x136 : 'tl',
    0x137 : 'tr',
    0x138 : 'tl2',
    0x139 : 'tr2',
    0x13a : 'select',
    0x13b : 'start',
    0x13c : 'mode',
    0x13d : 'thumbl',
    0x13e : 'thumbr',

    0x220 : 'dpad_up',
    0x221 : 'dpad_down',
    0x222 : 'dpad_left',
    0x223 : 'dpad_right',

    # XBox 360 controller uses these codes.
    0x2c0 : 'dpad_left',
    0x2c1 : 'dpad_right',
    0x2c2 : 'dpad_up',
    0x2c3 : 'dpad_down',
}

def readJoystick(jsdev, q):
    while True:
        evbuf = jsdev.read(8)
        if evbuf:
            q.put(evbuf)

def processJoystick(q, button_states, axis_states, button_map, axis_map):
    while q.qsize() > 0:
        evbuf = q.get()
        time, value, type, number = struct.unpack('IhBB', evbuf)

        #if type & 0x80:
             #print("(initial)"),

        if type & 0x01:
            button = button_map[number]
            if button:
                button_states[button] = value
                if value:
                    print("%s pressed" % (button))
                else:
                    print("%s released" % (button))

        if type & 0x02:
            axis = axis_map[number]
            if axis:
                fvalue = value / 32767.0
                axis_states[axis] = fvalue
                print("%s: %.3f" % (axis, fvalue))

def formatJoystickMessage(button_states, axis_states):
    msg = Joy()

    msg.axes.append(0)
    msg.axes.append(0)

    if "x" in axis_states:
        msg.axes[0] = axis_states["x"]

    if "ry" in axis_states:
        msg.axes[1] = -axis_states["ry"]

    return msg

def main():
    # Iterate over the joystick devices.
    print('Available devices:')

    for fn in os.listdir('/dev/input'):
        if fn.startswith('js'):
            print('\t/dev/input/%s' % (fn))

    # We'll store the states here.
    axis_states = {}
    button_states = {}

    axis_map = []
    button_map = []

    args = sys.argv
    joyname = ""
    if len(args) > 1:
        joyname = args[1]
    else:
        sys.exit()

    # Open the joystick device.
    fn = '/dev/input/' + joyname
    print('Opening %s...' % fn)
    jsdev = open(fn, 'rb')

    # Get the device name.
    #buf = bytearray(63)
    buf = array.array('b', [0] * 64)
    ioctl(jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
    js_name = buf.tostring()
    print('Device name: %s' % js_name)

    # Get number of axes and buttons.
    buf = array.array('B', [0])
    ioctl(jsdev, 0x80016a11, buf) # JSIOCGAXES
    num_axes = buf[0]

    buf = array.array('B', [0])
    ioctl(jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
    num_buttons = buf[0]

    # Get the axis map.
    buf = array.array('B', [0] * 0x40)
    ioctl(jsdev, 0x80406a32, buf) # JSIOCGAXMAP

    for axis in buf[:num_axes]:
        axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
        axis_map.append(axis_name)
        axis_states[axis_name] = 0.0

    # Get the button map.
    buf = array.array('H', [0] * 200)
    ioctl(jsdev, 0x80406a34, buf) # JSIOCGBTNMAP

    for btn in buf[:num_buttons]:
        btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
        button_map.append(btn_name)
        button_states[btn_name] = 0

    print('%d axes found: %s' % (num_axes, ', '.join(axis_map)))
    print('%d buttons found: %s' % (num_buttons, ', '.join(button_map)))

    # Spin child process to read the joystick
    # with blocking reads
    events = Queue()
    t = Process(target=readJoystick, args=(jsdev, events))
    t.start()

    rclpy.init()
    node = Node("joystick_" + joyname)

    pubjoy = node.create_publisher(Joy, joyname + '/joystick')

    simTime = SimTimer(False, "sdsmt_simulator/timestamp", node)
    
    # Tick time at 10 hz
    dt = 0.1

    def cb():
        processJoystick(events, button_states, axis_states, button_map, axis_map)
        msg = formatJoystickMessage(button_states, axis_states)
        pubjoy.publish(msg)

    simTime.create_timer(dt, cb)

    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()

    t.terminate()

if __name__ == '__main__':
    main()