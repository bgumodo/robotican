#!/usr/bin/env python

from dynamic_reconfigure.parameter_generator_catkin import *

PACKAGE = 'robotican_hardware_interface'

gen = ParameterGenerator()

gen.add("motor_joint_name", str_t, 0,"Motor joint name", "left_wheel_joint")
gen.add("motor_lpf_hz", int_t, 0, "Left motor lpf loop hz", 30, 10, 1000)
gen.add("motor_lpf_alpha", double_t, 0, "Left motor lpf alpha", 0.7, 0.0, 1.0)
gen.add("motor_pid_hz", int_t, 0, "Left motor pid loop hz", 1000, 1000, 2000)
gen.add("motor_kp", double_t, 0, "Left motor kp", 3.0, 0.0, 100.0)
gen.add("motor_ki", double_t, 0, "Left motor kp", 10.0, 0.0, 100.0)
gen.add("motor_kd", double_t, 0, "Left motor kp", 0.0, 0.0, 100.0)

exit(gen.generate(PACKAGE, "robotican_hardware_interface_node", "RiCBoard"))
