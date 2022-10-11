#!/usr/bin/env python3

from __future__ import print_function

import odrive
from odrive.enums import *

import fibre.libfibre

odrv0 = odrive.find_any()

# motor config
odrv0.axis0.motor.config.current_lim = 25

# controller
odrv0.axis0.controller.config.vel_limit = 100    # turn/s

odrv0.axis0.trap_traj.config.vel_limit = 75	# slighty lower than controller config
odrv0.axis0.trap_traj.config.accel_limit = 25
odrv0.axis0.trap_traj.config.decel_limit = 25
odrv0.axis0.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ

# gpio config
odrv0.config.gpio6_mode = GPIO_MODE_DIGITAL
odrv0.axis0.min_endstop.config.gpio_num = 6
odrv0.axis0.min_endstop.config.is_active_high = False
odrv0.axis0.min_endstop.config.offset = 5
odrv0.axis0.min_endstop.config.enabled = True
odrv0.config.gpio6_mode = GPIO_MODE_DIGITAL_PULL_UP

odrv0.config.gpio7_mode = GPIO_MODE_DIGITAL
odrv0.axis0.max_endstop.config.gpio_num = 7
odrv0.axis0.max_endstop.config.is_active_high = False
odrv0.axis0.max_endstop.config.enabled = True
odrv0.config.gpio7_mode = GPIO_MODE_DIGITAL_PULL_UP

# homing setup
odrv0.axis0.controller.config.homing_speed = 5
odrv0.axis0.config.startup_homing = True

# save
try:
    print("Saving configuration...")
    odrv0.save_configuration()
except fibre.libfibre.ObjectLostError:
    print("Rebooting ODrive...")
