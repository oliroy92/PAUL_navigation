#!/usr/bin/env python3

from __future__ import print_function

import odrive
from odrive.enums import *

import fibre.libfibre

odrv0 = odrive.find_any()

# odrive
odrv0.config.enable_brake_resistor = True
odrv0.config.max_regen_current = 3.0
odrv0.config.dc_max_negative_current = -10.0

# encoder config
odrv0.axis0.config.startup_encoder_index_search = True
odrv0.axis0.config.startup_encoder_offset_calibration = True
odrv0.axis0.config.startup_motor_calibration = True

# motor
odrv0.axis0.motor.config.current_lim = 25


# controller
odrv0.axis0.controller.config.vel_limit = 100    # turn/s

odrv0.axis0.trap_traj.config.vel_limit = 75	# slighty lower than controller config
odrv0.axis0.trap_traj.config.accel_limit = 25
odrv0.axis0.trap_traj.config.decel_limit = 25
odrv0.axis0.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ


try:
    print("Saving configuration...")
    odrv0.save_configuration()
except fibre.libfibre.ObjectLostError:
    print("Rebooting ODrive...")
