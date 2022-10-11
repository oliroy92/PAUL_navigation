#!/usr/bin/env python3

from __future__ import print_function

import odrive
from odrive.enums import *

import fibre.libfibre

odrv0 = odrive.find_any()

# motor
odrv0.axis0.motor.config.current_lim = 25
odrv0.axis1.motor.config.current_lim = 25

# controller
odrv0.axis0.controller.config.vel_limit = 50    # turn/s
odrv0.axis0.controller.config.vel_ramp_rate = 100    # turn/s^2
odrv0.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP

odrv0.axis1.controller.config.vel_limit = 50    # turn/s
odrv0.axis1.controller.config.vel_ramp_rate = 100    # turn/s^2
odrv0.axis1.controller.config.input_mode = INPUT_MODE_VEL_RAMP

try:
    print("Saving configuration...")
    odrv0.save_configuration()
except fibre.libfibre.ObjectLostError:
    print("Rebooting ODrive...")
