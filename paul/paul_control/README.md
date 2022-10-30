# paul_control

This packages configures the controllers for the propulsion and elevation systems. It includes the launch files required to start the ODrive's hardware interface in the correct mode.

## Folders

- config:

    The *config* folder includes the files defining the controllers. 

- launch:

    The *launch* folder contains a few files that can be used to control the motors.

- src:

    The *src* folder contains scripts used to test the system and implement the service server to movve the elevation system.

## ODrive and Motor Calibration

Before using the ODrives, be sure to configure them properly with the following commands. You will need to use the **odrivetool** in order to run them.

You will also need to find the ODrive's ID in order to congiure the *propulsion.yaml* and *elevation.yaml* files.

### Elevation System

```bash
# reset config (this will reboot the odrive)
odrv0.erase_configuration()

# brake resistance config (required to absorb back current)
odrv0.config.enable_brake_resistor = True
odrv0.config.brake_resistance = 0.5     # 50WR5J provided resistor
odrv0.config.max_regen_current = 3.0
odrv0.config.dc_max_negative_current = -10.0

# encoder index to avoid calibration on each reboot
odrv0.axis0.encoder.config.use_index = True

odrv0.clear_errors()

# start motor and encoder calibration
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

# save parameters for next calibration
odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis0.config.startup_encoder_index_search = True

# motor config
odrv0.axis0.motor.config.current_lim = 20

# controller
odrv0.axis0.controller.config.vel_limit = 75    # turn/s

odrv0.axis0.trap_traj.config.vel_limit = 50	# slighty lower than controller config
odrv0.axis0.trap_traj.config.accel_limit = 20
odrv0.axis0.trap_traj.config.decel_limit = 20
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
odrv0.axis0.config.startup_homing = False

# save config
odrv0.save_configuration()
```

### Propulsion System

```bash
# reset config (this will reboot the odrive)
odrv0.erase_configuration()

# brake resistance config (required to absorb back current)
odrv0.config.enable_brake_resistor = True
odrv0.config.brake_resistance = 0.5     # 50WR5J provided resistor
odrv0.config.max_regen_current = 3.0
odrv0.config.dc_max_negative_current = -10.0

# encoder index to avoid calibration on each reboot
odrv0.axis0.encoder.config.use_index = True
odrv0.axis1.encoder.config.use_index = True

odrv0.clear_errors()

# start motor and encoder calibration
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

# save parameters for next calibration
odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis0.config.startup_encoder_index_search = True

odrv0.axis1.encoder.config.pre_calibrated = True
odrv0.axis1.motor.config.pre_calibrated = True
odrv0.axis1.config.startup_encoder_index_search = True

# motor
odrv0.axis0.motor.config.current_lim = 20
odrv0.axis1.motor.config.current_lim = 20

# controller
odrv0.axis0.controller.config.vel_limit = 50    # turn/s
odrv0.axis0.controller.config.vel_ramp_rate = 25    # turn/s^2
odrv0.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

odrv0.axis1.controller.config.vel_limit = 50    # turn/s
odrv0.axis1.controller.config.vel_ramp_rate = 25    # turn/s^2
odrv0.axis1.controller.config.input_mode = INPUT_MODE_VEL_RAMP
odrv0.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

# save config
odrv0.save_configuration()
```