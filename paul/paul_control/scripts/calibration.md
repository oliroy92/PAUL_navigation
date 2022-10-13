# Motor Calibration
## Instructions

Be sure to run the following commands in **NO LOAD**. 

```bash
# reset config (this will reboot the odrive)
odrv0.erase_configuration()

# brake resistance config (required to absorb back current)
odrv0.config.enable_brake_resistor = True
odrv0.config.brake_resistance = 0.5     # 50WR5J provided resistor
odrv0.config.max_regen_current = 10.0
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

# save config
odrv0.save_configuration()
```

For the propulsion system, be sure to run the following commands too:

```bash
# encoder index to avoid calibration on each reboot
odrv0.axis1.encoder.config.use_index = True

# start motor and encoder calibration
odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

# save parameters for next calibration
odrv0.axis1.encoder.config.pre_calibrated = True
odrv0.axis1.motor.config.pre_calibrated = True
odrv0.axis1.config.startup_encoder_index_search = True

# save config
odrv0.save_configuration()
```