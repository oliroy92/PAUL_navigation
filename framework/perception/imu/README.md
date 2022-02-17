# IMU

La documentation pour faire l'installation, la calibration et le fonctionnement du package ROS avec l'IMU BNO055 est disponible sur l'adresse suivante:
https://github.com/RoboticArts/ros_imu_bno055

## Launch files

Pour lancer le noeud ROS de l'IMU, lancer la commande suivante:
```bash
roslaunch imu imu.launch
```

La calibration de l'IMU peut être faite en lançant la commande suivante:
```bash
roslaunch ros_imu_bno055 imu_calibration.launch
```

Pour visualiser l'IMU dans RVIZ:
```bash
roslaunch ros_imu_bno055 view_imu.launch
```

## Paramètres

Il est possible d'utiliser certains arguments:

| Argument | Default value | Description |
| ------ | ------ | ------ |
| serial_port | /dev/ttyUSB0 | USB port where the IMU is connected (using a USB Serial Converter ) |
| frame_id | imu_link | Name of the link that the tf will use |
| operation_mode | IMU |  Type of sensory fusion used by the IMU. The next section will explain each mode in detail. |
| use_magnetometer | false | Enables topic imu/magnetometer |
| use_temperature | false | Enables topic imu/temperature |

Voici les différents modes d'opération:

| Operation mode | Description |
| ------ | ------ |
| IMU | The relative orientation of the BNO055 in space is calculated from the accelerometer and gyroscope data. |
| COMPASS | This mode is intended to measure the magnetic earth field and calculate the geographic direction. The heading  can only be calculated when considering gravity and magnetic field at the same time  |
| M4G | Similar to the IMU mode, but instead of using the gyroscope signal to detect rotation, the changing orientation of the magnetometer in the magnetic field is used. |
| NDOF_FMC_OFF | This fusion mode is same as NDOF mode, but with the Fast Magnetometer Calibration turned ‘OFF’.  |
| NDOF | The absolute orientation data is calculated from accelerometer, gyroscope and the magnetometer.  In this mode the Fast Magnetometer calibration is  turned  ON  and  thereby  resulting  in  quick  calibration  of  the  magnetometer |