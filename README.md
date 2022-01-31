# paul_ros

Répertoire principal contenant les différents packages ROS pour le projet. On y retrouve présentement les fichiers pour l'environnement de simulation, pour le modèle du robot et le lancement de ceux-ci.

## Installation

L'installation se fait facilement à partir du code source. Avant de commencer, assurez-vous d'installer les packages requis.

```bash
sudo apt install ros-melodic-realsense2-description ros-melodic-realsense2-camera ros-melodic-rtabmap-ros ros-melodic-move-base ros-melodic-rviz-imu-plugin ros-melodic-rplidar-ros
```

Par la suite, vous pouvez procéder à la création et à la compilation de votre environnement catkin.

1. Création d'un workspace:

    ```bash
    mkdir -p catkin_ws/src
    ```

2. Copie du répertoire:

    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/pmc-paul/paul_ros.git
    ```

3. Téléchargement du plugin de la Realsense D435 pour la simulation:

    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/pal-robotics/realsense_gazebo_plugin.git
    ```

4. Téléchargement et configuration de ros_kortex pour contrôler le bras Kinova:

    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/Kinovarobotics/ros_kortex.git
    cd ros_kortex
    git checkout melodic-devel
    cd ../..
    rosdep install --from-paths src --ignore-src -y
    sudo python3 -m pip install conan
    conan config set general.revisions_enabled=1
    conan profile new default --detect > /dev/null
    conan profile update settings.compiler.libcxx=libstdc++11 default
    ```

5. Installation de l'IMU (BNO055):
    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/RoboticArts/ros_imu_bno055.git
    python3 -m pip install pyserial rospkg
    sudo adduser $USER dialout # Redémarrer si la communication USB ne marche pas
    ```
6. Compilation des packages:

    ```bash
    cd ~/catkin_ws
    catkin_make
    ```

Si aucune erreur a lieu lors de la compilation, vous pouvez passer à la prochaine étape.

## Utilisation

Le package *paul_bringup* est responsable de lancer les différents mode du robot, autant pour le robot réel qu'en simulation. Présentement, un seul launch file est disponible. 

Dans un nouveau terminal, effectuez les commandes suivantes:

```bash
cd catkin_ws/
source devel/setup.bash
```

Par la suite, lancez le programme avec:

```bash
roslaunch paul_bringup paul.launch
```

### Paramètres

Plusieurs paramètres sont disponibles pour modifier le lancement dans le launch file. Les principaux sont listés ci-dessous.

- simulation (par défaut: false): 

    Permet de lancer le robot en simulation ou non. La simulation lance Gazebo et simule les données des capteurs.

- gazebo_gui (par défaut: true): 

    Paramètre seulement important si en simulation. Permet à l'usager d'ouvrir le GUI de Gazebo ou non lors du lancement. Utile pour débugger.

- rviz (par défaut: true): 

    Par défaut, RViz est ouvert autant pour la simulation que pour le robot réel. C'est un outil très utile pour visualiser les données, mais il est possible de le désactiver en le mettant à false.

- with_d435 (par défaut: true): 

    Paramètre seulement utile pour le robot réel. Il est possible de ne pas utiliser la caméra en mettant le paramètre à false. La D435 va toutefois rester sur le modèle du robot, sans publier de données.

Ainsi, quelqu'un désirant lancer la simulation sans le GUI de Gazebo peut le faire avec la commande suivante:

```bash
roslaunch paul_bringup paul.launch simulation:=true gazebo_gui:=false
```

## TODO

- [x] Commencer la documentation
- [ ] Ajouter des éléments à la TODO
