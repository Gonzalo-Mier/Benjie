# Benjie

Benjie is a little three-wheeled differentially steered robot we made to follow an aruco marker. It uses a tm4c1294exp with the boosterpack. 

<p align="center">
  <img src="doc/imgs/benjie.png" width="350" title="Benjie robot">
</p>

Benjie use a kinect to localize himself and the objective. This kinect is placed in a structure to obtain the Top-down perspective.

<p align="center">
  <img src="doc/imgs/structure.jpg" width="350" title="Kinect in a structure">
</p>

Benjie has two ways to be controlled: using a PC controller or autonomously following an aruco marker. 


## Getting Started


### Hardware

To start the deployment, you have to place the kinect as follows:

<p align="center">
  <img src="doc/imgs/whole_structure.jpg" width="350" title="Deployment of the hardware">
</p>

In Benjie, the boosterpack is connected with tm4c1294exp in the port 2.  

###Software

#### Prerequisites

The software is split in two parts: the [benjie](src/robot_part) and [PC](src/laptop_part).

In the PC, we used Ubuntu 12.04 and ROS fuerte. 

Also, to use the controller and the kinect, we need:

```
sudo apt-get install ros-fuerte-openni-camera
sudo apt-get install ros-fuerte-openni-launch
sudo add-apt-repository ppa:grumbel/ppa
sudo apt-get update
sudo apt-get install xboxdrv
sudo apt-get install ros-fuerte-joy
``` 

For benjie, install the [Texas Instrument software](http://processors.wiki.ti.com/index.php/Creating_IoT_Solutions_with_the_TM4C1294XL_Connected_LaunchPad_Workshop?keyMatch=tiva%20connected%20launchpad%20workshop&amp;tisearch=Search-EN)


### Installing



To download the package:

```
git clone https://github.com/gonmiermu/Benjie
```

First, install the [benjie code](src/robot_part) on the benjie board using the [Texas Instrument software](http://processors.wiki.ti.com/index.php/Creating_IoT_Solutions_with_the_TM4C1294XL_Connected_LaunchPad_Workshop?keyMatch=tiva%20connected%20launchpad%20workshop&amp;tisearch=Search-EN)

Then, in the PC part, move yourself inside the src

```
cd Benjie/src
```

And compile the package using rosmake

```
rosmake laptop_part
```


## Deployment

To run the code, run first the robot part with the texas instrument software.

Then, in your laptop, open 5 terminals and execute one of each command in each terminal:

```
roslaunch openni_launch openni.launch
rosrun laptop_part kinect_aruco
rosrun laptop_part mando.py
rosrun joy joy_node
sudo xboxdrv --silent
```

To control benjie, the commands are:

Left joystick: Move forward or backward.
Right joystick : Turn.
Botón Start: Activate/Deactivate the automatic controller.
Botón X: Activate/Deactivate the manual controller.
Botón Select: Get new reference (where the aruco marker is)


## Built With

* [ROS fuerte](https://wiki.ros.org/fuerte)
* [Texas instrument board](http://processors.wiki.ti.com/index.php/Creating_IoT_Solutions_with_the_TM4C1294XL_Connected_LaunchPad_Workshop?keyMatch=tiva%20connected%20launchpad%20workshop&amp;tisearch=Search-EN)
* [ArUco](https://www.uco.es/investiga/grupos/ava/node/26)


## Authors

* **Gonzalo Mier** - *PC part* - [gonmiermu](https://github.com/gonmiermu)
* **Julio Lopez Paneque** - *Benjie part* - [JulioLP](https://github.com/JulioLP)

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* This project was made in 2016 as part of the robotic degree in the Seville University.
* The benjie code is duplicated [here](https://github.com/JulioLP/TrabajoSEPA).

 

