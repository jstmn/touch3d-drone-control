# Touch3d Drone Control


## Touch3d Driver Setup

1. Install system dependencies

``` bash
sudo apt-get install libncurses5-dev libncursesw5-dev # ncurses
sudo apt-get install freeglut3 # freeglut
sudo apt install build-essential # build-essential
sudo apt install freeglut3-dev libncurses5-dev zlib1g-dev # Needed by `openhaptics_3.4-0-developer-edition-amd64/install` script
sudo apt install libncurses5
```

1. Install the driver (see 'Driver installation guide' on the webpage below)
Webpage: https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34?language=en_US

2. Install library (see 'openhaptics_3.4-0-developer-edition-amd64/README_INSTALL')

3. Calibrate device. Run `/home/hamed/Downloads/TouchDriver2022_04_04/bin/Touch_Diagnostic`



## AirSim Installation

1. Install unity. 

Run editor: `/home/hermes/UnrealEngine-4.27/Engine/Binaries/Linux/UE4Editor ~/AirSim/Unreal/Environments/Blocks/Blocks.uproject`


``` bash
/home/hermes/UnrealEngine-4.27/GenerateProjectFiles.sh ~/AirSim/Unreal/Environments/Blocks/Blocks.uproject -game
cd ~/AirSim/Unreal/Environments/Blocks/
make Blocks
```

Cannot open Blocks.uproject:
https://github.com/microsoft/AirSim/issues/4535


## Hardware testing

The crazyswarm api used to control the drones. Documentation is here: https://crazyswarm.readthedocs.io/en/latest/api.html 

**Initial crazyflie setup**

Run the following to setup a new drone. `chooser.py` reads from `/home/hamed/crazyswarm/ros_ws/src/crazyswarm/launch/allCrazyflies.yaml` and let's users _choose_ which of the drones in that file to activate. If there is a timeout error when starting up, it's probably because this file has selected drones that are not in the workspace. A good first step is to make sure you can read the battery from each drone in the chooser menu.

``` bash 
cfclient
python3 /home/hamed/crazyswarm/ros_ws/src/crazyswarm/scripts/chooser.py
```

Note: You need to run `chooser.py` after updating the position value in `allCrazyflies.yaml`. Inside the chooser.py gui, select and unselect the drone that's starting position was changed.



**Running the system**

``` bash
roslaunch crazyswarm hover_swarm.launch # terminal 1
python3 crazyflie_interface.py  # terminal 2
make && ./main;                 # terminal 3
```

