# Touch3d Drone Control


## Installation



**Touch3d Driver**
Webpage: https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34?language=en_US

``` bash
sudo apt-get install libncurses5-dev libncursesw5-dev # ncurses
sudo apt-get install freeglut3 # freeglut
sudo apt install build-essential # build-essential
sudo apt install freeglut3-dev libncurses5-dev zlib1g-dev # Needed by `openhaptics_3.4-0-developer-edition-amd64/install` script
sudo apt install libncurses5


```
1. Install the driver (see 'Driver installation guide' on the webpage)
2. Install library (see 'openhaptics_3.4-0-developer-edition-amd64/README_INSTALL')

**AirSim**

1. Install unity. 

Run editor: `/home/hermes/UnrealEngine-4.27/Engine/Binaries/Linux/UE4Editor ~/AirSim/Unreal/Environments/Blocks/Blocks.uproject`


``` bash
/home/hermes/UnrealEngine-4.27/GenerateProjectFiles.sh ~/AirSim/Unreal/Environments/Blocks/Blocks.uproject -game
cd ~/AirSim/Unreal/Environments/Blocks/
make Blocks
```

Cannot open Blocks.uproject:
https://github.com/microsoft/AirSim/issues/4535


**crazyswarm**

https://crazyswarm.readthedocs.io/en/latest/api.html


