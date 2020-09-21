# LocalizationFusionLibrary
In order to be able to run the LocalizationFusionLibrary as a ROS node do the following:

1. Follow the guide below to install Ubuntu 18.04 in WSL and MobaXterm to open GUIs
2. Create a catkin workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
``` 
3. Clone this library in /catkin_ws/src
#### Directory Structure
Parts of the code are organized as follows. Motivation is that we can develop them seperately.

```
src
└── motion_model                       # contains the base motion class and all its specific variations 
    ├── design                          
    ├── include                         
    ├── src                             
    ├── test                            
    └── CMakeLists.txt                  
└──kalman_filter                       # contains a FilterWraper, KalmanFilterBase and all filter variations
    ├── design                          
    ├── include                         
    ├── src                             
    ├── test                            
    └── CMakeLists.txt                  
└── meassurement                       # contains files related to measurements and their timekeeping
    ├── design                          
    ├── include                         
    ├── src                             
    ├── test                            
    └── CMakeLists.txt    
```
## How to make ROS Kinetic/Melodic work on Windows 10

This approach uses Windows Subsystems for Linux, WSL for short.

1. Install Ubnutu 18.04 LTS (for ROS Melodic) or Ubnutu 16.04 LTS (for ROS Kinetic) via WSL in Windows. 
2. Open the installed Ubuntu distro you just installed and install ROS on it
3. Install MobaXterm by following the following [article](https://medium.com/@lixis630/extra-setup-on-wsl-for-ros-7c539463370a)
4. Install [VSCode](https://code.visualstudio.com/download) and the [Remote-WSL](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-wsl) plugin.

### Update bashrc
Add the following lines to your ~/.bashrc

```bash
export DISPLAY=192.168.0.183:0.0 # or whatever you read on MobaXterm
alias open='explorer.exe .'
cd ~ 
```
To try out if it works, open MobaXterm and minimize it. In a Linux terminal give ```xterm```. If a graphical window shows up, everythingis working. Now we could even open up other GUIs such as RVIZ or RQT.

### Troubleshooting
If faced with error messages related to D-bus read this [article](https://x410.dev/cookbook/wsl/setting-up-wsl-for-linux-gui-apps/)

If faced with error related to Segmentation error **Segmentation fault (core dumped)** 
add the following to your ~/.bashrc 
```bash
export LIBGL_ALWAYS_INDIRECT=
```
The solution was found [here](https://github.com/ros-visualization/rviz/issues/1438)

With the help of [this](https://www.digitalocean.com/community/tutorials/how-to-read-and-set-environmental-and-shell-variables-on-a-linux-vps)
 
## How to make Windows Terminal behave like Terminator from Ubuntu
Open Windows Terminal and press Ctrl+, and copy the setting in [profile.json](https://gitlab.iavgroup.local/-/snippets/177)
