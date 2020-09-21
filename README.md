# LocalizationFusionLibrary
In order to be able to run the LocalizationFusionLibrary as a ROS node do the following:

1. Follow the [guide](#guide) below to install Ubuntu 18.04 with ROS melodic in WSL and MobaXterm to open GUIs
2. Create a catkin workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
git clone ...   # Clone the library in /catkin_ws/src
cp -r path/to/library/localiztionfusionlibrary  ~/catkin_ws/src/ # or copy
catkin_make # Build the ros package
```
3. In order to start the node on a test bag do
```bash
cd ~/catkin_ws/
source devel/setup.bash
roslaunch LocalizationFusionLibrary filter_ros_node.launch 
```
## Tips and tricks
In order to get autocompletion capabilities for ROS related code and to add VSCode shortcuts for building and running add the folder and files as following:
```
LocalizationFusionLibrary
├── ...
└── .vscode
    ├── c_cpp_properties.json
    └── settings.json
```
After following [a](#a.-enable-autocomplition-for-ROS-in-C++) and [b](#b.-shortcuts-for-build-and-run), the workflow looks like:
1. Open library with vscode: ```code ~/catkin_ws/src/localizationfusionlibrary ```
2. Build the ROS package: **Ctrl** + **Shift** + **B**
3. Roslaunch a test bag and the filternode: **Ctrl** + **Shift** + **3**


### a. Enable autocomplition for ROS in C++
##### c_cpp_properties.json
```json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c11",
            "cppStandard": "c++14",
            "intelliSenseMode": "gcc-x64",
            "compileCommands": "~/catkin_ws/build/compile_commands.json"
        }
    ],
    "version": 4
}
```

### b. Shortcuts for build and run
Add shortcuts to build and run the roslaunch file in file->preferences->Keyboard Shortcuts( or **Ctrl** + **KS**) and then 
1. Set **Tasks: Run Build Task** with **Ctrl**+**Shift**+**B**
2. Set **Tasks: Run Test Task** with **Ctrl**+**Shift**+**3**

##### tasks.json
```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "catkin_make",
            "type": "shell",
            "command": "catkin_make",
            "args": [
                "-j4",
                "-DCMAKE_BUILD_TYPE=Release",
                "-DCMAKE_EXPORT_COMPILE_COMMANDS=1",
                "-DCMAKE_CXX_STANDARD=14",
                "-C=../../"
            ],
            "problemMatcher": [],
            "group": {
                "kind": "build",
                "isDefault": true
            }
        },
        {
            "label": "launch_test_1",
            "type": "shell",
            "command": "source ../../devel/setup.bash; roslaunch LocalizationFusionLibrary filter_ros_node.launch",
            "problemMatcher": [],
            "group": {
                "kind": "test",
                "isDefault": true
            }
        }
    ]
}
```
### Directory Structure
The directory structure should look like this at the end:
```
catkin_ws
├── build
├── devel
└── src
    └── LocalizationFusionLibrary
        ├── ...
        ├── .vscode
            ├── c_cpp_properties.json
            └── settings.json
        ├── package.xml
        └── ros_srcs
            ├── ...
            ├── package.xml
            └── CMakeLists.txt
   
```
## GUIDE
### How to make ROS Kinetic/Melodic work on Windows 10

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

1. Split vertically: **Ctrl**+**Shift**+**B**
2. Split horizontally: **Ctrl**+**Shift**+**3**
3. Close split: **Ctrl**+**W**
4. Move around splits: **Alt** + **up**/**down**/**left**/**right**
