# __User Guide for the repository TM5-900_ROS__

This repository is made for the project Dynamics of Intelligent Robots and Vehicle of the course held by the Professor Andrea Bonci
The authors are Nicola Misurati, Laura Piccini and Valerio Procaccioli.

## Prerequisites
[Ubuntu] 18.04.6 LTS (Bionic Beaver) running; for this project it was necessary to create a virtual machine using [Oracle VM VirtualBox]. It is raccomended to give the VM at least half the RAM available and to reserve about 20 GB for the virtual hard disk. [Guest Additions] are also required. 

### ROS Installation
[Ros Melodic] is the version required based on the Linux release selected.
The steps for the installation of ROS are:

Setup the _sources.list_ so that the VM can accept software from _packages.ros.org_ and to setup the keys:
```sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
```
sudo apt install curl 
```
```
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
After checking that the package information are updated from all of the configured sources with:
```
sudo apt update && sudo apt upgrade
```
is it recommended to install the _Desktop-Full Install_ version launching the command: 
```
sudo apt install ros-melodic-desktop-full
```
A best practice to manually source ROS environment variables every time a new shell is launched is by automatically sourcing them using the terminal core code:
```
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

The ROS enviroment is based on workspaces and packages; to create and manage them there are tools that permits this and much more, the main dependencies can be installed using:
```
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```
Before utilizing ROS tools, a command-line tool for installing system dependencies is needed. Its name is [rosdep] and is installed and initialized using:
```
sudo apt install python-rosdep && sudo rosdep init
rosdep update
```

To check that the ROS environment is loaded and its variables like _ROS_ROOT_ and _ROS_PACKAGE_PATH_ are correctly set we can launch:
```
printenv | grep ROS
```

### ROS Managing
Any ROS enviroment need a special folder where it is possible to modify, build, and install catkin packages. Its name is [catkin_workspace], that it is created and built using:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
The first building of the _catkin_ workspace, will create a _devel_ and a _build_ folder and a _CMakeLists.txt_ file in the previous created _src_ folder.

To use the packages contained inside this custom workspace, it is needed to manually source this workspace on top of the environment by using:
```
source devel/setup.bash
```

### MoveIt Installation
[MoveIt] is the most widely used software for manipulation, and to incorporate it into our project it was installed from source and not from pre-built binaries.
To speed up the compiling time of MoveIt it was installed a compiler cache named [ccache], particularly suitable for the [gcc] compiler.
The installation is straightforward:
```
sudo apt install ccache
```
and the compiler cache has to be enabled on startup in bash:
```
echo 'export PATH=/usr/lib/ccache:$PATH' >> $HOME/.bashrc
source $HOME/.bashrc
```

The source code has to be downloaded into a ROS workspace, that can be created separately if necessary, using:
```
wstool init src
wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
wstool update -t src
rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Finally, it is possible to build MoveIt simply using:
```
catkin build
```

### Gazebo Setup
[Gazebo] is one of the more popular open-source 3D robotics simulator. Gazebo is already downloaded as a pre-build Ubuntu debian when the ROS _Desktop-Full Install_ version is installed, but an important step is to achieve ROS integration with the stand-alone simulator.

A set of ROS packages named _gazebo_ros_pkgs_ provides wrappers to provide the necessary interfaces to simulate a robot using ROS messages, services and dynamic reconfiguration. The best method to follow is to install them from source.

In the chosen catkin workspace, it is needed to clone the GitHub repository using:
```
cd ~/catkin_ws/src
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b melodic-devel
```
It is recommended to check for any missing dependencies using _rosdep_:
```
rosdep update
rosdep check --from-paths . --ignore-src --rosdistro kinetic
```
and to install them via:
```
rosdep install --from-paths . --ignore-src --rosdistro melodic 
```
To build the Gazebo ROS integration packages, has to run the following commands:
```
cd ~/catkin_ws/
catkin_make
```

## Simulation and Control Phase
This chapter describes the files needed to implement the TM5-900 on ROS, simulate its movements on Gazebo and control it via MoveIt.

Part of the repository needed is forked by the official [TechmanRobotInc] GitHub repository, used to connect TM Robot’s operating software (TMflow) with external program and transfer the control of the robot in between based on users need. In this case, instead of connecting with the real robot, these GitHub files are modified to simulate the TM5-900 and its movements on Gazebo and then a trajectory can be planned and executed by bringing up MoveIt environment in simulation mode with virtual TM Robot.

The first step is to manually install the robot controllers using the command:

```
sudo apt-get install ros-melodic-joint-state-controller
```
and 
```
sudo apt-get install ros-melodic-joint-trajectory-controller
```

Another prerequisite are the [industrial_core] packages, that provide nodes and libraries for communication with industrial robot controllers.
The installation is made launching:
```
sudo apt-get install ros-melodic-industrial-core
```

The manipulator can be simulated on Gazebo and then a trajectory can be planned on MoveIt and actuated on Gazebo. This is possible because ROS allows communication between different nodes, as long as [roscore] is running. Roscore is a collection of nodes and programs that are pre-requisites of a ROS-based system.

Therefore in the first terminal you have to launch:
```
roscore
```
then, to launch the robot simulation on Gazebo, on another terminal:
```
roslaunch tm5_900_moveit_config tm5_900_gazebo.launch
```
It loads Gazebo with the _TM5-900_ in an empty world and it starts the controllers. 

On a third terminal the MoveIt simulation can be started by launching: 
```
roslaunch tm5_900_moveit_config tm5_900_moveit_planning_execution.launch
```

A notable information to add is that into the _tm5_900_gazebo.launch_ file was necessary to add this line to solve an error:
```
<rosparam file=“($find tm5 900 moveit config)/config/gazebo ros control params.yaml” command=“load” / >
```

The _Motion Planning_ plugin on MoveIt allows, among other things, to plan and execute a trajectory indicating an initial and a final state. The possible states are defined, and editable, in the _SRDF_ file.

The movement planned on MoveIt it will be executed also on Gazebo, as if it was the real physical robot.

### Python Script for Smart Simulation
A Python script was written to perform the above commands automatically and then we used the Python package MoveIt commander that offers wrappers for the functionality provided in MoveIt.

Using some functions provided by these package, the script plan on MoveIt and execute on Gazebo first a trajectory from the state _home_ to the state _ready1_, then back to _home_. 

Next it adds a box on MoveIt to simulate an obstacle, and again the robot goes to the _ready1_ state. 
The trajectory will be different, because the MoveIt controllers detect the box like a collision object and the trajectory is planned to avoid it. 

Lastly the robot goes back to the _home_ state and the simulation is completed.

### Kinect Camera Modelling on Gazebo and MoveIt
The aim of this part of the project was to create a Gazebo model that includes a ROS depth camera plugin to view the depth camera’s output in RViz. 

Kinect is a line of motion sensing input devices manufactured by Microsoft and first released in 2010. This device typically includes an RGB camera and infrared projector and detector that maps depth by structured light or flight time calculations. Among other features, it performs real-time gesture recognition and skeletal detection.

Gazebo and ROS are separate projects that do not depend on each other. The sensors from the
_gazebo models_ repository (such as depth cameras) do not include ROS plugins by default.
Because Gazebo and ROS are separate projects that do not depend on each other, sensors from the gazebo_models repository (such as depth cameras) do not include ROS plugins by default. 

The first step consists in making a custom camera based from the ones defined on the Gazebo model repository, and then add the new <plugin> tag to make the depth camera data publish point clouds and images to ROS topics.

The Kinect sensor is packed from the gazebo_models repository and it is downloadable [here]. The _kinect_ folder has to be copied in the _~/.gazebo/models_ directory and immediately afterwards the model’s name has to be changed and updated. 

To accomplish that it has to be updated:
- the folder name
- the _<name>_ stored in the _.config_ file
- the model name in the _model.sdf_ file.

The next step is to add the _Openni Kinect ROS_ plugin to publish depth camera information and output to ROS topics. 

The _model.sdf_ file in the new model’s directory is modified by adding the SDF markup listed below inside the _<sensor>_ tag, immediately after the closing _</camera>_ tag.

```
 <plugin name="camera_plugin" filename="libgazebo_ros_openni_kinect.so">
          <baseline>0.2</baseline>
          <alwaysOn>true</alwaysOn>
          <!-- Keep this zero, update_rate in the parent <sensor> tag
            will control the frame rate. -->
          <updateRate>0.0</updateRate>
          <cameraName>camera_ir</cameraName>
          <imageTopicName>/camera/color/image_raw</imageTopicName>
          <cameraInfoTopicName>/camera/color/camera_info</cameraInfoTopicName>
          <depthImageTopicName>/camera/depth/image_raw</depthImageTopicName>
          <depthImageCameraInfoTopicName>/camera/depth/camera_info</depthImageCameraInfoTopicName>
          <pointCloudTopicName>/camera/depth/points</pointCloudTopicName>
          <frameName>camera_link</frameName>
          <pointCloudCutoff>0.5</pointCloudCutoff>
          <pointCloudCutoffMax>3.0</pointCloudCutoffMax>
          <distortionK1>0</distortionK1>
          <distortionK2>0</distortionK2>
          <distortionK3>0</distortionK3>
          <distortionT1>0</distortionT1>
          <distortionT2>0</distortionT2>
          <CxPrime>0</CxPrime>
          <Cx>0</Cx>
          <Cy>0</Cy>
          <focalLength>0</focalLength>
          <hackBaseline>0</hackBaseline>
        </plugin>
```
By default, the Kinect is not a static object in Gazebo but it is possible to further edit the _.sdf_ to add the line _<static> true </static>_, which will allow the camera to float in the air.

When the camera is in the Gazebo scene, it should be publishing images and point clouds to ROS topics. 
It is possibile to check the topics that are being published by running in a new terminal:
```
rostopic list
```

#### Gazebo2RViz
Another part of the project was installing a particular package containing nodes to automatically import all entities simulated in a ROS-enabled Gazebo or described in a set of SDF files into RViz through the TF and Marker plugin.

Furthermore, it adds objects from an _.SDF_ file as MoveIt collision objects.

The commands to import this are:
```
git clone https://github.com/andreasBihlmaier/gazebo2rviz
```

### FAQ and Common Errors
The management of errors that may be encountered during development is fundamental for the
correct realization of the project. 
Often in fact, the research and resolution of errors is a complex process, especially in the robotic field where hardware and physics put the sticks between wheels to those looking to perform software tests and updates. 

Below there a list of the main errors that have occurred together with the best practices to avoid or resolve them:
-  ```
   [Err] [REST.cc:205] Error in REST request
   libcurl: (51) SSL: no alternative certificate subject name matches target host name 'api.ignitionfuel.org'
    ```
    The error was due to the _∼/.ignition/fuel/config.yaml_ file that pointed to the old ignition fuel domain. It can be solved by replacing in the same file the url: _https://api.ignitionfuel.org_ with the url: _https://api.ignitionrobotics.org_
    &nbsp;    
- ```
  [WARN] [1638864622]: Skipping virtual joint "FixedBase" because its child frame "base" does not match the URDF frame "world"
   ```
  Solved this error by modifying in _∼/catkin ws/src/tm5_900_moveit_config/config_ the virtual joint line as: 
    ```
  <virtual joint name= "FixedBase" type= "fixed" parent frame= "base link" child link= "world" />
    ```
    &nbsp;
- ``` 
  ERROR: cannot launch node of type [robot_state_publisher/state_publisher]: robot_state_publisher
  ``` 
  In the _tm5 900_moveit_planning_execution.launch_ file and in the _tm5_900_gazebo.launch_ file there are two nodes with the same name and function (_robot_state_publisher_).
  The node has to be commented on in either one of the two files to work out the issue.
   &nbsp;
- ``` 
  “No p gain specified for pid. Namespace: /gazebo_ros_control/pid_gains/shoulder_1_joint ...”
  ``` 
  It’s not really an error. In fact, if PID parameters were found, Gazebo will use PID controllers in ROS to control the joints; otherwise, the joints will be controlled with Gazebo methods.
  
   [Ubuntu]: <https://releases.ubuntu.com/18.04/>
   [Oracle VM VirtualBox]: <https://www.virtualbox.org/>
   [Guest Additions]: <https://www.virtualbox.org/manual/ch04.html>
   [ROS Melodic]: <http://wiki.ros.org/melodic>
   [MoveIt!]: <https://moveit.ros.org/>
   [Gazebo]: <https://gazebosim.org/>
   [Industrial_Core]: <http://wiki.ros.org/industrial_core>
   [TechmanRobotInc]: <https://github.com/TechmanRobotInc/tmr_ros1>
   [rosdep]: <http://wiki.ros.org/rosdep>
   [catkin_workspace]: <http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment>
   [ccache]: <https://ccache.dev/>
   [gcc]: <https://gcc.gnu.org/>
   [roscore]:<http://wiki.ros.org/roscore>
   [industrial_core]:<http://wiki.ros.org/industrial_core>
   [here]:<http://github.com/osrf/gazebo_tutorials/raw/master/ros_depth_camera/files/kinect.zip>
