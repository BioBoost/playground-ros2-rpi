# Running ROS2 on the Raspberry Pi 4

Using Ubuntu Server 20.04 (as recommended). We can later try 22.04.

Used following tutorials:

* [https://roboticsbackend.com/install-ros2-on-raspberry-pi/](https://roboticsbackend.com/install-ros2-on-raspberry-pi/)

May need to remove the `unattended-upgrades` package. It just keeps updating in the background and locking apt.

```bash
sudo apt remove unattended-upgrades
```

## What's Next

Learning to write a Node:

* [https://roboticsbackend.com/write-minimal-ros2-python-node/](https://roboticsbackend.com/write-minimal-ros2-python-node/)
* [https://roboticsbackend.com/write-minimal-ros2-cpp-node/](https://roboticsbackend.com/write-minimal-ros2-cpp-node/)

Let's start with the C++ one :).

Before starting you should create a package for the node: [https://roboticsbackend.com/create-a-ros2-cpp-package/](https://roboticsbackend.com/create-a-ros2-cpp-package/)

**Good tip! Don't use dashes in the package name !**

### Hello Name

Just go into the workspace and execute the build command:

```bash
cd playground-ros2-rpi
colcon build
colcon build --packages-select hello_name
source ./install/setup.bash
ros2 run hello_name hello_name_node
```

**Package 'hello_name' not found ?** Important. Don't forget to source the bash script after creating/compiling (yes every time) a package.

### Going Further

There is much more you can do: components, lifecycled nodes, running multiple nodes in the same executable with intra-process communication, etc.

But the most important thing is that you first clearly understand how to write the code foundation that you’ll need for your nodes. Once you’re clear with that, start to work with ROS2 publishers, subscribers, parameters, services. Those are the most important functionalities you have to learn. And you’ll see, if you already have a good code structure for your nodes, adding more ROS2 functionalities will be quite straightforward.

## Temperature Nodes - Publisher and Subscriber

Following part of the guide [https://docs.ros.org/en/humble/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html](https://docs.ros.org/en/humble/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html) and adding some own creative functionality :).

```bash
cd playground-ros2-rpi
colcon build
colcon build --packages-select temperature_publisher
source ./install/setup.bash
ros2 run temperature_publisher temperature_publisher_node
```

For the temperature alarm:

```bash
cd playground-ros2-rpi
colcon build
colcon build --packages-select temperature_alarm
source ./install/setup.bash
ros2 run temperature_alarm temperature_alarm_node
```

Attention. Original example used `topic_callback(const std_msgs::msg::String & msg)` but it seems it needs to be `topic_callback(std_msgs::msg::String::UniquePtr msg)`.

How to run all? Not sure yet.

## Gamepad Controller

Actually node to run on a PC or laptop.

```bash
cd playground-ros2-rpi
colcon build
colcon build --packages-select gamepad_control
source ./install/setup.bash
ros2 run gamepad_control gamepad_control_node
```

## Thumper Drive

Node that takes in the gamepad buttons and drives trex controller

```bash
cd playground-ros2-rpi
colcon build
colcon build --packages-select thumper_drive
source ./install/setup.bash
ros2 run thumper_drive thumper_drive_node
```

## Multi-Machine Setup

Probable a good idea to also setup ROS2 on laptop. You can then develop and run packages on laptop and command other ROS2 instance.

This might be a good start: [https://roboticsbackend.com/ros2-multiple-machines-including-raspberry-pi/](https://roboticsbackend.com/ros2-multiple-machines-including-raspberry-pi/)

Damn apparently you just need to put the devices in the same network :).

## Launching All at Once

https://roboticsbackend.com/ros2-launch-file-example/

## Docs

* [https://docs.ros.org/en/humble/index.html](https://docs.ros.org/en/humble/index.html)
* [https://github.com/ros2/examples/tree/humble](https://github.com/ros2/examples/tree/humble)

## Useful commands

| Command | Description |
| --- | --- |
| `ros2 topic list -t` | List all topics and their types |
| `ros2 topic echo <name>` | Output the data being published on a topic |
| `ros2 topic info <name>` | Show topic stats |
| `ros2 node list` | List all nodes |
| `ros2 interface show <msg>` | Show detailed info about message |
| `ros2 param list` | List node parameters |
| `ros2 param set <node_name> <parameter_name> <value>` | Set parameter |

Creating a package:

```bash
ros2 pkg create --build-type ament_cmake PACKAGE_NAME
```

## Using Existing Packages

For example `joy` package [https://index.ros.org/p/joy/github-ros-drivers-joystick_drivers/](https://index.ros.org/p/joy/github-ros-drivers-joystick_drivers/).

```bash
sudo apt update
sudo apt install ros-foxy-joy
```

Running the node

```bash
ros2 run joy joy_node
```