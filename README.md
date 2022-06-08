# Running ROS2 on the Raspberry Pi 4 of the Thumper

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
cd ros2_ws
colcon build --packages-select hello_name
source ./install/setup.bash
ros2 run hello_name hello_name_node
```

**Package 'hello_name' not found ?** Important. Don't forget to source the bash script after creating a new package.

## Multi-Machine Setup

Probable a good idea to also setup ROS2 on laptop. You can then develop and run packages on laptop and command other ROS2 instance.

This might be a good start: [https://roboticsbackend.com/ros2-multiple-machines-including-raspberry-pi/](https://roboticsbackend.com/ros2-multiple-machines-including-raspberry-pi/)