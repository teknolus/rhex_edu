# Notes on ROS2

## Creating a Workspace
First you need to create a workspace to work on. This workspace will contain all the packages that you will create or use.
Which can be done via command line by running the following command:
```
mkdir ~/ros2_ws
```
The above command will create a directory named `ros2_ws` in your home directory. As a convention, ROS2 workspaces are named as `ros2_ws` but you can name it whatever you want. For example, In the rhex_edu repository, the workspace is named as `rhex_ws`.

Then you need to create a `src` directory inside the workspace. This directory will contain all the packages that you will create or use.
```
mkdir ~/ros2_ws/src
```
All the packages and nodes will be inside this `src` directory.

- Also you can combine these two commands into one by running:
```
mkdir -p ~/ros2_ws/src
```
Where `-p` flag is used to create the parent directories if they don't exist. Above command first creates the `ros2_ws` directory then creates the `src` directory inside it.

## Creating a Package
After creating the workspace you can create a package inside the `src` directory. This package will contain the nodes and other files that you will use in your project. 

To do this you can use the following command:
```
ros2 pkg create <package_name> --build-type ament_python --dependencies rclpy
```
`<package_name>` is the name of the package that you want to create. You can name it whatever you want but it is recommended to use lowercase letters and underscores instead of spaces and there is a convention for naming the packages which is related with their purpose that will be explained in this tutorial. 

For example, if you want to create a package named `my_package` you can run the following command:
```
ros2 pkg create my_package --build-type ament_python --dependencies rclpy
```
`--build-type ament_python` flag is used to specify the build type of the package. In this case, it is a Python package. If you want to create a C++ package you can use `--build-type ament_cmake` flag instead or you can omit this flag to create a package with both Python and C++ files.

`--dependencies rclpy` flag is used to specify the dependencies of the package. In this case, it is `rclpy` which is the ROS2 Python client library which can be left blank if you don't have any dependencies. They can be added later by editing the `package.xml` file.

After running the above command you will see a new directory with the name of the package that you created inside the `src` directory. This directory will contain the necessary files to start working on your project.

If you created a package with Python build type you will see the following files and directories inside the package directory:
```
ros2_ws
├── src
    └── my_package
        ├── my_package
        │   ├── __init__.py
        ├── setup.py
        ├── package.xml
        └── setup.cfg
```
As you can see there is a directory with the same name as the package inside the package directory. This directory will contain the Python files that you will write for your project.

- **package.xml File**

package.xml file contains the metadata of the package such as name, version, description, maintainer, license, dependencies, etc. You can edit this file to add dependencies or change the metadata of the package. If you provide any dependencies while creating the package, they will be added to the `package.xml` file automatically. If you used `--dependencies rclpy` flag while creating the package you will see the following lines inside the `<depend>` tag in the `package.xml` file:
```
<depend>rclpy</depend>

```

Providing dependencies while creating packages is not a must, if you want you can add dependencies later. To add dependencies to the package you can edit the `package.xml` file and add the following lines inside the `<depend>` tag:
```
<depend>example_interfaces</depend>

```

And below you will see
```
<build_type>ament_python</build_type>
```
Which specifies the build type of the package.

---
If you created a package with C++ build type you will see the following files and directories inside the package directory:
```
ros2_ws
├── src
    └── my_package
        ├── include
        │   └── my_package
        ├── src
        │   └── my_package
        ├── CMakeLists.txt
        └── package.xml
```
As you can see there are `include` and `src` directories inside the package directory. `include` directory will contain the header files and `src` directory will contain the source files that you will write for your project.

## Building the Workspace
After creating the package you need to build the workspace to make the package available for use. To build the workspace you can run the following command:
```
colcon build
```
This command will build all the packages inside the workspace. If you have multiple packages inside the `src` directory, all of them will be built. After running the above command you will see a new directories named `build` and `install` inside the workspace directory. These directories will contain the build files and the installed files respectively.

**Important #1:** Be careful! Do not to run the `colcon build` command in the `src` directory of the workspace. Always run it inside of the workspace directory. Which should have `src` folder in it. You can check it by running `ls` command in the terminal.

**Troubleshooting #1:** If SetupTools error pops out after colcon build, then downgrade the version of setuptools by running the following command:
```
pip3 install setuptools==58.2.0
```
Then `colcon build` again.

Also you can use the following command to build the specific package:
```
colcon build --packages-select <package_name>
```
`<package_name>` is the name of the package that you want to build. If you have multiple packages inside the `src` directory and you want to build only one of them you can use the above command.

After building the workspace you need to run the following command to activate your workspace:
```
source ~/ros2_ws/install/setup.bash
```
This command will add the workspace to the ROS2 environment. After running the above command you can use the packages that you created or installed in your workspace. You can add this line to the `~/.bashrc`, so that workspace will be automatically activated when a new terminal is opened. To achieve this run the following code in the terminal:
```
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## ROS2 Node
ROS2 Node is subpart of the application and has a single purpose. Applications will contain many nodes and these nodes will be put in the packages. Then nodes will communicate with each other using ROS Topics, services and parameters. All nodes names must be unique do not give the same name.

To create a node you need to write a Python or C++ file that contains the node. This file will contain the necessary code to create the node and make it do its job.

For example if you want to write a Python code inside your python package named `my_package` you can create a Python file named `my_node.py` inside the `my_package` directory which is located at `~/ros2_ws/src/my_package/my_package`. 

### Running a ROS2 Node
To run a ROS2 node `ros2 run <package name> <executable/node name>` command is used. For example if you want to run a node named `my_node` inside the `my_package` package you can run the following command:
```
ros2 run my_package my_node
```
Sometimes we want to run the same node more than once but due to being the same node they will have the same name but it is not desired. So if needed you can change the node name from command line via following code:
```
ros2 run package_name node_name --ros-args -r __node:=new_node_name
```
`package_name` is the name of the package that contains the node. `node_name` is the name of the node that you want to run. `new_node_name` is the new name that you want to give to the node.

**Important #2:** After any changes you made in the Node code you have to do colcon build in order to changes to take place. However you can use --symlink-install flag to create symbolic links to the files. Code is as follows:
```
colcon build --symlink-install
```
Compile once with this flag and you do not have to compile it again for python, for cpp you must compile everytime. Also, `node_py` file inside the `src/package_name/package_name` must be executable in order to this flag work. You can make it executable by running the following command:
```
chmod +x node_py
```

You can view the list of nodes that are running by running the following command:
```
ros2 node list
```

## ROS2 Topics
ROS2 Topic is a bus over which nodes communicate. Topics are used when data must be sent. Subscribers and publishers must send messages with same data structure. 

Subscribers and publishers are Nodes. Subscribers do not know other subscribers and do not know who is publishing into topic. Publishers is not aware of other publishers and who is receiving data. Publishers can send different data structures to different topics. A node can be both subscriber and publisher at the same time.

You can view the list of topics that are available by running the following command:
```
ros2 topic list
```

It will list all the topics that are available in the system. You can view the type of the topic by running the following command:
```
ros2 topic info /topic_name
```
`/topic_name` is the name of the topic that you want to get information about. It will print the type of the topic and the publisher of the topic. Output will be like:
```
Type: example_interfaces/msg/String
Publisher count: 1
Subscription count: 0

```
If you want to learn more about the message type you can run the following command:
```
ros2 interface show package_name/msg/msg_type
```
Helps you find the structure of messages. With show you print it on the screen.

To print the messages that are published to the topic you can run the following command:
```
ros2 topic echo /topic_name
```

**Important #3:** When publisher is terminated, subscriber does not terminated subscriber node is still running and waiting a message. When publisher node is started again subscriber starts receiving messages.

## ROS2 Launch Files
ROS2 launch file is used for starting multiple nodes at the same time. 

First you have to create a new package for the launch file. It should be a package since launch file is going to launch some nodes from different packages. You can create a package with the following command:
```
ros2 pkg create my_robot_bringup
```
In the package name `_bringup` is a convention. It is used to indicate that this package is used to bring up the robot. You can name it whatever you want but it is recommended to use lowercase letters and underscores instead of spaces.

Since no build type is specified cmake is used. Go into the package and delete include and src folders. Then create a launch directory.

After creating launch files dependencies of desired nodes to be launched must be added to package.xml

Add following lines to CMakeLists.txt to install launch folder:
```
install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
    )
```

Launch files are named as `<name>.launch.py` as a convention. Then create a python file with the name of the launch file. Then make it executable.

To launch a launch file you can run the following command:
```
ros2 launch package_name launch_file_name.launch.py
```
You can create different launch files inside launch directory. For example you can create a launch file named `start_sim.launch.py` which will launch all the necessary nodes for the simulation such as in this rhex_ws repository.