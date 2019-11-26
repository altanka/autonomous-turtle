# Autonomous Turtle

Example project for autonomous driving Turtlebot3

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

If you use Ubuntu WSL, you should download a Windows X-Server

* [VcXsrv](https://sourceforge.net/projects/vcxsrv/) - X-Server for Windows that I used

Remember to uncheck Native OpenGL

You should run this commands to run Gazebo from Ubuntu WSL

```
echo "export DISPLAY=:0" >> ~/.bashrc
echo "export GAZEBO_IP=127.0.0.1" >> ~/.bashrc
```

We choose Turtlebot3 Burger Model for Gazebo
```
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
```


### Installing

After cloning the project, you should build it.


```
catkin_make
```


## Running the nodes

Nodes will be explained here




