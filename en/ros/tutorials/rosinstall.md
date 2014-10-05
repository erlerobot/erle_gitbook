# Installing and Configuring Your ROS Environment


### Ubuntu Precise (12.04)
Refer to [Ubuntu ARM install of ROS Hydro](http://wiki.ros.org/hydro/Installation/UbuntuARM).

### Debian Wheezy (7.5)
There are no precompiled binaries of ROS for Debian, while it seems that most of the dependencies are being pulled into the main repositories there are still some missing so compiling requires that the dependencies be made from source as well. The following steps describe how to compile everything from source doing this for a machine running Debian Wheezy.

#### Prerequisites
Configure your Debian repositories Setup your computer to accept software from packages.ros.org:

``` bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu wheezy main" > /etc/apt/sources.list.d/ros-latest.list'
```
Set up your keys:
``` bash
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
```
Refresh:
```bash
sudo apt-get update
```

Install bootstrap dependencies :
```bash
sudo apt-get install python-rosdep python-rosinstall-generator python-wstool build-essential
```

----

*NOTE: 15th of July, 2014*

From today, when trying to install the packages i get:
```
root@beaglebone:~# sudo apt-get install python-rosdep python-rosinstall-generator python-wstool build-essential
Reading package lists... Done
Building dependency tree
Reading state information... Done
build-essential is already the newest version.
Some packages could not be installed. This may mean that you have
requested an impossible situation or if you are using the unstable
distribution that some required packages have not yet been created
or been moved out of Incoming.
The following information may help to resolve the situation:

The following packages have unmet dependencies:
 python-rosdep : Depends: python-catkin-pkg but it is not going to be installed
                 Depends: python-rosdistro (>= 0.3.0) but it is not going to be installed
 python-rosinstall-generator : Depends: python-catkin-pkg (>= 0.1.28) but it is not going to be installed
                               Depends: python-rosdistro (>= 0.3.4) but it is not going to be installed
E: Unable to correct problems, you have held broken packages.

```


*NOTE 2: 15th of July, 2014*
It seems that `rosdep` cannot be installed in non-ubuntu systems from `apt-get`. To install it, run:
```bash
sudo pip install -U rosdep rosinstall_generator wstool rosinstall
```

*NOTE 3: 4th of October, 2014*
`nose` package nedded to be insalled before running `sudo pip install -U rosdep rosinstall_generator wstool rosinstall`.
```bash
pip install -U nose
```

----


Next ensure rosdep has been initialized:
```bash
sudo rosdep init
rosdep update
```

####Create a catkin Workspace
In order to build the core packages, you will need a catkin work-space. Create one now:
```bash
mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws
```
Next we will want to fetch the core packages so we can build them. We will use wstool for this. Select the wstool command for the particular variant you want to install:

---

According to previous versions of ROS:
- *Desktop-Full Install*: ROS, rqt, rviz, robot-generic libraries, 2D/3D simulators, navigation and 2D/3D perception [desktop-full]
- *Desktop Install*: ROS, rqt, rviz, and robot-generic libraries [desktop]
- *ROS-Comm*: (Bare Bones) ROS package, build, and communication libraries. No GUI tools. [ros_comm]

---

We really just need the communication layer thereby we are installing it bare bones.
```bash
rosinstall_generator ros_comm --rosdistro hydro --deps --wet-only > hydro-barebones-full-wet.rosinstall
```
```bash
wstool init src hydro-barebones-full-wet.rosinstall
```
or for Indigo:
```bash
rosinstall_generator ros_comm --rosdistro indigo --deps --wet-only > indigo-barebones-full-wet.rosinstall
```
```bash
wstool init src indigo-barebones-full-wet.rosinstall
```


This will add all of the catkin or wet packages in the given variant and then fetch the sources into the `~/ros_catkin_ws/src` directory. The command will take a few minutes to download all of the core ROS packages into the src folder.
If you have download cuts out restart it with the following
```bash
wstool update -t src
```

#### Resolving Dependencies
Before you can build your catkin work-space you need to make sure that you have all the required dependencies. We use the rosdep tool for this:

----

Installing `sblc` might be needed but at the time of writing it's not available, neither was possible to compile it from source thereby the Lips dependencies are removed.

In order to remove Lips dependencies, the following steps are required:
```bash
cd src
wstool rm roslisp
rm -rf roslisp
```

----


```bash
cd ..
rosdep install --from-paths src --ignore-src --rosdistro hydro -y -r --os=debian:wheezy
```

This will look at all of the packages in the src directory and find all of the dependencies they have. Then it will recursively install the dependencies.

The `--from-paths` option indicates we want to install the dependencies for an entire directory of packages, in this case `src`.
The `--ignore-src` option indicates to rosdep that it shouldn't try to install any ROS packages in the `src` folder from the package manager, we don't need it to since we are building them ourselves.
The `--rosdistro` option is required because we don't have a ROS environment setup yet, so we have to indicate to rosdep what version of ROS we are building for.
The `-y` option indicates to rosdep that we don't want to be bothered by too many prompts from the package manager. After a while (and maybe some prompts for your password) rosdep will finish installing system dependencies and you can continue.
The `-r` flag causes rosdep to ignore minor errors. This is necessary to get the base dependencies installed. After the packages following this section have been installed it would be a good idea to run this command again without the `-r` to see if there are any dependencies that you forgot or didn't have in your base system (i.e. packages different from what the author of this tutorial had installed).
The `--os` flag was necessary for the author because of differences in the way that the wheezy version of debian stores version information and what rosdep is expecting. If you are having trouble with any of the dependencies rosdep is supposed to get, make sure that you have the package name right by getting the `rosdep base.yaml` from this `http://docs.ros.org/independent/api/rosdep/html/contributing_rules.html`

Once all of the dependencies below are installed you should be able to run rosdep install excluding the -r option without errors by makeing sure all your dependencies are listed in the base.yaml.

----

*NOTE: 16th July, 2014*

```bash
root@beaglebone:~/ros_catkin_ws# rosdep install --from-paths src --ignore-src --rosdistro hydro -y -r --os=debian:wheezy
executing command [sudo apt-get install -y python-catkin-pkg]
Reading package lists... Done
Building dependency tree
Reading state information... Done
Some packages could not be installed. This may mean that you have
requested an impossible situation or if you are using the unstable
distribution that some required packages have not yet been created
or been moved out of Incoming.
The following information may help to resolve the situation:

The following packages have unmet dependencies:
 python-catkin-pkg : Depends: python:any (>= 2.7.1-0ubuntu2) but it is not installable
E: Unable to correct problems, you have held broken packages.
executing command [sudo apt-get install -y python-rosdep]
Reading package lists... Done
Building dependency tree
Reading state information... Done
Some packages could not be installed. This may mean that you have
requested an impossible situation or if you are using the unstable
distribution that some required packages have not yet been created
or been moved out of Incoming.
The following information may help to resolve the situation:

The following packages have unmet dependencies:
 python-rosdep : Depends: python-catkin-pkg but it is not going to be installed
                 Depends: python-rosdistro (>= 0.3.0) but it is not going to be installed
E: Unable to correct problems, you have held broken packages.
ERROR: the following rosdeps failed to install
  apt: command [sudo apt-get install -y python-catkin-pkg] failed
  apt: command [sudo apt-get install -y python-rosdep] failed
  apt: Failed to detect successful installation of [python-catkin-pkg]
  apt: Failed to detect successful installation of [python-rosdep]

```

Seems like the problem keeps on. Still the building step is performed and ROS works.

----


#### Building ROS

Finally run the build and install command. To install somewhere other than your home directory use the --install-space option. (Check [catkin_make_isolated REP](http://www.ros.org/reps/rep-0134.html)).

```bash
cd ~/ros_catkin_ws
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release
echo "source ~/ros_catkin_ws/install_isolated/setup.bash" >> ~/.bashrc
```

Note: You might want to select a different CMake build type (e.g. `RelWithDebInfo` or `Debug`, see http://cmake.org/cmake/help/v2.8.12/cmake.html#variable:CMAKE_BUILD_TYPE).

Now you have to reboot.

### Sources:
- [Installing Hydro in Debian](http://wiki.ros.org/hydro/Installation/Debian)
- [Installing Groovy in Debian](http://wiki.ros.org/groovy/Installation/Debian)
- [catkin_make_isolated](http://www.ros.org/reps/rep-0134.html)
