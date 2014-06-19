# Building a ROS Package

This tutorial covers the toolchain to build a package.

###Building Packages
As long as all of the system dependencies of your package are installed, we can now build your new package.

Before continuing remember to source your environment setup file if you have not already (on the provided images, this is done in the `.bashrc` file).

``` bash
source /root/catkin_ws/devel/setup.bash
```

####Using catkin_make

`catkin_make is a command line tool which adds some convenience to the standard catkin workflow. You can imagine that [catkin_make](http://wiki.ros.org/catkin/commands/catkin_make) combines the calls to `cmake` and `make` in the _standard CMake workflow_.

Usage:

```
# In a catkin workspace
$ catkin_make [make_targets] [-DCMAKE_VARIABLES=...]
```

For people who are unfamiliar with the standard CMake workflow, it breaks down as follows:

---

Note: If you run the below commands it will not work, as this is just an example of how CMake generally works.

---

``` bash
# In a CMake project
$ mkdir build
$ cd build
$ cmake ..
$ make
$ make install  # (optionally)
```

This process is run for each CMake project. In contrast catkin projects can be built together in workspaces. Building zero to many catkin packages in a workspace follows this work flow:

``` bash
# In a catkin workspace
$ catkin_make
$ catkin_make install  # (optionally)
```

The above commands will build any catkin projects found in the src folder. This follows the recommendations set by [REP128](http://www.ros.org/reps/rep-0128.html). If your source code is in a different place, say `my_src` then you would call catkin_make like this:

---

Note: If you run the below commands it will not work, as the directory `my_src` does not exist.

---

```bash
# In a catkin workspace
$ catkin_make --source my_src
$ catkin_make install --source my_src  # (optionally)
```

For more advanced uses of catkin_make see the documentation: [catkin/commands/catkin_make](http://wiki.ros.org/catkin/commands/catkin_make).

####Building Your Package

You should already have a catkin workspace and a new catkin package called `erle_beginner_tutorials` from the previous tutorial. Go into the catkin workspace if you are not already there and look in the src folder:

```bash
root@erlerobot:~# cd ~/catkin_ws/src
root@erlerobot:~/catkin_ws/src# ls
CMakeLists.txt  erle_beginner_tutorials
````

We can now build that package using `catkin_make`:

```bash
root@erlerobot:~# cd ~/catkin_ws
root@erlerobot:~/catkin_ws# catkin_make
Base path: /root/catkin_ws
Source space: /root/catkin_ws/src
Build space: /root/catkin_ws/build
Devel space: /root/catkin_ws/devel
Install space: /root/catkin_ws/install
####
#### Running command: "cmake /root/catkin_ws/src -DCATKIN_DEVEL_PREFIX=/root/catkin_ws/devel -DCMAKE_INSTALL_PREFIX=/root/catkin_ws/install" in "/root/catkin_ws/build"
####
-- Using CATKIN_DEVEL_PREFIX: /root/catkin_ws/devel
-- Using CMAKE_PREFIX_PATH: /root/catkin_ws/devel;/opt/ros/groovy
-- This workspace overlays: /root/catkin_ws/devel;/opt/ros/groovy
-- Found gtest sources under '/usr/src/gtest': gtests will be built
-- Using CATKIN_TEST_RESULTS_DIR: /root/catkin_ws/build/test_results
-- catkin 0.5.65
-- BUILD_SHARED_LIBS is on
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- ~~  traversing 1 packages in topological order:
-- ~~  - erle_beginner_tutorials
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- +++ processing catkin package: 'erle_beginner_tutorials'
-- ==> add_subdirectory(erle_beginner_tutorials)
-- Configuring done
-- Generating done
-- Build files have been written to: /root/catkin_ws/build
####
#### Running command: "make -j1 -l1" in "/root/catkin_ws/build"
####
-- Using CATKIN_DEVEL_PREFIX: /root/catkin_ws/devel
-- Using CMAKE_PREFIX_PATH: /root/catkin_ws/devel;/opt/ros/groovy
-- This workspace overlays: /root/catkin_ws/devel;/opt/ros/groovy
-- Found gtest sources under '/usr/src/gtest': gtests will be built
-- Using CATKIN_TEST_RESULTS_DIR: /root/catkin_ws/build/test_results
-- catkin 0.5.65
-- BUILD_SHARED_LIBS is on
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- ~~  traversing 1 packages in topological order:
-- ~~  - erle_beginner_tutorials
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- +++ processing catkin package: 'erle_beginner_tutorials'
-- ==> add_subdirectory(erle_beginner_tutorials)
-- Configuring done
-- Generating done
-- Build files have been written to: /root/catkin_ws/build

```

Note that `catkin_make` first displays what paths it is using for each of the 'spaces'. The spaces are described in the [REP128](http://www.ros.org/reps/rep-0128.html) and by documentation about catkin workspaces on the wiki: [catkin/workspaces](http://wiki.ros.org/catkin/workspaces). The important thing to notice is that because of these default values several folders have been created in your catkin workspace. Take a look with ls:


```bash
root@erlerobot:~/catkin_ws# ls
build  devel  install  src
```

The build folder is the default location of the build space and is where cmake and make are called to configure and build your packages. The devel folder is the default location of the devel space, which is where your executables and libraries go before you install your packages.
