# Creating a ROS Package

This tutorial covers using `roscreate-pkg` or `catkin` to create a new package, and `rospack` to list package dependencies.


###What makes up a catkin Package?
For a package to be considered a catkin package it must meet a few requirements:

- The package must contain a catkin compliant `package.xml` file. This `package.xml` file provides meta information about the package.
- The package must contain a `CMakeLists.txt` which uses `catkin`. Catkin metapackages must have a boilerplate `CMakeLists.txt` file.
- There can be no more than one package in each folder. This means no nested packages nor multiple packages sharing the same directory

The simplest possible package might look like this:
```
my_package/
  CMakeLists.txt
  package.xml
```

### Packages in a catkin Workspace
The recommended method of working with catkin packages is using a [catkin workspace](http://wiki.ros.org/catkin/workspaces), but you can also build catkin packages standalone. A trivial workspace might look like this:
```
workspace_folder/         -- CATKIN WORKSPACE
  src/                    -- SOURCE SPACE
    CMakeLists.txt        -- The 'toplevel' CMake file
    package_1/
      CMakeLists.txt
      package.xml
      ...
    package_n/
      CMakeLists.txt
      package.xml
      ...
  build/                  -- BUILD SPACE
    CATKIN_IGNORE         -- Keeps catkin from walking this directory
  devel/                  -- DEVELOPMENT SPACE (set by CATKIN_DEVEL_PREFIX)
    bin/
    etc/
    include/
    lib/
    share/
    .catkin
    env.bash
    setup.bash
    setup.sh
    ...
  install/                -- INSTALL SPACE (set by CMAKE_INSTALL_PREFIX)
    bin/
    etc/
    include/
    lib/
    share/
    .catkin
    env.bash
    setup.bash
    setup.sh
    ...
```

At the [Erle-board](http://erlerobot.com), the catkin workspace is at `/root/catkin_ws`.

If you wish to create another catkin workspace, follow the [Creating a workspace for catkin tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

###Creating a catkin Package
This section will demonstrate how to use the `catkin_create_pkg` script to create a new catkin package, and what you can do with it after it has been created.

First change to the source space directory of the catkin workspace you created in the Creating a Workspace for catkin tutorial:

``` bash
# You should have created this in the Creating a Workspace Tutorial
$ cd ~/catkin_ws/src
```
Now use the `catkin_create_pkg` script to create a new package called 'erle_beginner_tutorials' which depends on `std_msgs`, `roscpp`, and `rospy`:

``` bash
root@erlerobot:~/catkin_ws/src# catkin_create_pkg erle_beginner_tutorials std_msgs rospy roscpp
Created file erle_beginner_tutorials/package.xml
Created file erle_beginner_tutorials/CMakeLists.txt
Created folder erle_beginner_tutorials/include
Created folder erle_beginner_tutorials/src
Successfully created files in /root/catkin_ws/src/erle_beginner_tutorials. Please adjust the values in package.xml.
```

This will create a `erle_beginner_tutorials` folder which contains a `package.xml` and a `CMakeLists.txt`, which have been partially filled out with the information you gave `catkin_create_pkg`.

```
root@erlerobot:~/catkin_ws/src# tree erle_beginner_tutorials/
erle_beginner_tutorials/
├── CMakeLists.txt
├── include
├── package.xml
└── src

2 directories, 2 files

```

`catkin_create_pkg` requires that you give it a `<package_name>` and optionally a list of dependencies on which that package depends:

``` bash
# This is an example, do not try to run this
# catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
```

`catkin_create_pkg` also has more advanced functionalities which is described in [catkin/commands/catkin_create_pkg](http://wiki.ros.org/catkin/commands/catkin_create_pkg).

### Package dependencies
####First-order dependencies
When using `catkin_create_pkg` earlier, a few package dependencies were provided. These first-order dependencies can now be reviewed with the `rospack` tool.

``` bash
root@erlerobot:~# rospack depends1 erle_beginner_tutorials
roscpp
rospy
std_msgs
```
As you can see, `rospack` lists the same dependencies that were used as arguments when running `catkin_create_pkg`. These dependencies for a package are stored in the `package.xml` file:

```
root@erlerobot:~# roscd erle_beginner_tutorials/
root@erlerobot:~/catkin_ws/src/erle_beginner_tutorials# cat package.xml
```
```xml
<?xml version="1.0"?>
<package>
  <name>erle_beginner_tutorials</name>
  <version>0.0.0</version>
  <description>The erle_beginner_tutorials package</description>

  <!-- One maintainer tag required, multiple allowed, one person per tag -->
  <!-- Example:  -->
  <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  <maintainer email="root@todo.todo">root</maintainer>


  <!-- One license tag required, multiple allowed, one license per tag -->
  <!-- Commonly used license strings: -->
  <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  <license>TODO</license>


  <!-- Url tags are optional, but mutiple are allowed, one per tag -->
  <!-- Optional attribute type can be: website, bugtracker, or repository -->
  <!-- Example: -->
  <!-- <url type="website">http://ros.org/wiki/erle_beginner_tutorials</url> -->


  <!-- Author tags are optional, mutiple are allowed, one per tag -->
  <!-- Authors do not have to be maintianers, but could be -->
  <!-- Example: -->
  <!-- <author email="jane.doe@example.com">Jane Doe</author> -->


  <!-- The *_depend tags are used to specify dependencies -->
  <!-- Dependencies can be catkin packages or system dependencies -->
  <!-- Examples: -->
  <!-- Use build_depend for packages you need at compile time: -->
  <!--   <build_depend>message_generation</build_depend> -->
  <!-- Use buildtool_depend for build tool packages: -->
  <!--   <buildtool_depend>catkin</buildtool_depend> -->
  <!-- Use run_depend for packages you need at runtime: -->
  <!--   <run_depend>message_runtime</run_depend> -->
  <!-- Use test_depend for packages you need only for testing: -->
  <!--   <test_depend>gtest</test_depend> -->
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <run_depend>roscpp</run_depend>
  <run_depend>rospy</run_depend>
  <run_depend>std_msgs</run_depend>


  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- You can specify that this package is a metapackage here: -->
    <!-- <metapackage/> -->

    <!-- Other tools can request additional information be placed here -->

  </export>
</package>
```

####Indirect dependencies
In many cases, a dependency will also have its own dependencies. For instance, rospy has other dependencies:


``` bash
root@erlerobot:~/catkin_ws/src/erle_beginner_tutorials# rospack depends1 rospy
genpy
rosgraph
rosgraph_msgs
roslib
std_msgs
```

A package can have quite a few indirect dependencies. Luckily rospack can recursively determine all nested dependencies.

``` bash
root@erlerobot:~/catkin_ws/src/erle_beginner_tutorials# rospack depends erle_beginner_tutorials
cpp_common
rostime
roscpp_traits
roscpp_serialization
genmsg
genpy
message_runtime
rosconsole
std_msgs
rosgraph_msgs
xmlrpcpp
roscpp
rosgraph
catkin
rospack
roslib
rospy

```

###Customizing Your Package
This part of the tutorial will look at each file generated by `catkin_create_pkg` and describe, line by line, each component of those files and how you can customize them for your package.

####Customizing the package.xml
The generated package.xml should be in your new package. Now lets go through the new package.xml and touch up any elements that need your attention.

description tag

First update the description tag:


[des]activar nros. de línea
   5   <description>The beginner_tutorials package</description>

Change the description to anything you like, but by convention the first sentence should be short while covering the scope of the package. If it is hard to describe the package in a single sentence then it might need to be broken up.

maintainer tags

Next comes the maintainer tag:


[des]activar nros. de línea
   7   <!-- One maintainer tag required, multiple allowed, one person per tag -->
   8   <!-- Example:  -->
   9   <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  10   <maintainer email="user@todo.todo">user</maintainer>

This is a required and important tag for the package.xml because it lets others know who to contact about the package. At least one maintainer is required, but you can have many if you like. The name of the maintainer goes into the body of the tag, but there is also an email attribute that should be filled out:


[des]activar nros. de línea
   7   <maintainer email="you@yourdomain.tld">Your Name</maintainer>

license tags

Next is the license tag, which is also required:


[des]activar nros. de línea
  12   <!-- One license tag required, multiple allowed, one license per tag -->
  13   <!-- Commonly used license strings: -->
  14   <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  15   <license>TODO</license>

You should choose a license and fill it in here. Some common open source licenses are BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, and LGPLv3. You can read about several of these at the Open Source Initiative. For this tutorial we'll use the BSD license because the rest of the core ROS components use it already:


[des]activar nros. de línea
   8   <license>BSD</license>

dependencies tags

The next set of tags describe the dependencies of your package. The dependencies are split into build_depend, buildtool_depend, run_depend, test_depend. For a more detailed explination of these tags see the documentation about Catkin Dependencies. Since we passed std_msgs, roscpp, and rospy as arguments to catkin_create_pkg, the dependencies will look like this:


[des]activar nros. de línea
  27   <!-- The *_depend tags are used to specify dependencies -->
  28   <!-- Dependencies can be catkin packages or system dependencies -->
  29   <!-- Examples: -->
  30   <!-- Use build_depend for packages you need at compile time: -->
  31   <!--   <build_depend>genmsg</build_depend> -->
  32   <!-- Use buildtool_depend for build tool packages: -->
  33   <!--   <buildtool_depend>catkin</buildtool_depend> -->
  34   <!-- Use run_depend for packages you need at runtime: -->
  35   <!--   <run_depend>python-yaml</run_depend> -->
  36   <!-- Use test_depend for packages you need only for testing: -->
  37   <!--   <test_depend>gtest</test_depend> -->
  38   <buildtool_depend>catkin</buildtool_depend>
  39   <build_depend>roscpp</build_depend>
  40   <build_depend>rospy</build_depend>
  41   <build_depend>std_msgs</build_depend>

All of our listed dependencies have been added as a build_depend for us, in addition to the default buildtool_depend on catkin. In this case we want all of our specified dependencies to be available at build and run time, so we'll add a run_depend tag for each of them as well:


[des]activar nros. de línea
  12   <buildtool_depend>catkin</buildtool_depend>
  13
  14   <build_depend>roscpp</build_depend>
  15   <build_depend>rospy</build_depend>
  16   <build_depend>std_msgs</build_depend>
  17
  18   <run_depend>roscpp</run_depend>
  19   <run_depend>rospy</run_depend>
  20   <run_depend>std_msgs</run_depend>

Final package.xml

As you can see the final package.xml, without comments and unused tags, is much more concise:


[des]activar nros. de línea
   1 <?xml version="1.0"?>
   2 <package>
   3   <name>beginner_tutorials</name>
   4   <version>0.1.0</version>
   5   <description>The beginner_tutorials package</description>
   6
   7   <maintainer email="you@yourdomain.tld">Your Name</maintainer>
   8   <license>BSD</license>
   9   <url type="website">http://wiki.ros.org/beginner_tutorials</url>
  10   <author email="you@yourdomain.tld">Jane Doe</author>
  11
  12   <buildtool_depend>catkin</buildtool_depend>
  13
  14   <build_depend>roscpp</build_depend>
  15   <build_depend>rospy</build_depend>
  16   <build_depend>std_msgs</build_depend>
  17
  18   <run_depend>roscpp</run_depend>
  19   <run_depend>rospy</run_depend>
  20   <run_depend>std_msgs</run_depend>
  21
  22 </package>

Customizing the CMakeLists.txt
Now that the package.xml, which contains meta information, has been tailored to your package, you are ready to move on in the tutorials. The CMakeLists.txt file created by catkin_create_pkg will be covered in the later tutorials about building ROS code.

Now that you've made a new ROS package, let's build our ROS package.
