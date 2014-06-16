#ROS: Concepts


ROS has three levels of concepts: the **Filesystem level**, the **Computation Graph level**, and the **Community level**. These levels and concepts are summarized below and later sections go into each of these in greater detail.

In addition to the three levels of concepts, ROS also defines two types of names: **Package Resource Names** and **Graph Resource Names**, also discussed below.

###Filesystem level


The filesystem level concepts mainly cover ROS resources that you encounter on disk, such as:

- **Packages**: Packages are the main unit for organizing software in ROS. A package may contain ROS runtime processes (nodes), a ROS-dependent library, datasets, configuration files, or anything else that is usefully organized together. Packages are the most atomic build item and release item in ROS. Meaning that the most granular thing you can build and release is a package.
- **Metapackages**: Metapackages are specialized Packages which only serve to represent a group of related other packages. Most commonly metapackages are used as a backwards compatible place holder for converted rosbuild Stacks.
- **Package Manifests**: Manifests (package.xml) provide metadata about a package, including its name, version, description, license information, dependencies, and other meta information like exported packages. The package.xml package manifest is defined in REP-0127.
- **Repositories**: A collection of packages which share a common VCS system. Packages which share a VCS share the same version and can be released together using the catkin release automation tool bloom. Often these repositories will map to converted rosbuild Stacks. Repositories can also contain only one package.
- **Message (msg) types**: Message descriptions, stored in my_package/msg/MyMessageType.msg, define the data structures for messages sent in ROS.
- **Service (srv) types**: Service descriptions, stored in my_package/srv/MyServiceType.srv, define the request and response data structures for services in ROS.

###Computation Graph level


The Computation Graph is the peer-to-peer network of ROS processes that are processing data together. The basic Computation Graph concepts of ROS are *nodes*, *Master*, *Parameter Server*, *messages*, *services*, *topics*, and *bags*, all of which provide data to the Graph in different ways.

These concepts are implemented in the [ros_comm](http://wiki.ros.org/ros_comm) repository.

- [**Nodes**](http://wiki.ros.org/Nodes): Nodes are processes that perform computation. ROS is designed to be modular at a fine-grained scale; a robot control system usually comprises many nodes. For example, one node controls a laser range-finder, one node controls the wheel motors, one node performs localization, one node performs path planning, one Node provides a graphical view of the system, and so on. A ROS node is written with the use of a ROS [client library](http://wiki.ros.org/Client%20Libraries), such as [roscpp](http://wiki.ros.org/roscpp) or [rospy](http://wiki.ros.org/rospy).
- [**Master**](http://wiki.ros.org/Master): The ROS Master provides name registration and lookup to the rest of the Computation Graph. Without the Master, nodes would not be able to find each other, exchange messages, or invoke services.
- [**Parameter Server**](http://wiki.ros.org/Parameter%20Server): The Parameter Server allows data to be stored by key in a central location. It is currently part of the Master.
- [**Messages**](http://wiki.ros.org/Messages): Nodes communicate with each other by passing [messages](http://wiki.ros.org/Messages). A message is simply a data structure, comprising typed fields. Standard primitive types (integer, floating point, boolean, etc.) are supported, as are arrays of primitive types. Messages can include arbitrarily nested structures and arrays (much like C structs).
- [**Topics**](http://wiki.ros.org/Topics): Messages are routed via a transport system with publish / subscribe semantics. A node sends out a message by publishing it to a given [topic](http://wiki.ros.org/Topics). The topic is a [name](http://wiki.ros.org/Names) that is used to identify the content of the message. A node that is interested in a certain kind of data will subscribe to the appropriate topic. There may be multiple concurrent publishers and subscribers for a single topic, and a single node may publish and/or subscribe to multiple topics. In general, publishers and subscribers are not aware of each others' existence. The idea is to decouple the production of information from its consumption. Logically, one can think of a topic as a strongly typed message bus. Each bus has a name, and anyone can connect to the bus to send or receive messages as long as they are the right type.
![](http://ros.org/images/wiki/ROS_basic_concepts.png)

- [**Services**](http://wiki.ros.org/Services): The publish / subscribe model is a very flexible communication paradigm, but its many-to-many, one-way transport is not appropriate for request / reply interactions, which are often required in a distributed system. Request / reply is done via services, which are defined by a pair of message structures: one for the request and one for the reply. A providing node offers a service under a name and a client uses the service by sending the request message and awaiting the reply. ROS client libraries generally present this interaction to the programmer as if it were a remote procedure call.
- [**Bags**](http://wiki.ros.org/Bags): Bags are a format for saving and playing back ROS message data. Bags are an important mechanism for storing data, such as sensor data, that can be difficult to collect but is necessary for developing and testing algorithms.

The ROS Master acts as a nameservice in the ROS Computation Graph. It stores topics and services registration information for ROS nodes. Nodes communicate with the Master to report their registration information. As these nodes communicate with the Master, they can receive information about other registered nodes and make connections as appropriate. The Master will also make callbacks to these nodes when this registration information changes, which allows nodes to dynamically create connections as new nodes are run.

Nodes connect to other nodes directly; the Master only provides lookup information, much like a DNS server. Nodes that subscribe to a topic will request connections from nodes that publish that topic, and will establish that connection over an agreed upon connection protocol. The most common protocol used in a ROS is called [TCPROS](http://wiki.ros.org/ROS/TCPROS), which uses standard TCP/IP sockets.


This architecture allows for decoupled operation, where the names are the primary means by which larger and more complex systems can be built. Names have a very important role in ROS: nodes, topics, services, and parameters all have names. Every ROS client library supports [command-line remapping of names](http://wiki.ros.org/Remapping%20Arguments), which means a compiled program can be reconfigured at runtime to operate in a different Computation Graph topology.

----

For example, to control a Hokuyo laser range-finder, we can start the hokuyo_node driver, which talks to the laser and publishes sensor_msgs/LaserScan messages on the scan topic. To process that data, we might write a node using laser_filters that subscribes to messages on the scan topic. After subscription, our filter would automatically start receiving messages from the laser.

Note how the two sides are decoupled. All the hokuyo_node node does is publish scans, without knowledge of whether anyone is subscribed. All the filter does is subscribe to scans, without knowledge of whether anyone is publishing them. The two nodes can be started, killed, and restarted, in any order, without inducing any error conditions.

Later we might add another laser to our robot, so we need to reconfigure our system. All we need to do is remap the names that are used. When we start our first hokuyo_node, we could tell it instead to remap scan to base_scan, and do the same with our filter node. Now, both of these nodes will communicate using the base_scan topic instead and not hear messages on the scan topic. Then we can just start another hokuyo_node for the new laser range finder.



###Community level


The ROS Community Level concepts are ROS resources that enable separate communities to exchange software and knowledge. These resources include:

- [**Distributions**](http://wiki.ros.org/Distributions): ROS Distributions are collections of versioned stacks that you can install. Distributions play a similar role to Linux distributions: they make it easier to install a collection of software, and they also maintain consistent versions across a set of software.
- [**Repositories**](http://wiki.ros.org/Repositories): ROS relies on a federated network of code repositories, where different institutions can develop and release their own robot software components.
- [**The ROS Wiki**](http://wiki.ros.org/Documentation): The ROS community Wiki is the main forum for documenting information about ROS. Anyone can sign up for an account and contribute their own documentation, provide corrections or updates, write tutorials, and more.
- **Bug Ticket System**: Please see [Tickets](http://wiki.ros.org/Tickets) for information about file tickets.
- [**Mailing Lists**](http://wiki.ros.org/Mailing%20Lists): The ros-users mailing list is the primary communication channel about new updates to ROS, as well as a forum to ask questions about ROS software.
- [**ROS Answers**](http://answers.ros.org/): A Q&A site for answering your ROS-related questions.
- [**Blog**](http://www.willowgarage.com/blog): The [Willow Garage Blog](http://www.willowgarage.com/blog) provides regular updates, including photos and videos.

###Naming


####Graph Resource Names

Graph Resource Names provide a hierarchical naming structure that is used for all resources in a ROS Computation Graph, such as Nodes, Parameters, Topics, and Services. These names are very powerful in ROS and central to how larger and more complicated systems are composed in ROS, so it is critical to understand how these names work and how you can manipulate them.

Before we describe names further, here are some example names:

- / (the global namespace)
- /foo
- /stanford/robot/name
- /wg/node1

Graph Resource Names are an important mechanism in ROS for providing encapsulation. Each resource is defined within a namespace, which it may share with many other resources. In general, resources can create resources within their namespace and they can access resources within or above their own namespace. Connections can be made between resources in distinct namespaces, but this is generally done by integration code above both namespaces. This encapsulation isolates different portions of the system from accidentally grabbing the wrong named resource or globally hijacking names.

Names are resolved relatively, so resources do not need to be aware of which namespace they are in. This simplifies programming as nodes that work together can be written as if they are all in the top-level namespace. When these Nodes are integrated into a larger system, they can be pushed down into a namespace that defines their collection of code. For example, one could take a Stanford demo and a Willow Garage demo and merge them into a new demo with stanford and wg subgraphs. If both demos had a Node named 'camera', they would not conflict. Tools (e.g. graph visualization) as well as parameters (e.g. demo_name) that need to be visible to the entire graph can be created by top-level Nodes.

#####Valid Names

A valid name has the following characteristics:

1. First character is an alpha character ([a-z|A-Z]), tilde (~) or forward slash (/)
2. Subsequent characters can be alphanumeric ([0-9|a-z|A-Z]), underscores (_), or forward slashes (/)

------

**Exception**: base names (described below) cannot have forward slashes (/) or tildes (~) in them.

------

#####Resolving

There are four types of Graph Resource Names in ROS: base, relative, global, and private, which have the following syntax:

- base
- relative/name
- /global/name
- ~private/name


By default, resolution is done relative to the node's namespace. For example, the node `/wg/node1` has the namespace `/wg`, so the name node2 will resolve to `/wg/node2`.

Names with no namespace qualifiers whatsoever are base names. Base names are actually a subclass of relative names and have the same resolution rules. Base names are most frequently used to initialize the node name.

Names that start with a "/" are global -- they are considered fully resolved. *Global names should be avoided as much as possible as they limit code portability*.

Names that start with a "~" are private. They convert the node's name into a namespace. For example, node1 in namespace `/wg/` has the private namespace `/wg/node1`. Private names are useful for passing parameters to a specific node via the parameter server.

Here are some name resolution examples:

|Node | Relative (default) | Global | Private |
| -- | -- | -- | -- |
| `/node1` | `bar -> /bar` | `/bar -> /bar` | `~bar -> /node1/bar`|
|`/wg/node2` | `bar -> /wg/bar` | `/bar -> /bar` | `~bar -> /wg/node2/bar` |
| `/wg/node3` | `foo/bar -> /wg/foo/bar` | `/foo/bar -> /foo/bar` | `~foo/bar -> /wg/node3/foo/bar` |

#####Remapping

Any name within a ROS Node can be remapped when the Node is launched at the command-line. For more information on this feature, see [Remapping Arguments](http://wiki.ros.org/Remapping%20Arguments).

####Package Resource Names

Package Resource Names are used in ROS with Filesystem-Level concepts to simplify the process of referring to files and data types on disk. Package Resource Names are very simple: they are just the name of the Package that the resource is in plus the name of the resource. For example, the name `std_msgs/String` refers to the "String" message type in the "std_msgs" Package.

Some of the ROS-related files that may be referred to using Package Resource Names include:

- [Message (msg) types](http://wiki.ros.org/msg)
- [Service (srv) types](http://wiki.ros.org/srv)
- [Node types](http://wiki.ros.org/Nodes)

Package Resource Names are very similar to file paths, except they are much shorter. This is due to the ability of ROS to locate Packages on disk and make additional assumptions about their contents. For example, Message descriptions are always stored in the `msg` subdirectory and have the `.msg` extension, so `std_msgs/String` is shorthand for `path/to/std_msgs/msg/String.msg`. Similarly, the Node type `foo/bar` is equivalent to searching for a file named `bar` in Package `foo` with executable permissions.

#####Valid Names

Package Resource Names have strict naming rules as they are often used in auto-generated code. For this reason, a ROS package cannot have special characters other than an underscore, and they must start with an alphabetical character. A valid name has the following characteristics:

1. First character is an alpha character ([a-z|A-Z])
2. Subsequent characters can be alphanumeric ([0-9|a-z|A-Z]), underscores (_) or a forward slash (/)
3. There is at most one forward slash ('/').

