# MAVProxy: A UAV ground station software package for MAVLink based systems

MAVProxy is a fully-functioning Ground Control Station for UAV's (Unmanned Aerial Vehicles). The intent is for a minimalist, portable and extendable Ground Control Station (GCS) for any UAV supporting the MAVLink protocol.

### Features:

- It is a command-line, console based app. There are plugins included in MAVProxy to provide a basic GUI.
- Can be networked and run over any number of computers.
- It's portable; it should run on any _POSIX_ OS with `python`, `pyserial`, and `select()` function calls, which means _Linux_, _OS X_, _Windows_, and others.
- The *light-weight* design means it can run on small netbooks with ease.
- It supports loadable modules, and has modules to support console/s, moving maps, joysticks, antenna trackers, etc.
- Tab-completion of commands.

You can launch `mavproxy` with the following options:

```
mavproxy.py -h
Usage: mavproxy.py [options]

Options:
  -h, --help            show this help message and exit
  --master=DEVICE[,BAUD]
                        MAVLink master port and optional baud rate
  --out=DEVICE[,BAUD]   MAVLink output port and optional baud rate
  --baudrate=BAUDRATE   default serial baud rate
  --sitl=SITL           SITL output port
  --streamrate=STREAMRATE
                        MAVLink stream rate
  --source-system=SOURCE_SYSTEM
                        MAVLink source system for this GCS
  --target-system=TARGET_SYSTEM
                        MAVLink target master system
  --target-component=TARGET_COMPONENT
                        MAVLink target master component
  --logfile=LOGFILE     MAVLink master logfile
  -a, --append-log      Append to log files
  --quadcopter          use quadcopter controls
  --setup               start in setup mode
  --nodtr               disable DTR drop on close
  --show-errors         show MAVLink error packets
  --speech              use text to speach
  --num-cells=NUM_CELLS
                        number of LiPo battery cells
  --aircraft=AIRCRAFT   aircraft name
  --cmd=CMD             initial commands
  --console             use GUI console
  --map                 load map module
  --load-module=LOAD_MODULE
                        Load the specified module. Can be used multiple times,
                        or with a comma separated list
  --mav09               Use MAVLink protocol 0.9
  --auto-protocol       Auto detect MAVLink protocol version
  --nowait              don't wait for HEARTBEAT on startup
  --continue            continue logs
  --dialect=DIALECT     MAVLink dialect
  --rtscts              enable hardware RTS/CTS flow control
  --mission=MISSION     mission name

```

Usually you want to launch `mavproxy` either through a serial or network interface:
- `mavproxy.py --master=/dev/ttyUSB0``
- `mavproxy.py --master=192.168.1.1:14550`


Sources:
- [MAVProxy GitHub.io Pages](http://tridge.github.io/MAVProxy/)
