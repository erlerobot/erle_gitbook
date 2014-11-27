# MAVProxy: Un software para una Estación base de UAV basado en MAVLink

MAVProxy es una estación de control de tierra con funcionalidad completa para UAVs (vehículos no tripulados). La intención es tener una Estación de Control de Tierra (GCS) minimalista, portable y extensible para cualquier UAV que soporte el protocolo MAVLink.

### Características:

- Se trata de una aplicación de linea de comandos. Hay un *plugin* incluido en MAVProxy que proporcionado un GUI.
- Puede ser conectado por red y ejecutar a través de varias máquinas.
- Es portable; debería ejecutar en cualquier sistema operativo _POSIX_ con `python`, `pyserial`, y llamadas al sistema `select()`, lo que significa _Linux_, _OS X_, _Windows_, y otros.
- El *diseño ligero* significa que se puede ejecutar en netbooks pequeños con facilidad.
- Puede cargar ciertos módulos, tiene móduclos de apoyo a la consola, mapas, joysticks, rastreadores de antena, etc. 
- Autocompletado de comando con el tabulador

Tu puede ejecutar `mavproxy` con las siguiente opcicones:

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

Usualmente se ejecutará `mavproxy` ya sea a través de un interfaz serie o de red:
- `mavproxy.py --master=/dev/ttyUSB0`
- `mavproxy.py --master=192.168.1.1:14550`


Fuentes:
- [MAVProxy GitHub.io Pages](http://tridge.github.io/MAVProxy/)
