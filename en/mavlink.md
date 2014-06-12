# MAVLink

 MavLink is a communication protocol for MAV (Micro Aerial Vehicles) that has nowadays been extended to all kind of drones (both aerial and terrestrial).


### MAVLink message

The MAVLink message is basically a stream of bytes encoded and sent over some kind of transductor (via USB serial, RC frequencies, WiFi, GPRS, etc.). By encoding we mean  to put the packet into a data structure in a *smart way* adding **checksums**, **sequence numbers** and send it via the channel byte by byte.

#### Structure of the message

Each MAVLink message is structured in the following way:

```
message length = 17 bytes (6 bytes header + 9 bytes payload + 2 bytes checksum)
```

The following table describes the message parts and bytes:

| **Message part** | **Byte** | **Description** |
|------------------|----------|-----------------|
| Header | 0 | message header, always `0xFE`|
| Header | 1 | Message length |
| Header | 2 | Sequence number, rolls from 0 to 255 |
| Header | 3 | System ID,  This is the source (i.e. Mission planner) sending a message. The software does a regular check so as to know that this message is for itself. |
| Header | 4 | Component ID, what component of the system is sending the message |
| Header | 5 | Message ID (e.g. 0 = heartbeat). What is this message about. |
| *Payload* | 6 | Variable sized payload specified by the byte |
| *Payload* | 7 - 14 | Variable sized payload specified by the byte |
| **Checksum** | 15 - 16 | Checksum |


### MAVLink in action

Micro Aerial Vehicle Link (MavLink) as the name suggests, isn’t quite a true name is currently not us It can be used with ground robots as well. It was named this way, because it started with copters (if my conjecture is correct).
The ‘message’ is a data bundle that contains a ‘constant number of bytes’ (i.e. 17, as already described). APM gets streaming bytes (from the air), gets it to the hardware interface (e.g. via UART serial OR Telemetry) and decodes the message in software. Note that the message contains the payload, which we would extract.
We are interested in the payload, but hey, along with the payload is the message ID (see above) to know what it represents. Before all this, these are a few steps on how the code interprets any MavLink message:
1) We have a method called handlemessage (msg). This is what you need to learn and know! Go and hunt for it in GCS_MavLink.pde (in Arducopter/ ArduPlane).
It basically asks the packet: Hey, who are you packet? Are you meant for me or trying to hack into my system? Let me read the System ID and Component ID of yours first, before I allow you. Any system using MavLink has a System ID and Component ID. E.g. the MP and Quadcopter you are flying will have the same System ID to begin with. The Component ID is for a ‘sub system’ attached to the APM/PX4.
Note: Currently, the System ID and Component ID are hardcoded to be the same.


### Sources:
- [MavLink Tutorial for Absolute Dummies (Part –I)](http://api.ning.com/files/i*tFWQTF2R*7Mmw7hksAU-u9IABKNDO9apguOiSOCfvi2znk1tXhur0Bt00jTOldFvob-Sczg3*lDcgChG26QaHZpzEcISM5/MAVLINK_FOR_DUMMIESPart1_v.1.1.pdf)
