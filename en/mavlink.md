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
| Header | 3 | System ID,  what system is sending this message |
| Header | 4 | Component ID, what component of the system is sending the message |
| Header | 5 | Message ID (e.g. 0 = heartbeat) |
| *Payload* | 6 | Variable sized payload specified by the byte |
| *Payload* | 7 - 14 | Variable sized payload specified by the byte |
| **Checksum** | 15 - 16 | Checksum |






### Sources:
- MavLink Tutorial for Absolute Dummies (Part –I)
