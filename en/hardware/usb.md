# USB


### USB0 Port
The HUB connects direct to the USB0 port on the processor. This allows that port to be accessible from the same USB connector as the Serial and JTAG ports.

#### USB Client Port
Access to USB0 is provided via the onboard USB Hub. It will show up on a PC as a standard USB serial device (usually `/dev/tty[something]`).

### USB1 Port
On the board is a single USB Type A connector with full LS/FS/HS Host support that connects to USB1 on the processor. The port can provide power on/off control and **up to 500mA** of current at 5V.


```
Under USB power, the board will not be able to supply the full 500mA, but should be sufficient to supply enough current for a lower power USB device.
If more current is needed we recommend using a battery.

You can use a wireless keyboard/mouse/screen configuration or you can add a HUB for interfacing with devices if required.

```
