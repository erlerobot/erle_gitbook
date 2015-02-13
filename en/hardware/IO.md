# More about I/O

###Inter Integrated Circuits (I²C)

I²C (Inter-Integrated Circuit)(alternately spelled I2C or IIC, most commonly pronounced I-squared-C) is a multimaster serial single-ended computer bus invented by the Philips semiconductor division, today NXP Semiconductors, and **used for attaching low-speed peripherals to a motherboard, embedded system, cellphone, or other digital electronic devices**.

Several competitors, such as Siemens AG (later Infineon Technologies AG, now Intel mobile communications), NEC, Texas Instruments, STMicroelectronics (formerly SGS-Thomson), Motorola (later Freescale), and Intersil, have introduced compatible I²C products to the market since the mid-1990s.

Since October 10, 2006, no licensing fees are required to implement the I²C protocol. However, fees are still required to obtain I²C slave addresses allocated by NXP.

#####Interesting remark
```
SMBus, defined by Intel in 1995, is a subset of I²C that defines the protocols more strictly. One purpose of SMBus is to promote robustness and interoperability. Accordingly, modern I²C systems incorporate policies and rules from SMBus, sometimes supporting both I²C and SMBus, requiring only minimal reconfiguration.
```

####I²C in the Erle-Brain

Our educational Erle-Brain includes 3 I²C busses available under `/dev`: `/dev/i2c-0`, `/dev/i2c-1` and `/dev/i2c-2`.

----

`/dev/i2c-2` is by default not activated. In order to activate it you should type:

``` bash
echo BB-I2C1 > $SLOTS
```

You can verify that it has been activated typing:
``` bash
cat $SLOTS
```
---

####Inspecting I²C

A good tool to inspect the I²C bus is the `i2cdetect` bash command:
```
i2cdetect -r 1
WARNING! This program can confuse your I2C bus, cause data loss and worse!
I will probe file /dev/i2c-1 using read byte commands.
I will probe address range 0x03-0x77.
Continue? [Y/n]
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- 13 -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- UU UU UU UU -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --

```

This command sends a ping to the `0x03`-`0x77` range (in hexadecimal) and records the answers in every address thereby it's able to figure out if there are any devices. Particularly we can see that the addresses `0x13` and `0x68` answered which correspond to the IR and IMU sensors respectively.

####Instantiate I²C devices from the userspace


The Linux kernel also allows to manually configure I²C devices through a sysfs interface:

> In general, the kernel should know which I2C devices are connected and
> what addresses they live at. However, in certain cases, it does not, so a
> sysfs interface was added to let the user provide the information. This
> interface is made of 2 attribute files which are created in every I2C bus
> directory: new_device and delete_device. Both files are write only and you
> must write the right parameters to them in order to properly instantiate,
> respectively delete, an I2C device.
>
> File new_device takes 2 parameters: the name of the I2C device (a string)
> and the address of the I2C device (a number, typically expressed in
> hexadecimal starting with 0x, but can also be expressed in decimal.)
>
> File delete_device takes a single parameter: the address of the I2C
> device. As no two devices can live at the same address on a given I2C
> segment, the address is sufficient to uniquely identify the device to be
> deleted.
>
> Example:
> ```echo eeprom 0x50 > /sys/bus/i2c/devices/i2c-3/new_device```
>
> While this interface should only be used when in-kernel device declaration
> can't be done, there is a variety of cases where it can be helpful:
> * The I2C driver usually detects devices (method 3 above) but the bus
>   segment your device lives on doesn't have the proper class bit set and
>   thus detection doesn't trigger.
> * The I2C driver usually detects devices, but your device lives at an
>   unexpected address.
> * The I2C driver usually detects devices, but your device is not detected,
>   either because the detection routine is too strict, or because your
>   device is not officially supported yet but you know it is compatible.
> * You are developing a driver on a test board, where you soldered the I2C
>   device yourself.
>
> This interface is a replacement for the force_* module parameters some I2C
> drivers implement. Being implemented in i2c-core rather than in each
> device driver individually, it is much more efficient, and also has the
> advantage that you do not have to reload the driver to change a setting.
> You can also instantiate the device before the driver is loaded or even
> available, and you don't need to know what driver the device needs.


Considering this, we could instantiate a sensor (hih6130) connected to `/dev/i2c-1` and with address `0x27` doing:

```
echo hih6130 0x27 > /sys/bus/i2c/devices/i2c-1/new_device
```

After that, the device will be available under `/sys/bus/i2c/drivers/hih6130/1-0027`.

To remove the device you can use:
```
echo 0x27 > /sys/bus/i2c/devices/i2c-1/delete_device
```

### Sources

- [Wikipedia](http://en.wikipedia.org/wiki/I%C2%B2C)
- [Linux kernel documentation](http://lxr.free-electrons.com/source/Documentation/i2c/instantiating-devices)
