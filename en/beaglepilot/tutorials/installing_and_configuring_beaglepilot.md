# Installing and Configuring BeaglePilot

As stated before, [BeaglePilot](https://github.com/BeaglePilot/ardupilot) relies on the `AP_HAL_Linux` of [ardupilot](https://github.com/diydrones/ardupilot).

Installing BeaglePilot is as easy as cloning the code and making it:

```
git clone https://github.com/BeaglePilot/ardupilot
cd ardupilot
```
[ardupilot](https://github.com/diydrones/ardupilot) supports for the time being three kind of vehicles: *copters*, *planes* and *rovers*. Let's focus on *copters* for now:
```
cd ArduCopter
```
Now we generate some config files (just needed once):
```
make configure
```
And finally we compile the code:
```
make erle
```
You could target other boards, no only `erle`. At the time of writting, `pxf` is also supported.

A log of the compilation process is provided [here](https://gist.github.com/vmayoral/4b78947bfa32f7446549).

----

**In a BeagleBone (white) running at 720 MHz (perf. mode) the compilation time is about 10 minutes.**

----

After the compilation, you should have an `ArduCopter.build/ArduCopter.elf` at the `/tmp` directory.

----

If you wish to deploy the executables somewhere else define the enviromental variable `TMPDIR`. This variable might be defined already if you use one of our images.

----
