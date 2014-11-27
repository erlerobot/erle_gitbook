# sensors.pde


Link to the code: [sensors.pde](https://github.com/diydrones/ardupilot/blob/master/APMrover2/sensors.pde).This file includes functions for enabling and dealing with sensors.

```cpp

// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static void init_barometer(void)
{
    gcs_send_text_P(SEVERITY_LOW, PSTR("Calibrating barometer"));
    barometer.calibrate();
    gcs_send_text_P(SEVERITY_LOW, PSTR("barometer calibration complete"));
}


...
```
This function initialize the barometer, calls the [calibrate()](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_Baro/AP_Baro.cpp#L59) function from `AP_Baro` and sent a gcs message to inform.

```cpp
static void init_sonar(void)
{
    sonar.init();
}
...
```
Initialize the sonar.

```cpp
// read_battery - reads battery voltage and current and invokes failsafe
// should be called at 10hz
static void read_battery(void)
{
    battery.read();
}
...
```
Reads the battery status.
```cpp
// read the receiver RSSI as an 8 bit number for MAVLink
// RC_CHANNELS_SCALED message
void read_receiver_rssi(void)
{
    rssi_analog_source->set_pin(g.rssi_pin);
    float ret = rssi_analog_source->voltage_average() * 50;
    receiver_rssi = constrain_int16(ret, 0, 255);
}
...
```
Sets the pin for reading the RSSI analog signal.And red the `voltage_average()`registered.

```cpp

// read the sonars
static void read_sonars(void)
{
    sonar.update();

...
```
This function will read the data recordered by the sonar. first of all the sonar need to be up-to-date.
```cpp
    if (!sonar.healthy()) {
        // this makes it possible to disable sonar at runtime
        return;
    }
...
```
If the sonar is ok, we can continu, if not get out.

```cpp

    if (sonar.healthy(1)) {
        // we have two sonars
        obstacle.sonar1_distance_cm = sonar.distance_cm(0);
        obstacle.sonar2_distance_cm = sonar.distance_cm(1);

...
```
If the sonar is health, the distance to the objects is recordered.Take into account thisis the case when there are **two sonars**.
```cpp
        if (obstacle.sonar1_distance_cm <= (uint16_t)g.sonar_trigger_cm &&
            obstacle.sonar2_distance_cm <= (uint16_t)obstacle.sonar2_distance_cm)  {
            // we have an object on the left
            if (obstacle.detected_count < 127) {
                obstacle.detected_count++;
            }
            if (obstacle.detected_count == g.sonar_debounce) {
                gcs_send_text_fmt(PSTR("Sonar1 obstacle %u cm"),
                                  (unsigned)obstacle.sonar1_distance_cm);
            }
            obstacle.detected_time_ms = hal.scheduler->millis();
            obstacle.turn_angle = g.sonar_turn_angle;
        ...
        ```
If the conditions above are suplied, that means that there is an object on the rigth.A gcs message is sent and the `tun_angle`function is called to avoid it.

```cpp
        } else if (obstacle.sonar2_distance_cm <= (uint16_t)g.sonar_trigger_cm) {
            // we have an object on the right
            if (obstacle.detected_count < 127) {
                obstacle.detected_count++;
            }
            if (obstacle.detected_count == g.sonar_debounce) {
                gcs_send_text_fmt(PSTR("Sonar2 obstacle %u cm"),
                                  (unsigned)obstacle.sonar2_distance_cm);
            }
            obstacle.detected_time_ms = hal.scheduler->millis();
            obstacle.turn_angle = -g.sonar_turn_angle;
        }
...
```
Here, the same as above is done, but take into account the conditions: here the obstacle is on the rigth.That's way the `sonar_turn_angle`cames with the - sign.
```cpp
    } else {
        // we have a single sonar
        obstacle.sonar1_distance_cm = sonar.distance_cm(0);
        obstacle.sonar2_distance_cm = 0;
        if (obstacle.sonar1_distance_cm <= (uint16_t)g.sonar_trigger_cm)  {
            // obstacle detected in front
            if (obstacle.detected_count < 127) {
                obstacle.detected_count++;
            }
            if (obstacle.detected_count == g.sonar_debounce) {
                gcs_send_text_fmt(PSTR("Sonar obstacle %u cm"),
                                  (unsigned)obstacle.sonar1_distance_cm);
            }
            obstacle.detected_time_ms = hal.scheduler->millis();
            obstacle.turn_angle = g.sonar_turn_angle;
        }
    }
...
```
The same as above is done here, but for the case when there is a **single sonar**.

```cpp
    Log_Write_Sonar();

    // no object detected - reset after the turn time
    if (obstacle.detected_count >= g.sonar_debounce &&
        hal.scheduler->millis() > obstacle.detected_time_ms + g.sonar_turn_time*1000) {
        gcs_send_text_fmt(PSTR("Obstacle passed"));
        obstacle.detected_count = 0;
        obstacle.turn_angle = 0;
    }
}
```
Here is set the case when there is no obstacle.Then the values of `turn_angle`and `detected_count`are reseted for the next call to the sonar.
