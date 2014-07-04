# ArduPilot tasks

As its predecesor (ardupilot), BeaglePilot uses the following structure to schedule tasks within the autopilot (the following code can be located in `ArduCopter/ArduCopter.pde`, `ArduPlane/ArduPlane.pde` and `APMRover2/APMRover2.pde`). This is done in this way so that the *underlying real-time operative system* accomplishes these maximum latencies:

```
static const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
...
    { gcs_send_heartbeat, 100, 150 },
    { update_notify, 2, 100 },
    { one_hz_loop, 100, 420 },
    { gcs_check_input, 2, 550 },
    { gcs_send_heartbeat, 100, 150,
    { gcs_send_deferred, 2, 720 },
    { gcs_data_stream_send, 2, 950 },
...
}
```
- The *first parameter* is the function name,
- The *second* is the ‘time it is supposed to take’ in 10 ms units ( e.g.: 2 means 20ms which produces 50Hz thereby this function runs 50 times a second).
- The *third parameter* is the ‘max time beyond which the function should not run’.

