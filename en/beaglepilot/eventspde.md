# events.pde


Link to the code:[events.pde](https://github.com/BeaglePilot/ardupilot/blob/master/APMrover2/events.pde). This file includes a unique function:

```cpp
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


static void update_events(void)
{
    ServoRelayEvents.update_events();
}
```
This funtion, `update_events`calls the `update_events()`function form the [AP_ServoRElayEvents](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_ServoRelayEvents/AP_ServoRelayEvents.h#L36) for updating these events.

