# commands_process.pde

Link to the code: [commands_proccess](https://github.com/diydrones/ardupilot/blob/master/APMrover2/commands_process.pde)

This file contains only one function:
```cpp
RawBlameHistory
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// called by update navigation at 10Hz
// --------------------
static void update_commands(void)
{
    if(control_mode == AUTO) {
        if(home_is_set == true && mission.num_commands() > 1) {
            mission.update();
        }
    }
}
```
The `update_commands`function checks if the mode is AUTO, if the `home_is_set` and if there are mission commands to update.The [mission.update](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_Mission/AP_Mission.cpp#L166) is implemented in AP_Mission and ensures the command queues are loaded with the next command and calls main programs `command_init` and `command_verify` functions to progress the mission.
