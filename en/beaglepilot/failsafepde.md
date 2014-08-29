# failsafe.pde


The [failsafe.pde](https://github.com/BeaglePilot/ardupilot/blob/master/APMrover2/failsafe.pde) file contains functions and variables needed for *failsafe strategy*, that consist on detecting main loop lockup.


```cpp
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
  failsafe support
  Andrew Tridgell, December 2011
 */

/*
  our failsafe strategy is to detect main loop lockup and switch to
  passing inputs straight from the RC inputs to RC outputs.
 */

/*
  this failsafe_check function is called from the core timer interrupt
  at 1kHz.
 */
 ...
 ```
 Here is the intro and use recommendations.
 ```cpp
 static void failsafe_check()
{
    static uint16_t last_mainLoop_count;
    static uint32_t last_timestamp;
    static bool in_failsafe;
    uint32_t tnow = hal.scheduler->micros();
    ...
    ```
The file starts with the definition of the `failsafe_check`function and the definition of some internal variables of these function.
```cpp
    if (mainLoop_count != last_mainLoop_count) {
        // the main loop is running, all is OK
        last_mainLoop_count = mainLoop_count;
        last_timestamp = tnow;
        in_failsafe = false;
        return;
    }
    ...
    ```
This slice of code checks if the main loop is running and there is no lockup.
```cpp

    if (tnow - last_timestamp > 200000) {
        // we have gone at least 0.2 seconds since the main loop
        // ran. That means we're in trouble, or perhaps are in
        // an initialisation routine or log erase. Start passing RC
        // inputs through to outputs
        in_failsafe = true;
    }
...
```
If the (tnow-last_timestamp) value is greater than 200 ms, there is a lockup. The RC inputs are passed to the outputs.
```cpp
    if (in_failsafe && tnow - last_timestamp > 20000 &&
        channel_throttle->read() >= (uint16_t)g.fs_throttle_value) {
        // pass RC inputs to outputs every 20ms
        last_timestamp = tnow;
        hal.rcin->clear_overrides();
        uint8_t start_ch = 0;
        for (uint8_t ch=start_ch; ch<4; ch++) {
            hal.rcout->write(ch, hal.rcin->read(ch));
        }
        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_manual, true);
    }
}

```
If in adition to the value greater than 200 ms the `channel_throttle` is greater then the `fs_throttle_value` RC inputs are passed to outputs every 20 ms; also the overlays are erased.
