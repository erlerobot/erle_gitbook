# system.pde

The link to the code: [system.pde](https://github.com/diydrones/ardupilot/blob/master/APMrover2/system.pde).
This file contains funtions for the main control actions.

```cpp
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*****************************************************************************
The init_ardupilot function processes everything we need for an in - air restart
	We will determine later if we are actually on the ground and process a
	ground start in that case.

*****************************************************************************/

#if CLI_ENABLED == ENABLED

// Functions called from the top-level menu
static int8_t	process_logs(uint8_t argc, const Menu::arg *argv);	// in Log.pde
static int8_t	setup_mode(uint8_t argc, const Menu::arg *argv);	// in setup.pde
static int8_t	test_mode(uint8_t argc, const Menu::arg *argv);		// in test.cpp
static int8_t   reboot_board(uint8_t argc, const Menu::arg *argv);
...
```

Enables the Cli and definessome funtions that can be called from the main, as Menu class.

```cpp
// This is the help function
// PSTR is an AVR macro to read strings from flash memory
// printf_P is a version of print_f that reads from flash memory
static int8_t	main_menu_help(uint8_t argc, const Menu::arg *argv)
{
	cliSerial->printf_P(PSTR("Commands:\n"
						 "  logs        log readback/setup mode\n"
						 "  setup       setup mode\n"
						 "  test        test mode\n"
						 "\n"
						 "Move the slide switch and reset to FLY.\n"
						 "\n"));
	return(0);
}
...
```
This function defines a macro for the menu, which presents the posible options.

```cpp
// Command/function table for the top-level menu.
static const struct Menu::command main_menu_commands[] PROGMEM = {
//   command		function called
//   =======        ===============
	{"logs",		process_logs},
	{"setup",		setup_mode},
	{"test",		test_mode},
    {"reboot",      reboot_board},
	{"help",		main_menu_help}
};
...
```
This slice of code presents a table for the menu options.

```cpp
// Create the top-level menu object.
MENU(main_menu, THISFIRMWARE, main_menu_commands);

static int8_t reboot_board(uint8_t argc, const Menu::arg *argv)
{
    hal.scheduler->reboot(false);
    return 0;
}

...
```
Here the main `MENU` is implementes and  the schedules [reboot](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_HAL/Scheduler.h#L46) funtion is disabled.

```cpp
// the user wants the CLI. It never exits
static void run_cli(AP_HAL::UARTDriver *port)
{
    // disable the failsafe code in the CLI
    hal.scheduler->register_timer_failsafe(NULL,1);

    // disable the mavlink delay callback
    hal.scheduler->register_delay_callback(NULL, 5);

    cliSerial = port;
    Menu::set_port(port);
    port->set_blocking_writes(true);

    while (1) {
        main_menu.run();
    }
}

#endif // CLI_ENABLED
...
```
This slice of code ensures to have the ClI enabled, updating the values of the necessary funtions.

```cpp
static void init_ardupilot()
{
...
```
ìnit_ardupilot`can be considered the most important function over this file.
	// Console serial port
	//This function preocesses everything needed to have the robot Ready to Drive.

```cpp
	// The console port buffers are defined to be sufficiently large to support
	// the console's use as a logging device, optionally as the GPS port when
	// GPS_PROTOCOL_IMU is selected, and as the telemetry port.
	//
	// XXX This could be optimised to reduce the buffer sizes in the cases
	// where they are not otherwise required.
	//
    hal.uartA->begin(SERIAL0_BAUD, 128, 128);

	// GPS serial port.
	//
	// XXX currently the EM406 (SiRF receiver) is nominally configured
	// at 57600, however it's not been supported to date.  We should
	// probably standardise on 38400.
	//
	// XXX the 128 byte receive buffer may be too small for NMEA, depending
	// on the message set configured.
	//
    // standard gps running
    hal.uartB->begin(38400, 256, 16);
...
```
Here you can fnd the specifications needed.
```cppp
#if GPS2_ENABLE
    if (hal.uartE != NULL) {
        hal.uartE->begin(38400, 256, 16);
    }
#endif
...
```
Checks if the GPS2 is enabled and if yes enable the uartE devide.


```cpp
	cliSerial->printf_P(PSTR("\n\nInit " FIRMWARE_STRING
						 "\n\nFree RAM: %u\n"),
                        hal.util->available_memory());

	//
	// Check the EEPROM format version before loading any parameters from EEPROM.
	//
...
```
Checks the EEPROM status for loading parameters later.

```cpp
    load_parameters();

    BoardConfig.init();

    ServoRelayEvents.set_channel_mask(0xFFF0);

    set_control_channels();
...
```
Set the parameters, the board configuration and some others.

```cpp
    // after parameter load setup correct baud rate on uartA
    hal.uartA->begin(map_baudrate(g.serial0_baud));

    // keep a record of how many resets have happened. This can be
    // used to detect in-flight resets
    g.num_resets.set_and_save(g.num_resets+1);
...
```
Here takes care of the number of reboots.

```cpp
    // init baro before we start the GCS, so that the CLI baro test works
    barometer.init();

	// init the GCS
	gcs[0].init(hal.uartA);

    // we start by assuming USB connected, as we initialed the serial
    // port with SERIAL0_BAUD. check_usb_mux() fixes this if need be.
    usb_connected = true;
    check_usb_mux();

    // we have a 2nd serial port for telemetry
    gcs[1].setup_uart(hal.uartC, map_baudrate(g.serial1_baud), 128, 128);

...
```
Confgure and enable all ports and devices.

```cpp
#if MAVLINK_COMM_NUM_BUFFERS > 2
    if (g.serial2_protocol == SERIAL2_FRSKY_DPORT ||
        g.serial2_protocol == SERIAL2_FRSKY_SPORT) {
        frsky_telemetry.init(hal.uartD, g.serial2_protocol);
    } else {
        gcs[2].setup_uart(hal.uartD, map_baudrate(g.serial2_baud), 128, 128);
    }
#endif

	mavlink_system.sysid = g.sysid_this_mav;
	...
	```
This slice of code stablish the MAVLINK protocol status.
```cpp

#if LOGGING_ENABLED == ENABLED
	DataFlash.Init(log_structure, sizeof(log_structure)/sizeof(log_structure[0]));
    if (!DataFlash.CardInserted()) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("No dataflash card inserted"));
        g.log_bitmask.set(0);
    } else if (DataFlash.NeedErase()) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("ERASING LOGS"));
		do_erase_logs();
    }
	if (g.log_bitmask != 0) {
		start_logging();
	}
#endif
...
```
Here the logs recording is initialized.
```cpp

    // Register mavlink_delay_cb, which will run anytime you have
    // more than 5ms remaining in your call to hal.scheduler->delay
    hal.scheduler->register_delay_callback(mavlink_delay_cb, 5);
...
```
This slice of code register the delay status.

```cpp
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
    apm1_adc.Init();      // APM ADC library initialization
#endif

	if (g.compass_enabled==true) {
		if (!compass.init()|| !compass.read()) {
            cliSerial->println_P(PSTR("Compass initialisation failed!"));
            g.compass_enabled = false;
        } else {
            ahrs.set_compass(&compass);
            //compass.get_offsets();						// load offsets to account for airframe magnetic interference
        }
	}

...
```
If the board is the correct one, the compass data set ups are done.

```cpp
// initialise sonar
    init_sonar();

    // and baro for EKF
    init_barometer();

	// Do GPS init
	gps.init(&DataFlash);
...
```
Here initialized the sonar, the GPs and the barometer.

```cpp

	//mavlink_system.sysid = MAV_SYSTEM_ID;				// Using g.sysid_this_mav
	mavlink_system.compid = 1;	//MAV_COMP_ID_IMU;   // We do not check for comp id
	mavlink_system.type = MAV_TYPE_GROUND_ROVER;

...
```
This slice of code take care of some `mavlink_system` variables.

```cpp
    rc_override_active = hal.rcin->set_overrides(rc_override, 8);

	init_rc_in();		// sets up rc channels from radio
	init_rc_out();		// sets up the timer libs

    relay.init();
...
```
The RC I/O are initialized.

```cpp
/*
      setup the 'main loop is dead' check. Note that this relies on
      the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check, 1000);

	// If the switch is in 'menu' mode, run the main menu.
	//
	// Since we can't be sure that the setup or test mode won't leave
	// the system in an odd state, we don't let the user exit the top
	// menu; they must reset in order to fly.
	//
    const prog_char_t *msg = PSTR("\nPress ENTER 3 times to start interactive setup\n");
    cliSerial->println_P(msg);
    if (gcs[1].initialised) {
        hal.uartC->println_P(msg);
    }
...
```
the main loop begins here.With three enters (which mean `num_gcs`>2) the interactive mode is initialized.  
```cpp
    if (num_gcs > 2 && gcs[2].initialised) {
        hal.uartD->println_P(msg);
    }

	startup_ground();

	if (should_log(MASK_LOG_CMD)) {
        Log_Write_Startup(TYPE_GROUNDSTART_MSG);
    }

    set_mode((enum mode)g.initial_mode.get());

	// set the correct flight mode
	// ---------------------------
	reset_control_switch();
}
...
```
The control mode is set, by using the switch.

```cpp
//********************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//********************************************************************************
static void startup_ground(void)
{
    set_mode(INITIALISING);

	gcs_send_text_P(SEVERITY_LOW,PSTR("<startup_ground> GROUND START"));

	#if(GROUND_START_DELAY > 0)
		gcs_send_text_P(SEVERITY_LOW,PSTR("<startup_ground> With Delay"));
		delay(GROUND_START_DELAY * 1000);
	#endif

	//IMU ground start
	//------------------------
    //

	startup_INS_ground(false);

	// read the radio to set trims
	// ---------------------------
	trim_radio();

    // initialise mission library
    mission.init();

    hal.uartA->set_blocking_writes(false);
    hal.uartB->set_blocking_writes(false);
    hal.uartC->set_blocking_writes(false);

	gcs_send_text_P(SEVERITY_LOW,PSTR("\n\n Ready to drive."));
}
...
```
This fucntions launch the `GROUND_START`, reads the radio trims, initilized mission() and blocks the write in the uartA drivers.

```cpp

/*
  set the in_reverse flag
  reset the throttle integrator if this changes in_reverse
 */
static void set_reverse(bool reverse)
{
    if (in_reverse == reverse) {
        return;
    }
    g.pidSpeedThrottle.reset_I();
    in_reverse = reverse;
}

...
```
Resets the throttle and changes the values form the `in_reverse`and `reverse`functions.

```cpp
static void set_mode(enum mode mode)
{

	if(control_mode == mode){
		// don't switch modes if we are already in the correct mode.
		return;
	}
	...
	```
Checks if the `control_mode`set is the correct one.
```cpp
	control_mode = mode;
    throttle_last = 0;
    throttle = 500;
    set_reverse(false);
    g.pidSpeedThrottle.reset_I();
...
```
Resets the throttle and some variables values.

```cpp
if (control_mode != AUTO) {
        auto_triggered = false;
    }
...
```
Disables the `auto_trittered`in case the mode is AUTO

```
	switch(control_mode)
	{
		case MANUAL:
		case HOLD:
		case LEARNING:
		case STEERING:
			break;

		case AUTO:
            rtl_complete = false;
            restart_nav();
			break;

		case RTL:
			do_RTL();
			break;

		default:
			do_RTL();
			break;
	}
...
```
Enters a case where the modes are set.

```cpp
	if (should_log(MASK_LOG_MODE)) {
		Log_Write_Mode();
    }
}
...
```
Writes the log corrsponding to the selected mode.

```cpp
/*
  called to set/unset a failsafe event.
 */
static void failsafe_trigger(uint8_t failsafe_type, bool on)
{
    uint8_t old_bits = failsafe.bits;
    if (on) {
        failsafe.bits |= failsafe_type;
    } else {
        failsafe.bits &= ~failsafe_type;
    }
...
```
Initializes the values of some variables.

```cpp
    if (old_bits == 0 && failsafe.bits != 0) {
        // a failsafe event has started
        failsafe.start_time = millis();
    }
    ...
    ```
Verifies if the failsafe event has started
```cpp
    if (failsafe.triggered != 0 && failsafe.bits == 0) {
        // a failsafe event has ended
        gcs_send_text_fmt(PSTR("Failsafe ended"));
    }
...
```
Verifies when the failsafe envent has finished.

```cpp
    failsafe.triggered &= failsafe.bits;

    if (failsafe.triggered == 0 &&
        failsafe.bits != 0 &&
        millis() - failsafe.start_time > g.fs_timeout*1000 &&
        control_mode != RTL &&
        control_mode != HOLD) {
        failsafe.triggered = failsafe.bits;
        gcs_send_text_fmt(PSTR("Failsafe trigger 0x%x"), (unsigned)failsafe.triggered);
        switch (g.fs_action) {
        case 0:
            break;
        case 1:
            set_mode(RTL);
            break;
        case 2:
            set_mode(HOLD);
            break;
        }
    }
}
...
```
Checks some conditions ans enters a case, for changing to the appropiate mode.

```cpp
static void startup_INS_ground(bool force_accel_level)
{
    gcs_send_text_P(SEVERITY_MEDIUM, PSTR("Warming up ADC..."));
 	mavlink_delay(500);

	// Makes the servos wiggle twice - about to begin INS calibration - HOLD LEVEL AND STILL!!
	// -----------------------
    gcs_send_text_P(SEVERITY_MEDIUM, PSTR("Beginning INS calibration; do not move vehicle"));
	mavlink_delay(1000);

    ahrs.init();
	ahrs.set_fly_forward(true);
    ahrs.set_vehicle_class(AHRS_VEHICLE_GROUND);
...
```
Sets the AHRS needed calibrations
```cpp
    AP_InertialSensor::Start_style style;
    if (g.skip_gyro_cal && !force_accel_level) {
        style = AP_InertialSensor::WARM_START;
    } else {
        style = AP_InertialSensor::COLD_START;
    }

	ins.init(style, ins_sample_rate);
...
```
If everything is all rigth wamrs the inertialsensors ans init the INS.
```cpp
    if (force_accel_level) {
        // when MANUAL_LEVEL is set to 1 we don't do accelerometer
        // levelling on each boot, and instead rely on the user to do
        // it once via the ground station
        ins.init_accel();
        ahrs.set_trim(Vector3f(0, 0, 0));
	}
    ahrs.reset();
}
...
```
If the `MANUAL_LEVEL` is enables  the  INS  `init_accel`and AHRS are updated.

```cpp

// updates the notify state
// should be called at 50hz
static void update_notify()
{
    notify.update();
}

...
```
Updates the notify state.

```cpp
static void resetPerfData(void) {
	mainLoop_count 			= 0;
	G_Dt_max 				= 0;
	perf_mon_timer 			= millis();
}
...
```

Resets some variables.
```cpp

static void check_usb_mux(void)
{
    bool usb_check = hal.gpio->usb_connected();
    if (usb_check == usb_connected) {
        return;
    }

    // the user has switched to/from the telemetry port
    usb_connected = usb_check;

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    // the APM2 has a MUX setup where the first serial port switches
    // between USB and a TTL serial connection. When on USB we use
    // SERIAL0_BAUD, but when connected as a TTL serial port we run it
    // at SERIAL1_BAUD.
    if (usb_connected) {
        hal.uartA->begin(SERIAL0_BAUD);
    } else {
        hal.uartA->begin(map_baudrate(g.serial1_baud));
    }
#endif
}

...
```
Checks if the usb is connected and begins it as a uartA device.

```cpp
static void
print_mode(AP_HAL::BetterStream *port, uint8_t mode)
{
    switch (mode) {
    case MANUAL:
        port->print_P(PSTR("Manual"));
        break;
    case HOLD:
        port->print_P(PSTR("HOLD"));
        break;
    case LEARNING:
        port->print_P(PSTR("Learning"));
        break;
    case STEERING:
        port->print_P(PSTR("Steering"));
        break;
    case AUTO:
        port->print_P(PSTR("AUTO"));
        break;
    case RTL:
        port->print_P(PSTR("RTL"));
        break;
    default:
        port->printf_P(PSTR("Mode(%u)"), (unsigned)mode);
        break;
    }
}
...
```
Print a message with the current mode.

```cpp
/*
  check a digitial pin for high,low (1/0)
 */
static uint8_t check_digital_pin(uint8_t pin)
{
    int8_t dpin = hal.gpio->analogPinToDigitalPin(pin);
    if (dpin == -1) {
        return 0;
    }
    // ensure we are in input mode
    hal.gpio->pinMode(dpin, HAL_GPIO_INPUT);

    // enable pullup
    hal.gpio->write(dpin, 1);

    return hal.gpio->read(dpin);
}
...
```

Ensures that the pin is in input mode and read the status to check it.

```cpp
/*
  write to a servo
 */
static void servo_write(uint8_t ch, uint16_t pwm)
{
#if HIL_MODE != HIL_MODE_DISABLED
    if (ch < 8) {
        RC_Channel::rc_channel(ch)->radio_out = pwm;
    }
#else
    hal.rcout->enable_ch(ch);
    hal.rcout->write(ch, pwm);
#endif
}
...
```
If the HIL mode is enabled the radio output is set to pwm, using `radio_out`function, if not other procedure is followed to do the same: ènable_ch`and `write`on it.
```cpp
/*
  should we log a message type now?
 */
static bool should_log(uint32_t mask)
{
    if (!(mask & g.log_bitmask) || in_mavlink_delay) {
        return false;
    }
    bool ret = ahrs.get_armed() || (g.log_bitmask & MASK_LOG_WHEN_DISARMED) != 0;
    if (ret && !DataFlash.logging_started() && !in_log_download) {
        // we have to set in_mavlink_delay to prevent logging while
        // writing headers
        in_mavlink_delay = true;
        start_logging();
        in_mavlink_delay = false;
    }
    return ret;
}
...
```
Checks the status of the bitmask and the mavlink protocol delay.A `in_mavlink_delay`is send to prevent logging while, and after that the `start_logging`function is called.
```cpp
/*
  send FrSky telemetry. Should be called at 5Hz by scheduler
 */
static void telemetry_send(void)
{
#if FRSKY_TELEM_ENABLED == ENABLED
    frsky_telemetry.send_frames((uint8_t)control_mode,
                                (AP_Frsky_Telem::FrSkyProtocol)g.serial2_protocol.get());
#endif
```
The fr Sky for telemetry is enabled and the info is read.
