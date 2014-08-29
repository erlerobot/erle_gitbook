# commands_logic.pde


Link to the code: [commands_logic.pde](https://github.com/BeaglePilot/ardupilot/blob/master/APMrover2/commands_logic.pde)

```cpp
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// forward declarations to make compiler happy
static void do_nav_wp(const AP_Mission::Mission_Command& cmd);
static void do_wait_delay(const AP_Mission::Mission_Command& cmd);
static void do_within_distance(const AP_Mission::Mission_Command& cmd);
static void do_change_speed(const AP_Mission::Mission_Command& cmd);
static void do_set_home(const AP_Mission::Mission_Command& cmd);
static bool verify_nav_wp(const AP_Mission::Mission_Command& cmd);

...
```
Here you can find the definitions for some instance of [AP_Mission](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_Mission/AP_Mission.h#L43) class.

```cpp

static bool
start_command(const AP_Mission::Mission_Command& cmd)
{
    // log when new commands start
    if (should_log(MASK_LOG_CMD)) {
        Log_Write_Cmd(cmd);
    }

    // exit immediately if not in AUTO mode
    if (control_mode != AUTO) {
        return false;
    }
    ...
    ````
`start_commadn`funtion enables command use.If `MASK_LOG_CMD ` is active then logs for commands.
Also checks if the mode is AUTO, if not exits.

```cpp
 gcs_send_text_fmt(PSTR("Executing command ID #%i"),cmd.id);

    // remember the course of our next navigation leg
    next_navigation_leg_cd = mission.get_next_ground_course_cd(0);

...
```
A message is send to the ground station.And the next navigation leg course is set.
```cpp

	switch(cmd.id){
		case MAV_CMD_NAV_WAYPOINT:	// Navigate to Waypoint
			do_nav_wp(cmd);
			break;

		case MAV_CMD_NAV_RETURN_TO_LAUNCH:
			do_RTL();
			break;

        // Conditional commands
		case MAV_CMD_CONDITION_DELAY:
			do_wait_delay(cmd);
			break;

		case MAV_CMD_CONDITION_DISTANCE:
			do_within_distance(cmd);
			break;

        // Do commands
		case MAV_CMD_DO_CHANGE_SPEED:
			do_change_speed(cmd);
			break;

		case MAV_CMD_DO_SET_HOME:
			do_set_home(cmd);
			break;
...
```
Depending on the `MAV_CMD` (mavlink commad) the corresponding funtion is called.
For example in case `MAV_CMD_NAV_WAYPOINT`is input `do_nav_wp` is called for leading the robot to the wp.
The same structure is followed in the next functions.
```cpp
case MAV_CMD_DO_SET_SERVO:
            ServoRelayEvents.do_set_servo(cmd.content.servo.channel, cmd.content.servo.pwm);
            break;

    	case MAV_CMD_DO_SET_RELAY:
            ServoRelayEvents.do_set_relay(cmd.content.relay.num, cmd.content.relay.state);
            break;

    	case MAV_CMD_DO_REPEAT_SERVO:
            ServoRelayEvents.do_repeat_servo(cmd.content.repeat_servo.channel, cmd.content.repeat_servo.pwm,
                                             cmd.content.repeat_servo.repeat_count, cmd.content.repeat_servo.cycle_time * 1000.0f);
            break;

    	case MAV_CMD_DO_REPEAT_RELAY:
            ServoRelayEvents.do_repeat_relay(cmd.content.repeat_relay.num, cmd.content.repeat_relay.repeat_count,
                                             cmd.content.repeat_relay.cycle_time * 1000.0f);
            break;
...
```
This options of the case follow the same structure, but the funtions implemented are related to [AP_ServoRelayEvents](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_ServoRelayEvents/AP_ServoRelayEvents.h#L33).

```cpp
#if CAMERA == ENABLED
        case MAV_CMD_DO_CONTROL_VIDEO:                      // Control on-board camera capturing. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|
            break;

        case MAV_CMD_DO_DIGICAM_CONFIGURE:                  // Mission command to configure an on-board camera controller system. |Modes: P, TV, AV, M, Etc| Shutter speed: Divisor number for one second| Aperture: F stop number| ISO number e.g. 80, 100, 200, Etc| Exposure type enumerator| Command Identity| Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)|
            break;

        case MAV_CMD_DO_DIGICAM_CONTROL:                    // Mission command to control an on-board camera controller system. |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Empty|
            do_take_picture();
            break;

        case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
            camera.set_trigger_distance(cmd.content.cam_trigg_dist.meters);
            break;
#endif
...
```
This slice of code checks if the CAMERA is enabled, if yes it enters in a case that includes commands for dealing with the camera.

```cpp
#if MOUNT == ENABLED
		// Sets the region of interest (ROI) for a sensor set or the
		// vehicle itself. This can then be used by the vehicles control
		// system to control the vehicle attitude and the attitude of various
		// devices such as cameras.
		//    |Region of interest mode. (see MAV_ROI enum)| Waypoint index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple cameras etc.)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|
        case MAV_CMD_DO_SET_ROI:
            if (cmd.content.location.alt == 0 && cmd.content.location.lat == 0 && cmd.content.location.lng == 0) {
                // switch off the camera tracking if enabled
                if (camera_mount.get_mode() == MAV_MOUNT_MODE_GPS_POINT) {
                    camera_mount.set_mode_to_default();
                }
            } else {
                // send the command to the camera mount
                camera_mount.set_roi_cmd(&cmd.content.location);
            }
            break;

		case MAV_CMD_DO_MOUNT_CONFIGURE:	// Mission command to configure a camera mount |Mount operation mode (see MAV_CONFIGURE_MOUNT_MODE enum)| stabilize roll? (1 = yes, 0 = no)| stabilize pitch? (1 = yes, 0 = no)| stabilize yaw? (1 = yes, 0 = no)| Empty| Empty| Empty|
			camera_mount.configure_cmd();
			break;

		case MAV_CMD_DO_MOUNT_CONTROL:		// Mission command to control a camera mount |pitch(deg*100) or lat, depending on mount mode.| roll(deg*100) or lon depending on mount mode| yaw(deg*100) or alt (in cm) depending on mount mode| Empty| Empty| Empty| Empty|
			camera_mount.control_cmd();
			break;
#endif
...
```
The same happend if the MOUNT is enabled; enters a case with command options related to the mount.

```cpp
default:
		    // return false for unhandled commands
		    return false;
	}

	// if we got this far we must have been successful
	return true;
}
...
```
The end of the case returns True if successful or false if not.

```cpp
// exit_mission - callback function called from ap-mission when the mission has completed
//      we double check that the flight mode is AUTO to avoid the possibility of ap-mission triggering actions while we're not in AUTO mode
static void exit_mission()
{
    if (control_mode == AUTO) {
        gcs_send_text_fmt(PSTR("No commands - setting HOLD"));
        set_mode(HOLD);
    }
}

...
```
`exit_mission()`function change to HOLD mode from AUTO.
```cpp
/********************************************************************************/
// Verify command Handlers
//      Returns true if command complete
/********************************************************************************/

static bool verify_command(const AP_Mission::Mission_Command& cmd)
{
    // exit immediately if not in AUTO mode
    // we return true or we will continue to be called by ap-mission
    if (control_mode != AUTO) {
        return true;
    }

	switch(cmd.id) {

		case MAV_CMD_NAV_WAYPOINT:
			return verify_nav_wp(cmd);

		case MAV_CMD_NAV_RETURN_TO_LAUNCH:
			return verify_RTL();

        case MAV_CMD_CONDITION_DELAY:
            return verify_wait_delay();
            break;

        case MAV_CMD_CONDITION_DISTANCE:
            return verify_within_distance();
            break;

        default:
            if (cmd.id > MAV_CMD_CONDITION_LAST) {
                // this is a command that doesn't require verify
                return true;
            }
            gcs_send_text_P(SEVERITY_HIGH,PSTR("verify_conditon: Unsupported command"));
            return true;
            break;
	}
    return false;
}
...
```
The `verify_command` funtion, first, checks id the mode is AUTO.Then enters a case with the `cmd.id`as variable. If non of the case options are selected a message is sent to the gcs.

```cpp
/********************************************************************************/
//  Nav (Must) commands
/********************************************************************************/

static void do_RTL(void)
{
    prev_WP = current_loc;
	control_mode 	= RTL;
	next_WP = home;
}

static void do_nav_wp(const AP_Mission::Mission_Command& cmd)
{
	set_next_WP(cmd.content.location);
}
...
```
This slice of code presents `do_RTL`funtion and `do_nav_wp`funtion.The first one updated the `prev_WP` and the `next_wp`in RTL `control_mode`.
The second function ,`set_next_WP` to the `location` value.

```cpp

/********************************************************************************/
//  Verify Nav (Must) commands
/********************************************************************************/
static bool verify_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    if ((wp_distance > 0) && (wp_distance <= g.waypoint_radius)) {
        gcs_send_text_fmt(PSTR("Reached Waypoint #%i dist %um"),
                          (unsigned)cmd.index,
                          (unsigned)get_distance(current_loc, next_WP));
        return true;
    }

    // have we gone past the waypoint?
    if (location_passed_point(current_loc, prev_WP, next_WP)) {
        gcs_send_text_fmt(PSTR("Passed Waypoint #%i dist %um"),
                          (unsigned)cmd.index,
                          (unsigned)get_distance(current_loc, next_WP));
        return true;
    }

    return false;
}
...
```
This slice of code includes `verify_nav_wp`funtion.If `wp_distance` is greater than 0 and `wp_distance` becomes equal or under the `g.waypoint_radius`value, the target has been reached.And if the `location_passed_point`fills the conditions the waypoint has been passed.
```cpp
static bool verify_RTL()
{
	if (wp_distance <= g.waypoint_radius) {
		gcs_send_text_P(SEVERITY_LOW,PSTR("Reached home"));
                rtl_complete = true;
		return true;
	}

    // have we gone past the waypoint?
    if (location_passed_point(current_loc, prev_WP, next_WP)) {
        gcs_send_text_fmt(PSTR("Reached Home dist %um"),
                          (unsigned)get_distance(current_loc, next_WP));
        return true;
    }

    return false;
}
...
```
The `verify_RTL` checks if the `wp_distance`is less than the `waypoint_radius` for sending a "reached home" message.Also checks if the robot has gone past the wp.

```cpp
/********************************************************************************/
//  Condition (May) commands
/********************************************************************************/

static void do_wait_delay(const AP_Mission::Mission_Command& cmd)
{
	condition_start = millis();
	condition_value  = cmd.content.delay.seconds * 1000;    // convert seconds to milliseconds
}

static void do_within_distance(const AP_Mission::Mission_Command& cmd)
{
	condition_value  = cmd.content.distance.meters;
}
...
````
`do_wait_delay` funtion change the `cmd.content.delay` to miliseconds and the `do_within_distance`
contains the value of the distance in meters.

```cpp

/********************************************************************************/
// Verify Condition (May) commands
/********************************************************************************/

static bool verify_wait_delay()
{
	if ((uint32_t)(millis() - condition_start) > (uint32_t)condition_value){
		condition_value 	= 0;
		return true;
	}
	return false;
}

static bool verify_within_distance()
{
	if (wp_distance < condition_value){
		condition_value = 0;
		return true;
	}
	return false;
}
...
```
The two functions in the code above code verify that the delay and the distance are 0.

```cpp
/********************************************************************************/
//  Do (Now) commands
/********************************************************************************/

static void do_change_speed(const AP_Mission::Mission_Command& cmd)
{
	switch (cmd.p1)
	{
		case 0:
			if (cmd.content.speed.target_ms > 0) {
				g.speed_cruise.set(cmd.content.speed.target_ms);
                gcs_send_text_fmt(PSTR("Cruise speed: %.1f m/s"), g.speed_cruise.get());
            }
			break;
	}

	if (cmd.content.speed.throttle_pct > 0 && cmd.content.speed.throttle_pct <= 100) {
		g.throttle_cruise.set(cmd.content.speed.throttle_pct);
        gcs_send_text_fmt(PSTR("Cruise throttle: %.1f"), g.throttle_cruise.get());
    }
}

...
```
This slice of code contains the `do_change_speed`
funtion.If the `cmd.p1` is equal to 0 sets the `speed_cruise`the `throttle_cruise`andsends messages to gcs.

```cpp
static void do_set_home(const AP_Mission::Mission_Command& cmd)
{
	if(cmd.p1 == 1 && have_position) {
		init_home();
	} else {
        ahrs.set_home(cmd.content.location);
		home_is_set = true;
	}
}

...
```
The funtion `do_set_home`alls the `init_home` defines in [commadns.pde](https://github.com/BeaglePilot/ardupilot/blob/master/APMrover2/commands.pde#L57) and updates `set_home` varible from attitude and heading reference system (ahrs).

```cpp
// do_take_picture - take a picture with the camera library
static void do_take_picture()
{
#if CAMERA == ENABLED
    camera.trigger_pic();
    if (should_log(MASK_LOG_CAMERA)) {
        DataFlash.Log_Write_Camera(ahrs, gps, current_loc);
    }
#endif
...
```
This last function takes a photo if the CAMERA is enabled. and stores it in dataflash.
