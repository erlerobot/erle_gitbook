# comands.pde

Link to the code: [commands.pde](https://github.com/BeaglePilot/ardupilot/blob/master/APMrover2/commands.pde)

```cpp
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* Functions in this file:
	void set_next_WP(const AP_Mission::Mission_Command& cmd)
	void set_guided_WP(void)
	void init_home()
	void restart_nav()
************************************************************
*/
...
```
This is the beginning of the `commands.pde`file and here are mentiones the functions contained by this file. We will see the implementation rigth now.

```cpp

/*
 *  set_next_WP - sets the target location the vehicle should fly to
 */
static void set_next_WP(const struct Location& loc)
{
	// copy the current WP into the OldWP slot
	// ---------------------------------------
	prev_WP = next_WP;

	// Load the next_WP slot
	// ---------------------
	next_WP = loc;

    // are we already past the waypoint? This happens when we jump
    // waypoints, and it can cause us to skip a waypoint. If we are
    // past the waypoint when we start on a leg, then use the current
    // location as the previous waypoint, to prevent immediately
    // considering the waypoint complete
    if (location_passed_point(current_loc, prev_WP, next_WP)) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("Resetting prev_WP"));
        prev_WP = current_loc;
    }

	// this is handy for the groundstation
	wp_totalDistance 	= get_distance(current_loc, next_WP);
	wp_distance 		= wp_totalDistance;
}
...
```
This slice of code sets the next destination for the vehicle.
First copies the current waypoint, contained in `next_WP` into the variable `prev_WP`and then updates the `next_WP`to the next WP. This values are passed to [location_passed_point](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_Math/location.cpp#L83). This function is defined in `location.cpp` file, part of the `AP_Math`library.This function checks if point1 is our previous waypoint and point2 is our target waypoint then this function returns true if we have flown past the target waypoint.
After this the distance to the target is calculated through [get_distance](https://github.com/diydrones/ardupilot/blob/master/libraries/AP_Math/location.cpp#L55) function.
```cpp

static void set_guided_WP(void)
{
	// copy the current location into the OldWP slot
	// ---------------------------------------
	prev_WP = current_loc;

	// Load the next_WP slot
	// ---------------------
	next_WP = guided_WP;

	// this is handy for the groundstation
	wp_totalDistance 	= get_distance(current_loc, next_WP);
	wp_distance 		= wp_totalDistance;
}
...
```
This function does the same as the previous one, but without checking `location_passed_point`.

```cpp

// run this at setup on the ground
// -------------------------------
void init_home()
{
    if (!have_position) {
        // we need position information
        return;
    }

	gcs_send_text_P(SEVERITY_LOW, PSTR("init home"));

    ahrs.set_home(gps.location());
	home_is_set = true;

	// Save Home to EEPROM
	mission.write_home_to_storage();

	// Save prev loc
	// -------------
	next_WP = prev_WP = home;

	// Load home for a default guided_WP
	// -------------
	guided_WP = home;
}
...
```
The following function is `init_home`, this function checks if `have_position`variable contains the position.Then sets the "home point" as the current `gps.location()`.Then the home is stored in the memory and the `next_WP` and `prev_WP` are updated to home value. Then `guided_WP`is updated to home.
```cpp
static void restart_nav()
{
    g.pidSpeedThrottle.reset_I();
    prev_WP = current_loc;
    mission.start_or_resume();
}
```
The last funtion here is the `restart_nav`.This function reset the values of the speed throttle controlle, the `prev_WP`and the mission.
