# navigation.pde

The [navigation.pde](https://github.com/diydrones/ardupilot/blob/master/APMrover2/navigation.pde) file includes functions  used(for calculations) when moving to the desired destination.

```cpp
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//****************************************************************
// Function that will calculate the desired direction to fly and distance
//****************************************************************
static void navigate()
{
	// do not navigate with corrupt data
	// ---------------------------------
	if (!have_position) {
		return;
	}

...
```
This function, first of all, checks if the `have_position` exists and is uupdated.

```cpp
	if ((next_WP.lat == 0)||(home_is_set==false)){
		return;
	}
...
```
Next, checks if the next waypoint is set, as well as the `home`.

```cpp
	// waypoint distance from rover
	// ----------------------------
	wp_distance = get_distance(current_loc, next_WP);

	if (wp_distance < 0){
		gcs_send_text_P(SEVERITY_HIGH,PSTR("<navigate> WP error - distance < 0"));
		return;
	}
	...
	```
then the distance to the waypoint is calculated through `get_distance`.If this distance is under 0, the code gives an error, the rover has passed the wp.

```cpp

	// control mode specific updates to nav_bearing
	// --------------------------------------------
	update_navigation();
}
...
```
Lastly, `updates_navigation()`function is called for updaiting the wp, and home datas.

```cpp

void reached_waypoint()
{

}
```
This function is not implementes yet. Its aim is to notify when the wp is reached.
