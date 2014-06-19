# MAVLink in action

A MAVLink message is handled by `ArduCopter`, `ArduPlane` or `APMRover` inside a function called `handleMessage` that belongs to  `GCS_MavLink.pde`:


The following sections describe how the code interprets a MAVLink message:

### Parse message

The code reads the System ID and Component ID (any system using MavLink has a System ID and Component ID) and checks whether is fine to continue.

Then it handles Message ID:

```
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:      // MAV ID: 0
    {
        ...
    }

    case MAVLINK_MSG_ID_SET_MODE:       // MAV ID: 11
    {
        ...
    }

    ...
```

#### `MAVLINK_MSG_ID` types
The following content describes some of the most relevant `MAVLINK_MSG_ID` kinds:

1. `MAVLINK_MSG_ID_HEARTBEAT`
This is the most important message. The GCS keeps sending a message to the autopilot to find out whether it is connected to it (every 1 second). This is to make sure the MP is in sync with APM when you update some parameters. If a number of heartbeats are missed, a failsafe (can be) is triggered and copter lands, continues the mission or Returns to launch (also called, RTL). The failsafe option can be enabled/disabled. **Heartbeats cannot be stopped**.
2. `MAVLINK_MSG_ID_REQUEST_DATA_STREAM`
Sensors, RC channels, GPS position, status.
3. `MAVLINK_MSG_ID_COMMAND_LONG`
Loiter unlimited, RTL, Land, Mission start, Arm/Disarm, Reboot
4. `SET_MODE`
E.g. set_mode(packet.custom_mode);
5. `MAVLINK_MSG_ID_MISSION_REQUEST_LIST`
Total waypoints: command_total variable of parameters. This stores the total number of waypoints that are present (except home location, for multi-copters)
6. `MAVLINK_MSG_ID_MISSION_REQUEST`
Set of MAV_CMD value enum members, such as: (MAV_CMD_)CHANGE_ALT, SET_HOME, CONDITION_YAW, TAKE_OFF, NAV_LOITER_TIME
7. `MAVLINK_MSG_ID_MISSION_ACK`
turn off waypoint send
8. `MAVLINK_MSG_ID_PARAM_REQUEST_LIST`
count_parameters (Count the total parameters)
9. `MAVLINK_MSG_ID_PARAM_REQUEST_READ`
Receive and decode parameters (Make sense of Param name and Id)
10. `MAVLINK_MSG_ID_MISSION_CLEAR_ALL`
When you use mission planner flight data screen and say, Clear Mission with the mouse menu, this is where it goes. It clears the EEPROM memory from the autopilot.
11. `MAVLINK_MSG_ID_MISSION_SET_CURRENT`
This is used to change active command during mid mission. E.g. when you click on MP google map screen and click ‘Fly To Here’, as an example.
12. `MAVLINK_MSG_ID_MISSION_COUNT`
Save the total number of waypoints (excluding home location) -> for Multicopters.
13. `MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST`
Just keeping a global variable stating that the autopilot is receiving commands now. This is to avoid other MAVLink actions while important parameters are being set.
14. `MAVLINK_MSG_ID_SET_MAG_OFFSETS`
Set the mag_ofs_x, mag_ofs_y, mag_ofs_z, say after compass calibration to EEPROM of the autopilot. Mission Planner (MP) automatically does this or you can do this too by going to Full Parameters List under SOFTWARE CONFIGURATION.
15. `MAVLINK_MSG_ID_MISSION_ITEM`
This is an interesting part. This message contains sub messages for taking real-time action. Like setting waypoints and advanced features:
    - Receive a Waypoint (WP) from GCS and store in EEPROM of the autopilot.
    - Sends 4 params (e.g. Delay, HitRad, -, and Yaw Angle) for LOITER_TIME (as ID) + (Lat, Long, Alt: Defines the 3D position of object in space). These parameters are defined as an Enum in code + Options (1 = Store Altitude (Alt) relative to home altitude). Each Command (or ID) may have different parameters of interest. Mission Planner shows ‘blank’ header for column, because there is no parameter defined for this ID.
As a summary, the interesting parameters sent with every action are:
4 params + ID (of action) + (Lat, Long, Alt) defines 3D position of copter. Note the 4 params can be some sort of custom action as for camera setting, camera trigger, Loiter time, etc.
    - As in figure below from Mission Planner, each ID defines a waypoint (AFAIK). LOITER_TIME, LOITER_UNLIMITED, WAYPOINT all are waypoints that send along with other parameters (LATITUDE, LONGITUDE and ALTITUDE) because each is saved as a waypoint in the autopilot (ALTITUDE is always relative to home altitude with the current design.).
    - You can define ‘these actions’ in Common.xml and use Python GUI generator to generate the code that the autopilot will use.
16. `MAVLINK_MSG_ID_PARAM_SET`
Set the parameter. Remember, we can also set a value to a parameter in the Ground Control Stations (e.g. Mission Planner, ‘Full Parameter List’).
17. `MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE`
Override RC channel values for HIL (Hardware-In-Loop-Simulation) (Or complete GCS control of the switch position via GUI)
18. `MAVLINK_MSG_ID_HIL_STATE`
Used for HIL simulation. It is a virtual reality with your copter/ plane.
19. `MAVLINK_MSG_ID_DIGICAM_CONFIGURE`
20. `MAVLINK_MSG_ID_MOUNT_CONFIGURE`
21. `MAVLINK_MSG_ID_MOUNT_CONTROL`
22. `MAVLINK_MSG_ID_MOUNT_STATUS`
So far, as the name suggests, configures the appropriate command settings as set by the user.
23. `MAVLINK_MSG_ID_RADIO, MAVLINK_MSG_ID_RADIO_STATUS`
Studies the packet rate of the telemetry/USB and auto adjusts the delay between sending receiving packets if the signal strength is lower than expected or errors are getting higher. It is like an adaptive software flow control. Look at `__mavlink_radio_t`
in C++ (BeaglePilot/ardupilot code).


### Extract the payload
The **payload data** is extracted from the message and put into a packet. A packet is a data structure based on a **type** of information.

The packet is put into an *appropriate data structure*. There are plenty of data structures e.g. for Attitude (pitch, roll, yaw orientation), GPS, RC channels, etc. that groups similar things together, to be it modular, understandable. These data structures are ‘perfectly 100% alike’ at the sending and receiving ends.

In the following code, this step is performed at `mavlink_msg_rc_channels_override_decode`.

```
    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:       // MAV ID: 70
    {
        // allow override of RC channel values for HIL
        // or for complete GCS control of switch position
        // and RC PWM values.
        if(msg->sysid != g.sysid_my_gcs) break;                         // Only accept control from our gcs
        mavlink_rc_channels_override_t packet;
        int16_t v[8];
        mavlink_msg_rc_channels_override_decode(msg, &packet);

        // exit immediately if this command is not meant for this vehicle
        if (mavlink_check_target(packet.target_system,packet.target_component)) {
            break;
        }

        v[0] = packet.chan1_raw;
        v[1] = packet.chan2_raw;
        v[2] = packet.chan3_raw;
        v[3] = packet.chan4_raw;
        v[4] = packet.chan5_raw;
        v[5] = packet.chan6_raw;
        v[6] = packet.chan7_raw;
        v[7] = packet.chan8_raw;
        hal.rcin->set_overrides(v, 8);

        // record that rc are overwritten so we can trigger a failsafe if we lose contact with groundstation
        failsafe.rc_override_active = true;
        // a RC override message is consiered to be a 'heartbeat' from the ground station for failsafe purposes
        failsafe.last_heartbeat_ms = millis();
        break;
    }
```


------

### Reference code

------


The following snippet presents the code of the function `handleMessage':

``` C

void GCS_MAVLINK::handleMessage(mavlink_message_t* msg)
{
    uint8_t result = MAV_RESULT_FAILED;         // assume failure.  Each messages id is responsible for return ACK or NAK if required

    switch (msg->msgid) {

    case MAVLINK_MSG_ID_HEARTBEAT:      // MAV ID: 0
    {
        // We keep track of the last time we received a heartbeat from our GCS for failsafe purposes
        if(msg->sysid != g.sysid_my_gcs) break;
        failsafe.last_heartbeat_ms = millis();
        pmTest1++;
        break;
    }

    case MAVLINK_MSG_ID_SET_MODE:       // MAV ID: 11
    {
        // decode
        mavlink_set_mode_t packet;
        mavlink_msg_set_mode_decode(msg, &packet);

        // only accept custom modes because there is no easy mapping from Mavlink flight modes to AC flight modes
        if (packet.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) {
            if (set_mode(packet.custom_mode)) {
                result = MAV_RESULT_ACCEPTED;
            }
        }

        // send ACK or NAK
        mavlink_msg_command_ack_send_buf(msg, chan, MAVLINK_MSG_ID_SET_MODE, result);
        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:         // MAV ID: 20
    {
        handle_param_request_read(msg);
        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:         // MAV ID: 21
    {
        handle_param_request_list(msg);
        break;
    }

    case MAVLINK_MSG_ID_PARAM_SET:     // 23
    {
        handle_param_set(msg, &DataFlash);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST: // MAV ID: 38
    {
        handle_mission_write_partial_list(mission, msg);
        break;
    }

#ifdef MAVLINK_MSG_ID_SET_MAG_OFFSETS
    case MAVLINK_MSG_ID_SET_MAG_OFFSETS:
    {
        mavlink_set_mag_offsets_t packet;
        mavlink_msg_set_mag_offsets_decode(msg, &packet);
        // exit immediately if this command is not meant for this vehicle
        if (mavlink_check_target(packet.target_system,packet.target_component)) {
            break;
        }
        compass.set_offsets(Vector3f(packet.mag_ofs_x, packet.mag_ofs_y, packet.mag_ofs_z));
        break;
    }
#endif

    // GCS has sent us a command from GCS, store to EEPROM
    case MAVLINK_MSG_ID_MISSION_ITEM:           // MAV ID: 39
    {
        handle_mission_item(msg, mission);
        break;
    }

    // read an individual command from EEPROM and send it to the GCS
    case MAVLINK_MSG_ID_MISSION_REQUEST:     // MAV ID: 40
    {
        handle_mission_request(mission, msg);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:    // MAV ID: 41
    {
        handle_mission_set_current(mission, msg);
        break;
    }

    // GCS request the full list of commands, we return just the number and leave the GCS to then request each command individually
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:       // MAV ID: 43
    {
        handle_mission_request_list(mission, msg);
        break;
    }

    // GCS provides the full number of commands it wishes to upload
    //  individual commands will then be sent from the GCS using the MAVLINK_MSG_ID_MISSION_ITEM message
    case MAVLINK_MSG_ID_MISSION_COUNT:          // MAV ID: 44
    {
        handle_mission_count(mission, msg);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:      // MAV ID: 45
    {
        handle_mission_clear_all(mission, msg);
        break;
    }

    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:    // MAV ID: 66
    {
        handle_request_data_stream(msg, false);
        break;
    }

    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:       // MAV ID: 70
    {
        // allow override of RC channel values for HIL
        // or for complete GCS control of switch position
        // and RC PWM values.
        if(msg->sysid != g.sysid_my_gcs) break;                         // Only accept control from our gcs
        mavlink_rc_channels_override_t packet;
        int16_t v[8];
        mavlink_msg_rc_channels_override_decode(msg, &packet);

        // exit immediately if this command is not meant for this vehicle
        if (mavlink_check_target(packet.target_system,packet.target_component)) {
            break;
        }

        v[0] = packet.chan1_raw;
        v[1] = packet.chan2_raw;
        v[2] = packet.chan3_raw;
        v[3] = packet.chan4_raw;
        v[4] = packet.chan5_raw;
        v[5] = packet.chan6_raw;
        v[6] = packet.chan7_raw;
        v[7] = packet.chan8_raw;
        hal.rcin->set_overrides(v, 8);

        // record that rc are overwritten so we can trigger a failsafe if we lose contact with groundstation
        failsafe.rc_override_active = true;
        // a RC override message is consiered to be a 'heartbeat' from the ground station for failsafe purposes
        failsafe.last_heartbeat_ms = millis();
        break;
    }

    // Pre-Flight calibration requests
    case MAVLINK_MSG_ID_COMMAND_LONG:       // MAV ID: 76
    {
        // decode packet
        mavlink_command_long_t packet;
        mavlink_msg_command_long_decode(msg, &packet);

        // exit immediately if this command is not meant for this vehicle
        if (mavlink_check_target(packet.target_system, packet.target_component)) {
            break;
        }

        switch(packet.command) {

        case MAV_CMD_NAV_LOITER_UNLIM:
            if (set_mode(LOITER)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_NAV_RETURN_TO_LAUNCH:
            if (set_mode(RTL)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_NAV_LAND:
            if (set_mode(LAND)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_MISSION_START:
            if (set_mode(AUTO)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_PREFLIGHT_CALIBRATION:
            if (packet.param1 == 1 ||
                packet.param2 == 1) {
                ins.init_accel();
                ahrs.set_trim(Vector3f(0,0,0));             // clear out saved trim
                result = MAV_RESULT_ACCEPTED;
            }
            if (packet.param3 == 1) {
                init_barometer(false);                      // fast barometer calibration
                result = MAV_RESULT_ACCEPTED;
            }
            if (packet.param4 == 1) {
                trim_radio();
                result = MAV_RESULT_ACCEPTED;
            }
            if (packet.param5 == 1) {
                float trim_roll, trim_pitch;
                // this blocks
                AP_InertialSensor_UserInteract_MAVLink interact(chan);
                if(ins.calibrate_accel(&interact, trim_roll, trim_pitch)) {
                    // reset ahrs's trim to suggested values from calibration routine
                    ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
                }
                result = MAV_RESULT_ACCEPTED;
            }
            if (packet.param6 == 1) {
                // compassmot calibration
                result = mavlink_compassmot(chan);
            }
            break;

        case MAV_CMD_COMPONENT_ARM_DISARM:
            if (packet.param1 == 1.0f) {
                // run pre_arm_checks and arm_checks and display failures
                pre_arm_checks(true);
                if(ap.pre_arm_check && arm_checks(true)) {
                    init_arm_motors();
                    result = MAV_RESULT_ACCEPTED;
                }else{
                    AP_Notify::flags.arming_failed = true;  // init_arm_motors function will reset flag back to false
                    result = MAV_RESULT_UNSUPPORTED;
                }
            } else if (packet.param1 == 0.0f)  {
                init_disarm_motors();
                result = MAV_RESULT_ACCEPTED;
            } else {
                result = MAV_RESULT_UNSUPPORTED;
            }
            break;

        case MAV_CMD_DO_SET_SERVO:
            if (ServoRelayEvents.do_set_servo(packet.param1, packet.param2)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_REPEAT_SERVO:
            if (ServoRelayEvents.do_repeat_servo(packet.param1, packet.param2, packet.param3, packet.param4*1000)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_SET_RELAY:
            if (ServoRelayEvents.do_set_relay(packet.param1, packet.param2)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_REPEAT_RELAY:
            if (ServoRelayEvents.do_repeat_relay(packet.param1, packet.param2, packet.param3*1000)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            if (packet.param1 == 1 || packet.param1 == 3) {
                // when packet.param1 == 3 we reboot to hold in bootloader
                hal.scheduler->reboot(packet.param1 == 3);
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_FENCE_ENABLE:
#if AC_FENCE == ENABLED
            result = MAV_RESULT_ACCEPTED;
            switch ((uint16_t)packet.param1) {
                case 0:
                    fence.enable(false);
                    break;
                case 1:
                    fence.enable(true);
                    break;
                default:
                    result = MAV_RESULT_FAILED;
                    break;
            }
#else
            // if fence code is not included return failure
            result = MAV_RESULT_FAILED;
#endif
            break;

        case MAV_CMD_DO_MOTOR_TEST:
            // param1 : motor sequence number (a number from 1 to max number of motors on the vehicle)
            // param2 : throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)
            // param3 : throttle (range depends upon param2)
            // param4 : timeout (in seconds)
            result = mavlink_motor_test_start(chan, (uint8_t)packet.param1, (uint8_t)packet.param2, (uint16_t)packet.param3, packet.param4);
            break;

        default:
            result = MAV_RESULT_UNSUPPORTED;
            break;
        }

        // send ACK or NAK
        mavlink_msg_command_ack_send_buf(msg, chan, packet.command, result);

        break;
    }

    case MAVLINK_MSG_ID_COMMAND_ACK:        // MAV ID: 77
    {
        command_ack_counter++;
        break;
    }

#if HIL_MODE != HIL_MODE_DISABLED
    case MAVLINK_MSG_ID_HIL_STATE:          // MAV ID: 90
    {
        mavlink_hil_state_t packet;
        mavlink_msg_hil_state_decode(msg, &packet);

        // set gps hil sensor
        Location loc;
        loc.lat = packet.lat;
        loc.lng = packet.lon;
        loc.alt = packet.alt/10;
        Vector3f vel(packet.vx, packet.vy, packet.vz);
        vel *= 0.01f;

        gps.setHIL(0, AP_GPS::GPS_OK_FIX_3D,
                   packet.time_usec/1000,
                   loc, vel, 10, 0, true);

        if (!ap.home_is_set) {
            init_home();
        }


        // rad/sec
        Vector3f gyros;
        gyros.x = packet.rollspeed;
        gyros.y = packet.pitchspeed;
        gyros.z = packet.yawspeed;

        // m/s/s
        Vector3f accels;
        accels.x = packet.xacc * (GRAVITY_MSS/1000.0);
        accels.y = packet.yacc * (GRAVITY_MSS/1000.0);
        accels.z = packet.zacc * (GRAVITY_MSS/1000.0);

        ins.set_gyro(0, gyros);

        ins.set_accel(0, accels);

        barometer.setHIL(packet.alt*0.001f);
        compass.setHIL(packet.roll, packet.pitch, packet.yaw);

        break;
    }
#endif //  HIL_MODE != HIL_MODE_DISABLED

    case MAVLINK_MSG_ID_RADIO:
    case MAVLINK_MSG_ID_RADIO_STATUS:       // MAV ID: 109
    {
        handle_radio_status(msg, DataFlash, (g.log_bitmask & MASK_LOG_PM) != 0);
        break;
    }

    case MAVLINK_MSG_ID_LOG_REQUEST_LIST ... MAVLINK_MSG_ID_LOG_REQUEST_END:    // MAV ID: 117 ... 122
        if (!in_mavlink_delay && !motors.armed()) {
            handle_log_message(msg, DataFlash);
        }
        break;

#if HAL_CPU_CLASS > HAL_CPU_CLASS_16
    case MAVLINK_MSG_ID_SERIAL_CONTROL:
        handle_serial_control(msg, gps);
        break;
#endif

#if CAMERA == ENABLED
    case MAVLINK_MSG_ID_DIGICAM_CONFIGURE:      // MAV ID: 202
        camera.configure_msg(msg);
        break;

    case MAVLINK_MSG_ID_DIGICAM_CONTROL:
        camera.control_msg(msg);
        break;
#endif // CAMERA == ENABLED

#if MOUNT == ENABLED
    case MAVLINK_MSG_ID_MOUNT_CONFIGURE:        // MAV ID: 204
        camera_mount.configure_msg(msg);
        break;

    case MAVLINK_MSG_ID_MOUNT_CONTROL:
        camera_mount.control_msg(msg);
        break;

    case MAVLINK_MSG_ID_MOUNT_STATUS:
        camera_mount.status_msg(msg);
        break;
#endif // MOUNT == ENABLED

#if AC_RALLY == ENABLED
    // receive a rally point from GCS and store in EEPROM
    case MAVLINK_MSG_ID_RALLY_POINT: {
        mavlink_rally_point_t packet;
        mavlink_msg_rally_point_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;

        if (packet.idx >= rally.get_rally_total() ||
            packet.idx >= MAX_RALLYPOINTS) {
            send_text_P(SEVERITY_LOW,PSTR("bad rally point message ID"));
            break;
        }

        if (packet.count != rally.get_rally_total()) {
            send_text_P(SEVERITY_LOW,PSTR("bad rally point message count"));
            break;
        }

        RallyLocation rally_point;
        rally_point.lat = packet.lat;
        rally_point.lng = packet.lng;
        rally_point.alt = packet.alt;
        rally_point.break_alt = packet.break_alt;
        rally_point.land_dir = packet.land_dir;
        rally_point.flags = packet.flags;

        if (!rally.set_rally_point_with_index(packet.idx, rally_point)) {
            send_text_P(SEVERITY_HIGH, PSTR("error setting rally point"));
        }

        break;
    }

    //send a rally point to the GCS
    case MAVLINK_MSG_ID_RALLY_FETCH_POINT: {
        //send_text_P(SEVERITY_HIGH, PSTR("## getting rally point in GCS_Mavlink.pde 1")); // #### TEMP

        mavlink_rally_fetch_point_t packet;
        mavlink_msg_rally_fetch_point_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;

        //send_text_P(SEVERITY_HIGH, PSTR("## getting rally point in GCS_Mavlink.pde 2")); // #### TEMP

        if (packet.idx > rally.get_rally_total()) {
            send_text_P(SEVERITY_LOW, PSTR("bad rally point index"));
            break;
        }

        //send_text_P(SEVERITY_HIGH, PSTR("## getting rally point in GCS_Mavlink.pde 3")); // #### TEMP

        RallyLocation rally_point;
        if (!rally.get_rally_point_with_index(packet.idx, rally_point)) {
           send_text_P(SEVERITY_LOW, PSTR("failed to set rally point"));
           break;
        }

        //send_text_P(SEVERITY_HIGH, PSTR("## getting rally point in GCS_Mavlink.pde 4")); // #### TEMP

        mavlink_msg_rally_point_send_buf(msg,
                                         chan, msg->sysid, msg->compid, packet.idx,
                                         rally.get_rally_total(), rally_point.lat, rally_point.lng,
                                         rally_point.alt, rally_point.break_alt, rally_point.land_dir,
                                         rally_point.flags);

        //send_text_P(SEVERITY_HIGH, PSTR("## getting rally point in GCS_Mavlink.pde 5")); // #### TEMP

        break;
    }
#endif // AC_RALLY == ENABLED


    }     // end switch
} // end handle mavlink
```

### Sources:
- [MavLink Tutorial for Absolute Dummies (Part –I)](http://api.ning.com/files/i*tFWQTF2R*7Mmw7hksAU-u9IABKNDO9apguOiSOCfvi2znk1tXhur0Bt00jTOldFvob-Sczg3*lDcgChG26QaHZpzEcISM5/MAVLINK_FOR_DUMMIESPart1_v.1.1.pdf)
- [MAVLink source code](https://github.com/mavlink/mavlink)
