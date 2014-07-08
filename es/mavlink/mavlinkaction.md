# MAVLink en acción

Un mensaje MACLink puede ser manejado por `ArduCopter`, `ArduPlane` o `APMRover` dentro de la función `handleMessage` que pertenece a `GCS_MavLink.pde`:

La siguiente sección describe como interpreta el código un mensaje MAVLink:

### Analizar mensaje

El código lee el ID del sistema y el componente (cualquier sistema que use MAVLink tiene un ID de sistema y un ID de componente) y comprueba que está bien para continuar:

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

#### Tipos: `MAVLINK_MSG_ID`
El siguiente contenido describe algunos de los mensajes tipo más relevantes `MAVLINK_MSG_ID`: 

1. `MAVLINK_MSG_ID_HEARTBEAT`
Este es el mensaje más imporante. El GCS mantiene el envío de un mensaje al piloto automático para averguar si está conectado a ella (cada segundo). Esto es para asegurar que el MP esta sincronixzado con el APM al actualizar algunos parámetros. Si el bit de vida se pierde, un mecanismo de seguridad puede ser lanzado y el avión aterriza, continúa la misión o se vuelve a lanzar (también denominado RTL). La opción a prueba de fallos se puede activar/desactivar. **El bit de vida no se puede parar**.
2. `MAVLINK_MSG_ID_REQUEST_DATA_STREAM`
Sensores, canales de radio frecuencia. posición GPS, estado.
3. `MAVLINK_MSG_ID_COMMAND_LONG`
Loiter ilimitado, RTL, Tierra, Comienzo de misión, Armar/desarmar, Reiniciar.
4. `SET_MODE`
Por ejemplo: set_mode(packet.custom_mode);
5. `MAVLINK_MSG_ID_MISSION_REQUEST_LIST`
Puntos de interés totales: Parámetro command_total. Esto almacena el número de puntos de interés que están presentes (excepto la ubicación de salida, para los multi-copters)
6. `MAVLINK_MSG_ID_MISSION_REQUEST`
Conjunto de valores enumerados MAV_CMD, como: (MAV_CMD_)CHANGE_ALT, SET_HOME, CONDITION_YAW, TAKE_OFF, NAV_LOITER_TIME
7. `MAVLINK_MSG_ID_MISSION_ACK`
Desactivar el envío de puntos de interés.
8. `MAVLINK_MSG_ID_PARAM_REQUEST_LIST`
count_parameters (Cuenta el total de parametros).
9. `MAVLINK_MSG_ID_PARAM_REQUEST_READ`
Recibe y decodifica los parámetros (Da sentido al nombre del los parámetros y el ID)
10. `MAVLINK_MSG_ID_MISSION_CLEAR_ALL`
Cuando usas el planificador de misión de vuelo y dices: *Misión terminada* en el menú del ratón,  . Se borra la EEPROM del piloto automático.
11. `MAVLINK_MSG_ID_MISSION_SET_CURRENT`
Esto se utiliza para cambiar el comando activo durante la mitad de la misión: Por ejemplo: cuando al hacer click en la pantalla de Google Maps y haces click en "Vuela aquí"
12. `MAVLINK_MSG_ID_MISSION_COUNT`
Guarda el número total de puntos de interés (exclutendo la salida) -> para Multicopters.
13. `MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST`
Sólo mantiene una variable global que indica que el piloto automático está recibiendo comandos ahora. Esto para evitar otras acciones MAVLink mientras que los parámetros importantes se están estableciendo.
14. `MAVLINK_MSG_ID_SET_MAG_OFFSETS`
Asigna mag_ofs_x, mag_ofs_y, mag_ofs_z, después de calibrar la brújula en la EEPROM del piloto automático. El planificador de misiones (MP) hace automaticamente esto o se puede hacer esto también 
en la lista de parámetros bajo SOFTWARE CONFIGURATION.
15. `MAVLINK_MSG_ID_MISSION_ITEM`
Esta es un parte interesante. Este mensaje contien submensajes para la indicar acciones en tiempo real. Al igual que cuando se establecen puntos de interés y funciones avanzadas:
    - Recibir puntos de interés (WP) de GCS y cuando en la EEPROM del piloto automático.
    - Envía 4 parámetros (por ejemplo Delay, HitRad, -, and Yaw Angle) para LOITER_TIME (ID) + (Lat, Long, Alt: Define la posición 3D del objeto en el espacio). Estos parámetros son definidos como un enumerado en el código + Opciones (1 = Store Altitude (Alt) relative to home altitude). Cada comando (o ID) puede tener diferentes parámetros de interes. El planificador de misiones muestra en `blanco+ 
la cabecera de la columna porque ese parámetro no tiene una ID definida.
A modo de resumen, los parámetros interesantes enviados en cada acción son los siguientes:
4 parámetros  ID (acción) + (Lat, Long, Alt) define la posición 3D del avión. Tenga en cuenta que los 4 parámetros pueden ser algún tipo de acción personalizada en cuando a configuración de la cámara, disparador de la cámara, Loiter time, etc.
    - Al igual que en la figura siguiente del planificador de misiones cada ID define un punto de interés (AFAIK). LOITER_TIME, LOITER_UNLIMITED, WAYPOINT todos son puntos de interés que se envían junto con otros parámetros (LATITUDE, LONGITUDE and ALTITUDE) porque cada uno se guarda como un punto de referencia en el piloto automático (la altitud es siempre relativa a la altitud de salida según el diseño actual)
    - Se puede definir 'estas acciones' en common.xml y usar la GUI de Python para generar el código que el piloto automático utilizará.
16. `MAVLINK_MSG_ID_PARAM_SET`
Asigna los parámetros. Recuerde, que también se puede establecer un valor de un parámetro en la Estación de control de tierra (por ejemplo, el planificador de misiones, ‘Full Parameter List’).
17. `MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE`
Sobreescribe los valores de radio frecuencia para HIL(Hardware-In-Loop-Simulation) ( o completa el control del interruptor de posición a través de la GUI)
18. `MAVLINK_MSG_ID_HIL_STATE`
Usa el simulador HIL. Es un realidad virtual con tu avión.
19. `MAVLINK_MSG_ID_DIGICAM_CONFIGURE`
20. `MAVLINK_MSG_ID_MOUNT_CONFIGURE`
21. `MAVLINK_MSG_ID_MOUNT_CONTROL`
22. `MAVLINK_MSG_ID_MOUNT_STATUS`
Hasta el momento, como su nombre indica, configura el comando oportuno asignado por el usuario
23. `MAVLINK_MSG_ID_RADIO, MAVLINK_MSG_ID_RADIO_STATUS`
Los estudias de la tasa de telemetría/USB y el retraso del auto ajuste entre el envío y la recepción de mensajes. Si la intensidad de la señal es más baja de lo esperado o los errores se están haciendo cada vez más altos. Es como un control adaptivo de flujo. Mira `__mavlink_radio_t` en C++ (en el código fuente de BeaglePilot/ardupilot).


### Extrayendo el contenido
El **contenido** se extraen a partir del mensaje y se ponen en un paquete. Un paquete es un estructura de datos basada en el tipo de información.

El paquete se asigna a una estructura de datos apropiada. Hay un montón de estructuras de datos, por ejemplo, para *Attitude* (pitch, roll, yaw orientation), GPS, canales de radio frecuencia, etc. grupos de cosas similares juntas, para ser más modular, comprensible.Estas estructuras de datos son ‘100% iguales’ en los extremos que envían y reciben.

En el siguiente código, este paso se realiza en `mavlink_msg_rc_channels_override_decode`.

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


El siguiente fragmenteo de código presenta el código de la función `handleMessage`:

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

### Fuentes:
- [MavLink Tutorial for Absolute Dummies (Part –I)](http://api.ning.com/files/i*tFWQTF2R*7Mmw7hksAU-u9IABKNDO9apguOiSOCfvi2znk1tXhur0Bt00jTOldFvob-Sczg3*lDcgChG26QaHZpzEcISM5/MAVLINK_FOR_DUMMIESPart1_v.1.1.pdf)
- [MAVLink source code](https://github.com/mavlink/mavlink)
