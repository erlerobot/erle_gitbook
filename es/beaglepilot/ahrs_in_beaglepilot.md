# AHRS in ArduPilot

Let's take a look at some measurements (this data has been taken from mavproxy):

`RAW_IMU`:
```
2000-01-01 00:05:13.32: RAW_IMU {time_usec : 92575068, xacc : 289, yacc : -95, zacc : 224, xgyro : -4, ygyro : 0, zgyro : 3, xmag : 0, ymag : 0, zmag : 0}
2000-01-01 00:05:13.58: RAW_IMU {time_usec : 92833820, xacc : 386, yacc : -150, zacc : 176, xgyro : -5, ygyro : 0, zgyro : 3, xmag : 0, ymag : 0, zmag : 0}
2000-01-01 00:05:13.84: RAW_IMU {time_usec : 93092729, xacc : 374, yacc : -63, zacc : 117, xgyro : -1, ygyro : 7, zgyro : -2, xmag : 0, ymag : 0, zmag : 0}
2000-01-01 00:05:14.09: RAW_IMU {time_usec : 93351529, xacc : 288, yacc : -115, zacc : 144, xgyro : 3, ygyro : -4, zgyro : 2, xmag : 0, ymag : 0, zmag : 0}
2000-01-01 00:05:14.36: RAW_IMU {time_usec : 93610297, xacc : 298, yacc : -49, zacc : 208, xgyro : 1, ygyro : 1, zgyro : -1, xmag : 0, ymag : 0, zmag : 0}
2000-01-01 00:05:14.61: RAW_IMU {time_usec : 93869070, xacc : 236, yacc : -137, zacc : 60, xgyro : 0, ygyro : 1, zgyro : 3, xmag : 0, ymag : 0, zmag : 0}
2000-01-01 00:05:14.87: RAW_IMU {time_usec : 94127964, xacc : 312, yacc : -128, zacc : 177, xgyro : 0, ygyro : 5, zgyro : -3, xmag : 0, ymag : 0, zmag : 0}
2000-01-01 00:05:15.14: RAW_IMU {time_usec : 94386940, xacc : 379, yacc : -178, zacc : 267, xgyro : -4, ygyro : -1, zgyro : -2, xmag : 0, ymag : 0, zmag : 0}
2000-01-01 00:05:15.39: RAW_IMU {time_usec : 94645530, xacc : 160, yacc : -187, zacc : 224, xgyro : -4, ygyro : 8, zgyro : 7, xmag : 0, ymag : 0, zmag : 0}
2000-01-01 00:05:15.65: RAW_IMU {time_usec : 94904309, xacc : 339, yacc : -137, zacc : 78, xgyro : 4, ygyro : 2, zgyro : -6, xmag : 0, ymag : 0, zmag : 0}
2000-01-01 00:05:15.90: RAW_IMU {time_usec : 95163245, xacc : 362, yacc : -126, zacc : 121, xgyro : 0, ygyro : 0, zgyro : 3, xmag : 0, ymag : 0, zmag : 0}
2000-01-01 00:05:16.17: RAW_IMU {time_usec : 95422068, xacc : 327, yacc : -66, zacc : 156, xgyro : 4, ygyro : 1, zgyro : 0, xmag : 0, ymag : 0, zmag : 0}
2000-01-01 00:05:16.43: RAW_IMU {time_usec : 95681006, xacc : 357, yacc : -39, zacc : 146, xgyro : 1, ygyro : 10, zgyro : 0, xmag : 0, ymag : 0, zmag : 0}
2000-01-01 00:05:16.68: RAW_IMU {time_usec : 95939736, xacc : 308, yacc : -128, zacc : 172, xgyro : -1, ygyro : -11, zgyro : 8, xmag : 0, ymag : 0, zmag : 0}
2000-01-01 00:05:16.94: RAW_IMU {time_usec : 96198758, xacc : 233, yacc : -259, zacc : 159, xgyro : 1, ygyro : 0, zgyro : 0, xmag : 0, ymag : 0, zmag : 0}
2000-01-01 00:05:17.20: RAW_IMU {time_usec : 96457637, xacc : 304, yacc : -75, zacc : 206, xgyro : 0, ygyro : 6, zgyro : 1, xmag : 0, ymag : 0, zmag : 0}
```

`ATTITUDE`:
```
2000-01-01 00:05:15.45: ATTITUDE {time_boot_ms : 94705, roll : 2.52946019173, pitch : 0.973517298698, yaw : 1.95986223221, rollspeed : 0.00637999176979, pitchspeed : 0.00718349218369, yawspeed : -0.00310219824314}
2000-01-01 00:05:15.71: ATTITUDE {time_boot_ms : 94964, roll : 2.52297258377, pitch : 0.97235751152, yaw : 1.95445096493, rollspeed : -0.00585854053497, pitchspeed : 0.00399082899094, yawspeed : 0.000622570514679}
2000-01-01 00:05:15.97: ATTITUDE {time_boot_ms : 95223, roll : 2.51813912392, pitch : 0.984237790108, yaw : 1.95032703876, rollspeed : 0.00425156950951, pitchspeed : -0.00186237692833, yawspeed : -0.0100196264684}
2000-01-01 00:05:16.23: ATTITUDE {time_boot_ms : 95481, roll : 2.51992321014, pitch : 0.99168664217, yaw : 1.95116615295, rollspeed : -0.00585854053497, pitchspeed : -0.00186237692833, yawspeed : -0.000441648066044}
2000-01-01 00:05:16.48: ATTITUDE {time_boot_ms : 95740, roll : 2.53586769104, pitch : 0.976905345917, yaw : 1.96385073662, rollspeed : -0.000537425279617, pitchspeed : -0.000798165798187, yawspeed : -0.00203797966242}
2000-01-01 00:05:16.75: ATTITUDE {time_boot_ms : 95999, roll : 2.54653191566, pitch : 0.986998200417, yaw : 1.97242736816, rollspeed : -0.00160163640976, pitchspeed : -0.000798165798187, yawspeed : -0.00576274842024}
2000-01-01 00:05:17.01: ATTITUDE {time_boot_ms : 96258, roll : 2.54613947868, pitch : 0.982476115227, yaw : 1.97158503532, rollspeed : 0.00637999176979, pitchspeed : 0.000266045331955, yawspeed : 0.0016867890954}
2000-01-01 00:05:17.26: ATTITUDE {time_boot_ms : 96517, roll : 2.5438811779, pitch : 0.978500187397, yaw : 1.96952974796, rollspeed : 0.0021231174469, pitchspeed : 0.00558716058731, yawspeed : 0.00115468353033}

```

