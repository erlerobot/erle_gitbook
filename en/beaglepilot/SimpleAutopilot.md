A simplified autopilot
==========

This module is going to introduce a simple autopilot coded in `python` that offers the following characterístics:

- Control loop using *P*ropopotional *I*ntegrate *D*erivate controllers (PID)
- 4 PWM output signals using PyBBIO
- Sensing input from the IMU (InvenSense MPU9150) at 50 Hz rate
- Bluetooth Link to provide external input

The code can be found [here](https://github.com/erlerobot/erle_control).
The following sections will address the different modules necessary to implement a **simple autopilot**.

Motors
-----

This class ([code in GitHub](https://github.com/erlerobot/erle_control/blob/master/motors.py)) allows to control the motors speed using Pulse Width Modulation (PWM).

```python

class Motor:
    """ @brief Class for interfacing with the motors.

        The class controls the motors speed using the sysfs PWM provided
        by Adafruit_BBIO.
        The duty should be provided in the range (0,100) and the speed
        in this range. The hardware is preset to satisfy the robot needs (two motors CW and two CCW)

        @warning in previous versions the range (-100,100) was accepted. New code just accepts
        values in the (0,100).
        @note the hardware is installed in a way that the motors rotating direction is the one needed
        (e.g.: 1,3 CW and 2,4 CCW)
    """


    def __init__(self, motor_number=1, max_speed=100, min_speed=0):
        self.speed = 0;
        # self.motor_pins_list = [["P9_14", "P9_16"],
        #                     ["P8_19", "P8_13"],
        #                     ["P9_22", "P9_21"],
        #                     ["P9_42", "P9_28"]]
        #self.motor_pins_list = ["P9_14", "P9_21","P9_22", "P9_42"]
        self.motor_pins_list = ["P9_16", "P8_13","P9_21", "P9_28"]
```

eHRPWM compatible pins (high resolution PWM capable). Check the [datasheet of the processor](http://www.ti.com/product/am3359) for more information.

```python
        if (motor_number > 4) or (motor_number < 1):
            raise Exception("Motor number provided out of bounds! ([1-4])")
        self.motor_number = motor_number # 1, 2, 3 or 4
        self.max_speed= max_speed
        self.min_speed= min_speed
```

`max_speed` and `min_speed` help while testing (it also saves the robot to crash inmmediately ;)).
``` python
        #self.frequency = 2000
        self.frequency = 943
        self.motor_pin = self.motor_pins_list[self.motor_number - 1]
        #perform PWM initialization
        # DC Brushed motors
        # self.duty_IN1 = 0 # duty input 1 of the motor controller IC
        # self.duty_IN2 = 0
        # PWM.start(self.motor_pin[0], self.duty_IN1, self.frequency)
        # PWM.start(self.motor_pin[1], self.duty_IN2, self.frequency)
```
The class was initially programmed for *brushed motors* and afterwards it was adapted to function with *brushless* ones.
``` python
        # DC Brushless motors
        self.duty = 0
        PWM.start(self.motor_pin, self.duty, self.frequency)

    def setSpeed(self, speed):
        """ @brief Set the duties according to the speed attribute for the DC Brushed motors

            @warning Not to be used with the brushless configuration
        """
        if speed<=self.max_speed or speed>=self.min_speed:
            if speed > 0:
                self.duty_IN1 = abs(speed)
                self.duty_IN2 = 0
            else:
                self.duty_IN1 = 0
                self.duty_IN2 = abs(speed)
        else:
            raise Exception("Speed provided not in the [0,100] range!")

    def setSpeedBrushless(self, speed):
        """ @brief Set the duties according to the speed attribute for the DC Brushless motors
        """
        if speed<=self.max_speed or speed>=self.min_speed:
            self.duty = speed
        else:
            raise Exception("Speed provided not in the [0,100] range!")

```
after setting the desired speed with `setSpeedBrushless` (the brushed mode is deprecated), the `go` method sends the duty cycle to the eHRPWM submodule.
``` python

    def go(self):
        """ @brief update the motor PWM according to the class duty attributes
        """
        # DC Brushed motors
        # PWM.set_duty_cycle(self.motor_pin[0],self.duty_IN1)
        # PWM.set_duty_cycle(self.motor_pin[1],self.duty_IN2)
        # DC Brushless motors
        PWM.set_duty_cycle(self.motor_pin, self.duty)


```

IMU
----
The following class ([code in GitHub](https://github.com/erlerobot/erle_control/blob/master/imu.py)) is programmed for the InvenSense MPU9150 sensor. The code makes use of the Linux sensor driver compiled as a shared library (`libimu.so`) through the `python Ctypes`:

``` python
# array class
Vector3d_t = 3*c_float
# array class
Quaternion_t = 4*c_float

# struct with C types
class Mpudata_t(Structure):
         _fields_ = [("rawGyro", c_short*3),
                     ("rawAccel", c_short*3),
                     ("rawQuat", c_long*4),
                     ("dmpTimestamp", c_ulong),
                     ("rawMag", c_short*4),
                     ("magTimestamp", c_ulong),
                     ("calibratedAccel", c_short*4),
                     ("calibratedMag", c_short*4),
                     ("fusedQuat", 4*c_float),
                     ("fusedEuler", 3*c_float),
                     ("lastDMPYaw", c_float),
                     ("lastYaw", c_float)
                    ]
```
Compatible C-types defined.
``` python

class IMU:
    """ Interface with the Inertial Measurement Unit.

    The IMU consists of an InvenSense 9-axis MPU-9150. This sensor provides
    readings from 3 accelerometers, 3 magnetometers and 3 gyroscopes.
    Furthermore, the module has a DMP (Digital Motion Processor) integrated
    that makes the calculations necessary to provide filtered outputs.
    """
    def __init__(self):
        #TODO Set I2C interface, make sure that calibrations files are available and make some readings
        # through ctypes.
        self.lib = CDLL("/root/erle_control/imu/libimu.so")
        self.i2c_bus = 2
        self.lib.mpu9150_set_debug(0) # 1
        self.sample_rate = 50 # 50 Hz
```
We are using the sensor DMP so this is the fastest we can go.
``` python
        self.yaw_mix_factor = 3

        # initialize the IMU
        res = self.lib.mpu9150_init(self.i2c_bus, self.sample_rate, self.yaw_mix_factor)
        if res:
            Exception("Error when initializing the IMU!")
```
This *calibration files* should have been generated before. Check the [driver code](https://github.com/erlerobot/erle_control/tree/master/imu) for more information.
``` python
        # set calibration files
        res = self.lib.set_cal(0, "./imu/accelcal.txt")
        if res != 0:
            Exception("Error while calibration: accelcal.txt")
        res = self.lib.set_cal(1, "./imu/magcal.txt")
        if res != 0:
            Exception("Error while calibration: magcal.txt")
```
The following methods receive the data from the C functions. This is done so that code can be written in python *using the C driver*.
``` python

    """ Reads the raw gyro data from the sensor.
            pass a "timing = 1" parameter to measure the time for the measurement.
        @return  gyroX, gyroY, gyroZ
    """
    def read_rawGyro(self, timing = 0):
        if timing:
            start = clock()
        while 1:
                # Parameters to be passed by reference
                x = c_short(0)
                y = c_short(0)
                z = c_short(0)
                function = self.lib.read_rawGyro
                function.argtypes = [POINTER(c_float), POINTER(c_float), POINTER(c_float)]
                res = function(byref(x), byref(y), byref(z))
                if res == 0:
                        time_s = clock() - start
                        print time_s
                        return x.value, y.value, z.value

    """ Reads fused euler angles
            pass a "timing = 1" parameter to measure the time for the measurement.
        @return  eulerX, eulerY, eulerZ (degrees)
    """
    def read_fusedEuler(self, timing = 0):
        if timing:
            start = clock()

        # DMP fused euler angles
        fusedX = c_float(0)
        fusedY = c_float(0)
        fusedZ = c_float(0)
        function = self.lib.read_fusedEuler
        function.argtypes = [POINTER(c_float), POINTER(c_float), POINTER(c_float)]
        while 1:
                res = function(byref(fusedX), byref(fusedY), byref(fusedZ))
                # if timing:
                #     time_s = clock() - start
                #     print "before res:"+str(time_s)
                if res == 0:
                        if timing:
                            time_s = clock() - start
                            print time_s
                        return fusedX.value, fusedY.value, fusedZ.value

                # if the measurement is not ready yet, wait the sampling freq
                sleep(1./self.sample_rate)


    """ Reads all the IMU sensor information and stores it into a Mpudata_t.
            pass a "timing = 1" parameter to measure the time for the measurement.

            TODO: Eventually substitute this way of getting data for a ctypes direct cast
    """
    def read_mpudata_t(self, timing = 0):
        if timing:
            start = clock()
        while 1:
                # Parameters to be passed by reference
                # Raw gyro values
                gyroX = c_short(0)
                gyroY = c_short(0)
                gyroZ = c_short(0)
                # Raw accel values
                accelX = c_short(0)
                accelY = c_short(0)
                accelZ = c_short(0)
                # Raw quaternion values
                quat1 = c_long(0)
                quat2 = c_long(0)
                quat3 = c_long(0)
                quat4 = c_long(0)
                # DMP timestamp
                dmpTimestamp = c_ulong(0)
                # Raw accel values
                magX = c_short(0)
                magY = c_short(0)
                magZ = c_short(0)
                # magnetometer timestamp
                magTimestamp = c_ulong(0)
                # Calibrated accelerometer values
                calibratedAccelX = c_short(0)
                calibratedAccelY = c_short(0)
                calibratedAccelZ = c_short(0)
                # Calibrated magnetometer values
                calibratedMagX = c_short(0)
                calibratedMagY = c_short(0)
                calibratedMagZ = c_short(0)
                # DMP fused quaternions
                fusedQuat1 = c_float(0)
                fusedQuat2 = c_float(0)
                fusedQuat3 = c_float(0)
                fusedQuat4 = c_float(0)
                # DMP fused euler angles
                fusedX = c_float(0)
                fusedY = c_float(0)
                fusedZ = c_float(0)
                # Last DMP Yaw
                lastDMPYaw = c_float(0)
                # Last Yaw
                lastYaw = c_float(0)

                function = self.lib.read_mpudata_t
                function.argtypes = [POINTER(c_short), POINTER(c_short), POINTER(c_short),
                                    POINTER(c_short), POINTER(c_short), POINTER(c_short),
                                    POINTER(c_long), POINTER(c_long), POINTER(c_long), POINTER(c_long),
                                    POINTER(c_ulong),POINTER(c_short), POINTER(c_short), POINTER(c_short),
                                    POINTER(c_ulong),POINTER(c_short), POINTER(c_short), POINTER(c_short),
                                    POINTER(c_short), POINTER(c_short), POINTER(c_short),
                                    POINTER(c_float), POINTER(c_float), POINTER(c_float), POINTER(c_float),
                                    POINTER(c_float), POINTER(c_float), POINTER(c_float),
                                    POINTER(c_float), POINTER(c_float)]

                res = function(byref(gyroX), byref(gyroY), byref(gyroZ),
                                byref(accelX), byref(accelY), byref(accelZ),
                                byref(quat1), byref(quat2), byref(quat3), byref(quat4),
                                byref(dmpTimestamp), byref(magX), byref(magY), byref(magZ),
                                byref(magTimestamp), byref(calibratedAccelX), byref(calibratedAccelY), byref(calibratedAccelZ),
                                byref(calibratedMagX), byref(calibratedMagY), byref(calibratedMagZ),
                                byref(fusedQuat1), byref(fusedQuat2), byref(fusedQuat3), byref(fusedQuat4),
                                byref(fusedX), byref(fusedY), byref(fusedZ), byref(lastDMPYaw), byref(lastYaw))

                if res == 0:
                        if timing:
                            time_s = clock() - start
                            print time_s
                        # Construct an instance of Mpudata_t
                        mpudata_t = Mpudata_t(rawGyro = (c_short*3)(*[gyroX.value, gyroY.value, gyroZ.value]),
                                                rawAccel = (c_short*3)(*[accelX.value, accelY.value, accelZ.value]),
                                                rawQuat = (c_long*3)(*[quat1.value, quat2.value, quat3.value,quat4.value]),
                                                dmpTimestamp = (c_ulong)(dmpTimestamp.value),
                                                rawMag = (c_short*3)(*[magX.value, magY.value, magZ.value]),
                                                magTimestamp = (c_ulong)(magTimestamp.value),
                                                calibratedAccel = (c_float*3)(*[calibratedAccelX.value, calibratedAccelY.value, calibratedAccelZ.value]),
                                                calibratedMag = (c_float*3)(*[calibratedMagX.value, calibratedMagY.value, calibratedMagZ.value]),
                                                fusedQuat = (c_float*4)(*[fusedQuat1.value, fusedQuat2.value, fusedQuat3.value, fusedQuat4.value]),
                                                fusedEuler = (c_float*3)(*[fusedX.value, fusedY.value, fusedZ.value]),
                                                lastDMPYaw = (c_float)(lastDMPYaw),
                                                lastYaw = (c_float)(lastYaw)

                                            )
                        return mpudata_t

```


PID
----
This class ([code in GitHub](https://github.com/erlerobot/erle_control/blob/master/pid.py)) implemments a simple PID control algorithm. Gains K_p` (proportional), `K_d` (derivative) and `K_i` (integral) can be initially passed as parameters when the class is created.

``` python
class PID:
    """ Simple PID control.

        This class implements a simplistic PID control algorithm. When first
        instantiated all the gain variables are set to zero, so calling
        the method GenOut will just return zero.
    """
    def __init__(self, Kp = 1, Kd = 0, Ki = 0):
        # initialze gains
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki

        self.Initialize()
```
Gain setters.
``` python
    def SetKp(self, invar):
        """ Set proportional gain. """
        self.Kp = invar

    def SetKi(self, invar):
        """ Set integral gain. """
        self.Ki = invar

    def SetKd(self, invar):
        """ Set derivative gain. """
        self.Kd = invar

    def SetPrevErr(self, preverr):
        """ Set previous error value. """
        self.prev_err = preverr

    def Initialize(self):
        # initialize delta t variables
        self.currtm = time.time()
        self.prevtm = self.currtm

        self.prev_err = 0

        # term result variables
        self.Cp = 0
        self.Ci = 0
        self.Cd = 0

```
A `debug` flag is included. If `true`, the `update` method becomes **verbose**.
``` python
    def update(self, error, debug = 0):
        """ Performs a PID computation and returns a control value based on
            the elapsed time (dt) and the error signal from a summing junction
            (the error parameter).
        """
        if debug:
            print "   ****************************"
            print "   PID Debug"
            print "   ****************************"
            print "   error:"+str(error)

        self.currtm = time.time()               # get t
        dt = self.currtm - self.prevtm          # get delta t
        if debug:
            print "   dt:"+str(dt)
        de = error - self.prev_err              # get delta error
        if debug:
            print "   de:"+str(de)

        self.Cp = error               # proportional term
        if debug:
            print "   Proportional term (Cp*Kp):"+str(self.Cp)
        self.Ci += error * dt                   # integral term
        if debug:
            print "   Integral term (Ci*Ki):"+str(self.Ci * self.Ki)

        self.Cd = 0
        if dt > 0:                              # no div by zero
            self.Cd = de/dt                     # derivative term
            if debug:
                print "   Derivative term (Cd*Kd):"+str(self.Cd * self.Kd)

        self.prevtm = self.currtm               # save t for next pass
        self.prev_err = error                   # save t-1 error

        # sum the terms and return the result
        terms_sum = self.Cp * self.Kp  + (self.Ki * self.Ci) + (self.Kd * self.Cd)
```
`terms_sum` implemments the PID control equation:

\begin{equation}u(t) = K_p \cdot e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de(t)}{dt}\end{equation}

``` python
        if debug:
            print "   Terms sum (self.Cp + (self.Ki * self.Ci) + (self.Kd * self.Cd)):"+str(terms_sum)
            print "   ****************************"
        return terms_sum
```

### PID Calibration
#### Intuition

| CL RESPONSE   | RISE TIME  | OVERSHOOT  | SETTLING TIME | S-S ERROR |
| ------------- |:-------------:| -----:|------------------|----------|
| Kp | Decrease | Increase | Small Change | Decrease |
| Ki | Decrease | Increase | Increase | Eliminate |
| Kd | Small Change | Decrease | Decrease | No Change |


Quadcopters are symmetric so you can set the same PID Gain values for Pitch, and Roll. The value for Yaw is not as important as those of Pitch and Roll so it’s probably OK to set the same values as for Pitch/Roll to start with (even it might not be the best). After your drone is relatively stable, you can start alter the Yaw gains. For non-symmetric drones like hexacopter or tricopter, you might want to fine tune the pitch and roll separately, after you have some flight experience.

#### P tunning
For P gain, start low, until you notice it’s producing oscillation. Fine tune it until you get to a point it’s not sluggish and there is not oscillation.

#### I tunning
For the I gain, again start low, and increase slowly. Roll and pitch your quad left and right, pay attention to the how long does it take to stop and stabilize. You want to get to a point where it stabilize very quickly as you release the stick and it doesn’t wander around for too long. You might also want to test it under windy condition to get a reliable I-value.

#### D tunning
For D gain, it can get into a complicated interaction with P and I values. When using D gain, you need to go back and fine tune P and I to keep the plant well stabilized.


Dynamical Model
---------

This class ([code in GitHub](https://github.com/erlerobot/erle_control/blob/master/dynamical_model.py)) implemments the **dynamical model** of the quadcopter:

``` python

class Dynamical_Model:
    def __init__(self):
        # general paramters
        self.g = 9.806 # Gravity constant [m s^-2 ]
        self.rho = 1.293 # Air density [kg m^-3]
        self.nu = 1.8e-5 # Air viscosity at 20 degrees Celsius [Pa s]

        # quadrotor parameters
        self.P = 4 # number of propellers
        self.L = 29.9974e-3 # Arm length [m]
        self.Vol = 0.00281784516 # Volume [m3] (((86.36*86.36)/1000) *
        self.m = 60e-3 # quadrotor mass [kg]
        self.h = 17e-3 # Vertical distance between CoG and propellers plan [m]                         #                  (8*29.99/1000) * (1.5748/1000))
        self.b = 3.13e-5 # thrust factor in hover [N s^2]
        self.d = 7.5e-7 # drag factor in hover [N m s^2]
        self.W_prop = (self.m * self.g)/self.P # weight of the quadrotor per propeller [N]
        self.Omega_H = math.sqrt(self.W_prop/self.b) # propeller speed at hover

        # propellers
        self.N = 3 # Number of blades per propeller
        self.R = 32.5e-3 # Propeller radius [m]
        self.A = math.pi*math.pow(self.R, 2)# Propeller disk area [m^2]
        self.c = 0.0394 # Chord [m]
        self.theta_0 = 0.2618 # Pitch of incidence [rad]
        self.theta_tw = 0.045 # Twist pitch [rad]
        self.sigma = self.N * self.c/(math.pi * self.R) # Solidity ratio (rotor fill ratio) [rad^-1]
        self.a = 5.7 # Lift slope
        self.C_d = 0.052 # Airfoil drag coefficient
        self.A_c = 0.005 # Helicopter center hub area [m^2]
        # Longitudinal drag coefficients
        self.Cx = 1.32
        self.Cy = 1.32
        self.Cz = 1.32

        # Inertia components [kg m^2]
        self.Ixx = 6.228e-3
        self.Iyy = 6.228e-3
        self.Izz = 1.121e-2

        # motor parameters
        # TODO complete the motor parameters
        self.k_m = 1 #TODO # torque constant
        self.tau = 1 #TODO # motor time constant
        self.eta = 1 #TODO # motor efficiency
        self.Omega_0 = 1 #TODO # point of linearization of the rotor speeds

        self.r = 4 # Reduction ratio
        self.J_t = 6.0100e-5 # Rotor inertia [kg m^2]

        # matrix for calculating the motor voltages from the control inputs
        self.m = np.matrix( ((1/(4*self.b),0, 1/(2*self.b), -1/(4*self.b)),
                        (1/(4*self.b),-1/(2*self.b), 0, 1/(4*self.b)),
                        (1/(4*self.b),0, -1/(2*self.b), -1/(4*self.b)),
                        (1/(4*self.b),1/(2*self.b), 0 ,  1/(4*self.b))) )



    """ Compute the motor voltages from the control inputs following M.Wieremma MS thesis (MSc_thesis_X-UFO). Keep in mind when
        passing parameters the following correspondences.
            - U1: thrust
            - U2: roll
            - U3: pitch
            - U4: yaw

        @returns: u=[u_m1, u_m2, u_m3, u_m3], motor voltages
    """
    def motor_inversion1(self, thrust, roll, pitch, yaw, logging = 0):
        # the control inputs
        U = np.array( ((thrust, roll, pitch, yaw)) )
        Um = np.matrix(U).T
        # the motor voltages
        u = (self.k_m * self.tau) * ((1/self.tau + 2*self.d*self.Omega_0/(self.eta*np.power(self.r,3)*self.J_t))\
            * np.sqrt(np.dot(self.m,U))- self.d*np.power(self.Omega_0,3)/(self.eta*np.power(self.r,3)*self.J_t))

        # u comes in the form [[ 351.0911185   117.65355114  286.29403363  nan]] where nan denotes that this value
        # should be put to 0
        # values goes more or less up to 1500 so they are divided by 15 so that they fall in the 0-100 range.

        if math.isnan(u[0,0]):
            motorPowerM1 = 0
        else:
            motorPowerM1 = u[0,0]/15

        if math.isnan(u[0,1]):
            motorPowerM2 = 0
        else:
            motorPowerM2 = u[0,1]/15

        if math.isnan(u[0,2]):
            motorPowerM3 = 0
        else:
            motorPowerM3 = u[0,2]/15

        if math.isnan(u[0,3]):
            motorPowerM4 = 0
        else:
            motorPowerM4 = u[0,3]/15

        if logging:
            #Log the motor powers:
            print "------------------------"
            print "motorPowerM1 (method 1):" + str(motorPowerM1)
            print "motorPowerM2 (method 1):" + str(motorPowerM2)
            print "motorPowerM3 (method 1):" + str(motorPowerM3)
            print "motorPowerM4 (method 1):" + str(motorPowerM4)
            print "**************************"

        ur = [motorPowerM1, motorPowerM2, motorPowerM3, motorPowerM4] # to be limited
        return ur

    """ Compute the motor voltages from the control inputs
        following bitcraze Crazyflie implementation.

        @returns: u=[u_m1, u_m2, u_m3, u_m3], motor voltages
    """
    def motor_inversion2(self, thrust, roll, pitch, yaw, logging = 0):
        #QUAD_FORMATION_NORMAL
        motorPowerM1 = thrust + pitch + yaw
        motorPowerM2 = thrust - roll - yaw
        motorPowerM3 = thrust - pitch + yaw
        motorPowerM4 = thrust + roll - yaw

        if logging:
            #Log the motor powers:
            print "------------------------"
            print "motorPowerM1 (method 2):" + str(motorPowerM1)
            print "motorPowerM2 (method 2):" + str(motorPowerM2)
            print "motorPowerM3 (method 2):" + str(motorPowerM3)
            print "motorPowerM4 (method 2):" + str(motorPowerM4)
            print "**************************"

        ur = [motorPowerM1, motorPowerM2, motorPowerM3, motorPowerM4] # to be limited
        return ur


    """ Compute the motor voltages from the control inputs
        using a HACKED version of the implementation used in the crazyflie.
        This hack is because the reference frames of Erle are different from the ones
        adopted by the crazyflie.

        M1 <-> M3
        M2 <-> M4

        @returns: u=[u_m1, u_m2, u_m3, u_m3], motor voltages
    """
    def motor_inversion3(self, thrust, roll, pitch, yaw, logging = 0):
        #QUAD_FORMATION_NORMAL
        motorPowerM3 = thrust + pitch + yaw
        motorPowerM4 = thrust - roll - yaw
        motorPowerM1 = thrust - pitch + yaw
        motorPowerM2 = thrust + roll - yaw

        if logging:
            #Log the motor powers:
            print "------------------------"
            print "motorPowerM1 (method 3):" + str(motorPowerM1)
            print "motorPowerM2 (method 3):" + str(motorPowerM2)
            print "motorPowerM3 (method 3):" + str(motorPowerM3)
            print "motorPowerM4 (method 3):" + str(motorPowerM4)
            print "**************************"

        ur = [motorPowerM1, motorPowerM2, motorPowerM3, motorPowerM4] # to be limited
        return ur

    """ Compute the motor voltages from the control inputs following M.Wieremma MS thesis (MSc_thesis_X-UFO).

        THE DYNAMICAL MODEL DOCUMENTED IN THE MS Thesis HAS BEEN HACKED (M1 <-> M3, M2 <-> M4) BECAUSE THE REFERENCE FRAMES ADOPTED
        IN THE DOCUMENT INDICATES THAT THE MOTORS ROTATE IN OPPOSITE DIRECTIONS.

        Keep in mind when passing parameters the following correspondences.

            - U1: thrust
            - U2: roll
            - U3: pitch
            - U4: yaw

        @returns: u=[u_m1, u_m2, u_m3, u_m3], motor voltages
    """
    def motor_inversion4(self, thrust, roll, pitch, yaw, logging = 0):
        # the control inputs
        U = np.array( ((thrust, roll, pitch, yaw)) )
        Um = np.matrix(U).T
        # the motor voltages
        u = (self.k_m * self.tau) * ((1/self.tau + 2*self.d*self.Omega_0/(self.eta*np.power(self.r,3)*self.J_t))\
            * np.sqrt(np.dot(self.m,U))- self.d*np.power(self.Omega_0,3)/(self.eta*np.power(self.r,3)*self.J_t))

        # u comes in the form [[ 351.0911185   117.65355114  286.29403363  nan]] where nan denotes that this value
        # should be put to 0
        # values goes more or less up to 1500 so they are divided by 15 so that they fall in the 0-100 range.

        # FIXED APPLIED DUE TO THE DIFFERENCE IN THE REFERENCE FRAMES

        if math.isnan(u[0,0]):
            motorPowerM3 = 0
        else:
            motorPowerM3 = u[0,0]/15

        if math.isnan(u[0,1]):
            motorPowerM4 = 0
        else:
            motorPowerM4 = u[0,1]/15

        if math.isnan(u[0,2]):
            motorPowerM1 = 0
        else:
            motorPowerM1 = u[0,2]/15

        if math.isnan(u[0,3]):
            motorPowerM2 = 0
        else:
            motorPowerM2 = u[0,3]/15

        if logging:
            #Log the motor powers:
            print "------------------------"
            print "motorPowerM1 (method 4):" + str(motorPowerM1)
            print "motorPowerM2 (method 4):" + str(motorPowerM2)
            print "motorPowerM3 (method 4):" + str(motorPowerM3)
            print "motorPowerM4 (method 4):" + str(motorPowerM4)
            print "**************************"

        ur = [motorPowerM1, motorPowerM2, motorPowerM3, motorPowerM4] # to be limited
        return ur

```

Bluetooth Controller
------
This class ([Code in GitHub](https://github.com/erlerobot/erle_control/blob/master/bt_controller.py)) creates a bluetooth server awaiting for incoming connections. It allows the **simple autopilot** to receive input from the outside (e.g. using and [android app](https://github.com/erlerobot/erle_android)).

``` python
class BT_Controller:
  """
  """

  def __init__(self, thrust_d = 0, pitch_d = 0, roll_d = 0, yaw_d = 0):
    self.thrust_d = thrust_d
    self.pitch_d = pitch_d
    self.roll_d = roll_d
    self.yaw_d = yaw_d
    self.t = threading.Thread(target=self.server, args = (self.thrust_d,))

  def getThrust(self):
    return self.thrust_d

  def run(self):
    self.t.daemon = True
    self.t.start()
    # t.join()

  def stop(self):
    self.t.exit()

  def server(self, thrust):
    """
    """
    while True:
      server_sock=BluetoothSocket( RFCOMM )
      server_sock.bind(("",PORT_ANY))
      server_sock.listen(2)

      port = server_sock.getsockname()[1]

      uuid = "00001101-0000-1000-8000-00805F9B34FB"

      advertise_service( server_sock, "SampleServer",
                         service_id = uuid,
                         service_classes = [ uuid, SERIAL_PORT_CLASS ],
                         profiles = [ SERIAL_PORT_PROFILE ],
    #                     protocols = [ OBEX_UUID ]
                          )

      print("Waiting for connection on RFCOMM channel %d" % port)

      client_sock, client_info = server_sock.accept()
      print("Accepted connection from ", client_info)

      try:
          while True:
              data = client_sock.recv(1024)
              if len(data) == 0: break
              firstByte = data[0]
              firstByte_hexlify = binascii.hexlify(data[0])

              if firstByte == "U":
                """
                The received data follows the following pattern:
                55 [U]
                01 [left joystick "intensity"]
                05 [left joystick "angle"]
                00 [right joystick "intensity"]
                00 [right joystick "angle"]
                """
                # print "U received"
                # print "thrust: "+str(int(binascii.hexlify(data[1]),16)*10)
                self.thrust_d = int(binascii.hexlify(data[1]),16)*10
              elif firstByte == "T":
                print "T received"
              elif firstByte == "A":
                print "A received"
              elif firstByte == "B":
                print "B received"
              elif firstByte == "C":
                print "C received"
              elif firstByte == "D":
                print "D received"
              else:
                # print data
                print "-----not recognized-----"
                for d in data:
                  print binascii.hexlify(d)

      except IOError:
          #pass
          print("disconnected")
          client_sock.close()
          server_sock.close()
```

**For now just the thrust input is wired up**.

Stabilize
-------
The following class ([code in GitHub](https://github.com/erlerobot/erle_control/blob/master/stabilize.py)) puts together all the previous elements and runs the **stabilization loop** at 50 Hz.

``` python
import os
from subprocess import call
from imu import IMU
from motors import Motor
from pid import PID
from time import sleep
import datetime as dt
from dynamical_model import Dynamical_Model
import signal
import sys
from bt_controller import BT_Controller

"""
When testing is important to power off the motors when the
script is stoped through SIGINT
"""
def signal_handler(signal, frame):
        print 'Setting all motors to 0...'
        motor1.setSpeedBrushless(0)
        motor2.setSpeedBrushless(0)
        motor3.setSpeedBrushless(0)
        motor4.setSpeedBrushless(0)

        for mot in motors:
            mot.go()

        # calculate the frequency of the main loop
        sum = 0
        for f in frequencies:
            sum+=f
        print "average frequency (Hz): "+str(sum/len(frequencies))
        print "minimum frequency (Hz): "+str(min(frequencies))

        print "killing the controller thread (bt thread)..."
        # # stop bt-controller NOT WORKING
        # bt.stop()

        sys.exit(0)
```
This handler is programmed so that when we press `Ctrl+C` to stop the stabilize script, the robot doesn't continue working (or even goes crazy). The handler, sets all the motors speed to 0 and outputs an average of the IMU update rate.

``` python

""" Limits the thrust passed to the motors
    in the range (-100,100)
"""
def limitThrust(thrust, upperLimit = 100, lowerLimit = 0):
    if thrust > upperLimit:
        thrust = upperLimit
    elif thrust < lowerLimit:
        thrust = lowerLimit
    return thrust

```
This function allows to limit the thrust. Useful for tests. Make sure you use it if you play with it at home and you care about your lamps.
``` python

###############################
#    INIT
###############################
# Set the handler for SIGINT
signal.signal(signal.SIGINT, signal_handler)

# Activate I2C-2
retvalue = os.system("/bin/echo BB-I2C1 > $SLOTS")

# Activate the motors with the init script
retvalue = os.system("/usr/bin/python /root/erle_control/init_motors.py")

# First time the IMU sensor raises an error (probably an error with the firmware or so). The following program readies the sensor to work properly:
#retvalue = os.system("/root/erle_control/imu/imu -b 2")
call(["/root/erle_control/imu/imu", "-b2"])


############################
# variables
############################

logging = 0
limit_thrust = 70

roll_d = 0
pitch_d = 0
yaw_d = 0
z_d = 0
#xpos = 0
#ypos = 0

# attitude constants
if len(sys.argv) > 1:
    Kp = float(sys.argv[1])
    Kd = float(sys.argv[2])
    Ki = float(sys.argv[3])

else:
    Kp = 0.9
    Kd = 0.2
    Ki = 0

############################

#instantiate the BT controller
bt = BT_Controller(z_d)
bt.run()

#instantiate IMU
#TODO see how to import C interface to python
imu=IMU()
#MyKalman=KalmanFilter(....)

# dynamical model instance
dyn_model = Dynamical_Model()

#instantiate motors and put them together in a list
motor1=Motor(1)
motor2=Motor(2)
motor3=Motor(3)
motor4=Motor(4)
motors=[motor1,motor2,motor3,motor4]

# instantiate PID controllers
rollPID=PID(Kp, Kd, Ki) # Kp, Kd, Ki
pitchPID=PID(Kp, Kd, Ki)
yawPID=PID(0, 0, 0)
zPID=PID(1, 0, 0)
#xposPID=PID(-0.09, -0.1, 0)
#yposPID=PID(-0.09, -0.1, 0)

# test variables
frequencies = []

if logging:
    print "------------------------"
    print "     stabilize loop     "
    print "------------------------"
############################
#loop
############################
```
This is the main loop. Data is acquired from the IMU (using the DMP), the PIDs are updated and the output is forwarded to the motors.
``` python
while 1:
    start = dt.datetime.now()

    # process the input from the controller
    z_d = bt.getThrust()
    # print "z_d: "+str(z_d)
    # pitch, roll and yaw DESIRED, get FROM THE TELEOPERATOR (for now, global vars are used)
    # roll_d = 0
    # pitch_d = 0
    # yaw_d = 0
    # z_d = 10
    # #xpos = 0
    # #ypos = 0

    #Measure angles
    #roll_m, pitch_m, yaw_m = imu.read_fusedEuler()
    roll_m, pitch_m, yaw_m = imu.read_fusedEuler(0)
    #MyKalman.measure([roll,pitch, yaw])

    z_m = 0 # putting this is the same as telling it to always increase the thrust (go up)

    #Run the PIDs
    roll = rollPID.update(roll_d - roll_m, 0)
    pitch = pitchPID.update(pitch_d - pitch_m, 0)
    yaw = yawPID.update(yaw_d - yaw_m, 0)
    z = zPID.update(z_d - z_m, 0)
    #xpos = xposPID.update(xpos_d - xpos_m)
    #ypos = yposPID.update(ypos_d - ypos_m)

    #TODO change this parameter and see the behaviour
    #thrust is provided by the controller (NOTE: this is also treated as "z" and it should use the zPID controller)
    # the point of hovering is 35% duty cycle in the motors


    if logging:
        #Log the values:
        print "**************************"
        print "Desired angles:"
        print "     pitch:" + str(pitch_d)
        print "     roll:" + str(roll_d)
        print "     yaw:" + str(yaw_d)
        print "     z:" + str(z_d)
        print "Measured angles:"
        print "     pitch:" + str(pitch_m)
        print "     roll:" + str(roll_m)
        print "     yaw:" + str(yaw_m)
        # print "     thrust (z):" + str(z_d)    # maybe using some sensor?
        print "PID output angles:"
        print "     pitch:" + str(pitch)
        print "     roll:" + str(roll)
        print "     yaw:" + str(yaw)
        print "     z:" + str(z)

    # using M. Wieremma's thesis
    #u = dyn_model.motor_inversion1(z, roll, pitch, yaw, logging)

    # using Crazyflie's implementation
    #u = dyn_model.motor_inversion2(z, roll, pitch, yaw, logging)

    # using Crazyflie's HACKED implementation
    #u = dyn_model.motor_inversion3(z, roll, pitch, yaw, logging)

    # using M. Wieremma's thesis HACKED impl
    u = dyn_model.motor_inversion4(z, roll, pitch, yaw, logging)

    #if the controller says that the thrust is 0, all motors to 0
    if z_d == 0:
        motorPowerM1 = 0;
        motorPowerM2 = 0;
        motorPowerM3 = 0;
        motorPowerM4 = 0;
    else:
        motorPowerM1 = limitThrust(u[0], limit_thrust);
        motorPowerM2 = limitThrust(u[1], limit_thrust);
        motorPowerM3 = limitThrust(u[2], limit_thrust);
        motorPowerM4 = limitThrust(u[3], limit_thrust);

    if logging:
        #Log the motor powers:
        print "------------------------"
        print "motorPowerM1 (limited):" + str(motorPowerM1)
        print "motorPowerM2 (limited):" + str(motorPowerM2)
        print "motorPowerM3 (limited):" + str(motorPowerM3)
        print "motorPowerM4 (limited):" + str(motorPowerM4)
        print "**************************"

    #Set motor speeds
    motor1.setSpeedBrushless(motorPowerM1)
    motor2.setSpeedBrushless(motorPowerM2)
    motor3.setSpeedBrushless(motorPowerM3)
    motor4.setSpeedBrushless(motorPowerM4)

    #Start Motors
    for mot in motors:
        mot.go()


    #Kalman Prediction
    #MyKalman.predict()


    # calculate the time each iteration takes
    time_u = (dt.datetime.now() - start).microseconds
    time_s = time_u/1e6
    frequency = 1e6/time_u

    # force 50 Hz loop
    if time_s < 20e-3: #50 Hz
        sleep(20e-3 - time_s)

    # # force 60 Hz loop
    # if time_s < 16.66e-3: #50 Hz
    #     sleep(16.66e-3 - time_s)

    # # force 70 Hz loop
    # if time_s < 14.28e-3: #50 Hz
    #     sleep(14.28e-3 - time_s)


    time_u = (dt.datetime.now() - start).microseconds
    frequency = 1e6/time_u
    frequencies.append(frequency)


############################

```

The output of this algorithm can be seen [here](https://www.youtube.com/watch?v=Ry3MyQYa7hY) and [here](https://www.youtube.com/watch?v=kLQffwO8k4U). Not bad for a simple autopilot right?

Still, there's much room for improvement. Here are some suggestions:

- Code a (python) driver for the MPU9150 that gets raw data from the gyroscopes, acelerometers and magnetometers.
- Implement a module/method/function that gets euler and quaternions from raw data.
- Implement a Kalman Filter (the kalman prediction commented code)
- Improve the current 50 Hz loop to 100 Hz or more (for this the previous two suggestions should be accomplished first)

Feel free to reach us if you are interested in continuing learning ;).

Sources:
--------

- [Quadcopter PID Explained and Tuning](http://blog.oscarliang.net/quadcopter-pid-explained-tuning/)
- [Introduction: PID Controller Design](http://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlPID)
