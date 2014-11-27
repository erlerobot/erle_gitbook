Un piloto automático simplificado
==========

Este módulo introduce un sencillo piloto automático programado en `python` que ofrece las siguiente caraterísticas:

* Bucle de control usando *P*roporcional *I*ntegral *D*erivada (PID)
* 4 señales de salida utilizando PyBBIO
* Sensores de entrada,IMU (InvenSense MPU9150), a una frecuencia de 50Hz 
* Enlace Bluetooth para proporcionar un entrada externa

El código se puede encontrar [aqui](https://github.com/erlerobot/erle_control).
En las siguientes secciones se abordarán los diferentes módulos necesarios para implementaar un **piloto automático sencillo**

Motors
-----

Esta clase ([código en GitHub](https://github.com/erlerobot/erle_control/blob/master/motors.py)) pertime controlar los motores de velocidad mediante modulación de ancho de pulso (**PWM**).

``` python

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
````
Los pines complatibles eHRPWM( capaces de una alta resolución PWM). Consulta la [hoja de características del procesador](http://www.ti.com/product/am3359) para más información.

``` python
        if (motor_number > 4) or (motor_number < 1):
            raise Exception("Motor number provided out of bounds! ([1-4])")
        self.motor_number = motor_number # 1, 2, 3 or 4
        self.max_speed= max_speed
        self.min_speed= min_speed
```
`max_speed` y `min_speed` ayudan  mientras se esta testeando.
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
La clase esta programada inicialmente para *motores brushed* y posteriormente fue adaptada para funcionar escobillas.
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
después de ajustar la velocidad deseada con `setSpeedBrushless` ( el método brushed esta obsoleto), el método `go` envía la acción cada ciclo al submodulo eHRPWM.
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
La siguiente clase([código en GitHub](https://github.com/erlerobot/erle_control/blob/master/imu.py)) esta programada para el sensor *InvenSense MPU9150*. El código hace uso del controlador del sensor en Linux, compilado como una libreria compartida (`libimu.so`) a través de `python Ctypes`:

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
Compatible con las definiciones de C-types.
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
Se esta utilizando el sensor DMP porque lo que es lo más rápido que podemos funcionar.
``` python
        self.yaw_mix_factor = 3

        # initialize the IMU
        res = self.lib.mpu9150_init(self.i2c_bus, self.sample_rate, self.yaw_mix_factor)
        if res:
            Exception("Error when initializing the IMU!")
```
El *archico de calibración* debería haber sido generado previamente. Comprueba el  [código del driver](https://github.com/erlerobot/erle_control/tree/master/imu) para más información.
``` python
        # set calibration files
        res = self.lib.set_cal(0, "./imu/accelcal.txt")
        if res != 0:
            Exception("Error while calibration: accelcal.txt")
        res = self.lib.set_cal(1, "./imu/magcal.txt")
        if res != 0:
            Exception("Error while calibration: magcal.txt")
```
Los siguiente métodos reciben los datos de las funciones en C. Esto se hace para que el código pueda ser escrito en python pero *usando el controlador escrito en C*.
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
Esta clase ([código en GitHub](https://github.com/erlerobot/erle_control/blob/master/pid.py)) implementa un sencillo algoritmo de control PID. Las ganancias `K_p` (proporcional), `K_d` (derivada) and `K_i` (integral) se pueden pasar como parámetros al inicializar la clases.

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
Setters para las ganancias.
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
Un flag para la ` depuración ` es incluido. Si esta a `true`, el método ` update ` de volvera **verbose** 
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
`terms_sum` implementa la ecuación del control PID.

\begin{equation}u(t) = K_p \cdot e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de(t)}{dt}\end{equation}

``` python
        if debug:
            print "   Terms sum (self.Cp + (self.Ki * self.Ci) + (self.Kd * self.Cd)):"+str(terms_sum)
            print "   ****************************"
        return terms_sum
```

### PID Calibration
#### Intuition

| Respuesta CL   | Tiempo de ascenso  | OVERSHOOT  | Tiempo de establecimiento | S-S ERROR |
| ------------- |:-------------:| -----:|------------------|----------|
| Kp | Reducir | Incrementar |  Pequeño cambio| Reducir |
| Ki | Reducir | Incrementar | Increase | Eliminar |
| Kd | Pequeño cambio | Reducir | Decrease | No cambiar |


Los quadricopteros son simetricos, por lo tanto se pueden establecer los mismo valores del PID para el *pitch* y *roll*. El valor de desvío no es tan importante como los de *pitch* y *roll*. Así que para comenzar se establecen los mismos valores para *pitch* y *roll* (aunque tal vez no sea lo mejor). Después de que su drone este relativamente estable, puede empezar a alterar las ganancias *Yaw*. Para drones no simétricos como *Hexacopteros* o *Tricopteros*, es posible que afinar el *pitch* y *roll* por separado, después de haber tenido un poco de experiencia de vuelo.

#### Ajuste de la P
Para la ganancia P, empezar poco a poco, hasta que se note que se esta produciendo la oscilación. El ajuste fino se realiza hasta que el movimiento no sea lento ni oscilaciones.

#### Ajuste de la I 
Para la ganancia I, de nuevo empezar despacio e ir increcementado lentamente. Hay que prestar atención al tiempo que necesita para detenerse y estabilizarse. Se quiere llegar a un punto donde se estabilice muy rápido y que no se pase mucho tiempo dando vueltas. Para obtener un mejor valor de I se puede poner a prueba bajo condiciones de viento.

#### Ajuste de la D 
Para la ganancia D, puede entrar en una complicada interacción con los valores P e I. Cuando se estabiliza la ganancia D, es necesario volver atrás y afinar P para mantener bien estabilizada la planta.

Modelo dinámico
---------

Esta clase ([código en GitHub](https://github.com/erlerobot/erle_control/blob/master/dynamical_model.py)) implementa el **modelo dinámico** del cuadricoptero:

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

Controlador Bluetooth
------
Esta clase ([Código en GitHub](https://github.com/erlerobot/erle_control/blob/master/bt_controller.py)) crea un servidor bluetooth que espera conexiones entrantes. Pertime  al **piloto automático** recivir la entradas desde el exterior (Por ejemplo usando una [aplicación Android](https://github.com/erlerobot/erle_android))

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

**Por ahora solo las entradas que se usan están cableadas**.

Estabilización
-------
La siguiente clase ([código en GitHub](https://github.com/erlerobot/erle_control/blob/master/stabilize.py)) reúne todos los elementos anteriores y ejecuta el bucle de estabilización a 50 Hz.

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
Este controlador esta programado para que cuando se prescione `Ctrl+C` detenga el script de estabilización, el robot no continua funcionando (o incluso se vuelve loco).El controlador, establece la velocidad de motores a 0 y da salida a una medida de la velocidad de actualización de la IMU.

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
Esta función permite limitar la potencia. Es util para pruebas. Asegurate de utilizarlo si lo utilizas en casa y ten cuidado con las lamparas.
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
############################

```

La salida de este algoritmo se puede ver [aqui](https://www.youtube.com/watch?v=Ry3MyQYa7hY) y [aqui](https://www.youtube.com/watch?v=kLQffwO8k4U). Nada mal para un piloto automático, ¿verdad?

Sin embargo, hay mucho margen de mejora. He aquí algunas sugerencias:

- Un driver en Python para el MPU9150  que reciba datos en bruto de los giroscopios y magnetómetros y acelerómetros.
- Implementar un modulo/método/función que devuelva los ángulos de Euler o Quaterniones desde los datos en bruto.
- Implementar un filtro de Kalman.
- Mejorar los 50 Hz a 100 Hz o más (para ellos las dos sugerencias anteriores deben de llevarse a cabo).

No dude en contactar con nosotros si está interesado en continuar aprendiendo.

Fuentes:
--------

- [Quadcopter PID Explained and Tuning](http://blog.oscarliang.net/quadcopter-pid-explained-tuning/)
- [Introduction: PID Controller Design](http://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlPID)
