# Autopilot

Un **Autopiloto es un sotfware que proporciona asistencia mientras se controla un drone**. Mientras que muchos autopilotos permiten al drone volar/moverse siguiendo algunos puntos geográficos (por ejemplo GPS) de manera autónoma, el movimiento autónomo es una característica del autopiloto, pero no es autopiloto.

De acuerdo con la [Wikipedia](http://en.wikipedia.org/wiki/Autopilot):

>An autopilot is a system used to control the trajectory of a vehicle without constant 'hands-on' control by a human operator being required. Autopilots do not replace a human operator, but assist them in controlling the vehicle, allowing them to focus on broader aspects of operation, such as monitoring the trajectory, weather and systems.[1] Autopilots are used in aircraft, boats (known as self-steering gear), spacecraft, missiles, and others. Autopilots have evolved significantly over time, from early autopilots that merely held an attitude to modern autopilots capable of performing automated landings under the supervision of a pilot.

To understand this better, let's analyze different ways of controlling a **quadcopter**, a rotary flying drone:

![quad-control](../img/quad-control.png)

Depending on the controlled variable the perception of the control of a quadrotor by a pilot is perceived diﬀerently. According to the image, the easiest case for the pilot is controlling the **desired ($_d$) positions** through $x_d$, $y_d$ and $z_d$ (there's still one more level which corresponds to complete autonomous flight where the pilot can set desired begin and endpoint).

```
The task of an autopilot is to abstract the user from the different physical parameters (such as velocity, angular rates or moments) and offer a simple interface so that the piloting is as easy as possible.
```

### Autopilots in Erle

We have been working hard on [BeaglePilot](../beaglepilot/BeaglePilot.md), a complete Linux-based autopilot based on ardupilot that provides all the necessary tools and has been built by a collaboration between different entities and contributors.

Although *it shouldn't be used in real drones*, we also provide a [simplified autopilot](../beaglepilot/SimpleAutopilot.md) implemented in python that should be used for pedagogical matters.

###Sources

- [Design, implementation and ﬂight test of indoor navigation and control system for a quadrotor UAV](http://www.st.ewi.tudelft.nl/~koen/in4073/Resources/MSc_thesis_X-UFO.pdf)
- [Wikipedia](http://en.wikipedia.org/wiki/Autopilot)


