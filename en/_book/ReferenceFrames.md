Reference Frames
=======

The reference frames are defined as:

- The earth-fixed inertial reference frame ($0_e$$X_e$$Y_e$$Z_e$) is a right-handed orthogonal axis-system with the origin at the quadrotor’s centre of gravity at the beginning of the considered motion. This reference frame is fixed to the earth and is considered as the inertial frame of reference under simplifying conditions.
- The body-fixed reference frame ($0_b$$X_b$$Y_b$$Z_b$) is a right-handed orthogonal axis-system with the origin at the quadrotor’s centre of gravity. The reference frame remains fixed to the quadrotor even in perturbed motion.

![frames](img/frames.png)

The absolute position of the quadrotor is described by the three coordinates (x,y,z) of the centre of mass with respect to the earth reference frame. Its absolute attitude is described by the three Euler’s angles ($\psi$,$\theta$,$\phi$). These three angles
are respectively called yaw angle (−$\pi$ ≤ $\psi$ < $\pi$), pitch angle (−$\frac{\pi}{2}$ < $\theta$ < $\frac{\pi}{2}$) and roll angle (−$\frac{\pi}{2}$ < $\phi$ < $\frac{\pi}{2}$) (*this last assumption is because for now the quadrotor will not perform aerobatics*), it is possible to use the Euler angles in the boundaries given.

Sources
-----
- [Design, implementation and ﬂight test of indoor navigation and control system for a quadrotor UAV](http://www.st.ewi.tudelft.nl/~koen/in4073/Resources/MSc_thesis_X-UFO.pdf)
