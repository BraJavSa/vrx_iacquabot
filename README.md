# Virtual RobotX (VRX)
This repository is the home to the source code and software documentation for the VRX simulation environment, which supports simulation of unmanned surface vehicles in marine environments.



## High-Fidelity Thruster Curve Integration (Fork Changes)

This fork updates the WAM-V model to use a high-fidelity **Asymmetric Richard's Generalized Logistic Curve** mapping instead of an idealized linear model, aligning simulation forces with real-world BlueRobotics T200 thruster dynamics.

### Thruster Curve Mathematical Formulation

The normalized thruster command $u \in [-1, 1]$ is mapped to actual thrust force $T$ (in Newtons) using the following asymmetric formulation:

$$T(u) = A + \frac{K - A}{\left( C + e^{-B(u - M)} \right)^{\frac{1}{v}}}$$

Where the parameters are configured as follows:

| Direction | Parameter $A$ | Parameter $K$ | Parameter $B$ | Parameter $v$ | Parameter $C$ | Parameter $M$ | Max Thrust (N) |
|---|---|---|---|---|---|---|---|
| **Positive ($u > 0.01$)** | $1.0 \times 10^{-6}$ | $40.0209$ | $2.6249$ | $0.1615$ | $0.9432$ | $1.0 \times 10^{-5}$ | $+36.38\text{ N}$ |
| **Negative ($u < -0.01$)** | $-31.4990$ | $-1.0 \times 10^{-5}$ | $3.6986$ | $0.3264$ | $0.9713$ | $-1.0000$ | $-28.44\text{ N}$ |

Commands inside the deadband $|u| \le 0.01$ output exactly $0.0\text{ N}$.

### Build and Launch Instructions

To compile the workspace and launch the simulation environment with Nvidia GPU offloading:

```bash
colcon build --merge-install --symlink-install
source install/setup.bash
```
For launch:
```
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia ros2 launch vrx_gz competition.launch.py
```

## Reference

If you use the VRX simulation in your work, please cite our summary publication, [Toward Maritime Robotic Simulation in Gazebo](https://wiki.nps.edu/display/BB/Publications?preview=/1173263776/1173263778/PID6131719.pdf):

```
@InProceedings{bingham19toward,
  Title                    = {Toward Maritime Robotic Simulation in Gazebo},
  Author                   = {Brian Bingham and Carlos Aguero and Michael McCarrin and Joseph Klamo and Joshua Malia and Kevin Allen and Tyler Lum and Marshall Rawson and Rumman Waqar},
  Booktitle                = {Proceedings of MTS/IEEE OCEANS Conference},
  Year                     = {2019},
  Address                  = {Seattle, WA},
  Month                    = {October}
}
```


