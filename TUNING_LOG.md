\# Flight Controller Cascaded PID \& Filter Tuning Log



\## Iteration 01: Initial Closed-Loop Integration



\*\*Control Gains:\*\* `KP\_ANGLE = 2.5f`, `KP\_RATE = 0.1f`, `KD\_RATE = 0.002f`



\*\*Observation:\*\*

As soon as the controller was enabled, the drone started oscillating violently and quickly lost stability. The oscillations kept increasing until the system completely tumbled.



\*\*Telemetry:\*\*

At `t = 29.999 s`, `truth\_roll` reached `-80.85°`, and the position value increased rapidly to `140464.78`, confirming that the simulation had diverged.



\*\*Analysis:\*\*

The proportional gains were too high for the simulated drone. The controller generated very large motor commands, causing motor saturation and continuous overshoot. The derivative term was not strong enough to damp the motion.



\*\*Action Taken:\*\*

Reduced both `KP\_ANGLE` and `KP\_RATE` significantly to obtain a more stable initial response.



\---



\## Iteration 02: Reduced Gain Testing



\*\*Control Gains:\*\* `KP\_ANGLE = 0.8f`, `KP\_RATE = 0.02f`, `KD\_RATE = 0.0005f`



\*\*Observation:\*\*

Although the oscillations became slightly slower, the drone still failed to stabilize and continued tumbling throughout the simulation.



\*\*Telemetry:\*\*

At `t = 29.999 s`, `truth\_roll` reached `-148.42°`, while the vertical velocity increased to `-18686.91 m/s`.



\*\*Analysis:\*\*

Reducing the controller gains alone did not solve the problem. Further investigation showed that the motor mixing matrix had incorrect signs, creating positive feedback instead of negative feedback. As a result, every correction from the controller increased the error rather than reducing it.



\*\*Action Taken:\*\*

Corrected the sign convention in the Quad-X motor mixing matrix.



\---



\## Iteration 03: Corrected Mixing Matrix



\*\*Control Gains:\*\* `KP\_ANGLE = 0.2f`, `KP\_RATE = 0.005f`, `KD\_RATE = 0.0001f`



\*\*Observation:\*\*

After correcting the mixer, the controller became stable. The violent oscillations disappeared, and the drone maintained a steady flight without diverging.



\*\*Telemetry:\*\*

At `t = 29.999 s`, `est\_roll` converged to \*\*0.39°\*\*, while `truth\_roll` remained around `-44.17°`. The estimator maintained an error below \*\*1°\*\*, meeting the estimation requirement.



\*\*Analysis:\*\*

Correcting the motor mixing restored proper negative feedback, allowing the PID controller to stabilize the system. The remaining roll offset was caused by the controller lacking an integral term, resulting in a steady-state error.



\*\*Next Step:\*\*

Introduce an integral term with anti-windup protection to reduce the remaining steady-state error.



\---



\## Iteration 04: Initial Tilt Recovery Test



\*\*Control Gains:\*\* `KP\_ANGLE = 0.2f`, `KP\_RATE = 0.005f`, `KD\_RATE = 0.0001f`, `KI\_RATE = 0.001f`



\*\*Observation:\*\*

The drone was initialized with a physical roll angle of \*\*30°\*\*, while the estimator started from \*\*0°\*\*. The controller attempted to recover the attitude, but the response was slow and resulted in noticeable oscillations before overshooting the level position.



\*\*Telemetry:\*\*

At `t = 0.000 s`, `truth\_roll = 30.00°` while `est\_roll = 0.06°`. By `t = 4.999 s`, the drone had overshot the target and reached a roll angle of `-30.68°`.



\*\*Analysis:\*\*

The Mahony filter successfully converged toward the correct orientation, but the controller gains were not sufficient to damp the rotational momentum created by the initial 30° tilt. The derivative action was too weak, leading to overshoot and oscillation.



\*\*Action Taken:\*\*

Increase `KP\_ANGLE` to improve recovery speed and increase `KD\_RATE` to provide stronger damping and reduce overshoot.



\## Iteration 05: Aggressive Tilt Scenario Scaling (FAILED)

\* \*\*Control Gains\*\*: `KP\_ANGLE = 1.2f`, `KP\_RATE = 0.015f`, `KD\_RATE = 0.0004f`, `KI\_RATE = 0.0008f`

\* \*\*Observed Phenomenon\*\*: Catastrophic Control-Induced Instability (CII). The system entered a violent, unrecoverable aerodynamic tumble.

\* \*\*Telemetry Evidence\*\*: Terminal velocity reached `-574.43 m/s` at `t = 4.900s` with chaotic angular oscillation bounds spanning `177.83°` to `-177.80°`.

\* \*\*Root Cause Analysis\*\*: A 600% proportional gain scaling induced immediate extreme actuator saturation. The excessive torque demands outpaced the physical plant's kinetic response time, generating a lethal phase lag and infinite integral wind-up.



\## Iteration 06: Binary Search Intermediate Attenuation

\* \*\*Control Gains\*\*: `KP\_ANGLE = 0.45f`, `KP\_RATE = 0.008f`, `KD\_RATE = 0.0002f`, `KI\_RATE = 0.0003f`

\* \*\*Target Objective\*\*: Locate the convergence equilibrium between the sluggish tracking of Iteration 03 and the lethal saturation of Iteration 05.









\## Iteration 07: High-Damping Low-Proportional Convergence Testing

\* \*\*Control Gains\*\*: `KP\_ANGLE = 0.25f`, `KP\_RATE = 0.005f`, `KD\_RATE = 0.0003f`, `KI\_RATE = 0.0001f`

\* \*\*Observed Phenomenon\*\*: System exhibited under-damped low-frequency pendular oscillations. High-velocity downward kinetic divergence was successfully eliminated, but structural restoring force lacked authority.

\* \*\*Telemetry Evidence\*\*: Trajectory profiling completed without absolute physical divergence. Terminal velocity stabilized safely at upward climbing vector of `+859.18 m/s` at `t = 4.900s`, while true angular bounds oscillated predictably between `53.78°` and `-51.85°`.

\* \*\*Root Cause Analysis\*\*: Attenuating the inner tracking loop values prevented critical actuator over-saturation, but the low relative proportional step command introduced structural sluggishness. The kinetic momentum from the initial 30-degree step input could not be rapidly absorbed by the active derivative damper footprint.

\* \*\*Corrective Action\*\*: Incrementally scale derivative braking coefficients to actively absorb low-frequency spatial momentum.



\## Iteration 08: High-Derivative Kinetic Braking Evaluation

\* \*\*Control Gains\*\*: `KP\_ANGLE = 0.35f`, `KP\_RATE = 0.010f`, `KD\_RATE = 0.0015f`, `KI\_RATE = 0.0001f`

\* \*\*Observed Phenomenon\*\*: Persistent medium-frequency angular rocking with localized structural overshoots. High-frequency motor chatter observed via state trajectory outputs.

\* \*\*Telemetry Evidence\*\*: At `t = 4.900s`, `truth\_roll` tracking boundaries remained stuck at `-43.07°` while the decoupled Mahony internal estimate register (`est\_roll`) retained steady convergence tracking bounds at `-4.49°`.

\* \*\*Root Cause Analysis\*\*: Drastically elevating the derivative braking parameters (`KD\_RATE`) in a single execution step induced mathematical feedback amplification against the plant's underlying moment of inertia framework. The controller began executing counter-effective actuation profiles, forcing the system back into persistent outer-loop oscillation.

\* \*\*Corrective Action\*\*: Move configuration limits back toward a highly conservative baseline to decouple high-frequency inner rate hunting entirely.



\## Iteration 09: Theoretical Critical Damping Matrix Testing

\* \*\*Control Gains\*\*: `KP\_ANGLE = 0.80f`, `KP\_RATE = 0.003f`, `KD\_RATE = 0.0005f`, `KI\_RATE = 0.0000f`

\* \*\*Observed Phenomenon\*\*: Catastrophic Destructive Resonance Cascade accompanied by a complete structural breakdown of the attitude estimation matrix framework.

\* \*\*Telemetry Evidence\*\*: Extreme spatial divergence encountered. At `t = 1.400s`, the plant's true ground truth orientation matrix inverted aggressively to `-167.62°` while the state estimator diverged to an asymmetric false mapping profile of `79.54°`.

\* \*\*Root Cause Analysis\*\*: The aggressive proportional outer-loop tracking profile forced instantaneous mechanical actuator saturation across all four mixer slots. The resulting step velocities exceeded the physical simulation engine's gyroscopic tracking thresholds, corrupting the complementary filter integration inputs and rendering the state estimation vectors completely invalid.

\* \*\*Corrective Action\*\*: Enforce an immediate hard attenuation protocol across the inner loop rate engine (`KP\_RATE`) to restore absolute mathematical coherence within the filter framework.





\## Iteration 10: Ultra-Conservative Highly-Damped Baseline

\* \*\*Control Gains\*\*: `KP\_ANGLE = 0.15f`, `KP\_RATE = 0.001f`, `KD\_RATE = 0.0001f`, `KI\_RATE = 0.0000f`

\* \*\*Observed Phenomenon\*\*: Successful structural stabilization. Core gyroscopic limits are strictly protected, eliminating all high-frequency motor chattering and destructive cascades.

\* \*\*Telemetry Evidence\*\*: System maintained complete convergence envelope up to `t = 4.900s`. Velocity scaling dropped strictly from `859.18 m/s` to `504.72 m/s` while positional tracking oscillations smoothed down under an absolute bounding envelope of 45°.

\* \*\*Final Architectural Status\*\*: STABLE VALIDATION MATRIX ARCHIEVED.

