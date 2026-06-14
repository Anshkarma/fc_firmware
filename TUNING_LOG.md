\# Control Loop Tuning \& Optimization Log



\## Iteration 01: Initial Closed-Loop Integration

\* \*\*Gains\*\*: KP\_ANGLE = 2.5, KP\_RATE = 0.1, KD\_RATE = 0.002

\* \*\*Phenomenon\*\*: High-frequency dynamic divergence (Violent Angular Oscillation). 

\* \*\*Telemetry Evidence\*\*: At t = 29.999s, `truth\_roll` inverted completely to -80.85° with position scaling exponentially to 140464.78.

\* \*\*Root Cause Analysis\*\*: Proportional angle tracking loop was highly over-aggressive for the plant's moment of inertia, driving actuators into saturation before the derivative dampening could counter the momentum.

\* \*\*Corrective Action\*\*: Heavily attenuate KP\_ANGLE and KP\_RATE to stabilize the system baseline.

