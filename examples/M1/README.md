# M1
M1 motor changes the stator angle and keeps the stator position.

Changes
- Decrease `LPF_velocity.Tf` value to react faster to changes in velocity.
- Increase `P_angle.P` value to improve position control responsiveness.

> [!TIP]
> For large stators (OD > 81mm), you may need to decrease `P_angle.P` value to prevent overshooting. 
(e.g., `P_angle.P = 5` for OD = 83mm)