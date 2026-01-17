# M1
M1 motor changes the stator angle and keeps the stator position.

Changes
- Decrease `LPF_velocity.Tf` value to react faster to changes in velocity.
- Increase `P_angle.P` value to improve position control responsiveness.

For eaglepower motors, you don't need to change these values. If you do, the motor may vibrate.