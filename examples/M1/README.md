# M1
M1 motor changes the stator angle and keeps the stator position.

Changes
- Decrease `LPF_velocity.Tf` value to react faster to changes in velocity.
- Increase `P_angle.P` value to improve position control responsiveness.

e.g.
```
LPF_velocity.Tf: 0.005
P_angle.P: 25.0
```

- Change the line1 in `src/CANProfile.h` to set the motor ID to `0x01` for M1 like the following:
```cpp
#define MOTOR_ID 0x01 // M1
```

> [!TIP]
> For large stators (OD > 81mm), you may need to decrease `P_angle.P` value to prevent overshooting. 
(e.g., `P_angle.P = 5` for OD = 83mm)