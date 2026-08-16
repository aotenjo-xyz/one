# M3
M3 motor controls the wire tension using closed‑loop torque control in voltage mode, as defined by the SimpleFOC framework.

For background, see:

[Torque Control — SimpleFOC Documentation](https://docs.simplefoc.com/torque_control)

Changes
- Torque control with [voltage mode](https://docs.simplefoc.com/voltage_torque_mode)
- The motor torque is now commanded directly via the q‑axis voltage (targetVoltage), allowing smooth and predictable tension control.

> [!TIP]
> Please update the `PHASE_RESISTANCE` value in `src/CANProfile.h` according to your motor's internal resistance. The value should be set to half of the internal resistance of the motor. For example, if the internal resistance is 17.0Ω, set `PHASE_RESISTANCE` to 8.5Ω.

> [!WARNING]
> You can increase the `VOLTAGE_LIMIT` value in `src/CANProfile.h` to allow for higher voltage output, but be cautious as this may lead to overcurrent conditions. 
> You may increase this value to allow higher torque output, but be aware:
> - Higher voltage → higher phase current
> - Higher current → risk of overheating or exceeding DRV8313’s 2.5 A limit
> - Incorrect values may damage the driver or power supply
> 
> The default value is set to 3.0V, which is a safe limit for most applications. For detailed guidance, see:
> 
> [Configuration and Torque Limits - SimpleFOC documentation](https://docs.simplefoc.com/voltage_torque_mode#configuration-and-torque-limits).
