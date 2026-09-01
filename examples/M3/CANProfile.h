#define TARGET_CNTL_CMD_OFFSET 0x20
#define TARGET_REQUEST_CMD_OFFSET 0x30
#define VSENSE_CMD_OFFSET 0x40
#define ESTOP 0xff
#define MOTOR_ID 0x03 // M3
#define TARGET_CNTL_CMD (MOTOR_ID + TARGET_CNTL_CMD_OFFSET)
#define TARGET_REQUEST_CMD (MOTOR_ID + TARGET_REQUEST_CMD_OFFSET)
#define VSENSE_CMD (MOTOR_ID + VSENSE_CMD_OFFSET)

// phase resistance of the motor = internal resistance / 2
// 17.0Ω / 2 = 8.5Ω
#define PHASE_RESISTANCE 8.5
// Seting a voltage limit too high can cause the motor to overheat and damage the driver.
// reference: https://docs.simplefoc.com/voltage_torque_mode#configuration-and-torque-limits
#define VOLTAGE_LIMIT 8.0

typedef enum { SET_TARGET = 0, REQUEST_TARGET, NONE } MESSAGE_STATUS;