#define TARGET_CNTL_CMD_OFFSET 0x20
#define TARGET_REQUEST_CMD_OFFSET 0x30
#define ESTOP 0xff
#define MOTOR_ID 0x03 // M3
#define TARGET_CNTL_CMD (MOTOR_ID + TARGET_CNTL_CMD_OFFSET)
#define TARGET_REQUEST_CMD (MOTOR_ID + TARGET_REQUEST_CMD_OFFSET)

// phase resistance of the motor = internal resistance / 2
// 14.0Ω / 2 = 7.0Ω
#define PHASE_RESISTANCE 7.0

typedef enum { SET_TARGET = 0, REQUEST_TARGET, NONE } MESSAGE_STATUS;

class PingPongNotificationsFromCAN {
public:
  virtual void SetMotorPosition(const uint8_t *data) = 0;
  virtual void ReturnMotorTarget() = 0;
  virtual void EmergencyStop() = 0;
};

class CANPingPong : public SimpleCANProfile {
public:
  CANPingPong(SimpleCan *pCan, PingPongNotificationsFromCAN *_pRxCommands)
      : SimpleCANProfile(pCan) {
    pRxCommands = _pRxCommands;
  }

  void CANRequestCommand(uint8_t CanId) { Can1->RequestMessage(0, CanId); }

  void HandleCanMessage(const SimpleCanRxHeader rxHeader,
                        const uint8_t *rxData) {
    // Serial1.println("@");

    switch (rxHeader.Identifier) {
    case TARGET_CNTL_CMD:
      pRxCommands->SetMotorPosition(rxData);
      break;
    case TARGET_REQUEST_CMD:
      pRxCommands->ReturnMotorTarget();
      break;
    case ESTOP:
      Serial1.println("ESTOP");
      pRxCommands->EmergencyStop();
      break;
    default:
      Serial1.printf("y:0x%x DLC=0x%x ", rxHeader.Identifier,
                     rxHeader.DataLength);
      Serial1.println();
    }
  }

private:
  PingPongNotificationsFromCAN *pRxCommands;
};
