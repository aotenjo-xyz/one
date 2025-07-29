#define ANGLE_COMMAND_OFFSET 0x20
#define POS_COMMAND_OFFSET 0x30
#define ESTOP 0xff
#define MOTOR_ID 0x02 // M2
#define ANGL_CNTL_CMD (MOTOR_ID + ANGLE_COMMAND_OFFSET)
#define ANGL_REQUEST_CMD (MOTOR_ID + POS_COMMAND_OFFSET)

typedef enum { SET_POSITION = 0, REQUEST_POSITION, NONE } MESSAGE_STATUS;

class PingPongNotificationsFromCAN {
public:
  virtual void SetMotorPosition(const uint8_t *data) = 0;
  virtual void ReturnMotorPosition() = 0;
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
    case ANGL_CNTL_CMD:
      pRxCommands->SetMotorPosition(rxData);
      break;
    case ANGL_REQUEST_CMD:
      pRxCommands->ReturnMotorPosition();
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
