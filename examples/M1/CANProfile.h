/* Symbolic names for ID of CAN message                                      */
typedef enum {
  M0_ANGLE_CNTL = 0x20,
  M1_ANGLE_CNTL,
  M2_ANGLE_CNTL,
  M3_ANGLE_CNTL,
  M0_POS,
  M1_POS,
  M2_POS,
  M3_POS,
  ESTOP
} CAN_ID;

// change these to match the motor control IDs
#define ANGL_CNTL_CMD M1_ANGLE_CNTL
#define ANGL_REQUEST_CMD M1_POS

/* Symbolic names for ID of Motor                                            */
typedef enum { MOTOR_0_ID = 0, MOTOR_1_ID, MOTOR_2_ID, MOTOR_3_ID } MOTOR_ID;

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
