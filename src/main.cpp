// clang-format off
#include <Arduino.h>
#include <math.h>
#include "SimpleCAN.h"
#include "CANProfile.h"
#include "STM32CAN.h"
#include "SPI.h"
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/mt6701/MagneticSensorMT6701SSI.h"
// clang-format on

#define SENSOR1_CS PA4
MagneticSensorMT6701SSI sensor1(SENSOR1_CS);

HardwareSerial Serial1(PA3, PA2);

BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA8, PA9, PA10, PC6);

int DRIVER_RESET = PB6;
int DRIVER_SLEEP = PB5;
int DRIVER_FAULT = PB4;
int LED_PIN = PC4;

void configureFOC();

float targetAngle = 0;

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

class RxFromCAN : public PingPongNotificationsFromCAN {
public:
  RxFromCAN() : MessageStatus(NONE), TargetAngle(0.0f){};

  void SetMotorPosition(const uint8_t *data) {
    TargetAngle = unpackAngleFromCanMessage(data);
    Serial1.print("Target angle: ");
    Serial1.print(TargetAngle);
    MessageStatus = SET_POSITION;
  };

  void ReturnMotorPosition() { MessageStatus = REQUEST_POSITION; };

  void EmergencyStop() {
    Serial1.println("Emergency stop");
    digitalWrite(DRIVER_SLEEP, LOW);
    digitalWrite(LED_PIN, HIGH);
    while (true) {
      delay(1000);
    }
  };

  MESSAGE_STATUS MessageStatus;
  float TargetAngle;
};

void configureFOC() {
  pinMode(DRIVER_RESET, OUTPUT);
  digitalWrite(DRIVER_RESET, HIGH);
  pinMode(DRIVER_SLEEP, OUTPUT);
  digitalWrite(DRIVER_SLEEP, HIGH);
  pinMode(DRIVER_FAULT, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  sensor1.init();
  motor.linkSensor(&sensor1);

  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::angle;
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0.001;
  motor.voltage_limit = 3;
  motor.LPF_velocity.Tf = 0.01f;
  motor.P_angle.P = 20;
  motor.velocity_limit = 10;

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  Serial1.println(F("Motor ready."));
  _delay(1000);
}

RxFromCAN CANBroker;

CANPingPong CANDevice(CreateCanLib(PB9, PB8), &CANBroker);

void setup() {
  HAL_Init();
  delay(100);

  Serial1.begin(115200);

  delay(1000);

  configureFOC();
  // while (!Serial);

  CANDevice.Init();

  CANDevice.Can1->EnableBlinkOnActivity(LED_PIN);
}

void loop() {
  motor.loopFOC();
  motor.move(targetAngle);

  CAN_msg_t CAN_TX_msg;

  switch (CANBroker.MessageStatus) {
  case SET_POSITION:
    targetAngle = CANBroker.TargetAngle;
    // Reset the received ID to NONE.
    CANBroker.MessageStatus = NONE;
    break;
  case REQUEST_POSITION:
    // Send the motor position back to the sender.
    CAN_TX_msg.id = ANGL_REQUEST_CMD;
    packAngleIntoCanMessage(&CAN_TX_msg, motor.shaft_angle);
    CANDevice.CANSendByte(CAN_TX_msg.data, CAN_TX_msg.id);
    // Reset the received ID to NONE.
    CANBroker.MessageStatus = NONE;
    break;
  }

  // Update message queues.
  CANDevice.Can1->Loop();
}
