// clang-format off
#include <Arduino.h>
#include <math.h>
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

// CAN FD Configuration
FDCAN_HandleTypeDef hfdcan1;
FDCAN_TxHeaderTypeDef txHeader;
FDCAN_RxHeaderTypeDef rxHeader;
uint8_t txData[64]; // CAN FD can support up to 64 bytes
uint8_t rxData[64];

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

/**
 * @brief FDCAN1 Initialization Function
 * @param None
 * @retval None
 */
void MX_FDCAN1_Init(void) {
  hfdcan1.Instance = FDCAN1;

  // Basic configuration
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS; // CAN FD with Bit Rate Switch
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;

  // Nominal Bit Timing (Classic CAN part): 500 kbit/s
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;

  // Data Bit Timing (CAN FD part): 2 Mbit/s
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 13;
  hfdcan1.Init.DataTimeSeg2 = 2;

  // Message RAM configuration
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;

  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
    Serial1.println("FDCAN initialization failed!");
    Error_Handler();
  }

  // Configure global filter to accept all messages
  if (HAL_FDCAN_ConfigGlobalFilter(
          &hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
          FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
    Serial1.println("FDCAN global filter configuration failed!");
    Error_Handler();
  }

  // Start FDCAN
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    Serial1.println("FDCAN start failed!");
    Error_Handler();
  }

  Serial1.println("FDCAN initialized successfully!");
}

/**
 * @brief GPIO Initialization Function for FDCAN1
 * @param None
 * @retval None
 */
void MX_FDCAN1_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Enable GPIOB clock
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Configure GPIO pins for FDCAN1
  // PB8: FDCAN1_RX
  // PB9: FDCAN1_TX
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void EmergencyStop() {
  Serial1.println("Emergency stop");
  digitalWrite(DRIVER_SLEEP, LOW);
  digitalWrite(LED_PIN, HIGH);
  while (true) {
    delay(1000);
  }
};

/**
 * @brief Sends a CAN FD message with specified ID and data payload
 *
 * @param id The CAN message identifier (11-bit or 29-bit depending on
 * configuration)
 * @param data Pointer to the data buffer containing the message payload
 * @param length Number of bytes in the data payload (0-64 bytes for CAN FD)
 *
 * @return HAL_StatusTypeDef Returns HAL_OK on success, or appropriate error
 * code on failure
 */
HAL_StatusTypeDef CANFD_SendMessage(uint32_t id, uint8_t *data,
                                    uint8_t length) {
  // Configure transmission header
  txHeader.Identifier = id;
  txHeader.IdType = FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = GetFDCANDataLengthCode(length);
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_ON; // Enable bit rate switching for CAN FD
  txHeader.FDFormat = FDCAN_FD_CAN;      // CAN FD format
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker = 0;

  // Send message
  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, data);
}

/**
 * @brief Check for received CAN FD messages
 * @param None
 * @retval None
 */
void CANFD_CheckReceived(void) {
  if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, rxData) ==
      HAL_OK) {

    uint8_t dataLength = GetBytesFromFDCANDataLength(rxHeader.DataLength);

    Serial1.print("ID: 0x");
    Serial1.print(rxHeader.Identifier, HEX);
    Serial1.print(" (");
    Serial1.print(dataLength);
    Serial1.print(" bytes)");

    // Check if this is CAN FD format
    if (rxHeader.FDFormat == FDCAN_FD_CAN) {
      Serial1.print(" [CAN FD]");
      if (rxHeader.BitRateSwitch == FDCAN_BRS_ON) {
        Serial1.print(" [BRS]");
      }
    } else {
      Serial1.print(" [Classic CAN]");
    }
    Serial1.println();

    if (dataLength <= 8) {
      switch (rxHeader.Identifier) {
      case ANGL_CNTL_CMD:
        targetAngle = unpackAngleFromCanMessage(rxData);
        Serial1.print("Target angle set to: ");
        Serial1.println(targetAngle);
        break;
      case ANGL_REQUEST_CMD:
        Serial1.print("Current motor angle: ");
        Serial1.println(motor.shaft_angle);
        packAngleIntoCanMessage(txData, motor.shaft_angle);
        CANFD_SendMessage(ANGL_REQUEST_CMD, txData, 4);
        break;
      case ESTOP:
        Serial1.println("ESTOP");
        EmergencyStop();
        break;
      default:
        Serial1.printf("y:0x%x DLC=0x%x ", rxHeader.Identifier,
                       rxHeader.DataLength);
        Serial1.println();
      }
    }

    // Toggle LED when message received
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}

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

void setup() {
  HAL_Init();
  delay(100);

  Serial1.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  delay(1000);

  // Initialize FDCAN GPIO
  MX_FDCAN1_GPIO_Init();

  // Enable FDCAN1 clock
  __HAL_RCC_FDCAN_CLK_ENABLE();

  // Initialize FDCAN1
  MX_FDCAN1_Init();

  configureFOC();
  // while (!Serial);
}

void loop() {
  motor.loopFOC();
  motor.move(targetAngle);

  // Check for received messages
  CANFD_CheckReceived();
}
