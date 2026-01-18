#include <Arduino.h>
#include <assert.h>

typedef enum {
  ANG_CONTROL = 0,
  REQ_DATA,
  EMERGENCY_STOP,
  INVALID_COMMAND
} PARSE_RESULT;

void packAngleIntoCanMessage(uint8_t *message, float angle);
float unpackAngleFromCanMessage(const uint8_t *data);
uint32_t GetFDCANDataLengthCode(uint8_t bytes);
uint8_t GetBytesFromFDCANDataLength(uint32_t dlc);