#include <Arduino.h>
#include <assert.h>

void packFloatIntoCanMessage(uint8_t *message, float value);
float unpackFloatFromCanMessage(const uint8_t *data);
uint32_t GetFDCANDataLengthCode(uint8_t bytes);
uint8_t GetBytesFromFDCANDataLength(uint32_t dlc);