#include <Arduino.h>
#include <assert.h>

void packAngleIntoCanMessage(uint8_t *message, float angle);
float unpackAngleFromCanMessage(const uint8_t *data);
uint32_t GetFDCANDataLengthCode(uint8_t bytes);
uint8_t GetBytesFromFDCANDataLength(uint32_t dlc);