#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Can1.begin();
  Can1.setBaudRate(1000000);
  Can1.setMaxMB(16);
  Can1.enableFIFO();
  Can1.enableFIFOInterrupt();
  Can1.onReceive(canSniff);

  Serial.println("CAN1 started with SN65HVD230");
}

void loop() {
}

void canSniff(const CAN_message_t &msg) {
  Serial.print("ID:0x");
  Serial.print(msg.id, HEX);
  Serial.print(" DLC:");
  Serial.print(msg.len);
  Serial.print(" Data:");
  for (int i = 0; i < msg.len; i++) {
    Serial.print(" ");
    Serial.print(msg.buf[i], HEX);
  }
  Serial.println();
}
