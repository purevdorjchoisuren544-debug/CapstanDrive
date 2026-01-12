#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

/* ---------------- ODrive CAN definitions ---------------- */
#define ODRIVE_NODE_ID 0

#define CMD_SET_AXIS_STATE   0x07
#define CMD_SET_INPUT_VEL    0x0D
#define CMD_HEARTBEAT        0x01
#define CMD_ENCODER_EST     0x03

#define AXIS_STATE_CLOSED_LOOP_CONTROL 8

/* -------------------------------------------------------- */

void sendCAN(uint16_t id, const void *data, uint8_t len) {
  CAN_message_t msg;
  msg.id  = id;
  msg.len = len;
  memcpy(msg.buf, data, len);
  Can1.write(msg);
}

void setAxisState(uint32_t state) {
  uint16_t id = (ODRIVE_NODE_ID << 5) | CMD_SET_AXIS_STATE;
  sendCAN(id, &state, 4);
}

void setVelocity(float vel, float torque_ff = 0.0f) {
  uint16_t id = (ODRIVE_NODE_ID << 5) | CMD_SET_INPUT_VEL;
  uint8_t data[8];
  memcpy(data + 0, &vel, 4);
  memcpy(data + 4, &torque_ff, 4);
  sendCAN(id, data, 8);
}

/* ---------------- Setup ---------------- */

void setup() {
  Serial.begin(115200);
  delay(1000);

  Can1.begin();
  Can1.setBaudRate(1000000);
  Can1.enableFIFO();
  Can1.onReceive(canSniff);

  Serial.println("CAN1 started");

  delay(1000);

  /* Enter closed loop */
  setAxisState(AXIS_STATE_CLOSED_LOOP_CONTROL);
  Serial.println("Requested CLOSED_LOOP_CONTROL");
}

/* ---------------- Loop ---------------- */

void loop() {
  static uint32_t t0 = millis();
  static int phase = 0;

  uint32_t now = millis();

  switch (phase) {
    case 0: // forward
      setVelocity(2.0f);
      if (now - t0 > 3000) { t0 = now; phase = 1; }
      break;

    case 1: // stop
      setVelocity(0.0f);
      if (now - t0 > 2000) { t0 = now; phase = 2; }
      break;

    case 2: // reverse
      setVelocity(-2.0f);
      if (now - t0 > 3000) { t0 = now; phase = 3; }
      break;

    case 3: // stop
      setVelocity(0.0f);
      if (now - t0 > 2000) { t0 = now; phase = 0; }
      break;
  }

  delay(10); // ~100 Hz command rate
}

/* ---------------- CAN RX ---------------- */

void canSniff(const CAN_message_t &msg) {
  uint8_t node_id = msg.id >> 5;
  uint8_t cmd_id  = msg.id & 0x1F;

  if (node_id != ODRIVE_NODE_ID) return;

  if (cmd_id == CMD_ENCODER_EST) {
    float pos, vel;
    memcpy(&pos, msg.buf + 0, 4);
    memcpy(&vel, msg.buf + 4, 4);

    Serial.print("ENC | Pos=");
    Serial.print(pos, 3);
    Serial.print(" rad  Vel=");
    Serial.print(vel, 3);
    Serial.println(" rad/s");
  }
}
