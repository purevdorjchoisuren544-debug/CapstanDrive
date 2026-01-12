#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

/* ---------------- ODrive CAN definitions ---------------- */
#define ODRIVE_NODE_ID_ONE 0
#define ODRIVE_NODE_ID_TWO 1

#define CMD_SET_AXIS_STATE   0x07
#define CMD_SET_INPUT_VEL    0x0D
#define CMD_SET_INPUT_POS    0x0C
#define CMD_HEARTBEAT        0x01
#define CMD_ENCODER_EST     0x03

#define AXIS_STATE_CLOSED_LOOP_CONTROL 8

#define DEG_TO_RAD (3.14159265358979323846f / 180.0f)

#define GEAR_RATIO 0.163f 

enum Motor {
    ONE,
    TWO
};

/* -------------------------------------------------------- */

void sendCAN(Motor motor_id, uint16_t id, const void *data, uint8_t len) {
  CAN_message_t msg;
  msg.id  = id;
  msg.len = len;
  memcpy(msg.buf, data, len);

  switch(motor_id){
    case Motor::ONE:
      Can1.write(msg);
      Serial.println("Can 1 updated");
      break;
    case Motor::TWO:
      Can3.write(msg);
      Serial.println("Can 3 updated");
      break;
  }
}

void setAxisState(Motor motor_id, uint32_t state) {

  uint8_t node = (motor_id == Motor::ONE) ? ODRIVE_NODE_ID_ONE : ODRIVE_NODE_ID_TWO;
  uint16_t id = (node << 5) | CMD_SET_AXIS_STATE;
  sendCAN(motor_id, id, &state, 4);
}

void setVelocity(Motor motor_id, float vel, float torque_ff = 0.0f) {

  uint8_t node = (motor_id == Motor::ONE) ? ODRIVE_NODE_ID_ONE : ODRIVE_NODE_ID_TWO;
  uint16_t id = (node << 5) | CMD_SET_INPUT_VEL;
  uint8_t data[8];
  memcpy(data + 0, &vel, 4);
  memcpy(data + 4, &torque_ff, 4);
  sendCAN(motor_id, id, data, 8);
}

void setPositionRad(Motor motor_id, float pos_rad, float vel_ff = 0.0f) {
  
  uint8_t node = (motor_id == Motor::ONE) ? ODRIVE_NODE_ID_ONE : ODRIVE_NODE_ID_TWO;
  uint16_t id = (node << 5) | CMD_SET_INPUT_POS;
  
  float adjusted_pos_rad = pos_rad * GEAR_RATIO;

  uint8_t data[8];
  memcpy(data + 0, &adjusted_pos_rad, 4);
  memcpy(data + 4, &vel_ff, 4);

  sendCAN(motor_id, id, data, 8);
}

void setPosition(Motor motor_id, float pos_deg, float vel_ff = 0.0f) {
  float pos_rad = pos_deg * DEG_TO_RAD;
  setPositionRad(motor_id, pos_rad, vel_ff);
}

/* ---------------- Setup ---------------- */

void setup() {
  Serial.begin(115200);
  delay(1000);

  Can1.begin();
  Can1.setBaudRate(1000000);
  Can1.enableFIFO();
  Can1.onReceive(canSniff);

  Can3.begin();
  Can3.setBaudRate(1000000);
  Can3.enableFIFO();
  Can3.onReceive(canSniff);

  Serial.println("CAN1 started");
  Serial.println("CAN3 started");

  delay(1000);

  /* Enter closed loop */
  setAxisState(Motor::ONE, AXIS_STATE_CLOSED_LOOP_CONTROL);
  setAxisState(Motor::TWO, AXIS_STATE_CLOSED_LOOP_CONTROL);
  Serial.println("Requested CLOSED_LOOP_CONTROL");
}

/* ---------------- Loop ---------------- */

void loop() {
  static uint32_t t0 = millis();
  static int phase = 0;

  uint32_t now = millis();

  switch (phase) {
    case 0: // 0
      setPosition(Motor::ONE, 0.0f);
      setPosition(Motor::TWO, 0.0f);
      if (now - t0 > 3000) { t0 = now; phase = 1; }
      Serial.println("0");
      break;

    case 1: // 90
      setPosition(Motor::ONE, 90.0f);
      setPosition(Motor::TWO, 90.0f);
      if (now - t0 > 2000) { t0 = now; phase = 2; }
      Serial.println("90");
      break;

    case 2: // 180
      setPosition(Motor::ONE, 180.0f);
      setPosition(Motor::TWO, 180.0f);
      if (now - t0 > 3000) { t0 = now; phase = 3; }
      Serial.println("180");
      break;

    case 3: // 270
      setPosition(Motor::ONE, 270.0f);
      setPosition(Motor::TWO, 270.0f);
      if (now - t0 > 2000) { t0 = now; phase = 4; }
      Serial.println("270");
      break;

    case 4: // 360
      setPosition(Motor::ONE, 360.0f);
      setPosition(Motor::TWO, 360.0f);
      if (now - t0 > 2000) { t0 = now; phase = 0; }
      Serial.println("360");
      break;
  }

  delay(2000); // ~100 Hz command rate
}

/* ---------------- CAN RX ---------------- */

void canSniff(const CAN_message_t &msg) {
  uint8_t node_id = msg.id >> 5;
  uint8_t cmd_id  = msg.id & 0x1F;

  if (node_id != ODRIVE_NODE_ID_ONE || node_id != ODRIVE_NODE_ID_TWO) return; //TODO: fix

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
