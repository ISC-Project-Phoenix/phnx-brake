#include <Arduino.h>
#include <FlexCAN_T4.h>

#define ACTUATOR_CMD_ID 0xFF0000
#define MAX_ACTUATOR_DIST 3000

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> h_priority;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> actuator;
bool training_mode;
uint16_t last_dist;
IntervalTimer keep_alive;


void send_brake_cmd(CAN_message_t &msg){
  CAN_message_t act_msg;
  act_msg.id = ACTUATOR_CMD_ID;
  act_msg.flags.extended = 1;
  //Convert percentage value to distance to move to
  //Max distance is 3.125", for saftey limit to 3.0"
  //Equates to 3000 steps of 0.001, 500 unit offset is applied
  if((msg.buf[0] > 100 && msg.buf[0] != 0xFF) || msg.buf[0] < 0){
    Serial.print("ERROR! Vaue out of range :(");
    return;
  }
  if(msg.buf[0] == 0xFF){
    //Send auto zero message
    last_dist = 0;
    act_msg.buf[0] = 0x7E;
    act_msg.buf[1] = 0x02;
    act_msg.buf[2] = 0x12;
    act_msg.buf[3] = 0x34;
    act_msg.buf[4] = 0x56;
    act_msg.buf[5] = 0xAB;
    act_msg.buf[6] = 0xCD;
    act_msg.buf[7] = 0xEF;
    actuator.write(act_msg);
    return;
  }
  float percentage = float(msg.buf[0]) / 100.0;
  last_dist = (percentage * MAX_ACTUATOR_DIST) + 500;
  //First two bytes are standard for control messages
  act_msg.buf[0] = 0xF;
  act_msg.buf[1] = 0x4A;
  //First break up 16 bit value of distance into least and most significant bytes
  act_msg.buf[2] = static_cast<uint8_t>(last_dist & 0x00FF);
  act_msg.buf[3] = static_cast<uint8_t>((last_dist & 0xFF00) >> 8);
  //Set last two bits of byte 3 to turn on clutch and motor
  act_msg.buf[3] |= 1UL << 7;
  act_msg.buf[3] |= 1UL << 6;
  actuator.write(act_msg);
}

void actu_keep_alive(){
  //Callback to resend the last recieved brake message so the actuator doesnt go to sleep
  CAN_message_t act_msg;
  act_msg.id = ACTUATOR_CMD_ID;
  act_msg.flags.extended = 1;
  //First two bytes are standard for control messages
  act_msg.buf[0] = 0xF;
  act_msg.buf[1] = 0x4A;
  //First break up 16 bit value of distance into least and most significant bytes
  act_msg.buf[2] = static_cast<uint8_t>(last_dist & 0x00FF);
  act_msg.buf[3] = static_cast<uint8_t>((last_dist & 0xFF00) >> 8);
  //Set last two bits of byte 3 to turn on clutch and motor
  act_msg.buf[3] |= 1UL << 7;
  act_msg.buf[3] |= 1UL << 6;
  actuator.write(act_msg);
  Serial.print("Keeping the actuator living");
}


void setup() {
    Serial.begin(115200);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);

    //Start with training mode disabled
    training_mode = false;

    //Set up actuator CAN bus
    actuator.begin();
    actuator.setBaudRate(250000);

    //Set up high priority CAN bus
    h_priority.begin();
    h_priority.setBaudRate(500000);

    /*
     * Setup mailboxes for high priority bus
     */
    //Mailbox dedicated to getting control messages from the bus
    h_priority.setMB(MB0, RX, EXT);

    //Mailbox dedicated to sending control messages to the bus for training mode
    h_priority.setMB(MB1, TX, EXT);

    //Mailbox dedicated to sending kill auton messages when pedal input detected
    h_priority.setMB(MB2, TX, EXT);
    h_priority.setMBFilter(REJECT_ALL);
    h_priority.enableMBInterrupts();
    h_priority.setMBFilter(MB0, 0x1);
    h_priority.onReceive(MB0, send_brake_cmd);

    /*
     * Setup mailboxes for actuator bus
     */
    //Mailbox dedicated to transmitting messages to the physical brake actuator
    actuator.setMB(MB0, TX, EXT);
    actuator.enableMBInterrupts();
    keep_alive.begin(actu_keep_alive, 100000);
}

void loop() {
  h_priority.events();
  actuator.events();
}