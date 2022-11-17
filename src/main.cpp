#include <Arduino.h>
#include <FlexCAN_T4.h>

#define ACTUATOR_CMD_ID 0xFF0000

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> h_priority;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> actuator;
bool training_mode;


void send_brake_commands(CAN_message_t &msg){
  CAN_message_t act_msg;
  act_msg.id = ACTUATOR_CMD_ID;
  act_msg.flags.extended = 1;
  //Convert percentage value to distance to move to

}

void actu_keep_alive(){
  //Resend the same command every 100ms so that the brake doesnt go to sleep
  CAN_message_t act_msg;
  act_msg.id = ACTUATOR_CMD_ID;
  act_msg.flags.extended = 1;
  
}


void setup() {
  // put your setup code here, to run once:
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
    h_priority.onReceive(MB0, send_brake_commands);

    /*
     * Setup mailboxes for actuator bus
     */
    //Mailbox dedicated to transmitting messages to the physical brake actuator
    actuator.setMB(MB0, TX, EXT);
    actuator.enableMBInterrupts();
}

void loop() {
  h_priority.events();
  actuator.events();
}