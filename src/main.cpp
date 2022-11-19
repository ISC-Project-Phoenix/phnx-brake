#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "brake.hpp"

constexpr int PEDAL_INPUT = 20;
constexpr int PEDAL_POLL_RATE = 100; //In microseconds
constexpr int KEEP_ALIVE_RATE = 100000; // In microseconds
constexpr int TRAINING_MODE_ID = 8;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> h_priority;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> actuator;

//Actuator Command ID, Maximum actuator distance
Brake brake_ecu(0xFF0000, 1000);
bool auton_disabled;
bool training_mode;

IntervalTimer keep_alive;
IntervalTimer poll_pedal;

void send_can_cmd(CAN_message_t &msg) {
    if (!training_mode) {
        if (msg.id == TRAINING_MODE_ID) {
            training_mode = true;
            digitalWrite(LED_BUILTIN, HIGH);
            return;
        }
        auton_disabled = false;
        CAN_message_t act_msg;

        brake_ecu.generate_brk_msg(msg, act_msg);

        actuator.write(act_msg);
    }
}

void actu_keep_alive() {
    //Callback to resend the last received brake message so the actuator doesnt go to sleep
    //If this is not sent the actuator will still take in new messages but wont hold its last
    //position
    CAN_message_t act_msg;

    brake_ecu.generate_brk_msg(act_msg);

    actuator.write(act_msg);
}

void poll_pedal_value() {
    //Callback to get new reading from the pedal and if that reading is above 5% put it onto the actuator bus
    //Poll pedal for input, get raw value and convert to voltage
    // 0 - 1023 => 0.0v - 3.3v
    float vol = analogRead(PEDAL_INPUT);
    vol = vol * (3.3 / 1023.0);
    float resistance = -(7500.0 * vol / (vol - 5.0));
    uint8_t percent = ((resistance / 5000.0) * 100.0);

    if (percent > 5) {
        CAN_message_t act_msg;
        if (!auton_disabled) {
            auton_disabled = true;
            CAN_message_t kill_auton;
            kill_auton.id = 0x0;
            kill_auton.flags.extended = 1;
            h_priority.write(kill_auton);
        }

        brake_ecu.generate_brk_msg(percent, act_msg);

        actuator.write(act_msg);
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(PEDAL_INPUT, INPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);

    //Start with training mode disabled
    training_mode = false;
    auton_disabled = false;

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
    h_priority.setMB(MB2, TX, EXT);
    h_priority.setMBFilter(REJECT_ALL);
    h_priority.enableMBInterrupts();
    h_priority.setMBFilter(MB0, 0x1, 0x7);
    h_priority.onReceive(MB0, send_can_cmd);

    /*
     * Setup mailboxes for actuator bus
     */
    //Mailbox dedicated to transmitting messages to the physical brake actuator
    actuator.setMB(MB0, TX, EXT);
    actuator.enableMBInterrupts();

    //Interrupts to keep the actuator alive and poll the pedal for input
    keep_alive.begin(actu_keep_alive, KEEP_ALIVE_RATE);
    poll_pedal.begin(poll_pedal_value, PEDAL_POLL_RATE);
}

void loop() {
    h_priority.events();
    actuator.events();
    asm volatile("wfi":: : "memory");
}