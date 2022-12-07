#include "brake.hpp"

enum CanMappings {
    KillAuton = 0x0,
    SetBrake = 0x1,
    LockBrake = 0x2,
    UnlockBrake = 0x3,
    TrainingMode = 0x8,
};

constexpr int PEDAL_INPUT_PIN = 20;
constexpr int PEDAL_POLL_RATE = 100; //In microseconds
constexpr int KEEP_ALIVE_RATE = 100000; // In microseconds

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> h_priority;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> actuator;

/// Stores actuator state. Must be synchronised
Brake brake_ecu{0xFF0000, 2000, 0};

bool auton_disabled;
bool training_mode;
bool brake_lock;

/// Timer to send heartbeat messages to actuator
IntervalTimer keep_alive;
/// Timer to poll pedal ADC
IntervalTimer poll_pedal;

/// Called on can message received in the main loop context.
///
/// \param msg Either a Set Brake or Training Mode message
void send_can_cmd(CAN_message_t &msg) {
    if (!training_mode) {
        if (msg.id == CanMappings::TrainingMode) {
            training_mode = true;
            digitalWrite(LED_BUILTIN, HIGH);
            return;
        } else if (msg.id == CanMappings::SetBrake && !brake_lock) {
            auton_disabled = false;
            CAN_message_t act_msg;

            noInterrupts();
            brake_ecu.generate_brk_msg(msg, act_msg);
            interrupts();

            actuator.write(act_msg);
        } else if (msg.id == CanMappings::LockBrake) {
            brake_lock = true;
        } else if (msg.id == CanMappings::UnlockBrake) {
            //Set actuator last dist to what we've defined our starting point to be
            brake_ecu.set_last_dist(brake_ecu.get_min_dist());
            brake_lock = false;
        } else {
            Serial.printf("Received invalid CAN id: %d from priority bus!", msg.id);
        }
    }
}

/// Callback to resend the last received brake message so the actuator doesnt go to sleep.
/// If this is not sent the actuator will still take in new messages but wont hold its last
/// position.
void actu_keep_alive() {
    if (brake_lock || brake_ecu.get_last_dist() != brake_ecu.get_min_dist()) {
        return;
    }
    CAN_message_t act_msg;

    noInterrupts();
    brake_ecu.generate_brk_msg(act_msg);
    interrupts();

    actuator.write(act_msg);
}

/// Polls the pedal ADC
void poll_pedal_value() {
    //Callback to get new reading from the pedal and if that reading is above 5% put it onto the actuator bus
    //Poll pedal for input, get raw value and convert to voltage
    // 0 - 1023 => 0.0v - 3.3v
    auto vol = float(analogRead(PEDAL_INPUT_PIN));
    vol *= (3.3 / 1023.0);
    float resistance = -(7500.0f * vol / (vol - 5.0f));

    // Throttle down percent
    auto percent = uint8_t((resistance / 5000.0f) * 100.0f);

    if (training_mode) {
        //Send CAN message with current pedal value, will send messages as fast as we poll the pedal
        CAN_message_t training_msg;

        training_msg.id = CanMappings::SetBrake;
        training_msg.flags.extended = true;
        training_msg.buf[0] = percent;

        h_priority.write(training_msg);
    }

    if (percent > 5) {
        CAN_message_t act_msg;

        // Disable auton if not, since the pedal is being pressed
        if (!auton_disabled) {
            auton_disabled = true;
            CAN_message_t kill_auton;
            kill_auton.id = CanMappings::KillAuton;
            kill_auton.flags.extended = true;
            h_priority.write(kill_auton);
        }

        noInterrupts();
        brake_ecu.generate_brk_msg(percent, act_msg);
        interrupts();

        actuator.write(act_msg);
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(PEDAL_INPUT_PIN, INPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);

    //Start with training mode disabled
    training_mode = false;
    auton_disabled = false;
    brake_lock = false;

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
    h_priority.setMBFilter(MB0, CanMappings::SetBrake, CanMappings::TrainingMode);
    h_priority.onReceive(MB0, reinterpret_cast<_MB_ptr>(send_can_cmd));

    /*
     * Setup mailboxes for actuator bus
     */
    //Mailbox dedicated to transmitting messages to the physical brake actuator
    actuator.setMB(MB0, TX, EXT);
    actuator.enableMBInterrupts();

    //Fire off one message to bring actuator to configured zero point
    CAN_message_t msg;
    noInterrupts();
    brake_ecu.generate_brk_msg(0, msg);
    interrupts();
    actuator.write(msg);

    //Interrupts to keep the actuator alive and poll the pedal for input
    keep_alive.priority(0);
    poll_pedal.priority(1);
    keep_alive.begin(actu_keep_alive, KEEP_ALIVE_RATE);
    poll_pedal.begin(poll_pedal_value, PEDAL_POLL_RATE);
}

void loop() {
    h_priority.events();
    actuator.events();
    // Puts the proc in low power until the next interrupt fires
    asm volatile("wfi":: : "memory");
}