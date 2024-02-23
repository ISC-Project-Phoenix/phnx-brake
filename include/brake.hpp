#include <Arduino.h>
#include <FlexCAN_T4.h>

/// Stores brake actuator state
class Brake {
private:
    uint16_t last_dist;
    uint32_t actuator_cmd_id;
    uint16_t max_actuator_dist;

    void set_common_flags(CAN_message_t &msg) const;

public:
    /// \param act_cmd_id CAN message id for actuator cmd messages.
    /// \param max_dist Max distance the actuator can go (TODO units?)
    Brake(uint32_t act_cmd_id, uint16_t max_dist);

    /// Generate output message for actuator CAN bus based on Set Brake message.
    void generate_brk_msg(CAN_message_t &in_msg, CAN_message_t &out_msg);

    /// Generate output message for actuator CAN bus based on input pedal percentage.
    void generate_brk_msg(uint8_t percent, CAN_message_t &out_msg, bool true_zero = false);

    /// Generate output message for actuator CAN bus matching last received distance.
    void generate_brk_msg(CAN_message_t &out_msg) const;

};