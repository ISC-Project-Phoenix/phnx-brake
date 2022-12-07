#include <Arduino.h>
#include <FlexCAN_T4.h>

/// Stores brake actuator state
class Brake {
private:
    uint16_t last_dist;
    uint32_t actuator_cmd_id;
    uint16_t max_actuator_dist;
    uint16_t min_actuator_dist;

    void set_common_flags(CAN_message_t &msg) const;

public:
    /// \param act_cmd_id CAN message id for actuator cmd messages.
    /// \param max_dist Max distance the actuator can go (0.001 inch steps)
    /// \param min_dist Distance the actuator will return to when 0 is received either via pedal or CAN (0.001 inch steps)
    Brake(uint32_t act_cmd_id, uint16_t max_dist, uint16_t min_dist);

    /// Generate output message for actuator CAN bus based on Set Brake message.
    void generate_brk_msg(CAN_message_t &in_msg, CAN_message_t &out_msg);

    /// Generate output message for actuator CAN bus based on input pedal percentage.
    void generate_brk_msg(uint8_t percent, CAN_message_t &out_msg);

    /// Generate output message for actuator CAN bus matching last received distance.
    void generate_brk_msg(CAN_message_t &out_msg) const;

    ///Return the last measured distance from either the pedal or the CAN bus
    uint16_t get_last_dist() const;

    ///Return minimum distance actuator can move
    uint16_t get_min_dist() const;

};