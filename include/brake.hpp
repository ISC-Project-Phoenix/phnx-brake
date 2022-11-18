#include <Arduino.h>
#include <FlexCAN_T4.h>
class Brake{
    private:
        uint16_t last_dist;
        uint32_t actuator_cmd_id;
        uint16_t max_actuator_dist;
    public:
        Brake(uint32_t act_cmd_id, uint16_t max_dist);
        void generate_brk_msg(CAN_message_t &in_msg, CAN_message_t &out_msg);
        void generate_brk_msg(uint8_t percent, CAN_message_t &out_msg);
        void generate_brk_msg(CAN_message_t &out_msg);

};