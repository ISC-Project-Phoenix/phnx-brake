#include "brake.hpp"

Brake::Brake(uint32_t act_cmd_id, uint16_t max_dist) {
    this->actuator_cmd_id = act_cmd_id;
    this->max_actuator_dist = max_dist;
    this->last_dist = 0;
}

void Brake::generate_brk_msg(CAN_message_t &in_msg, CAN_message_t &out_msg) {
    ///Generate output message for actuator CAN bus based on input CAN message
    out_msg.id = this->actuator_cmd_id;
    out_msg.flags.extended = 1;
    //Convert percentage value to distance to move to
    //Max distance is 3.125", for saftey limit to 3.0"
    //Equates to 3000 steps of 0.001, 500 unit offset is applied
    if ((in_msg.buf[0] > 100 && in_msg.buf[0] != 0xFF)) {
        Serial.print("ERROR! Vaue out of range :(");
        return;
    }
    if (in_msg.buf[0] == 0xFF) {
        //Send auto zero message
        last_dist = 0;
        out_msg.buf[0] = 0x7E;
        out_msg.buf[1] = 0x02;
        out_msg.buf[2] = 0x12;
        out_msg.buf[3] = 0x34;
        out_msg.buf[4] = 0x56;
        out_msg.buf[5] = 0xAB;
        out_msg.buf[6] = 0xCD;
        out_msg.buf[7] = 0xEF;
        //actuator.write(act_msg);
        return;
    }
    float percentage = float(in_msg.buf[0]) / 100.0;
    last_dist = (percentage * this->max_actuator_dist) + 500;

    //Fill the buffer with correct bytes for command messages

    //First two bytes are standard for control messages
    out_msg.buf[0] = 0xF;
    out_msg.buf[1] = 0x4A;
    //First break up 16 bit value of distance into least and most significant bytes
    out_msg.buf[2] = static_cast<uint8_t>(this->last_dist & 0x00FF);
    out_msg.buf[3] = static_cast<uint8_t>((this->last_dist & 0xFF00) >> 8);
    //Set last two bits of byte 3 to turn on clutch and motor
    out_msg.buf[3] |= 1UL << 7;
    out_msg.buf[3] |= 1UL << 6;
}


void Brake::generate_brk_msg(uint8_t percent, CAN_message_t &out_msg) {
    ///Generate output message for actuator CAN bus based on input pedal percentage
    out_msg.id = this->actuator_cmd_id;
    out_msg.flags.extended = 1;
    float percentage = float(percent) / 100.0;
    this->last_dist = (percentage * this->max_actuator_dist) + 500;
    //Fill the buffer with correct bytes for command messages

    //First two bytes are standard for control messages
    out_msg.buf[0] = 0xF;
    out_msg.buf[1] = 0x4A;
    //First break up 16 bit value of distance into least and most significant bytes
    out_msg.buf[2] = static_cast<uint8_t>(this->last_dist & 0x00FF);
    out_msg.buf[3] = static_cast<uint8_t>((this->last_dist & 0xFF00) >> 8);
    //Set last two bits of byte 3 to turn on clutch and motor
    out_msg.buf[3] |= 1UL << 7;
    out_msg.buf[3] |= 1UL << 6;
}

void Brake::generate_brk_msg(CAN_message_t &out_msg) {
    ///Generate output message for actuator CAN bus based last recieved distance
    out_msg.id = this->actuator_cmd_id;
    out_msg.flags.extended = 1;
    //Fill the buffer with correct bytes for command messages

    //First two bytes are standard for control messages
    out_msg.buf[0] = 0xF;
    out_msg.buf[1] = 0x4A;
    //First break up 16 bit value of distance into least and most significant bytes
    out_msg.buf[2] = static_cast<uint8_t>(this->last_dist & 0x00FF);
    out_msg.buf[3] = static_cast<uint8_t>((this->last_dist & 0xFF00) >> 8);
    //Set last two bits of byte 3 to turn on clutch and motor
    out_msg.buf[3] |= 1UL << 7;
    out_msg.buf[3] |= 1UL << 6;
}