# Phoenix Brake ECU Board

This ECU board is responsible for taking in Set Brake messages from the high priority CAN bus and transmitting them to the brake
actuator via a dedicated CAN bus. This ECU will also handle transmitting training data back to the interface ECU

## Relevant Pins:
- High Priority Bus CAN RX Pin: 23
- High Priority Bus CAN TX Pin: 22
- Actuator Bus CAN RX Pin: 0
- Actuator Bus CAN TX Pin: 1
- Pedal Reference Pin: 20
