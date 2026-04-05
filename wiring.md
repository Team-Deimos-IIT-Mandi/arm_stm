# STM32F407 Robot Arm Wiring Diagram

This document lists all the physical connections between the STM32F407G-DISC1 board and the robotic arm components.

## 1. Motor Control (PWM & Direction)

All motors use a PWM pin for speed control and a standard GPIO pin for direction.

| Joint / Motor | PWM Pin (Timer Channel) | DIR Pin |
| :--- | :--- | :--- |
| **Motor 1 (M1)** | PE9 (TIM1_CH1) | PE7 |
| **Motor 2 (M2)** | PE11 (TIM1_CH2) | PE8 |
| **Motor 3 (M3)** | PE13 (TIM1_CH3) | PE10 |
| **Pitch** | PE14 (TIM1_CH4) | PE12 |
| **Roll** | PA1 (TIM2_CH2) | PE15 |
| **Z-Axis** | PA2 (TIM2_CH3) | PD0 |

## 2. Sensor Inputs

### Potentiometers (M1, M2 Feedback)
Connected to ADC1 for absolute position feedback.

| Sensor | Pin | Function |
| :--- | :--- | :--- |
| **Pot 1 (M1)** | PB0 | ADC1_IN8 |
| **Pot 2 (M2)** | PB1 | ADC1_IN9 |

### I2C Encoders (M3, Pitch, Roll, Z Feedback)
All AS5600 encoders are connected via a TCA9548A I2C Multiplexer.

| Bus | Pin | Function |
| :--- | :--- | :--- |
| **I2C1 SCL** | PB6 | Clock to Multiplexer |
| **I2C1 SDA** | PB9 | Data to Multiplexer |

**MUX Channels (configured in RobotCore.cpp):**
- Channel 5: M3 Encoder
- Channel 0: Pitch Encoder
- Channel 1: Roll Encoder
- Channel 2: Z-Axis Encoder

## 3. Communication

### ROS Serial (UART)
Connect this to your ROS host (PC or Raspberry Pi).

| Signal | Pin | STM32 Mode |
| :--- | :--- | :--- |
| **TX** | PC10 | UART4_TX |
| **RX** | PC11 | UART4_RX |
| **GND** | GND | Common Ground |

*Baud Rate: 115200*

## 4. Status Indicators (Onboard LEDs)

| LED | Pin | Meaning |
| :--- | :--- | :--- |
| **LD4 (Green)** | PD12 | System Connected (Active ROS Comms) |
| **LD5 (Red)** | PD14 | System Disconnected / Watchdog Timeout |

## 5. Power Requirements
- **STM32 Board:** Powered via Mini-B USB.
- **Motors/Servos:** External power supply (usually 12V or 24V) with common ground connected to the STM32 board.
- **Sensors:** 3.3V power from the STM32 board.
