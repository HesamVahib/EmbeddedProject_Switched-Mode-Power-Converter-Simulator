# Real-Time Switched-Mode Power Converter Simulator on STM32F4

![Hardware setup](https://media.licdn.com/dms/image/v2/D4D2DAQEJOBQn6fvBDg/profile-treasury-image-shrink_8192_8192/B4DZwqNfO3HEAg-/0/1770234705511?e=1781856000&v=beta&t=4K48RfgKBe13XYC-K_qWdCF3uNhdkCVkWg-EZrHYpkQ)

## Overview

This repository contains an embedded systems academic project implementing a **real-time switched-mode power converter simulator** on an **STM32F4-series development board**.

The project simulates a discrete-time power converter plant in real time and regulates its normalized output using a **PI controller**. The controller output is applied as a **PWM duty command**, while a **UART command-line interface** and hardware buttons allow monitoring, mode switching, and parameter tuning.

The firmware is interrupt-driven and designed for deterministic real-time execution at a fixed sampling rate of **1 kHz**.

This project was developed as part of the **Embedded System Programming Project** in the Master Programme in Electrical Transportation Systems at **Lappeenranta–Lahti University of Technology LUT**.

## Key Features

* Real-time discrete-time plant simulation on STM32F4
* Sixth-order state-space plant model
* PI controller with anti-windup
* PWM generation using STM32 timer peripherals
* UART command-line interface for monitoring and tuning
* Hardware button interface with software debouncing
* LED-based operating mode indication
* Three operating modes: IDLE, MOD, and CONFIG
* Safe startup with zero PWM duty
* Interface arbitration between UART and buttons
* Bonus 1: sinusoidal inverter-style reference generator
* Bonus 2: PWM-event-driven plant stepping

## Hardware Platform

The project is implemented on an **STM32F4 Nucleo-F411RE** development board using STM32 HAL.

### Main Peripherals

| Peripheral | Function                     |
| ---------- | ---------------------------- |
| USART2     | UART command-line interface  |
| TIM3       | 1 kHz control-loop interrupt |
| TIM2       | PWM generation on Channel 1  |
| GPIO       | LEDs and push buttons        |

### Pin Summary

| Function        | Pin        | Description                             |
| --------------- | ---------- | --------------------------------------- |
| UART TX         | PA2        | USART2 transmit                         |
| UART RX         | PA3        | USART2 receive                          |
| MOD LED         | PA6        | ON in MOD mode                          |
| CONFIG LED      | PA7        | ON in CONFIG mode                       |
| MODE button     | PC13 / PA0 | Mode switching                          |
| Increase button | PA1        | Increase reference or selected gain     |
| Decrease button | PB0        | Decrease reference or selected gain     |
| Select button   | PA4        | Toggle between Kp and Ki in CONFIG mode |

## Software Architecture

The firmware is designed as an interrupt-driven embedded application.

After initialization, the main loop remains in a wait-for-interrupt state. The real-time tasks are executed through timer and peripheral interrupts.

At each 1 ms control tick, the firmware performs:

1. Button sampling and debouncing
2. Operating mode evaluation
3. Plant output measurement and normalization
4. Measurement filtering
5. PI control calculation
6. PWM duty update
7. Plant model stepping

This structure ensures deterministic timing and low CPU load.

## Mathematical Model

The plant is represented by a sixth-order discrete-time state-space model:

```text
x[k+1] = A x[k] + B u[k]
y[k]   = x6[k]
```

The plant output is normalized to the range `[0, 1]` and filtered using a first-order low-pass filter before being used as feedback for the PI controller.

The PI controller regulates the filtered measurement toward the reference value and generates a PWM duty command limited to the range `[0, 1]`.

## Operating Modes

### IDLE Mode

Safe startup and inactive state.

* PWM duty is set to zero
* Control action is disabled
* Measurement is forced to zero
* LEDs are off

### MOD Mode

Closed-loop modulation mode.

* PI controller is active
* PWM duty is updated in real time
* Plant output tracks the selected reference
* PA6 LED is ON

### CONFIG Mode

Controller tuning mode.

* Kp and Ki can be adjusted using hardware buttons
* UART parameter modification is locked during configuration
* PA7 LED is ON
* A constant PWM duty is applied for tuning support

## UART Command-Line Interface

The UART CLI allows monitoring and parameter control through a serial terminal.

Default UART configuration:

```text
Baud rate: 115200
Data bits: 8
Parity: None
Stop bits: 1
```

### Supported Commands

| Command         | Description                                       |
| --------------- | ------------------------------------------------- |
| `help`          | Show available commands                           |
| `status`        | Print system status                               |
| `mode idle`     | Switch to IDLE mode                               |
| `mode mod`      | Switch to MOD mode                                |
| `mode cfg`      | Switch to CONFIG mode                             |
| `kp <value>`    | Set proportional gain                             |
| `ki <value>`    | Set integral gain                                 |
| `ref <value>`   | Set reference value                               |
| `bonus1 on/off` | Enable or disable sine reference                  |
| `freq <value>`  | Set sine reference frequency                      |
| `amp <value>`   | Set sine reference amplitude                      |
| `bonus2 on/off` | Enable or disable PWM-event-driven plant stepping |

## Bonus Features

### Bonus 1: Sinusoidal Reference Generator

The firmware can generate an inverter-style sinusoidal reference signal. The reference is mapped to the normalized range and used by the PI controller in MOD mode.

This demonstrates tracking of a time-varying reference.

### Bonus 2: PWM-Event-Driven Plant Stepping

Instead of stepping the plant using an averaged PWM duty value, the plant can be stepped using PWM timer events.

* TIM2 update event: applies positive or negative input
* TIM2 compare event: applies zero input during PWM off-time

This creates a more switching-like input behavior and better represents switched-mode converter operation.

## Verification

The project was verified using UART terminal outputs and LED indicators.

The following tests were performed:

* Boot and IDLE state verification
* MOD mode closed-loop tracking
* CONFIG mode gain tuning
* UART and button arbitration
* Sinusoidal reference generation
* PWM-event-driven plant stepping

The results confirmed correct mode transitions, stable closed-loop operation, reliable parameter tuning, and safe interaction between UART and button interfaces.


## How to Build and Run

1. Clone this repository:

```bash
git clone https://github.com/your-username/real-time-smps-simulator-stm32f4.git
```

2. Open the project in **STM32CubeIDE**.

3. Connect the STM32F4 Nucleo-F411RE board using USB.

4. Build and flash the firmware.

5. Open a serial terminal such as PuTTY, Tera Term, or the STM32CubeIDE serial monitor.

6. Configure the terminal:

```text
Baud rate: 115200
Data bits: 8
Parity: None
Stop bits: 1
```

7. Use the UART commands to monitor and control the system.

Example:

```text
help
status
mode mod
ref 0.5
kp 1.0
ki 30
```

## Project Status

This project is complete as an academic embedded systems project. It demonstrates real-time control, PWM generation, UART communication, user-interface handling, and discrete-time simulation on an STM32 microcontroller.

## Limitations

This project simulates the power converter plant in software. It does not implement a physical power converter stage. The PWM signal and plant dynamics are used for real-time embedded control demonstration and verification.

Future improvements could include:

* Connection to an external converter circuit
* Oscilloscope-based PWM verification
* More advanced plant models
* Data logging to a PC
* Real-time plotting of UART output
* Hardware-in-the-loop extension

## Authors

* Hesam Vahib
* David Rybàk
* Fatemeh Kholardi

## Academic Context

Course: Embedded System Programming Project
Programme: Master Programme in Electrical Transportation Systems
University: Lappeenranta–Lahti University of Technology LUT
Year: 2025

## License

This repository is shared for academic and educational purposes. If source code is included, an MIT License is recommended unless restricted by course or university policy.

## Citation

If you use this project, please cite the archived Zenodo version of the repository.

```bibtex
@misc{stm32f4_realtime_smps_simulator,
  title        = {Real-Time Switched-Mode Power Converter Simulator on STM32F4},
  author       = {Vahib, Hesam and Rybàk, David and Kholardi, Fatemeh},
  year         = {2025},
  note         = {Academic project, Embedded System Programming Project, LUT University},
  howpublished = {GitHub repository and Zenodo archive}
}
```
