# Switched Mode Power Converter Simulator / Emulator

## Description

This project implements a real-time switched mode power converter simulator/emulator on an STM32 microcontroller.  
The system consists of a discrete-time converter model controlled by a PI controller and operated via buttons, LEDs, and UART communication.

The application follows a polling-with-interrupts structure and separates application logic from hardware abstraction layers. Shared resources between UART and button inputs are protected using semaphore mechanisms.

## Implemented Features

- Real-time program structure (polling with interrupts)
- Discrete PI controller
  - Adjustable **Kp** and **Ki** parameters
  - Controller output drives PWM duty cycle
- Discrete-time converter model based on the given state-space equations
- Three operating modes:
  - **IDLE**: Converter off, output forced to zero
  - **MOD**: Closed-loop modulation using PI controller
  - **CONFIG**: Parameter configuration mode
- User interface:
  - Buttons for mode switching and parameter adjustment
  - UART console for parameter updates and status messages
  - LED indicators for current operating mode
- Protection of shared data using semaphore logic between UART and button inputs

