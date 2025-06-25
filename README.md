# Autonomous-Wall-Follower-Robot-Car
TM4C-based autonomous wall follower robot car that is built to navigate through a track with walls on both sides.

## Overview
This project involves designing and programming an autonomous robot car to navigate a prebuilt track with walls on both sides using a TM4C123 microcontroller. The robot employs three IR sensors: two angled at 45° to detect left and right wall distances, maintaining equal spacing by adjusting power to each wheel via hardware PWM, and a third forward-facing sensor to prevent head-on collisions. Key embedded system components include GPIO, hardware timers, interrupts, ADC for sensor input, and PWM for motor control, with the SysTick timer managing IR sensor sampling rates. LEDs indicate the robot’s status: yellow for startup, purple at track’s end, red when too close to walls, green or blue for proximity to left or right walls, white for speed setting mode, and no light when centered. The project is divided into two phases:  
1. Straight-Line Wall Follower: The robot navigates a 2m-long, 70cm-wide straight track with 10cm-high walls, tested at starting positions 15cm from the left wall, center, and 15cm from the right wall, stopping at the track’s end.  
2. Power Wall Follower: The robot traverses a track with a 2m straight section, a 90° right turn (1m), and a 90° left turn (0.5m), tested at starting positions 20cm from the left wall, center, and 20cm from the right wall, stopping at the end.

## Hardware design
![Hardware Setup](https://github.com/user-attachments/assets/fffdbaa0-43ca-497a-bc38-52e315e71565)

## Schematic
![Schematic](https://github.com/user-attachments/assets/1ea1ff8c-1daa-464a-8eae-2fee81a82e1c)

## Demo Video
Straight Wall Follower: https://youtube.com/shorts/F17Lw64sHN0?feature=share  
Power Wall Follower:  https://youtube.com/shorts/vJIUpQGYRWo?feature=share

## Challenges
PROBLEM 1: The sensors on the car didn’t work although we programmed the sensors to work.  
SOLUTION: Realized that our sensors were placed in the wrong pins.
