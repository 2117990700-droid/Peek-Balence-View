# Peek-Balence-View
Designed for children aged 2-6,parents and rehabilitationteachers,this tool assists in children's vestibular training andprediction of movement risks.It synchronises daily trends anoweekly reports across mobile and web platforms,enablingparents and teachers to adjust training accordingly and shareprogress updates.
# Vestibular Balance Kit

Dual-IMU wearable + 6-channel plantar pressure board  
(Arduino Nano 33 IoT + dual MPU6050 + Arduino UNO)

## Overview

This repo contains two sketches for balance sensing:

1. **Wearable unit** (Nano 33 IoT + two MPU6050)  
   - Measures chest and waist acceleration  
   - Computes postural sway and chest–waist imbalance  
   - Estimates step frequency and fall risk (`LOW / MEDIUM / HIGH / FALL`)  
   - Sends JSON over WebSocket to a Node.js relay or browser UI

2. **Plantar pressure board** (UNO + 6 analog sensors)  
   - Reads 6 foot force channels (A0–A5)  
   - Supports zero + load calibration per channel  
   - Runs a 5 s sway test when a user stands on the board  
   - Outputs LR/AP sway RMS and Green/Amber/Red via Serial

## Files

- `nano33_balance_real.ino`  
- `footsense.ino`

## Quick start

1. Wire hardware as in code comments (I2C 0x68/0x69; sensors A0–A5).  
2. Edit Wi-Fi SSID, password and `SERVER_HOST` in Nano code.  
3. Upload sketches; open Serial at 115200 baud.  
4. On footboard: run `zero all`, `load <ch> <N>`, optional `base <sway0>`.  
5. Run a Node.js `ws` server to relay JSON to UI.

> **Disclaimer:** research prototype, not a medical device.  
> **License:** choose one (e.g. MIT).
