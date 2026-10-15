# Quadcopter-Code-

DIY Quadcopter based on Arduino

The firmware of my DIY quadcopter. The Arduino gets data from MPU-6050 on how much the frame is tilted, controls the motors to keep it steady, and accepts text commands from my phone through HM-10 Bluetooth module.

It's still in development. The pitch and roll are balanced, there is no yaw stabilization and no failsafe yet, so please, read carefully about safety precautions before attaching any propellers.

Parts list

For this project you will need Arduino Uno or Nano, 4 brushless motors with ESCs, MPU-6050 (GY-521 breakout for me), HM-10 BLE module and LiPo battery with PDB.

Wiring diagram
          FRONT
    M1 (D10)   M2 (D9)
        \     /
         [   ]
        /     \
    M4 (D6)   M3 (D5)
          BACK

Connect the ESCs' signal lines to the pins shown above, connect all ESCs' ground lines to GND. If there are several ESCs that provide BEC, power up the Arduino only by one ESC. M1 and M3 rotate clockwise, M2 and M4 rotate counter-clockwise.

Connect the MPU-6050’s SDA to A4 and SCL to A5. Secure the MPU-6050 on top of the flight controller board, right in the middle of the frame. Connect HM-10’s TX to D2 and RX to D3. Add a voltage divider to D3 pin as the HM-10’s RX pin uses 3.3 volts.

Getting started

First, calibrate the ESCs with props removed (sketch does not do it), then upload the sketch using the Arduino IDE. There are no additional libraries required.

On powering up, place your drone flat and motionless for 5 seconds for calibration of the gyroscope. Your phone will be notified once HM10 READY + MPU OK message appears, indicating readiness to fly. Motors will not turn on until the ARM command is received.

Commands

This drone is controlled using Serial Bluetooth Terminal application for Android. Within this application, go to the menu, select Devices, then click the Bluetooth LE tab and connect to HM-10 module . 

THR 0     -> sets throttle (0-100%)
ARM       -> arms motors
DISARM    -> disarms motors and resets throttle to 0
How it works

The program combines data from the gyroscope (high rate of update, drifts) with the data from the accelerometer (low rate of update, does not drift) to calculate the angle of the tilt. Then the PD controller accelerates motors on the low side and decelerates motors on the high side. The values of gains (Kp , Kd ) are different based on frame size and weight distribution.

Before flying, tilt the drone by hand with the props off. The Serial Monitor should show pitch and roll changing the right way for how the sensor is mounted, and the motors on the low side should speed up.
