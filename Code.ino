
#include <Servo.h>
#include <SoftwareSerial.h>
#include <Wire.h>


// =======================
// BLUETOOTH SERIAL (HM-10)
// =======================
SoftwareSerial BT(2, 3);  // RX = D2 (from HM-10 TX), TX = D3


// =======================
// ESC CONFIG
// =======================
// M1=front-left, M2=front-right, M3=back-right, M4=back-left
// wiring: M1=10, M2=9, M3=5, M4=6
const int NUM_MOTORS = 4;
const int escPins[NUM_MOTORS] = {10, 9, 5, 6};


const int MIN_PULSE = 1000;   // μs
const int MAX_PULSE = 2000;   // μs


Servo esc[NUM_MOTORS];


// =======================
// CONTROL STATE
// =======================
bool armed = false;
int throttlePct = 0;    // 0–100%


String btLine = "";
String usbLine = "";


// =======================
// MPU-6050 CONFIG
// =======================
const int MPU_ADDR = 0x68;


float anglePitch = 0.0f;
float angleRoll  = 0.0f;
float gyroXoffset = 0.0f;
float gyroYoffset = 0.0f;
float gyroXrate   = 0.0f;   // deg/s
float gyroYrate   = 0.0f;   // deg/s
unsigned long lastMPUTimeMs = 0;


// =======================
// PID GAINS
// =======================
// Simple angle + rate (PD-like)
float Kp_angle_pitch = 4.0f;
float Kd_rate_pitch  = 0.15f;


float Kp_angle_roll  = 4.0f;
float Kd_rate_roll   = 0.15f;


float pitchSetpoint = 0.0f;  // level
float rollSetpoint  = 0.0f;  // level


float pitchPID = 0.0f;
float rollPID  = 0.0f;


const float MAX_CORRECTION_US = 200.0f;


// =======================
// SET ALL ESC PULSES
// =======================
void setAllEscPulse(int us) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    esc[i].writeMicroseconds(us);
  }
}


// =======================
// READ RAW DATA
// =======================
void readMPU(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);


  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // temperature
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();
}


// =======================
// UPDATE ANGLES FOR MPU
// =======================
void updateMPU() {
  static unsigned long lastMicrosMPU = micros();
  unsigned long now = micros();
  float dt = (now - lastMicrosMPU) / 1000000.0f;
  lastMicrosMPU = now;


  int16_t ax, ay, az, gx, gy, gz;
  readMPU(ax, ay, az, gx, gy, gz);


  // Convert gyro to deg/s
  gyroXrate = (gx - gyroXoffset) / 131.0f;
  gyroYrate = (gy - gyroYoffset) / 131.0f;


  // Accelerometer angles (degrees)
  float accPitch = atan2(ay, az) * 180.0 / PI;
  float accRoll  = atan2(-ax, sqrt((long)ay * ay + (long)az * az)) * 180.0 / PI;


  // Complementary filter
  const float alpha = 0.98f;
  anglePitch = alpha * (anglePitch + gyroXrate * dt) + (1.0f - alpha) * accPitch;
  angleRoll  = alpha * (angleRoll  + gyroYrate * dt) + (1.0f - alpha) * accRoll;


  // (Optional) debug print at ~10 Hz
  if (millis() - lastMPUTimeMs > 100) {
    lastMPUTimeMs = millis();
    Serial.print("Pitch: ");
    Serial.print(anglePitch, 2);
    Serial.print("  Roll: ");
    Serial.println(angleRoll, 2);
  }
}


// =======================
// PID: COMPUTE PITCH & ROLL CORRECTIONS
// =======================
void updatePID() {
  // Error = desired - measured
  float errorPitch = pitchSetpoint - anglePitch;
  float errorRoll  = rollSetpoint  - angleRoll;


  // PD controller: angle error + gyro damping
  float pTermPitch = Kp_angle_pitch * errorPitch;
  float dTermPitch = -Kd_rate_pitch * gyroXrate; 
  pitchPID = pTermPitch + dTermPitch;


  float pTermRoll = Kp_angle_roll * errorRoll;
  float dTermRoll = -Kd_rate_roll * gyroYrate;
  rollPID = pTermRoll + dTermRoll;


  // Limit corrections
  if (pitchPID >  MAX_CORRECTION_US) pitchPID =  MAX_CORRECTION_US;
  if (pitchPID < -MAX_CORRECTION_US) pitchPID = -MAX_CORRECTION_US;


  if (rollPID >  MAX_CORRECTION_US) rollPID =  MAX_CORRECTION_US;
  if (rollPID < -MAX_CORRECTION_US) rollPID = -MAX_CORRECTION_US;
}


// =======================
// MOTOR MIXING + OUTPUT
// =======================
void updateMotors() {
  if (!armed) {
    setAllEscPulse(MIN_PULSE);
    return;
  }


  // Base throttle
  int basePulse = map(throttlePct, 0, 100, MIN_PULSE, MAX_PULSE);


  // M1: front-left  (pin 10) = base + pitch + roll
  // M2: front-right (pin 9)  = base + pitch - roll
  // M3: back-right  (pin 5)  = base - pitch - roll
  // M4: back-left   (pin 6)  = base - pitch + roll


  int m1 = basePulse + (int)pitchPID + (int)rollPID;
  int m2 = basePulse + (int)pitchPID - (int)rollPID;
  int m3 = basePulse - (int)pitchPID - (int)rollPID;
  int m4 = basePulse - (int)pitchPID + (int)rollPID;


  // Constrain to min/max
  m1 = constrain(m1, MIN_PULSE, MAX_PULSE);
  m2 = constrain(m2, MIN_PULSE, MAX_PULSE);
  m3 = constrain(m3, MIN_PULSE, MAX_PULSE);
  m4 = constrain(m4, MIN_PULSE, MAX_PULSE);


  esc[0].writeMicroseconds(m1); // pin 10
  esc[1].writeMicroseconds(m2); // pin 9
  esc[2].writeMicroseconds(m3); // pin 5
  esc[3].writeMicroseconds(m4); // pin 6
}


// =======================
// COMMAND HANDLER
// =======================
void handleCommand(String cmd, bool fromBT) {
  cmd.trim();
  cmd.toUpperCase();


  Serial.print("[CMD ");
  Serial.print(fromBT ? "BT" : "USB");
  Serial.print("] ");
  Serial.println(cmd);


  if (cmd == "ARM") {
    armed = true;
    Serial.println("-> ARMED");
    if (fromBT) BT.println("OK ARMED");
  }
  else if (cmd == "DISARM") {
    armed = false;
    throttlePct = 0;
    setAllEscPulse(MIN_PULSE);
    Serial.println("-> DISARMED");
    if (fromBT) BT.println("OK DISARMED");
  }
  else if (cmd.startsWith("THR")) {
    int spaceIndex = cmd.indexOf(' ');
    if (spaceIndex > 0) {
      int val = cmd.substring(spaceIndex + 1).toInt();
      val = constrain(val, 0, 100);
      throttlePct = val;


      Serial.print("-> THR = ");
      Serial.println(throttlePct);
      if (fromBT) {
        BT.print("OK THR ");
        BT.println(throttlePct);
      }
    } else {
      Serial.println("-> ERR: THR needs value");
      if (fromBT) BT.println("ERR THR NEEDS VALUE");
    }
  }
  else if (cmd == "PING") {
    Serial.println("-> PONG");
    if (fromBT) BT.println("PONG");
  }
  else if (cmd.length() > 0) {
    Serial.println("-> UNKNOWN CMD");
    if (fromBT) BT.println("ERR UNKNOWN");
  }
}


// =======================
// SETUP
// =======================
void setup() {
  Serial.begin(9600);
  BT.begin(9600);


  // MPU init
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0);    // wake up
  Wire.endTransmission(true);
  delay(100);


  // Gyro calibration 
  const int N = 2000;
  long sumX = 0, sumY = 0;
  Serial.println("Calibrating gyro, do not move the drone...");
  for (int i = 0; i < N; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    readMPU(ax, ay, az, gx, gy, gz);
    sumX += gx;
    sumY += gy;
    delay(2);
  }
  gyroXoffset = sumX / (float)N;
  gyroYoffset = sumY / (float)N;
  Serial.println("Gyro calibration done.");


  Serial.println("=== DRONE ESC + HM-10 + MPU6050 (LEVELING) ===");
  Serial.println("Commands: ARM, DISARM, THR <0-100>, PING");
  Serial.println("---------------------------------------------");


  BT.println("HM10 READY + MPU OK");


  // Attach ESCs
  for (int i = 0; i < NUM_MOTORS; i++) {
    esc[i].attach(escPins[i]);
  }


  // Start motors at minimum
  setAllEscPulse(MIN_PULSE);
  delay(1500);  // safe startup
}


// =======================
// LOOP
// =======================
void loop() {
  // 1) Update IMU + PID
  updateMPU();
  updatePID();


  // 2) Handle Bluetooth commands
  while (BT.available()) {
    char c = BT.read();
    if (c == '\n') {
      handleCommand(btLine, true);
      btLine = "";
    } else if (c != '\r') {
      btLine += c;
    }
  }


  // 3) Handle USB Serial commands (for debugging)
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      handleCommand(usbLine, false);
      usbLine = "";
    } else if (c != '\r') {
      usbLine += c;
    }
  }


  // 4) Update motors with current throttle + PID corrections
  updateMotors();
}

