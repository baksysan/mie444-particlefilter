#include <Wire.h>
#include <Servo.h>
#include <util/atomic.h>
#include <math.h>


// Classic ATmega328P Uno. Pinout explicitly confirmed by the user.
const uint8_t SLAVE_ADDRESS = 0x08;
const uint8_t enA = 11, in1 = 13, in2 = 4;
const uint8_t enB = 5, in3 = 8, in4 = 7;
const uint8_t ENCODER_A = 2, ENCODER_B = 3;
const uint8_t gripper_close_pin = 6, gripper_up_down_pin = 10;

// TUNING: placeholders, NOT measured values for your robot.
const float COUNTS_PER_REV_A = 823.1f, COUNTS_PER_REV_B = 823.1f;
const float STRAIGHT_RPM = 50.0f, TURN_RPM = 15.0f;
const float DECEL_RPM_PER_SEC = 500.0f;
const int STRAIGHT_PWM_LIMIT = 255, TURN_PWM_LIMIT = 255;
const unsigned long KEY_TIMEOUT_MS = 120;
const unsigned long SAMPLE_US = 20000UL, QUIET_MS = 100;
const float FILTER_SECONDS = 0.05f;

struct Wheel {
  float kp, ki, kd;
  float target, measured, previousMeasured, integral;
  int pwm;
};
// User-requested gains: Kp=5, Ki=10, Kd=0 (dt is in seconds).
Wheel motorA = {10.0f, 20.0f, 0.0f, 0, 0, 0, 0, 0};
Wheel motorB = {10.0f, 20.0f, 0.0f, 0, 0, 0, 0, 0};

Servo open_close_servo, up_down_servo;
volatile uint32_t encoderA = 0, encoderB = 0;
// Latest command wins, except x is latched until loop consumes it.
volatile char pendingCommand = 0;
volatile bool stopPending = false;
uint32_t previousCountA = 0, previousCountB = 0;
uint32_t observedCountA = 0, observedCountB = 0;
unsigned long lastSampleUs = 0, lastPrintMs = 0;
unsigned long lastPulseAMs = 0, lastPulseBMs = 0, lastKeyMs = 0;
int wantedDirA = 0, wantedDirB = 0;
int previousDriveDirA = 0, previousDriveDirB = 0;
int outputDirA = 0, outputDirB = 0;
int pwmLimit = STRAIGHT_PWM_LIMIT;
float requestedRPM = 0;
bool motionActive = false, reversing = false;
// Latched drive request: set by w/a/s/d, cleared by e (or x/gripper stop).
bool driveCommanded = false;

// Gripper: 0=idle, 1=down, 2=grip/release, 3=up.
uint8_t gripperStep = 0;
bool pickup = false;
unsigned long gripperStepMs = 0, lastGripperMessageMs = 0;
char gripperHeld = 0;

// ---- PC control (master's mode 'c') ---------------------------------------
// Wheel sides, taken from turn_WASD: a right turn runs A backward and B
// forward, so motor A = RIGHT wheel and motor B = LEFT wheel.
// Drive message from the master, 5 bytes:
//   'v', left direction (-1/0/+1), right direction (-1/0/+1), RPM x10 (low, high)
// Both wheels always get the same RPM (the robot drives straight or turns in
// place). It is kept apart from pendingCommand so a 'g' or 'l' is never
// overwritten by the next drive message.
const unsigned long VELOCITY_TIMEOUT_MS = 500; // stop if the master goes silent
const float MAX_COMMAND_RPM = 120.0f;
volatile bool velocityPending = false;
volatile int8_t pendingLeftDir = 0, pendingRightDir = 0;
volatile uint16_t pendingRpmX10 = 0;
bool velocityDrive = false;          // true while driven by 'v' messages
unsigned long lastVelocityMs = 0;
// Signed encoder totals for odometry. The encoders are single-channel, so the
// sign comes from the direction each motor was last driven in.
volatile int32_t odomLeft = 0, odomRight = 0;
uint32_t odomSeenA = 0, odomSeenB = 0;
int odomDirA = 1, odomDirB = 1;

void readEncoder_A() { ++encoderA; }
void readEncoder_B() { ++encoderB; }

void receiveEvent(int) {
  if (!Wire.available()) return;
  char cmd = (char)Wire.read();
  if (cmd == 'v') {
    if (Wire.available() >= 4) {
      int8_t l = (int8_t)Wire.read(), r = (int8_t)Wire.read();
      uint16_t lo = (uint8_t)Wire.read(), hi = (uint8_t)Wire.read();
      if (!stopPending) {
        pendingLeftDir = l; pendingRightDir = r;
        pendingRpmX10 = lo | (hi << 8);
        velocityPending = true;
      }
    }
    while (Wire.available()) Wire.read();
    return;
  }
  while (Wire.available()) Wire.read();
  if (cmd == 'x') { stopPending = true; pendingCommand = 0; velocityPending = false; }
  else if (!stopPending &&
           (cmd == 'w' || cmd == 's' || cmd == 'a' || cmd == 'd' ||
            cmd == 'e' || cmd == 'g' || cmd == 'l')) pendingCommand = cmd;
}

// The master asks for 8 bytes: left and right signed encoder totals.
void requestEvent() {
  int32_t counts[2] = { odomLeft, odomRight };
  Wire.write((uint8_t *)counts, sizeof(counts));
}

void snapshotCounts(uint32_t &a, uint32_t &b) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { a = encoderA; b = encoderB; }
}

// The only function that writes motor PWM. +1 retains the working sketch's
// LOW/HIGH forward polarity. Changing direction first disables that output.
void writeMotor(uint8_t en, uint8_t p1, uint8_t p2,
                int direction, int pwm, int &oldDirection) {
  pwm = constrain(pwm, 0, 255);
  if (direction != oldDirection) {
    analogWrite(en, 0);
    digitalWrite(p1, direction < 0 ? HIGH : LOW);
    digitalWrite(p2, direction > 0 ? HIGH : LOW);
    oldDirection = direction;
  }
  analogWrite(en, direction == 0 ? 0 : pwm);
}

void resetController(Wheel &m) {
  m.target = 0; m.integral = 0; m.pwm = 0;
  m.previousMeasured = m.measured;
}

void disableWheels() {
  resetController(motorA); resetController(motorB);
  writeMotor(enA, in1, in2, 0, 0, outputDirA);
  writeMotor(enB, in3, in4, 0, 0, outputDirB);
}

void stopMotion() {
  motionActive = false; reversing = false; requestedRPM = 0;
  driveCommanded = false;
  wantedDirA = wantedDirB = 0;
  disableWheels();
  // Retain previousDriveDir: a quick x followed by reverse still waits.
}

void requestMotion(int a, int b, bool turning, unsigned long now) {
  bool directionChanged = a != wantedDirA || b != wantedDirB;
  wantedDirA = a; wantedDirB = b;
  requestedRPM = turning ? TURN_RPM : STRAIGHT_RPM;
  pwmLimit = turning ? TURN_PWM_LIMIT : STRAIGHT_PWM_LIMIT;
  lastKeyMs = now; motionActive = true; driveCommanded = true;
  if (directionChanged) {
    reversing = (previousDriveDirA != 0 && a != previousDriveDirA) ||
                (previousDriveDirB != 0 && b != previousDriveDirB);
    disableWheels();
    if (reversing) lastPulseAMs = lastPulseBMs = now;
  }
}

void move_WASD(bool forward, unsigned long now) {
  requestMotion(forward ? 1 : -1, forward ? 1 : -1, false, now);
}
void turn_WASD(bool right, unsigned long now) {
  requestMotion(right ? -1 : 1, right ? 1 : -1, true, now);
}

void gripper_open() { open_close_servo.write(175); delay(500);}

void gripper_close() { open_close_servo.write(50); delay(500); }

  
void gripper_up() { up_down_servo.write(120); delay(1000); }
void gripper_down() { up_down_servo.write(40); delay(1000);}

void processCommand(unsigned long now) {
  char cmd;
  bool stop;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    stop = stopPending; stopPending = false;
    cmd = pendingCommand; pendingCommand = 0;
  }
  if (stop) {
    stopMotion();
    gripperStep = 0; // Cancel future servo steps; hold current servo angles.
    gripperHeld = 0;
    return;
  }
  if (!cmd) return;
  if (cmd == 'g' || cmd == 'l') {
    bool repeated = cmd == gripperHeld &&
                    (unsigned long)(now - lastGripperMessageMs) <= KEY_TIMEOUT_MS;
    gripperHeld = cmd; lastGripperMessageMs = now;
    if (gripperStep || repeated) return;
    stopMotion();
    pickup = cmd == 'g'; gripper_down();
    gripperStep = 1; gripperStepMs = now;
    return;
  }
  if (gripperStep) return; // Discard movement throughout the sequence.
  gripperHeld = 0;
  velocityDrive = false;   // w/a/s/d/e: manual driving, no 'v' timeout
  // e: end the latched drive; updateControl ramps down, then stops.
  if (cmd == 'e') driveCommanded = false;
  else if (cmd == 'w') move_WASD(true, now);
  else if (cmd == 's') move_WASD(false, now);
  else if (cmd == 'a') turn_WASD(false, now);
  else if (cmd == 'd') turn_WASD(true, now);
}

// Apply the newest 'v' drive message, and stop if they stop arriving.
void processVelocity(unsigned long now) {
  bool have; int8_t l, r; uint16_t rpmX10;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    have = velocityPending; velocityPending = false;
    l = pendingLeftDir; r = pendingRightDir; rpmX10 = pendingRpmX10;
  }
  if (have && !gripperStep) {       // ignored during a gripper sequence
    velocityDrive = true; lastVelocityMs = now;
    int left = l > 0 ? 1 : (l < 0 ? -1 : 0);
    int right = r > 0 ? 1 : (r < 0 ? -1 : 0);
    float rpm = fminf(rpmX10 / 10.0f, MAX_COMMAND_RPM);
    if (left == 0 || right == 0 || rpm <= 0) {
      driveCommanded = false;       // same as 'e': ramp down, then stop
    } else {
      gripperHeld = 0;
      requestMotion(right, left, left != right, now);  // A = right, B = left
      requestedRPM = rpm;           // replace the fixed STRAIGHT/TURN RPM
    }
  }
  if (velocityDrive && driveCommanded &&
      (unsigned long)(now - lastVelocityMs) > VELOCITY_TIMEOUT_MS)
    driveCommanded = false;         // master went silent
}

// Add new encoder pulses to the signed totals. A stopped or coasting wheel
// keeps counting in the direction it was last driven.
void updateOdometry() {
  uint32_t a, b;
  snapshotCounts(a, b);
  int32_t dA = (int32_t)(a - odomSeenA) * odomDirA;
  int32_t dB = (int32_t)(b - odomSeenB) * odomDirB;
  odomSeenA = a; odomSeenB = b;
  if (outputDirA) odomDirA = outputDirA;
  if (outputDirB) odomDirB = outputDirB;
  if (dA || dB) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { odomRight += dA; odomLeft += dB; }
  }
}

void updateGripper(unsigned long now) {
  if (!gripperStep || (unsigned long)(now - gripperStepMs) < 1000UL) return;
  gripperStepMs = now;
  if (gripperStep == 1) {
    if (pickup) gripper_close(); else gripper_open();
    gripperStep = 2;
  } else if (gripperStep == 2) {
    gripper_up(); gripperStep = 3;
  } else {
    gripperStep = 0;
    // Only commands received after completion may start moving again.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { pendingCommand = 0; }
  }
}

float approach(float value, float goal, float dt) {
  // Apply increases immediately; retain a controlled ramp-down after e.
  if (goal >= value) return goal;
  return fmaxf(value - DECEL_RPM_PER_SEC * dt, goal);
}

int computePID(Wheel &m, float dt, int limit) {
  if (m.target <= 0) { resetController(m); return 0; }
  float error = m.target - m.measured;
  float derivative = (m.measured - m.previousMeasured) / dt;
  m.previousMeasured = m.measured;
  if (m.ki <= 0) m.integral = 0;
  float candidate = m.integral + (m.ki > 0 ? error * dt : 0);
  float trial = m.kp * error + m.ki * candidate - m.kd * derivative;
  // Reject integration only when it would push farther into saturation.
  if (!((trial > limit && error > 0) || (trial < 0 && error < 0)))
    m.integral = candidate;
  float output = m.kp * error + m.ki * m.integral - m.kd * derivative;
  m.pwm = (int)(constrain(output, 0.0f, (float)limit) + 0.5f);
  return m.pwm;
}

void updateControl(unsigned long nowMs) {
  uint32_t a, b;
  snapshotCounts(a, b);
  // Observe counts every loop for the reversal quiet-time interlock.
  if (a != observedCountA) { observedCountA = a; lastPulseAMs = nowMs; }
  if (b != observedCountB) { observedCountB = b; lastPulseBMs = nowMs; }
  unsigned long nowUs = micros();
  unsigned long elapsed = nowUs - lastSampleUs;
  if (elapsed < SAMPLE_US) return;
  lastSampleUs = nowUs;
  float dt = elapsed / 1.0e6f;
  float rawA = (uint32_t)(a - previousCountA) * 60.0f / (COUNTS_PER_REV_A * dt);
  float rawB = (uint32_t)(b - previousCountB) * 60.0f / (COUNTS_PER_REV_B * dt);
  previousCountA = a; previousCountB = b;
  float alpha = dt / (FILTER_SECONDS + dt);
  motorA.measured += alpha * (rawA - motorA.measured);
  motorB.measured += alpha * (rawB - motorB.measured);

  if (!motionActive || gripperStep) { disableWheels(); return; }
  // Drive continues indefinitely until e (no key-repeat timeout).
  bool fresh = driveCommanded;
  if (reversing) {
    disableWheels();
    if (!fresh) { stopMotion(); return; }
    if ((unsigned long)(nowMs - lastPulseAMs) < QUIET_MS ||
        (unsigned long)(nowMs - lastPulseBMs) < QUIET_MS) return;
    reversing = false;
    Serial.print("slave reversal wait over at ms "); Serial.println(nowMs);  // debug
  }
  float goal = fresh ? requestedRPM : 0;
  motorA.target = approach(motorA.target, goal, dt);
  motorB.target = approach(motorB.target, goal, dt);
  if (!fresh && motorA.target <= 0 && motorB.target <= 0) {
    stopMotion(); return;
  }
  previousDriveDirA = wantedDirA; previousDriveDirB = wantedDirB;
  int aPWM = computePID(motorA, dt, pwmLimit);
  int bPWM = computePID(motorB, dt, pwmLimit);
  writeMotor(enA, in1, in2, wantedDirA, aPWM, outputDirA);
  writeMotor(enB, in3, in4, wantedDirB, bPWM, outputDirB);
}

void printTelemetry(unsigned long now) {
  if ((unsigned long)(now - lastPrintMs) < 100UL) return;
  lastPrintMs = now;
  // Integer RPM keeps a line short enough for the Uno's TX buffer.
  char line[64];
  int n = snprintf(line, sizeof(line), "%d,%d,%d,%d,%d,%d\n",
    (int)constrain(motorA.target, 0.0f, 30000.0f),
    (int)constrain(motorA.measured, 0.0f, 30000.0f), motorA.pwm,
    (int)constrain(motorB.target, 0.0f, 30000.0f),
    (int)constrain(motorB.measured, 0.0f, 30000.0f), motorB.pwm);
  if (n > 0 && n < (int)sizeof(line) && Serial.availableForWrite() >= n)
    Serial.write((const uint8_t *)line, n); // Skip rather than block.
}

void setup() {
  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
  stopMotion();
  // INPUT assumes driven encoder outputs. Open-collector outputs need pull-ups.
  pinMode(ENCODER_A, INPUT); pinMode(ENCODER_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), readEncoder_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), readEncoder_B, RISING);
  snapshotCounts(previousCountA, previousCountB);
  observedCountA = previousCountA; observedCountB = previousCountB;
  odomSeenA = previousCountA; odomSeenB = previousCountB;
  lastSampleUs = micros(); lastPulseAMs = lastPulseBMs = millis();
  open_close_servo.attach(gripper_close_pin);
  up_down_servo.attach(gripper_up_down_pin);

  open_close_servo.write(175);    //ADDED TO RESET INTERNAL GRIPPER

  Serial.begin(9600);
  Serial.println("targetA,rpmA,pwmA,targetB,rpmB,pwmB");
  Wire.begin(SLAVE_ADDRESS); Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop() {
  unsigned long now = millis();
  processCommand(now);
  processVelocity(now);
  updateGripper(now);
  updateControl(now);
  updateOdometry();
  printTelemetry(now);
}
