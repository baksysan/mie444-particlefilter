
#include <Wire.h>
#include <NewPing.h>

#define MPU 0x68 // MPU6050 I2C address
#define ArduinoSlave 0x08

//raw acceleration values from sensor. divide by 16384 to get acceleration in g's (9.8 m/s^2). (+/- 2g max measurement)
int16_t AccX, AccY, AccZ;

//raw gyroscope values from sensor. divide by 131 to obtain angular velocity in degrees/second (+/- 250 degrees/second max measurement)
int16_t gyroX, gyroY, gyroZ;

//offset values for the gyroscope, set via the callibrate function. values are in degrees/second.
float gyroOffsetX, gyroOffsetY, gyroOffsetZ;

//Ultrasonic Sensor Defenitions

#define TRIG_PIN1 13
#define ECHO_PIN1 11

#define TRIG_PIN2 10
#define ECHO_PIN2 8

#define TRIG_PIN3 12
#define ECHO_PIN3 9

#define TRIG_PIN4 6
#define ECHO_PIN4 5

#define IR1 A0
#define IR2 A1


#define MAX_DISTANCE 400

unsigned long pingTime;

NewPing sonar0(TRIG_PIN1, ECHO_PIN1, MAX_DISTANCE);
NewPing sonar1(TRIG_PIN2, ECHO_PIN2, MAX_DISTANCE);
NewPing sonar2(TRIG_PIN3, ECHO_PIN3, MAX_DISTANCE);
NewPing sonar3(TRIG_PIN4, ECHO_PIN4, MAX_DISTANCE);


char mode = 'n';

char input;
String command;

byte response = 0x00;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Wake up MPU6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); // Power management register
  Wire.write(0x08); // Wake up, disable temp sensor
  Wire.endTransmission(true);
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);

  //update gyro offsets to account for gyro drift. takes about 5 seconds to callibrate. decrease samples to increase callibration speed.
  //calibrateGyro(1000);

}

void loop() {
  


  Serial.print('\r');
  Serial.println("Choose operation mode by typing key:");
  Serial.println("w - wasd movement (disabled for now)");
  Serial.println("c - control code");
  Serial.println("s - sensor check + calibration (TRY THIS)");
  Serial.println("t - other...");

  bool firstRun = true;

  while (mode == 'n'){

    while (Serial.available() < 1){

    }

    mode = Serial.read();

  }

  //wasd control mode
  while (mode == 'w'){
    if (firstRun){
      Serial.println("Entered WASD control mode");
      //Add code to calibrate WASD -- I2C to arduino to calibrate it
      //sendByte(ArduinoSlave, (byte)mode);
      firstRun = false;
    }

    while (Serial.available() < 1){

    }

    input = Serial.read();

    //send input to slave
    if (input == 'w' || input == 'a' || input == 's' || input == 'd' || input == 'g' || input == 'l' || input == 'e') {
      sendByte(ArduinoSlave, input);
    }

    //request data from arduinoSlave
    //requestByte(ArduinoSlave, 1);

    if (input == 'x'){
      
      mode = 'n';
      break;
    }

    if(response == 'y'){
      Serial.println("calibration Done");
    }
    


  }

  while(mode == 'c'){

    // PC control: the Python programs (mcl.py, serial_tester.py) send C / G / L
    // lines and get R lines back. The code is in the "PC CONTROL MODE" section
    // at the bottom of this file. Send x to go back to the menu.
    if (firstRun){
      controlModeBegin();
      firstRun = false;
    }

    if (!controlModeStep()){
      mode = 'n';
      break;
    }

  }


  while(mode == 's'){

    if (firstRun){
      Serial.println("Entered Sensor Mode. Callibrating IMU. PLEASE HOLD IT STILL");
      // calibrateGyro(1000);
      // Serial.println("Callibration Done.");

      firstRun = false;
    }


      // while (Serial.available() < 1){
      //   printGyro(true);
      //   
      printAcc(false);
      //   printSonarLogic2();
      //   delay(100);

      // }
      input = Serial.read();

      if (input == 'w' || input == 'a' || input == 's' || input == 'd'  || input == 'g' || input == 'l' || input == 'x') {
        sendByte(ArduinoSlave, input);
      }

      if (input == 'x'){
      
        mode = 'n';
        break;

      }

      if (input == 'u') {
        // Get second byte of data
        while (Serial.available() < 1) {}
        char sensor_choice = Serial.read();

        if (sensor_choice >= '0' && sensor_choice <= '3') {

          int choice = sensor_choice - '0'; //Convert character to integer
          sonarAverage(choice, 10);
          Serial.print("[Ultrasonic:");
          Serial.print(float(pingTime / 2 * 0.034), 2);
          Serial.println("]");

        }


      }

  }

  while(Serial.available() > 0){
    Serial.read();
  }

  Serial.println("                              ");
  Serial.println("                              ");
  Serial.println("                              ");
  Serial.println("                              ");

  mode = 'n';
}

void sendByte(int address, byte data){

  Wire.beginTransmission(address);
  Wire.write(data);
  byte err = Wire.endTransmission(true);

  if (err != 0) {
    Serial.print("I2C error: ");
    Serial.println(err);
  }
}

void requestByte(int address, int numBytes){
  
  response = Wire.requestFrom(address, numBytes);

}

void readSonar(unsigned int sensorNum){

  pingTime = 0;


  if (sensorNum == 0){
    pingTime = sonar0.ping();
  }
  else if (sensorNum == 1){
    pingTime = sonar1.ping();
  }
  else if (sensorNum == 2){
    pingTime = sonar2.ping();
  }
  else if (sensorNum == 3){
    pingTime = sonar3.ping();
  }

}

void sonarAverage(unsigned int sensorNum, unsigned int samples){

  long sumPing = 0;

  for (unsigned int i = 0; i < samples; i++){
    readSonar(sensorNum);
    sumPing += pingTime;
    delay(5);

  }

  pingTime = sumPing/samples;

}

//update global acceleration variables to current IMU readings
void readAcc(){

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU, 6, true);

  if (Wire.available() == 6){

    AccX = (Wire.read() << 8) | Wire.read(); 
    AccY = (Wire.read() << 8) | Wire.read(); 
    AccY = (Wire.read() << 8) | Wire.read(); 
  }
}

//debug command for seeing all sonar sensor readings
void printSonarLogic(){

  Serial.print('\r');
  Serial.print("Front: ");
  sonarAverage(0, 10);
  if (float(pingTime/2*0.034) < 10){Serial.print('0');}
  Serial.print((float(pingTime/2*0.034)),2);

  Serial.print("    | Left US: ");
  sonarAverage(1, 10);
  if (float(pingTime/2*0.034) < 10){Serial.print('0');}
  Serial.print((float(pingTime/2*0.034)),2);  

  Serial.print("    | Right US: ");
  sonarAverage(2, 10);
  if (float(pingTime/2*0.034) < 10){Serial.print('0');}
  Serial.print((float(pingTime/2*0.034)),2);

  Serial.print("    | Right IR: ");
  if (analogRead(IR1) > 250) {Serial.print('X');}
  else {Serial.print('O');}
  Serial.print("    | Left IR: ");
  if (analogRead(IR2) > 250) {Serial.print('X');}
  else {Serial.print('O');}

  Serial.print("    | Back ");
  sonarAverage(3, 10);
  if (float(pingTime/2*0.034) < 10){Serial.print('0');}
  Serial.print((float(pingTime/2*0.034)),2);

  Serial.print("     ");

}

void printSonar(){

  Serial.print('\r');
  Serial.print("Front: ");
  sonarAverage(0, 10);
  Serial.print(float(pingTime/2*0.034), 2);

  Serial.print("    | Left: ");
  sonarAverage(1, 10);
  Serial.print(float(pingTime/2*0.034), 2);

  Serial.print("    | Right: ");
  sonarAverage(2, 10);
  Serial.print(float(pingTime/2*0.034), 2);

  Serial.print("    | Back ");
  sonarAverage(3, 10);
  Serial.print(float(pingTime/2*0.034), 2);

  Serial.print("     ");

}

//get the average acceleration of several IMU readings
void accAverage(unsigned int samples){

  long sumX = 0;
  long sumY = 0;
  long sumZ = 0;

  for (unsigned int i = 0; i < samples; i++){
    readAcc();

    sumX += AccX;
    sumY += AccY;
    sumZ += AccZ;

    delay(5);

  }

  AccX = int(sumX / float(samples));
  AccY = int(sumY / float(samples));
  AccZ = int(sumZ / float(samples));

}

//debug function moslty. Will display average acceleration readings from 10 samples
void printAcc(bool First){

  accAverage(10);

  float ax = AccX / 16384.0;
  float ay = AccY / 16384.0;
  float az = AccZ / 16384.0;

  if (First){
    Serial.print("\r");
  }
  

  Serial.print("Acc X: ");
  Serial.print(ax, 2);

  Serial.print(" | Acc Y: ");
  Serial.print(ay, 2);

  Serial.print(" | Acc Z: ");
  Serial.print(az, 2);

  Serial.print("     "); // Clear leftover characters

}

//update global gyro variables to current IMU readings
void readGyro(){

  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU, 6, true);

  if (Wire.available() == 6){

    gyroX = (Wire.read() << 8) | Wire.read();
    gyroY = (Wire.read() << 8) | Wire.read();
    gyroZ = (Wire.read() << 8) | Wire.read();
  }
}

//get the average gyro values of several IMU readings
void gyroAverage(unsigned int samples){
  
  long sumX = 0;
  long sumY = 0;
  long sumZ = 0;

  for (unsigned int i = 0; i < samples; i++){
    readGyro();

    sumX += gyroX;
    sumY += gyroY;
    sumZ += gyroZ;

    delay(5);

  }

  gyroX = int(sumX / float(samples));
  gyroY = int(sumY / float(samples));
  gyroZ = int(sumZ / float(samples));


}

//function should be ran at start up to callibrate gyroscope. will update gyro offset variables. 
void calibrateGyro(unsigned int samples){

  gyroAverage(samples);

  gyroOffsetX = gyroX / 131.0;
  gyroOffsetY = gyroY / 131.0;
  gyroOffsetZ = gyroZ / 131.0;

  Serial.println("\nGyro Calibrated Successfully.");
  Serial.println(gyroOffsetX, 2);
  Serial.println(gyroOffsetY, 2);
  Serial.println(gyroOffsetZ, 2);

}

//debug function moslty. Will display average gyro readings from 10 samples
void printGyro(bool First){

  gyroAverage(10);

  float gx = gyroX / 131.0 - gyroOffsetX;
  float gy = gyroY / 131.0 - gyroOffsetY;
  float gz = gyroZ / 131.0 - gyroOffsetZ;

  if (First){
    Serial.print("\r");
  }

  Serial.print("Gyro X: ");
  Serial.print(gx, 2);

  Serial.print(" | Gyro Y: ");
  Serial.print(gy, 2);

  Serial.print(" | Gyro Z: ");
  Serial.print(gz, 2);

  Serial.print("     "); // Clear leftover characters

}

void printSonarLogic2(){

  // ---------- Line 1: IR1 | Front | IR2 ----------
  Serial.print("\r\033[K");

  Serial.print("IR1: ");
  if (analogRead(IR2) > 250) {Serial.print('X');}
  else {Serial.print('O');}
  Serial.print("       | ");              // pads "IR1: X" out to 13 chars

  Serial.print("Front: ");
  sonarAverage(0, 10);
  if (float(pingTime/2*0.034) < 100){Serial.print(' ');}
  if (float(pingTime/2*0.034) < 10){Serial.print('0');}
  Serial.print(float(pingTime/2*0.034), 2);

  Serial.print(" | IR2: ");
  if (analogRead(IR1) > 250) {Serial.print('X');}
  else {Serial.print('O');}

  Serial.print("\r\n");

  // ---------- Line 2: Left | (blank) | Right ----------
  Serial.print("\033[K");

  Serial.print("Left: ");
  sonarAverage(1, 10);
  if (float(pingTime/2*0.034) < 100){Serial.print(' ');}
  if (float(pingTime/2*0.034) < 10){Serial.print('0');}
  Serial.print(float(pingTime/2*0.034), 2);

  Serial.print(" | ");                    // "Left: " is 1 char shorter than "Front: "...
  Serial.print("              | ");       // ...so this blank middle cell is 14 spaces, then the divider

  Serial.print("Right: ");
  sonarAverage(2, 10);
  if (float(pingTime/2*0.034) < 100){Serial.print(' ');}
  if (float(pingTime/2*0.034) < 10){Serial.print('0');}
  Serial.print(float(pingTime/2*0.034), 2);

  Serial.print("\033[A");                 // back up to line 1 for the next update
}


// ============================================================================
// PC CONTROL MODE (mode 'c')
//
// Talks to mcl.py and serial_tester.py. Everything here is non-blocking.
//
//   PC -> robot   C,<velocity in/s>,<omega deg/s, counter-clockwise +>
//                 G   grip          L   leave behind (release)
//                 x   back to the menu
//   robot -> PC   R,<odom_dist in>,<odom_heading deg>,<front>,<right>,<back>,
//                   <left>,<front_left>,<front_right>,<tof>      (all inches)
//
// A range of -1 means "no reading, ignore it".
// ============================================================================

// ---- ROBOT GEOMETRY AND ENCODERS (calibrated 2026-11-17) ------------------
// Measured with a ruler:
const float CTRL_WHEEL_DIAMETER_IN = 2.56;
const float CTRL_WHEEL_BASE_IN     = 5.5;   // between the tyre centres

// Encoder counts per WHEEL revolution, as the slave really counts them.
// Bench test: forward 12 in -> odometry said 11.9 in but the robot really
// moved 24 in, so the real count is 823.1 x 11.9 / 24 = 408.
// Check: wheels off the ground, turn both wheels exactly one revolution
// forward by hand; odometry should rise by pi x 2.56 = 8.04 in.
const float CTRL_COUNTS_PER_REV = 408.0;

// The slave's COUNTS_PER_REV (823.1). Its "RPM" (the PID target) is worked out
// with this number, so drive commands are converted into the slave's RPM
// units with it. The slave sketch and its PID tuning stay unchanged.
const float CTRL_SLAVE_COUNTS_PER_REV = 823.1;

// Turning scrub: the robot turns less than the wheel travel predicts (tyres
// slide sideways while turning). 90 deg turn tests: the wheels travelled
// enough for ~163 deg, the robot really turned ~132 deg -> 163 / 132 = 1.23.
// Used for the heading in R lines and for turn speeds.
// Fine-tune with test 3: odometry O deg, robot really turned R deg ->
// CTRL_TURN_SCRUB = CTRL_TURN_SCRUB x O / R.
const float CTRL_TURN_SCRUB = 1.23;
const float CTRL_TURN_BASE_IN = CTRL_WHEEL_BASE_IN * CTRL_TURN_SCRUB;  // effective

// Wheel speed limits sent to the slave's PID, in the slave's RPM units (half
// the real RPM). Below about 6 the encoders give too few pulses per PID
// sample to hold a steady speed.
const float CTRL_MIN_RPM = 6.0;
const float CTRL_MAX_RPM = 100.0;
const float CTRL_FRONT_STOP_IN = 2.0;       // no forward driving closer than this
const float CTRL_BACK_STOP_IN = 1.0;        // no reversing closer than this (back sonar;
                                            // it sits above the block, so it only sees walls)

// 1 = print a "D,..." line with the time whenever the drive command changes or
// the watchdog fires. The Python programs show these lines as robot log
// messages (serial_tester.py --log saves them with PC timestamps). Set to 0
// once driving works.
#define CTRL_DEBUG 0

// Which Sharp IR sensor is fitted (read the label on the sensor):
//   41 = GP2Y0A41SK0F, 4-30 cm (short range)
//   21 = GP2Y0A21YK0F, 10-80 cm
// Your bench readings (about 2x the real distance with the 21 formula) match
// the 41 model. Must agree with SENSOR_TYPES["ir"] in config.py.
#define CTRL_IR_MODEL 41

// Rear VL53L0X time-of-flight sensor: 5V, GND, SDA on A4, SCL on A5 (the same
// I2C bus as the MPU6050 at 0x68 and the slave at 0x08; the ToF is at 0x29).
// Needs the Pololu "VL53L0X" library (Library Manager: "VL53L0X" by Pololu).
// Set to 0 to send the tof field as -1 (ignored by the PC).
#define CTRL_USE_TOF 1
#if CTRL_USE_TOF
#include <VL53L0X.h>
VL53L0X ctrlTof;
bool ctrlTofOk = false;                      // init() succeeded
unsigned long ctrlTofRetryMs = 0;            // last init attempt
const unsigned long CTRL_TOF_RETRY_MS = 2000;  // retry a failed init this often
const float CTRL_TOF_MAX_IN = 100.0 / 2.54;    // must match SENSOR_TYPES["tof"] max_in
const unsigned long CTRL_TOF_STALE_MS = 500;   // no new reading this long -> re-init
unsigned long ctrlTofNewMs = 0;                // time of the last new reading
// Filtering. The VL53L0X drops single readings ("8190 mm" = no valid return)
// even with a target in front of it, so one dropout must not become "nothing
// in range". Good readings go through a median of the last 5; only this many
// dropouts IN A ROW count as "nothing in range" (~0.35 s at 50 ms per reading).
const uint8_t CTRL_TOF_MISSES_FOR_EMPTY = 7;
const uint8_t CTRL_TOF_MEDIAN_N = 5;
uint16_t ctrlTofGood[CTRL_TOF_MEDIAN_N];       // last good readings, mm (newest first)
uint8_t ctrlTofGoodCount = 0;                  // how many of them are filled
// Calibration: reported mm = raw mm * SCALE + OFFSET. Fill in from the
// flat-wall test (leave 1.0 / 0 until then).
const float CTRL_TOF_SCALE = 1.0;
const float CTRL_TOF_OFFSET_MM = 0.0;
uint8_t ctrlTofMisses = 0;                     // dropouts in a row
// Set to 1 to print "D,tof raw <mm> status <s> -> <inches>" for every reading
// (about 20 lines per second: use it with the serial monitor, not with mcl.py).
#define CTRL_TOF_DEBUG 1
void ctrlTofStart();
void ctrlTofSample(uint16_t mm, uint8_t status);
#endif

// Which sensor is which (same as printSonarLogic)
const uint8_t CTRL_SONAR_FRONT = 0, CTRL_SONAR_LEFT = 1, CTRL_SONAR_RIGHT = 2, CTRL_SONAR_BACK = 3;
const uint8_t CTRL_IR_FRONT_LEFT = IR2, CTRL_IR_FRONT_RIGHT = IR1;

// Timing. At 9600 baud one R line takes about 55 ms to send, so 10 lines per
// second is the most the link can carry.
const unsigned long CTRL_R_PERIOD_MS    = 100;  // send R this often
const unsigned long CTRL_PING_PERIOD_MS = 30;   // one sonar per slot (avoids cross-talk)
const unsigned long CTRL_TOF_PERIOD_MS  = 20;
const unsigned long CTRL_WATCHDOG_MS    = 500;  // no C line for this long -> stop

float velocity = 0;   // latest command: forward speed, inches per second
float omega = 0;      // latest command: turn rate, degrees per second, CCW positive

char ctrlLine[24];            // incoming line, without the newline
uint8_t ctrlLineLen = 0;
bool ctrlLineTooLong = false;
unsigned long ctrlLastCmdMs = 0, ctrlLastRMs = 0, ctrlLastPingMs = 0, ctrlLastTofMs = 0;
bool ctrlStopped = true;      // the slave has been told to stop
float ctrlSonarIn[4] = { -1, -1, -1, -1 };
uint8_t ctrlNextSonar = 0;
float ctrlTofIn = -1;
long ctrlLeftCount = 0, ctrlRightCount = 0;

// Called once on entering mode 'c'.
void controlModeBegin(){

  velocity = 0;
  omega = 0;
  ctrlLineLen = 0;
  ctrlLineTooLong = false;
  ctrlStopped = true;
  ctrlNextSonar = 0;
  ctrlTofIn = -1;
  for (uint8_t i = 0; i < 4; i++){ ctrlSonarIn[i] = -1; }

  ctrlSendDrive();    // make sure the wheels are stopped

  #if defined(WIRE_HAS_TIMEOUT)
  // a loose I2C wire must not freeze the loop (the motors would keep their
  // last command): give up on a stuck transfer after 3 ms and reset the bus
  Wire.setWireTimeout(3000, true);
  #endif

  #if CTRL_USE_TOF
  ctrlTofStart();
  #endif

  unsigned long now = millis();
  ctrlLastCmdMs = now;
  ctrlLastRMs = now;
  ctrlLastPingMs = now;
  ctrlLastTofMs = now;
}

// One pass of the control loop. Returns false when the PC sent x (leave the mode).
bool controlModeStep(){

  // 1. commands first, so the motors always get the newest one
  if (!ctrlReadSerial()){ return ctrlLeave(); }

  unsigned long now = millis();

  // 2. watchdog: the PC stopped sending C lines -> stop the wheels
  if (!ctrlStopped && now - ctrlLastCmdMs > CTRL_WATCHDOG_MS){
    velocity = 0;
    omega = 0;
    ctrlSendDrive();
    #if CTRL_DEBUG
    Serial.print("D,master watchdog stop at ms ");
    Serial.println(now);
    #endif
  }

  // 3. one ultrasonic sensor per time slot
  if (now - ctrlLastPingMs >= CTRL_PING_PERIOD_MS){
    ctrlLastPingMs = now;
    unsigned long us = ctrlPing(ctrlNextSonar); // microseconds, 0 = no echo
    // no echo within 150 cm = "nothing in range": report the 150 cm limit
    // (59 in) so the filter learns the way is open, instead of -1 (ignore)
    if (us == 0){ ctrlSonarIn[ctrlNextSonar] = 150.0 / 2.54; }
    else { ctrlSonarIn[ctrlNextSonar] = us * 0.0343 / 2.0 / 2.54; }
    ctrlNextSonar = (ctrlNextSonar + 1) % 4;
  }

  if (!ctrlReadSerial()){ return ctrlLeave(); }  // check again: keep up with the PC

  // 4. rear time-of-flight sensor: read only when a new measurement is ready
  #if CTRL_USE_TOF
  if (!ctrlTofOk){
    // not found at start (or lost): try again now and then
    if (now - ctrlTofRetryMs >= CTRL_TOF_RETRY_MS){ ctrlTofStart(); }
  }
  else if (now - ctrlLastTofMs >= CTRL_TOF_PERIOD_MS){
    ctrlLastTofMs = now;
    // only read when a new measurement is ready, so this never waits
    if ((ctrlTof.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) != 0){
      // device range status, bits 6..3: 11 = measurement completed normally
      uint8_t status = (ctrlTof.readReg(VL53L0X::RESULT_RANGE_STATUS) >> 3) & 0x0F;
      uint16_t mm = ctrlTof.readRangeContinuousMillimeters();  // also clears "ready"
      if (ctrlTof.timeoutOccurred()){
        ctrlTofOk = false;             // stopped answering: re-init later
        ctrlTofIn = -1;
      }
      else { ctrlTofSample(mm, status); }
      ctrlTofNewMs = now;
    }
    else if (now - ctrlTofNewMs > CTRL_TOF_STALE_MS){
      ctrlTofOk = false;               // unplugged or stuck: stop reporting an old value
      ctrlTofIn = -1;
    }
  }
  #endif

  // 5. R line on a timer
  if (now - ctrlLastRMs >= CTRL_R_PERIOD_MS){
    ctrlLastRMs = now;
    ctrlReadEncoders();
    ctrlSendR();
  }

  return true;
}

#if CTRL_USE_TOF
// Find the ToF and start continuous ranging. Prints a D line saying whether it worked.
void ctrlTofStart(){
  ctrlTofRetryMs = millis();
  ctrlTofIn = -1;
  ctrlTof.setTimeout(50);
  ctrlTofGoodCount = 0;
  ctrlTofMisses = 0;
  ctrlTofOk = ctrlTof.init();          // false if nothing answers at 0x29
  if (ctrlTofOk){
    // a longer measurement is less noisy and drops out less often; 50 ms
    // (20 per second) is still twice the R line rate
    ctrlTof.setMeasurementTimingBudget(50000);
    // keep the default signal limit (0.25 MCPS): lowering it lets weak,
    // mixed returns through, which read short and noisy
    ctrlTof.startContinuous();
    ctrlTofNewMs = millis();
    Serial.println("D,tof ok");
  }
  else {
    Serial.println("D,tof not found - check 5V, GND, SDA=A4, SCL=A5");
  }
}

// One new reading -> ctrlTofIn (inches) for the R line:
//   median of the last 3 good readings   normally
//   (unchanged)                          a few dropouts: keep the last value
//   max range (39.4 in)                  CTRL_TOF_MISSES_FOR_EMPTY dropouts in a row
//   -1 (PC ignores it)                   too close to measure (< 3 cm)
void ctrlTofSample(uint16_t mm, uint8_t status){

  bool good = (status == 11) && mm >= 30 && mm < 2000;  // 2 m: beyond the sensor's real reach

  if (good){
    ctrlTofMisses = 0;
    for (uint8_t i = CTRL_TOF_MEDIAN_N - 1; i > 0; i--){ ctrlTofGood[i] = ctrlTofGood[i - 1]; }
    ctrlTofGood[0] = mm;
    if (ctrlTofGoodCount < CTRL_TOF_MEDIAN_N){ ctrlTofGoodCount++; }

    // median of the readings we have (sort a small copy)
    uint16_t s[CTRL_TOF_MEDIAN_N];
    uint8_t n = ctrlTofGoodCount;
    for (uint8_t i = 0; i < n; i++){
      uint16_t v = ctrlTofGood[i];
      uint8_t j = i;
      while (j > 0 && s[j - 1] > v){ s[j] = s[j - 1]; j--; }
      s[j] = v;
    }
    float calMm = s[n / 2] * CTRL_TOF_SCALE + CTRL_TOF_OFFSET_MM;
    float in = calMm / 25.4;
    if (in < 30 / 25.4){ in = -1; }                            // calibrated below 3 cm
    ctrlTofIn = in > CTRL_TOF_MAX_IN ? CTRL_TOF_MAX_IN : in;
  }
  else if (status == 11 && mm < 30){
    ctrlTofIn = -1;                    // a real return, but too close to trust
    ctrlTofGoodCount = 0;
  }
  else {
    // dropout (8190/8191 mm or a failed range status)
    if (ctrlTofMisses < 255){ ctrlTofMisses++; }
    if (ctrlTofMisses >= CTRL_TOF_MISSES_FOR_EMPTY){
      ctrlTofIn = CTRL_TOF_MAX_IN;     // really nothing there
      ctrlTofGoodCount = 0;
    }
  }

  #if CTRL_TOF_DEBUG
  Serial.print("D,tof raw ");
  Serial.print(mm);
  Serial.print(" status ");
  Serial.print(status);
  Serial.print(" -> ");
  Serial.println(ctrlTofIn, 1);
  #endif
}
#endif

// Stop everything and leave control mode (the PC sent x).
bool ctrlLeave(){
  velocity = 0;
  omega = 0;
  sendByte(ArduinoSlave, 'x');
  return false;
}

// One ultrasonic ping, waiting for echoes up to 150 cm only (the maze needs
// less), so a sensor that hears nothing holds up the loop for ~9 ms, not ~23 ms.
unsigned long ctrlPing(uint8_t sensorNum){
  if (sensorNum == 0){ return sonar0.ping(150); }
  if (sensorNum == 1){ return sonar1.ping(150); }
  if (sensorNum == 2){ return sonar2.ping(150); }
  return sonar3.ping(150);
}

bool ctrlNewCommand = false;   // a C line arrived during this ctrlReadSerial() call

// Read everything that has arrived on Serial. If several C lines are waiting,
// only the newest one is passed on to the slave. Returns false if an x line arrived.
bool ctrlReadSerial(){

  ctrlNewCommand = false;
  bool keepGoing = ctrlDrainSerial();
  if (ctrlNewCommand){ ctrlSendDrive(); }       // one I2C message, with the newest command
  return keepGoing;
}

bool ctrlDrainSerial(){

  while (Serial.available() > 0){
    char ch = Serial.read();

    if (ch == '\r'){ continue; }

    if (ch != '\n'){
      if (ctrlLineLen < sizeof(ctrlLine) - 1){ ctrlLine[ctrlLineLen++] = ch; }
      else { ctrlLineTooLong = true; }          // too long: drop the whole line
      continue;
    }

    ctrlLine[ctrlLineLen] = '\0';
    bool keepGoing = ctrlLineTooLong || ctrlHandleLine();
    ctrlLineLen = 0;
    ctrlLineTooLong = false;
    if (!keepGoing){ return false; }
  }

  return true;
}

// Act on one complete line in ctrlLine. Returns false for x (leave the mode).
bool ctrlHandleLine(){

  if (ctrlLine[0] == 'C' && ctrlLine[1] == ','){
    char *end;
    char *p = ctrlLine + 2;
    float v = strtod(p, &end);
    if (end == p || *end != ','){ return true; }      // damaged line: ignore it
    p = end + 1;
    float w = strtod(p, &end);
    if (end == p){ return true; }
    #if CTRL_DEBUG
    if (v != velocity || w != omega){         // print only when the command changes
      Serial.print("D,master got C at ms ");
      Serial.print(millis());
      Serial.print(": v=");
      Serial.print(v, 2);
      Serial.print(" w=");
      Serial.println(w, 1);
    }
    #endif
    velocity = v;
    omega = w;
    ctrlLastCmdMs = millis();
    ctrlNewCommand = true;                     // sent to the slave by ctrlReadSerial()
  }
  else if (ctrlLine[1] == '\0'){
    if (ctrlLine[0] == 'G'){ sendByte(ArduinoSlave, 'g'); }
    else if (ctrlLine[0] == 'L'){ sendByte(ArduinoSlave, 'l'); }
    else if (ctrlLine[0] == 'x' || ctrlLine[0] == 'X'){ return false; }
  }
  // anything else (for example P lines) is ignored

  return true;
}

// Turn velocity / omega into a wheel direction pair and one wheel RPM, and
// send them to the slave: 'v', left dir, right dir, RPM x10 (low byte, high byte).
// The robot cannot drive and turn at once: if both are set, it turns.
void ctrlSendDrive(){

  int8_t left = 0, right = 0;
  float wheelInPerS = 0;

  if (fabs(omega) > 0.5){
    right = (omega > 0) ? 1 : -1;               // counter-clockwise = right wheel forward
    left = -right;
    wheelInPerS = fabs(omega) * PI / 180.0 * CTRL_TURN_BASE_IN / 2.0;
  }
  else if (fabs(velocity) > 0.05){
    left = right = (velocity > 0) ? 1 : -1;
    wheelInPerS = fabs(velocity);
    // safety: never drive into a wall the front / back sonar sees. The back
    // sonar is mounted above the block, so backing onto the block still works.
    float front = ctrlSonarIn[CTRL_SONAR_FRONT];
    float back = ctrlSonarIn[CTRL_SONAR_BACK];
    if (velocity > 0 && front > 0 && front < CTRL_FRONT_STOP_IN){ left = right = 0; }
    if (velocity < 0 && back > 0 && back < CTRL_BACK_STOP_IN){ left = right = 0; }
  }

  float rpm = 0;
  if (left != 0){
    // real wheel RPM, then into the slave's RPM units (it uses 823.1 counts/rev)
    rpm = wheelInPerS * 60.0 / (PI * CTRL_WHEEL_DIAMETER_IN)
          * CTRL_COUNTS_PER_REV / CTRL_SLAVE_COUNTS_PER_REV;
    rpm = constrain(rpm, CTRL_MIN_RPM, CTRL_MAX_RPM);
  }
  unsigned int rpmX10 = (unsigned int)(rpm * 10.0 + 0.5);

  Wire.beginTransmission(ArduinoSlave);
  Wire.write((uint8_t)'v');
  Wire.write((uint8_t)left);
  Wire.write((uint8_t)right);
  Wire.write(lowByte(rpmX10));
  Wire.write(highByte(rpmX10));
  Wire.endTransmission(true);

  ctrlStopped = (left == 0);
}

// Ask the slave for its signed encoder totals (left, right). Keeps the old
// values if the slave does not answer.
void ctrlReadEncoders(){

  long counts[2];

  if (Wire.requestFrom(ArduinoSlave, (int)sizeof(counts)) != sizeof(counts)){
    while (Wire.available() > 0){ Wire.read(); }
    return;
  }

  Wire.readBytes((uint8_t *)counts, sizeof(counts));
  ctrlLeftCount = counts[0];
  ctrlRightCount = counts[1];
}

// Sharp IR: analog voltage -> inches, for the model set by CTRL_IR_MODEL.
// Closer than its minimum the sensor reads too far; the Python side knows.
float ctrlReadIR(uint8_t pin){

  long sum = 0;
  for (uint8_t i = 0; i < 8; i++){ sum += analogRead(pin); }

  float volts = sum / 8.0 * 5.0 / 1023.0;

  #if CTRL_IR_MODEL == 41
  // GP2Y0A41SK0F, 4-30 cm: distance ~ 13 / volts (cm)
  if (volts < 0.3){ volts = 0.3; }              // nothing in range: reports about 43 cm
  return 13.0 / volts / 2.54;
  #else
  // GP2Y0A21YK0F, 10-80 cm
  if (volts < 0.35){ volts = 0.35; }            // nothing in range: reports about 98 cm
  return 27.728 * pow(volts, -1.2045) / 2.54;
  #endif
}

void ctrlSendR(){

  float inPerCount = PI * CTRL_WHEEL_DIAMETER_IN / CTRL_COUNTS_PER_REV;
  float odomDist = (ctrlLeftCount + ctrlRightCount) * 0.5 * inPerCount;
  float odomHeading = (ctrlRightCount - ctrlLeftCount) * inPerCount / CTRL_TURN_BASE_IN * 180.0 / PI;

  Serial.print("R,");
  Serial.print(odomDist, 2);                            Serial.print(',');
  Serial.print(odomHeading, 1);                         Serial.print(',');
  Serial.print(ctrlSonarIn[CTRL_SONAR_FRONT], 1);       Serial.print(',');
  Serial.print(ctrlSonarIn[CTRL_SONAR_RIGHT], 1);       Serial.print(',');
  Serial.print(ctrlSonarIn[CTRL_SONAR_BACK], 1);        Serial.print(',');
  Serial.print(ctrlSonarIn[CTRL_SONAR_LEFT], 1);        Serial.print(',');
  Serial.print(ctrlReadIR(CTRL_IR_FRONT_LEFT), 1);      Serial.print(',');
  Serial.print(ctrlReadIR(CTRL_IR_FRONT_RIGHT), 1);     Serial.print(',');
  Serial.println(ctrlTofIn, 1);
}

